/*
    kavr
 
    AVR Hexfile Boot Loader (pronounced "caviar")
 
    Copyright (C) 2009-2011 Frank Edelhaeuser
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
 
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <avr/boot.h>
#include <avr/common.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/setbaud.h>


// macros
#define PACKED          __attribute__((__packed__))
#define NOINLINE        __attribute__((noinline))
#define NAKED           __attribute__((naked))
#define SECTION(a)      __attribute__((section(a)))


// definitions
#define XON             0x11
#define XOFF            0x13
#define TIMEOUT_LOOPS   (((F_CPU/1000)*TIMEOUT)/33)

#define FIFOSIZE        64
#define XOFFLIMIT       32


// Use UART0 on architectures having two UARTs
#ifndef UDR
    #define UDR UDR0
    #define UCSRA UCSR0A
    #define UCSRB UCSR0B
    #define UCSRC UCSR0C
    #define URSEL 0
#ifndef RXC
    #define UCSZ0 UCSZ00
    #define UCSZ1 UCSZ01
    #define RXEN RXEN0
    #define TXEN TXEN0
    #define RXC RXC0
    #define UDRE UDRE0
#endif
    #define UBRRH UBRR0H
    #define UBRRL UBRR0L
#ifndef U2X
    #define U2X U2X0
#endif
#endif


// data
struct HEX_RECORD
{
    uint8_t             length;
    uint16_t            address;
    uint8_t             type;
    uint8_t             data[255 + 1];
} PACKED record;

uint8_t                 page[SPM_PAGESIZE];
uint8_t                 fifo[FIFOSIZE];
uint8_t                 fifo_fill;


// defined in linker script
extern void* __bss_end;


// prototypes
static void flash_wait_poll_uart(void);
static void flash_write(uint16_t page);
static void out_char(uint8_t c);


// implementation
void kavr(void) NAKED SECTION(".vectors");
void kavr(void)
{
    uint32_t            timeout;
    uint16_t            new_page;
    uint16_t            cur_page;
    uint16_t            addr;
    uint8_t*            rx_ptr;
    uint8_t             rx_dat;
    uint8_t             rx_crc;
    uint8_t             rx_len;
    uint8_t             rx_val;
    uint8_t             i;
    uint8_t*            dst;


    // Instead of linking __init:
    // (how to avoid this assembly code? Anyone?)
    asm volatile ("clr r1");
    asm volatile ("ldi r28, lo8(%0)" :: "i" (&__bss_end));
    asm volatile ("ldi r29, hi8(%0)" :: "i" (&__bss_end));

    SREG = 0;
    SP   = RAMEND;
#if defined(RAMPD)
    RAMPD = 0;
#endif
#if defined(RAMPX)
    RAMPX = 0;
#endif
#if defined(RAMPY)
    RAMPY = 0;
#endif
#if defined(RAMPZ)
    RAMPZ = 0;
#endif

    // Configure UART for $(BAUD),8N1
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;

    #if USE_2X
    UCSRA = _BV(U2X);
    #else
    UCSRA = 0;
    #endif

    // Enable receiver and transmitter
    UCSRB = _BV(RXEN) | _BV(TXEN);

    // 8N1
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);

    // Hello world!
    out_char('K');
    out_char('A');
    out_char('V');
    out_char('R');

    // XON
    out_char(XON);

    // Initialize overall state
    cur_page  = BOOTADDR;
    fifo_fill = 0;
    timeout   = 0;

    // (Re-)Initialize line state
restart_line:
    rx_val = 0;
    rx_len = 0;
    rx_crc = 0;
    rx_ptr = (uint8_t*) &record;

    // Download data records until timeout
    while (timeout < TIMEOUT_LOOPS)
    {
        // Poll UART for receive characters
        flash_wait_poll_uart();
        if (fifo_fill)
        {
            // Extract oldest character from receive FIFO
            rx_dat = fifo[0];
            fifo_fill --;
            i = 0;
            do fifo[i] = fifo[i + 1]; while (++ i < fifo_fill);

            // Convert to binary nibble
            rx_dat -= '0';
            if (rx_dat > 9)
            {
                rx_dat -= 7;
                if ((rx_dat < 10) || (rx_dat > 15))
                    goto restart_line;
            }

            // Valid hex character. Restart timeout.
            timeout = 0;

            // Two nibbles = 1 byte
            rx_val = (rx_val << 4) + rx_dat;
            if ((++ rx_len & 1) == 0)
            {
                rx_crc += rx_val;
                *rx_ptr ++ = rx_val;

                if (rx_ptr == (uint8_t*) &record.data[record.length + 1])
                {
                    if (rx_crc)
                    {
                        // Checksum error. Abort
                        out_char('?');
                        break;
                    }
                    else if (record.type == 0)
                    {
                        // Data record
                        addr = (record.address << 8) | (record.address >> 8);
                        for (i = 0; i < record.length; i ++)
                        {
                            // Determine page base for current address
                            new_page = addr & ~(SPM_PAGESIZE - 1);
                            if (new_page != cur_page)
                            {
                                // Write updated RAM page buffer into flash
                                flash_write(cur_page);

                                // Load page at new address into RAM page buffer
                                cur_page = new_page;

                                // memcpy_P(page, (PGM_P) new_page, SPM_PAGESIZE);
                                dst = page;
                                while (dst < &page[SPM_PAGESIZE])
                                    *dst ++ = pgm_read_byte((PGM_P) new_page ++);
                            }

                            // Update RAM page buffer from data record
                            page[addr & (SPM_PAGESIZE - 1)] = record.data[i];
                            addr ++;
                        }

                        goto restart_line;
                    }
                    else if (record.type == 1)
                    {
                        // End of file record. Finish
                        flash_write(cur_page);
                        out_char('S');
                        break;
                    }
                }
            }
        }

        timeout ++;
    }

    out_char(XOFF);

    typedef void APP(void);
    ((APP*)0)();
}


static void out_char(uint8_t c)
{
    while (!(UCSRA & _BV(UDRE)));
    UDR = c;
}


static void flash_write(uint16_t addr)
{
    uint16_t  dst;
    uint16_t* src;
    uint8_t   i;

    if (addr < BOOTADDR)
    {
        out_char(XOFF);

        // Copy RAM addr buffer into SPM addr buffer
        dst = addr;
        src = (uint16_t*) page;
        i = 0;
        do
        {
            boot_page_fill(dst, *src ++);
            dst += 2;
        }
        while (++ i < SPM_PAGESIZE / 2);

        // Erase and program
        boot_page_erase(addr);
        flash_wait_poll_uart();

        boot_page_write(addr);
        flash_wait_poll_uart();

        boot_rww_enable();

        out_char(XON);
    }
}


static void flash_wait_poll_uart(void)
{
    do
    {
        if ((fifo_fill < FIFOSIZE) && (UCSRA & _BV(RXC)))
            fifo[fifo_fill ++] = UDR;
        wdt_reset();
    }
    while (boot_spm_busy());
}
