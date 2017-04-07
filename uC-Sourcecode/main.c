/* **********************************************************************
 * AVR-GCC source code for Freifunk-Open-MPP-Solar-Tracker
 * Copyright (C) 2017  by Corinna 'Elektra' Aichele 
 * 
 * This file is part of the Open-Hardware and Open-Software project 
 * Freifunk-Open-MPP-Solar-Tracker.
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This source code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this source file. If not, see http://www.gnu.org/licenses/. 
 *************************************************************************/


#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

/* UART init */

#ifndef F_CPU
#warning "F_CPU not defined in Makefile. Using 3.686400 MHz"
#define F_CPU 3686400UL 
#endif

/* Define baud rate for serial port */
#define BAUD 115200UL

/* Calculate UART BAUD register setting */
#define UBRR_SETTING ((F_CPU+BAUD*8)/(BAUD*16)-1)



    uint16_t v_out_adcval;
    uint16_t solar_in_adcval;
    uint16_t v_in_value;
    uint16_t current_out_adcval;
    uint16_t v_out_value;
    uint16_t u_zero_current;
    char vo[] = "";
    uint8_t duty;
    uint16_t upper_mpp_current_value;
    uint16_t lower_mpp_current_value;
    uint16_t medium_mpp_current_value;
    uint16_t v_out_max = 14900;
    uint16_t v_mpp_estimate = 0;
    uint8_t step = 0x2; // Three point measurement step size
    volatile uint16_t ticks = 0;
    

/* ADC init */
void ADC_Init(void)
{
  // Reference: Use Vcc as AVcc
  ADMUX = (1<<REFS0);    
  
  /* Bit ADFR ("free running") in ADCSRA is zero 
   * by default which means single conversion */
  
  // Enable frequency divider
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);
  // set ADC enable
  ADCSRA |= (1<<ADEN);                             

  /* After activating the ADC, a warm-up readout is 
   * recommended to increase accuracy */
  
  // run ADC readout
  ADCSRA |= (1<<ADSC);
  // wait until finished
  while (ADCSRA & (1<<ADSC) ) {               
  }
}



/* ADC read, single conversion */
uint16_t ADC_Read( uint8_t channel )
{
  /* Select ADC channel */
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  // single conversion
  ADCSRA |= (1<<ADSC);
  // wait for conversion until finished
  while (ADCSRA & (1<<ADSC) ) {             
  }
  return ADCW;
}



/* Multiple ADC readouts, calculate average  */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t nsamples )
    {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < nsamples; ++i ) {
    sum += ADC_Read( channel );
            }

    return (uint16_t)( sum / nsamples );
    }


/* UART init */
void uart_tx_enable(void)
    {
        UBRRH = UBRR_SETTING >> 8;
        UBRRL = UBRR_SETTING & 0xFF;
	
	// enable UART TX
        UCSRB |= (1<<TXEN);
	// select asynchronous mode 8N1 
        UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // asynchronous mode 8N1 
        }

        
/* UART disable */
void uart_tx_disable(void)
    {
        UCSRB |= (0<<TXEN);  // disable UART TX 
        }
  
  

/* UART send single character */
int uart_putc(unsigned char c)
    {
        // wait until transmit is possible 
        while (!(UCSRA & (1<<UDRE)))  
        {
        }                             
	// send character
        UDR = c;                      
        return 0;
        }

    
/* UART send string */
void uart_puts(char *s) 
    {
        while (*s)
        {   //transmit as long as  *s != '\0' 
            uart_putc(*s);
            s++;
        }
    }
    
/* Low voltage disconnect */                    
void low_voltage_disconnect (uint16_t voltage)
    {
        // Battery voltage level that enables load in mV
        if (voltage > 12300) 
        { 
        PORTD = (1<<PD7) ;
        }
         // Battery voltage level that enables load in mV     
        if (voltage < 11800) 
        {
        PORTD = (0<<PD7);
        }
                    
        if (PORTD == (1<<PD7))
        {
        uart_puts("Load enabled\r\n\n");
        }
        else 
        {
        uart_puts("Load disabled\r\n\n");
        }
                    
    }

/* Power saving sleep routine based on counter/timer2 
 * Sleep (idle) time with 3.684 MHz clock is 70.6ms */
void interrupt_based_sleep (void)
	{
  
	TIFR |= (1<<TOV2);
	// enable counter2 overflow interrupt
	TIMSK |= (1<<TOIE2);
	// Set counter2 to zero
	TCNT2 = 0x00;
	sei();  
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_mode();
	cli();
	}

/* ISR TIMER2 overflow routine */
ISR(TIMER2_OVF_vect)
      {
      return;
      }
    
    
/* Measure output current */
uint16_t current_out(void)

    {   
	// wait until the current tracking point setting has settled before measurement
	_delay_ms(12);
	/* Measure output current 
	 * In order to measure current with the OP-Amp differential amplifier
	 * we need to know the OP-Amp output voltage at zero current 
	 * We calculate that from output voltage */
        
	
	/* Measure output voltage */
	
	// Use ADC channel 0 (V_out) and average for N (10) measurements
        v_out_adcval = ADC_Read_Avg(0, 10);
	// Take the voltage divider ratio into account and calculate actual output voltage in mV
        v_out_value = (17.57 * v_out_adcval); 
                
	// Take the voltage divider ratio into account and calculate voltage at zero current
	// The values depend on the voltage divider ratio (1.5) and Zener diode voltage drop
	
	// Old board:
        //u_zero_current = ((value / 1.5) - 6200) / 2;
	
	// 1.455 divider ratio OP-AMP input voltage divider
	// Divider ratio at Atmega8 ADC input for 15k/ 69k voltage divider
	
	u_zero_current = (v_out_value / 1.455) / 4.77 ;
	
	// Perform measurement. Use ADC channel 1 (V_out) and average for N measurements
        current_out_adcval = ADC_Read_Avg(1,32);
	
	// Calculate actual output current in mA
	uint16_t current_value = ((current_out_adcval * 3.1738) - u_zero_current) * 1.342 ;
	return current_value;
    }            


                
                
    void serialdatareport (void)
    {
		    // Experiment: Prevent serial data from getting garbled.
		    _delay_ms(20);
		    // Prepare UART for sending data 
		    uart_tx_enable();
		    // Read ADC channel 2, calculate average from 10 readings
                    solar_in_adcval = ADC_Read_Avg(2, 32);  
                    v_in_value = (29.13 * solar_in_adcval);
                    uart_puts("V_in ");
		    // convert to ascii using 10 for radix 10 -> decimalsystem
                    itoa( v_in_value, vo, 10 ); 
                    uart_puts( vo );
                    uart_puts(" mV""\r\n");
                    
                    v_out_adcval = ADC_Read_Avg(0, 32); 
                    v_out_value = (17.57 * v_out_adcval);
                    uart_puts("V_out "); 
                    itoa( v_out_value, vo, 10 ); 
                    uart_puts( vo );
                    uart_puts(" mV""\r\n");
                    
		    u_zero_current = (v_out_value / 1.455) / 5.6 ;
		    
                    uart_puts( "V_zero_current ");
                    itoa( u_zero_current, vo, 10 );
                    uart_puts( vo );
                    uart_puts(" mV""\r\n"); 
                    
                    
                    medium_mpp_current_value = current_out();
                    uart_puts("I_out "); 
                    itoa(medium_mpp_current_value, vo, 10 );
                    uart_puts( vo );
                    uart_puts(" mA""\r\n");
                
                    uart_puts ("New PWM value: 0x");
                    itoa(OCR1A, vo, 16 );
                    uart_puts (vo);
                    uart_puts ("\r\n");      
                    
		    uart_tx_disable();
		    
                }
                


int main(void)
{
    
    // Set up GPIOs PD7 and PB1 as output ports
    DDRD  = (1<<PD7);
    DDRB = (1<<PB1);
    
    // Enable ADC
    ADC_Init();
    
    // Set up PWM1 to control refence voltage of OP-AMP
    TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(0<<WGM13)|(1<<WGM12)|(1<<WGM11)|(1<<WGM10);
    TCCR1B|=(1<<CS10);
    ICR1=0x3ff;
    OCR1A = 0xf0;
    
    // Enable Timer/Counter2 to generate interrups for timer based sleep
    TCCR2 |= ( 1<<CS02 )| ( 1<<CS01)| ( 1<<CS00 );  // Use counter2, set prescaler to 1024
    
    
    


        while (1) {
            
		/* Measure battery voltage and call low voltage disconnect check */
                 v_out_adcval = ADC_Read_Avg(0, 32); 
                 v_out_value = (17.57 * v_out_adcval);
                 low_voltage_disconnect(v_out_value);
		 
		 uart_puts("Parsed low voltage disconnect"); 
		 uart_puts ("\r\n");
		 
		 
		 
		 /* Stop charging at 14.9 Volt */
		 
		 if (v_out_value > v_out_max) {
		   
		   uart_puts("If condition true, v_out >");
		   itoa(v_out_max, vo, 10 );
		   uart_puts(vo);
		    while (v_out_value > v_out_max) {
		      OCR1A += 1;
		      _delay_ms(10);
		      v_out_adcval = ADC_Read_Avg(0, 32);
		      v_out_value = (17.57 * v_out_adcval); 
		      uart_puts("While loop, v_out greater v_out_max");
		      uart_puts ("\r\n");
		      }	   
		  _delay_ms(1000);
		}
		 
		 else {
		 
		 /* Measure solar input voltage */ 
		 solar_in_adcval = ADC_Read_Avg(2, 32);  
                 v_in_value = (29.13 * solar_in_adcval);
		 
		/* Check if there is actually power from the solar panel.
		 * If not, sleep for a while */
		
		if (v_out_value < v_in_value) {

		  OCR1A = 0x355;
		  _delay_ms(2000);
		  solar_in_adcval = ADC_Read_Avg(2, 32);
		  v_in_value = (29.13 * solar_in_adcval);
		  uart_puts("V_in_idle "); 
		  itoa( v_in_value, vo, 10 ); 
		  uart_puts( vo );
		  uart_puts(" mV""\r\n");
		  v_mpp_estimate = v_in_value / 1.31;
		  
		  OCR1A = 0x0;
		  _delay_ms(1000);
		  solar_in_adcval = ADC_Read_Avg(2, 32);
		  v_in_value = (29.13 * solar_in_adcval);
		  
		  uart_puts("Calculated V_mpp "); 
		  itoa( v_mpp_estimate, vo, 10 ); 
		  uart_puts( vo );
		  uart_puts(" mV""\r\n");
		  
		  while (v_in_value < v_mpp_estimate) {
		    
		    _delay_ms(10);
		    solar_in_adcval = ADC_Read_Avg(2, 32);
		    v_in_value = (29.13 * solar_in_adcval);
		    OCR1A += step;
		  }
		  
		 serialdatareport();
		 
                
		ticks = 0;
		uart_puts("Going to sleep. MPP should be set");
		uart_puts ("\r\n");
		while (ticks != 6000) {
                      interrupt_based_sleep();
		      ticks ++;
                
		}
		  }
		
		else {
		serialdatareport();
		ticks = 0;
		uart_puts("Going to sleep, input voltage too low");
		uart_puts ("\r\n");
		while (ticks != 6000) {
                      interrupt_based_sleep();
		      ticks ++;
                  }
                
		}
		
		}
		
            }

   return 0; // never reached 
        }
