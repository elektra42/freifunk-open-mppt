# Progam bootloader with dragon_isp

avrdude -p atmega8 -P usb -c dragon_isp   -U flash:w:kavr.hex
