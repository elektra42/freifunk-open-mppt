/usr/bin/avrdude -C /etc/avrdude.conf -p m8 -P usb -c dragon_isp -B 100 -u -U hfuse:w:0xDA:m -U lfuse:w:0x9F:m 
