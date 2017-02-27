These are preliminary sources and CAD files from the
Freifunk-Open-MPP-Tracker project.


SOME LICENSE-FOO:
+++++++++++++++++ 

This is a Open-Source and Open-Hardware project under General Public License
v2 and Creative Commons - Attribution - ShareAlike 3.0 

For more information, check out the LICENSE file.


A quick overview of the subdirectories:
+++++++++++++++++++++++++++++++++++++++

AVR-FUSES
+++++++++

Contains a XML configuration file to program the fuses of the AVR. I am
using the AVR dragon board and AVR-Burn-O-Mat software to program the fuses.



SCHEMATIC AND BOARD
+++++++++++++++++++

ECAD files and component libraries, made with KiCAD. KiCAD is a free and
open ECAD tool.  Due to the intend to provide a open hardware and open
software project for DIY home-brewers, I have decided to use a free and open
source ECAD software tool, too. It is my first project with KiCAD, though.


uC-SOURCECODE
+++++++++++++

AVR-GCC source and Makefile. 


SOME NOTES ABOUT THIS DESIGN
++++++++++++++++++++++++++++

Basically this hardware has two building blocks: 

The MPP controller section and the buck converter.

At the moment, the buck converter section is based on the el cheapo and
low-fi MC33063 controller chip.  It can easily be replaced by a faster and
more fancy switch converter IC to construct a more powerful and slightly more 
efficient buck converter section.  However, with the external powerful P-MOS
switches, the efficiency was already around 94% with the first prototype 
(measured at 2 Ampere output current).

One reason to replace the MC33063 might be noise. Under low load, the
MC33063 will skip pulses of the PWM, so the switching frequency becomes
audible. 

