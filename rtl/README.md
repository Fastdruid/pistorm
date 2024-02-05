# HOW TO BUILD

1) Download Quartus Prime Lite from [intel](https://www.intel.com/content/www/us/en/software-kit/795188/intel-quartus-prime-lite-edition-design-software-version-23-1-for-windows.html)
The original make used 20.1 however I used 23.1 (latest). 

2) Install the free "lite" version 

3) Edit the "make.bat" in this folder changing the path for where you installed Quartus (mine installed to intelFPGA\lite\23.1std rather than the original make which installed to intelFPGA\lite\20.1

4) Change the IP and if required password for your Pi

5) Make your changes to pistorm.v (and pistorm.qsf) and then run make.bat it will compile, send to the pi and flash.


