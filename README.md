## ehal_playground

A bare metal embedded rust application that 
allows us to play with various embedded-hal devices
such as an OLED display (the SSD1306).

It uses an i2c connection to the OLED display, 
with a common +3VDC power supply .  

For an initial pass I used the 
STM32F401CxBx dev board

Note that this board does not include pull-up resistors on the i2c SCL and SDA lines, 
however many breakout boards (such as the "GY-BMP280") do include these. 
If your hardware does not include pull-up resistors, you may need to enable these in software. 
