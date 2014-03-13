
Reflow Oven Controller
Based on  https://github.com/CvW/Reflow-Oven-Controller/blob/master/reflowOvenController.pde
Which is based on http://github.com/rocketscream/Reflow-Oven-Controller

BoM

1x Arduino Pro-Mini 3.3 volts https://www.sparkfun.com/products/11114
1x LCD Display monochrome 84x84 http://www.adafruit.com/products/338
1x Red LED
1x Green LED
1x Solid State Relay
1x Fuse Holder
1x Fuse
1x Thermocouple Breakout board http://www.adafruit.com/products/269
1x Thermocouple probe
1x Contactless IR sensor http://www.adafruit.com/products/1747
2x pushbuttons
1x Diode for motor noise reduction
1x Cap for motor noise reduction
1x MOSFET to run motor
2x Strip heter, McMaster 3619K831
2ft Oven Wire
Calcium Silicate Insulation McMaster 9353K32 9353K31
1x High temp gasketing adhesive McMaster 7573A63
1x DC Motor
1x RC boat prop, metal
1x shaft Coupling
1x Thermistor Digikey KC024N-ND
1x AC/DC Converter Digikey 102-2386-ND, data sheet http://www.cui.com/product/resource/vsk-s5.pdf
1x 5v to 3v voltage regulator - might be able to use regulator in Pro-Mini
1x 1uF cap for AC/DC supply
1x 47uF cap for AC/DC supply
1x MOV Varistor
1x Relay for heater Digikey CLA394-ND, 6 amp, Input: 1.2 volts 1.5mA  Datasheet http://www.clare.com/home/pdfs.nsf/www/CPC1907B.pdf/$file/CPC1907B.pdf
1x 1.5k Resistor for Relay
1x Buzzer
2x I2C pullup resistors


To Do: 
See if buzzer works on 3v


PCB Design
Have 4 port terminal block in case you want to use IR sesnsor, it's I2C
Power thermocouple breakout board on 3 volt Vo pin - verify this will work
