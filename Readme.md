Reflow Oven Controller <br>

I found a nice toaster oven when I when I was dropping some garbage off at our town's e-waste dumpster.  It was in excellent condition.  It has heating three heating elements on the top and three on the bottom . It had a microcontroller to control everything, so it was a bit more complex then a toaster with just a couple of knobs.  My plan was to remove the micro controller PCB and replace it with my own.  The heating elements are controlled by a couple of (triacs BTA26-600B).  I hadn't used triacs before and had to do a little research on them.  One unusual aspect of the toaster design is that the microcontroller board was floating on the 110 VAC line.  This means that the 5 VDC supply voltage was over 100 volts if measure from mains ground.  But the digital ground was only 5 volts less.  So to the micro controller board this was fine, but the whole PCB was essentially about 110 VAC.  I wasn't really comfortable with this, so I hacked the power supply board a bit so it output straight 5 volts.  it also output -9 volts. <br>

I wanted to continue to use the big start button from the front panel, so I took the original micro controller PCB and cut it in half to keep the start button and the LEDs that lit it up.  Then I soldered some wires in a header that connected to the button and LEDs.<br>

I designed a PCB with Cadsoft Eagle and milled it out on my [ShapeOko](http://www.shapeoko.com/) CNC router.  This was one of the first PCBs I did on the ShapeOko and it turned out pretty well (after a few revs). 
I had to figure out how to trigger the triacs properly.  At first I didn't realize I had to trigger when the AC voltage crossed zero volts, this cause some magic smoke to be released from one of the resistors.  This was easily fixed.  Then I got a couple of Fairchild [MOC3032](http://www.digikey.com/product-search/en?keywords=MOC3032M-ND) zero-crossover opto-couplers.  I had a Nokia 5110 display from Adafruit which was a great fit for the existing display window.  I used an Arduino [Pro-mini](http://www.sparkfun.com/products/11114) (3.3 v) from Sparkfun to control everything.  To measure the temperature I have a [thermocouple](http://www.adafruit.com/products/270) from Adafruit and Adafruit's [thermocouple amplifier](http://www.adafruit.com/products/269).  <br>

For the Arduino code I started with code from here: http://github.com/CvW/Reflow-Oven-Controller/, which is based on code from [Rocketscream](http://github.com/rocketscream/Reflow-Oven-Controller).  There was a bug in the debounce code I fixed and I replaced the MAX6675 with MAX31855 thermocouple amplifier.   There a bunch of other little changes, nothing major.  The code has two solder temperature profiles that you can choose between (by pressing the bake button) - one for leaded paste and one for a low-temp paste <br>

This toaster is not a convection toaster where it circulates the air around, but I thought this would be a nice feature. So I used a little motor that I salvaged from an inkjet printer to power a 3.5 inch aluminum fan blade I ordered from McMaster (17545K63).  I used a 3/16 bronze flange busing (McMaster 9440T63) to support the brass shaft through the sheet metal side. The fan is a bit noisy, but works pretty well.  Since the reflow oven isn't an that long, not a lot of heat is conducted to the motor from the shaft. <br>  

I would like to have an interior light to illuminate the PCB as it's being reflowed, but I haven't figured out how to do that yet. <br>

You can see some pictures on [flickr](https://www.flickr.com/photos/37101040@N00/sets/72157645550432296/) or in the \photos directory of the repo. <br>





