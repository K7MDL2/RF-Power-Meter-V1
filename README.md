New in:
1.00 - Original release
1.01 - Some version control housekeeping, typo and a few small bug fixes.  Fixed meter needle disappearing at 0 input.
1.02 - Under dev -
      1. Place the window in the upper right corner of screen on app open. Plan to create a config file to remember placement position later.
      2. Changed the name of pyRFPowerMeter.py removing the Version number in the name.
      3. Added more serial port error handling.  For example, if usb is unplugged, the comms the ON/OFF button turns off and red.  Whe problem is fixed turn on aagain and it will gracefully resume.  Also atempting to handle startup error in case specified port not there. Plan to offer list of ports and hide the raw exception text.


# RF-Power-Meter
Arduino based RF wattmeter with optional Python based remote monitoring and control desktop app.

Version 1.0 RF Power Meter code running on a M5Stack (http://M5Stack.com) Arduino CPU module. Build your own RF wattmeter for HF through microwave frequencies with low cost off-the-shelf modules and some glue wiring and a box and a RF dual directional coupler. 

For remote monitoring a Python based application runs on yor PC and gets data cvia the USB serial port. It can optionally leverage WSJT-X status message UDP broadcasts to read your radio dial frequency and automatically command the meter to load the appropriate calibration values (10 bands supported). An awesome Python library is used to decode WSJT-X packets and is found at  https://github.com/bmo/py-wsjtx.

Documentation and some pictures and a screen shot are on this project's wikipages. Look there for configuration instructions for Python setup and app confiuguration as wel las the hardware description. Button Operating Procedures are on the Project Wiki page also.

You can also find more information about this and my other projexcts such as the Multiband LO and Remote antenna switch at my website.  This project can be found there at https://k7mdl2.wixsite.com/k7mdl/arduino-rf-remote-wattmeter.

While V1 is now created, there are many features yet to add.

I plan to enable wireless data connection in a future version.  With the support for remote commands that now exists the requirement for a local graphics screen is mostly removed so I am looking at enabling a headless mode to enable running on simple inexpensive Arduino boards to enable lower cost, simpler packaging, and remote placement of the detector and CPU. Enabling wireless will be that more compelling for remote placements.  

I think it would be interesting to place 2 identically calibrated units at each end of your coax and watch for the changes over time and measure the actual cable loss. 

I have ordered an ADL5519 1MHz - 10GHz dual detector module from http://sv1afn.com. Will try that on the headless development.
 
RF power detectors used for the first version:
2 Log Power Detectors are used with their outputs fed to the AD input of the Arduino. They connect to a coupler of your choice. You could likely also use RF detector outputs built into some transverters and amps, or diode based detector designs (such as from W6PQL or W1GHZ).

I am using inexpensive 6GHz capable AD8318 based imported modules commonly found on Amazon. They are under $10 each these days. It covers up to 8GHz, might work well enough to 10GHz, I have not checked yet. You could use most any RF detector with a possible change to limit the voltage to your Arduino’s AD input voltage spec and likely modify some of the Arduino side calibration related code and constants for slope, offset and inversion. The AD8318 and similar modules just plug in electrically-wise, and the AD8318 in particular outputs an inverted voltage curve between 0.5VDC and 2.3VDC compared to the AD8307 600MHz detector which outputs a normal rising voltage 0 to 2.5VDC with corresponding to rising power input. 

I built a small metal box to house the the 2 detector modules, included 9V and 5V regulators, powered it from 12V.  The 5V is powered from the 9V to reduce heating and is used to supply the 5V power to the Arduino via the USB port. You could put the Arduino in the same box as the detector to save cabling and cost.  That will be the case for the headless version planned (future).
 
Hardware pictures can be seen on the project Wiki pages
 
RF Remote Power Meter application:
The meter features 10 “Bands” or sets of calibration values.  Each set contains Band Name, Forward and Reflected Port Coupling Factor. The coupling factor is a value in dB representing the coupler port’s coupling factor at a given frequency plus any added attenuators and finally accounts for minor cabling and detector related variances (fudge factor). Direct Band changes (10 bands) are possible using serial port 1 byte commands in addition to emulating the hardware buttons to change Band, Scale, SWR. Saves a lot of time cycling through 10 bands.

The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing 8 comma delimited character strings representing Meter ID, a sequence number, Band Name, Forward Power, Reflected Power, a SWR value. The data values for both forward and reflected power are sent out in dBm and Watts for convenience. 

One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script).  If using WSJT-X the app will automatically change the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.
