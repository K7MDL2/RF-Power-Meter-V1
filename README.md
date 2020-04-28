# RF-Power-Meter-V1
Arduino based RF wattmeter with optional Python based remote monitoring and control desktop app

Version 1.0 RF Power Meter code running on a M5Stack (http://M5Stack.com) Arduino CPU module. Build yuour own RF milli-watmeter for HF through microwave frequencies with low cost off-the-shelf modules and some glue wiring and a box. 

Optionally leverages WSJT-X status message UDP broadcasts to read the radio dial frequency and automatically command the meter to load the appropriate calibration values (10 bands supported). Using the awesome Python library to decode WSJT-X packets and is found at  https://github.com/bmo/py-wsjtx.


I plan to enable wireless data connection in a future version.  With the recent changes to support remote commands the requirement for a graphics screen is mostly removed so I am looking at enabling a headless mode to enable running this on simple inexpensive Arduino boards to enable lower cost, simpler packaging and remote placement of the detector and CPU.  Enabling wireless will be that more compelling for remote placements.  

I think it would be interesting to place 2 identically calibrated units at each end of your coax and watch for the changes over time and measure the actual cable loss.  The meter features 10 “Bands” or sets of calibration values.  Each set contains Band Name, Forward and Reflected Port Coupling Factor.  The coupling factor is a value in dB representing the coupler port’s coupling factor at a given frequency plus any added attenuators and finally accounts for minor cabling and detector related variances (fudge factor).
Direct Band changes (10 bands) are possible using serial port 1 byte commands in addition to emulating the hardware buttons to change Band, Scale, SWR. Saves a lot of time cycling through 10 bands.
 
RF power detectors:
2 Log Power Detectors are used with their outputs fed to the AD input of the Arduino. They connect to a coupler of your choice. You could likely also use RF detector outputs built into some transverters and amps, or diode based detector designs (such as from W6PQL or W1GHZ).

I am using AD8318 based imported modules commonly found on Amazon.  They are under $10 each these days. It covers up to 8Gz, might work well enough to 10GHz, I have not checked yet.  You could use any RF detector with a possible change to limit the voltage to your Arduino’s AD input voltage spec and likely modify some of the Arduino side calibration related code and constants for slope, offset and inversion.  The AD8318 outputs an inverted voltage curve bewtween 0.5 adn 2.3VDC compared to the AD8307 600MHz detector which outputs a normal rising voltage 0 to 2.XVDC with rising power input. 

I built a small metal box to house the the 2 detector modules, included 9V and 5V regulators, powered it from 12V.  The 5V is powered frm teh 9V to resduce heating and is used to supply the power to the Arduino via the USB port.  You could put the Arduino in the same box as the detector to save cabling and cost.  That will be the case for the headless version planned (future).
 
Hardware pictures can be seen on teh project Wiki pages
 
RF Remote Power Meter application:
The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing the Meter ID, a sequence number, Band Name, Forward Power, Reflected Power, an SWR value.  The data values for both forward and reflected power are sent out in dBm and Watts for convenience. One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script).  If using WSJT-X the app will automatically change the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.
 
Snapshot on 432Mhz with a well matched antenna

 <insert pictures here>
 
Sample on 1296 with a not so good antenna match.  Over 3.0 and SWR will turn to RED.

 <insert pictures here> 
 

Button Operating Procedures are on the Project Wiki page
