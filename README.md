## RF-Power-Meter
DIY Arduino based RF SWR\Wattmeter reads output from a pair of power detector modules that you assemble. They are attached to a RF dual directional coupler to read forward and reflected power. Optional Python based remote monitoring and control desktop app monitors the USB serial port output from the meter and can change calibration sets for different frequency bands. If using WSJT-X the app will use the broadcasted dial frequency (over UDP) to automatically set the right calibration set.

### Revision History:

1.00 - Original release

1.01 - Current Release (see Release Page for the zip file)
   1. Some version control housekeeping, typo and a few small bug fixes.  
   2. Fixed meter needle disappearing at 0 input.

1.02 - Under dev in branch 1.02 as of May 5, 2020

** NOTE ** 
The remote data protocol has changed for RX and TX to both be string based with comma separated values and have similar structure.  This means you cannot mix previous versions of Arduino or Python code (1.000/1.01) with this or later code (1.02+).  Ths was done to support full headless operation with expanded command messages and to support multipe meter instances (each on their own serial port).

Key changes:
   1. Place the window in the upper right corner of screen on app open. Plan to create a config file to remember placement position later.
   2. Changed the name of pyRFPowerMeter.py removing the Version number in the name.
   3. Added more serial port error handling. For example, if usb is unplugged, the comms the ON/OFF button turns off and red and shuts down the serial thread.  When problem is fixed turn on again and it will gracefully resume. Also added startup error in case specified port is not there. Offers a list of USB ports.  Also you can now run without any serial port (has limited use but good for the curious without a meter. 
   4. The network thread is now decoupled from the On/Off button and is always running.  
   5. Using network data and heartbeat from WSJTX to implement a failover on data source form Band and Radio ID.  Color and ID name changes to UI fields show data source changed.  Using the WSJTX heartbeat message to timeout the netowrk sourced data.
   6. Applied factory A/D correction library functions or A/D non-linearity on the Arduino (ESP32 CPU).
   7. Sped up the sample rate in the Arduino and in the GUI update rate for faster meter response.  
   8. Bug fix - Scale on wattmeter should return to the last used scale however the scale is incremented after a reset and the needle does not know it so is pegged out until you hit the scale button (local or remote) which syncs things back up. Fix tries to make the scale stay put as it was last.  Awaits further testing - still an open issue as of 5/7
   9. Created RF Wattmeter.ino as a headless version of Arduino code. Most fo the screen draws are removed. As of 5/8/2020 it can take remote commands to edit any of the 20 calibration values. The Python app is updated to send some test comamnds using the previous scale, speed and SWR buttons. The previous scale button now dumps the meter's curent cal values (Cal table).  Changed to a multibyte command message formate similar to the meter data power level out messages.  Changed the sequence number (2nd field) tto now be a msg_type field to enable future expansion of cammon categoaries and for more protection against random data looking like a legit message.  Noticed just recently a new behavior in using the Arduino IDE serial monitor to manually send command bytes, it now causes the CPU to reboot. Commands sent via the Python app have no problem.  Possibly a difference in handling different terminating characters, TBI.
   

### Info:
Version 1.0 RF Power Meter code running on a M5Stack (http://M5Stack.com) Arduino Basic Core CPU module.  No extra core modules required.  You will need 2 AD8318 based RF log power detector modules, or suitable alternatives with some code minor adjustments to adapt the calculation for different output V slope and offset and if no slope inversion.

The meter features 10 “Bands” or sets of calibration values. Each set contains Band Name, Forward and Reflected Port Coupling Factor. The coupling factor is a value in dB representing the coupler port’s coupling factor at a given frequency plus any added attenuators and also accounts for minor cabling and detector related variances (a fudge factor). Direct Band changes (10 bands) are possible using serial port 1 byte commands in addition to emulating the hardware buttons to change Band, Scale, SWR. Saves a lot of time cycling through 10 bands.

For remote monitoring a Python based application runs on your PC and gets data via the USB serial port. It can optionally leverage WSJT-X status message UDP broadcasts to read your radio dial frequency and automatically command the meter to load the appropriate calibration values (10 bands supported). An awesome Python library is used to decode WSJT-X packets and is found at  https://github.com/bmo/py-wsjtx.

Documentation and some pictures and a screen shot are on this project's Wiki Pages. Look there for configuration instructions for Python setup and app configuration as well as the hardware description. Button Operating Procedures are on the Project Wiki page also.

You can also find more information about this and my other projects such as the Multiband LO and Remote Antenna Switch at my website.  This project can be found there at https://k7mdl2.wixsite.com/k7mdl/arduino-rf-remote-wattmeter.

While V1 is now created, there are many features yet to add. I plan to enable wireless data connection in a future version.  With the support for remote commands that now exists the requirement for a local graphics screen is mostly removed so I am looking at enabling a headless mode to enable running on simpler inexpensive Arduino boards to enable lower cost, simplify packaging, and make for easier remote placement of the detector and CPU by enabling a wireless data connection. I think it would be interesting to place 2 identically calibrated units at each end of your coax and watch for the changes over time and measure the actual cable loss. 
 
### RF power detectors used for the first version:
2 Log Power Detectors are used with their outputs fed to the A/D input of the Arduino. They connect to a RF coupler of your choice. You could likely also use RF detector outputs built into some transverters and amps, or diode based detector designs (such as from W6PQL or W1GHZ).

I am using inexpensive 6GHz capable AD8318 based imported modules commonly found on Amazon. They are under $10 each these days. It covers up to 8GHz with lesser accuracy and might even work well enough to 10GHz, I have not checked yet. You could use most any RF detector with a possible change to limit the voltage to your Arduino’s A/D input voltage spec and likely modify some of the Arduino side calibration related code and constants for slope, offset and inversion. The AD8318 and similar modules just plug in electrically-wise, and the AD8318 in particular outputs an inverted voltage curve between 0.5VDC and 2.3VDC compared to the AD8307 600MHz detector which outputs a normal rising voltage 0 to 2.5VDC which corresponds to rising power input. 

I built a small metal box to house the the 2 detector modules, included 9V and 5V regulators, powered it from 12V.  The 5V is powered from the 9V to reduce heating and is used to supply the 5V power to the Arduino via the USB port. You could put the Arduino in the same box as the detector to save cabling and cost.  That will be the case for the headless version planned (future). With the headless option you can package the Arduino in the same box as the detectors and mount on or near the RF coupler.

I have ordered an ADL5519 1MHz - 10GHz dual detector module from http://sv1afn.com. Will try that on the headless development.
 
Hardware pictures can be seen on the project Wiki pages
 
### RF Remote Power Meter application:
The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing 8 comma delimited character strings representing Meter ID, a sequence number, Band Name, Forward Power, Reflected Power, a SWR value. The data values for both forward and reflected power are sent out in dBm and Watts for convenience. 

One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script).  If using WSJT-X the app will automatically change the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.
