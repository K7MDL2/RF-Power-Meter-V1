# RF-Power-Meter-V1
Arduino based RF wattmeter with optional Python based remote monitoring and control desktop app

Version 1.0 RF Power Meter code running on a M5Stack (http://M5Stack.com) Arduino CPU module. Build yuour own RF milli-wattmeer for HF though microwave freuencies with low cost off the shelf modules and some glue wiring and a box. 

Optionally leverages WSJT-X status message UDP broadcasts to read the radio dial frequency and automatically command the meter to load the appropriate calibration values (10 bands supported). Using the awesome Python library to decode WSJT-X packets and is found at  https://github.com/bmo/py-wsjtx.

Components of the system:
1. Arduino with graphics display and buttons
2. A pair of Log Power detectors, one for forward power, one for relected power.  These are < $10 online.  AD8318 module is good to over 6GHz.  SWR is calculated and displayed on the meter face and digitally.  House them in a suitable small metal box.
3. RF dual directional coupler. Usually found surplus/used, pick one suitable for your power and frequency. Add additional SMA attenutors to handle higher power levels. Goal is to get full power at the detector input to be close to 0dBm (1mW), or the max input of your choice of detector.
4. Optional companion Remote Power Meter application written in Python and tested on Windows 10. 
    a. The Arduino send out serial port data over USB. Has GUI with buttons to select one of 10 bands holding calibration values and display the power in watts and dBm and the SWR. 
    b. Since it is simple Python it should also run in Linux and other supported OS versions. 
    c. The app is a small GUI window for your PC desktop that displays the USB serial data from the power meter. 
    d. Monitors WSJT-X (2.1.X tested) UDP broadcasts for the current radio frequewncy and automatically changes the meter calibration set via serial command.  Otherwise use one of the 10 buttons.
    e. Can be configured to work with JTAlert by changing the UDP for JTAlert re-broadcasts.
 
The CPU reads a pair of log power detectors connected to a dual directional coupler commonly found in the surplus market.  This could be for any frequency up to your chosen RF Log Power detector’s limits. With the AD8318 I am using today, that is 6Ghz to maybe even 10GHz. The Grpahics display has a analog meter face for power i Watts with selectable scales and a digial data bar below the meter face with digital values for Forward, Refelected and the SWR.  The power levels are also displayed in dBm.
 
RF Power Meter hardware:
The RF Power Meter is a Arduino CPU module with built in small TFT graphics screen with 3 buttons, and features Wi-Fi, Bluetooth, SD card, and a USB C port.  

I plan to enable wireless data connection in a future version.  With the recent changes to support remote commands the requirement for a graphics screen is mostly removed so I am looking at enabling a headless mode to enable running this on simple inexpensive Arduino boards to enable lower cost, simpler packaging and remote placement of the detector and CPU.  Enabling wireless will be that more compelling for remote placements.  

I think it would be interesting to place 2 identically calibrated units at each end of your coax and watch for the changes over time and measure the actual cable loss.  The meter features 10 “Bands” or sets of calibration values.  Each set contains Band Name, Forward and Reflected Port Coupling Factor.  The coupling factor is a value in dB representing the coupler port’s coupling factor at a given frequency plus any added attenuators and finally accounts for minor cabling and detector related variances (fudge factor).
Direct Band changes (10 bands) are possible using serial port 1 byte commands in addition to emulating the hardware buttons to change Band, Scale, SWR. Saves a lot of time cycling through 10 bands.
 
RF power detectors:
2 Log Power Detectors are used with their outputs fed to the AD input of the Arduino. They connect to a coupler of your choice. You could likely also use RF detector outputs built into some transverters and amps, or diode based detector designs (such as from W6PQL or W1GHZ).

I am using AD8318 based imported modules commonly found on Amazon.  They are under $10 each these days. It covers up to 8Gz, might work well enough to 10GHz, I have not checked yet.  You could use any RF detector with a possible change to limit the voltage to your Arduino’s AD input voltage spec and likely modify some of the Arduino side calibration related code and constants for slope, offset and inversion.  The AD8318 outputs an inverted voltage curve bewtween 0.5 adn 2.3VDC compared to the AD8307 600MHz detector which outputs a normal rising voltage 0 to 2.XVDC with rising power input. 

I built a small metal box to house the the 2 detector modules, included 9V and 5V regulators, powered it from 12V.  The 5V is powered frm teh 9V to resduce heating and is used to supply the power to the Arduino via the USB port.  You could put the Arduino in the same box as the detector to save cabling and cost.  That will be the case for the headless version planned (future).
 
 <insert pictures here>
 
RF Remote Power Meter application:
The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing the Meter ID, a sequence number, Band Name, Forward Power, Reflected Power, an SWR value.  The data values for both forward and reflected power are sent out in dBm and Watts for convenience. One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script).  If using WSJT-X the app will automatically change the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.
 
Snapshot on 432Mhz with a well matched antenna

 <insert pictures here>
 
Sample on 1296 with a not so good antenna match.  Over 3.0 and SWR will turn to RED.

 <insert pictures here> 
 

Arduino side notes:
1.	On startup if the EEPROM is not marked as having valid cal and state data written to it, default cal data and current state data will be written to EEPROM
2.	Factory Reset EEPROM
    a.	Must be on SWR or Watts screen for buttons to be read for this procedure
    b.	Press A button for 5 sec
    c.	Press C button for 5 sec in sequence
    d.	Press any other button now to abandon
    e.	The EEPROM is marked for erase (byte 0 no longer ‘G’ for Good 😊
    f.	Immediately power cycle the CPU.  On startup the EEPROM will be rewritten with default cal and current data
    g.	Pressing any buttons that save state or cal before the power cycle mark the EEPROM good again and not reset the EEPROM data since changes save sate and remark the EEPROM byte 0
3.	Press C button for over 10 seconds will toggle the serial port data output.  
    a.	It is on by default.  Switch is stored in Byte 4 of EEPROM so survives power cycles.
    b.	Set to 115200 baud over the USB.  Saves CPU and battery.
    c.	Must be on SWR or Watts screen for button to be read for this
    d.	Serial port is still active for commands and status and debug, just does not send out power dat


Future stuff I have in mind later as I see no urgent need for this right now.
1.	Wi-Fi version of serial data logging and control
2.	Remote cal data editing (send any value to one of the 20 cal values stored (2 per band)
    a.	Add a Menu system for advanced config and cal upload t a headless system
    b. Include sliders or up/down buttons to dial in while watching output values
3.	Reduce the averaging on the Arduino some for more data response onscreen.
4.	Try to speed up the response of the data (due to long term averaging in the Arduino  for better accuracy – the AD seems noisy
5.	Headless version on Nano or other Arduino CPU – #2 makes this more complete but can be tested today as there are enough functions to be useful.  Things like EEPROM reset, serial toggle can be achieved by programming or adding new remote commands, same for  changing the cal data.  I see this as usefully to lower the cost and use the PC or a smartphone/tablet instead. A Nano sized CPU could fit in the same housing as the detectors easily and be located next to the detectors.  Use of the diode detectors would also shrink the total package size and cost, fewer cables, connectors and the relatively expensive coupler. 
6.	Standardize button sizes in the GUI.  Change background color.
7.	Play PC system alarm sound on high SWR value
8. Multiple meter support – This is a PC app side problem only. Just need to change the meter ID in the Arduino code for each unit to be unique. The Python side filters on the meter ID number. To support multiple meters on the Python side, the graphics need to be extended or jsut run several instances of the current version on their unique com ports with uniquew meter_id set.  This likely works now, I have not tested it yet.


Button function summary
Button A (left)
1.	Short press – 
    a.	selects scale screen if not already on it
    b.	If scale screen is displayed, increments analog meter scale.  Digital outputs have no scale.
    c.	In Cal Screen, navigates down and left
    d.	In Cal edit mode, decrements value
2.	5 sec press – Used as 1 of 2 button sequence to reset EEPROM to default.
Button B (middle)
1.	Short press – Changes Bands.  There are 10 bands for better frequency calibrations  
2.	1 sec press
    a.	Enter and exits Cal mode
    b.	Selects a value to edit
    c.	Commits a value being edited
Button C (right)
1.	Short Press – 
    a.	Selects SWR analog meter display (No effect on digital fields)
2.	5 sec press – Used as 2nd button of 2 button sequence to reset EEPROM to defaults
a   .	Pressing any other button before power cycling the CPU will cancel EEPROM overwrite
    b.	EEPROM reset cannot be performed while in the Cal screen
3.	10 sec press – Toggles the power meter serial data output.
    a.	Does not affect normal serial status messages or ability to accept remote commands
    b.	115200 baud if you want to hook up an ASCII terminal program to see the output.  Can easily change in the code.

Python monitor and control program
1.	Start from command line with or without a COMX port as argument
    a.easiest with a shortcut adding the COMX port name on te command line.  Set the window to open minimized.  
    b. Can some status text scrolling inte text (terminal) window
4.  This version uses a threaded system to run the serial and network in independent threads. 
    a. The On/Off GUI button turns the serial port handler and the netowrk port handler on and off together.  They could be separated out if desired.
3.	Buttons send control commands to change bands, change speed of output data slightly
4.	Turn on or off he PC side serial port
5.	Customizable Title Bar by editing text in script
6.	Uses pySerial and Sys packages

