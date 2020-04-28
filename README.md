# RF-Power-Meter-V1
Arduino based RF wattmeter with Python desktop monitoring and control app

Arduino side
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
    d.	Serial port is still active for commands and status and debug, just does not send out power data
4.	Added Direct Band change (10 bands) commands
    a.	In addition to the previous Band, Scale, SWR, and serial output data rate buttons.  
    b.	Saves a lot of time cycling through 10 bands.
    c.	Has potential use later for the PC side program to read the current radio active band via direct, or a logger or a UDP message from WSJT-X.


Future stuff I have in mind later as I see no urgent need for this right now.
1.	Wi-Fi version of serial data logging and control
2.	Remote cal data editing (send any value to one of the 20 cal values stored (2 per band)
a.	Add a Menu system for advanced config and cal upload t a headless system
3.  Include sliders or up/down buttons to dial in while watching output values
4.	reduce the averaging on the Arduino some for more data response onscreen.
5.	Try to speed up the response of the data (due to long term averaging in the Arduino  for better accuracy – the AD seems noisy
6.	Headless version on Nano or other Arduino CPU – #2 makes this more complete but can be tested today as there are enough functions to be useful.  Things like EEPROM reset, serial toggle can be achieved by programming or adding new remote commands, same for  changing the cal data.  I see this as usefully to lower the cost and use the PC or a smartphone/tablet instead. A Nano sized CPU could fit in the same housing as the detectors easily and be located next to the detectors.  Use of the diode detectors would also shrink the total package size and cost, fewer cables, connectors and the relatively expensive coupler. 
7.	Standardize button sizes in the GUI.  Change background color.
8.	Play PC system alarm sound on high SWR value

This major update includes a reworked Python GUI
1.	Long thin format window to tuck away on the desktop easier. Not fancy but feels usable enough
2.	Buttons for direct band changes
a.	I future could read band data from the radio or a logging program (UDP, virtual serial port, WSJT-X UDP broadcast to loggers)
3.	Easier reading font (size, color and bold)
4.	Window title label you can customize
5.	Command line argument for COM port option.  Makes it easy to use with a shortcut to launch with the cmd window minimized full time.

 Multiple meter monitor support – This is a PC app problem, only need to change the meter ID in the Arduino code for each unit to be unique.  Current Python app filters on meter ID field t permit multiple unit monitoring later.


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
a.	Pressing any other button before power cycling the CPU will cancel EEPROM overwrite
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

