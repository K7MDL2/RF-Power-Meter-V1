## RF-Power-Meter

*** Release V2 created July 1, 2020 ***

*** Note: V2.3 uploaded to Master Branch on 11/6/2020.   See Revision History in teh Wiki pages for full list of changes over time.  This release added LoRa to remote the Nextion screen wirelessly, removed SWR display spikes during TX On/Off transients, updated 3.5" Screen update to match 2.4" screen with band decoder changes, enabled 28v and 13V Voltage readings for embedded use in high power RF amplifier using the small KitProg board.  A few bug fixxes added though 11/13/2020 to make the SWR more relaible during TX-> RX and RX-TX transistions and to force SWR to 0 while in RX.  A new set of PSoC5 drawings uploaded to the PSoC5 folder showing all 4 IDE design drawing pages: Main, Band Decoder, Amp and Antenna Selection, and SWR Voltage Output for Amps.  Calibration for the temp, current and HV and 14VDC are manaully calculated for now.  Need to add a touchscreena nd Desktop app config capability for these vcalues.  The touchscreen already allows configuriung the Max value alarm thresholds.

*** V2.2 uploaded to master Branch on 7/27/2020.  
This is a PSoC code change only to add initial support for AUX IO BCD pin outputs and Meter band change from N1MM+ logger using the Antennas tab. This should be particularly valuable for N1MM users who do not have a IF radio with native support for transverters.
N1MM+ uses OTRSP protocol over a serial port to issue many radio commands mostly for SO2R ops. We are just picking off the AUX commands for transverter and antenna control and wattmeter band change control. If using the USB hub with UART converter there are no wiring changes except bring out wires from the 4 new GPIO pins on port 0 to control your devices. See Wiki Revisipon History page for more details on limitations and setup instructions in N1MM, as well as likely near term enhancements on this. 
*** Note - 7/27 and 7/28 made updates to fix dropped messages and discovered random AUX messages for Radio #2 showing up.  Added parsing for the AUX 2 (Radio2) messages and added 4 more IO outputs on Port_1 pins 3-7 for AUX2. Rewrote the parser to be a single function, deleted the Serial2.C file as it was just a 2nd buffer, and now pull all incoming characters into a string before processing them for easier message debugging.
** Note - 8/1 - Continued debugging Aux msg error and Meter band display switching back to HF after each band change. Modified Serial parsing to look for possible BAND msg, have not seen one. Also not seeing any more unexpected AUX2 messages, believe they were parser errors, now cleaning out the buffer each use and looking for any length message to max of 16 chars vs fixed 7 byte length. All seems to be working as it should now.  Tested with N1MM on 2 K3s. One with real transverters configured up to 1296, the second with no actual transverters using various HF IFs. This simulates a radio that does not natively support transverters. Set N1MM to do transverter offset for each band (right click in the Bandmap Window) through 10GHz and assign to the proper IF. Right click in teh Callsign Entry Windows to enable each band wanted. The Antenna Table (In Config) has antennas (in my test setup) defined on slots 1-10 for each band from any HF on slot 0 (all bands comma-separated 1.8 to 28), slot 1=50, slot2=144, and so on for 222, 432, 902, 1296, 2300, 3400, 5650, and finally 10000 on slot 10. This gives you AUX output codes on a 4 bit port for values of 0-10 BCD. AUX2 ports are provided as well in case Radio 2 is configured.  A problem I think I found is one radio with transverter offsets defined will not work with a radio with real transverters in SO2R. The transverter offset table is not assignable to a radio. This should be a feature request to the N1MM team.

General Notes:

Planned and unplanned work is complete and rolled up in Release V2 download.  Check out the Wiki pages for Bill of Materials, a drawing, and more details.  https://github.com/K7MDL2/RF-Power-Meter-V1/wiki

I have started work on the next features. It will include N1MM logging program antenna/transverter selection, scanning for RF among multiple couplers using a solid state RF switch.  A programmable attenuator added to remove (most) of the need for external fixed attenuators. If I can find something, a frequency counter board to eliminate manual or extrnal calibration selection. If I have time I will merge the features from the PSoC version to the Arduino. The M5Stack did get a merge about a month ago so it is fairly feature rich as is. I have previously used a SS SP6T RF switch and 31dB programmable ss attenuator in my Multiband central LO project last year.

### Summary Description

DIY Arduino and PSoC based RF SWR\Wattmeter for any band HF through microwave depending on the coupler and detector modules you choose. Reads output from a pair of power detector modules that you assemble into a box. This code is currently using either 1 dual 10Ghz detector (ADL5519) 2 8GHz detector modules (2x AD8318). They are attached to a RF dual directional coupler to read forward and reflected power. Optional Python based remote monitoring and control desktop app monitors the USB serial port output from the meter and can change calibration sets for different frequency bands. If using WSJT-X the app will use the broadcasted dial frequency (over UDP) to automatically set the right calibration set. Also has support for optional OLED and/or Nextion color LCD touch screens. You can see the latest pictures on my web site project pages. The PSoC version with 10GHz detector, OLED and 3.5" touchscreen is here at https://k7mdl2.wixsite.com/k7mdl/rf-wattmeter-on-psoc5lp. With the PSoC5, using the optional bootloader component you can use the Kitprog board (the small 1" square break-off PSoC5 programming board) as the host CPU.

The hardware flow for the latest build is: https://github.com/K7MDL2/RF-Power-Meter-V1/blob/master/RF%20Wattmeter%20Hardware%20Block%20Diagram.JPG and also on the Wiki page https://github.com/K7MDL2/RF-Power-Meter-V1/wiki/Components-of-the-System
                                                                      
For ease of dev I am using a small USB 4 port hub with onboard UART TTL converter with the KitPRog plugged in for dev work.  It reduces 3 or 4 USB connections to 1.  This hub also enables extended scenarios such as N1MM+ logger program interfacing for antenna or transverter control.


### Key files
    RF_Power_meter.ino: Main Arduino code that runs on the M5Stack with graphics and buttons.
            Has some new ADS1100 files to support external I2C connected ADSS1100 16bit ADC units from M5Stack.  
            They work really well, 0-12V inputs, 15 bit useful range. I have a 4 channel version module very similar
            to try out based on the ADS1115. Intended for adding measurements for voltage and temperature.
            Some of the features from the PSoC are merged into the M5Stack at times.

    RF_Nano_Headless.ino: Arduino code ported to the Nano CPU. 
            All screen and button code removed, complete remote control.
            Could be merged with the other version's features as needed but for now is not being updated.

    RF_Wattmeter_PSoC5LP: Cypress PSoC5LP platform used for far better AD and signal processing. 
            Use headless or with optional Nextion and/or small OLED display.
            I am developing new features on this platform first. Can use the main module or the programmer
            module via a simple bootloader procedure. This is an archive file (like a zip file) produced
            from the PSoC Creator 4.3 IDE. Drop the expanded archive into your workspace to open in PSoC Creator
            Current code is setup for the ADL5519 Dual 10GHz power detector from SV1AFN.com.  2 AD8318 modules can be used
            instead for up to 8Ghz and slightly less accuracy due to component variances.

    pyPowerMeter.py: Desktop Python app. Can run multiple instances on unique serial port and meter IDs.
            Now has a separate config screen supporting new auto calibration function for Fwd and Ref power.
            Host CPU will calculate offset and slope. Coupler attenuation is now fixed value (Feature in PSoC only for now).
    
    *.HMI files: These are config files for the 2.4" and 3.5" Nextion intelligent displays.  They have the same layout/fucntion scaled to size.
            The code to support these 2 displays and a 0.96" OLED display exist only in the PSoC version today.
            The original Nextion library for Arduino is on GitHub so would be easy to merge this code into the Arduino platform.
            The Nextion library used for the PSoC is a community sourced adaption from Arduino C++ to C.
            Makes a great desktop display or stand alone instrument. 
            Added analog inputs for measuring high and low DC, temperature and current with alarm setpoints configured in
            one of the Nextion screens (PSoC5 only).
            No need to disconnect Nextion display for programming, the PSoC will switch the serial lines between USB ports.

### Revision History:
See Wiki page here https://github.com/K7MDL2/RF-Power-Meter-V1/wiki/Revision-History

1.00 - Original release

1.01 - Some version control housekeeping, typo and a few small bug fixes.  Fixed meter needle disappearing at 0 input.

1.02 - Most of the new feature work was done here. Master and V1.02 in sync on May 15. These all work together on the same protocol now.

** NOTE ** The remote data protocol has changed for RX and TX to both be string based with comma separated values and have similar structure. This means you cannot mix previous versions of Arduino or Python code (1.00/1.01) with (1.02+). Ths was done to support full headless operation with expanded command messages and to support multiple meter instances (each on their own USB serial port). The 1st message value is meterID as before, the 2nd is now Msg_Type (150 for meter power data out, 180 for cal table dump, 120 for command to meter, 170, 16x, etc), and the rest of the fields are payload (variable length) with \r\n terminating each message. For commands, the 3rd value is the actual command (0-255), the 4th value is an optional data value for that command (such as coupling factor number for the 432 Fwd port). 

2.0 - Major feature adds July 1, 2020- Available in the Release V2 download.

2.1 - Nextion Screen adds July 25, 2020.  Changes committed to Master branch.  2nd Nextion PSoC RF Wattmeter build completed with 4 port hub and UART converter.

2.2 - Added LoRa to remote the Nextion screen wirelessly, removed SWR display spikes during TX On/OFF trnasients, updated 3.5" Screen update to match 2.4" screen with band decoder changes, enabled Voltage readings for embedded use in hgh power RF amplifier.


### Info:
Version 1.0 RF Power Meter code running on a M5Stack (http://M5Stack.com) Arduino Basic Core CPU module.  No extra core modules required.  You will need 2 AD8318 based RF log power detector modules, or suitable alternatives with some code minor adjustments to adapt the calculation for different output V slope and offset and if no slope inversion.

Later versions 1.02+ also run on Arduino Nano (headless) or Cypress Semiconductors PSoC5LP (CY8CKIT-059 dev module). New features are usually created on PSoC first since it has a better IDE and debugger.

The meter features 11 “Bands” or sets of calibration values. Each set contains Band Name, Forward and Reflected Port Coupling Factor. The coupling factor is a value in dB representing the coupler port’s coupling factor at a given frequency plus any added attenuators and also accounts for minor cabling and detector related variances (a fudge factor). Direct Band changes (11 bands) are possible using serial port commands in addition to emulating the hardware buttons to change Band, Scale, SWR. Saves a lot of time cycling through 11 bands.

For remote monitoring a Python based application runs on your PC and gets data via the USB serial port. It can optionally leverage WSJT-X status message UDP broadcasts to read your radio dial frequency and automatically command the meter to load the appropriate calibration values (11 bands supported). An awesome Python library is used to decode WSJT-X packets and is found at  https://github.com/bmo/py-wsjtx.

Documentation and some pictures and a screen shot are on this project's Wiki Pages. Look there for configuration instructions for Python setup and app configuration as well as the hardware description. Button Operating Procedures are on the Project Wiki page also.

You can also find more information about this and my other projects such as the Multiband LO and Remote Antenna Switch at my website.  This project can be found there at https://k7mdl2.wixsite.com/k7mdl/arduino-rf-remote-wattmeter.

While V1 is now created, there are many features yet to add. I plan to enable wireless data connection in a future version.  With the support for remote commands that now exists the requirement for a local graphics screen is mostly removed so headless mode is possibe running on simpler inexpensive Arduino boards if desired to enable lower cost, simplify packaging, and make for easier remote placement of the detector and CPU by enabling a wireless data connection. I think it would be interesting to place 2 identically calibrated units at each end of your coax and watch for the changes over time and measure the actual cable loss. 

Choice of CPU platform should include your AD Cnverter accuracy needs. Some have internal ADC subject to noise or low resolution but may stil be usable. An easy improvement is to use a ADS1100 or ADS1115 type extenral ADC module connected by I2C bus.  These have 15 usable bit when connectd as single ended devices and have shown to be very stable and quiet.   M5Stack makes a low cost 1 channel ADS1100 "Unit" in a small case that is nice.  I found a small 4 port ADS1115 online also inexpensive.  The extra channels are good for adding voltage and temperature that might be needed for monitoring a RF amplifier for example.
 
### RF power detectors used for the first version:
2 Log Power Detectors are used with their outputs fed to the A/D input of the Arduino. They connect to a RF coupler of your choice. You could likely also use RF detector outputs built into some transverters and amps, or diode based detector designs (such as from W6PQL or W1GHZ).

I am using inexpensive 6GHz capable AD8318 based imported modules commonly found on Amazon. They are under $10 each these days. It covers up to 8GHz with lesser accuracy and might even work well enough to 10GHz, I have not checked yet. You could use most any RF detector with a possible change to limit the voltage to your Arduino’s A/D input voltage spec and likely modify some of the Arduino side calibration related code and constants for slope, offset and inversion. The AD8318 and similar modules just plug in electrically-wise, and the AD8318 in particular outputs an inverted voltage curve between 0.5VDC and 2.3VDC compared to the AD8307 600MHz detector which outputs a normal rising voltage 0 to 2.5VDC which corresponds to rising power input. 

I built a small metal box to house the the 2 detector modules, included 9V and 5V regulators, powered it from 12V.  The 5V is powered from the 9V to reduce heating and is used to supply the 5V power to the Arduino via the USB port. You could put the Arduino in the same box as the detector to save cabling and cost.  That will be the case for the headless version planned (future). With the headless option you can package the Arduino in the same box as the detectors and mount on or near the RF coupler.

Later meter builds featured an OLED dot matrix display driven by the PSoC5 KitPRog programming board. It is intended to be embedded in a high power RF amplifier. The latest 2 builds feature the ADL5519 1MHz - 10GHz dual detector modules from http://sv1afn.com, Nextion intelligent LCD color touch screen displays, and use  using the PSoC5. One used 2.4" and the other a 3.5" screen. I found CNC cut bezels for the Nextion displays at (http://compfranon.uk). They go into metal cases for a 10GHz portable test instrument or as a shack monitor enabling remote monitoring via remote desktop session. 
 
Hardware pictures can be seen on the project Wiki pages, more on my website.
 
### RF Remote Power Meter application:
The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing comma delimited character strings representing Meter ID, Message Type, and a variable data payload based on message type.  For example: Forward Power dBm, Reflected Power dBm, Forward Power W, Reflected Power W, and SWR. Calibration data is sent and received also along withi status messages.

One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script). If using WSJT-X the app will automatically load the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.  The Nextion display, if used, also uses color on any of the voltages, temps, current and SWR to indicate maximum value exceeded.  

A configuration edit screen is available in the menu.  It handles things like calibration, factory reset, and data dumps. It performs the detector calibration for forward and reflected power using a hi and a lo RF carrier of known power level to capture the ADC voltages and thgen calculates the Slope and Interncept point for each band and takes into account any detector output inversion such as from the AD8318 and ADL5519 detectors.
