## RF-Power-Meter

*** Release V2 created July 1, 2020 ***

Most of the planned work (and much more unplanned) is complete in time for a break while I start traveling in July. 

I plan to build a new mtere with some new features later this summer for my 2nd QTH. It will include scanning for RF among multiple couplers using a solid state RF switch, programmable attenuator and if I can find something, a frequency counter board. If I have time I will merge the features from the PSoC version to the Arduino.  The M5Stack did get a merge about a month ago so it is fairly feature rich as is. I have previously used a SS SP6T RF switch and 31dB programmable ss attenuator in my Multiband central LO project last year.

### Summary Description

DIY Arduino and PSoC based RF SWR\Wattmeter for any band HF through microwave depending on the coupler and detector modules you choose. Reads output from a pair of power detector modules that you assemble into a box. This code is currently using 2 8GHz rated detector modules. They are attached to a RF dual directional coupler to read forward and reflected power. Optional Python based remote monitoring and control desktop app monitors the USB serial port output from the meter and can change calibration sets for different frequency bands. If using WSJT-X the app will use the broadcasted dial frequency (over UDP) to automatically set the right calibration set.   Also has support for optional OLED and Nextion color LCD touch screens.  You can see the latest pictures on my web site project pages. The PSoC version with 10GHz detector, OLED and 3.5" touchscreen is here at https://k7mdl2.wixsite.com/k7mdl/rf-wattmeter-on-psoc5lp.

### Key files
    RF_Power_meter.ino: Main Arduino code that runs on the M5Stack with graphics and buttons.
            Has some new ADS1100 files to support external I2C connected ADSS1100 16bit ADC units frm M5Stack.  
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
            from the PSoC Creator 4.3 IDE. Drop the expended archive into your workspace to open in PSoC Creator

    pyPowerMeter.py: Desktop Python app. Can run multiple instances on unique serial port and meter IDs.
            Now has a separate config screen (very early work) supporting new auto calibration function for Fwd and Ref power.
            Host CPU will calculate offset and slope now. Coupling attenuation is now fixed value (Feature in PSoC ony for now).
    
    *.HMI files: These are config files for the 2.4 and 3.5" Nextion intellgent displays. 
            The code to support these 2 displays and a 0.96" OLED display exist only in the PSoC version today.
            There is a near identical Nextion library for Arduino on GitHub and merging this code into the Arduino
            version would not be difficult.
            The Nextion library used for the PSoC is a community sourced adaption from Arduino C++ to C.
            Makes a great desktop display or stand alone instrument. 
            Added analog inputs for measuring high and low DC, temperature and current with alarm setpoints configured in
            one of the Nextion screens.  

### Revision History:

1.00 - Original release

1.01 - Some version control housekeeping, typo and a few small bug fixes.  Fixed meter needle disappearing at 0 input.

1.02 - Under dev in branch 1.02 as of May 5, 2020.  Master and V1.02 in sync on May 15. These all work together on the same protocol now. 

** NOTE ** The remote data protocol has changed for RX and TX to both be string based with comma separated values and have similar structure. This means you cannot mix previous versions of Arduino or Python code (1.00/1.01) with this or later code (1.02+). Ths was done to support full headless operation with expanded command messages and to support multipe meter instances (each on their own serial port).

The 1st value is meterID as before, the 2nd is now Msg_Type (150 for meter power data out, 180 for cal table dump, 120 for command to meter), and the rest are payload with \r\n terminating each message.  For commands, the 3rd value is the actual command (0-255), the 4th value is an optional data value for that command (such as coupling factor number for the 432 Fwd port). 

Key Changes:
1. Place the window in the upper right corner of screen on app open. Plan to create a config file to remember placement position later.
2. Changed the name of pyRFPowerMeter.py removing the Version number in the name.
3. Added more serial port error handling. For example, if usb is unplugged, the comms the ON/OFF button turns off and red and shuts down the serial thread.  When problem is fixed turn on again and it will gracefully resume. Also added startup error in case specified port is not there. Offers a list of USB ports.  Also you can now run without any serial port (has limited use but good for the curious without a meter. 
4. The network thread is now decoupled from the On/Off button and is always running.  
5. Using network data and heartbeat from WSJTX to implement a failover on data source form Band and Radio ID.  Color and ID name changes to UI fields show data source changed.  Using the WSJTX heartbeat message to timeout the netowrk sourced data.
6. Applied factory A/D correction library functions or A/D non-linearity on the Arduino (ESP32 CPU).
7. Sped up the sample rate in the Arduino and in the GUI update rate for faster meter response.  
8. Bug fix - Scale on wattmeter should return to the last used scale however the scale is incremented after a reset and the needle does not know it so is pegged out until you hit the scale button (local or remote) which syncs things back up. Fix tries to make the scale stay put as it was last.  Awaits further testing - still an open issue as of 5/7
9. Created RF Wattmeter.ino as a headless version of Arduino code tested on the M5Stack. Most of the screen draws are removed. As of 5/8/2020 it can take remote commands to edit any of the 20 calibration values. The Python app is updated to send some test comamnds using the previous scale, speed and SWR buttons. The previous scale button now dumps the meter's current cal values (Cal table).  Changed to a multibyte command message formate similar to the meter data power level out messages.  Changed the sequence number (2nd field) to now be a msg_type field to enable future expansion of cammon categoaries and for more protection against random data looking like a legit message.  Noticed just recently a new behavior in using the Arduino IDE serial monitor to manually send command bytes, it now causes the CPU to reboot. Commands sent via the Python app have no problem.  Possibly a difference in handling different terminating characters, TBI.
10. Created RF_Nano_Headless.ino derived from the headless M5Stack version in RF_Wattmeter.ino as the headless version tested on an Arduino Nano, all M5Stack dependencies for screen and buttons and ADC are removed, now using solely remote commands. Added remote commands for Meter CPU reset, Reset EEPROM to default values, turn on and off serial power level output messages.  Assigned the greyed out buttons in the Dsktop UI to temporary commands to test these. 3.4G = CPU reset. 5.7G followed by 10G will erase EEPROM byte 0.  Next reboot will force EEPROM to be reloaded from defualt values.  Use the 3.4G button to do that or hit the CPU reset button. Other commands issued befoer the reboot is done will normally abort the EEPROM overwrite since they cause an EEPROM write to store current values thus setting EPROM byte 0 back to 'G', so no EEPROM overwite will occur on next reboot.  
11. Also in this version the desktop UI handles a serial hang when CPU is reset (Catches a serial decode error condition)
12. Now supports multiple meter monitoring. Can accept a new MeterID on the UI app command line as the 2nd argument. You can now run multiple instances of the UI app as long as you match each instance to the correct serial port and Meter ID. The meter ID is hard coded into the meter code. WSJTX messages are braodcast with no filtering done by the app at this point. If not using multicast (as with JTAlert rebroadcasts) then the first instance to start will grab the connection and the 2nd will not hear the broadcasts.
13. Have nearly completed porting the Nano_Headless version over to Cypress Semiconductor PSoc5LP which is similar cost and size but far higher AD specs with multiple reference voltage level options and programmable hardware digital and analog blocks for signal conditioning and more I/O for controlling things.  Looking at embedding these meters in my new high power RF amplifier builds, implement sequencing and fault trips, temp monitoring, remote bypass/operate commands and remote monitoring. I will probably add back some sort of screen for local status on the amp, but will be set up to be easily used or not.  Could also just use a few LEDs.  See my Multiband LO project for a good example.
14. Updated RF_Power_Meter.ini to have the complete remote command set and new serial message protocol merged from the Nano_Headless version.  Not compatible with V1.01 or V1.00.
15. Deleted Remote_Wattmeter arduino code. A headless M5stack version is not useful now that a Nano and PSoC version is working.
16. As of 5/27/202 the headless version was succesfully ported to a Cypress Semiconductors PSoC5LP CY8CL-059 dev module (~$10 each) with instrumentation quality AD converters for lower noise and higher accuracy in close to the same form factor as the Nano. fast converson with upt o 20bits resolutions, uses an configurable Analog Mux on a DeltagSigma AD converter component with op amps in front for gain scaling and buffering.  The VRef is more flexible as well.
17. 5/28 - Afer initial calibration testing on the PSoC version, the speed and stability of the PSoC is much better than the other platforms. Using 20 bit resolution, reduced the averaging to 1 ADC sample feeding the smoothing function now only 2 deep. Increased the serial data output rate to take advantage of the rapid stable readings.  Now remote meter response is at final value in 2 serial port updates, far faster than before, just short of instantaneous, limited by the serial data output interval.  The PSoC files are what is contained in a "minimal archive" meaning you can drop the set of files as organized into your PSoC Creator 4.2+ folder area and you can open it and it should be complete and compile and run no problem (once you set your target CPU correctly in the IDE). 2 USB ports are used, one for the programmer connection during development (has full hardware debugger), the other is the onboard micro USB connector which all application messaging take place on (USB Serial port).
18. 5/31 - Fixed problem not reading EEPROM back into memory (EEPROM code is quite different then on the Arduino), tuned the ADC area a bit, 0 fudge factors used now, cal values are almost same as the printed coupler cal chart now.  Added SSD1306 controler based OLED driver code in prep for testing a 1" OLED display intended for embedding into a high power RF amplifier.  Far happier with the PSoC ADC performance so switching to it for the handheld 10GHz version and the RF amp embedded version.   Need to expnad the desktop app to make calibration easier such as just using a slider control or +/= buttons to dial in the desired power level target and not worry about the actual values.
19. 6/10/2020 - Updated some of the M5Stack code with changes from the PSoC code.  Changed the Fwd ADC to an external 16-bit ADS1100 unit from M5Stack and tested very stable. Need to buy or build a Grove cable Tee and convert the Ref port ADC to a 2nd ADS1100 unit. As an alternative I have a pair of ADS1115 4 channel I2C ADC PCBs ($4 each) for testing. It is a near cousin and should also work well. This is interesting for embedding in something like an RF amp to measure temp, HV, 12V and current as well as fwd and ref power.  This is in the M5Stack code only right now.  The PSoC internal ADC is even higher resolution and just as good in a smaller PCB size using the KitPRog board if desired.
20. 6/10/2020 - Reassigned some I/O pins on the PSoC5LP design to match the available pins on the KitProg programming break-off board. Same chip with fewer available I/O pins. These are usually broken off after development is done and collected in a jar. They are 1" square and very capable. They require you add a bootloader compnent and configure it, select the different CPU device and upload the same code. Very easy. You develop and debug on the main module then use a bootloader utility to upload to a Kitprog with bootloader in it. 
21. 6/10/2020 - Add support for a 2.4" Nextion intelligent display.  Still support the OLED.  Put all display code in #ifdef statements to select either/none/both displays. Have 3 pages. The main status page now showing Current, temp, HV voltage, 12V. Numbers turn red when they exceed a max threshold. The thresholds can be set by the 3rd page with sliders for each. Also the meter ID and manual band change is there. The 2nd page has 2 manual sliders to fine tune the ref and fwd coupler cal.   The auto cal method developed days later then this is a beter approach, but changing coupler values can still get close and works when you cannot genrqte a steady RF carrier to measure.  Remote commands added to set slope and offset for each port (fwd and ref) as detector boards can be different.  This is set for the active band.  The current band is displayed.  A Save COnfig button commits  changes to EEPROM on both config pages. This is currently in the PSoC5LP version.
22. 6/15/2020 - Added 2 different auto cal mechanisms.  The first one takes a serial commands with a target power level in Watts.  It then loops adusting the coupling factor number to get the power value to match.  This worked OK but does nothing to improve the slope and offset.  While in the adjustment loop the desktop app Cal button turns red.  This is triggered by new output messages 161 (in progress) and 160 (finish). The ADC voltage is displayed in these messages when you need to see the raw voltage measured. This is currently in the PSoC5LP version.
23. 6/16/2020 - Added a better auto cal method.  Ths calculates offset and slope based on 2 measurements. ADC voltage measurments are taken on serial command and mapped to a supplied power level (in Watts).  High power is measured first. If the low power is hit without a vaild high power reading made, it skips any changes. A carrier is applied, the command is sent with power level value.  Then the power lowered and 2nd command is sent with that low value in watts. For example 100W and 10W. The CPU then does the math to determine slope and offset. You enter the coupler+attenuator values for each band beforehand and this method does not alter them like all the methods before did. The advantages are: Accounts better for detector variations in frequency respnse, offset and slope parameters. The slope inversion, if any, is now automatically figured, as is any offset, if any.  It does require a known accurate RF carrier be applied at 2 levels, preferably at the high and low ends of the range. Supplying best guess numbers before cal will save some calc time but it is still fast.  No manual voltage measurement or super accurate attenuator/coupler numbers required anymore.  This is currently in the PSoC5LP version.  With separate slope and offeset for each port now, the Cal Table structure was expanded to store them per band along with the coupling factor and attenutor already there.  The EEPROM functions had to be updated to match. A CPU that has not seen these changes you need to set a #define RESET_EEPROM to force an ovewrwrite of the EEPROM to the new structure or you could get hangs from weird values read back from the EEPROM since the cal table structures are completely changed to support the auto cal slope and offset storage.
24. 6/19/2020 - Completed GUI startup screen to set Com port and MeterID values. 2 listboxes are presented with an OK button. For MeterID the default is the value named in the source file. It will be overridden by cmd line arg or the GUI input. Serial Port will show only USB serial ports available and default to the first one, if any. Hit OK button will accept the defaults or any listbox selections made. You can start the GUI app without a serial connection, just cannot do much. The WSJTX part can be seen to work, and later will be able to go into the config edit screeens and config file read and save features when written. Also when the new auto-cal is active, the Cal button will turn red.  During this time you want to leave the power level steady until measurement is completed and stored in EEPROM.  Only takes a few seconds.
25. 6/25/2020 - More progress on the Config screen windowing and all buttons now work as intended except for the color changing part durng calibration. Added support for a 3.5" Nextion display, adding new dBm fields, arranged things to better fit the larger display area, added a new graphing screen for Fwd, Ref, and SWR. The 2.4" display should continue to work, nearly all the work is in the display side code. The host is writing some new fields for dBm on the main page and power/SWR on the new page that will kick back serial error messages from the display. Plan to add the same pages into the 2.4" version later.  The display code for both Nextion and OLED is easily included or not with #ifdef for each display type. The PSoC is the most current version code since it is a more cable IDE with hardware level debugging and ability to enable bootloader (or not) to utilize the Kitprog snap-off programming boards. Periodically I merge most features into the Arduino time permitting. One way to use the Nextion dispay is to package the display separate of the CPU/detector box and connect with a 4 wire cable with +5V, Gnd, Serial TX and RX.  Baud rate can be changed to suit length of cable.
26. 6/28/2020 - Built new RF Wattmeter box using the 3.5" display, a full PSoC5LP module (main and KitProg boards), 5V regualtor, a 4 port USB hub module (a Raspberry Pi Zero Hat) with onboard USB-UART converter, and a 10GHz Dual RF Detector (ADL5519). The ADL 5519 is really 2 AD8318 single detectors like used in this project but are on the same silicon so comparisons between forward and reflected measurements will be more accurate. It has a wider linear range up to 8GHz, 10GHz is less linear but usuable with cal. A shared Vref and common temp (and temp correction) make it more accurate as well between channels. It has comparators off the Vout lines creating terminals OutP and OutN. These are positive and inverterd levels that are equivalent to Return Loss. The temp port can be read and further temp correction can be applied for greater accuracy still. The only changes needed were:
###
    a. Added mux and demux to switch the Nextion display serial lines between the CPU UART and the USB hub UART. This enables uploading new files to the Nextion display from the PC without touching any physical switches or wires. A 1-bit wide control register flips the switch on command from a remote serial command. Normally the display will be connected to the CPU UART. Keeping the program on the main PSoC CPU device (vs using the Kitprog) enables development without needing manual resets for bootloader operation like required with a KitPRog board. 2 USB ports are required, one for programming/debugging, one for the CPU normal USB serial output. A 3rd USB is required for the Nextion display updates. That is why the compact 4 port hub module with onboard UART is so useful.  This also achieves my goal of remote development such as in a Remote Desktop Session.  The TX to the USB UART is set as a SIO pin with Vref at Vdd/2 (2.5V) to suit the 3.3V RX input on the USB board since Raspberry Pi GPIO is 3.3V. The Nextion is 3.3V on the serial lines but tolerates 5V input on RxD.  It is now working good with remote switch commands.
    b. New remote command for the UART switching.
    c. Now reading temperature from the detector board Temp output pin. This is not the same as a RF amplifier heat sink temp as originally envisioned and displayed on the Nextion screen but is potentially useful for calibration corrections.
    d. Not using using the voltage divder inputs yet so those still have dummy values.
27. 6/29/2020 - Changed SWR graph scale to be more accurate with more vertical axis lablels. Added vertical line separator on the power graph each time the scale button is pressed to separate the scale data visually.  
28. ADL5519 results were nearly the same as the dual 8318s, just a recalibration needed. Results were very close to spec sheet values. Being a single board, construction and accuracy are both improved, slightly higher cost. My source is from SV1AFN.com in Greece. I will be ordering a couple more. It took the postal service 6 weeks to get it here last time. It sat for weeks at various locations.
28. 6/30/2020 - Completed Desktop app Config page. Button layout done with on screen calibration instructions. 2 text entry boxes provided to specify the hi and lo power levels to be measured. Apply carrier at each power level, then hit measure button to caclclate the slope and intercept.  Previous Host commands altered a bit to seprate the measure from the calculate actions. To commit, use the "Save to Meter" button to write values to EEPROM. Future additions would be saving last screen positions and upload and download complete cal tables from a file.
29. 7/1/2020 - Now shows the ADC voltages captured during Cal. Calculate (fwd or Ref) buttons are disabled until both a hi and lo reading are taken since the Config window was opened. After that you can just reread either one, the calculation uses the last saved values. Went through a compelete calibration on both forward and reflected on 6 bands 50-1296MHz and the resulting slope and offset numbers are tightly grouped and close to the ADL5519 spec sheet. Will probably extend the Config UI to accept power level input in either dBm or Watts for convenience.  Note the desktop app Fwd and Ref do nothing at this time. YOu can edit hte script to plug in any command-calue pair you like for testing. This about wraps up this phase of work and will create a V1.2 release package soon.
30. 7/1/2020 - Added choice of dBm or Watts for calibration power levels.  Displays the measured ADC voltages for each power level.  New commands added for dBm measurement levels.  Renumbered the related cal commands.  Desktop and PSoC versions only have these so far.  M5Stack will work standalone and most serial commands from the desktop app should work but it is not fully up to date with the latest latest PSoC changes so expect some issues.
31. 7/1/2020 - Release V2 created.  There is more to do but they are in niche corners and significant new areas like wireless control.  
 

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

My ADL5519 1MHz - 10GHz dual detector module from http://sv1afn.com just arrived on 6/13/2020. Also received are 2.4" and 3.5" CNC cut bezels for the Nextion displays.  They are from the UK (http://compfranon.uk). One of them will will go into a case for a 10GHz portable test instrument.  The other into a box on my desktop as a dedicated station RF Power meter.
 
Hardware pictures can be seen on the project Wiki pages, more on my website.
 
### RF Remote Power Meter application:
The RF Remote Power Meter companion application is a small GUI display app that monitors the Power Meter USB serial data information containing comma delimited character strings representing Meter ID, Message Type, and a variable data payload based on message type.  For example: Forward Power dBm, Reflected Power dBm, Forward Power W, Reflected Power W, and SWR. Calibration data is sent and received also along withi status messages.

One simple feature of the app is to turn the SWR value field background red for SWR value > 3.0 (or any number you want in the script).  If using WSJT-X the app will automatically load the Power Meter’s calibration set to match the current radio frequency band. Otherwise there are manual band buttons in the app to change the meter’s current calibration set. The Python script code contains lots of usage and configuration details in the comments at top of the file.  The Nextion display, if used, also uses color on any of the voltages, temps, current and SWR to indicate maximum value exceeded.  

A configuration edit screen is available in the menu.  It handles things like calibration, factory reset, and data dumps.
