## RF-Power-Meter

4/11/2022 - Testing continues with PCB improvements with rev 0.04.  Fixed up some more footprints.  DB15, DB9 and RCA PTT jacks too close to use connector shells so spaced them out more.  Added 12V to pin 15 on each DB15 for external relay power. Added 2pin header in line with switched 12V so an external watchdog card (Swtich Doc Labs) can interrupt power to reboot the system.  Put a jumper on the pins if not using. There are 2 WD strobe signals on a connector (WD1 and WD2). Still need to test the internal watchdog capability so that the external card can be eliminated.  Tested all the input and output lines, PTT jack, all work. Calibrated the RF Sensors, temp, all working, installed resistors for onboard 12V measurement.  Current and HV are set to INPUT_PULLUP while nothing is connected to them.  Have not test the USB Host connector but not using that in this project.  The ethernet 6 pin header near the enet connector is 1mm pitch, should have been 2mm. Fixed on newer PCB layouts.  For this board will try soldering the ribbon cable wires in direct.  Made connector adapters to plug this PCB in place of the old prototype box to place this in service to start outdoor testing tomorrow.

4/9/2022 - I have created a PCB using opto-couplers and board edge connectors designed to go into a metal case for better RFI resistance (500W at 144MHz to a 9el yagi just 20 ft away).  The old internal proto board wiring acted as antennas on one of these units I have installed outside the house with several VHF high power amps located near the VHF antennas. On 2M when the antenna was alignbed with the box and > 100W, a CPU lockup resulted in PTT being stuck on.  A wachdog card added helped to recover, very important since I operate remote. On the 3/31/2022 version PCBs (8 made) the only internal wires are the USB Host port which is unused so not installed, and the ethernet 6 conductor ribbon cable and a 3.3VDC wire jumper (missed a trace). On the 4/9/2022 version PCB these wires are now routed on the board. Software to support the PCB related chages updated.  Has some new internal Watchdog stuff I am trying that may not work yet.

4/9/2022
   1. Added internal watchdog timer in hopes of removing the external WD card.
   2. Reassigned some pins to support a new PCB (V1) designed to hopefully be highly RFI resistant.  
   3. Added a Combo Ethernet/USB2 jack to the PCB and routed lines from the Teensy to it.
   4. PCB is using opto-couplers so some signals are inverted.
   5. Both Port A and B are 8 ports each, Port C is 3, Input has 6.  1x PTT Input, 2x PTT outputs. 
   6. Input uses a floating ground (Input_Return) that should be ties to the band decoder source ground.  
   7. Each input should supply about 1-3ma to activate the optocoupler. Change the resistors as required.
   8. If using the Nextion display, the serial port baudrate must be set to match the display firmware, the Nextin library (NexHardware.cpp) and what is set in the code. I am using 115200.
 


*** Release V2.5 created January 1, 2021 ***

 a. RF Wattmetter and Band Decoder combo on Arduino Teensy 4.1 with Ethernet option or PSoC5LP with Nextion Touchscreen, OLED, and/or headless  display options.
 
 b. Arduino remote controlled rotator controller with integraton into Desktop App UI.

Update 2/10/2021: Added command list for rotator remote commands in Wiki pages
Update 1/28/2021: Added new WIKI pages for the Arduino Rotator controller including a few pictures, BOM.
Update 1/24/2021: Needed a rotator controller I could operate remotely via Remote desktop or over the internet so I built one.  Did not need flashy features, sat or moon tracking or a display though a touchscreen will be added later that will be shared with a remoted (via UDP) RF Wattmeter screen. The rotator CPU (A Teensy 4.1) will be a UDP to Serial Gateway for the meter since the meter is located outside my house. Currently operates headless over UDP connection.

For now using the RF Wattmeter Desktop app and the existing button command codes except changed the cmd2 values (2nd value in the Python script command function, usually blank) and changed the IP address and ports to point to the rotator controller.  Will add Yeasu RS232 standard codes later.  You can view the rotor status streaming in the Python command window or the Serial Monitor.  

Supports stall detection, offset, start position, manual CW and CCW limits, stopband, fixed at 360 rotation today, north center (though the offset can shift that).  Fast and Slow with commands to go to right or left limits or to a heading.  10 Presets (0-9).  Automatcially turns correct direction to not pass though dead zone defined by rotor limits.  Slowdown value (default about 10 degrees) causes relay to switch to slow speed when nearing a preset or manual limit.  In the case of my HD-73 20VAC 3-wire rotor, that means inserting a 3A diode with electroliytic cap in parallel to reduce the voltage slowing it way down.  
 
Update 1/7/2021: Fixed bugs in Serial and Enet comms.  Added ADS1115 16-bit 4 channel ADC board for better RF power measuring performance. Putting the first complete Arduino Teensy Wattmeter and Band Decoder build into service in my remote VHF+ transverter/amplifier/antenna switch outdoor boxes this week, it now seems ready to go.
1. Modifed code to read a Bird Line section output which is roughly in the form of dBV. Got it working mostly but requires too many cal points to warrant the effort.  The AD83xx boards (including the ADL5519 dual detector I use these days output a linear, higher accuracy, temperature compensated voltage output representing a log power measurement.  If close cal with the Bird is not required then it does seem to work.  A significant UI effort would be needed to put that data in for every band.  It cannot be calibrated using this program's 2 point straight line approach (slope and intercept).  See the "Meter Builder" offering for a product that added digital cal on top of the Bird slugs extending the use of a single slug over wider frequencies and power ranges.
2. Measuring temperature, RF Fwd & Ref, all 3 on the ADS1115 ADC board. The onboard ADC is still usable for more things like voltage, current, Icom CI-V.
3. The ADS1115 is far higher resolution with a 16 bit SAR ADC and has PGA with more suitable input ranges (using 4V single ended inputs) and a decent onboard Vref source. The PSoC5 version uses a 20bit SAR ADC.
4. Using ADS1115 SingleShot mode. Continuous mode does not work well unless a 20ms delay is added between changing mux ports and measuring. Too short of delay also causes a startup hang.
5. Increased the i2c bus speed from 100KHz to 400KHz (fast mode) for better ADC results.
6. Found 8SPS on the ADS1115 was too slow.  64 or 128KSPS seem good.
7. Degugged several ethernet and serial port issues arising from toggling on and off the power data output streams. Goals is to always function regardless of outside cpomms state.  
8. Removed delays where possible.
9. Using the ADS1115_WE library per MIT license.  Tried a few, I liked this one best.  https://github.com/wollewald/ADS1115_WE
10. NOTE: Using the Teensy 4.1 alternate i2c pins. This requires using "Wire1" instead of Wire. Used #defines Wire Wire1 but I modified the library .cpp and .h fle and those #defines seemed to be ignored at times.
11. I have included on the Arduino side some code snippets from Band Decoder mk2 opens source Band Decoder project from http://RemoteQTH.com. The code is not used at this point but is staged (commented out) as a reference to implementing Icom CI-V support soon. This is particularly for IC-9700 users driving transverters and amplifiers but several other ICOM rigs will benefit.
12. 3 of the 4 ToDo items from below are still unfinished.  These are just nice to have features mostly to keep the Nextion in feature step with the Desktop App.

Update 1/2/2021: Added ethernet support into Desktop App.
1. Can choose to skip selecting a USB com port at startup. Probably need to turn off ethernet UDP commands if serial is enabled to prevent duplicate message side effects, if any.   
2. On the Arduino, started adding in support for Icom CIV and ACC jack band select voltage for optional band decoder inputs.

Update 1/1/2021: Added ethernet support to the Wattmeter/Decoder.  
1. Ethernet is a compile time #define ENET option, has an EEPROM setting (ENET_ENABLE) and config command (#53) to enable and disable it.  The same serial data to and from the CPU is now mirrored on ethernet. If ethernet is enabled but not working (no cable, hub) it will try to restart the enet system every 10 minutes and skips enet functions until
it is working.  
2. A patched NativeEthernet.cpp is provided to workaround a hang at startup if no ethernet link is active. You cannot use the Hardware or Link status functions until the Ethernet.begin is called, but the .begin hung on no link or hardware, a catch 22. DHCP times out properly so only an issue with static IP assignment.
3. Also updated was the debug message format. Can direct them to any serial port and now start with a first char of '>' to better separate them from data.  
4. The Desktop App was updated to print out the debug messages clearly and skip further parsing of these messages.
5. Added 3 small UDP test programs uploaded under the Teensy folder.  2 are simple examples modified to have 2 Arduinos talk to each other. UDP-CPU-2 was further modified to receive data from the Decoder and send some commands for testing.  The Desktop app will receive similar updates for ethernet comms to the Decoder.  The UDP-Radio-Test program has NTPClient and WSJT-X decoding, now with the decode packet time field working.  DT field is not yet.
6. Completed build of a new compact Teensy 4.1 based wattmeer/Band Decoder for remote installation outside of the house with my amplifers and transverter for 50 through 1296MHz.  Will use ethernet to monitor and control it. 4-wire BCD wiring from a radio will control the band selection enabling power on the selected amplifer, selecting the band on the 5-band transverter, and selecting a Bird line section output to connect to a peak-reading wattmeter meter co-located with the amps which is read by the Wattmeter.
7. ToDo: Update the Desktop App to support ethernet comms with the Decoder/Wattmeter - Done
8. ToDo: Add NTPClient and a new Nextion page for Clock display.
9. ToDo: Add new Nextion page for Config of all the new Band Decoder features equivalent to the Desktop App
10. ToDo: redirect Nextion Serial data from serial hardware to ethernet to a companion Arduino based program similar to UDP-CPU-2 test program.
11. Add Icom CIV and ACC Voltage Band Decoder features, esepcially for IC-9700 users.

Update 12/29/2020: Added a UDP test program on Arduino Teensy 4.1 which has native ethernet chip onboard.  Decoding selected WSJT-X messages the same as the Desktop App does to extract the radio dial frequency for band decoding use.  Plan to extend this to N1MM message decoding and to remote the Nextion Display and Decoder/Wattmeter over ethernet instead of a USB extender.  

Updates to 12/23/2020 added Band Decoder PTT Input and Output Active LO/HI settings.  Same for output ports so any or all can follow PTT, also with Active HI/LO setting options.  The latter is useful for routing PTT to specific amps or transverters, per band. Config screen controls added for these new PTT options.  Tested OLED and optimized layout for Combo Wattmeter and Decoder status.  Added picture of this new screen to pictures folder.  Changed default IO pin assignments to make wiring easier with ULN2803 drivers.  Also minor changes to account for the driver chip signal inversion.  There is a lot of debug prints which eventually foul up the serial port parsing, so will be shutting them down as I close out testing and changes coincident with the contruction of a new band decoder/wattmeter for my transverter/amp/antenna installation.  Added a 2nd PTT Out port with configurable delay to perform sequencing.  The decoder is between the radio and the transverter(s) and amps, in my case, PTT1 goes to the transverter(s) and PTT2 goes to all my amps. They share a common PTT, diode isolated - see Wiki page for this example of my setup. Amps will be keyed 1st on TX, last on transition back to RX.  The amps control the relays. This may not work for you.  In my case I am using split IF (separate TX and RX) with 1mW drive max so there is no chance of any damage to a transverter if the radio puts out a carrier before the transverter is switched over.  The ideal case is to pass back Tx_Inhibit or ALC to the radio to suppress the carrier.  Maybe later.

For December 2020 I added a full featured Band Decoder function with updated Desktop App Configuration page.  The full Band Decoder feature is only on the Arduino Teensy 4.1.  PSoC5 has a portion of this and will be updated to match soon.  Program will not fit on a Nano but does fit on the Mega2560.  Uses about 500bytes of EEPROM.  IO count and maybe serial port count can be a key factor depending on needs.

Either OTRSP serial commands or a hardware band input port (with 5 pins) can work to change bands and will operate 3x 8-bit ports, A, B and C.

By default Port A will mirror the input pins, useful for intercepting a BCD or 1-of-8 Band decoder from a radio on the input port then pass it on through Port A to a stack of transverters, or the Q5 Signal 5-Band transverter.

2 additional ports of 8 pins can serve amps and antennas or power meter coupler selection.

### The Decoder Modes:
A cool feature is the variety of translation modes for every port except as noted.  Each port generally has 5 translation modes.
    
    1. Transparent
      a. For input port read input as presented.
      b. For output ports write the input value direct to the output if enabled on that port. 
      
    2. 1-of-8 decode
      a. For input ports look for the first high bit and grab the value.
      b. For output ports the input value's is converted a demuxed value. Only one port pin goes high. The pin number matches the value. 
          Valid values are 0-7 equating to pins 0-7).
          
    3. Custom pattern (unique value stored per port, per band)
      a. For the input port it is used to search the Cal_table for a band with a matching record then change to that band
      b. For an output port write the pattern stored in that bands Port X field.  Enables can use this to emulate BCD or 1 of 8 type operation also.
      c. Can set a pattern to split the bits (thus pin) to operate multiple equipment on each band change.
      d. This is one way to link all 3 ouput ports to each band change. 
          You can change bands while the Config Screen is up and the custom values will update to that band's values.
      e. Port C will ignore AUX2 commands when it is set to the is mode.  The port follows the band change (from any source).  
      
    4. OTRSP Lookup  
      a. Output Ports only. AUX 1 is mapped to Port A and Port B.  AUX2 is mapped to Port C so it is independent of any other source of band changes.
      b. Will use the band matching AUXx value's custom value field on the output pins. Ignores the actual band change process.  
          Similar to Custom but only for Port A and B.
      c. Changing AUX2 will use the Port C values per band and does not affect Port A or B.  
          In Custom mode Port C follows the band change, here it does not.
      d. This is a way to link up Ports A and B output ports if each ports is first configured with a custom mode value that is the same (per band).
          You can change bands while the Config Screen is up and the custom values will update to that bands values.   
      e. If Disable OTRSP Band Change is active, The band will not change on AUX1 commands. All ports (with this feature enabled) will update according to 
          the received AUXx value.  This makes the port follow N1MM commands only without changing bands using the band's custom values for each port, per band.
          
    5. OTRSP Direct
      a. Output Ports only. AUX 1 is mapped to Port A and Port B.  AUX2 is mapped to Port C.
      b. Ignores any band relationship though the band will still change unless the Disable OTRSP Band Change option is on. 
          The value from the AUX command is directly presented on the port pins.
          
    6. Disable Band Change on OTRSP Commands 
      a. Enabled by default.  The band will change to match a new OTRSP AUX1 (only) command if sent.  
          AUX 2 never changes the Decoder band, just Port C.
      b. When Disabled, OTRSP messages do not change bands. AUX commands still apply to the output ports when appropriate.
          The Decoder still follows band changes from other sources.
      c. This makes the decoder an extension of N1MM directly controlling the 3 ports, A+B by AUX1 and Port C by AUX2 ignoring other sources.
          If the decoder receives band change info from other sources like the Desktop App or the Input Port, possibly initiated by N1MM via
          a radio control port connection, then the decoder band will change and the results should look the same as OTRSP Direct. If there are
          no other sources active then N1MM has complete control. If this is what you need then do not operate the Desktop App buttons and
          consider not connecting the hardware input port. Can get the same effect if all 3 ports were set for OTRSP Direct.
          
    7. Output Port Follow PTT Mode
      a. Any of the output ports with translation modes set to Transparent, 1-of-8, or Custom can now follow PTT. The 2 options are for Active High
         and Active Low.  This feature is useful to route PTT signals to specific equipment such as a LNA, transverter or amplifier.
      b. A logical enhancement would be to have configurable delays for each for sequencing. For example, Port A selects a transverter, Port B is transverter PTT, Port C is              remote LNA switch.  Port A acts 1st, followed by Port C on PTT then Port B.  Classic sequencer scenario.
         
    8. OLED screen updated to show Band Name and Band Decoder input and output port status in HEX
      a. Config is done via the Desktop App, later the Nextion screen also.
         It is possible to run both the Nextion and the OLED screens, as well as the Desktop App.

This is all stored in EEPROM.  Also fixed a bug where I calculated the EEPROM size of the main data table wrong causing a lack of EEPROM storage.  There is plenty of EEPROM after this fix, 11 bands and state variables consume a tad over 1K.

This work was done on a Teensy 4.1 and changes will be ported back to the PSoC5 platform.  Other Arduinos should work with some mods depending on the platform capabilities. I am using 2 USB serial ports (Main data and OTRSP), 1 hardware serial port (Nextion, optional), I2C port for OLED display (optional), and over 1K of EEPROM. Can reassign the OTRSP port, if used, to a hardeware serial port.  Can reduce the number of band records to fit into available EEPROM space.  Many of these features have an #ifdef created to skip these features at compile time if not wanted.  This is both the RF Wattmeter and Band Decoder in one.  Limitations of the internal Arduino ADCs need to be considered for RF power measurement detectors that have a small output voltage range. This can be worked around with external high resolutions ADC or amplification.  The Teensy is only 10Bit with the 3.3V power supply and reference.  Set expectations approriately. The PSoC5 has an internal 20bit ADC with optional on-chip PGA and multiple ref voltage options.

Known problems:
1. Desktop App: - No user impact. The Toggle Serial feature is used to suppress the data output to make seeing debug messages easier. This is disabled for now because the Desktop app seems to need RxD characters received before it will send out chars  The CPU side is OK, can manually toggle data OK.  Cmd = 100,120,239,X where X=1 or 0.
2. Desktop App: - No user impact. Observed that a 0.1 second delay is required between issuing a CPU command and receiving a reply if expected.  This is likely a queuing problem in the Serial Thread of the Python app.  This is annoying when trying to update button status for long duration operations or some configure actions that require a status update to refresh a screen.  The delays used today in the app may be installation/platform specific so are not a great solution.  Likely related to above issue.

Unfinished planned work:
1. Create configuration screens on the Nextion display for Band Decoder and for Voltage, Current and Temperature inputs. Can use the Desktop app for all of this today.
2. N1MM CW and PTT control using DTR and RTS signals over USB Serial Port is not working yet. This works on the PSoC5 but I have yet to make it work on the Arduino.
3. Analog Band Decode Input for radios with a 0-5VDC band decoder output.  FT-817 and IC-706 for example.  These radios use an analog voltage to represent the current band.  Teensy 4.1 is a 3.3V device so a voltage divider is needed.
4. Add SPI bus connected HI and LO side driver chip support to expand the IO. Would use TPIC6595N and MIC5891YN driver chips.  Also looking to use optocouplers on the input pins to beef things up if I elect to make PCB.  Would then use edge connectors to minimize internal wiring. These 2 chips are really 5V partrs with min input at 3.5VDC so not great with the 3.3V Teensy, will work fine with 5V Arduinos and the PSoC.

*** V2.4 updated on Master Branch on 12/12/2020.   This is the first working port from the PSoC5 to Arduino Teensy 4.1.  I have switched to the standard Arduino Nextion library fixing and resolved all warnings in the Nextion libary and the project compile. Everything seems to be working now except LoRa which is still the PSoC5 version so should remain disabled for now.  The OTRSP code has been reworked as well and now seems very robust and can decode BANDxY and AUXxYY commands from a 2nd serial port.  For the Teensy 4.1, in the Arduino IDE setup Dual USB ports. Serial is the main port, SerialUSB1 is the 2nd assigned to OTRSP comms.  The Desktop App works equally well with PSoC5 or this Teensy Arduino build.  Band decoding input should work but the output requires more coding and pin assignments. More below.  

The SSD1306 OLED library has been replaced with the Adafruit_GFX and Adafruit_SSD1306 libraries and works well. The LoRa wireless extension for the Nextion is not tested. Added several minor tweaks toteh Nextion library to reduce missed page events and now have very few missing page events or display write errors noted. 

So far the AD performance looks stable & adequate using a 10K pot to similate RF detector voltage. I will know more once I see an actual RF detector connected since very small voltage changes (log scale) result in large changes when converted to Watts (linear scale). The Teensy is still only a 10bit ADC but looks like the 3.3V regulator onboard results in a decently clean VRef voltage (3.3V).  Unlike the Nano and PSoC the Teensy is a 3.3 V part so you have to keep that in mind when connecting peripherals.  It does accept external 5V power (3.6 to 5.5V) and can be powered from USB 5V.  The Nextion uses an external 5V supply but has 3.3V tolerant TX and RX pins.  The Nextion firmware was updated very slightly to fix a bug or two and expand the OTRSP to 3 digits. The display is independent of CPU platform.

The other thing I am trying to is get VS Code to program the Teensy 4.1.  I can compile OK but VS Code does not see the Teensy in the Board manager.  I am trying out a utility call VirtualTeensy but have not got it working (fully) yet.  I do see it launch the Teensyduino programming tool like the Arduino IDE does.

Like the latest PSoC version 2.3 this has analog ports defined for 14V and HVDC, current detectror and temperature. 

This build has the band decoding features of the PSoC version Serial OTRSP and BCD input and parallel outputs.  The OTRSP Serial input decoding and display is working, will change the Wattmeter bands. I changed the OTRSP input parsing to be more robust and also now accepts true BCD hex value and displays as decimal on the Nextion. You connect via a USB comm port on a PC and send AUXxZZ and a \r (carriage return or CR).  x is Radio 1 or Radio 2, ZZ is 00 to FF BCD (0 to 255).  I believe N1MM+ and other loggers would only send out 0x00-0x0F BCD.  I have the band change function working when a valid AUX1ZZ message is received from Radio 1 (only).  It will also accept the command BANDxY where x is 1 or 2 (Radio 1 or Radio 2) and Y is 0x0-0xF representing 16 bands. It does nothing today, not sure if this is a real command in the OTRSP specs or not. 

Need 2 new Nextion pages.  One for calibrating the voltage, temp and current inputs.  The desktop app can be used for now.  2nd is a page to setup custom band decoding patterns. 

*** V2.3 uploaded to Master Branch on 11/6/2020.   See Revision History in the Wiki pages for full list of changes over time.  This release added LoRa to remote the Nextion screen wirelessly, removed SWR display spikes during TX On/Off transients, updated 3.5" Screen update to match 2.4" screen with band decoder changes, enabled 28v and 13V Voltage readings for embedded use in high power RF amplifier using the small KitProg board.  A few bug fixes added though 11/13/2020 to make the SWR more relaible during TX-> RX and RX-TX transitions and to force SWR to 0 while in RX. A new set of PSoC5 drawings uploaded to the PSoC5 folder showing all 4 IDE design drawing pages: Main, Band Decoder, Amp and Antenna Selection, and SWR Voltage Output for Amps. Calibration for the temp, current and HV and 14VDC are manaully calculated for now.  Need to add a touchscreen and Desktop app config capability for these values. The touchscreen already allows configuriung the Max value alarm thresholds. I finally hooked up my station 28V and 14V to the Wattmeter so I can remote monitor.  A month ago my 28V went offline while I was 3,000 miles away, I knew something was wrong becasue my amp output was 3W. It either false tripped on Hi SWR. SWR was good at 3W so I must have lost 28V amp power. A phone call confirmed my 28V power supply was offline due to an unknown fault. Turned out to be a burned output protection relay coil. Now using a FET power switch, same as used in my amps.  Now I can see the actual power common to all my 28VDC amps. The 1296 amp displays its own voltage info on its own OLED display but I am not remote monitoring that today. Also updated the PSoC code to send out the voltage, current and temp data in a new message type 171 and read and display that message in the Python Desktop app.  Can now hide the title frame to save desktop space by specifying HIDE or hide as the 3rd argument on the command line. I create a desktop shortcut for each meter instance with meter ID, Com port and Hide as the args and the Python cmd window minimized.  Can close using Exit in the File menu. The voltage, current and temp calibration values are now stored in EEPROM and commands 84-88 are used to set the calibration by supplying the actual voltage.  The supplied voltage will be divided by the measured voltage resulting in a cal factor that is saved when you press the Save to Meter button. Cmd 85 simply reads the current sensor voltage when there is NO load on the source to set the 0 current offset. Needed for sensors like the ACS712 which are AC and DC so 0 is 2.5V.  Cmd 86 takes the externally measured value and (like all the others) and figures a scale factor based on the current ADC voltage.  For current it does this after subtracting the zero offset voltage. Nov 24 continued building out the volt/curr/temp support by adding Cal buttons for these measurements ont eh Desktop App Config Screen.  Still need to add the equivalent to the Nextion display pages. Tweaked the Current No load cal process to be able to tweak the Current Zero Offset value up or down some (change only in main.c).  On 11/25/2020 I moved some files around into folders to clean up things a bit.  Pictures, Nextion files are grouped, Wiki pages are updated, and the Desktop app is in its own folder now also along with the pywsjtx files.

*** V2.2 uploaded to master Branch on 7/27/2020.  
This is a PSoC code change only to add initial support for AUX IO BCD pin outputs and Meter band change from N1MM+ logger using the Antennas tab. This should be particularly valuable for N1MM users who do not have a IF radio with native support for transverters.
N1MM+ uses OTRSP protocol over a serial port to issue many radio commands mostly for SO2R ops. We are just picking off the AUX commands for transverter and antenna control and wattmeter band change control. If using the USB hub with UART converter there are no wiring changes except bring out wires from the 4 new GPIO pins on port 0 to control your devices. See Wiki Revisipon History page for more details on limitations and setup instructions in N1MM, as well as likely near term enhancements on this. 
*** Note - 7/27 and 7/28 made updates to fix dropped messages and discovered random AUX messages for Radio #2 showing up.  Added parsing for the AUX 2 (Radio2) messages and added 4 more IO outputs on Port_1 pins 3-7 for AUX2. Rewrote the parser to be a single function, deleted the Serial2.C file as it was just a 2nd buffer, and now pull all incoming characters into a string before processing them for easier message debugging.
** Note - 8/1 - Continued debugging Aux msg error and Meter band display switching back to HF after each band change. Modified Serial parsing to look for possible BAND msg, have not seen one. Also not seeing any more unexpected AUX2 messages, believe they were parser errors, now cleaning out the buffer each use and looking for any length message to max of 16 chars vs fixed 7 byte length. All seems to be working as it should now.  Tested with N1MM on 2 K3s. One with real transverters configured up to 1296, the second with no actual transverters using various HF IFs. This simulates a radio that does not natively support transverters. Set N1MM to do transverter offset for each band (right click in the Bandmap Window) through 10GHz and assign to the proper IF. Right click in the Callsign Entry Windows to enable each band wanted. The Antenna Table (In Config) has antennas (in my test setup) defined on slots 1-10 for each band from any HF on slot 0 (all bands comma-separated 1.8 to 28), slot 1=50, slot2=144, and so on for 222, 432, 902, 1296, 2300, 3400, 5650, and finally 10000 on slot 10. This gives you AUX output codes on a 4 bit port for values of 0-10 BCD. AUX2 ports are provided as well in case Radio 2 is configured.  A problem I think I found is one radio with transverter offsets defined will not work with a radio with real transverters in SO2R. The transverter offset table is not assignable to a radio. This should be a feature request to the N1MM team.

General Notes:

Planned and unplanned work is complete and rolled up in Release V2 download.  Check out the Wiki pages for Bill of Materials, a drawing, and more details.  https://github.com/K7MDL2/RF-Power-Meter-V1/wiki

I have started work on the next features. It will include N1MM logging program antenna/transverter selection, scanning for RF among multiple couplers using a solid state RF switch.  A programmable attenuator added to remove (most) of the need for external fixed attenuators. If I can find something, a frequency counter board to eliminate manual or extrnal calibration selection. If I have time I will merge the features from the PSoC version to the Arduino. The M5Stack did get a merge about a month ago so it is fairly feature rich as is. I have previously used a SS SP6T RF switch and 31dB programmable ss attenuator in my Multiband central LO project last year.

### Summary Description

DIY Arduino and PSoC based RF SWR\Wattmeter for any band HF through microwave depending on the coupler and detector modules you choose. Reads output from a pair of power detector modules that you assemble into a box. This code is currently using either 1 dual 10Ghz detector (ADL5519) 2 8GHz detector modules (2x AD8318). They are attached to a RF dual directional coupler to read forward and reflected power. Optional Python based remote monitoring and control desktop app monitors the USB serial port output from the meter and can change calibration sets for different frequency bands. If using WSJT-X the app will use the broadcasted dial frequency (over UDP) to automatically set the right calibration set. Also has support for optional OLED and/or Nextion color LCD touch screens. You can see the latest pictures on my web site project pages. The PSoC version with 10GHz detector, OLED and 3.5" touchscreen is here at https://k7mdl2.wixsite.com/k7mdl/rf-wattmeter-on-psoc5lp. With the PSoC5, using the optional bootloader component you can use the Kitprog board (the small 1" square break-off PSoC5 programming board) as the host CPU.

The hardware flow for the latest build is: https://github.com/K7MDL2/RF-Power-Meter-V1/blob/master/Pictures/RF%20Wattmeter%20Hardware%20Block%20Diagram.JPG and also on the Wiki page https://github.com/K7MDL2/RF-Power-Meter-V1/wiki/Components-of-the-System
                                                                      
For ease of dev I am using a small USB 4 port hub with onboard UART TTL converter with the KitPRog plugged in for dev work.  It reduces 3 or 4 USB connections to 1.  This hub also enables extended scenarios such as N1MM+ logger program interfacing for antenna or transverter control.


### Key files and folders
    
    RF_Wattmeter_Teensy41: Arduino verson on the Teensy 4.1 CPU.  Ported over from the PSoC5 version Dec 2020. 
            Will attempt to keep this version in sync with the PSoC5 version.  OLED, Nextion and headless display options as before.
            In this verson I am using the standard Nextion Arduino library (included) with some minor changes to better track page
            changes and resolve compiler warnings.  No Nextion serial line switching feature for firmware updates in this hardware.  
            ADC is only 10-bits but seems to have low noise so might work for you well enough when measuring RF power.  
            Be prepared to add an external 16bit ADC module for more accurate RF power measurements if needed (such as the ADS1115 or ADS1100).
            Works fine for votlage, current and temp measurements.  I wil be using this version to monitor a Bird peak reading wattmeter
            buffered output which is in Watts, so it is a linear output vs. the usual log output from a normal Rf detector.
            
    Rotor-Controller-UDP-Server.ino and related files: headless Rotator Controller operating on a Teensy 4.1 over ethernet UDP.  
            Uses the RF Wattmeter Desktop App for now resusing the same buttons. App to be updated for rotor support later.
    
    RF_Power_meter.ino: Arduino code that runs on the M5Stack with graphics and buttons.
            Has some new ADS1100 files to support external I2C connected ADSS1100 16bit ADC units from M5Stack.  
            They work really well, 0-12V inputs, 15 bit useful range. I have a 4 channel version module very similar
            to try out based on the ADS1115. Intended for adding measurements for voltage and temperature.
            Some of the features from the PSoC are merged into the M5Stack at times.

    RF_Nano_Headless: Arduino code ported to the Nano CPU. 
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
    
    Nextion Files: The *.HMI files are config files for the 2.4" and 3.5" Nextion intelligent displays.  They have the same layout/fucntion scaled to size.
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

2.2 - Added LoRa to remote the Nextion screen wirelessly, removed SWR display spikes during TX On/OFF trnasients, updated 3.5" Screen update to match 2.4" screen with band decoder changes, enabled Voltage readings for embedded use in high power RF amplifier.

2.3 - Added option to hide the titlebar in the desktop app, additional work on voltages and SWR spike control, new serial message to output voltage, current and temp. Desktop app now displays and configures those values.  Current sensor voltage input moved to pin P3_7 since the Kitprog version (which has limited I/O) needed the old pin for SWR output voltage in the amplifier HI SWR circuit.  Clean up files grouping things together some.

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
