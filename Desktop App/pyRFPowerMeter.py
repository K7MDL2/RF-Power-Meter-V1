# pyRFPowerMeter.py

from threading import Thread, Timer
import time
import socket
import select
import serial
import serial.tools.list_ports as cports
import sys
import tkinter as tk
import tkinter.font as tkFont
from tkinter import *
from tkinter.filedialog import askopenfilename

print('__file__={0:<35} | __name__={1:<20} | __package__={2:<20}'.format(__file__,__name__,str(__package__)))

import wsjtx_packets as pywsjtx
import simple_server
import tkinter.messagebox


PowerMeterVersionNum = "2.3"
# pyRFPowerMeter  Version 2.3  Nov 24, 2020
# Author: M. Lewis K7MDL
#
#   Companion Desktop app to montor and control the Arduino or PSoC5 version of the RF Wattmeter.
#
#   See project details on these websites
#   Github Wiki: https://github.com/K7MDL2/RF-Power-Meter-V1/wiki
#   GitHUb project Readme: https://github.com/K7MDL2/RF-Power-Meter-V1
#   K7MDL web site  - PSoC5 version: https://k7mdl2.wixsite.com/k7mdl/rf-wattmeter-on-psoc5lp
#                   - Arduino Version: https://k7mdl2.wixsite.com/k7mdl/arduino-rf-remote-wattmeter
#
#   Since the PSoC5 has more capable hardware, higher performance, better IDE with full debugging,
#       uses standard C language, similar size dev modules and costs the same I am leading new
#       development on the PSoC platform first, then occasionally merge appropriate features back.
#
#   Displays Supported  (can use #ifdef in .h file to exclude/include these): 
#       None (headless)
#       Nextion 3.5"
#       0.96" OLED (SSD1306 compatible)

#   There are some PSoC5 programmable hardware features (digital Mux/deMux and voltage controlled GPIO 
#       port) used to switch the Nextion display (if used) between the CPU and a USB serial line
#       to allow remote upload to the Nextion.  The latest hardware build I used a 4 port USB hub
#       with onboard USB USART TTL converter which is at 3.3V since it was meant for a Raspberry
#       Pi Zero.  
#
#   USB Port usage:
#       Main CPU serial connection to a desktop app (not required if using a local display - helpful for calibration from a PC app like this app)
#       Nextion Programming (only needed if the display is used - can use a temporary connection with a USB UART TTL converter)
#       KitProg programming/debugging board for main CPU (only needed for initial programming or firmware updates later)
#
#   KitProg usage (in place of the main board):
#       The programming board may be snapped off the Main CY8CKIT-059 dev module and used 
#       standalone. It has a special bootloader installed that can load a user app. A Bootloader component
#       included in the TopDesign drawing must be enabled and a new target CPU platform selected
#       (-039 variant from the list, the -097 is the main device variant) 
#       The Tools->Bootloader Host utility is used to upload the app program to the bootloader on the 
#       KitProg board. To see the board in teh Bootloader Host, unplug the KitProg, hoid the reset button
#       down, plug the KitProg back into the USB cable then let go of the switch.  You can now download.
#       At completion of download, power cycle the Kitprog and your app wil run normally thereafter.
#
#   All capabilities are controlled by serial port commands.  Not all commands are used anymore as better
#       commands such as in the calibraton area are preferred.
#    
#   This program reuses a version of the Nextion Arduino library on GitHub ported to C by another Github user.
#       https://github.com/itead/ITEADLIB_Arduino_Nextion
#       
#       Some modifications were needed in the UART area and some likely compiler bugs were found and worked
#       around.  Not all commands are supported by library calls so sprintf + sendcommand() functions are used
#       to send any command.  SOmetimes a result is expected and the library functions will collect those.  Not
#       doing so results in some delay or confusion if you are looking for a specific return message (like a get
#       value command stacked behind a status message)
#      
#   Uses the awesome WSJT-X python decoding package py-WSJTX 
#       https://github.com/bmo/py-wsjtx
#
#   The files are included in this release package. For original py-WSJTX download the files and extract into
#        the same folder where you locate this script.
#           for example this script uses the path to import the module from   
#                   pywsjtx.extra.simple_server.SimpleServer     
#           which is the same as ".\pywsjtx\extra\" on Windows OS so keep these pathname structures in sync

#   Requires the pySerial package to be installed in your Python environment.  
#   If you use pip try the following to install them if needed.
#       pip install pySerial

#  *********************************************************************************************************
#   Usage:  pyRFPowerMeter 
#       prompts you for a COM port name such as "COM1" where your K7MDL Arduino RF power meter is attached
#   Usage:  pyRFPowerMeter COM6
#       Specify the port name on the command line and it will start up without user interaction
#  *********************************************************************************************************

# This app is the companion application to the PSOC5LP and Arduino based RF power meter poject by K7MDL
#   It has no usage without one or more of the Arduino/PSoC5 meters.  This app listens to a serial port and 
#   displays the results in a small fixed size GUI window.  
#   It also sends commands to the meter such as to change calibration sets when you change frequency
#   You could emulate the same simple data format of 8 string fields and use this for your own device
#   The deafult serial rate (found at bottom of code below) is 115200 to match the default in my PSoC5/Arduino meter code

# This app accepts 2 command line arguments --  The serial port to use and the meter ID (100-119).
#   It will prompt for a USB Serial Port if one is not supplied on teh command line
#   In normal usage you would specify the com port to be used on the command line in a desktop shortcut or a batch file.  
#   You could also specify the "port_name" in the code at the bottom of this script

# Change these 2 lines to suit your station
myTitle = ("K7MDL Remote Power Meter " + PowerMeterVersionNum)      # Windows Title Bar Text

# edit these to match your meter ID and Rig/Location text for this meter instance
myRig = "K3 Camano"       # Rig name and location - about 10 characters max
myRig_meter_ID = "100"                 #  Change to set your default meter ID.  Overridden on cmd line or config file
                           # --> Always 3 digits, 100 to 119 only allowed.  
myWSJTX_ID = "WSJT-X"      # "WSJT-X" default as of WSJT-X version V2.1.   Change this to match your WSJT-X instance name. See below.

# examples to inspire ....
#myRig = "K3 WA"           # My home in WA state
#myRig_meter_ID = "102"    # Change to match your power meter ID.

#myRig = "HF"              # the HF rig
#myRig_meter_ID = "103"    # Change to match your power meter ID.

#myRig = "MicroWave"       # a microwave transverter maybe
#myRig_meter_ID = "104"    # Change to match your power meter ID.

#myRig = "Field"           # sample name suggesting meter is a standalone used for testing, not assigned to a antenna or radio 
#myRig_meter_ID = "105"    # Change to match your power meter ID.


#UDP_IP = '239.255.0.1'       # multicast address and port alternative
#UDP_PORT = 2237
UDP_IP = "127.0.0.1"        # default local machine address
UDP_PORT = 2334            # change to match your WSJTX source of data port number. 2237 is a common WSJTX default port.  See below for more info...
# I am using 2334 with JTAlert re-broadcasting

#  This program can optionally use WSJT-X UDP reporting broadcasts to automatically track your radio's frequency and send a command
#       to the Arduino RF Power meter to load the approriate calibration set.  11 bands are provided today.  
#       The remaining bands are all the VHF+ bands up to 10GHz by default  
#       You can change the labels in the code for the button band names to be anything.  Can also change them in the Arduino side.
#       The name is just a label, in the Arduino side is it used as a more friendly way to say BandX and represent a correspnding set of 
#           coupling factor + any attenuators + any cal correction factor for the detectors at a given frequency for 
#           the forward and reflected ports of a particular dual direction coupler.  
#       If you change couplers be sure to set new calibratiuon factors in the Arduino side as they are frequency sensitive as well as
#           could have different coupling factors such as 20dB or 30dB on each port.  
#       Single port couplers can be used as the dBm fields will be zero when the Forward port Watts is reported as 0W.  
#           The values will drift around on the unused port at other time and the SWR value may fluctuate.  Can edit these fields to blank if desired
#       In a pinch you could adjsut the incoming data values here.
#  WSJT uses 2237 to talk to JTAlert.  JTAlert can rebroadcast and uses 2334 by default.  
#  These ports can be changed in each program
#  Cannot open both this app and JTAlert on 2237 - bind conflict so set this app's port to 2334 to match the JTAlert
#  ==> If not using JTAlert make sure this port number is the same as configured in WSJT-X "UDP Port Number" field in Setting->Reporting tab
#  WSJT uses 2333 default for logged contact broadcast  
#  Can also use multicast address with WSJT but not sure how other programs like JTAlert work with it

#  If running multiple instances of WSJT-X (including on the same address for multicast usage)
#      you neeed to use the WSJTX ID to map the instance messages to the correct power meter app ID field.
#      Change the ID in the Arduino side code and set myRig__meter_ID above to match.  Must use a unique com port for each meter
#  By default the code here has wsjtx_id set to None and will read any and all WSJT-X instance status messages received.  
#      This will have the effect of constantly switching bands if the instances are reporting different frequency bands
#      You can asign a bogus UDP port nnumber to shut off WSJT-X message reception or tailor the WSJT-ID value described next.
#  
#  Each instance of WSJT-X has an instance name set with -rigname.  This name wil be used as the WSJTX_ID in all network messages
#  By default the this program is not filtering on this ID so wil lread any instance, including multiple instances which can get confusing
#  Change the WSJT-X ID in this code from None to teh name that matches your rigname.  
#  You can see the actual WSJTX ID received as it is dislayed in the terminal window each time a heartbeat is received
#      This is a section of the relevent code used in this script
#           if wsjtx_id is None and (type(the_packet) == pywsjtx.HeartBeatPacket):
#               # We have an instance of WSJTX
#               print("WSJT-X is detected, id is {}".format(the_packet.wsjtx_id))
#               print("--> HeartBeatPacket Received")
#               wsjtx_id = the_packet.wsjtx_id

# Global Vars
global s            # used to get network packet data
own_call = ""       # used for meter ID in Radio field of GUI when only network is on.
last_freq = ""      # used for detecting band changes from network
# This boolean variable will save the communications (comms) status
comms  = None   #  False is off.  App.comm wil then toggle to on state.
restart_serial = 0
heartbeat_timer = 0
send_meter_cmd_flag = False   # Boolean to gate Serial thread to send cmd byte to meter.  Cmd comes from USB thread
cmd = ""
cmd_data = ""
meter_data = ["","","","","","","","","",""]
meter_data_fl  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,] # stores the same info in same psotion when possible as a float
meter_data2 = ["","","","","","","","","",""]
meter_data_fl2  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,] # stores the same info in same psotion when possible as a float
cal_flag = 0
cmd_flag = 0
FwdVal_Hi = ""
FwdVal_Lo = ""
RefVal_Hi = ""
RefVal_Lo = ""
hide_titlebar = FALSE
Dis_OTRPS_Ch_flag = 0
Inp_Val = 0
PortA_Val = 0
PortB_Val = 0
PortC_Val = 0
InputPort_pattern = 0
PortA_pattern = 0
PortB_pattern = 0
PortC_pattern = 0
PTT_IN_POLARITY_Val = 0
PTT_OUT_POLARITY_Val = 0
CW_KEY_OUT_POLARITY_val = 0
PORTA_IS_PTT_Val = 0
PORTB_IS_PTT_Val = 0
PORTC_IS_PTT_Val = 0

def isfloat(x):
    # Check if the received 4 characters can be converted to a float
    try:
        float(x)
        return True
    except ValueError:
        return False

#_________  Serial Port Handler in its own thread______________________________________________________________________________
#   
class Serial_RX(Thread):
    def __init__(self):
        # Call Thread constructor
        super().__init__()
        self.open_serial_port()
        self.keep_running = True
        #print(" serial thread startup ^^^^^^^^^^")

    def stop(self):
        # Call this from another thread to stop the serial handling process
        self.keep_running = False
        self.close_serial_port()

    def run(self):
        # This will run when you call .start method
        while self.keep_running:
            self.meter_reader()

    def open_serial_port(self):
        if ser.isOpen() == False:        
            try:
                ser.open()
                ser.flushInput()
            except serial.SerialException:
                print ("error communicating...: " + str(port_name))
        else:
            print("Cannot open serial port "  + str(port_name))

    def close_serial_port(self):
        if ser.isOpen():        
            try:
                ser.close()                
            except serial.SerialException:
                print ("error communicating while closing serial port: " + str(port_name))
        else:
            print("Serial port already closed:  "  + str(port_name))

    def ser_meter_cmd(self):
        global send_meter_cmd_flag
        #global cmd
        #global cmd_data

        if send_meter_cmd_flag == True:   # if True then the UDP thread has a cmd waiting to send out.       
                if ser.isOpen():
                    try:              
                        #print("---> Write Cmd : " + str(cmd))
                        ser.write("{},120,{},{},{}" .format(myRig_meter_ID,cmd,cmd_data,'\n').encode())
                        send_meter_cmd_flag = False                             
                    except serial.SerialException:
                        print ("error communicating while writing serial port: " + str(port_name))
                else:
                    print("cannot access serial port to send commands")     
        
    def meter_reader(self):       
        global restart_serial

        out = ""  # Preparing the out variable 
        try:
            if restart_serial == 0:
                if ser.isOpen():        
                    if ser.inWaiting() > 0:
                        try:
                            out = ser.readline().decode() 
                        except UnicodeDecodeError: # catch error and ignore it
                            print("Unicode decode error caught")  # will get this on CPU resets
                        except serial.SerialException:
                            # There is nothing
                            print("No Data waiting at serial port: " + str(port_name))
                            return None
                        except TypeError as e:
                            restart_serial = 1      # restart serial_Rx thread to recover
                            print("Error communicating while reading serial port: " + str(port_name))
                            print(e)
                        else:
                            #print(out)
                            self.get_power_data(out)
                    else:
                        pass
                else:
                    print("Cannot access serial port to read input data")
                    restart_serial = 1      # restart serial_Rx thread to recover
        except serial.SerialException as e:
            print(" Error: Port access issue detected. Possibly disconnected USB cable.")
            print(" --> Shutting off comms. Hit \'On\' button to resume once problem is resolved. Actual error below:")                
            print(e)
            restart_serial = 1

    def get_power_data(self, s_data):
        global meter_data
        global meter_data_fl
        global meter_data2
        global meter_data_fl2
        #global comms
        global cal_flag
        global cmd_flag
        global FwdVal_Hi
        global FwdVal_Lo
        global RefVal_Hi
        global RefVal_Lo
        global Dis_OTRPS_Ch_flag
        global Inp_Val
        global PortA_Val
        global PortB_Val
        global PortC_Val
        global InputPort_pattern
        global PortA_pattern
        global PortB_pattern
        global PortC_pattern
        global PTT_IN_POLARITY_Val
        global PTT_OUT_POLARITY_Val
        global CW_KEY_OUT_POLARITY_val
        global PORTA_IS_PTT_Val
        global PORTB_IS_PTT_Val
        global PORTC_IS_PTT_Val
        
        try:
            if s_data != '':
                tempstr =  str(s_data).split('\r')
                #print("1  DATA HERE {}" .format(tempstr))
                meter_data_tmp = tempstr[0].split(",")  # break comma separated values into a list
                #print("2 TMP raw str   = {}" .format(meter_data_tmp))
                if len(meter_data_tmp) >= 3:
                    meter_data[0] = meter_data_tmp[0]  
                    if meter_data_tmp[0] == myRig_meter_ID: 
                        if meter_data_tmp[1] == "161":   # Cal progress message start
                            cal_flag = 1
                            FwdVal_Hi = meter_data_tmp[3]
                            FwdVal_Lo = meter_data_tmp[4]
                            RefVal_Hi = meter_data_tmp[5]
                            RefVal_Lo = meter_data_tmp[6]  
                            print("Voltages = ", meter_data_tmp)
                        elif meter_data_tmp[1] == "160":   # Cal progress message end
                            cal_flag = 0
                        elif meter_data_tmp[1] == "163":   # Cmd progress message start
                            cmd_flag = 1       
                            print(" -------------cmd flag = 1 -------------")                            
                        elif meter_data_tmp[1] == "162":   # Cmd progress message end
                            cmd_flag = 0                            
                            print(" -------------cmd flag = 0 -------------")
                        elif meter_data_tmp[1] == "172":   # Get Band Decoder Translation Mode Values resulting from Message to CPU #60
                            Dis_OTRPS_Ch_flag = meter_data_tmp[2]   # Get OTRSP band change enable state
                            Inp_Val = meter_data_tmp[3]             # Band Decoder Input Port Translation Value
                            PortA_Val = meter_data_tmp[4]           # Band Decoder Port A Translation Value
                            PortB_Val = meter_data_tmp[5]           # Band Decoder Port B Translation Value
                            PortC_Val = meter_data_tmp[6]           # Band Decoder Port C Translation Value
                            InputPort_pattern = meter_data_tmp[7]   # Band Decoder Input Port Custom Pattern Value
                            PortA_pattern = meter_data_tmp[8]       # Band Decoder Port A Custom Pattern Value
                            PortB_pattern = meter_data_tmp[9]      # Band Decoder Port B Custom Pattern Value
                            PortC_pattern = meter_data_tmp[10]      # Band Decoder Port C Custom Pattern Value
                            PTT_IN_POLARITY_Val = meter_data_tmp[11]     # Band Decoder PTT input Active HI/LO
                            PTT_OUT_POLARITY_Val = meter_data_tmp[12]    # Band Decoder PTT output Active HI/LO
                            CW_KEY_OUT_POLARITY_val = meter_data_tmp[13]    # Band Decoder CW_KET from OTRSP Active HI/LO
                            PORTA_IS_PTT_Val = meter_data_tmp[14]    # Band Decoder PortA follow PTT Active HI/LO
                            PORTB_IS_PTT_Val = meter_data_tmp[15]    # Band Decoder PortB follow PTT Active HI/LO
                            PORTC_IS_PTT_Val = meter_data_tmp[16]    # Band Decoder PortC follow PTT Active HI/LO
                            print("Band Decoder Translation Mode Values from CPU = ", meter_data_tmp)
                        elif meter_data_tmp[1] == "171":   # voltage, current and temperature data
                            meter_data2 = meter_data_tmp  
                            #print(" HV, 14V, Curr and Temp Data = {}" .format(meter_data_tmp))      
                            for i in range(len(meter_data2)):                    
                                if isfloat(meter_data2[i]):                                                                    
                                    meter_data_fl2[i] = float(meter_data2[i])
                                else: # Not a float so zero fill the field
                                    meter_data_fl2[i] = 0.0             
                        elif meter_data_tmp[1] == "170":   # normal power data
                            meter_data = meter_data_tmp        
                            for i in range(len(meter_data)):                    
                                if isfloat(meter_data[i]):                                                                    
                                    meter_data_fl[i] = float(meter_data[i])
                                else: # Not a float so zero fill the field
                                    meter_data_fl[i] = 0.0
                            meter_data_fl[2] = float(meter_data[2][:-3])    # convert band label to a number.  Ideally would use a RegEx to split at teh end of the numbers
                            if meter_data_fl[5] == 0 and cmd_flag != 1:
                                meter_data[3] = "0.0"          #  zero out the dBm values when F watts is zero
                                meter_data[4] = "0.0"
                            #print("{0:}    = {1:}" .format("3 ID Match Data ", meter_data))
                            #print("{0:} FLT= {1:}" .format("3 ID Match Data ", meter_data_fl))
                        else:                            
                            print(" CMD message as received by the meter = {}" .format(meter_data_tmp))
                            for i in len(meter_data):
                                meter_data[i] = ""
                                meter_data_fl[i] = 0.0
                    else:  # No ID Match
                        #print("Non Matching Meter ID = ", meter_data[0])
                        #self.debug_meter_string("4 No ID Match   ")
                        #meter_data[0] = "NA"          # no meter ID match so tell the UI  
                        pass
                else:   # Not 8 Elements
                    s_data = ""
                    #self.debug_meter_string("5 Not 8 Elements")                    
                    #meter_data[0] = "NA"          # no meter ID match so tell the UI  
            else:
                #self.debug_meter_string("6 Empty String  ")
                meter_data[0] = "NA"          # no meter ID match so tell the UI  
        except:
                pass
        self.ser_meter_cmd() # use period of no RX input to call cmd sender which will check a flag set by the UDP thread to see if any commands are wating to send out.


    def debug_meter_string(self, debug_msg):
        for i in range(len(meter_data)):
            meter_data[i] = ""
            meter_data_fl[i] = 0.0
        print("{0:}    = {1:}" .format(debug_msg, meter_data))
        print("{0:} FLT= {1:}" .format(debug_msg, meter_data_fl))

#__________  Network handler in its own thread.  ________________________________________________________________
#               Monitors WSJT-X broadcast packets to extract frequency for band changing
#   
class Receiver(Thread):
    def __init__(self):
        # Call Thread constructor
        super().__init__()
        self.keep_running = True
        global s
        s = simple_server.SimpleServer(UDP_IP, UDP_PORT, timeout=2.0)
        #s = pywsjtx.extra.simple_server.SimpleServer(UDP_IP, UDP_PORT, timeout=2.0)
        # print(" Starting network thread")

    def stop(self):
        # Call this from another thread to stop the receiver
        self.keep_running = False
        print(" Stopping network thread")

    def run(self):
        # This will run when you call .start method
        while self.keep_running:          
            self.wsjt_reader()

    def wsjt_reader(self):
        # We use select here so that we are not *hung* forever in recvfrom.
        # We'll wake up every .5 seconds to check whether we should keep running
        #rfds, _wfds, _xfds = select.select([self.sock], [], [], 0.5)
        #if self.sock in rfds:    
        wsjtx_id = None    
        global last_freq
        global meter_data
        global own_call
        global heartbeat_timer

        time.sleep(0.5)
        (pkt, addr_port) = s.rx_packet()
        if (pkt != None):
            the_packet = pywsjtx.WSJTXPacketClassFactory.from_udp_packet(addr_port, pkt)
            print("Looking only for WSJT-X ID of : {}" .format(myWSJTX_ID))
            if wsjtx_id is None and (type(the_packet) == pywsjtx.HeartBeatPacket):                
                # we have an instance of WSJTX
                print("WSJT-X is detected, id is {}".format(the_packet.wsjtx_id))
                print("--> HeartBeatPacket Received")
                wsjtx_id = the_packet.wsjtx_id
                heartbeat_timer = 0  # reset upcounter
            if type(the_packet) == pywsjtx.StatusPacket:
                wsjtx_id = the_packet.wsjtx_id
                own_call = the_packet.de_call
                if wsjtx_id == myWSJTX_ID:
                    print("Status message received from WSJT-X ID : " + wsjtx_id + own_call)
                    freq = str(the_packet.dial_frequency)
                    freq = str(freq[:-6])                    
                    if  freq != "":                     
                            band = meter_data[2]
                            band = str(band[:-3])
                            if band != freq or freq != last_freq:
                                #print(" Meter band not matched to radio: Meter: {}  Radio: {}" .format(band, freq))                    
                                self.send_meter_cmd(int(freq), "", False)
                                print('{} {} {} {} {} {}' .format("Dial Frequency is : ", freq, "    Was : ", last_freq, "   Meter Band : ", band))                            
                                last_freq = freq                                
                            else:
                                print("Frequency Now " + freq)         

    def send_meter_cmd(self, cmd_str, cmd_data_str, direct_cmd):
        # cmd is type chr to be converted to byte
        # direct_cmd is BOOL to specify if it is a direct command such as button push for 144 
        #    vs a UDP freqwuency which can vary in a range
        # direct_cmd is True to do a direct command
        # since serial and networka nd GUI are seperate threads, using a semaphore to specify that
        #   a command is waitng to go out.  The serial thread will pick it up when there is time between receives

        global send_meter_cmd_flag
        global cmd
        global cmd_data

        # parse the frequency passed to call specific band command byte to be sent by another function
        #print("Meter Cmd Order is to " + str(cmd))
        send_meter_cmd_flag = True
        if direct_cmd == True:  
            cmd = cmd_str           # now handle direct bands change commands
            cmd_data = cmd_data_str     # set data value if any. 
            #print(" ******> Direct cmd byte = " + str(cmd))
        else:            
            cmd_data = 0        # data value always zero for band change commands
            band = int(cmd_str)           # non direct band change commands
            if band < 49:
                cmd = "240"
            elif 49 < band < 70:
                cmd = "241"
            elif 69 < band < 200:
                cmd = "242"
            elif 199 < band < 300:
                cmd = "243"
            elif 299 < band < 600:
                cmd = "244"
            elif 599 < band < 1000:
                cmd = "245"
            elif 999 < band < 2000: 
                cmd = "246"
            elif 1999 < band < 3000: 
                cmd = "247"
            elif 2999 < band < 4000: 
                cmd = "248"
            elif 3999 < band < 8000: 
                cmd = "249"
            elif band > 7999: 
                cmd = "250"
            else: 
                pass                    # in case we add more

# # # _____________________Window Frame Handler for the GUI and managing starting and stopping._____________________________
# # #                       
class App(tk.Frame):
    STRIDE = 8
    DELAY = 100

    def __init__(self, master=None):
        # Call superclass constructor
        super().__init__(master)
        self.serial_rx = None
        self.receiver = None
        #self.grid()
        self.pack_propagate(0) # don't shrink
        self.pack(fill=BOTH, expand=1)
        self.createWidgets()
        self.update()
        self.master.protocol("WM_DELETE_WINDOW", self.exit_protocol)

    def exit_protocol(self):
        # Will be called when the main window is closed
        # It should close the serial port if it has not
        # been previously closed
        #global comms

        if comms:
            if ser.isOpen() == True:                
                if self.serial_rx:
                    self.serial_rx.stop()
                if self.receiver:
                    self.receiver.stop() 
        self.master.destroy()  # Destroy root window
        self.master.quit()  # Exiting the main loop

    #---- Create the GUI Layout ----
    def createWidgets(self):
        global meter_data
        global meter_data_fl
        global meter_data2
        global meter_data_fl2
        global restart_serial
        global own_call

        self.btn_font = tkFont.Font(family="Helvetica", size=10, weight='bold')

        self.QUIT = tk.Button(self,
                                    text = format("ON"),
                                    font = self.btn_font,
                                    relief = tk.RIDGE,
                                    fg = "black",
                                    padx = 4,
                                    #state='disabled',
                                    command = self.comm)     # Will call the comms procedure to toggle all comms 
        self.QUIT.place(x=10, y=0, bordermode=OUTSIDE, height=20, width=40) 
        if restart_serial:
            self.QUIT.configure(fg='black', bg="light grey") 
        else:
            self.QUIT.configure(fg='white', bg="red") 
        if comms == None:
            self.QUIT.configure(text = format("OFF"), fg='grey', bg="light grey")

        self.band_f = tk.Button(self)
        self.band_f["text"] = "Band"
        self.band_f["command"] = self.change_band  # Change the band (cycle through them)
        self.band_f.configure(font=self.btn_font)
        self.band_f.place(x=50, y=0, bordermode=OUTSIDE, height=20, width=40) 

        self.scale = tk.Button(self)
        self.scale["text"] = "Cal?"
        self.scale["command"] = self.get_cal_table  # Change to Watts and change scale
        self.scale.configure(font=self.btn_font)
        self.scale.place(x=90, y=0, bordermode=OUTSIDE, height=20, width=40) 

#       These bands can be enabled but will need to place them and slide around the voltage, curr and temp

#        self.band_23G = tk.Button(self)
#        self.band_23G["text"] = "2.3G"
#        self.band_23G["command"] = self.band_2300  # Jump to Band X
#        self.band_23G.configure(fg='grey',font=self.btn_font, padx=1, state='normal')
#        self.scale.place(x=XX, y=0, bordermode=OUTSIDE, height=20, width=40)

#        self.band_34G = tk.Button(self)
#        self.band_34G["text"] = "3.4G"
#        self.band_34G["command"] = self.band_3400  # Jump to Band X
#        self.band_34G.configure(fg='grey',font=self.btn_font, padx=1, state='normal')
#        self.scale.place(x=XX, y=0, bordermode=OUTSIDE, height=20, width=40)

#        self.band_57G = tk.Button(self)
#        self.band_57G["text"] = "5.7G"
#        self.band_57G["command"] = self.band_5700  # Jump to Band X
#        self.band_57G.configure(fg='grey',font=self.btn_font, padx=1, state='normal')
#        self.scale.place(x=XX, y=0, bordermode=OUTSIDE, height=20, width=40)

#        self.band_10G = tk.Button(self)
#        self.band_10G["text"] = "10G"
#        self.band_10G["command"] = self.band_10g  # Jump to Band X
#        self.band_10G.configure(fg='grey',font=self.btn_font, padx=3, state='normal')
#        self.scale.place(x=XX, y=0, bordermode=OUTSIDE, height=20, width=40)

        self.band_HFM = tk.Button(self)
        self.band_HFM["text"] = " HF "
        self.band_HFM["command"] = self.band_HF  # Jump to Band X
        self.band_HFM.configure(fg='black',font=self.btn_font, padx=2, state='normal')
        self.band_HFM.place(x=135, y=0, bordermode=OUTSIDE, height=20, width=36)

        self.band_50M = tk.Button(self)
        self.band_50M["text"] = " 50 "
        self.band_50M["command"] = self.band_50  # Jump to Band X
        self.band_50M.configure(fg='black',font=self.btn_font, padx=2, state='normal')
        self.band_50M.place(x=171, y=0, bordermode=OUTSIDE, height=20, width=36)

        self.band_144M = tk.Button(self)
        self.band_144M["text"] = "144"
        self.band_144M["command"] = self.band_144  # Jump to Band X
        self.band_144M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_144M.place(x=208, y=0, bordermode=OUTSIDE, height=20, width=36)

        self.band_222M = tk.Button(self)
        self.band_222M["text"] = "222"
        self.band_222M["command"] = self.band_222  # Jump to Band X
        self.band_222M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_222M.place(x=245, y=0, bordermode=OUTSIDE, height=20, width=36) 

        self.band_432M = tk.Button(self)
        self.band_432M["text"] = "432"
        self.band_432M["command"] = self.band_432  # Jump to Band X
        self.band_432M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_432M.place(x=282, y=0, bordermode=OUTSIDE, height=20, width=36) 

        self.band_902M = tk.Button(self)
        self.band_902M["text"] = "902"
        self.band_902M["command"] = self.band_902  # Jump to Band X
        self.band_902M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_902M.place(x=319, y=0, bordermode=OUTSIDE, height=20, width=36) 

        self.band_1296M = tk.Button(self)
        self.band_1296M["text"] = "1296"
        self.band_1296M["command"] = self.band_1296  # Jump to Band X
        self.band_1296M.configure(fg='black',font=self.btn_font, padx=1, state='normal')
        self.band_1296M.place(x=356, y=0, bordermode=OUTSIDE, height=20, width=36) 

        #  Voltage, Current and Temp
        self.hv = tk.Label(self, text='28V:',font=('Helvetica', 10, 'bold'))
        self.hv.configure(font=self.btn_font)
        self.hv.place(x=404, y=0, bordermode=OUTSIDE, height=20, width=30)

        self.hv_a = tk.Label(self, text=' ', font=('Helvetica', 10, 'bold'))
        self.hv_a.configure(fg='yellow', bg="black", pady = 0)  
        self.hv_a.place(x=434, y=0, bordermode=OUTSIDE, height=20, width=34)

        self.v14 = tk.Label(self, text='12V:',font=('Helvetica', 10, 'bold'))
        self.v14.configure(font=self.btn_font)
        self.v14.place(x=476, y=0, bordermode=OUTSIDE, height=20, width=30)

        self.v14_a = tk.Label(self, text=' ', font=('Helvetica', 10, 'bold'))
        self.v14_a.configure(fg='yellow', bg="black", pady = 0)          
        self.v14_a.place(x=506, y=0, bordermode=OUTSIDE, height=20, width=34)
 
        self.curr = tk.Label(self, text='Amps:',font=('Helvetica', 10, 'bold'))
        self.curr.configure(font=self.btn_font)
        self.curr.place(x=549, y=0, bordermode=OUTSIDE, height=20, width=40)
       
        self.curr_a = tk.Label(self, text=' ', font=('Helvetica', 10, 'bold'))
        self.curr_a.configure(fg='yellow', bg="black", pady = 0)          
        self.curr_a.place(x=592, y=0, bordermode=OUTSIDE, height=20, width=34)
        
        self.temperature = tk.Label(self, text='T:',font=('Helvetica', 10, 'bold'))
        self.temperature.configure(font=self.btn_font)
        self.temperature.place(x=634, y=0, bordermode=OUTSIDE, height=20, width=17)

        self.temperature_a = tk.Label(self, text=' ', font=('Helvetica', 10, 'bold'))
        self.temperature_a.configure(fg='yellow', bg="black", pady = 0)          
        self.temperature_a.place(x=651, y=0, bordermode=OUTSIDE, height=20, width=48)
        
        #This label is just a buffer to prevent the SWR value from touching the right edge of a resized window
        self.spacer3 = tk.Label(self, text=' ', font=('Helvetica', 10, 'bold'))
        self.spacer3.place(x=706, y=0, bordermode=OUTSIDE, height=20, width=10)
        
        # Fill in text label to help identify multiple meters.  Text not needed for 1 meter usage
        self.spacer4 = tk.Label(self, text='  ', font=('Helvetica', 10, 'bold'))
        self.spacer4.place(x=0, y=22, bordermode=OUTSIDE, height=20, width=10)
               
        self.meter_id_f = tk.Label(self, text='Radio: ',font=('Helvetica', 10, 'bold'),pady=0,anchor="w", relief=tk.FLAT, borderwidth=1)  #, width=21)
        self.meter_id_f.configure(font=self.btn_font)
        self.meter_id_f.place(x=10, y=22, bordermode=OUTSIDE, height=20, width=170) 
 
        self.band_f = tk.Label(self, text='Band:',font=('Helvetica', 10, 'bold'),padx = 5,pady = 0, anchor="w", width=7)
        self.band_f.configure(font=self.btn_font)
        self.band_f.place(x=180, y=22, bordermode=OUTSIDE, height=20, width=70)  
         
        self.F_Watts_f = tk.Label(self, text=' FWD:', font=('Helvetica', 12, 'bold'),anchor="e",width=5)
        self.F_Watts_f.configure(font=self.btn_font, pady = 0)          
        self.F_Watts_f.place(x=250, y=22, bordermode=OUTSIDE, height=20, width=44) 

        self.F_Watts_a = tk.Label(self, text=' ', font=('Helvetica', 12, 'bold'),anchor="e", width=7)
        self.F_Watts_a.configure(fg='yellow', bg="black", pady = 0)          
        self.F_Watts_a.place(x=294, y=22, bordermode=OUTSIDE, height=20, width=60)  

        self.F_dBm_f = tk.Label(self, text='(00.0dBm) ', font=('Helvetica', 9, 'bold'),anchor="w", pady=2, width=9)
        self.F_dBm_f.configure(fg='cyan', bg="black")
        self.F_dBm_f.place(x=354, y=22, bordermode=OUTSIDE, height=20, width=80)  
        
        self.R_Watts_f = tk.Label(self, text='REF:', font=('Helvetica', 12, 'bold'),anchor="e",width=6)
        self.R_Watts_f.configure(font=self.btn_font, pady = 0)        
        self.R_Watts_f.place(x=434, y=22, bordermode=OUTSIDE, height=20, width=40)

        self.R_Watts_a = tk.Label(self, text=' ', font=('Helvetica', 12, 'bold'),anchor="e",width=5)
        self.R_Watts_a.configure(fg='yellow', bg="black", pady = 0)        
        self.R_Watts_a.place(x=477, y=22, bordermode=OUTSIDE, height=20, width=60)

        self.R_dBm_f = tk.Label(self, text='(00.0dBm) ', font=('Helvetica', 9, 'bold'),anchor="w", pady=2, width=9)
        self.R_dBm_f.configure(fg='cyan', bg="black")
        self.R_dBm_f.place(x=537, y=22, bordermode=OUTSIDE, height=20, width=80)

        self.SWR_f = tk.Label(self, text='SWR:', font=('Helvetica', 12, 'bold'),pady=0,anchor="w",width = 5)
        self.SWR_f.configure(font=self.btn_font)
        self.SWR_f.place(x=626, y=22, bordermode=OUTSIDE, height=20, width=40)

        self.SWR_a = tk.Label(self, text=' ', font=('Helvetica', 12, 'bold'),pady=0,anchor="e",width = 3)
        self.SWR_a.configure(font=self.btn_font)
        self.SWR_a.place(x=666, y=22, bordermode=OUTSIDE, height=20, width=40)
        
        #This label is just a buffer to prevent the SWR value from touching the right edge of a resized window
        self.SWR_s = tk.Label(self, text=' ', font=('Helvetica', 12, 'bold'),pady=0,anchor="e",width = 0)
        self.SWR_s.configure(font=self.btn_font)
        self.SWR_s.place(x=706, y=22, bordermode=OUTSIDE, height=20, width=10)
                
        self.update_label() 

    # Update GUI text fields with Serial Data from power meter and maybe other places later 
    def update_label(self):
        global restart_serial    
        global own_call    
        global heartbeat_timer
        global last_freq

        if own_call == "":     # allow for running without serial port to meter connection, network still running 
            #ID = "NA"       # No ID available from any source
            ID = meter_data[0]
        elif myRig_meter_ID == meter_data[0]:     #  Assign myRig1 to ID 101.  Allow for future case to monitor multiple meters
            ID = myRig               
        elif heartbeat_timer < 200:        # updates every update cycle per .after setpoint.  Typical every 1/4 second.
            ID = own_call   # put something interesting up if network on.
            if heartbeat_timer == 199:      # set to at least 1 less than the if test above
                print(" * WSJ-TX Heartbeat_timer expiring at {}" .format(heartbeat_timer))     # post up the event for FYI
                own_call = ""
                last_freq = ""          # reset when no valid data arriving
            heartbeat_timer += 1        # increment Watchdog counter. Itis reset with every heartbeat received message in Receiver thread.
        else:
            ID = meter_data[0]
        self.meter_id_f.configure(text=' Radio: {0:11s}   Band:' .format(ID), width=21) 

        curr_band = meter_data[2]
        if curr_band == "":         # if blank then the meter is disconnected or turned off.  Instead post up WSJTX data if avaialble            
            curr_band = last_freq  
            self.band_f.configure(text='%7s Net' % curr_band, anchor="e", fg="cyan",bg="black", pady=1, width=7)  # band Value
        else:
            self.band_f.configure(text='%7s' % curr_band, anchor="e", fg="yellow",bg="black", pady=1, width=7)  # band Value

        self.F_Watts_f.configure(text=' FWD:', anchor="w", width=5)               
        self.R_Watts_f.configure(text='  REF:', anchor="w", width=5)        
        
        if curr_band != "HF":       #  HF band is a "dummy" band to show it is not active 
            if meter_data_fl[5] < 1.0:
                meter_data_fl[5] = meter_data_fl[6] = 0.0  # zero out values less then minimum detectable by meter
                meter_data[3] =  meter_data[4] = '0.0'   # also do the dBm values               
                self.F_Watts_a.configure(text='{0:4.1f}W' .format(meter_data_fl[5]), width=7)                        
            elif meter_data_fl[5] > 9999:   # limit to under 10KW (9999.1W)
                self.F_Watts_a.configure(text='*OVER* ', width=7)              
            elif meter_data_fl[5] > 99.9:
                self.F_Watts_a.configure(text='{0:4.0f}W' .format(meter_data_fl[5]), width=7)      
            else:
                self.F_Watts_a.configure(text='{0:4.1f}W' .format(meter_data_fl[5]), width=7)                        
            self.F_dBm_f.configure(text='(%6sdBm)' % meter_data[3], anchor="e")
            self.R_Watts_a.configure(text='{0:6.1f}W' .format(meter_data_fl[6]), width=6)
            self.R_dBm_f.configure(text='(%6sdBm)' % meter_data[4], anchor="e")
            self.hv_a.configure(text='%4s' % meter_data2[2], anchor="e")
            self.v14_a.configure(text='%4s' % meter_data2[3], anchor="e")
            self.curr_a.configure(text='%4s' % meter_data2[4], anchor="e") 
            self.temperature_a.configure(text='%4sF' % meter_data2[5], anchor="e")     
        else:           
            self.F_Watts_a.configure(text='   NA  ', width=7)
            self.F_dBm_f.configure(text='(     dBm)', anchor="e")  
            self.R_Watts_a.configure(text='  NA  ', width=6)
            self.R_dBm_f.configure(text='(     dBm)', anchor="e")  
            self.hv_a.configure(text='%4s' % meter_data2[2], anchor="e")
            self.v14_a.configure(text='%4s' % meter_data2[3], anchor="e")
            self.curr_a.configure(text='%4s' % meter_data2[4], anchor="e") 
            self.temperature_a.configure(text='%4sF' % meter_data2[5], anchor="e")

        swr = meter_data_fl[7]
        if swr ==  0.0:
            self.SWR_a.configure(text='NA  ',font=('Helvetica', 12, 'bold'), bg="grey94", fg="black", width=4)   # not transmitting
        else:       
            if swr > 9.9:
                self.SWR_a.configure(text='OVR', font=('Helvetica', 12, 'bold'), bg="red", fg="black", width=4)      
            elif swr > 3.0:
                self.SWR_a.configure(text='{0:4.1f}  ' .format(swr), font=('Helvetica', 12, 'bold'), bg="red", fg="black", width=4) 
            else:
                self.SWR_a.configure(text='{0:4.1f}  ' .format(swr), font=('Helvetica', 12, 'bold'), bg="light green", fg="black", width=4) 

        if cmd_flag == 1:  # Coupler cal is in progress.             
            self.scale.configure(font=self.btn_font, bg="red",)    
        if cmd_flag == 0:  # Coupler cal is finished.
            self.scale.configure(font=self.btn_font, bg="grey94")

        # check if the serial is open and update button to warn user if it is closed and not supposed to be
        if restart_serial:
            self.comm()
            self.QUIT.configure(fg='white', bg="red")             
            restart_serial = 0   

        self.meter_id_f.after(200, self.update_label)  # refresh the live data display in the GUI window
               
    # These functions are called by a button to do something with the power meter such as change cal sets for a new band

    def get_cal_table(self):
        rx = Receiver()
        print("Get the meter's calibation table")
        # Write command to change meter scale and change meter face to Watts
        rx.send_meter_cmd("252", "", True)          # Direct Cm is True to send the 2 bytes out direct.  WSJTX calls with False set.
       
    def change_band(self):
        rx = Receiver()
        print("Go to Next Band ")
        # Write command to change Band
        rx.send_meter_cmd("254","", True)

    def cpl_Fwd(self): 
        rx = Receiver()
        print("Change Fwd Port Coupling Value ")
        # Write command to change meter face to SWR
        rx.send_meter_cmd("","", True)

    def cpl_Ref(self):
        rx = Receiver()
        print("Change Ref Port Coupling Value ")
        # Write command to slow data rate output from meter
        rx.send_meter_cmd("","", True)
        
    def band_10g(self):
        rx = Receiver()
        print("Jump to 10GHz Band ")
        # Write command to jump to band 9
        rx.send_meter_cmd("194","", True)
        
    def band_5700(self):
        rx = Receiver()
        print("Jump to 5.7GHz Band ")
        # Write command to jump to band 8
        rx.send_meter_cmd("193","", True)
        
    def band_3400(self):
        rx = Receiver()
        print("Jump to 3.4GHz Band ")
        # Write command to jump to band 7
        rx.send_meter_cmd("195","", True)
        
    def band_2300(self):
        rx = Receiver()
        print("Jump to 2.3GHz Band ")
        # Write command to jump to band 6
        rx.send_meter_cmd("247","", True)
        
    def band_1296(self):
        rx = Receiver()
        print("Jump to 1296MHz Band ")
        # Write command to jump to band 5
        rx.send_meter_cmd("246","", True)
        
    def band_902(self):
        rx = Receiver()
        print("Jump to 902MHz Band ")
        # Write command to jump to band 4
        rx.send_meter_cmd("245","", True)

    def band_432(self):
        rx = Receiver()
        print("Jump to 432MHz Band ")
        # Write command to jump to band 3
        rx.send_meter_cmd("244","", True)

    def band_222(self):
        rx = Receiver()
        print("Jump to 222MHz Band ")
        # Write command to jump to band 2
        rx.send_meter_cmd("243","", True)

    def band_144(self):
        rx = Receiver()
        print("Jump to 144MHz Band ")
        # Write command to jump to band 1
        rx.send_meter_cmd("242","", True)
    
    def band_50(self):
        rx = Receiver()
        print("Jump to 50MHz Band ")
        # Write command to jump to band 0
        rx.send_meter_cmd("241","", True)
        
    def band_HF(self):
        rx = Receiver()
        print("Jump to HF Band")
        # Write command to speed up data rate output from meter
        rx.send_meter_cmd("240","", True)

    def comm(self):         # toggle if on or off, do noting if neither (started up with out a serial port for example)
        global comms
        if comms == True:
            # Closing comms   - do not call this if they are already off!
            comms = False
            self.QUIT.configure(text = format("Off"))
            self.QUIT.configure(fg='black', bg="light grey")
            #self.receiver.stop()
            #self.receiver.join()
            #self.receiver = None            
            self.serial_rx.stop()
            self.serial_rx.join()
            self.serial_rx = None
            #  Zero Out Serial Data
            for i in range(len(meter_data)):
                meter_data[i] = ""
                meter_data_fl[i] = 0
            print(" Serial thread stopped ")
        elif comms == False:
            comms = True
            self.QUIT.configure(text = format("On"))
            self.QUIT.configure(fg="black", bg="green")
            #self.receiver = Receiver()
            #self.receiver.start()
            self.serial_rx = Serial_RX()
            self.serial_rx.start() 
            print(" Serial thread started ")
        else:
            pass     

    def NewFile(self):
        print("New File!")

    def OpenFile(self):
        name = askopenfilename()
        print(name)

    def About(self):
        print("RF Wattmeter Remote\nby K7MDL\nV2.3 November 2020")
        abt = tk.Tk()      
        #   Later improve to save config file and remember the last position 
        screen_width = abt.winfo_screenwidth()
        screen_height = abt.winfo_screenheight()                
        w = 700   # width of our app window
        h = 200   # height of our app window
        x = screen_width/3
        y = screen_height/4
        print('Window size and placement is %dx%d+%d+%d' % (w, h, x, y))
        abt.title("Remote RF Wattmeter Desktop Companion")
        abt.geometry('%dx%d+%d+%d' % (w, h, x, y))
        self.abt_label = tk.Label(abt, text="November 2020\n\nby K7MDL\n\nhttps://github.com/K7MDL2/RF-Power-Meter-V1\n\n\nIncludes portions of work from https://github.com/itead/ITEADLIB_Arduino_Nextion and\n  https://github.com/bmo/py-wsjtx ",font=('Helvetica', 10, 'bold'), bg="grey94", fg="black")
        self.abt_label.place(x=60, y=0)

    def mainloop(self, *args):
        # Overriding mainloop so that we can do cleanup of our threads
        # *If* any arguments were provided, we would pass them on to Tk.frame
        super().mainloop(*args)

        # When main loop finishes, shutdown Serial_RX and/or UDP receiver if necessary
        if self.serial_rx:
            self.serial_rx.stop()
        if self.receiver:
            self.receiver.stop()

    def start_cfg(self):
        cfg = Cfg_Mtr()
        print(" ---->  Started Config Window")


class Cfg_Mtr(tk.Frame):      
#    def __init__(self):
        # Call superclass constructor
#        super().__init__()   
    def __init__(self, master=None): 
        # Call superclass constructor
        super().__init__(master) 
        self.Hi_Flag = 0
        self.Lo_Flag = 0
        self.Old_Band = 0
        self.Cfg_Window()
        self.master.protocol("WM_DELETE_WINDOW", self.exit_protocol) 

    def exit_protocol(self):
        # Will be called when the main window is closed
        self.master.destroy()  # Destroy root window
        self.master.quit()  # Exiting the main loop
        
    def NexProgram(self):
        rx = Receiver()
        print("Switch Nextion to Program Connection")
        rx.send_meter_cmd("96","", True)
        
    def NexOperate(self):
        rx = Receiver()
        print("Switch Nextion to Operate Connection")
        rx.send_meter_cmd("95","", True)

    def Cal_Dump(self):
        rx = Receiver()
        print("Dump Cal Table")
        rx.send_meter_cmd("252","", True)

    def Cal_Temp(self):
        rx = Receiver()
        print("Cal Temperature")
        rx.send_meter_cmd("84",self.Temp.get(), True)

    def Cal_HVDC(self):
        rx = Receiver()
        print("Cal HV DC")
        rx.send_meter_cmd("88",self.HVDC.get(), True)

    def Cal_V14(self):
        rx = Receiver()
        print("Cal 14VDC")
        rx.send_meter_cmd("87",self.V14.get(), True)
  
    def Cal_Curr(self):
        rx = Receiver()
        print("Cal Current")
        rx.send_meter_cmd("86",self.Curr.get(), True)
    
    def Cal_Curr0(self):
        rx = Receiver()
        print("Cal No Load Current")
        rx.send_meter_cmd("85",self.Curr0.get(), True)

    def Cal_Hi(self):
        rx = Receiver()
        print("Measure Fwd and Ref High Power ADC Voltage at {}{}" .format(self.Pwr_Hi.get(), self.P_Units.get()))        
        if self.P_Units.get() == "dBm":
            rx.send_meter_cmd("78",self.Pwr_Hi.get(), True)
        else:
            rx.send_meter_cmd("79",self.Pwr_Hi.get(), True)
        time.sleep(2)
        self.Cal_HiV_F_Text.config(text="F:"+FwdVal_Hi+"VDC", font=('Helvetica', 12, 'bold'))   
        self.Cal_HiV_R_Text.config(text="R:"+RefVal_Hi+"VDC", font=('Helvetica', 12, 'bold'))        
        if self.Lo_Flag == 1:
            self.Cal_Fwd_btn.config(state='normal')
            self.Cal_Ref_btn.config(state='normal')
            print(RefVal_Hi+"    "+RefVal_Lo+"  "+FwdVal_Hi+"    "+FwdVal_Lo)                            
        self.Hi_Flag = 1
        
    def Cal_Lo(self):
        rx = Receiver()
        print("Measure Fwd and Ref Low Power ADC Voltage at {}{}" .format(self.Pwr_Lo.get(), self.P_Units.get()))                
        if self.P_Units.get() == "dBm":
            rx.send_meter_cmd("76",self.Pwr_Lo.get(), True)
        else:
            rx.send_meter_cmd("77",self.Pwr_Lo.get(), True)
        time.sleep(2)
        self.Cal_LoV_F_Text.config(text="F:"+FwdVal_Lo+"VDC", font=('Helvetica', 12, 'bold')) 
        self.Cal_LoV_R_Text.config(text="R:"+RefVal_Lo+"VDC", font=('Helvetica', 12, 'bold'))  
        if self.Hi_Flag == 1:
            self.Cal_Fwd_btn.config(state='normal')
            self.Cal_Ref_btn.config(state='normal')
            print(RefVal_Hi+"    "+RefVal_Lo+"  "+FwdVal_Hi+"    "+FwdVal_Lo)    
        self.Lo_Flag = 1

    def Cal_Fwd(self):    # used measured hi and lo values sent to host earlier, now calculate
        rx = Receiver()
        print("Calculate Fwd Cal using {}{} Hi and {}{} Lo".format(self.Pwr_Hi.get(), self.P_Units.get(), self.Pwr_Lo.get(), self.P_Units.get()))
        rx.send_meter_cmd("75","", True)        
    
    def Cal_Ref(self):
        rx = Receiver()
        print("Calculate Ref Cal using {}{} Hi and {}{} Lo".format(self.Pwr_Hi.get(), self.P_Units.get(), self.Pwr_Lo.get(), self.P_Units.get()))
        rx.send_meter_cmd("74","", True)    

    def Set_B_Dec_In_Mode(self):
        rx = Receiver()
        print("Applying Translation Mode {} to Band Decode Input Port".format(self.BDec_In.get()))
        rx.send_meter_cmd("65",self.BDec_In.get(), True)    
        if self.BDec_In.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Applying Custom Pattern to Band Decode Input Port using {}".format(self.CustomIn.get()))
            time.sleep(0.1)
            rx.send_meter_cmd("69",self.CustomIn.get(), True)  
        time.sleep(0.1)
        self.GetDecoderValues()  
        time.sleep(0.1)           

    def Set_B_Dec_A_Mode(self):
        rx = Receiver()
        print("Applying Translation Mode {} to Band Decode Output Port A".format(self.BDec_A.get()))
        rx.send_meter_cmd("64",self.BDec_A.get(), True)    
        if self.BDec_A.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Applying Custom Pattern to Band Decode Output Port A using {}".format(self.CustomA.get()))
            time.sleep(0.1)
            rx.send_meter_cmd("68",self.CustomA.get(), True)
        time.sleep(0.1)
        self.GetDecoderValues()  
        time.sleep(0.1)           

    def Set_B_Dec_B_Mode(self):
        rx = Receiver()
        print("Applying Translation Mode {} to Band Decode Output Port B".format(self.BDec_B.get()))
        rx.send_meter_cmd("63",self.BDec_B.get(), True)    
        if self.BDec_B.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Applying Custom Pattern to Band Decode Output Port B using {}".format(self.CustomB.get()))
            time.sleep(0.1)
            rx.send_meter_cmd("67",self.CustomB.get(), True)
        time.sleep(0.1)
        self.GetDecoderValues()              
        time.sleep(0.1)

    def Set_B_Dec_C_Mode(self):
        rx = Receiver()
        print("Applying Translation Mode {} to Band Decode Output Port C".format(self.BDec_C.get()))
        rx.send_meter_cmd("62",self.BDec_C.get(), True)    
        if self.BDec_C.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Applying Custom Pattern to Band Decode Output Port C using {}".format(self.CustomC.get()))
            time.sleep(0.1)
            rx.send_meter_cmd("66",self.CustomC.get(), True)
        time.sleep(0.1)  
        self.GetDecoderValues()
        time.sleep(0.1)

    def Dis_OTRSP_Band_Change(self):   # Commit to EEPROM
        rx = Receiver()
        if self.Dis_OTRPS_Ch.get() == 1:   # If custom mode then get the entered pattern and store it
            print("DISABLE Band Change by OTRSP AUX1 Command")  
            rx.send_meter_cmd("61","1", True)  
        else:
            print("ENABLE Band Change by OTRSP AUX1 Command")  
            rx.send_meter_cmd("61","0", True) 
        time.sleep(0.1)
        self.GetDecoderValues()
        time.sleep(0.1)

    def PTT_Input_Polarity(self):   # Commit to EEPROM
        rx = Receiver()
        if self.PTT_IN_pol.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set PTT Input to Active HI mode")  
            rx.send_meter_cmd("59","1", True)  
        else:
            print("Set PTT Input to Active LOW mode")  
            rx.send_meter_cmd("59","0", True) 
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)
    
    def PTT_Ouput_Polarity(self):   # Commit to EEPROM
        rx = Receiver()
        if self.PTT_OUT_pol.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set PTT Output to Active HI mode")  
            rx.send_meter_cmd("58","1", True)  
        else:
            print("Set PTT Output to Active LOW mode")  
            rx.send_meter_cmd("58","0", True) 
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)
    
    def CW_Key_Polarity(self):   # Commit to EEPROM
        rx = Receiver()
        if self.CW_KEY_OUT_pol.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set CW Key Out to Active HI mode")  
            rx.send_meter_cmd("57","1", True)  
        else:
            print("Set CW Key Out to Active LOW mode")  
            rx.send_meter_cmd("57","0", True) 
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)

    def PortA_is_PTT(self):   # Commit to EEPROM
        rx = Receiver()
        if self.PORTA_IS_PTT_var.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Set Port A PTT Mode to Active HI mode")  
            rx.send_meter_cmd("56","2", True)  
        elif self.PORTA_IS_PTT_var.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set Port A PTT Mode to Active LOW mode")  
            rx.send_meter_cmd("56","1", True)  
        else:
            print("Set Port A PTT Mode OFF")  
            rx.send_meter_cmd("56","0", True) 
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)

    def PortB_is_PTT(self):   # Commit to EEPROM
        rx = Receiver()
        if self.PORTB_IS_PTT_var.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Set Port B PTT Mode to Active HI mode")  
            rx.send_meter_cmd("55","2", True)  
        elif self.PORTB_IS_PTT_var.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set Port B PTT Mode to Active LOW mode")  
            rx.send_meter_cmd("55","1", True)  
        else:
            print("Set Port B PTT Mode to OFF")
            rx.send_meter_cmd("55","0", True)   
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)

    def PortC_is_PTT(self):   # Commit to EEPROM
        rx = Receiver()
        if self.PORTC_IS_PTT_var.get() == 2:   # If custom mode then get the entered pattern and store it
            print("Set Port C PTT Mode to Active HI mode")  
            rx.send_meter_cmd("54","2", True)  
        elif self.PORTC_IS_PTT_var.get() == 1:   # If custom mode then get the entered pattern and store it
            print("Set Port C PTT Mode to Active LOW mode")  
            rx.send_meter_cmd("54","1", True)  
        else:
            print("Set Port C PTT Mode to OFF")  
            rx.send_meter_cmd("54","0", True) 
        time.sleep(0.2)
        self.GetDecoderValues()
        time.sleep(0.2)
    
    def GetDecoderValues(self):   # Commit to EEPROM
        rx = Receiver()
        print("Retrieve Band Decoder Translation Modes and current band custom patterns")    # Used to prepopulate the Config Screen Band Decoder section to reflect current vlaues
        rx.send_meter_cmd("60","", True)    # Will result in a reply message from CPU in message type 172 which will stash values into global variable
        time.sleep(0.1)

    def Save_to_Meter(self):   # Commit to EEPROM
        rx = Receiver()
        print("Save cal table changes to Meter's EEPROM")
        rx.send_meter_cmd("195","", True)    

    def Toggle_Ser_Data(self):
        rx = Receiver()
        print("Toggle Meter Data Output Stream")
        rx.send_meter_cmd("239","1", True)    
    
    def Show_MeterID(self):
        print("Meter ID received is ", meter_data[0])

    # this page will collect Auto Cal hi and lo power levels issueing commands for each
    # the power levels can be slider, or best is to type it in and remember the last value
    #   in a cfg file entry for future use
    # also read in a cfg file and save one back out.
    #start with issueing auto cal commands. Add factory reset button.
    def Cfg_Window(self):   
        global Dis_OTRPS_Ch_flag
        global Inp_Val
        global PortA_Val
        global PortB_Val
        global PortC_Val
        global InputPort_pattern
        global PortA_pattern
        global PortB_pattern
        global PortC_pattern
        global PTT_IN_POLARITY_Val
        global PTT_OUT_POLARITY_Val
        global CW_KEY_OUT_POLARITY_val
        global PORTA_IS_PTT_Val
        global PORTB_IS_PTT_Val
        global PORTC_IS_PTT_Val                
        print("Config Screen Goes Here")
        cfg = tk.Tk()      
        #   Later improve to save config file and remember the last position 
        screen_width = cfg.winfo_screenwidth()
        screen_height = cfg.winfo_screenheight()                
        w = 1000   # width of our app window
        h = 940   # height of our app window
        x = screen_width/3
        y = screen_height/30
        print('Window size and placement is %dx%d+%d+%d' % (w, h, x, y))
        cfg.title("Remote Wattmeter Configuration Editor")
        cfg.geometry('%dx%d+%d+%d' % (w, h, x, y))
        self.Cfg_Band_label = tk.Label(cfg, text="Current Band for Edit is {}" .format(meter_data[2]),font=('Helvetica', 18, 'bold'), bg="grey94", fg="black")
        self.Cfg_Band_label.place(x=310, y=0) 
        self.NexOperate_btn = tk.Button(cfg, text='Nextion\nOperate', command = self.NexOperate,font=('Helvetica', 12, 'bold'))
        self.NexOperate_btn.place(x=90, y=50, height=60, width=100) 
        self.NexProgram_btn = tk.Button(cfg, text='Nextion\nProgram', command = self.NexProgram,font=('Helvetica', 12, 'bold'))
        self.NexProgram_btn.place(x=210, y=50, height=60, width=100)  
        self.Toggle_Ser_Data_btn = tk.Button(cfg, text='Toggle\nData', command = self.Toggle_Ser_Data, font=('Helvetica', 12, 'bold'))
        self.Toggle_Ser_Data_btn.place(x=330, y=50, height=60, width=100) 
        self.Cal_Dump_btn = tk.Button(cfg, text='Dump Cal\nTable', command = self.Cal_Dump, font=('Helvetica', 12, 'bold'))
        self.Cal_Dump_btn.place(x=450, y=50, height=60, width=100) 
        self.Show_MeterID_btn = tk.Button(cfg, text='Show \nMeter ID', command = self.Show_MeterID,font=('Helvetica', 12, 'bold'))
        self.Show_MeterID_btn.place(x=570, y=50, height=60, width=100)
        self.Reset_btn = tk.Button(cfg, text='Factory\nReset', command = self.Factory_Reset,font=('Helvetica', 12, 'bold'))
        self.Reset_btn.place(x=690, y=50, height=60, width=100)
        # Save any and all changes to EEPROM
        self.Save_to_Meter_btn = tk.Button(cfg, text='Save to\nMeter', command = self.Save_to_Meter, font=('Helvetica', 12, 'bold'), state='normal')
        self.Save_to_Meter_btn.place(x=810, y=50, height=60, width=100)
    
        self.Pwr_Hi = StringVar(cfg)
        self.Pwr_Hi.set(50)  # default entry
        self.Pwr_Lo = StringVar(cfg)
        self.Pwr_Lo.set(40)  # default entry
        self.Measure_Units = StringVar(cfg)
        self.Measure_Units.set("dBm")

        self.Cal_Text = tk.Label(cfg,text='--------Calibrate Fwd and Ref Power Measurements--------', font=('Helvetica', 12, 'bold'), justify=LEFT)
        self.Cal_Text.place(x=40, y=140, height=20)
        self.Cal1_Text = tk.Label(cfg,text='1. Choose Units then enter high and low power levels for this band\n2. Transmit a steady carrier at each power level pushing Measure for each\n3. Push Cal Fwd Power or Cal Ref Power button to calculate', font=('Helvetica', 10), justify=LEFT)
        self.Cal1_Text.place(x=40, y=160, height=60)

        self.P_Units = StringVar(cfg, "dBm")
        #self.var.set("dBm")
        self.Cal_Choose_Units_L = tk.Label(cfg, text="Choose Units: ",font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Choose_Units_L.place(x=100, y=225, height=20, width=130)
        self.Cal_Choose_Watts = tk.Radiobutton(cfg, text="Watts", variable=self.P_Units, value="Watts", command = self.Choice_Units, font=('Helvetica', 12, 'bold'))
        self.Cal_Choose_Watts.place(x=230, y=225, height=20)
        self.Cal_Choose_dBm = tk.Radiobutton(cfg, text="dBm", variable=self.P_Units, value="dBm", command = self.Choice_Units, font=('Helvetica', 12, 'bold'))
        self.Cal_Choose_dBm.place(x=320, y=225, height=20)

        self.Cal_Hi_Text = tk.Label(cfg,text='Enter Hi Pwr:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Hi_Text.place(x=30, y=270, height=20, width=130) 
        self.Cal_Hi_Entry = tk.Entry(cfg, textvariable=self.Pwr_Hi, font=('Helvetica', 12, 'bold'))
        self.Cal_Hi_Entry.place(x=150, y=270, height=20, width=60)     
        self.Cal_Hi_Entry.bind('<Return>', self.get_Hi_Watts)                        
        self.Cal_Hi_btn = tk.Button(cfg, text='Measure', command=self.Cal_Hi, font=('Helvetica', 12, 'bold'))
        self.Cal_Hi_btn.place(x=230, y=260, height=40, width=100) 
        self.Cal_HiV_F_Text = tk.Label(cfg,text="F:0.00000VDC", font=('Helvetica', 12, 'bold'))
        self.Cal_HiV_F_Text.place(x=350, y=260, height=20, width=110) 
        self.Cal_HiV_R_Text = tk.Label(cfg,text="R:0.00000VDC", font=('Helvetica', 12, 'bold'))
        self.Cal_HiV_R_Text.place(x=350, y=280, height=20, width=110) 
        
        self.Cal_Lo_Text = tk.Label(cfg,text='Enter Lo Pwr:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Lo_Text.place(x=30, y=320, height=20, width=130)
        self.Cal_Lo_Entry = tk.Entry(cfg, textvariable=self.Pwr_Lo, font=('Helvetica', 12, 'bold'))
        self.Cal_Lo_Entry.place(x=150, y=320, height=20, width=60)         
        self.Cal_Lo_Entry.bind('<Return>', self.get_Lo_Watts)                
        self.Cal_Lo_btn = tk.Button(cfg, text='Measure', command=self.Cal_Lo, font=('Helvetica', 12, 'bold'))
        self.Cal_Lo_btn.place(x=230, y=310, height=40, width=100)
        self.Cal_LoV_F_Text = tk.Label(cfg,text="F:0.00000VDC", font=('Helvetica', 12, 'bold'))
        self.Cal_LoV_F_Text.place(x=350, y=310, height=20, width=110)      
        self.Cal_LoV_R_Text = tk.Label(cfg,text="R:0.00000VDC", font=('Helvetica', 12, 'bold'))
        self.Cal_LoV_R_Text.place(x=350, y=330, height=20, width=110)      

        self.Cal_Text = tk.Label(cfg,text='After measuring both high and low power for a given direction\n(Fwd or Ref) push the appropriate button to calibrate on this band\nCommit changes with Save to Meter button', font=('Helvetica', 10))  #, 'bold'))
        self.Cal_Text.place(x=40, y=360, height=60)

        self.Cal_Fwd_btn = tk.Button(cfg, text='Cal Fwd\nPower', command=self.Cal_Fwd, font=('Helvetica', 12, 'bold'), state='disabled')
        self.Cal_Fwd_btn.place(x=80, y=430, height=60, width=100) 
        self.Cal_Ref_btn = tk.Button(cfg,text='Cal Ref\nPower', command = self.Cal_Ref, font=('Helvetica', 12, 'bold'), state='disabled')
        self.Cal_Ref_btn.place(x=200, y=430, height=60, width=100)    
        
        #
        #  Cal Temp, Voltage and Current
        #
        self.Temp = StringVar(cfg)
        self.Temp.set(0)  # default entry
        self.HVDC = StringVar(cfg)
        self.HVDC.set(28.0)  # default entry
        self.V14 = StringVar(cfg)
        self.V14.set(13.6)  # default entry
        self.Curr = StringVar(cfg)
        self.Curr.set(10.0)  # default entry
        self.Curr0 = StringVar(cfg)
        self.Curr0.set(0.0)  # default entry

        self.Cal_VText = tk.Label(cfg,text='--------Calibrate Temp Volts and Current--------', font=('Helvetica', 12, 'bold'), justify=LEFT)
        self.Cal_VText.place(x=560, y=140, height=20)
        self.Cal1_VText = tk.Label(cfg,text='1. Enter your measured values\n2. For Temp a value of 0 = no scaling applied\n3. For No Load Current turn off power\n4. Push Measure to calculate', font=('Helvetica', 10), justify=LEFT)
        self.Cal1_VText.place(x=560, y=160, height=60)

        self.Cal_Temp_Text = tk.Label(cfg,text='Enter Temp :', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Temp_Text.place(x=580, y=235, height=20, width=130) 
        self.Cal_Temp_Entry = tk.Entry(cfg, textvariable=self.Temp, font=('Helvetica', 12, 'bold'))
        self.Cal_Temp_Entry.place(x=700, y=235, height=20, width=60)     
        self.Cal_Temp_Entry.bind('<Return>', self.get_Temp)                        
        self.Cal_Temp_btn = tk.Button(cfg, text='Measure', command=self.Cal_Temp, font=('Helvetica', 12, 'bold'))
        self.Cal_Temp_btn.place(x=780, y=225, height=40, width=100)         

        self.Cal_HVDC_Text = tk.Label(cfg,text='Enter HV DC:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_HVDC_Text.place(x=580, y=275, height=20, width=130)
        self.Cal_HVDC_Entry = tk.Entry(cfg, textvariable=self.HVDC, font=('Helvetica', 12, 'bold'))
        self.Cal_HVDC_Entry.place(x=700, y=275, height=20, width=60)         
        self.Cal_HVDC_Entry.bind('<Return>', self.get_HVDC)                
        self.Cal_HVDC_btn = tk.Button(cfg, text='Measure', command=self.Cal_HVDC, font=('Helvetica', 12, 'bold'))
        self.Cal_HVDC_btn.place(x=780, y=265, height=40, width=100)

        self.Cal_V14_Text = tk.Label(cfg,text='Enter 14VDC:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_V14_Text.place(x=580, y=315, height=20, width=130)
        self.Cal_V14_Entry = tk.Entry(cfg, textvariable=self.V14, font=('Helvetica', 12, 'bold'))
        self.Cal_V14_Entry.place(x=700, y=315, height=20, width=60)         
        self.Cal_V14_Entry.bind('<Return>', self.get_V14)                
        self.Cal_V14_btn = tk.Button(cfg, text='Measure', command=self.Cal_V14, font=('Helvetica', 12, 'bold'))
        self.Cal_V14_btn.place(x=780, y=305, height=40, width=100)

        self.Cal_Curr_Text = tk.Label(cfg,text='Load Current:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Curr_Text.place(x=578, y=355, height=20, width=130)
        self.Cal_Curr_Entry = tk.Entry(cfg, textvariable=self.Curr, font=('Helvetica', 12, 'bold'))
        self.Cal_Curr_Entry.place(x=700, y=355, height=20, width=60)         
        self.Cal_Curr_Entry.bind('<Return>', self.get_Curr)                
        self.Cal_Curr_btn = tk.Button(cfg, text='Measure', command=self.Cal_Curr, font=('Helvetica', 12, 'bold'))
        self.Cal_Curr_btn.place(x=780, y=345, height=40, width=100)
        
        self.Cal_Curr0_Text = tk.Label(cfg,text='No Load Current:', font=('Helvetica', 12, 'bold'), justify='right')
        self.Cal_Curr0_Text.place(x=560, y=395, height=20, width=130)
        self.Cal_Curr0_Entry = tk.Entry(cfg, textvariable=self.Curr0, font=('Helvetica', 12, 'bold'))
        self.Cal_Curr0_Entry.place(x=700, y=395, height=20, width=60)         
        self.Cal_Curr0_Entry.bind('<Return>', self.get_Curr0)                
        self.Cal_Curr0_btn = tk.Button(cfg, text='Measure', command=self.Cal_Curr0, font=('Helvetica', 12, 'bold'))
        self.Cal_Curr0_btn.place(x=780, y=385, height=40, width=100)

        self.CalV_Text = tk.Label(cfg,text='After measuring push the Save to Meter button\n to commit changes to EEPROM', font=('Helvetica', 10))  #, 'bold'))
        self.CalV_Text.place(x=600, y=425, height=60)

        #  Band Decoder Configuration
        self.CustomIn = StringVar(cfg)
        # Preset this value by reading results of Message 60 query when this window opened        
        self.CustomA = StringVar(cfg)        
        self.CustomB = StringVar(cfg)        
        self.CustomC = StringVar(cfg)
        self.BDec_In = IntVar(cfg)
        self.BDec_A = IntVar(cfg)
        self.BDec_B = IntVar(cfg)
        self.BDec_C = IntVar(cfg)
        self.Dis_OTRPS_Ch = IntVar(cfg)
        self.PTT_IN_pol = IntVar(cfg)
        self.PTT_OUT_pol = IntVar(cfg)
        self.CW_KEY_OUT_pol = IntVar(cfg)
        self.PORTA_IS_PTT_var = IntVar(cfg)
        self.PORTB_IS_PTT_var = IntVar(cfg)
        self.PORTC_IS_PTT_var = IntVar(cfg)

        self.Update_cfg_Decoder()
        self.CustomIn.set(InputPort_pattern)  # default entry
        self.CustomA.set(PortA_pattern)  # default entry
        self.CustomB.set(PortB_pattern)  # default entry
        self.CustomC.set(PortC_pattern)  # default entry
        self.BDec_In.set(Inp_Val)
        self.BDec_A.set(PortA_Val)
        self.BDec_B.set(PortB_Val)
        self.BDec_C.set(PortC_Val)
        self.Dis_OTRPS_Ch.set(Dis_OTRPS_Ch_flag)
        self.PTT_IN_pol.set(PTT_IN_POLARITY_Val)
        self.PTT_OUT_pol.set(PTT_OUT_POLARITY_Val)
        self.CW_KEY_OUT_pol.set(CW_KEY_OUT_POLARITY_val)
        self.PORTA_IS_PTT_var.set(PORTA_IS_PTT_Val)
        self.PORTB_IS_PTT_var.set(PORTB_IS_PTT_Val)
        self.PORTC_IS_PTT_var.set(PORTC_IS_PTT_Val)

        self.B_Decode_Text = tk.Label(cfg,text='------------Band Decoder Configuration------------', font=('Helvetica', 12, 'bold'))
        self.B_Decode_Text.place(x=340, y=490, height=60)

        self.B_Decode1_Text = tk.Label(cfg,text='Choose the Translation Mode ports will use on this band. If \'Custom\' then enter the pattern in decimal form.  Press Apply when complete.', font=('Helvetica', 10))
        self.B_Decode1_Text.place(x=80, y=530, height=60)

        self.B_Dec_In_Text = tk.Label(cfg,text='Input Port Mode ', font=('Helvetica', 10, 'bold'), justify='right')
        self.B_Dec_In_Text.place(x=25, y=570, height=40, width=210)                        
        self.B_Dec_In_Radio = tk.Radiobutton(cfg, text="Transparent", variable=self.BDec_In, value=0, font=('Helvetica', 10))
        self.B_Dec_In_Radio.place(x=220, y=570, height=40)
        self.B_Dec_In_Radio = tk.Radiobutton(cfg, text="1-of-8", variable=self.BDec_In, value=1, font=('Helvetica', 10))
        self.B_Dec_In_Radio.place(x=335, y=570, height=40)
        self.B_Dec_In_Radio = tk.Radiobutton(cfg, text="Custom", variable=self.BDec_In, value=2, font=('Helvetica', 10))
        self.B_Dec_In_Radio.place(x=670, y=570, height=40)
        self.B_Dec_In_Entry = tk.Entry(cfg, textvariable=self.CustomIn, font=('Helvetica', 10))
        self.B_Dec_In_Entry.place(x=750, y=580, height=20, width=60)         
        self.B_Dec_In_Entry.bind('<Return>', self.get_CustomIn)                
        self.B_Dec_In_btn = tk.Button(cfg, text='Apply', command=self.Set_B_Dec_In_Mode, font=('Helvetica', 10, 'bold'))
        self.B_Dec_In_btn.place(x=840, y=575, height=30, width=100)

        self.B_Decode2_Text = tk.Label(cfg,text='____________________________________________________________________________________________________________________________________________________')
        self.B_Decode2_Text.place(x=80, y=603, height=13)

        self.B_Dec_A_Text = tk.Label(cfg,text='Port A Mode ', font=('Helvetica', 10, 'bold'), justify='right')
        self.B_Dec_A_Text.place(x=25, y=620, height=40, width=210)      
        self.B_Dec_A_Radio = tk.Radiobutton(cfg, text="Transparent", variable=self.BDec_A, value=0, font=('Helvetica', 10))
        self.B_Dec_A_Radio.place(x=220, y=620, height=40)
        self.B_Dec_A_Radio = tk.Radiobutton(cfg, text="1-of-8", variable=self.BDec_A, value=1, font=('Helvetica', 10))
        self.B_Dec_A_Radio.place(x=335, y=620, height=40)
        self.B_Dec_A_Radio = tk.Radiobutton(cfg, text="OTRSP Lookup", variable=self.BDec_A, value=3, font=('Helvetica', 10))
        self.B_Dec_A_Radio.place(x=410, y=620, height=40)
        self.B_Dec_A_Radio = tk.Radiobutton(cfg, text="OTRSP Direct", variable=self.BDec_A, value=4, font=('Helvetica', 10))
        self.B_Dec_A_Radio.place(x=540, y=620, height=40)
        self.B_Dec_A_Radio = tk.Radiobutton(cfg, text="Custom", variable=self.BDec_A, value=2, font=('Helvetica', 10))
        self.B_Dec_A_Radio.place(x=670, y=620, height=40)
        self.B_Dec_A_Entry = tk.Entry(cfg, textvariable=self.CustomA, font=('Helvetica', 10))
        self.B_Dec_A_Entry.place(x=750, y=630, height=20, width=60)         
        self.B_Dec_A_Entry.bind('<Return>', self.get_CustomA)                
        self.B_Dec_A_btn = tk.Button(cfg, text='Apply', command=self.Set_B_Dec_A_Mode, font=('Helvetica', 10, 'bold'))
        self.B_Dec_A_btn.place(x=840, y=625, height=30, width=100)

        self.B_Decode3_Text = tk.Label(cfg,text='____________________________________________________________________________________________________________________________________________________')
        self.B_Decode3_Text.place(x=80, y=653, height=13)

        self.B_Dec_B_Text = tk.Label(cfg,text='Port B Mode ', font=('Helvetica', 10, 'bold'), justify='right')
        self.B_Dec_B_Text.place(x=25, y=670, height=40, width=210)

        self.B_Dec_B_Radio = tk.Radiobutton(cfg, text="Transparent", variable=self.BDec_B, value=0, font=('Helvetica', 10))
        self.B_Dec_B_Radio.place(x=220, y=670, height=40)
        self.B_Dec_B_Radio = tk.Radiobutton(cfg, text="1-of-8", variable=self.BDec_B, value=1, font=('Helvetica', 10))
        self.B_Dec_B_Radio.place(x=335, y=670, height=40)
        self.B_Dec_B_Radio = tk.Radiobutton(cfg, text="OTRSP Lookup", variable=self.BDec_B, value=3, font=('Helvetica', 10))
        self.B_Dec_B_Radio.place(x=410, y=670, height=40)
        self.B_Dec_B_Radio = tk.Radiobutton(cfg, text="OTRSP Direct", variable=self.BDec_B, value=4, font=('Helvetica', 10))
        self.B_Dec_B_Radio.place(x=540, y=670, height=40)
        self.B_Dec_B_Radio = tk.Radiobutton(cfg, text="Custom", variable=self.BDec_B, value=2, font=('Helvetica', 10))
        self.B_Dec_B_Radio.place(x=670, y=670, height=40)
        self.B_Dec_B_Entry = tk.Entry(cfg, textvariable=self.CustomB, font=('Helvetica', 10))
        self.B_Dec_B_Entry.place(x=750, y=680, height=20, width=60)         
        self.B_Dec_B_Entry.bind('<Return>', self.get_CustomB)                
        self.B_Dec_B_btn = tk.Button(cfg, text='Apply', command=self.Set_B_Dec_B_Mode, font=('Helvetica', 10, 'bold'))
        self.B_Dec_B_btn.place(x=840, y=675, height=30, width=100)

        self.B_Decode4_Text = tk.Label(cfg,text='____________________________________________________________________________________________________________________________________________________')
        self.B_Decode4_Text.place(x=80, y=703, height=13)

        self.B_Dec_C_Text = tk.Label(cfg,text='Port C Mode ', font=('Helvetica', 10, 'bold'), justify='right')
        self.B_Dec_C_Text.place(x=25, y=720, height=40, width=210)
        self.B_Dec_C_Radio = tk.Radiobutton(cfg, text="Transparent", variable=self.BDec_C, value=0, font=('Helvetica', 10))
        self.B_Dec_C_Radio.place(x=220, y=720, height=40)
        self.B_Dec_C_Radio = tk.Radiobutton(cfg, text="1-of-8", variable=self.BDec_C, value=1, font=('Helvetica', 10))
        self.B_Dec_C_Radio.place(x=335, y=720, height=40)
        self.B_Dec_C_Radio = tk.Radiobutton(cfg, text="OTRSP Lookup", variable=self.BDec_C, value=3, font=('Helvetica', 10))
        self.B_Dec_C_Radio.place(x=410, y=720, height=40)
        self.B_Dec_C_Radio = tk.Radiobutton(cfg, text="OTRSP Direct", variable=self.BDec_C, value=4, font=('Helvetica', 10))
        self.B_Dec_C_Radio.place(x=540, y=720, height=40)
        self.B_Dec_C_Radio = tk.Radiobutton(cfg, text="Custom", variable=self.BDec_C, value=2, font=('Helvetica', 10))
        self.B_Dec_C_Radio.place(x=670, y=720, height=40)
        self.B_Dec_C_Entry = tk.Entry(cfg, textvariable=self.CustomC, font=('Helvetica', 10))
        self.B_Dec_C_Entry.place(x=750, y=730, height=20, width=60)         
        self.B_Dec_C_Entry.bind('<Return>', self.get_CustomC)                
        self.B_Dec_C_btn = tk.Button(cfg, text='Apply', command=self.Set_B_Dec_C_Mode, font=('Helvetica', 10, 'bold'))
        self.B_Dec_C_btn.place(x=840, y=725, height=30, width=100)
        self.B_Dec_Disable = tk.Checkbutton(cfg, text='Disable Band Change on OTRSP Commands', variable=self.Dis_OTRPS_Ch, onvalue=1, command=self.Dis_OTRSP_Band_Change, font=('Helvetica', 10))
        self.B_Dec_Disable.place(x=80, y=760, height=40)

        self.B_Decode4_Text = tk.Label(cfg,text='____________________________________________________________________________________________________________________________________________________')
        self.B_Decode4_Text.place(x=80, y=790, height=13)

        self.B_Dec_PTT_OPTS = tk.Label(cfg,text='PTT Options ', font=('Helvetica', 10, 'bold'), justify='right')
        self.B_Dec_PTT_OPTS.place(x=25, y=810, height=20, width=210)
        self.B_Dec_PTT_IN_Pol = tk.Checkbutton(cfg, text='PTT Input Active High ', variable=self.PTT_IN_pol, onvalue=1, command=self.PTT_Input_Polarity, font=('Helvetica', 10))
        self.B_Dec_PTT_IN_Pol.place(x=220, y=810, height=20)
        self.B_Dec_PTT_OUT_Pol = tk.Checkbutton(cfg, text='PTT Output Active High', variable=self.PTT_OUT_pol, onvalue=1, command=self.PTT_Ouput_Polarity, font=('Helvetica', 10))
        self.B_Dec_PTT_OUT_Pol.place(x=400, y=810, height=20)
        self.CW_KEY_Pol = tk.Checkbutton(cfg, text="CW Key Polarity High", variable=self.CW_KEY_OUT_pol, onvalue=1, command=self.CW_Key_Polarity,font=('Helvetica', 10))
        self.CW_KEY_Pol.place(x=580, y=810, height=20)

        self.PortA_is_PTT_HI = tk.Radiobutton(cfg, text="Port A Follows PTT (Active High) ----- ", variable=self.PORTA_IS_PTT_var, value=2, command=self.PortA_is_PTT, font=('Helvetica', 10))
        self.PortA_is_PTT_HI.place(x=220, y=840, height=20)
        self.PortA_is_PTT_LO = tk.Radiobutton(cfg, text="Port A Follows PTT (Active Low) -----", variable=self.PORTA_IS_PTT_var, value=1, command=self.PortA_is_PTT, font=('Helvetica', 10))
        self.PortA_is_PTT_LO.place(x=460, y=840, height=20)
        self.PortA_is_PTT_OFF = tk.Radiobutton(cfg, text="Port A Follows PTT OFF", variable=self.PORTA_IS_PTT_var, value=0, command=self.PortA_is_PTT, font=('Helvetica', 10))
        self.PortA_is_PTT_OFF.place(x=700, y=840, height=20)
        
        self.PortB_is_PTT_HI = tk.Radiobutton(cfg, text="Port B Follows PTT (Active High) ----- ", variable=self.PORTB_IS_PTT_var, value=2, command=self.PortB_is_PTT, font=('Helvetica', 10))
        self.PortB_is_PTT_HI.place(x=220, y=870, height=20)
        self.PortB_is_PTT_LO = tk.Radiobutton(cfg, text="Port B Follows PTT (Active Low) ----- ", variable=self.PORTB_IS_PTT_var, value=1, command=self.PortB_is_PTT, font=('Helvetica', 10))
        self.PortB_is_PTT_LO.place(x=460, y=870, height=20)
        self.PortB_is_PTT_OFF = tk.Radiobutton(cfg, text="Port B Follows PTT OFF", variable=self.PORTB_IS_PTT_var, value=0, command=self.PortB_is_PTT, font=('Helvetica', 10))
        self.PortB_is_PTT_OFF.place(x=700, y=870, height=20)

        self.PortC_is_PTT_HI = tk.Radiobutton(cfg, text="Port C Follows PTT (Active High) ----- ", variable=self.PORTC_IS_PTT_var, value=2, command=self.PortC_is_PTT, font=('Helvetica', 10))
        self.PortC_is_PTT_HI.place(x=220, y=900, height=20)
        self.PortC_is_PTT_LO = tk.Radiobutton(cfg, text="Port C Follows PTT (Active Low) ----- ", variable=self.PORTC_IS_PTT_var, value=1, command=self.PortC_is_PTT, font=('Helvetica', 10))
        self.PortC_is_PTT_LO.place(x=460, y=900, height=20)
        self.PortC_is_PTT_OFF = tk.Radiobutton(cfg, text="Port C Follows PTT OFF", variable=self.PORTC_IS_PTT_var, value=0, command=self.PortC_is_PTT, font=('Helvetica', 10))
        self.PortC_is_PTT_OFF.place(x=700, y=900, height=20)
        
        self.update_cfg_win()

    def update_cfg_win(self):
        self.Cfg_Band_label.config(text="Current Band for Edit is {}" .format(meter_data[2]),font=('Helvetica', 18, 'bold'), bg="grey94", fg="black")
        if meter_data[2] != self.Old_Band:
            self.Update_cfg_Decoder()
        self.Old_Band = meter_data[2]
        self.Cfg_Band_label.after(500, self.update_cfg_win)   

    def Update_cfg_Decoder(self):  
        self.GetDecoderValues()
        time.sleep(0.1)
        self.CustomIn.set(InputPort_pattern)
        self.CustomA.set(PortA_pattern)
        self.CustomB.set(PortB_pattern)
        self.CustomC.set(PortC_pattern)
        self.BDec_In.set(Inp_Val)
        self.BDec_A.set(PortA_Val)
        self.BDec_B.set(PortB_Val)
        self.BDec_C.set(PortC_Val)
        self.Dis_OTRPS_Ch.set(Dis_OTRPS_Ch_flag)
      
    def Choice_Units(self):
        #print(self.Measure_Units.get())
        print(self.P_Units.get())

    def get_Hi_Watts(self, event):
        #global FwdVal_Hi
        temp_val = self.Cal_Hi_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.Pwr_Hi.set(temp_val)             
        print(self.Pwr_Hi.get())

    def get_Lo_Watts(self, event):
        temp_val = self.Cal_Lo_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.Pwr_Lo.set(temp_val)
        print(self.Pwr_Lo.get())

    def get_Temp(self, event):
        temp_val = self.Cal_Temp_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.Temp.set(temp_val)             
        print(self.Temp.get())

    def get_HVDC(self, event):
        temp_val = self.Cal_HVDC_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.HVDC.set(temp_val)             
        print(self.HVDC.get())

    def get_V14(self, event):
        temp_val = self.Cal_V14_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.V14.set(temp_val)             
        print(self.V14.get())

    def get_Curr(self, event):
        temp_val = self.Cal_Curr_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.Curr.set(temp_val)             
        print(self.Curr.get())
        
    def get_Curr0(self, event):
        temp_val = self.Cal_Curr0_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.Curr0.set(temp_val)             
        print(self.Curr0.get())

    def get_CustomIn(self, event):
        temp_val = self.CustomIn_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.CustomIn.set(temp_val)             
        print(self.CustomIn.get())
        self.Update_cfg_Decoder()

    def get_CustomA(self, event):
        temp_val = self.CustomA_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.CustomA.set(temp_val)             
        print(self.CustomA.get())
        self.Update_cfg_Decoder()        

    def get_CustomB(self, event):
        temp_val = self.CustomB_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.CustomB.set(temp_val)             
        print(self.CustomB.get())
        self.Update_cfg_Decoder()        

    def get_CustomC(self, event):
        temp_val = self.CustomC_Entry.get()
        if temp_val == "":
            temp_val = 1
        self.CustomC.set(temp_val)             
        print(self.CustomC.get())
        self.Update_cfg_Decoder()

    def Factory_Reset(self):      
        rx = Receiver()
        print("Sending part 1 of 2 commands for Factory Reset")
        rx.send_meter_cmd("193","", True)
        while (cmd_flag == 0):            
            time.sleep(0.1)        
        rx.send_meter_cmd("194","", True)
        while (cmd_flag == 1):
            time.sleep(0.1)                    
        self.Reset_btn.configure(font=('Helvetica', 12, 'bold'), bg="grey94")                                

def main(): 
    global hide_titlebar 
    root = tk.Tk()
    # Place window in the upper right corner of the desktop display for now.  
    #   Later improve to save config file and remember the last position 
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    print('Screen Width and Height is ', screen_width, screen_height)
    # calculate position x and y coordinates
    w = 708   # width of our app window
    h = 44   # height of our app window
    x = screen_width - (w+10)
    y = 2
    print('Window size and placement is %dx%d+%d+%d' % (w, h, x, y))
    root.geometry('%dx%d+%d+%d' % (w, h, x, y))
    root.update_idletasks()
    app = App() # frame within the Menu type window
    menu = Menu(root)  # Menu type top level window
    root.config(menu=menu)
    app.master.title(myTitle)           # title can be edited in string constant at top of this file
    if (hide_titlebar) == TRUE:
        root.overrideredirect(1)
    filemenu = Menu(menu)
    menu.add_cascade(label="File", menu=filemenu)
    filemenu.add_command(label="New", command=(app.NewFile))
    filemenu.add_command(label="Open...", command=app.OpenFile)
    filemenu.add_separator()
    filemenu.add_command(label="Exit", command=app.quit)
    configmenu = Menu(menu)
    menu.add_cascade(label="Configuration", menu=configmenu)
    configmenu.add_command(label="Edit Config...", command=app.start_cfg)   
    helpmenu = Menu(app)
    menu.add_cascade(label="Help", menu=helpmenu)
    helpmenu.add_command(label="About...", command=app.About)   
    app.comm()           # calling this here (with comms=false) will toggle serial comms to start up and run and comms will be = True
    app.receiver = Receiver()
    app.receiver.start()      # start the receiver thread now
    app.mainloop()      # start the GUI

if __name__ == '__main__':
    port_name = ""
    
    #Program Startup happens here before the main app window is opened

    # Several support functions
    def validate_provided_port_name(port_name):
        #  List available ports for info
        print("Scanning for USB serial device matching cmd line provided port name {} (if any)" .format(port_name))    
        initial_serial_devices = set()
        result = {"state":"stable","port_id":[]}
        try:    
            ports = cports.comports()
            for port in ports:
                print("Port found: " + str(port))  #print all ports, we only want a USB one though
                if (port_name == str(port[0])) and ("USB" in port[1]): #check 'USB' string in device description
                    if str(port[0]) not in initial_serial_devices:
                        initial_serial_devices.add(str(port[0]))
                    print("Found a USB Port Match: " + str(port))
                    return 1
            else:
                print("\r\nNo valid matching USB serial port on command line (use format \'COMX\')")
                return 0
        except Exception as e:
            print("Error getting serial ports list: " + str(e))
            return 0

    # Interact with user in a popup GUI box to select a valid USB serial port if none named on the cmd line.
    def com_box():
        global comms 
        global myRig_meter_ID

        def ComPortSelect(event):              
            port_name = None
            index_list = listbox.curselection()      
            if (index_list != ()):            
                port = index_list[0]
                port_name = ask_for_input(port+1)   # +1 is to match the cmd line dialog that adjust the displayed list starting with 1, not 0
                com_port_var.set(port_name)                          
                listbox_label.configure(text='USB Port: {}' .format(com_port_var.get()))                
            else:
                com_port_var.set("")   
            print("listbox port_name value =", com_port_var.get())

        def MeterID_Select(event): 
            global myRig_meter_ID

            myRig_meter_ID_list = meterid_listbox.curselection()
            if (myRig_meter_ID_list != ()):
                myRig_meter_ID = str(myRig_meter_ID_list[0] + 100)                
                meterid_listbox.configure(selectbackground="BLUE")
                meterid_label.configure(text='Meter ID: {}' .format(myRig_meter_ID))
                print("listbox meter ID value =", myRig_meter_ID)
        
        def Select_done():
            dialog.destroy()     

        dialog = tk.Tk()
        dialog.title("RF Wattmeter Remote")         
        width = 420
        height = 150
        x = 400
        y = 100
        dialog.geometry('%dx%d+%d+%d' % (width, height, x, y))     
        com_port_var=tk.StringVar()
        com_port_var.set("")

        meterid_label = tk.Label(dialog, text='Meter ID: {}' .format(myRig_meter_ID), font=('Helvetica',12), padx=0, pady=6)
        meterid_label.place(x=180, y=0)                
        meterid_listbox = Listbox(dialog, font=16, selectmode = SINGLE, height=4, width=10, borderwidth=2)   
        m = 0   
        for m in range(100,120):
            meterid_listbox.insert(END, m) 

        m -= 100
        if (m > 4):
            meterid_listscroll = Scrollbar(dialog, orient= VERTICAL)            
            meterid_listbox.config(yscrollcommand = meterid_listscroll.set)
            meterid_listscroll.config(command=meterid_listbox.yview) 
            meterid_listscroll.place(x=276, y=55)       

        # defining a function that will get the comp port choice and print it on the screen  
        listbox_label = tk.Label(dialog, text='USB Port: {}' .format(com_port_var.get()), font=('Helvetica',12), padx=0, pady=6)
        listbox_label.place(x=20, y=0)  
        listbox = Listbox(dialog, font=16, selectmode = SINGLE, height=4, width=10, borderwidth=2) 

        ports = []
        i = 0
        for n, (port, desc, hwid) in enumerate(sorted(cports.comports()), 1):                        
            if "USB" in desc:   #  Only expecting USB serial ports for our Arduino
                i = i + 1
                ports.append(port)           
                #sys.stderr.write('--- {:2}: {:20} {!r}\n'.format(i, port, desc))
        for p in ports:
            listbox.insert(END, p)           
        if (len(ports) > 4):
            listscroll = Scrollbar(dialog, orient= VERTICAL)            
            listbox.config(yscrollcommand = listscroll.set)
            listscroll.config(command=listbox.yview) 
            listscroll.place(x=101, y=40, height=80 ) 
        
        if (len(ports) > 0):
            com_port_var.set(ports[0])                    
        listbox_label.configure(text='USB Port: {}' .format(com_port_var.get()))        
        listbox.bind("<ButtonRelease-1>", ComPortSelect) 
        meterid_listbox.bind("<ButtonRelease-1>", MeterID_Select)
        listbox.place(x=20, y=40)   #, width=80 ) 
        meterid_listbox.place(x=180, y=40 ) 
        listbox.selection_set(first=0)

        submit_btn = tk.Button(dialog, text='OK', font=16, command = Select_done) 
        submit_btn.place(x=350, y=56, height=50, width=50)       
        
        dialog.mainloop()
        port_name = com_port_var.get()
        return port_name 

    def ask_for_input(port_listbox):
        global comms
        global hide_titlebar          
        """
        Show a list of ports and ask the user for a choice. To make selection
        easier on systems with long device names, also allow the input of an
        index.
        """
        sys.stderr.write('\n--- Choose an available USB port to connect to your RF Power Meter:\n')        
        ports = []
        i = 0
        for n, (port, desc, hwid) in enumerate(sorted(cports.comports()), 1):                        
            if "USB" in desc:   #  Only expecting USB serial ports for our Arduino
                i = i + 1
                ports.append(port)           
                sys.stderr.write('--- {:2}: {:20} {!r}\n'.format(i, port, desc))
        #while True:  (for cmd line usage)
        #port = input('--- Enter port index number from list or any other key to continue without serial comms: ')       
        port = port_listbox        
        try:                
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                return None   #continue
            port_name = ports[index]
            comms = False
            return port_name
        except ValueError:
            print("  Starting with serial comms OFF ")        
            comms = None        # continue with comms off.
            port_name = None
            return None
        else:
            #port = ports[index] 
            port_name = None
            return None
        print("**** Missing condition if you got here! ****" + port)  

    # This is the actual startup code that calls the functions above as needed.  
    # Meter ID and Com port are harvested from the cmd line if provided.
    # If missing or wrong, the GUI startup screen will be shown

    if len(sys.argv) > 2:
        if (sys.argv[2] >= '100' and sys.argv[2] <'120'):
            myRig_meter_ID = sys.argv[2]    
    print("Meter ID now set to : " + myRig_meter_ID)

    port_name = None
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
        print("COM port name provided: " + sys.argv[1]) # Collect serial port COMX from command line or terminal input
        # print("\r\nArguments List: %s" % str(sys.argv))    # accept comm port via cmd line  
        if validate_provided_port_name(port_name):            
            comms = False
        else:       # no valid port match
            #port_name = ask_for_input()     # No valid COMM port match found (for cmd line usage)             
            port_name = com_box()     # No valid COMM port match found            
    else:
        #port_name = ask_for_input()      #  No COM port supplied on the command line  (for cmd line usage)
        port_name = com_box()     # No valid COMM port match found 

    if len(sys.argv) > 3:   # If there is a third arg assume it is to hide the title bar if value = 1
        if (sys.argv[3] == "HIDE" or sys.argv[3] == "hide"):
            hide_titlebar = TRUE

    print("Meter ID final value set to  : " + myRig_meter_ID)
    print("Com Port final value set to : ", port_name)
    if (port_name != ""):
        comms = False
    if (comms == False):
        print ("Port {} will be used" .format(port_name))
        print("Opening USB serial Port: " , port_name)
        ser = serial.Serial(
            port=port_name,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=0,
            rtscts=0,
            timeout=1
        )
        if ser: 
            #print("Testing port by closing")
            try:
                ser.close()             
            except:
                print(" Cannot open serial port")   

        print("** Started up Serial Port communication thread **")

    myTitle += " - " + port_name + " - Meter ID=" + myRig_meter_ID
       
main()      # start main app GUI and threads