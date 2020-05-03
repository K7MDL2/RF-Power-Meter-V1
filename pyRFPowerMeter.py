from threading import Thread
import time
import socket
import select
import serial
import serial.tools.list_ports as cports
import sys
import tkinter as tk
import tkinter.font as tkFont
import pywsjtx.extra.simple_server

PowerMeterVersionNum = "1.02"
# pyRFPowerMeter  Version 1.02  May 2 2020
# Author: M. Lewis K7MDL
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

# This app is the companion application to the Arduino based RF power meter poject by K7MDL
#   It has no usage without one or more of the Arduino meters.  This app listens to a serial port and 
#   displays the results in a small fixed size GUI window.  
#   It also sends commands to the meter such as to change calibration sets when you change frequency
#   You could emulate the same simple data format of 8 string fields and use this for your own device
#   The deafult serial rate (found at bottom of code below) is 115200 to match the default in my Arduino meter code

# This app accepts 1 command line argument --  The serial port to use.  It will prompt for one if not supplied in a text command window
# In normal usage you would specify the com port to be used on the command line in a desktop shortcut or a batch file.  
# You could also specify the "port_name" in the code at the bottom of this script

# Change these 2 lines to suit your station
myTitle = ("K7MDL Remote Power Meter " + PowerMeterVersionNum)      # Windows Title Bar Text

# edit these to match your meter ID and Rig/Location text for this meter instance
myRig = "K3 Florida"      # Rig name and location - about 10 characters max
myRig_meter_ID = "101"    # Change to match your power meter ID.  Always 3 digits, pad with leading 0 if needed on meter side
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


#UDP_IP = '224.1.1.1'       # multicast address and port alternative
#UDP_PORT = 5007
UDP_IP = "127.0.0.1"        # default local machine address
UDP_PORT = 2334             # change to match your WSJTX source of data port number. 2237 is a common WSJTX default port.  See below for more info...
# I am using 2334 with JTAlert re-broadcasting

#  This program can optionally use WSJT-X UDP Reporting broadcasts to automatically track your radio's frequency and send a command
#       to the Arduino RF Power meter to load the approriate calibration set.  10 bands are provided today.  50 is used for HF and 6M.  
#       The remingin bands are all the VHF+ bands up to 10GHz by default  
#       You can change the labels in the code for the button band names to be anything.  Can also change them in the Arduino side.
#       The name is just a label, in the Arduino side is it used as a more friendly way to say BandX and represent a correspnding set of 
#           coupling factor + any attenuators + any cal correction factor for the detectors at a given frequency for 
#           the forward and reflected ports of a particular dual direction coupler.  
#       If you change couplers be sure to set new calibratiuon factors in the Arduino side as they are frequency sensitive as well as
#           could have different coupling factors such as 20dB or 30dB on each port.  
#       Single port couplers can be uses the forward port to zero out the dBm fields when teh Forward port Watts is reported as 0W.  
#           The values will drift around on the unused port at other time and the SWR value may fluctuate.  Can edit these fields to blank if desired
#       In a pinch you coudl adjsut the incoming data values here.
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
#      This is a section of the relavent code used in this script
#           if wsjtx_id is None and (type(the_packet) == pywsjtx.HeartBeatPacket):
#               # We have an instance of WSJTX
#               print("WSJT-X is detected, id is {}".format(the_packet.wsjtx_id))
#               print("--> HeartBeatPacket Received")
#               wsjtx_id = the_packet.wsjtx_id

# Global Vars
global s            # used to get nmetwork packet data
own_call = ""       # used for meter ID in Radio field of GUI when only network is on.
last_freq = ""      # used for detecting band changes from network
# This boolean variable will save the communications (comms) status
comms = None   #  False is off.  App.comm wil then toggle to on state.
restart_serial = 0
send_meter_cmd_flag = False   # Boolean to gate Sreial thread to send cmd byte to meter.  Cmd comes from USB thread
cmd_byte = b'0'
meter_data = ["","","","","","","","","",""]
meter_data_fl  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,] # stores the same info in same psotion when possible as a float

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
        global cmd_byte

        if send_meter_cmd_flag == True:   # if True then the UDP thread has a cmd waiting to send out.       
                if ser.isOpen():
                    try:              
                        #print("---> Write Cmd : " + str(cmd_byte))
                        ser.write(cmd_byte)        
                        send_meter_cmd_flag = False     
                        cmd_byte = b'0'       
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
                            out = ser.readline().decode('ascii')
                        except serial.SerialException:
                            # There is nothing
                            return None
                        except TypeError as e:
                            restart_serial = 1      # restart serial_Rx thread to recover
                            print ("Error communicating while reading serial port: " + str(port_name))
                            print(e)
                        else:
                            self.get_power_data(out)
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
        global comms

        try:
            if s_data != '':
                tempstr =  str(s_data).split('\r')
                #print("1  DATA HERE {}" .format(tempstr))
                meter_data_tmp = tempstr[0].split(",")  # break comma separated values into a list
                #print("2 TMP raw str   = {}" .format(meter_data_tmp))
                if len(meter_data_tmp) == 8:
                    if meter_data_tmp[0] == myRig_meter_ID:        
                        meter_data = meter_data_tmp          
                        for i in range(len(meter_data)):                    
                            if isfloat(meter_data[i]):                                                                    
                                meter_data_fl[i] = float(meter_data[i])
                            else: # Not a float so zero fill the field
                                meter_data_fl[i] = 0.0
                        meter_data_fl[2] = float(meter_data[2][:-3])    # convert band label to a number.  Ideally would use a RegEx to split at teh end of the numbers
                        if meter_data_fl[5] == 0:
                            meter_data[3] = "0.0"          #  zero out the dBm values when F watts is zero
                            meter_data[4] = "0.0"
                        #print("{0:}    = {1:}" .format("3 ID Match Data ", meter_data))
                        #print("{0:} FLT= {1:}" .format("3 ID Match Data ", meter_data_fl))   hnh
                    else:  # No ID Match
                        #self.debug_meter_string("4 No ID Match   ")
                        meter_data[0] = "NA"          # no meter ID match so tell the UI  
                else:   # Not 8 Elements
                    s_data = ""
                    #self.debug_meter_string("5 Not 8 Elements")                    
                    meter_data[0] = "NA"          # no meter ID match so tell the UI  
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
        s = pywsjtx.extra.simple_server.SimpleServer(UDP_IP, UDP_PORT, timeout=2.0)
        print(" Starting network thread")

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
            if type(the_packet) == pywsjtx.StatusPacket:
                wsjtx_id = the_packet.wsjtx_id
                own_call = the_packet.de_call
                if wsjtx_id == myWSJTX_ID:
                    print("Status message received from WSJT-X ID : " + wsjtx_id)
                    freq = str(the_packet.dial_frequency)
                    freq = str(freq[:-6])                    
                    if  freq != "":                     
                            band = meter_data[2]
                            band = str(band[:-3])
                            if band != freq or freq != last_freq:
                                #print(" Meter band not matched to radio: Meter: {}  Radio: {}" .format(band, freq))                    
                                self.send_meter_cmd(int(freq), False)
                                print('{} {} {} {} {} {}' .format("Dial Drequency is : ", freq, "    Was : ", last_freq, "   Meter Band : ", band))                            
                                last_freq = freq                                
                            else:
                                print("Frequency Now " + freq)         

    def send_meter_cmd(self, cmd, direct_cmd):
        # cmd is type chr to be converted to byte
        # direct_cmd is BOOL to specify if it is a direct command such as button push for 144 
        #    vs a UDP freqwuency which can vary in a range
        # direct_cmd is True to do a direct command

        global send_meter_cmd_flag
        global cmd_byte

        # parse the frequency passed to call specific band command byte to be sent by another function
        #print("Meter Cmd Order is to " + str(cmd))
        send_meter_cmd_flag = True
        if direct_cmd == True:  
            cmd_byte = cmd           # now handle direct bands change commands
            #print(" ******> Direct cmd byte = " + str(cmd_byte))
        else:            
            band = int(cmd)           # non direct band change commands
            if band < 70:
                cmd_byte = b'240'
            elif 69 < band < 200:
                cmd_byte = b'241'
            elif 199 < band < 300:
                cmd_byte = b'242'
            elif 299 < band < 600:
                cmd_byte = b'243'
            elif 599 < band < 1000:
                cmd_byte = b'244'
            elif 999 < band < 2000: 
                cmd_byte = b'245'
            elif 1999 < band < 3000: 
                cmd_byte = b'246'
            elif 2999 < band < 4000: 
                cmd_byte = b'247'
            elif 3999 < band < 8000: 
                cmd_byte = b'248'
            elif band > 7999: 
                cmd_byte = b'249'
            else: 
                pass                    # incase we add more bands
              
            #print("###########> Freq cmd byte = " + str(cmd_byte))
            #print("Flag=" + str(send_meter_cmd_flag))


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
        self.pack()
        self.createWidgets()
        self.update()
        self.master.protocol("WM_DELETE_WINDOW", self.exit_protocol)

    def exit_protocol(self):
        # Will be called when the main window is closed
        # It should close the serial port if it has not
        # been previously closed
        global comms

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
        global restart_serial
        global own_call

        #self.configure(background='black')

        self.btn_font = tkFont.Font(family="Helvetica", size=10, weight='bold')
        
        self.QUIT = tk.Button(self,
                                    text = format("ON"),
                                    font = self.btn_font,
                                    relief = tk.RIDGE,
                                    fg = "black",
                                    padx = 4,
                                    #state='disabled',
                                    command = self.comm)     # Will call the comms procedure to toggle all comms
        self.QUIT.pack({"side": "left"})    
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
        self.band_f.pack({"side": "left"})

        self.scale = tk.Button(self)
        self.scale["text"] = "Scale"
        self.scale["command"] = self.change_scale  # Change to Watts and change scale
        self.scale.configure(font=self.btn_font)
        self.scale.pack({"side": "left"})

        self.swr = tk.Button(self)
        self.swr["text"] = "SWR"
        self.swr["command"] = self.change_to_swr  # Send cmd to change to SWR meter face procedure
        self.swr.configure(font=self.btn_font, padx = 3)
        self.swr.pack({"side": "left"})

        self.rate_p = tk.Button(self)
        self.rate_p["text"] = "Rate+"
        self.rate_p["command"] = self.faster  # Send cmd to change to slow data output rate
        self.rate_p.configure(font=self.btn_font)
        self.rate_p.pack({"side": "left"})

        self.rate_n = tk.Button(self)
        self.rate_n["text"] = "Rate-"
        self.rate_n["command"] = self.slower  # Send cmd to change to slow data output rate
        self.rate_n.configure(font=self.btn_font)
        self.rate_n.pack({"side": "left"})

        self.band_10G = tk.Button(self)
        self.band_10G["text"] = "10G"
        self.band_10G["command"] = self.band_10g  # Jump to Band X
        self.band_10G.configure(fg='grey',font=self.btn_font, padx=3, state='disabled')
        self.band_10G.pack({"side": "right"})

        self.band_57G = tk.Button(self)
        self.band_57G["text"] = "5.7G"
        self.band_57G["command"] = self.band_5700  # Jump to Band X
        self.band_57G.configure(fg='grey',font=self.btn_font, padx=1, state='disabled')
        self.band_57G.pack({"side": "right"})

        self.band_34G = tk.Button(self)
        self.band_34G["text"] = "3.4G"
        self.band_34G["command"] = self.band_3400  # Jump to Band X
        self.band_34G.configure(fg='grey',font=self.btn_font, padx=1, state='disabled')
        self.band_34G.pack({"side": "right"})

        self.band_23G = tk.Button(self)
        self.band_23G["text"] = "2.3G"
        self.band_23G["command"] = self.band_2300  # Jump to Band X
        self.band_23G.configure(fg='grey',font=self.btn_font, padx=1, state='disabled')
        self.band_23G.pack({"side": "right"})

        self.band_1296M = tk.Button(self)
        self.band_1296M["text"] = "1296"
        self.band_1296M["command"] = self.band_1296  # Jump to Band X
        self.band_1296M.configure(fg='black',font=self.btn_font, padx=1, state='normal')
        self.band_1296M.pack({"side": "right"})

        self.band_902M = tk.Button(self)
        self.band_902M["text"] = "902"
        self.band_902M["command"] = self.band_902  # Jump to Band X
        self.band_902M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_902M.pack({"side": "right"})

        self.band_432M = tk.Button(self)
        self.band_432M["text"] = "432"
        self.band_432M["command"] = self.band_432  # Jump to Band X
        self.band_432M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_432M.pack({"side": "right"})

        self.band_222M = tk.Button(self)
        self.band_222M["text"] = "222"
        self.band_222M["command"] = self.band_222  # Jump to Band X
        self.band_222M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_222M.pack({"side": "right"})

        self.band_144M = tk.Button(self)
        self.band_144M["text"] = "144"
        self.band_144M["command"] = self.band_144  # Jump to Band X
        self.band_144M.configure(fg='black',font=self.btn_font, padx=5, state='normal')
        self.band_144M.pack({"side": "right"})

        self.band_50M = tk.Button(self)
        self.band_50M["text"] = "HF-50"
        self.band_50M["command"] = self.band_50  # Jump to Band X
        self.band_50M.configure(fg='black',font=self.btn_font, padx=0, state='normal')
        self.band_50M.pack({"side": "right"})


        # Fill in text label to help identify multiple meters.  Text not needed for 1 meter usage    
        self.SWR_a = tk.Label(text='', font=('Helvetica', 12, 'bold'),pady=0,anchor="e",width = 0)
        self.SWR_a.configure(font=self.btn_font)
        self.SWR_a.pack({"side": "right"})
        
        self.SWR_a = tk.Label(text='', font=('Helvetica', 12, 'bold'),pady=0,anchor="e",width = 3)
        self.SWR_a.configure(font=self.btn_font)
        self.SWR_a.pack({"side": "right"})
        
        self.SWR_f = tk.Label(text=' SWR:', font=('Helvetica', 12, 'bold'),pady=0,anchor="w",width = 5)
        self.SWR_f.configure(font=self.btn_font)
        self.SWR_f.pack({"side": "right"})
                         
        self.R_dBm_f = tk.Label(text='', font=('Helvetica', 9, 'bold'),anchor="w", pady=2, width=9)
        self.R_dBm_f.configure(fg='cyan', bg="black")
        self.R_dBm_f.pack({"side": "right"})

        self.R_Watts_a = tk.Label(text='', font=('Helvetica', 12, 'bold'),anchor="e",width=5)
        self.R_Watts_a.configure(fg='yellow', bg="black", pady = 0)        
        self.R_Watts_a.pack({"side": "right"})

        self.R_Watts_f = tk.Label(text='  REF:', font=('Helvetica', 12, 'bold'),anchor="e",width=6)
        self.R_Watts_f.configure(font=self.btn_font, pady = 0)        
        self.R_Watts_f.pack({"side": "right"})

        self.F_dBm_f = tk.Label(text='(%sdBm)', font=('Helvetica', 9, 'bold'),anchor="w", pady=2, width=9)
        self.F_dBm_f.configure(fg='cyan', bg="black")
        self.F_dBm_f.pack({"side": "right"})  
        
        self.F_Watts_a = tk.Label(text='', font=('Helvetica', 12, 'bold'),anchor="e", width=7)
        self.F_Watts_a.configure(fg='yellow', bg="black", pady = 0)          
        self.F_Watts_a.pack({"side": "right"})

        self.F_Watts_f = tk.Label(text=' FWD:', font=('Helvetica', 12, 'bold'),anchor="e",width=5)
        self.F_Watts_f.configure(font=self.btn_font, pady = 0)          
        self.F_Watts_f.pack({"side": "right"})
 
        self.band_f = tk.Label(text='Band:',font=('Helvetica', 10, 'bold'),padx = 5,pady = 0, anchor="w", width=7)
        self.band_f.configure(font=self.btn_font)
        self.band_f.pack({"side": "right"})    
        
        self.meter_id_f = tk.Label(text='Radio: ',font=('Helvetica', 10, 'bold'),pady=0,anchor="w", relief=tk.FLAT, borderwidth=1, width=21)
        self.meter_id_f.configure(font=self.btn_font)
        self.meter_id_f.pack({"side": "right"})
                
        self.update_label() 

        # Update GUI text fields with Serial Data from power meter and maybe other places later 
    def update_label(self):
        global restart_serial    
        global own_call    
        print(own_call)
        if myRig_meter_ID == meter_data[0]:     #  Assign myRig1 to ID 101.  Allow for future case to monitor multiple meters
            ID = myRig               
        elif own_call == "":     # allow for running without serial port to meter connection, network still running 
            ID = "NA"       # No ID available from any source
        else:
            ID = own_call   # put something interesting up if network on.
        #else:
        #    ID = meter_data[0]
        self.meter_id_f.configure(text=' Radio: {0:11s}   Band:' .format(ID), width=21) 

        self.band_f.configure(text='%7s' % meter_data[2], anchor="e", fg="white",bg="grey", pady=1, width=7)  # band Value
        
        if (meter_data_fl[5]) > 9999:   # limit to under 10KW (9999.1W)
            meter_data[5] = "*OVER* "
        self.F_Watts_f.configure(text=' FWD:', anchor="w", width=5)       
        self.F_Watts_a.configure(text='{0:7.1f}W' .format(meter_data_fl[5]), width=7)  

        self.F_dBm_f.configure(text='(%6sdBm)' % meter_data[3], anchor="e")

        self.R_Watts_f.configure(text='  REF:', anchor="w", width=5)
        self.R_Watts_a.configure(text='{0:6.1f}W' .format(meter_data_fl[6]), width=6)
        
        self.R_dBm_f.configure(text='(%6sdBm) ' % meter_data[4], anchor="e")        
        
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

        # check if the serial is open and update button to warn user if it is closed and not supposed to be
        if restart_serial:
            self.comm()
            self.QUIT.configure(fg='white', bg="red")             
            restart_serial = 0

        self.meter_id_f.after(800, self.update_label)  # refresh the live data display in the GUI window
               
    # These functions are called by a button to do something with the power nter such as change cal sets for a new band

    def change_scale(self):
        rx = Receiver()
        print("Change to Watt Meter or Change Scale ")
        # Write command to change meter scale and change meter face to Watts
        rx.send_meter_cmd(b'255', True)

    def change_band(self):
        rx = Receiver()
        print("Go to Next Band ")
        # Write command to change Band
        rx.send_meter_cmd(b'254', True)

    def change_to_swr(self):
        rx = Receiver()
        print("Change to SWR mode ")
        # Write command to change meter face to SWR
        rx.send_meter_cmd(b'253', True)

    def slower(self):
        rx = Receiver()
        print("Slow down Data output rate ")
        # Write command to slow data rate output from meter
        rx.send_meter_cmd(b'252', True)

    def faster(self):
        rx = Receiver()
        print("Speed Up data ouput rate ")
        # Write command to speed up data rate output from meter
        rx.send_meter_cmd(b'251', True)
        
    def band_10g(self):
        rx = Receiver()
        print("Jump to 10GHz Band ")
        # Write command to jump to band 9
        rx.send_meter_cmd(b'249', True)
        
    def band_5700(self):
        rx = Receiver()
        print("Jump to 5.7GHz Band ")
        # Write command to jump to band 8
        rx.send_meter_cmd(b'248', True)
        
    def band_3400(self):
        rx = Receiver()
        print("Jump to 3.4GHz Band ")
        # Write command to jump to band 7
        rx.send_meter_cmd(b'247', True)
        
    def band_2300(self):
        rx = Receiver()
        print("Jump to 2.3GHz Band ")
        # Write command to jump to band 6
        rx.send_meter_cmd(b'246', True)
        
    def band_1296(self):
        rx = Receiver()
        print("Jump to 1296MHz Band ")
        # Write command to jump to band 5
        rx.send_meter_cmd(b'245', True)
        
    def band_902(self):
        rx = Receiver()
        print("Jump to 902MHz Band ")
        # Write command to jump to band 4
        rx.send_meter_cmd(b'244', True)

    def band_432(self):
        rx = Receiver()
        print("Jump to 432MHz Band ")
        # Write command to jump to band 3
        rx.send_meter_cmd(b'243', True)

    def band_222(self):
        rx = Receiver()
        print("Jump to 222MHz Band ")
        # Write command to jump to band 2
        rx.send_meter_cmd(b'242', True)

    def band_144(self):
        rx = Receiver()
        print("Jump to 144MHz Band ")
        # Write command to jump to band 1
        rx.send_meter_cmd(b'241', True)
    
    def band_50(self):
        rx = Receiver()
        print("Jump to 50MHz Band ")
        # Write command to jump to band 0
        rx.send_meter_cmd(b'240', True)

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

    # Place window in the upper right corner of the desktop display for now.  
    #   Later improve to save config file and remember the last position 
    def place_window(self, width, height):
        # get screen width and height
        screen_width = self.winfo_screenwidth()
        # screen_height = self.winfo_screenheight()
    
        # calculate position x and y coordinates
        x = screen_width - (width+10)
        y = 2
        self.master.geometry('%dx%d+%d+%d' % (width, height, x, y))

    def mainloop(self, *args):
        # Overriding mainloop so that we can do cleanup of our threads
        # *If* any arguments were provided, we would pass them on to Tk.frame
        super().mainloop(*args)

        # When main loop finishes, shutdown Serial_RX and/or UDP receiver if necessary
        if self.serial_rx:
            self.serial_rx.stop()
        if self.receiver:
            self.receiver.stop()

def main():   
    global send_meter_cmd_flag
    send_meter_cmd_flag = False
    global cmd_byte    
    cmd_byte = b'0'
    global comms
   
    app = App()    
    app.master.title(myTitle)           # tite can be edited in string constant at top of this file
    w = 720   # width of our app window
    h = 49    # height of our app window
    app.place_window(w, h)

    app.comm()           # calling this here (with comms=false) will toggle serial comms to start up and run and comms will be = True
     
    app.receiver = Receiver()
    app.receiver.start()
    
    app.mainloop()      # start the GUI


if __name__ == '__main__':
    comms = None
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

    def ask_for_input():
        global comms            
        """
        Show a list of ports and ask the user for a choice. To make selection
        easier on systems with long device names, also allow the input of an
        index.
        """
        sys.stderr.write('\n--- Choose an available USB port to connect to yor RF Power Meter:\n')
        ports = []
        i = 0
        for n, (port, desc, hwid) in enumerate(sorted(cports.comports()), 1):                        
            if "USB" in desc:   #  Only expecting USB serial ports for our Arduino
                i = i + 1
                ports.append(port)           
                sys.stderr.write('--- {:2}: {:20} {!r}\n'.format(i, port, desc))
        while True:
            port = input('--- Enter port index number from list or any other key to continue without serial comms: ')
            try:                
                index = int(port) - 1
                if not 0 <= index < len(ports):
                    sys.stderr.write('--- Invalid index!\n')
                    continue
                port_name = ports[index]
                comms = False
                return port_name
            except ValueError:
                print("  Starting with serial comms OFF ")        
                comms = None        # continue with comms off.
                return None
            else:
                port = ports[index] 
        print("**** Missing condition if you got here! ****" + port)    
        
    #  Begin collection and validation of serial port
    port_name = ""
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
        print("COM port name provided: " + sys.argv[1]) # Collect serial port COMX from command line or terminal input
        # print("\r\nArguments List: %s" % str(sys.argv))    # accept comm port via cmd line  
        if validate_provided_port_name(port_name):            
            comms = False
        else:       # no valid port match
            port_name = ask_for_input()     # No valid COMM port match found              
    else:
        port_name = ask_for_input()      #  No COM port supplied on the command line
   
    #comms = False
    if comms == False:
        print (" Port {} will be used" .format(port_name))
        print("Opening USB serial Port: " + port_name)
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

        print("** Started up communication threads **")
        
main()      # start main app with GUI