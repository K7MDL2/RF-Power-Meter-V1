import socket

myRig_meter_ID = "100"
send_meter_cmd_flag = None
cmd_data = None
cmd = None
meter_sock = None

global s_data
comms = None

# addressing information of target
IPADDR_OF_METER = '192.168.2.188'
PORTNUM_OF_METER_LISTEN = 7942
PORTNUM_OF_METER_SENDTO = 7943
#UDP_IP = '239.255.0.1'       # multicast address and port alternative
#UDP_PORT = 2237
MY_UDP_IP = "127.0.0.1"        # default local machine address
WSJTX_UDP_PORT = 2237  



buf = {}
         
def UDP_Tx(self):
    global send_meter_cmd_flag
    global comms
    global cmd_data
    global cmd
    global myRig_meter_ID
    
    if (comms == None and send_meter_cmd_flag == True):  
        print("send commands")
        send_meter_cmd_flag = False  
        # Send out to ethernet via UDP also if enabled
        # initialize a socket, think of it as a cable
        # SOCK_DGRAM specifies that this is UDP
        t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # send the command
            m = "{},120,{},{},{}" .format(myRig_meter_ID,cmd,cmd_data,'\n').encode()
            t.sendto(m, (IPADDR_OF_METER, PORTNUM_OF_METER_SENDTO))
            print("TX to CPU Msg = {}" .format(m).encode) 
        except:
            print("TX to CPU FAILED Error = {}" .format(t).encode()) 
            #pass
        # close the socket
        t.close()  
           
def UDP_Rx(self):
    global s_data
    global comms
    global meter_sock
    #pd = Power_Data
    buf = {}
    
    if (comms == None):          
        #print(" listening for UDP messages")        
        try:                
            buf, sender = meter_sock.recvfrom(1024)
            #print("Received message before decode: {}" .format(buf))
            s_data = buf.decode()
            
            #try:
            #    s_data = buf.decode("utf-8")  #encoding='ascii', errors='ignore')
            #except:
            #    s_data = buf.decode("ascii", errors='ignore')
            if s_data[0] == ">":
                print("CMD received message echo: {}" .format(s_data))
            #else:
            #print("received message: {}" .format(s_data))
            #pd.get_power_data(pd, str(s_data))
            print(str(s_data))
        except UnicodeDecodeError: # catch error and ignore it
            print("Unicode decode error caught")  # will get this on CPU resets
        except socket.timeout:
            pass
            #print("Timeout on data received from UDP")
        except socket.error:
            #print("No data received from UDP")    
            pass  
        
def main():
    print("Rx Start")
    meter_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  #UDP
    meter_sock.bind((IPADDR_OF_METER, PORTNUM_OF_METER_LISTEN))
    meter_sock.settimeout(1.0)
    meter_sock.setblocking(0)

    while (1): 
        comms = True
        UDP_Rx
        print("Rx")

