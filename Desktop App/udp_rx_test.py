import socket



IPADDR_OF_METER = '192.168.2.188'
PORTNUM_OF_METER = 8888
#UDP_IP = '239.255.0.1'       # multicast address and port alternative
#UDP_PORT = 2237
MY_UDP_IP = "127.0.0.1"        # default local machine address
WSJTX_UDP_PORT = 2334            # change to match your WSJTX source of data port number. 2237 is a common WSJTX default port.  See below for more info...
# I am using 2334 with JTAlert re-broadcasting




global s_data
global comms

buf = {}

while (1):          
    r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  #UDP
    r.bind(("", PORTNUM_OF_METER))
    r.settimeout(2.0)
    r.setblocking(1)
    #print(" listening for UDP messages")        
    try:                
        buf, sender = r.recvfrom(1024)
        #print("Received message before decode: {}" .format(buf))
        s_data = buf.decode()
        
        #try:
        #    s_data = buf.decode("utf-8")  #encoding='ascii', errors='ignore')
        #except:
        #    s_data = buf.decode("ascii", errors='ignore')
        if s_data[0] == ">":
            print("CMD received message echo: {}" .format(s_data))
        #else:
        print("received message: {}" .format(s_data))
    except socket.timeout:
        pass
        #print("Timeout on data received from UDP")
    except socket.error:
        #print("No data received from UDP")    
        pass  