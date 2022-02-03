#!/usr/bin/env python

from nis import match
#from tkinter import SE
import serial, time

SERIALPORT = "/dev/ttyUSB2"
BAUDRATE = 115200

#######################################################################################################################
#
#                                                  AT Command List
#
#######################################################################################################################

Init_cmd_list = ["AT\r\n", 
                 "ATI\r\n",
                 "ATV1\r\n",
                 "ATE1\r\n",
                 "AT+CMEE=2\r\n",
                 "ATI\r\n",
                 "AT+CPIN?\r\n",
                 "AT+CIMI\r\n",
                 "AT+QCCID\r\n",
                 "AT+CSQ\r\n",
                 "AT+CREG?\r\n",
                 "AT+CGREG?\r\n",
                 "AT+COPS\r\n"]

sms_init_cmd = ["AT+CPIN\r\n",
                "AT+CREG?\r\n",
                "AT+CGREG?\r\n",
                "AT+CSQ\r\n",
                "AT+CSCA?\r\n"]

send_sms_cmd = ["ATE1\r\n",
                "AT+CPIN?\r\n",
                "AT+CMGF=1\r\n",
                "AT+CSMP=17,167,0,0\r\n"]
                #"AT+CSCS=\"GSM\"\r\n",
                #"AT+CMGS=\"9942377521\"\r\n", "\rHi.\u001A"]  #format : \r<msg><ctrl+z>

internet_init = ["AT+CPIN?\r\n",
                 "AT+CEREG?\r\n",
                 "AT+CSQ?\r\n",
                 "AT+CREG?\r\n",
                 "AT+CGREG?\r\n",
                 "AT+CPIN?\r\n"
                 "AT+QICSGP=1,1,\"airtelgprs.com\","","",0\r\n",
                 "AT+QIACT=1\r\n"
                 "AT+QIACT?\r\n"]

gnss_cmd = ["AT+QGPSCFG=\"nmeasrc\",1\r\n","AT+QGPSGNMEA=\"GGA\"\r\n"]

ser              = serial.Serial(SERIALPORT, BAUDRATE)
ser.bytesize     = serial.EIGHTBITS #number of bits per bytes
ser.parity       = serial.PARITY_NONE #set parity check: no parity
ser.stopbits     = serial.STOPBITS_ONE #number of stop bits
ser.timeout      = 2              #timeout block read
ser.xonxoff      = False     #disable software flow control
ser.rtscts       = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr       = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 0     #timeout for write

print ('[QModem] Starting Up Serial Monitor')

Init_done = 0;
number = 9942377521

def Init():
    print("Init")
    WriteCommand(Init_cmd_list)
    #WriteCommand(sms_init_cmd)
    #WriteCommand(internet_init)

def Call(number):
    print("Call")

def Send_SMS(number,msg):
    
    print("Send SMS to:" + number)
    num = "AT+CMGS=\"" + str(number) + "\"" + "\r\n"
    m = "\r" + msg + "\u001A"  
    send_sms_cmd.append(num)
    send_sms_cmd.append(m)
    WriteCommand(send_sms_cmd)

def GNSS_Data():
    print("Current location :")
    WriteCommand(gnss_cmd)

def Internet():
    print("Connecting to internet....")
    WriteCommand(internet_init)

def WriteCommand(cmd_list):
    
    print("Write Command")

    for cmd in cmd_list : 
            
        ser.write(cmd.encode())
        print("write data: " + cmd)
        time.sleep(1)
        numberOfLine = 0

        while True:
        
            response = ser.readline()
            #response.decode("utf-8").strip("\r\n")
            #str(response, "utf-8")

            print(str(response, "utf-8"))   

            numberOfLine = numberOfLine + 1
            if (numberOfLine >= 5):
                break

def QT_Comm():

    if ser.isOpen():

        try:
            
            while True:
            
                print("Enter choice to perform the action : ")
                print("1. Init\n2. Call\n3. Send SMS\n4. Get GNSS data\n5. Connect to internet\n")
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output

                uinput = input()
                
                match uinput:
                        case '1':
                            print("Case 1")
                            Init()
                        case '2':
                            print("Case 2")
                            Call(number)
                        case '3':
                             print("Case 3")
                             contact = input("Enter the mobile number: ")
                             msg = input("Enter the message: ")
                             Send_SMS(number,msg)
                             #Send_SMS()

                        case '4':
                            print("Case 4") 
                            GNSS_Data()
                        case '5':
                            print("Case 5")
                            Internet()       

            ser.close()    

        except Exception as e :
            print ("[QModem] Error communicating..." + str(e))

    else:
        print ("[QModem] Cannot open serial port ")


if __name__=="__main__":
    QT_Comm()