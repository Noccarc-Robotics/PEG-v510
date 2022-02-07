#!/usr/bin/env python

#from nis import match
#from tkinter import SE
import serial, time

SERIALPORT = "/dev/ttyUSB3"
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

sms_init_cmd = ["AT+CPIN?\r\n",
                "AT+CREG?\r\n",
                "AT+CGREG?\r\n",
                "AT+CSCA?\r\n"]

send_sms_cmd = ["ATE1\r\n",
                "AT+CPIN?\r\n",
                "AT+CMGF=1\r\n",
                "AT+CSMP=17,167,0,0\r\n"]
                #"AT+CSCS=\"GSM\"\r\n",
                #"AT+CMGS=\"9942377521\"\r\n", "\rHi.\u001A"]  #format : \r<msg><ctrl+z>

internet_init = ["AT+CPIN?\r\n",
                 "AT+CEREG?\r\n",
                 "AT+CREG?\r\n",
                 "AT+CPIN?\r\n",
                 "AT+QIACT=1\r\n",
                 "AT+QIACT?\r\n"]

gnss_cmd = ["AT+QGPSCFG=\"nmeasrc\",1\r\n","AT+QGPSGNMEA=\"GGA\"\r\n"]
gpgga_info = "$GPGGA,"

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

#######################################################################################################################
#
#                                       Sub Routines for Data Processing
#
#######################################################################################################################

def Init():
    print("Init")
    WriteCommand(Init_cmd_list)
    #WriteCommand(sms_init_cmd)
    #WriteCommand(internet_init)

def Call(number):
    print("Call")

def Send_SMS(number,msg):

    print("Performing sms init....")
    WriteCommand(sms_init_cmd)
    print("Sending SMS to:" + number)
    num = "AT+CMGS=\"" + number + "\"" + "\r\n"
    m = "\r" + msg + "\u001A"
    send_sms_cmd.append(num)
    send_sms_cmd.append(m)
    WriteCommand(send_sms_cmd)
    print("SMS Sent..!!")

def GNSS_Data():
    print("Current location :")
    WriteCommand(gnss_cmd)

def Internet():
    print("Connecting to internet....")
    cmd = "AT+QICSGP=1,1,\"airtelgprs.com\"" + "," + "\"\"" + "," + "\"\" " + "," + str(0) + "\r\n"
    if(internet_init[4] != cmd):
        internet_init.insert(4,cmd)
    #print(internet_init)
    WriteCommand(internet_init)

def Recover():
    print("In recovery...")

def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position

def process_location(NMEA_data):
    nmea_time = NMEA_data[1]                    #extract time from GPGGA string
    nmea_latitude = NMEA_data[2]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_data[4]               #extract longitude from GPGGA string
    print("NMEA Time", nmea_time , "\n")
    lat = float(nmea_latitude)
    lat = convert_to_degrees(lat)
    longi = float(nmea_longitude)
    longi = convert_to_degrees(longi)
    print ("NMEA Latitude:", lat, NMEA_data[3],"NMEA Longitude:", longi, NMEA_data[5])


def WriteCommand(cmd_list):

    #print("Write Command")

    for cmd in cmd_list :

        ser.write(cmd.encode())
        #print("write data: " + cmd)
        time.sleep(1)
        numberOfLine = 0
        size = len(cmd_list)

        while True:

            response = ser.readline()
            #print(str(response, "utf-8"))

            if (str(response, "utf-8").find('ERROR') != -1):
                 Recover() #To be done s
                #print("Response Good")
            else:

               if(cmd == "ATV1\r\n"):
                    index = str(response, "utf-8").find('Revision')
                    if(index >= 0):
                        print("FW " + str(response, 'utf-8')[index:26])
               elif(cmd == "AT+CREG?\r\n"):
                    index = str(response, "utf-8").find("+QCCID")
                    if( index>= 0):
                        print("SIM ID: "+ str(response, 'utf-8')[index+8:29])
               elif(cmd == "AT+CGREG?\r\n"):
                    index = str(response, "utf-8").find(":")
                    if(index >=0):
                        print("Signal Strength: "+ str(response, "utf-8")[index+2:index+4] + " dB\n")
               elif(cmd == "AT+QIACT?\r\n"):
                    index = str(response, "utf-8").find(":")
                    if(index >=0):
                        print("IP Address: "+ str(response, "utf-8")[index+9:index+22])
               elif(cmd == "AT+QGPSGNMEA=\"GGA\"\r\n"):
                    index = str(response, "utf-8").find(":")
                    if(index >=0):
                        GPGGA_buffer=str(response,"utf-8")[index+2:]
                        NMEA_buff = (GPGGA_buffer.split(","))
                        process_location(NMEA_buff)

            numberOfLine = numberOfLine + 1
            if (numberOfLine >= 3):
                break

def QT_Comm():
    if ser.isOpen():

        try:

            while True:

                print("Enter choice to perform the action : ")
                print("1. Init\n2. Call\n3. Send SMS\n4. Get GNSS data\n5. Connect to internet\n6. Exit\n")
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output

                uinput = input()

                if(uinput == '1'):
                    print("Case 1")
                    Init()
                elif(uinput == '2'):
                    print("Case 2")
                    #Call(number)
                elif(uinput == '3'):
                    print("Case 3")
                    contact = input("Enter the contact no: ")
                    msg = input("Enter the msg: ")
                    Send_SMS(contact,msg)
                elif(uinput == '4'):
                    print("Case 4")
                    GNSS_Data()
                elif(uinput == '5'):
                    print("Case 5")
                    Internet()
                else:
                    print("Exit...")
                    break

            ser.close()

        except Exception as e :
            print ("[QModem] Error communicating..." + str(e))

    else:
        print ("[QModem] Cannot open serial port ")

##################################################### MAIN ############################################################
if __name__=="__main__":
    QT_Comm()