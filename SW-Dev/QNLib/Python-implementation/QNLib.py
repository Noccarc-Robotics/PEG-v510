#!/usr/bin/env python

import serial, time

SERIALPORT = "/dev/ttyUSB2"
BAUDRATE = 115200

Init_cmd_list = ["AT\r\n", "ATI\r\n","ATV1\r\n","ATE1\r\n","AT+CMEE=2\r\n","ATI\r\n","AT+CPIN?\r\n","AT+CIMI\r\n","AT+QCCID\r\n","AT+CSQ\r\n","AT+CREG?\r\n","AT+CGREG?\r\n","AT+COPS\r\n"]

ser = serial.Serial(SERIALPORT, BAUDRATE)
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 0     #timeout for write

print ('[QModem] Starting Up Serial Monitor')


if ser.isOpen():

    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output

        for cmd in Init_cmd_list : 
        
            ser.write(cmd.encode())
            print("write data: " + cmd)
            time.sleep(0.5)
            numberOfLine = 0

            while True:

                response = ser.readline()
                #response.decode("utf-8").strip("\r\n")
                #str(response, "utf-8")
                print(str(response, "utf-8"))

                numberOfLine = numberOfLine + 1
                if (numberOfLine >= 3):
                    break

        ser.close()

    except Exception as e :
        print ("[QModem] Error communicating..." + str(e))

else:
    print ("[QModem] Cannot open serial port ")
