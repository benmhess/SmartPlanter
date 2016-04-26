import Adafruit_BBIO.UART as UART
import serial, sys, getopt
from select import select
import sqlite3 as sqlite
from datetime import datetime

_DEBUG = True
_TIMEOUT = 1
_EXIT = 'exit\n'
_DB_NAME = 'systems.db'

dbConnection = sqlite.connect(_DB_NAME);
dbCursor = dbConnection.cursor()

def openUart():
        """
        Opens UART1 (TX = 24, RX = 26). Returns true if successful.
        """

        UART.setup("UART1")
        ser = serial.Serial(port = "/dev/ttyO1", baudrate=9600, timeout=_TIMEOUT)

        # Reset port
        ser.close()
        ser.open()

        return ser

def ioWithArduino(port):
        inData = ""
        inData2 = ""
        valueComplete = False
        while True:
            recieved = port.read()

            #print recieved

            if recieved == '\n' and ("emp" in inData or "ater" in inData or "umd" in inData or "ight" in inData):
                valueComplete = True
                #print "done"
            elif "emp" in inData or "ater" in inData or "umd" in inData or "ight" in inData:
                #print "equals"
                inData2 += recieved
                #print recieved
                if "emp" in inData:
                        inData = "temp"
                elif "ater" in inData:
                        inData = "water"
                elif "umd" in inData:
                        inData = "humd"
                elif "ight" in inData:
                        inData = "light"
            #elif recieved != '\n' and ("emp" not in inData and "ater" not in inData and "umd" not in inData and "ight" not in inData):
            #   inData += recieved
            #elif recieved == '\n' and ("emp" in inData or "ater" in inData or "umd" in inData or "ight" in inData):
        #       valueComplete = True
        #       print "done"
            elif recieved != '\n':
                inData += recieved

            #print inData
            #print inData2

            if valueComplete:
                if ("emp" in inData or "ater" in inData or "umd" in inData or "ight" in inData) and valueComplete :
                    save(inData, inData2)
                inData = ""
                inData2 = ""
                valueComplete = False

def save(typeName, value):
    #save data here
    print "saving"
    print typeName
    print value
    dbCursor.execute('INSERT INTO systems (record_type, record_value, record_timestamp) VALUES(?,?,?)', (typeName, value, datetime.now()))
    dbConnection.commit()

# Check command line option to turn debug statements on
if len(sys.argv) > 1:
        if sys.argv[1] == '-d' and sys.argv[2] == '1':
                _DEBUG = True
        else:
                print "Usage: python serialExample.py [-d 1]"
                exit()

# Attempt to open UART1
port = openUart()
if port.isOpen():
        if _DEBUG: print "Serial port has been opened"
else:
        print "Failed to open UART1. Exiting."
        exit()

# Begin communication
ioWithArduino(port)

if _DEBUG: print "Closing serial port"
port.close()
dbConnection.close()