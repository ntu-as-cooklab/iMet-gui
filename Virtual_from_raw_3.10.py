import serial
from serial.tools import list_ports

from struct import *

import time
from datetime import datetime

import csv

import sys
import os

'''from PyQt4 import QtCore, QtGui, uic'''

import crcmod
crc_aug_ccitt = crcmod.mkCrcFun(0x11021, rev = False, initCrc = 0x1D0F, xorOut = 0x0000)

#################################################################################################### 

class tee:
    def __init__(self, output_file_list = sys.stdout, output_text_list = []):
        self.output_file_list = output_file_list
        self.output_text_list = output_text_list

    def __del__(self):
        for output_file in self.output_file_list:
            if output_file != sys.stdout: output_file.close()

    def write(self, string):
        for output_file in self.output_file_list:
            output_file.write(string)
        for output_text in self.output_text_list:
            output_text.append(string)   

    def flush(self):
        for output_file in self.output_file_list: 
            output_file.flush()
            #os.fsync(output_file)

################################################################################

def open_COM_port(self, port = "",  baudrate = 1200):
    while(1):
        try:
            if port == "":
                for port_name in list(list_ports.comports()): print port_name
                port = raw_input(">>> Input Com port name: ")
            ser = serial.Serial(port, baudrate)
            print port + " opened"
            break
        except serial.SerialException:
            print "Failed to open " + port
            port = ""

    sys.stdout.flush()
    return ser

################################################################################

class Pkt:

    def __init__(self, string = "", timestr = ""):
        
        self.target_length = 2
        self.string = string
        self.PKT_ID = 0
        
        # If packet data given
        if self.string != "": self.unpack()

        # Timestamp
        if timestr != "":
            self.time = timestr
        else:
            #now = datetime.now()
            #self.time = "%s%s" % (now.strftime("%H:%M:%S."), str(now.microsecond)[:4])
            self.time = time.asctime()
    
    def unpack(self):
        if len(self.string) > 1: self.PKT_ID = ord(self.string[1])
        if 1 <= self.PKT_ID <= 4:
            pkt_target_length = [0, 14, 18, 5, 20]
            self.target_length = pkt_target_length[self.PKT_ID]
            if len(self.string) >= self.target_length: 
                if self.PKT_ID == 1:
                    self.P     = unpack("<I",self.string[4:7]+"\x00")[0]/100.0
                    self.T     = unpack("<H",self.string[7:9])[0]/100.0
                    self.U     = unpack("<H",self.string[9:11])[0]/100.0
                    self.Vbat  = unpack("<B",self.string[11])[0]/10.0
                    self.CRC   = unpack(">H",self.string[12:14])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:12])
                elif self.PKT_ID == 2:
                    self.Longitude   = unpack("<f",self.string[2:6])[0]
                    self.Latitude    = unpack("<f",self.string[6:10])[0]
                    self.Altitude    = unpack("<H",self.string[10:12])[0]-5000
                    self.nSat        = unpack("<B",self.string[12])[0]/10.0
                    self.Time_h      = unpack("<B",self.string[13])[0]/10.0
                    self.Time_m      = unpack("<B",self.string[14])[0]/10.0
                    self.Time_s      = unpack("<B",self.string[15])[0]/10.0
                    self.CRC         = unpack(">H",self.string[16:18])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:16])
                elif self.PKT_ID == 3:
                    self.N = ord(self.string[2])
                    self.XDATA = self.string[3:3+self.N].encode("hex")
                    self.CRC = unpack(">H",self.string[3+self.N:5+self.N])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:3+self.N])
                elif self.PKT_ID == 4:
                    self.P     = unpack("<I",self.string[4:7]+"\x00")[0]/100.0
                    self.T     = unpack("<H",self.string[7:9])[0]/100.0
                    self.U     = unpack("<H",self.string[9:11])[0]/100.0
                    self.Vbat  = unpack("<B",self.string[11])[0]/10.0
                    self.Tint  = unpack("<H",self.string[12:14])[0]/100.0
                    self.Tpr   = unpack("<H",self.string[14:16])[0]/100.0
                    self.Tu    = unpack("<H",self.string[16:18])[0]/100.0
                    self.CRC   = unpack(">H",self.string[18:20])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:18])
                if(self.CRC == self.CRC_check):
                    self.CRC_ok = 1
                else:
                    self.CRC_ok = 0         

####################################################################################################

class Sounding:

    def __init__(self, name = "iMet_", output_text_list = []):
        
        self.name = name + time.strftime("%Y.%m.%d") + "_" + time.strftime("%H.%M.%S")
        self.path = self.name + "\\"

        if not os.path.exists(self.path): os.makedirs(self.path)
        
        self.log_filename = self.path + self.name + ".log"
        self.raw_filename = self.path + self.name + ".raw"
        self.csv_filename = self.path + self.name + ".csv"
        
        self.log_file = open(self.log_filename, "w")
        '''global dialog_main'''
        sys.stdout = tee([sys.stdout, self.log_file], output_text_list)
        #sys.stdin = tee([sys.stdout, self.log_file])

        self.pkt_array = []

#--------------------------------------------------------------------------------------------------

    def __del__(self):
        self.log_file.close()
        
#--------------------------------------------------------------------------------------------------

    def print_pkt(self, pkt):
        print "==================== Packet start ===================="
        print "Computer time = " + repr(pkt.time)
        print "Packet string = " + pkt.string.encode("hex")
        if 1 <= pkt.PKT_ID <= 4:
            if pkt.PKT_ID == 1:
                print "[PTU packet]"
                print "PKT_ID = " + repr(pkt.PKT_ID)
                print "P = " + repr(pkt.P)
                print "T = " + repr(pkt.T)
                print "U = " + repr(pkt.U)
                print "Vbat = " + repr(pkt.Vbat)
            elif pkt.PKT_ID == 2:
                print "[GPS packet]"
                print "PKT_ID = " + repr(pkt.PKT_ID)
                print "Longitude = " + repr(pkt.Longitude)
                print "Latitude = " + repr(pkt.Latitude)
                print "Altitude = " + repr(pkt.Altitude)
                print "nSat = " + repr(pkt.nSat)
                print "Time = " + repr(pkt.Time_h) + ":" + repr(pkt.Time_m) + ":" + repr(pkt.Time_s)
            elif pkt.PKT_ID == 3:
                print "[XDATA packet]"
                print "PKT_ID = " + repr(pkt.PKT_ID)
                print "N = " + repr(pkt.N)
                print "XDATA = " + pkt.XDATA
            elif pkt.PKT_ID == 4:
                print "[PTUx packet]"
                print "PKT_ID = " + repr(pkt.PKT_ID)
                print "P = " + repr(pkt.P)
                print "T = " + repr(pkt.T)
                print "U = " + repr(pkt.U)
                print "Vbat = " + repr(pkt.Vbat)
                print "Tint = " + repr(pkt.Tint)
                print  "Tpr = " + repr(pkt.Tpr)
                print  "Tu = " + repr(pkt.Tu)
            print  "CRC = " + hex(pkt.CRC)
            print  "CRC check = " + hex(pkt.CRC_check)
            print  "CRC ok = " + repr(pkt.CRC_ok)

#--------------------------------------------------------------------------------------------------

    def recieve_iMet(self, port = ""):

        print "#####   Recieve data from iMet   #####"
    
        # Open COM port
        self.serial = open_COM_port(port)
        self.serial.timeout = 1.0

        self.raw_file = open(self.raw_filename, 'w')
        self.csv_file = open(self.csv_filename, 'w')        
        
        pkt_buffer_array = []

        n_pkt = [0]*5
        n_pkt_CRC_ok = [0]*5
        n_false = 0 

        while(1):
            try:
                # Read data from serial
                char = self.serial.read(1)
                self.raw_file.write(char)

                if len(char) == 1:

                    if ord(char) == 1: # Start new packet
                        pkt_buffer_array.append(Pkt())
                
                    for pkt in pkt_buffer_array: # For every packet in buffer
                        pkt.string = pkt.string + char

                        if len(pkt.string) == 2: # Get PKT_ID
                            pkt.PKT_ID = ord(pkt.string[1])
                            if 1 <= pkt.PKT_ID <=4:
                                pkt_target_length = [0, 14, 18, 5, 20]
                                pkt.target_length = pkt_target_length[pkt.PKT_ID]
                            else:
                                pkt_buffer_array.remove(pkt)
                                n_false += 1
                                break

                        if len(pkt.string) == 3 and pkt.PKT_ID == 3: # XDATA packet
                            pkt.N = ord(pkt.string[2])
                            pkt.target_length = pkt.N+5

                        if len(pkt.string) >= pkt.target_length: # End of packet
                            pkt.unpack()
                            self.pkt_array.append(pkt)
                            self.print_pkt(pkt)

                            self.csv_writer.writerow([pkt.time, pkt.string.encode("hex")])
                            
                            n_pkt[0] += 1
                            n_pkt[pkt.PKT_ID] += 1
                            if(pkt.CRC_ok == 1):
                                n_pkt_CRC_ok[0] += 1
                                n_pkt_CRC_ok[pkt.PKT_ID] += 1
                            print "Packets in buffer: "  +  repr(len(pkt_buffer_array))
                            print "False packet starts: "  +  repr(n_false)
                            print "Total packets: "  +  repr(n_pkt_CRC_ok[0]) + "/" + repr(n_pkt[0])
                            print "GPS packets: "  + repr(n_pkt_CRC_ok[2]) + "/" + repr(n_pkt[2])
                            print "XDATA packets: "  + repr(n_pkt_CRC_ok[3]) + "/" + repr(n_pkt[3])
                            print "PTUx packets: "  + repr(n_pkt_CRC_ok[4]) + "/" + repr(n_pkt[4])

                            sys.stdout.flush()
                            pkt_buffer_array.remove(pkt)
                        
            except KeyboardInterrupt:
                print "Reception ended\n"
                self.log_file.close()
                self.serial.close()
                self.raw_file.close()
                self.csv_file.close()

#---------------------------------------------------------------------------------------------------

    # Read packet data from raw data csv
    def read_csv_raw(self, filename):
        
        with open(filename) as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                timestr = row[0]
                for string in row[1:]:
                    string = string.replace("-", "")
                    string = string.replace(" ", "")
                    string = string.decode("hex")
                    self.pkt_array.append(Pkt(string,timestr))

#--------------------------------------------------------------------------------------------------

    # Set up a virtual iMet from packets
    def virtual_iMet_pkt(self, port = "", cyclic = 0, period = 1.0):

        print "#####   Set up virtual iMet from pkt  #####"
        
        # Open COM port
        self.serial = open_COM_port(port)
        self.serial.writeTimeout = 1.0

        self.log_file = open(self.log_filename, 'w')

        try:         
            i_pkt = 0   # Index of current packet in packet array
            offset = 0  # Current offset in packet   
            lasttime = time.time()
            while(i_pkt < len(self.pkt_array)):
                if offset < len(self.pkt_array[i_pkt].string):
                    self.serial.write(self.pkt_array[i_pkt].string[offset])
                    offset += 1
                if time.time()-lasttime >= period:
                    self.print_pkt(self.pkt_array[i_pkt])
                    i_pkt += 1
                    offset = 0
                    if cyclic == 1 and i_pkt >= len(self.pkt_array): i_pkt = 0
                    lasttime = time.time()

        except KeyboardInterrupt:
            print "Virtual iMet ended\n"
            self.log_file.close()
            self.serial.close()

#--------------------------------------------------------------------------------------------------

    # Set up a virtual iMet from raw
    def virtual_iMet_raw(self, filename, port = "", cyclic = 0, period = 1.0/1200.0):

        print "#####   Set up virtual iMet from raw   #####"
        
        # Open COM port
        self.serial = open_COM_port(port)
        self.serial.writeTimeout = 1.0

        self.log_file = open(self.log_filename, 'w')
        self.raw_file = open(filename, 'rb')

        raw = self.raw_file.read()

        while(1):
            try:
                i = 0
                lasttime = time.time()
                while(i<len(raw)):
                    if time.time()-lasttime >= period:
                        lasttime = time.time()
                        print raw[i],
                        self.serial.write(raw[i])
                        i += 1
                    
            except KeyboardInterrupt:
                print "Virtual iMet ended\n"
                self.log_file.close()
                self.serial.close()
                self.raw_file.close()
            except:
                print "Exception\n"
                self.log_file.close()
                self.serial.close()
                self.raw_file.close()
            
####################################################################################################         

def run_console_main():

    '''sounding = Sounding()
    sounding.recieve_iMet()'''
    
    '''# Start virtual iMet from csv
    virtual = Sounding("Virtual_")
    virtual.read_csv_raw("COM26rawData.csv")            
    virtual.virtual_iMet_pkt()'''

    # Start virtual iMet from raw
    virtual = Sounding("Virtual_")           
    virtual.virtual_iMet_raw("iMet_2015.01.26_11.42.02.bin")

####################################################################################################
'''
class Dialog_main(QtGui.QMainWindow, uic.loadUiType("dialog_main.ui")[0]):
    
    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        # Bind event handlers
        self.button_exit.clicked.connect(self.close)
        self.button_start_sounding.clicked.connect(self.start_sounding)  
 
    # Event handlers
    def start_sounding(self):
        self.console.append("start")
        sounding = Sounding([self.console])
        #sounding.recieve_iMet() 
        sounding.recieve_iMet("COM30") 
'''
####################################################################################################
'''
def run_dialog_main():

    # Run GUI
    app = QtGui.QApplication(sys.argv)
    dialog_main = Dialog_main(None)
    dialog_main.show()
    app.exec_()
'''
####################################################################################################

run_console_main()
#run_dialog_main() 
             

    

            
                
            


    
 

 



