import serial
from serial.tools import list_ports
import BaseHTTPServer
import thread
import sys, os
import time
from PyQt4 import QtCore, QtGui, uic
from math import pi, sqrt, cos, sin, tan, asin, acos, atan
from datetime import datetime
from struct import unpack

import crcmod
crc_aug_ccitt = crcmod.mkCrcFun(0x11021, rev = False, initCrc = 0x1D0F, xorOut = 0x0000)

#Station location at NTU AS
station_lat = 25.014852
station_lon = 121.538715
station_alt = 10

accumulate_gps = ''

####################################################################################################

class HTTPServer(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):

        if self.path == '/iMet.kml':
            kml = (
               '<?xml version="1.0" encoding="UTF-8"?>\n'
               '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
               '    <Placemark>\n'
               '        <name>iMet path</name>\n'
               '        <Style>\n'
               '            <LineStyle>\n'
               '                <color>ff0000ff</color>\n'
               '                <width>5</width>\n'
               '            </LineStyle>\n'
               '        </Style>\n'
               '        <LineString id="iMetPath">\n'
               '            <altitudeMode>absolute</altitudeMode>\n'
               '            <tessellate>1</tessellate>\n'
               '            <coordinates></coordinates>\n'
               '        </LineString>\n'
               '    </Placemark>\n'
               '</kml>'
               )
        elif self.path == '/iMet-update.kml':
            global accumulate_gps
            kml = (
               '<?xml version="1.0" encoding="UTF-8"?>\n'
               '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
               '    <NetworkLinkControl>\n'
               '        <Update>\n'
               '            <targetHref>http://localhost:8000/iMet.kml</targetHref>\n'
               '            <Change>\n'
               '                <LineString targetId="iMetPath">\n'
               '                    <coordinates>\n%s</coordinates>\n'
               '                </LineString>\n'
               '            </Change>\n'
               '        </Update>\n'
               '    </NetworkLinkControl>\n'
               '</kml>'
               ) %(accumulate_gps)

        self.send_response(200)
        self.send_header("Content-type", "application/vnd.google-earth.kml+xml")
        self.send_header("Connection", "close")
        self.send_header("Content-Length", len(kml))
        self.end_headers()
        self.wfile.write(kml)

def run(server_class = BaseHTTPServer.HTTPServer, handler_class = HTTPServer):
    server_address = ('', 8000)
    httpd = server_class(server_address, handler_class)
    httpd.serve_forever()

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

def cls():
    os.system(['clear','cls'][os.name == 'nt'])

################################################################################

def view_params(lat1D, lon1D, alt1, lat2D, lon2D, alt2):
    # Input: lat of observer, lon of observer, altitude(m) of observer
    #        lat of object,   lon of object,   altitude(m) of object

    ## Haversine formula
    ## Vincenty's formulae
    ## Great-circle navigation
    ## World Geodetic System

    R = 6378100 # Radius of Earth (km)

    # Convert degrees to radians
    lat1 = lat1D * pi/180
    lat2 = lat2D * pi/180
    lon1 = lon1D * pi/180
    lon2 = lon2D * pi/180

    # Difference in latitude/longitude
    dLon = lon2 - lon1
    dLat = lat2 - lat1

    # Distance: haversine formula
    h = sin(dLat/2)*sin(dLat/2) + cos(lat1)*cos(lat2)*sin(dLon/2)* sin(dLon/2)
    if h > 1 : h = 1
    d = 2 * R * asin(sqrt(h))

    # Azimuth
    a1 = atan(sin(dLon) / (cos(lat1)*tan(lat2) - sin(lat1)*cos(dLon)))
    if dLat < 0: a1 += pi
    if a1 < 0 : a1 += 2*pi

    # Elevation angle
    lmd = atan( (alt2-alt1)/d )

    # Convert radians to degrees
    a1D = a1 * (180/pi)
    lmdD = lmd * (180/pi)

    return d/1000.0, a1D, lmdD # Distance(km), Azimuth(deg), Elevation angle(deg)

################################################################################

class Pkt:  # iMet data packet

    def __init__(self, string = "", timestr = ""):

        self.target_length = 2
        self.string = string    # Packet raw data string
        self.PKT_ID = 0

        # If packet raw data is given by string, unpack data
        if self.string != "": self.unpack()

        # Timestamp
        if timestr != "":   # If time is given, use as timestamp
            self.time = timestr
        else:               # If time is not given, use current time as timestamp
            self.time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S+08:00")

            self.P     = 0
            self.T     = 0
            self.U     = 0
            self.Vbat  = 0
            self.Latitude    = 0
            self.Longitude   = 0
            self.Altitude    = 0
            self.nSat        = 0
            self.Time_h      = 0
            self.Time_m      = 0
            self.Time_s      = 0
            self.N = 0
            self.XDATA = ""
            self.Tint  = 0
            self.Tpr   = 0
            self.Tu    = 0
            self.CRC   = 0
            self.CRC_check   = 0
            self.CRC_ok = 0
            self.Data = [0]*25

    def unpack(self):
        if len(self.string) > 1: self.PKT_ID = ord(self.string[1]) # Type of packet(PKT_ID) determined by byte at offset 1
        if 1 <= self.PKT_ID <= 4: # Type of packet(PKT_ID) must be 1~4
            pkt_target_length = [0, 14, 18, 5, 20] # Correct length (in bytes) is [1] PTU: 14 [2] GPS: 18 [3] XDATA: varies, minimun 5 [4] PTUx: 20
            self.target_length = pkt_target_length[self.PKT_ID] # If packet is of correct length, unpack packet
            if len(self.string) >= self.target_length:
                if self.PKT_ID == 1:    # PTU packet
                    self.P     = unpack("<I",self.string[4:7]+"\x00")[0]/100.0  # Pressure:         offset 4~6,     unsigned int,       little-endian
                    self.T     = unpack("<h",self.string[7:9])[0]/100.0         # Temperature:      offset 7~8,     signed short,       little-endian
                    self.U     = unpack("<H",self.string[9:11])[0]/100.0        # Humidity:         offset 9~10,    unsigned short,     little-endian
                    self.Vbat  = unpack("<B",self.string[11])[0]/10.0           # Battery voltage:  offset 11,      unsigned char,      little-endian
                    self.CRC   = unpack(">H",self.string[12:14])[0]             # CRC:              offset 12~13,   unsigned short,     big-endian
                    self.CRC_check   = crc_aug_ccitt(self.string[:12])          # Calculate CRC from offset 0~11
                elif self.PKT_ID == 2:  # GPS packet
                    self.Latitude    = unpack("<f",self.string[2:6])[0]
                    self.Longitude   = unpack("<f",self.string[6:10])[0]
                    self.Altitude    = unpack("<H",self.string[10:12])[0]-5000
                    self.nSat        = unpack("<B",self.string[12])[0]
                    self.Time_h      = unpack("<B",self.string[13])[0]
                    self.Time_m      = unpack("<B",self.string[14])[0]
                    self.Time_s      = unpack("<B",self.string[15])[0]
                    self.CRC         = unpack(">H",self.string[16:18])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:16])          # Calculate CRC from offset 0~15
                elif self.PKT_ID == 3:  # XDATA packet
                    self.N = ord(self.string[2]) # XDATA packet length determined by byte at offset 2
                    if len(self.string) >= 3+self.N:
                        self.XDATA = self.string[3:3+self.N]
                        if self.N >= 2:
                            self.Data[0] = unpack(">H",self.XDATA[0:2])[0]
                        if self.N >= 26:
                            self.Data[1] = unpack(">H",self.XDATA[2:4])[0]
                            self.Data[2] = unpack(">H",self.XDATA[4:6])[0]
                            self.Data[3] = unpack(">H",self.XDATA[6:8])[0]
                            self.Data[4] = unpack(">H",self.XDATA[8:10])[0]
                            self.Data[5] = unpack(">H",self.XDATA[10:12])[0]
                            self.Data[6] = unpack(">H",self.XDATA[12:14])[0]
                            self.Data[7] = unpack(">H",self.XDATA[14:16])[0]
                            self.Data[8] = unpack(">H",self.XDATA[16:18])[0]
                            self.Data[9] = unpack(">H",self.XDATA[18:20])[0]
                            self.Data[10] = unpack(">H",self.XDATA[20:22])[0]
                            self.Data[11] = unpack(">H",self.XDATA[22:24])[0]
                            self.Data[12] = unpack(">H",self.XDATA[24:26])[0]
                        if self.N >= 50:
                            self.Data[13] = unpack(">H",self.XDATA[26:28])[0]
                            self.Data[14] = unpack(">H",self.XDATA[28:30])[0]
                            self.Data[15] = unpack(">H",self.XDATA[30:32])[0]
                            self.Data[16] = unpack(">H",self.XDATA[32:34])[0]
                            self.Data[17] = unpack(">H",self.XDATA[34:36])[0]
                            self.Data[18] = unpack(">H",self.XDATA[36:38])[0]
                            self.Data[19] = unpack(">H",self.XDATA[38:40])[0]
                            self.Data[20] = unpack(">H",self.XDATA[40:42])[0]
                            self.Data[21] = unpack(">H",self.XDATA[42:44])[0]
                            self.Data[22] = unpack(">H",self.XDATA[44:46])[0]
                            self.Data[23] = unpack(">H",self.XDATA[46:48])[0]
                            self.Data[24] = unpack(">H",self.XDATA[48:50])[0]
                    self.CRC = unpack(">H",self.string[3+self.N:5+self.N])[0]   # Calculate CRC
                    self.CRC_check   = crc_aug_ccitt(self.string[:3+self.N])
                elif self.PKT_ID == 4:  # PTUx packet (PTU enhanced)
                    self.P     = unpack("<I",self.string[4:7]+"\x00")[0]/100.0
                    self.T     = unpack("<h",self.string[7:9])[0]/100.0
                    self.U     = unpack("<H",self.string[9:11])[0]/100.0
                    self.Vbat  = unpack("<B",self.string[11])[0]/10.0
                    self.Tint  = unpack("<h",self.string[12:14])[0]/100.0
                    self.Tpr   = unpack("<h",self.string[14:16])[0]/100.0
                    self.Tu    = unpack("<h",self.string[16:18])[0]/100.0
                    self.CRC   = unpack(">H",self.string[18:20])[0]
                    self.CRC_check   = crc_aug_ccitt(self.string[:18])          # Calculate CRC from offset 0~17
                if(self.CRC == self.CRC_check): # Check if received and calculated CRC are in agreement
                    self.CRC_ok = 1
                else:
                    self.CRC_ok = 0

    def print_pkt(self, log_file): # Prints the contents of the data packet "pkt" to the console
        log_file.write ("==================== Packet start ====================\n")
        log_file.write ("Computer time = " + repr(self.time) + "\n")
        log_file.write ("Packet string = " + self.string.encode("hex") + "\n")
        if 1 <= self.PKT_ID <= 4:
            if self.PKT_ID == 1:     # PTU packet
                log_file.write ("[PTU packet]\n")
                log_file.write ("PKT_ID = " + repr(self.PKT_ID) + "\n")
                log_file.write ("P = " + repr(self.P) + "\n")
                log_file.write ("T = " + repr(self.T) + "\n")
                log_file.write ("U = " + repr(self.U) + "\n")
                log_file.write ("Vbat = " + repr(self.Vbat) + "\n")
            elif self.PKT_ID == 2:   # GPS packet
                log_file.write ("[GPS packet]\n")
                log_file.write ("PKT_ID = " + repr(self.PKT_ID) + "\n")
                log_file.write ("Longitude = " + repr(self.Longitude) + "\n")
                log_file.write ("Latitude = " + repr(self.Latitude) + "\n")
                log_file.write ("Altitude = " + repr(self.Altitude) + "\n")
                log_file.write ("nSat = " + repr(self.nSat) + "\n")
                log_file.write ("Time = " + repr(self.Time_h) + ":" + repr(self.Time_m) + ":" + repr(self.Time_s) + "\n")
            elif self.PKT_ID == 3:   # XDATA packet
                log_file.write ("[XDATA packet]\n")
                log_file.write ("PKT_ID = " + repr(self.PKT_ID) + "\n")
                log_file.write ("N = " + repr(self.N) + "\n")
                log_file.write ("XDATA = " + self.XDATA.encode("hex") + "\n")
            elif self.PKT_ID == 4:   # PTUx packet
                log_file.write ("[PTUx packet]\n")
                log_file.write ("PKT_ID = " + repr(self.PKT_ID) + "\n")
                log_file.write ("P = " + repr(self.P) + "\n")
                log_file.write ("T = " + repr(self.T) + "\n")
                log_file.write ("U = " + repr(self.U) + "\n")
                log_file.write ("Vbat = " + repr(self.Vbat) + "\n")
                log_file.write ("Tint = " + repr(self.Tint) + "\n")
                log_file.write ("Tpr = " + repr(self.Tpr) + "\n")
                log_file.write ("Tu = " + repr(self.Tu) + "\n")
            log_file.write  ("CRC = " + hex(self.CRC) + "\n")
            log_file.write  ("CRC check = " + hex(self.CRC_check) + "\n")
            log_file.write  ("CRC ok = " + repr(self.CRC_ok) + "\n")

    def extract(self, pkt):
        if 1 <= pkt.PKT_ID <= 4:
            self.PKT_ID = pkt.PKT_ID
            self.time = pkt.time
            if pkt.PKT_ID == 1:     # PTU packet
                self.P = pkt.P
                self.T = pkt.T
                self.U = pkt.U
                self.Vbat = pkt.Vbat
            elif pkt.PKT_ID == 2:   # GPS packet
                self.Longitude = pkt.Longitude
                self.Latitude = pkt.Latitude
                self.Altitude = pkt.Altitude
                self.nSat = pkt.nSat
                self.Time_h = pkt.Time_h
                self.Time_m = pkt.Time_m
                self.Time_s = pkt.Time_s
            elif pkt.PKT_ID == 3:   # XDATA packet
                self.N = pkt.N
                self.XDATA = pkt.XDATA
                self.Data = pkt.Data
            elif pkt.PKT_ID == 4:   # PTUx packet
                self.P = pkt.P
                self.T = pkt.T
                self.U = pkt.U
                self.Vbat = pkt.Vbat
                self.Tint = pkt.Tint
                self.Tpr = pkt.Tpr
                self.Tu = pkt.Tu
            self.CRC = pkt.CRC
            self.CRC_check = pkt.CRC_check
            self.CRC_ok = pkt.CRC_ok

####################################################################################################

class Sounding: # iMet sounding instance

    def __init__(self, name = "iMet_", output_text_list = []):

        self.name = name + time.strftime("%Y.%m.%d") + "_" + time.strftime("%H.%M.%S")  # Set output file path
        self.path = name + time.strftime("%Y.%m.%d") + "\\" + self.name + "\\"          # Set output file name

        if not os.path.exists(self.path): os.makedirs(self.path)

        self.log_file = open(self.path + self.name + ".log", "w") # Open log file
        self.raw_file = open(self.path + self.name + ".bin", 'w')
        self.csv_file = open(self.path + self.name + ".csv", 'w')

        self.write_csv_init()
        self.row = 0

        ## Initialize member variables

        self.pkt_array = []             # data packet array
        self.current = Pkt()            # cache for current data (unpacked)

        self.n_pkt = [0]*5              # number of packets received
        self.n_pkt_CRC_ok = [0]*5       # number of packets received (CRC ok)
        self.n_false = 0                # false starts

        self.pkt_buffer_array = []      # buffer for unprocessed data packets

        self.Distance, self.Azimuth, self.ElevationAngle = 0.0, 0.0, 0.0

#--------------------------------------------------------------------------------------------------

    def __del__(self):
        self.log_file.close()
        self.raw_file.close()
        self.csv_file.close()

#--------------------------------------------------------------------------------------------------

    def calc_view_params(self, pkt):
        self.Distance, self.Azimuth, self.ElevationAngle = view_params(station_lat, station_lon, station_alt, pkt.Latitude, pkt.Longitude, pkt.Altitude)

#--------------------------------------------------------------------------------------------------

    def write_csv_init(self):

        Row = 'Station:,'
        Row += repr(station_lon) + ","
        Row += repr(station_lat) + ","
        Row += repr(station_alt) + "\n"
        self.csv_file.write(Row)

        col = ['']*49

        Row = ""
        col[0] = "#"
        col[1] = "Time"
        # GPS
        col[2] = "Latitude"
        col[3] = "Longitude"
        col[4] = "Altitude"
        col[5] = "Number of Satellites"
        col[6] = "GPS time"
        col[7] = "Distance"
        col[8] = "Azimuth"
        col[9] = "Elevation angle"
        # PTU
        col[10] = "Pressure"
        col[11] = "Temperature"
        col[12] = "Humidity"
        col[13] = "Battery voltage"
        col[14] = "T_int"
        col[15] = "T_pr"
        col[16] = "T_u"
        # XDATA
        col[17] = "Length of XDATA packet"
        col[18] = "XDATA raw"
        col[19] = "XDATA#"
        col[20] = "12"
        col[21] = "23"
        col[22] = "34"
        col[23] = "45"
        col[24] = "123"
        col[25] = "234"
        col[26] = "345"
        col[27] = "1234"
        col[28] = "2345"
        col[29] = "12345"
        col[30] = "CRC"
        col[31] = "CRC"
        col[32] = "12"
        col[33] = "23"
        col[34] = "34"
        col[35] = "45"
        col[36] = "123"
        col[37] = "234"
        col[38] = "345"
        col[39] = "1234"
        col[40] = "2345"
        col[41] = "12345"
        col[42] = "CRC"
        col[43] = "CRC"
        # counting
        col[44] = "Total Packets"
        col[45] = "PTU Packets"
        col[46] = "GPS Packets"
        col[47] = "XDATA Packets"
        col[48] = "PTUx Packets"

        Row = ''
        for c in col: Row += (c + ",")
        Row += '\n'
        self.csv_file.write(Row)

#--------------------------------------------------------------------------------------------------

    def write_csv(self):

        col = ['']*49

        col[0] = repr(self.row)
        col[1] = repr(self.current.time)
        # GPS
        col[2] = repr(self.current.Latitude)
        col[3] = repr(self.current.Longitude)
        col[4] = repr(self.current.Altitude)
        col[5] = repr(self.current.nSat)
        col[6] = repr(self.current.Time_h) + ":" + repr(self.current.Time_m) + ":" + repr(self.current.Time_s)
        col[7] = repr(self.Distance)
        col[8] = repr(self.Azimuth)
        col[9] = repr(self.ElevationAngle)
        # PTU
        col[10] = repr(self.current.P)
        col[11] = repr(self.current.T)
        col[12] = repr(self.current.U)
        col[13] = repr(self.current.Vbat)
        col[14] = repr(self.current.Tint)
        col[15] = repr(self.current.Tpr)
        col[16] = repr(self.current.Tu)
        # XDATA
        col[17] = repr(self.current.N)
        col[18] = self.current.XDATA.encode("hex")
        col[19] = repr(self.current.Data[0])
        col[20] = repr(self.current.Data[1])
        col[21] = repr(self.current.Data[2])
        col[22] = repr(self.current.Data[3])
        col[23] = repr(self.current.Data[4])
        col[24] = repr(self.current.Data[5])
        col[25] = repr(self.current.Data[6])
        col[26] = repr(self.current.Data[7])
        col[27] = repr(self.current.Data[8])
        col[28] = repr(self.current.Data[9])
        col[29] = repr(self.current.Data[10])
        col[30] = repr(self.current.Data[11])
        col[31] = repr(self.current.Data[12])
        col[32] = repr(self.current.Data[13])
        col[33] = repr(self.current.Data[14])
        col[34] = repr(self.current.Data[15])
        col[35] = repr(self.current.Data[16])
        col[36] = repr(self.current.Data[17])
        col[37] = repr(self.current.Data[18])
        col[38] = repr(self.current.Data[19])
        col[39] = repr(self.current.Data[20])
        col[40] = repr(self.current.Data[21])
        col[41] = repr(self.current.Data[22])
        col[42] = repr(self.current.Data[23])
        col[43] = repr(self.current.Data[24])
        ## counting
        col[44] = repr(self.n_pkt_CRC_ok[0]) + "//" + repr(self.n_pkt[0])
        col[45] = repr(self.n_pkt_CRC_ok[1]) + "//" + repr(self.n_pkt[1])
        col[46] = repr(self.n_pkt_CRC_ok[2]) + "//" + repr(self.n_pkt[2])
        col[47] = repr(self.n_pkt_CRC_ok[3]) + "//" + repr(self.n_pkt[3])
        col[48] = repr(self.n_pkt_CRC_ok[4]) + "//" + repr(self.n_pkt[4])

        Row = ''
        for c in col: Row += (c + ",")
        Row += '\n'
        self.csv_file.write(Row)
        self.row += 1

#---------------------------------------------------------------------------------------------------

    def parse(self, char):

        pkt_target_length = [0, 14, 18, 5, 20]
        global accumulate_gps

        if ord(char) == 1: # start new packet in buffer
            self.pkt_buffer_array.append(Pkt())

        for pkt in self.pkt_buffer_array: # for every packet in buffer append char
            pkt.string += char

            # verify PKT_ID
            if len(pkt.string) == 2:
                pkt.PKT_ID = ord(pkt.string[1])
                if 1 <= pkt.PKT_ID <=4:
                    pkt.target_length = pkt_target_length[pkt.PKT_ID]
                else:
                    self.pkt_buffer_array.remove(pkt)
                    self.n_false += 1
                    break

            # for XDATA packet
            if len(pkt.string) == 3 and pkt.PKT_ID == 3:
                pkt.N = ord(pkt.string[2])
                pkt.target_length = pkt.N+5

            # end of packet --> unpack
            if len(pkt.string) >= pkt.target_length:
                pkt.unpack()
                self.pkt_array.append(pkt)
                self.current.extract(pkt)
                ###
                if pkt.PKT_ID == 2:
                    self.calc_view_params(pkt)  # calculate view parameters
                    self.write_csv()            # write to CSV after receiving GPS packets
                    if self.current.CRC_ok == 1:
                         accumulate_gps += ("%s,%s,%s\n" % (repr(self.current.Longitude), repr(self.current.Latitude), repr(self.current.Altitude)))
                ###
                self.n_pkt[0] += 1
                self.n_pkt[pkt.PKT_ID] += 1
                if pkt.CRC_ok == 1:
                    self.n_pkt_CRC_ok[0] += 1
                    self.n_pkt_CRC_ok[pkt.PKT_ID] += 1

                pkt.print_pkt(self.log_file)
                self.displayConsole()
                sys.stdout.flush()
                PKT_ID = pkt.PKT_ID
                CRC_ok = pkt.CRC_ok
                self.pkt_buffer_array.remove(pkt)
                if CRC_ok == 1: return PKT_ID
            else:
                return 0

    def display(self, PKT_ID):
        if 1 <= PKT_ID <= 4:
            disp = [PKT_ID]
            disp.append(repr(self.n_pkt_CRC_ok[PKT_ID]) + "/" + repr(self.n_pkt[PKT_ID]))
            if PKT_ID == 1:     # PTU packet
                disp.append(self.current.T)
                disp.append(self.current.P)
                disp.append(self.current.U)
                disp.append(0)
                disp.append(self.current.Vbat)
            elif PKT_ID == 2:   # GPS packet
                disp.append(self.current.Altitude)
                disp.append(self.current.Latitude)
                disp.append(self.current.Longitude)
                disp.append(self.current.Time_h)
                disp.append(self.current.Time_m)
                disp.append(self.current.Time_s)
                disp.append(self.current.nSat)
                disp.append(self.Distance)
                disp.append(self.Azimuth)
                disp.append(self.ElevationAngle)
            elif PKT_ID == 3:   # XDATA packet
                disp.extend(self.current.Data)
            elif PKT_ID == 4:   # PTUx packet
                disp.append(self.current.T)
                disp.append(self.current.P)
                disp.append(self.current.U)
                disp.append(0)
                disp.append(self.current.Vbat)
                disp.append(self.current.Tint)
                disp.append(self.current.Tpr)
                disp.append(self.current.Tu)
            return disp

    def displayConsole(self):

        message = "################################################################################\n"
        ##
        message += "[Station]\t"
        message += time.strftime("%Y.%m.%d") + "\t" + time.strftime("%H:%M:%S") + "\n\t\t"
        message += repr(station_lat) + ", "
        message += repr(station_lon) + ", "
        message += repr(station_alt) + "\n"
        message += "\n"
        ##
        message += "[Packet] \tPKT_ID = " + repr(self.current.PKT_ID) + "\t CRC OK = "
        message += repr(self.current.CRC_ok) + "\n"
        message += "\t\tReceived time: " + repr(self.current.time) + "\n"
        message += "\t\tPackets in buffer: "  +  repr(len(self.pkt_buffer_array)) + "\t"
        message += "False packet starts: "  +  repr(self.n_false) + "\n"
        message += "# of packets:\tTotal\t\tGPS\t\tXDATA\t\tPTUx\n\t\t"
        message += repr(self.n_pkt_CRC_ok[0]) + "/" + repr(self.n_pkt[0]) + "\t\t"
        message += repr(self.n_pkt_CRC_ok[2]) + "/" + repr(self.n_pkt[2]) + "\t\t"
        message += repr(self.n_pkt_CRC_ok[3]) + "/" + repr(self.n_pkt[3]) + "\t\t"
        message += repr(self.n_pkt_CRC_ok[4]) + "/" + repr(self.n_pkt[4]) + "\n"
        message += "\n"
        ##
        message += "[GPS]\t\t\tLat\t\tLon\t\tAlt\n"
        message += "\t\t\t" + repr(round(float(self.current.Latitude),6)) + "\t"
        message += repr(round(float(self.current.Longitude),6)) + "\t"
        message += repr(self.current.Altitude) + "\n"
        message += "nSat: " + repr(self.current.nSat) + "\n"
        message += "Time: " + repr(self.current.Time_h) + ":" + repr(self.current.Time_m) + ":" + repr(self.current.Time_s) + "\t"
        message += "\tDistance\tAzimuth\t\tElevation angle\n\t\t\t"
        message += repr(round(self.Distance,3)) + " km\t"
        message += repr(round(self.Azimuth,1)) + " deg\t"
        message += repr(round(self.ElevationAngle,1)) + " deg\n"

        message += "\n"
        ##
        message += "[PTU]\t\t\t"
        message += "Pressure\tTemperature\tHumidity\n"
        message += "\t\t\t" + repr(self.current.P) + "\t\t"
        message += repr(self.current.T) + "\t\t"
        message += repr(self.current.U) + "\n"
        message += "\t\t\tVbat\tTint\tTpr\tTu\n"
        message += "\t\t\t" + repr(self.current.Vbat) + "\t"
        message += repr(self.current.Tint) + "\t"
        message += repr(self.current.Tpr) + "\t"
        message += repr(self.current.Tu) + "\n"
        message += "\n"
        ##
        message += "[XDATA]\t\t"
        message += "# " + repr(self.current.Data[0])
        message += "\t\tN = " + repr(self.current.N) + "\n"
        #
        message += "\t\t12\t23\t34\t45\t123\t234\t345\t1234\t2345\t12345\t\tCRC\n"
        message += "\t\t-----\t-----\t-----\t-----\t-----\t-----\t-----\t-----\t-----\t-----\t\t-----\n"
        message += "\t\t"
        message += repr(self.current.Data[1]) + "\t"
        message += repr(self.current.Data[2]) + "\t"
        message += repr(self.current.Data[3]) + "\t"
        message += repr(self.current.Data[4]) + "\t"
        message += repr(self.current.Data[5]) + "\t"
        message += repr(self.current.Data[6]) + "\t"
        message += repr(self.current.Data[7]) + "\t"
        message += repr(self.current.Data[8]) + "\t"
        message += repr(self.current.Data[9]) + "\t"
        message += repr(self.current.Data[10]) + "\t\t"
        #
        message += repr(self.current.Data[11]) + "\t"
        message += repr(self.current.Data[12]) + "\n"
        #
        message += "\t\t"
        message += repr(self.current.Data[13]) + "\t"
        message += repr(self.current.Data[14]) + "\t"
        message += repr(self.current.Data[15]) + "\t"
        message += repr(self.current.Data[16]) + "\t"
        message += repr(self.current.Data[17]) + "\t"
        message += repr(self.current.Data[18]) + "\t"
        message += repr(self.current.Data[19]) + "\t"
        message += repr(self.current.Data[20]) + "\t"
        message += repr(self.current.Data[21]) + "\t"
        message += repr(self.current.Data[22]) + "\t\t"
        #
        message += repr(self.current.Data[23]) + "\t"
        message += repr(self.current.Data[24]) + "\n"

        cls()
        print (message)

####################################################################################################

# Thread for fetching data
class Data_fetch(QtCore.QThread):

    updated = QtCore.pyqtSignal(list)
    COM_changed = QtCore.pyqtSignal()

    def __init__(self, *args):
        super(Data_fetch, self).__init__(*args)
        self.COM = serial.Serial()
        self.COM.baudrate   = 1200
        self.COM.timeout    = 1.0

    def run(self):
        sounding = Sounding()
        while 1:
            if not self.COM.is_open:
                time.sleep(1)
            else:
                try:
                    char = self.COM.read(1)
                    sounding.raw_file.write(char)
                    if len(char) == 1:
                        display_no = sounding.parse(char)
                        if display_no > 0:
                            self.updated.emit(sounding.display(display_no))
                except Exception as e:
                    print (e)
                    raise
                    self.restartCOM()

    def startCOM(self, port):
        if not self.COM.is_open:
            try:
                self.COM.port = str(port)
                self.COM.open()
                self.COM_changed.emit()
            except Exception as e:
                print ("Failed to open COM Port")
                print (e)

    def stopCOM(self):
        if self.COM.is_open:
            try:
                self.COM.close()
                self.COM_changed.emit()
            except Exception as e:
                print (e)

    def restartCOM(self):
        pass

################################################################################

class Update_time(QtCore.QThread):
    updated = QtCore.pyqtSignal()
    def run(self):
        while 1:
            self.updated.emit()
            sleeptime = 1 - datetime.utcnow().microsecond/1000000.0
            time.sleep(sleeptime)

################################################################################

form_class = uic.loadUiType("iMet.ui")[0] # Load the UI
class MyWindowClass(QtGui.QMainWindow, form_class):

    timeSyncThread = Update_time()
    comThread = Data_fetch()
    openComSignal = QtCore.pyqtSignal(str)
    closeComSignal = QtCore.pyqtSignal()
    ComPorts = []

    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

        self.timeSyncThread.updated.connect(self.updateTime)
        self.comThread.updated.connect(self.updateData)
        self.comThread.COM_changed.connect(self.updateComOpened)

        self.COM1_PORTS.mouseReleaseEvent = self.updateComPorts
        self.OPEN_COM1.clicked.connect(self.openCom)
        self.openComSignal.connect(self.comThread.startCOM)
        self.closeComSignal.connect(self.comThread.stopCOM)

    def updateTime(self):
        self.Time.setText(time.strftime("%y/%m/%d  %H:%M:%S", time.localtime()))

    def updateData(self, data): # Update display data
        if data[0] == 1: #PTU
            self.PTU_NO.setText(data[1])
            self.TEMP.display(data[2])
            self.PRESSURE.display(data[3])
            self.HUMIDITY.display(data[4])
            self.BARO_ALT.display(data[5])
            self.VBAT.display(data[6])
        elif data[0] == 2: #GPS
            self.GPS_NO.setText(data[1])
            self.ALT.display(data[2])
            self.LAT.display(data[3])
            self.LON.display(data[4])
            self.GPS_H.display(data[5])
            self.GPS_M.display(data[6])
            self.GPS_S.display(data[7])
            self.NSAT.display(data[8])
            self.DIST.display(data[9])
            self.AZ.display(data[10])
            self.EL.display(data[11])
        elif data[0] == 3: #XDATA
            self.XDATA_NO.setText(data[1])
            self.DATA_NUM.display(data[2])
            self.DATA12.display(data[3])
            self.DATA23.display(data[4])
            self.DATA34.display(data[5])
            self.DATA45.display(data[6])
            self.DATA123.display(data[7])
            self.DATA234.display(data[8])
            self.DATA345.display(data[9])
            self.DATA1234.display(data[10])
            self.DATA2345.display(data[11])
            self.DATA12345.display(data[12])
        elif data[0] == 4: #PTUx
            self.PTU_NO.setText(data[1])
            self.TEMP.display(data[2])
            self.PRESSURE.display(data[3])
            self.HUMIDITY.display(data[4])
            self.BARO_ALT.display(data[5])
            self.VBAT.display(data[6])
            self.TINT.display(data[7])
            self.TPR.display(data[8])
            self.TU.display(data[9])

    def updateComOpened(self): # update serial 'opened' display
        if self.comThread.COM.is_open:
            self.COM1_OPENED.setText(self.comThread.COM.name + " opened")
            self.OPEN_COM1.setText("Close COM port")
        else:
            self.COM1_OPENED.setText("Closed")
            self.OPEN_COM1.setText("Open COM port")

    def updateComPorts(self, val):
        self.COM1_PORTS.clear()
        self.ComPorts = list(list_ports.comports()) #serial.tools.list_ports
        for x in range(0,len(self.ComPorts)):
            self.COM1_PORTS.addItem(self.ComPorts[x][1])

    def openCom(self):
        if not self.comThread.COM.is_open:
            if not (len(self.ComPorts) == 0):
                self.openComSignal.emit(self.ComPorts[self.COM1_PORTS.currentIndex()][0])
        else:
            self.closeComSignal.emit()

################################################################################

thread.start_new_thread(run,())

app = QtGui.QApplication(sys.argv)
myWindow = MyWindowClass(None)
myWindow.show()
myWindow.updateComPorts("")
myWindow.updateComOpened()
myWindow.timeSyncThread.start()
myWindow.comThread.start()

app.exec_()

myWindow.comThread.COM.close()
print "Program Ended"
thread.exit()
