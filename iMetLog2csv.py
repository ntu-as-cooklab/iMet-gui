import sys, os
from math import pi, sqrt, cos, sin, tan, asin, acos, atan
from datetime import datetime
from struct import unpack

import crcmod
crc_aug_ccitt = crcmod.mkCrcFun(0x11021, rev = False, initCrc = 0x1D0F, xorOut = 0x0000)

#Station location at NTU AS
station_lat = 25.014852
station_lon = 121.538715
station_alt = 10

filename = "iMet_2016.01.24"
   
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

        self.CRC   = 0
        self.CRC_check   = 0
        self.CRC_ok = 0
        self.Data = [0]*25

        # If packet raw data is given by string, unpack data
        if self.string != "": self.unpack()

        # Timestamp
        if timestr != "":   # If time is given, use as timestamp
            self.time = timestr
        else:               # If time is not given, use current time as timestamp
            self.time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S+08:00")

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
                    self.Distance, self.Azimuth, self.ElevationAngle = view_params(station_lat, station_lon, station_alt, self.Latitude, self.Longitude, self.Altitude)
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

class iMet_log2csv:

    def __init__(self, filename):
        self.filename = filename

    def log2csv(self):

        self.log_file = open(self.filename + ".log", "r")
        self.csv_file = open(self.filename + ".csv", 'w')

        print "Log file read"

        self.write_csv_init()

        lasttime = time = string = ""
        pkt_received = [False]*5
        self.coords = ""

        for line in self.log_file:
            if line.startswith("Computer time") :
                time = line.split("\'")[1]
                if self.current_row == 1: lasttime = time
            if line.startswith("Packet string") :
                string = line.split("= ")[1].strip("\n")
                pkt = Pkt(string.decode("hex"), time)
                if pkt.CRC_ok == 1:
                    if pkt_received[pkt.PKT_ID] or not time[:17] == lasttime[:17]:
                        self.write_cols()
                        lasttime = pkt.time
                        pkt_received = [False]*5
                    self.set_cols(pkt)
                    pkt_received[pkt.PKT_ID] = True

        self.write_cols()
        self.csv_file.close()
        self.log_file.close()
        print "CSV file written"

        self.write_kml()
        print "KML file written"
        
    def write_cols(self):
        row = ""
        for c in self.col:
            row += (c + ",")
        row = row.rstrip(",")
        row += "\n"
        self.csv_file.write(row)
        self.col = ['']*self.col_num
        self.current_row += 1

    def write_csv_init(self):

        self.col_num = 29
        self.col = ['']*self.col_num

        self.col[0] = "Row #"
        self.col[1] = "Time"
        # GPS
        self.col[2] = "Latitude"
        self.col[3] = "Longitude"
        self.col[4] = "Altitude"
        self.col[5] = "Number of Satellites"
        self.col[6] = "Start time"
        self.col[7] = "Distance"
        self.col[8] = "Azimuth"
        self.col[9] = "Elevation angle"
        # PTU
        self.col[10] = "Pressure"
        self.col[11] = "Temperature"
        self.col[12] = "Humidity"
        self.col[13] = "Battery voltage"
        self.col[14] = "T_int"
        self.col[15] = "T_pr"
        self.col[16] = "T_u"
        # XDATA
        self.col[17] = "Length of XDATA packet"
        self.col[18] = "XDATA#"
        self.col[19] = "12#"
        self.col[20] = "23#"
        self.col[21] = "34#"
        self.col[22] = "45#"
        self.col[23] = "123#"
        self.col[24] = "234#"
        self.col[25] = "345#"
        self.col[26] = "1234#"
        self.col[27] = "2345#"
        self.col[28] = "12345#"

        self.current_row = 0
        self.write_cols()

#--------------------------------------------------------------------------------------------------

    def set_cols(self, pkt):

        self.col[0] = repr(self.current_row)
        self.col[1] = repr(pkt.time)
        # GPS
        if pkt.PKT_ID == 2:
            self.col[2] = repr(pkt.Latitude)
            self.col[3] = repr(pkt.Longitude)
            self.col[4] = repr(pkt.Altitude)
            self.col[5] = repr(pkt.nSat)
            self.col[6] = repr(pkt.Time_h) + ":" + repr(pkt.Time_m) + ":" + repr(pkt.Time_s)
            self.col[7] = repr(pkt.Distance)
            self.col[8] = repr(pkt.Azimuth)
            self.col[9] = repr(pkt.ElevationAngle)
            self.coords += repr(pkt.Longitude) + "," + repr(pkt.Latitude) + "," + repr(pkt.Altitude) + "\n"
        # PTU
        elif pkt.PKT_ID == 4:
            self.col[10] = repr(pkt.P)
            self.col[11] = repr(pkt.T)
            self.col[12] = repr(pkt.U)
            self.col[13] = repr(pkt.Vbat)
            self.col[14] = repr(pkt.Tint)
            self.col[15] = repr(pkt.Tpr)
            self.col[16] = repr(pkt.Tu)
        # XDATA
        elif pkt.PKT_ID == 3:
            self.col[17] = repr(pkt.N)
            self.col[18] = repr(pkt.Data[0])
            self.col[19] = repr(pkt.Data[1])
            self.col[20] = repr(pkt.Data[2])
            self.col[21] = repr(pkt.Data[3])
            self.col[22] = repr(pkt.Data[4])
            self.col[23] = repr(pkt.Data[5])
            self.col[24] = repr(pkt.Data[6])
            self.col[25] = repr(pkt.Data[7])
            self.col[26] = repr(pkt.Data[8])
            self.col[27] = repr(pkt.Data[9])
            self.col[28] = repr(pkt.Data[10])

#--------------------------------------------------------------------------------------------------

    def write_kml(self):
    
        kml =(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
            '<Document>\n'
            '   <Style id="yellowPoly">\n'
            '       <LineStyle>\n'
            '       <color>7f00ffff</color>\n'
            '       <width>4</width>\n'
            '   </LineStyle>\n'
            '   <PolyStyle>\n'
            '       <color>7f00ff00</color>\n'
            '   </PolyStyle>\n'
            '   </Style>\n'
            '    <Placemark>\n'
            '        <name>iMet path</name>\n'
            '        <styleUrl>#yellowPoly</styleUrl>\n'
            '        <LineString id="iMetPath">\n'
            '            <altitudeMode>absolute</altitudeMode>\n'
            '            <extrude>1</extrude>\n'
            '            <tessellate>1</tessellate>\n'
            '            <coordinates>\n%s</coordinates>\n'
            '        </LineString>\n'
            '    </Placemark>\n'
            '</Document>\n'
            '</kml>'
        ) %(self.coords)

        self.kml_file = open(self.filename + ".kml", 'w')
        self.kml_file.write(kml)
        self.kml_file.close()

####################################################################################################

iMet_log2csv(filename).log2csv()
