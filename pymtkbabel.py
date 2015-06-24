#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Program to interact with the GPS chip in a QStarz 1300ST logger.

Usage: pymtkbabel [<options>]

Where <options> is zero or more of:
    -b    <binfile>         read data from BIN file and continue
    --bin <binfile>
    -d     <binfile>        dump memory to file and stop
    --dump <binfile>
    --debug <level>         set debug to number <level> and continue
    --erase                 erase data logger memory and stop
    --full stop|overlap     set handling of "memory full" and continue
    -g <gpxfile>            create GPX file (tracks and waypoints) and stop
    --gpx <gpxfile>
    -h                      print help and stop
    --help
    --log <time>:<distance>:<speed>
                            set logging criteria (zero to disable) and stop:
                               <time>       0.10 -> 9999999.90 seconds
                               <distance>   0.10 -> 9999999.90 meters
                               <speed>      0.10 -> 9999999.90 km/hour
    -p <port>               set serial communication port and continue
    --port <port>
    -s <speed>              set port speed and continue
    --speed <speed>
    --tracks <gpxfile>      create a GPX file with only tracks and stop
    -v                      print version and stop
    --version
    --waypoints <gpxfile>   create a GPX file with only waypoints and stop

For example, download tracks and waypoints and create a BIN and two GPX files:
    mtkbabel --tracks gpsdata_trk.gpx --waypoints gpsdata_wpt.gpx -d gpsdata.bin
"""

import sys
import glob
import getopt
import time
import array
import binascii
import serial

import log


# program name and version
Version = 'pymtkbabel 0.1'

# serial port and speed defauts
#DefaultPort = '/dev/tty.usbmodem1410'
MinPortSpeed = 300
MaxPortSpeed = 115200
DefaultPortSpeed = MaxPortSpeed
#DefaultPortPrefix = '/dev/tty.usb*'
#DefaultPortPrefix = '/dev/ttys*'
DefaultPortPrefix = '/dev/tty*'

# debug level stuff
DefaultDebugLevel = 20
# dict mapping symbolic name to level
DebugSymbolLevel = {
                    'CRITICAL': 50,
                    'ERROR': 40,
                    'WARN': 30,
                    'INFO': 20,
                    'DEBUG': 10,
                    'NOTSET': 0,
                   }
# min/max level numbers
MinDebugLevel = 0
MaxDebugLevel = 50



Timeout = 5             # sec
TimeoutPktPreamble = 20 # sec
TimeoutIdlePort = 5000  # msec

#port = '/dev/ttyACM0'
read_buffer = ''

LOG_FORMAT_UTC = 0x00000001
LOG_FORMAT_VALID = 0x00000002
LOG_FORMAT_LATITUDE = 0x00000004
LOG_FORMAT_LONGITUDE = 0x00000008
LOG_FORMAT_HEIGHT = 0x00000010
LOG_FORMAT_SPEED = 0x00000020
LOG_FORMAT_HEADING = 0x00000040
LOG_FORMAT_DSTA = 0x00000080
LOG_FORMAT_DAGE = 0x00000100
LOG_FORMAT_PDOP = 0x00000200
LOG_FORMAT_HDOP = 0x00000400
LOG_FORMAT_VDOP = 0x00000800
LOG_FORMAT_NSAT = 0x00001000
LOG_FORMAT_SID = 0x00002000
LOG_FORMAT_ELEVATION = 0x00004000
LOG_FORMAT_AZIMUTH = 0x00008000
LOG_FORMAT_SNR = 0x00010000
LOG_FORMAT_RCR = 0x00020000
LOG_FORMAT_MILLISECOND = 0x00040000
LOG_FORMAT_DISTANCE = 0x00080000

LOG_STATUS_AUTOLOG = 0x0002
LOG_STATUS_STOP_WHEN_FULL = 0x0004
LOG_STATUS_ENABLE = 0x0100
LOG_STATUS_DISABLE = 0x0200
LOG_STATUS_NEED_FORMAT = 0x0400
LOG_STATUS_FULL = 0x0800

RCD_METHOD_OVF = 1
RCD_METHOD_STP = 2

SIZEOF_BYTE = 1
SIZEOF_WORD = 2
SIZEOF_LONG = 4
SIZEOF_FLOAT3 = 3
SIZEOF_FLOAT = 4
SIZEOF_DOUBLE = 8

SIZEOF_CHUNK = 0x800
SIZEOF_SECTOR = 0x10000
SIZEOF_SECTOR_HEADER = 0x200
SIZEOF_SEPARATOR = 0x10

class QStarz(object):
    """Class to handle comms with chip in QStarz logger."""

    def __init__(self, device, speed):
        """Initialize the device.

        device  the device to use
        speed   comms speed
        """

        self.memory = None
        self.read_buffer = ''
        self.sane = True

        if device is None:
            device = self.find_device(speed)

        try:
            self.serial = serial.Serial(port=device, baudrate=speed, timeout=0)
        except OSError:
            log.debug('device %s is not sane' % device)
            self.sane = False
            return
        if not self.send('PMTK000'):
            log.debug('device %s is not sane' % device)
            self.sane = False
            return
        log.debug('****** device %s IS sane' % device)

        ret = self.recv('PMTK001,0,')
        if not ret or not ret.startswith('PMTK001,0,'):
            log.debug('device %s is not a QStarz device' % device)
            self.sane = False
            return

        log.debug('device %s is a QStarz device, created' % device)

    def init(self):
        if not self.sane:
            return False

        self.send('PMTK604')
        ret = self.recv('PMTK001,604,')
        self.version = ret.split(',')[2]
    
        self.send('PMTK605')
        ret = self.recv('PMTK705,')
        ret_list = ret.split(',')
        self.release = ret_list[1]
        self.model_id = ret_list[2]
    
        log.info('QStarz: MTK Firmware: Version %s, Release %s, Model ID %s' % (self.version, self.release, self.model_id))
    
        # query log format
        self.send('PMTK182,2,2')
        ret = self.recv('PMTK182,3,2,')
        fmt = ret.split(',')[3]
        self.log_format = int(fmt, 16)
        log.info('QStarz: Log format: %s' % describe_log_format(self.log_format))
    
        self.send('PMTK182,2,6')
        self.recv('PMTK001,182,2,3')
        method = self.recv('PMTK182,3,6,')
        self.rec_method = int(method.split(',')[3])
        log.info('QStarz: Recording method on memory full: %s' % describe_recording_method(self.rec_method))
    
        # query RCD_ADDR data
        self.send('PMTK182,2,8')
        ret = self.recv('PMTK182,3,8')
        self.recv('PMTK001,182,2,3')
        if ret:
            self.next_write_address = int(ret.split(',')[3], 16)
            log.info('QStarz: Next write address: 0x%04x (%d)' % (self.next_write_address, self.next_write_address))
    
        # query number of records written
        self.send('PMTK182,2,10')
        ret = self.recv('PMTK182,3,10')
        self.send('PMTK001,182,2,3')
        if ret:
            self.expected_records_total = ret.split(',')[3]
            log.info('QStarz: Number of records: %s (%d)' % (self.expected_records_total, int(self.expected_records_total, 16)))

        return True

    def __del__(self):
        if self.serial:
            del self.serial

    def read_memory(self):
        """Read device memory."""

        # bomb out if device data already read
        if self.memory is not None:
            return

        # read data
        if self.rec_method == RCD_METHOD_OVF:
            # in OVERLAP mode we don't know where data ends, read it all
            bytes_to_read = flash_memory_size(self.model_id)
        else:
            # in STOP mode read from zero to NextWriteAddress
            sectors = int(self.next_write_address / SIZEOF_SECTOR)
            if self.next_write_address % SIZEOF_SECTOR:
                sectors += 1
            bytes_to_read = sectors * SIZEOF_SECTOR
    
        log.info('Retrieving %d (0x%08x) bytes of log data from device' % (bytes_to_read, bytes_to_read))
    
        non_written_sector_found = False
    
        offset = 0
        data = ''
        while offset < bytes_to_read:
            self.send('PMTK182,7,%08x,%08x' % (offset, SIZEOF_CHUNK))
            msg = self.recv('PMTK182,8', 10)
            if msg:
                (address, buff) = msg.split(',')[2:]
                #print('len=%d, buff=%s' % (len(buff), buff))
                data += buff
                offset += SIZEOF_CHUNK
            self.recv('PMTK001,182,7,3', 10)
    
        data = data.decode('hex')
        log.debug('%d bytes read (expected %d), len(data)=%d' % (offset, bytes_to_read, len(data)))
        self.memory = data

    def get_memory(self):
        """Get device memory."""

        if self.memory is None:
            self.read_memory()
        return self.memory

    def set_memory(self, memory):
        """Set device memory."""

        self.memory = memory

    def send(self, msg):
        checksum = self.msg_checksum(msg)
        msg = '$%s*%02x\r\n' % (msg, checksum)
        try:
            self.serial.write(msg)
        except serial.SerialException:
            log.debug('QStarz.send: failed')
            return False
        log.debug('QStarz.send: %s' % msg[:-2])
        return True

    def recv(self, prefix, timeout=Timeout):
        """Receive message with given prefix."""

        max_time = time.time() + timeout
    
        while True:
            pkt = self.read_pkt(timeout=timeout)
            log.debug("QStarz.recv: prefix='%s', pkt='%-40s'" % (prefix, pkt))
            if pkt.startswith(prefix):
                log.debug('QStarz.recv: Got desired packet: %s' % prefix)
                return pkt
            if time.time() > max_time:
                log.info('##################### packet_wait: timeout')
                break
            time.sleep(0.01)
    
        return None

    def read_pkt(self, timeout=None):
        """Read a packet from the device.
        
        timeout  read timeout in seconds
        """

        if timeout is None:
            timeout = TimeoutIdlePort
        log.debug('read_pkt: timeout=%s' % str(timeout))

        then = time.time() + timeout

        pkt = ''
        while time.time() < then:
            if '\n' in self.read_buffer:
                result = self.read_buffer[:self.read_buffer.index('\n')+1]
                self.read_buffer = self.read_buffer[self.read_buffer.index('\n')+1:]
                # get packet, check checksum
                pkt = result[1:-5]
                checksum = result[-4:-2]
                log.debug("QStarz.read_pkt: pkt='%s', checksum='%s'" % (pkt, checksum))
                if checksum != self.msg_checksum(pkt):
                    log.info('Checksum error on read, got %s expected %s' %
                             (checksum, self.msg_checksum(pkt)))
#                    raise Exception('Checksum error on read, got %s expected %s' %
#                                    (checksum, packet_checksum(pkt)))
                return pkt
            try:
                data = self.serial.read(9999)
            except serial.SerialException:
                return pkt
            log.debug('read_pkt: read data=%s' % str(data))
            if len(data) > 0:
                self.read_buffer += data
                log.debug('read_pkt: appended to buffer=%s' % str(data))
            else:
                log.debug('read_pkt: sleeping')
                time.sleep(0.1)

        return pkt

    def msg_checksum(self, msg):
        result = 0
        for ch in msg:
            result ^= ord(ch)
        log.debug("msg='%s', checksum=0x%02x" % (msg, result))
        return result


def find_device(speed):
    """Find the device amongst /dev/*.
    
    Use the given device speed.  Looks under DefaultDevicePath.
    Return None if not found.
    """

    log.debug('find_device: speed=%d' % speed)
    for port in get_tty_port():
        log.debug('find_device: checking device %s' % port)
        gps = QStarz(port, speed)
        if not gps.init():
        #if gps is None:
            log.debug('find_device: device %s was invalid' % port)
            del gps
            continue
        log.debug('find_device: returning device %s' % port)
        del gps
        return port

    log.debug('find_device: No port found')
    return None

def describe_log_format(log_format):
    result = ''

    if log_format | LOG_FORMAT_UTC: result += ',UTC' 
    if log_format | LOG_FORMAT_VALID: result += ',VALID' 
    if log_format | LOG_FORMAT_LATITUDE: result += ',LATITUDE' 
    if log_format | LOG_FORMAT_LONGITUDE: result += ',LONGITUDE' 
    if log_format | LOG_FORMAT_HEIGHT: result += ',HEIGHT' 
    if log_format | LOG_FORMAT_SPEED: result += ',SPEED' 
    if log_format | LOG_FORMAT_HEADING: result += ',HEADING' 
    if log_format | LOG_FORMAT_DSTA: result += ',DSTA' 
    if log_format | LOG_FORMAT_DAGE: result += ',DAGE' 
    if log_format | LOG_FORMAT_PDOP: result += ',PDOP' 
    if log_format | LOG_FORMAT_HDOP: result += ',HDOP' 
    if log_format | LOG_FORMAT_VDOP: result += ',VDOP' 
    if log_format | LOG_FORMAT_NSAT: result += ',NSAT' 
    if log_format | LOG_FORMAT_SID: result += ',SID' 
    if log_format | LOG_FORMAT_ELEVATION: result += ',ELEVATION' 
    if log_format | LOG_FORMAT_AZIMUTH: result += ',AZIMUTH' 
    if log_format | LOG_FORMAT_SNR: result += ',SNR' 
    if log_format | LOG_FORMAT_RCR: result += ',RCR' 
    if log_format | LOG_FORMAT_MILLISECOND: result += ',MILLISECOND' 
    if log_format | LOG_FORMAT_DISTANCE: result += ',DISTANCE' 

    return result[1:]


def describe_log_status(log_status):
    result = ''

    result += ',AUTOLOG_ON' if log_status & LOG_STATUS_AUTOLOG \
                            else ',AUTOLOG_OFF'
    result += ',STOP_WHEN_FULL' if log_status & LOG_STATUS_STOP_WHEN_FULL \
                                else ',OVERLAP_WHEN_FULL'
    if log_status & LOG_STATUS_ENABLE: result += ',ENABLE_LOG'
    if log_status & LOG_STATUS_DISABLE: result += ',DISABLE_LOG'
    if log_status & LOG_STATUS_NEED_FORMAT: result += ',NEED_LOG'
    if log_status & LOG_STATUS_FULL: result += ',FULL'

    return result[1:]


def describe_recording_method(method):
    if method == RCD_METHOD_OVF:
        return 'OVERLAP'
    if method == RCD_METHOD_STP:
        return 'STOP'


def unpack(byte_array):
    """Unpack byte array into binary (LSB first)."""

#    print('unpack, byte_array=%s' % binascii.b2a_hex(byte_array))
    result = 0
    byte_array.reverse()
    for val in byte_array:
#        print('val=%d' % val)
        result = result*256 + val

    return result


def parse_sector_header(sector_header):
    """Parse a log sector header."""

    log.debug('sector_header=%s' % binascii.b2a_hex(sector_header))

    separator = chr(sector_header[-6])
    checksum = sector_header[-5]
    header_tail = binascii.b2a_hex(sector_header[-4:])
    if separator != '*' or header_tail != 'bbbbbbbb':
#        print('ERROR: Invalid datalog sector header, see log')
        log('ERROR: Invalid sector header, see above')
        sys.exit(1)

    offset = 0
    log_count = sector_header[offset:SIZEOF_WORD]
    offset += SIZEOF_WORD
    log_format = sector_header[offset:offset+SIZEOF_LONG]
    offset += SIZEOF_LONG
    log_status = sector_header[offset:offset+SIZEOF_WORD]
    offset += SIZEOF_WORD
    log_period = sector_header[offset:offset+SIZEOF_LONG]
    offset += SIZEOF_LONG
    log_distance = sector_header[offset:offset+SIZEOF_LONG]
    offset += SIZEOF_LONG
    log_speed = sector_header[offset:offset+SIZEOF_LONG]
    offset += SIZEOF_LONG
    log_failsect = sector_header[offset:offset+SIZEOF_BYTE*32]

    log.debug('log_count=%s' % binascii.b2a_hex(log_count))
    log.debug('log_format=%s' % binascii.b2a_hex(log_format))
    log.debug('log_status=%s' % binascii.b2a_hex(log_status))
    log.debug('log_period=%s' % binascii.b2a_hex(log_period))
    log.debug('log_distance=%s' % binascii.b2a_hex(log_distance))
    log.debug('log_speed=%s' % binascii.b2a_hex(log_speed))
    log.debug('log_failsect=%s' % binascii.b2a_hex(log_failsect))

    log.debug('len(log_count)=%d' % len(log_count))
    log.debug('len(log_format)=%d' % len(log_format))

    log_count = unpack(log_count)
    log_format = unpack(log_format)

    log.debug('log_count=%s' % str(log_count))
    log.debug('log_format=%s' % str(log_format))

    return (log_count, log_format)


def parse_log_data(data):
    """Parse log data.

    data  bytearray of log data
    """

    fp = 0
    size = len(data)

    while True:
#        print('>> Reading offset %08x' % fp)

        if (fp % SIZEOF_SECTOR) == 0:
            # reached the beginning of a log sector (every 0x10000 bytes),
            # get header (0x200 bytes)
            header = data[fp:fp+SIZEOF_SECTOR_HEADER]
            (expected_records_sector, log_format) = parse_sector_header(header)
#            print('expected_records_sector=%04x' % expected_records_sector)
#            print('log_format=%08x' % log_format)

            record_count_sector = 0

        if record_count_total >= expected_records_total:
#            print('Total record count: %d' % record_count_total)
            break

        if record_count_sector >= expected_records_sector:
            new_offset = SIZEOF_SECTOR * (fp/SIZEOF_SECTOR + 1)
            if new_offset < log_len:
                fp = new_offset
                continue
            else:
                # end of file
                pass
#                print('Total record count: %d' % record_count_total)
            break


#ser = serial.Serial(port=port, baudrate=115200, timeout=0)
#packet_send('PMTK000')
#ret = packet_wait('PMTK001,0,')
#if not ret.startswith('PMTK001,0,'):
#    raise Exception('MTK Test command failed')
#packet_send('PMTK604')
#ret = packet_wait('PMTK001,604,')
#version = ret.split(',')[2]
#
#packet_send('PMTK605')
#ret = packet_wait('PMTK705,')
#ret_list = ret.split(',')
#release = ret_list[1]
#model_id = ret_list[2]
#
#print('MTK Firmware: Version %s, Release %s, Model ID %s'
#      % (version, release, model_id))
#
## query log format
#packet_send('PMTK182,2,2')
#ret = packet_wait('PMTK182,3,2,')
#fmt = ret.split(',')[3]
#log_format = int(fmt, 16)
#print('Log format: (%s) %s' % (fmt, describe_log_format(log_format)))
#
## query recording criteria
#packet_send('PMTK182,2,3')
#ret = packet_wait('PMTK182,3,3,')
#period = float(ret.split(',')[3]) / 10
#print('Logging TIME interval:       %6.2f s' % period)
#
#packet_send('PMTK182,2,4')
#ret = packet_wait('PMTK182,3,4,')
#distance = float(ret.split(',')[3]) / 10
#print('Logging DISTANCE interval: %6.2f m' % distance)
#
#packet_send('PMTK182,2,5')
#ret = packet_wait('PMTK182,3,5,')
#speed = float(ret.split(',')[3]) / 10
#print('Logging SPEED limit:       %6.2f km/h' % speed)
#
## query recording method when full
#packet_send('PMTK182,2,6')
#packet_wait('PMTK001,182,2,3')
#method = packet_wait('PMTK182,3,6,')
#rec_method = int(method.split(',')[3])
#print('Recording method on memory full: (%d) %s'
#      % (rec_method, describe_recording_method(rec_method)))
#
## query log status
#packet_send('PMTK182,2,7')
#packet_wait('PMTK001,182,2,3')
#ret = packet_wait('PMTK182,3,7,')
#if ret:
#    status = ret.split(',')[3]
#    log_status = int(status)
#    print('Log status: (%s) %s'
#          % (bin(log_status), describe_log_status(log_status)))
#    if log_status & LOG_STATUS_NEED_FORMAT:
#        print('WARNING! Log status NEED_FORMAT, log data is not valid!')
#    if log_status & LOG_STATUS_DISABLE:
#        print('WARNING! Log status DISABLE_LOG, too many failed sectors!')
#
## query RCD_ADDR data
#packet_send('PMTK182,2,8')
#ret = packet_wait('PMTK182,3,8')
#packet_wait('PMTK001,182,2,3')
#if ret:
#    next_write_address = int(ret.split(',')[3], 16)
#    print('Next write address: 0x%04x (%d)'
#          % (next_write_address, next_write_address))
#
## query number of records written
#packet_send('PMTK182,2,10')
#ret = packet_wait('PMTK182,3,10')
#packet_wait('PMTK001,182,2,3')
#if ret:
#    expected_records_total = ret.split(',')[3]
#    print('Number of records: %s (%d)' % (expected_records_total,
#                                          int(expected_records_total, 16)))
#
########################################
## dump data
#if rec_method == RCD_METHOD_OVF:
#    # in OVERLAP mode we don't know where data ends, read it all
#    bytes_to_read = flash_memory_size(model_id)
#else:
#    # in STOP mode read from zero to NextWriteAddress
#    sectors = int(next_write_address / SIZEOF_SECTOR)
#    if next_write_address % SIZEOF_SECTOR:
#        sectors += 1
#    bytes_to_read = sectors * SIZEOF_SECTOR
#
#print('Retrieving %d (0x%08x) bytes of log data from device'
#      % (bytes_to_read, bytes_to_read))
#
#non_written_sector_found = False
#
#offset = 0
#data = ''
#while offset < bytes_to_read:
#    packet_send('PMTK182,7,%08x,%08x' % (offset, SIZEOF_CHUNK))
#    msg = packet_wait('PMTK182,8', 10)
#    if msg:
#        (address, buff) = msg.split(',')[2:]
#        data += buff
#        offset += SIZEOF_CHUNK
#        #log('data buffer address=%s' % address)
#    packet_wait('PMTK001,182,7,3', 10)
#print('')
#print('%d bytes read (expected %d)' % (offset, bytes_to_read))
#print('')
#
#data = bytearray.fromhex(data)
#with open('xyzzy.bin', 'wb') as fd:
#    fd.write(data)
#parse_log_data(data)
#
#ser.close()


def get_tty_port():
    """Guess the logger port.

    Return ports found.
    """

    ports = glob.glob(DefaultPortPrefix)
    return ports


def read_memory(port, speed):
    """Read memory data.

    port   the port to read
    speed  speed of the port
    """

    ser = serial.Serial(port=port, baudrate=speed, timeout=0)
    packet_send(ser, 'PMTK000')
    ret = packet_wait(ser, 'PMTK001,0,')
    if not ret.startswith('PMTK001,0,'):
        raise Exception('MTK Test command failed')
    packet_send(ser, 'PMTK604')
    ret = packet_wait(ser, 'PMTK001,604,')
    version = ret.split(',')[2]

    packet_send(ser, 'PMTK605')
    ret = packet_wait(ser, 'PMTK705,')
    ret_list = ret.split(',')
    release = ret_list[1]
    model_id = ret_list[2]

    print('MTK Firmware: Version %s, Release %s, Model ID %s' % (version, release, model_id))

    # query log format
    packet_send(ser, 'PMTK182,2,2')
    ret = packet_wait(ser, 'PMTK182,3,2,')
    fmt = ret.split(',')[3]
    log_format = int(fmt, 16)
    print('Log format: (%s) %s' % (fmt, describe_log_format(log_format)))

    packet_send(ser, 'PMTK182,2,6')
    packet_wait(ser, 'PMTK001,182,2,3')
    method = packet_wait(ser, 'PMTK182,3,6,')
    rec_method = int(method.split(',')[3])
    print('Recording method on memory full: (%d) %s' % (rec_method, describe_recording_method(rec_method)))

    # query RCD_ADDR data
    packet_send(ser, 'PMTK182,2,8')
    ret = packet_wait(ser, 'PMTK182,3,8')
    packet_wait(ser, 'PMTK001,182,2,3')
    if ret:
        next_write_address = int(ret.split(',')[3], 16)
        print('Next write address: 0x%04x (%d)' % (next_write_address, next_write_address))

    # query number of records written
    packet_send(ser, 'PMTK182,2,10')
    ret = packet_wait(ser, 'PMTK182,3,10')
    packet_wait(ser, 'PMTK001,182,2,3')
    if ret:
        expected_records_total = ret.split(',')[3]
        print('Number of records: %s (%d)' % (expected_records_total, int(expected_records_total, 16)))

    # read data
    if rec_method == RCD_METHOD_OVF:
        # in OVERLAP mode we don't know where data ends, read it all
        bytes_to_read = flash_memory_size(model_id)
    else:
        # in STOP mode read from zero to NextWriteAddress
        sectors = int(next_write_address / SIZEOF_SECTOR)
        if next_write_address % SIZEOF_SECTOR:
            sectors += 1
        bytes_to_read = sectors * SIZEOF_SECTOR

    print('Retrieving %d (0x%08x) bytes of log data from device' % (bytes_to_read, bytes_to_read))

    non_written_sector_found = False

    offset = 0
    data = ''
    while offset < bytes_to_read:
        packet_send(ser, 'PMTK182,7,%08x,%08x' % (offset, SIZEOF_CHUNK))
        msg = packet_wait(ser, 'PMTK182,8', 10)
        if msg:
            (address, buff) = msg.split(',')[2:]
#            print('len=%d, buff=%s' % (len(buff), buff))
            data += buff
            offset += SIZEOF_CHUNK
        packet_wait(ser, 'PMTK001,182,7,3', 10)

    data = data.decode('hex')
    print('%d bytes read (expected %d), len(data)=%d' % (offset, bytes_to_read, len(data)))

    return data

def usage(msg=None):
    print(__doc__)        # module docstring used
    if msg:
        print('-'*80)
        print(msg)
        print('-'*80)


def not_yet_implemented(op):
    print("Sorry, '%s' is not yet implemented" % op)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    try:
        opts, args = getopt.getopt(argv, 'b:d:g:hp:s:v',
                                   ['bin=', 'dump=', 'debug=', 'erase', 'full=',
                                    'gpx=', 'help', 'log=', 'port=', 'speed=',
                                    'tracks=', 'version', 'waypoints='])
    except getopt.error as msg:
        usage(str(msg))
        return 1

    if len(args) != 0:
        usage()
        return 1

    # get debug level, set up logger
    debug_level = DefaultDebugLevel
    for (opt, param) in opts:
        if opt in ['--debug']:
            try:
                debug_level = int(param)
            except ValueError:
                # not int, could be symbolic
                param = param.upper()
                if param in DebugSymbolLevel:
                    debug_level = DebugSymbolLevel[param]
                else:
                    debug_level = -1
                if debug_level < MinDebugLevel or debug_level > MaxDebugLevel:
                    usage("Option '%s' requires integer or symbolic level" % opt)
                    return 1
    global log
    log = log.Log('mtkbabel.log', debug_level)
    if debug_level != DefaultDebugLevel:
        log.critical('Debug level set to %d' % debug_level)
    log.critical('main: argv=%s' % str(argv))

    # set default values
    port = None
    ports = get_tty_port()
    if len(ports) == 1:
        port = ports[0]
    speed = DefaultPortSpeed
    log.debug('port=%s, speed=%s' % (str(port), str(speed)))

    # pick out help, device, speed and version options
    for (opt, param) in opts:
        if opt in ['-h', '--help']:
            usage()
            return 0
        if opt in ['-p', '--port']:
            port = param
            log.info('Set port to %s' % port)
        if opt in ['-s', '--speed']:
            try:
                speed = int(param)
            except ValueError:
                usage("Option '%s' requires integer speed" % opt)
            if speed < MinPortSpeed or speed > MaxPortSpeed:
                error('Speed error, allowable range is (%d, %d)' % (MinPortSpeed, MaxPortSpeed))
            log.info('Set port speed to %d' % speed)
        if opt in ['-v', '--version']:
            print(Version)
            return 0

    # create QStarz object, if possible
    if port is None:
        port = find_device(speed)
        if port is None:
            log.critical('No port specified & none found, choices: %s' % ', '.join(ports))
            print('No port specified & none found, choices: %s' % ', '.join(ports))
            return 1
    gps = QStarz(port, speed)
    if not gps.init():
        log.debug('Device is %s, speed %d is not a QStarz device' % (str(port), speed))
        return 1
    log.debug('Device is %s, speed %d' % (str(port), speed))

    # now handle remaining options
    for (opt, param) in opts:
        if opt in ['-b', '--bin']:
            with open(param, 'rb') as fd:
                memory = fd.read()
            gps.set_memory(memory)
        if opt in ['-d', '--dump']:
            log.debug('Dumping memory to file %s' % param)
            memory = gps.get_memory()
            log.info('Read %d bytes' % len(memory))
            with open(param, 'wb') as fd:
                fd.write(memory)
            log.info('Wrote %d bytes to file %s' % (len(memory), param))
            return 0
        if opt in ['--erase']:
            not_yet_implemented('erase memory')
            return 0
        if opt in ['--full']:
            not_yet_implemented('memory full handling')
            return 0
        if opt in ['-g', '--gpx']:
            log.debug('Got --gpx option')
            data = gps.get_memory()
            parse_log_data(data)
            #self.write_gpx(param)
        if opt in ['--log']:
            not_yet_implemented('set logging criteria')
            return 0
        if opt in ['--tracks']:
            not_yet_implemented('write tracks GPX')
            return 0
        if opt in ['--waypoints']:
            not_yet_implemented('write waypoints GPX')
            return 0



if __name__ == '__main__':
    import traceback

    # our own handler for uncaught exceptions
    def excepthook(type, value, tb):
        msg = '\n' + '=' * 80
        msg += '\nUncaught exception:\n'
        msg += ''.join(traceback.format_exception(type, value, tb))
        msg += '=' * 80 + '\n'
                                            
        print(msg)
        log.critical(msg)
        sys.exit(1)

    # plug our handler into the python system
    sys.excepthook = excepthook

    sys.exit(main(sys.argv[1:]))
