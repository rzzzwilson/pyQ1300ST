#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test program: find the GPS device.
"""

import sys
import glob
import time
import serial

import log


MaxPortSpeed = 115200
#MaxPortSpeed = 2400
DefaultPortPrefix = '/dev/tty*'

Timeout = 0.50          # sec, make bigger if device is slow
TimeoutPktPreamble = 20 # sec
#TimeoutIdlePort = 5000  # msec
TimeoutIdlePort = 500  # msec

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

RCD_METHOD_OVF = 1
RCD_METHOD_STP = 2


def describe_recording_method(method):
    if method == RCD_METHOD_OVF:
        return 'OVERLAP'
    if method == RCD_METHOD_STP:
        return 'STOP'

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

def send(ser, msg):
    checksum = msg_checksum(msg)
    msg = '$%s*%02x\r\n' % (msg, checksum)
    try:
        ser.write(msg)
    except serial.SerialException:
        log.debug('QStarz.send: failed')
        return False
    log.debug('QStarz.send: %s' % msg[:-2])
    return True

def recv(ser, prefix, timeout=Timeout):
    """Receive message with given prefix."""

    max_time = time.time() + timeout

    while True:
        pkt = ser.read_pkt(timeout=timeout)
        log.debug("QStarz.recv: prefix='%s', pkt='%-40s'" % (prefix, pkt))
        if pkt.startswith(prefix):
            log.debug('QStarz.recv: Got desired packet: %s' % prefix)
            return pkt
        if time.time() > max_time:
            log.info('##################### packet_wait: timeout')
            break
        time.sleep(0.1)

    return None



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
            log.debug('QStartz object must be given a valid port, not None')
            raise RuntimeError('QStartz object must be given a valid port, not None')
        try:
            self.serial = serial.Serial(port=device, baudrate=speed, timeout=0)
        except OSError:
            log.debug('device %s is not sane' % device)
            self.sane = False
            return
        except serial.SerialException:
            log.debug('device %s is not readable' % device)
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
        self.sane = True

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
        if hasattr(self, 'serial'):
#        if self.serial:
            log('__del__: self.serial=%s' % str(self.serial))
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
                # get complete response
                result = self.read_buffer[:self.read_buffer.index('\n')+1]
                self.read_buffer = self.read_buffer[self.read_buffer.index('\n')+1:]

                # get packet, check checksum
                pkt = result[1:-5]
                checksum = result[-4:-2]
                log.debug("QStarz.read_pkt: pkt='%s', checksum='%s'"
                          % (str(pkt), checksum))
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
            if len(data) > 0:
                self.read_buffer += data
            else:
                time.sleep(0.1)

        return pkt

    def msg_checksum(self, msg):
        result = 0
        for ch in msg:
            result ^= ord(ch)
        return result

def find_devices(speed):
    """Find the QStarz devices amongst /dev/*.

    Use the given device speed.  Looks under DefaultDevicePath.
    Return None if not found.
    """

    log.debug('find_device: speed=%d' % speed)
    result = []

    for device in get_tty_device():
        if device == '/dev/tty':
            # don't interrogate the console!
            continue

        log.debug('find_device: checking device %s' % device)
        gps = None
        try:
            gps = QStarz(device, speed)
        except serial.SerialError:
            del gps

        if gps and gps.sane:
            log.debug('find_device: adding device %s' % device)
            result.append(device)
            del gps

    log.debug('find_device: returning: %s' % str(result))
    return result


def get_tty_device():
    """Guess the logger device.

    Return devices found.
    """

    devices = glob.glob(DefaultPortPrefix)
#    devices.reverse()
    log.debug('get_tty_port() returns:\n%s' % str(devices))
    return devices


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
            data += buff
            offset += SIZEOF_CHUNK
        packet_wait(ser, 'PMTK001,182,7,3', 10)

    data = data.decode('hex')
    print('%d bytes read (expected %d), len(data)=%d' % (offset, bytes_to_read, len(data)))

    return data

def main(argv=None):
    global log
    log = log.Log('mtkbabel.log', 10)
    log.critical('main: argv=%s' % str(argv))

    # set default values
    speed = MaxPortSpeed
    devices = find_devices(speed)
    log.debug('Found devices=%s' % str(devices))
    if len(devices) == 0:
        print('No QStarz devices found!?')
    elif len(devices) == 1:
        print('Found device %s' % str(devices[0]))
    else:
        print('Found more than one device: %s' % ', '.join(devices))

main()
