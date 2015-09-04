#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class encapsulating the QStarz BT-Q1300ST logger handling.
"""

import sys
import glob
import time
import serial
import binascii

import log


class BTQ1300ST(object):
    """Class to handle comms with chip in QStarz BT-Q1300ST logger."""

    # this may need to be OS dependant
    DefaultDevicePath = '/dev/tty*'

    PortSpeeds = [115200, 57600, 38400, 19200, 14400, 9600, 4800]
    PortSpeeds.sort()

    Timeout = 0.50          # sec, make bigger if device is slow
    TimeoutPktPreamble = 20 # sec
    TimeoutIdlePort = 500   # msec

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

    def __init__(self, device, speed):
        """Initialize the device.

        device  the device to use
        speed   comms speed
        """

        self.memory = None
        self.read_buffer = ''
        self.sane = False

        if device is None:
            log.debug('QStartz object must be given a valid port, not None')
            raise RuntimeError('QStartz object must be given a valid port, not None')
        try:
            self.serial = serial.Serial(port=device, baudrate=speed, timeout=0)
        except OSError:
            log.debug('device %s is not sane' % device)
            return
        except serial.SerialException:
            log.debug('device %s is not readable' % device)
            return

        if not self.send('PMTK000'):
            log.debug('device %s is not sane' % device)
            return
        log.debug('****** device %s IS sane' % device)

        ret = self.recv('PMTK001,0,3')
        if not ret or not ret.startswith('PMTK001,0,'):
            log.debug('device %s is not a BT-Q1300ST device' % device)
            return

        log.debug('device %s is a BT-Q1300ST device' % device)
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

        log.info('BTQ1300ST: ******** MTK Firmware: Version %s, Release %s, Model ID %s' % (self.version, self.release, self.model_id))

        # query log format
        self.send('PMTK182,2,2')
        ret = self.recv('PMTK182,3,2,')
        fmt = ret.split(',')[3]
        self.log_format = int(fmt, 16)
        log.info('BTQ1300ST: ******** Log format: %s' % self.describe_log_format(self.log_format))

        self.send('PMTK182,2,6')
        self.recv('PMTK001,182,2,3')
        method = self.recv('PMTK182,3,6,')
        self.rec_method = int(method.split(',')[3])
        log.info('BTQ1300ST: ******** Recording method on memory full: %s' % self.describe_recording_method(self.rec_method))

        # query RCD_ADDR data
        self.send('PMTK182,2,8')
        ret = self.recv('PMTK182,3,8')
        self.recv('PMTK001,182,2,3')
        if ret:
            self.next_write_address = int(ret.split(',')[3], 16)
            log.info('BTQ1300ST: ******** Next write address: 0x%04x (%d)' % (self.next_write_address, self.next_write_address))

        # query number of records written
        self.send('PMTK182,2,10')
        ret = self.recv('PMTK182,3,10')
        self.send('PMTK001,182,2,3')
        if ret:
            self.expected_records_total = ret.split(',')[3]
            log.info('BTQ1300ST: ******** Number of records: %s (%d)' % (self.expected_records_total, int(self.expected_records_total, 16)))

        return True

    def __del__(self):
        if self.sane and hasattr(self, 'serial') and self.serial:
            del self.serial

    def read_memory(self):
        """Read device memory."""

        # compute the memory used by data log, round-up to the entire sector
        if self.rec_method == self.RCD_METHOD_OVF:
            # in OVERLAP mode we don't know where data ends, read it all
            log.info('read_memory: OVERLAP mode, read entire memory')
            bytes_to_read = flash_memory_size(self.model_id)
        else:
            # in STOP mode we read from zero to NextWriteAddress
            log.info('read_memory: STOP mode, read zero to next write position')
            log.debug('.next_write_address=%06x (%d), .SIZEOF_SECTOR=%06x (%d)'
                      % (self.next_write_address, self.next_write_address,
                         self.SIZEOF_SECTOR, self.SIZEOF_SECTOR))
            sectors = int(self.next_write_address / self.SIZEOF_SECTOR)
            if self.next_write_address % self.SIZEOF_SECTOR:
                sectors += 1
            bytes_to_read = sectors * self.SIZEOF_SECTOR

        log.info('Retrieving %d (0x%08x) bytes of log data from device' % (bytes_to_read, bytes_to_read))

        non_written_sector_found = False

        offset = 0
        data = ''

        while offset < bytes_to_read:
            # request the next CHUNK of data
            self.send('PMTK182,7,%08x,%08x' % (offset, self.SIZEOF_CHUNK))
            msg = self.recv('PMTK182,8', 10)
            if msg:
                (address, buff) = msg.split(',')[2:]
                data += buff

                if (offset % self.SIZEOF_SECTOR) == 0:
                    if msg[19:19+self.SIZEOF_SEPARATOR*2] == 'FF'*self.SIZEOF_SEPARATOR:
                        print('WARNING: Sector header at offset 0x%08X is non-written data' % offset)
                        log.debug('read_memory: Got sector of non-written data at 0x%06x, ending read' % offset)
                        break

                offset += self.SIZEOF_CHUNK

                # update user 'percent read' display
                percent = offset * 100.0 / bytes_to_read
                sys.stdout.write('\rSaved log data: %6.2f%%' % percent)
                sys.stdout.flush()

            self.recv('PMTK001,182,7,3', 10)

        print('')   # terminate user 'percent read' display

        while offset < bytes_to_read:
            # request the next CHUNK of data
            self.send('PMTK182,7,%08x,%08x' % (offset, self.SIZEOF_CHUNK))
            msg = self.recv('PMTK182,8', 10)
            if msg:
                (address, buff) = msg.split(',')[2:]
                data += buff

                if (offset % self.SIZEOF_SECTOR) == 0:
                    if msg[19:19+self.SIZEOF_SEPARATOR*2] == 'FF'*self.SIZEOF_SEPARATOR:
                        print('WARNING: Sector header at offset 0x%08X is non-written data' % offset)
                        log.debug('read_memory: Got sector of non-written data at 0x%06x, ending read' % offset)
                        break

                offset += self.SIZEOF_CHUNK

                # update user 'percent read' display
                percent = offset * 100.0 / bytes_to_read
                sys.stdout.write('\rSaved log data: %6.2f%%' % percent)
                sys.stdout.flush()

            self.recv('PMTK001,182,7,3', 10)

        print('')   # terminate user 'percent read' display

        log.debug('%d bytes read (expected %d), len(data)=%d' % (offset+self.SIZEOF_CHUNK, bytes_to_read, len(data)))
        self.memory = data.decode('hex')
        log.debug('self.memory=%s' % data)

        with open('debug.bin', 'wb') as fd:
            fd.write(self.memory)
        with open('debug.asc', 'wb') as fd:
            fd.write(data)
        log.critical("Dumped memory to files 'debug.asc' (%d bytes) and 'debug.bin' (%d bytes)"
                     % (len(data), len(self.memory)))

    def set_memory(self, memory):
        """Set device memory."""

        self.memory = memory

    def send(self, msg):
        checksum = self.calc_checksum(msg)
        msg = '$%s*%02x\r\n' % (msg, checksum)
        try:
            self.serial.write(msg)
        except serial.SerialException:
            log.debug('BTQ1300ST.send: failed')
            return False
        log.debug('BTQ1300ST.send: %s' % msg[:-2])
        return True

    def recv(self, prefix, timeout=Timeout):
        """Receive message with given prefix."""

        max_time = time.time() + timeout

        while True:
            pkt = self.read_pkt(timeout=timeout)
            if pkt.startswith(prefix):
                log.debug('BTQ1300ST.recv: Got desired packet: %s' % prefix)
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
                log.debug("BTQ1300ST.read_pkt: pkt='%s', checksum='%s'"
                          % (str(pkt), checksum))
                if int(checksum, 16) != self.calc_checksum(pkt):
                    log.info('Checksum error on read, got %s expected %s' %
                             (checksum, self.calc_checksum(pkt)))
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
                time.sleep(0.05)

        return pkt

    def calc_checksum(self, msg):
        result = 0
        for ch in msg:
            result ^= ord(ch)
        return result

    @staticmethod
    def find_devices(speed):
        """Find any BT-Q1300ST devices.

        Use the given device speed.  Looks under DefaultDevicePath.
        Return a list of found devices, [] if not found.
        """

        log.debug('find_devices: speed=%d' % speed)
        result = []

        for device in glob.glob(BTQ1300ST.DefaultDevicePath):
            if device == '/dev/tty':
                # don't interrogate the console!
                continue

            if BTQ1300ST.check_device(device, speed):
                result.append(device)

        log.debug('find_device: returning: %s' % str(result))
        return result

    @staticmethod
    def check_device(device, speed):
        """Check given device at given speed."""

        log.debug('check_device: checking device %s at speed %d' % (device, speed))

        gps = None
        try:
            gps = BTQ1300ST(device, speed)
        except serial.SerialException:
            del gps
            return False

        if gps and gps.sane:
            del gps
            return True

        return False

    @staticmethod
    def describe_recording_method(method):
        if method == BTQ1300ST.RCD_METHOD_OVF:
            return 'OVERLAP'
        if method == BTQ1300ST.RCD_METHOD_STP:
            return 'STOP'

    @staticmethod
    def describe_log_format(log_format):
        """Return a string describing the log format."""

        result = []

        if log_format | BTQ1300ST.LOG_FORMAT_UTC: result.append('UTC')
        if log_format | BTQ1300ST.LOG_FORMAT_VALID: result.append('VALID')
        if log_format | BTQ1300ST.LOG_FORMAT_LATITUDE: result.append('LATITUDE')
        if log_format | BTQ1300ST.LOG_FORMAT_LONGITUDE: result.append('LONGITUDE')
        if log_format | BTQ1300ST.LOG_FORMAT_HEIGHT: result.append('HEIGHT')
        if log_format | BTQ1300ST.LOG_FORMAT_SPEED: result.append('SPEED')
        if log_format | BTQ1300ST.LOG_FORMAT_HEADING: result.append('HEADING')
        if log_format | BTQ1300ST.LOG_FORMAT_DSTA: result.append('DSTA')
        if log_format | BTQ1300ST.LOG_FORMAT_DAGE: result.append('DAGE')
        if log_format | BTQ1300ST.LOG_FORMAT_PDOP: result.append('PDOP')
        if log_format | BTQ1300ST.LOG_FORMAT_HDOP: result.append('HDOP')
        if log_format | BTQ1300ST.LOG_FORMAT_VDOP: result.append('VDOP')
        if log_format | BTQ1300ST.LOG_FORMAT_NSAT: result.append('NSAT')
        if log_format | BTQ1300ST.LOG_FORMAT_SID: result.append('SID')
        if log_format | BTQ1300ST.LOG_FORMAT_ELEVATION: result.append('ELEVATION')
        if log_format | BTQ1300ST.LOG_FORMAT_AZIMUTH: result.append('AZIMUTH')
        if log_format | BTQ1300ST.LOG_FORMAT_SNR: result.append('SNR')
        if log_format | BTQ1300ST.LOG_FORMAT_RCR: result.append('RCR')
        if log_format | BTQ1300ST.LOG_FORMAT_MILLISECOND: result.append('MILLISECOND')
        if log_format | BTQ1300ST.LOG_FORMAT_DISTANCE: result.append('DISTANCE')

        return ','.join(result)

    @staticmethod
    def describe_log_status(log_status):
        """Return a string describing the log attributes."""

        result = []

        if log_status & LOG_STATUS_AUTOLOG:
            result.append('AUTOLOG_ON')
        else:
            result.append('AUTOLOG_OFF')

        if log_status & LOG_STATUS_STOP_WHEN_FULL:
            result.append('STOP_WHEN_FULL')
        else:
            result.append('OVERLAP_WHEN_FULL')

        if log_status & LOG_STATUS_ENABLE:
            result.append('ENABLE_LOG')

        if log_status & LOG_STATUS_DISABLE:
            result.append('DISABLE_LOG')

        if log_status & LOG_STATUS_NEED_FORMAT:
            result.append('NEED_LOG')

        if log_status & LOG_STATUS_FULL:
            result.append('FULL')

        return ','.join(result)

    @staticmethod
    def parse_sector_header(sector_header):
        """Parse a log sector header."""

        log.debug('sector_header=%s' % binascii.b2a_hex(sector_header))

        separator = sector_header[-6]
        checksum = sector_header[-5]
        header_tail = binascii.b2a_hex(sector_header[-4:])
        if separator != '*' or header_tail != 'bbbbbbbb':
            log('ERROR: Invalid sector header, see above')
            sys.exit(1)

        offset = 0
        log_count = sector_header[offset:BTQ1300ST.SIZEOF_WORD]
        offset += BTQ1300ST.SIZEOF_WORD
        log_format = sector_header[offset:offset+BTQ1300ST.SIZEOF_LONG]
        offset += BTQ1300ST.SIZEOF_LONG
        log_status = sector_header[offset:offset+BTQ1300ST.SIZEOF_WORD]
        offset += BTQ1300ST.SIZEOF_WORD
        log_period = sector_header[offset:offset+BTQ1300ST.SIZEOF_LONG]
        offset += BTQ1300ST.SIZEOF_LONG
        log_distance = sector_header[offset:offset+BTQ1300ST.SIZEOF_LONG]
        offset += BTQ1300ST.SIZEOF_LONG
        log_speed = sector_header[offset:offset+BTQ1300ST.SIZEOF_LONG]
        offset += BTQ1300ST.SIZEOF_LONG
        log_failsect = sector_header[offset:offset+BTQ1300ST.SIZEOF_BYTE*32]

#        log.debug('log_count=0x%s' % binascii.b2a_hex(log_count))
#        log.debug('log_format=0x%s' % binascii.b2a_hex(log_format))
#        log.debug('log_status=%s' % binascii.b2a_hex(log_status))
#        log.debug('log_period=%s' % binascii.b2a_hex(log_period))
#        log.debug('log_distance=%s' % binascii.b2a_hex(log_distance))
#        log.debug('log_speed=%s' % binascii.b2a_hex(log_speed))
#        log.debug('log_failsect=%s' % binascii.b2a_hex(log_failsect))
#
#        log.debug('len(log_count)=%d' % len(log_count))
#        log.debug('len(log_format)=%d' % len(log_format))

        log_count = BTQ1300ST.unpack(log_count)
        log_format = BTQ1300ST.unpack(log_format)

        log.debug('parse_sector_header: log_count=0x%06x (%d)' % (log_count, log_count))
        log.debug('parse_sector_header: log_format=0x%06x (%d)' % (log_format, log_format))

        return (log_count, log_format)

    def parse_log_data(self, data):
        """Parse log data.

        data  bytearray of log data
        """

        fp = 0
        log_len = len(data)
        record_count_total = 0

        log.debug('parse_log_data: log_len=0x%06x (%d)' % (log_len, log_len))

        while True:
            log.debug('>> Reading offset %08x' % fp)

            log.debug('fp %% BTQ1300ST.SIZEOF_SECTOR=%d' % (fp % BTQ1300ST.SIZEOF_SECTOR))
            if (fp % BTQ1300ST.SIZEOF_SECTOR) == 0:
                # reached the beginning of a log sector (every 0x10000 bytes),
                # get header (0x200 bytes)
                header = data[fp:fp + BTQ1300ST.SIZEOF_SECTOR_HEADER]
                (expected_records_sector, log_format) = BTQ1300ST.parse_sector_header(header)
                log.debug('expected_records_sector=0x%06x' % expected_records_sector)
                log.debug('log_format=0x%06x (%s)' % (log_format, BTQ1300ST.describe_log_format(log_format)))

                record_count_sector = 0

            if record_count_total >= self.expected_records_total:
                log.debug('Total record count: %d' % record_count_total)
                break

            log.debug('???: record_count_sector=%d, expected_records_sector=%d' % (record_count_sector, expected_records_sector))
            if record_count_sector >= expected_records_sector:
                log.debug('Calculating new offset')
                new_offset = self.SIZEOF_SECTOR * (fp/self.SIZEOF_SECTOR + 1)
                if new_offset < log_len:
                    fp = new_offset
                    continue
                else:
                    # end of file
                    log.debug('Total record count: %d' % record_count_total)
                    print('Total record count: %d' % record_count_total)
                break

    @staticmethod
    def unpack(byte_array):
        """Unpack byte array into binary (LSB first)."""

        result = 0
        for val in reversed(byte_array):
            result = result*256 + ord(val)

        return result

    @staticmethod
    def flash_memory_size(model_id):
        """Return flash memory size in MB given model ID string."""

        # 8 Mbit = 1 MB
        # [757/ZI v1, 757/ZI v2]
        if model_id in ['1388', '5202']:
            return (8 * 1024 * 1024 / 8)
        # 16 Mbit = 2 MB
        # [i-Blue 737/Qstarz 810/Polaris iBT-GPS/Holux M1000, Qstarz 815, i-Blue 747, Qstarz BT-Q1000/BGL-32, EB-85A]
        if model_id in ['0051', '0002', '001b', '001d', '0131']:
            return (16 * 1024 * 1024 / 8) 
        # 32 Mbit = 4 MB
        # [Holux M-1200E, Qstarz BT-Q1000P, 747 A+ GPS Trip Recorder, Pentagram PathFinder P 3106, iBlue 747A+, Holux M-1000C, Qstarz BT-1200]
        if model_id in ['0000', '0005', '0006', '0008', '000F', '005C', '8300']:
            return (32 * 1024 * 1024 / 8) 

         # default: 2
        return (16 * 1024 * 1024 / 8) 

#        log.critical('flash_memory_size: Unrecognized ID: %s' % str(model_id))
#        sys.exit(10)


if __name__ == '__main__':
    ######
    # This code is just to exercise the BTQ1300ST class a little.
    # For more paranoid tests, see test_btq1300st.py
    ######

    def main():
        # start the log
        global log
        log = log.Log('btq1300st.log', 10)
    
        # port speeds are sorted lowest to fastest, choose slowest speed
        test_speed = BTQ1300ST.PortSpeeds[0]
    
        # find any BT-Q1300ST devices that are out there
        devices = BTQ1300ST.find_devices(test_speed)
        log.debug('Found devices=%s' % str(devices))
        if len(devices) == 0:
            log.debug('No BT-Q1300ST devices found!?')
            print('No BT-Q1300ST devices found!?')
            return 1
        elif len(devices) == 1:
            device = devices[0]
            max_speed = test_speed
            for speed in BTQ1300ST.PortSpeeds[1:]:
                if not BTQ1300ST.check_device(device, speed):
                    break
                max_speed = speed
            log.debug("Found device '%s', max speed=%s" % (str(device), str(max_speed)))
            print("Found device '%s', max speed=%s" % (str(device), str(max_speed)))
        else:
            log.debug('Found more than one device: %s' % ', '.join(devices))
            print('Found more than one device: %s' % ', '.join(devices))
            return 2
    
        gps = BTQ1300ST(device, max_speed)
        gps.init()
        print('MTK Firmware: Version %s, Release %s, Model ID %s' % (gps.version, gps.release, gps.model_id))
        print('Flash memory size=0x%06x (%d)' % (gps.flash_memory_size(gps.model_id), gps.flash_memory_size(gps.model_id)))
        print('Log format: %s' % gps.describe_log_format(gps.log_format))
        print('Recording method on memory full: %s' % gps.describe_recording_method(gps.rec_method))
        print('Next write address: 0x%04x (%d)' % (gps.next_write_address, gps.next_write_address))
        print('Number of records: %s (%d)' % (gps.expected_records_total, int(gps.expected_records_total, 16)))
        gps.read_memory()
        print('%d bytes of memory read' % len(gps.memory))
#        gps.parse_log_data(gps.memory)

    sys.exit(main())
