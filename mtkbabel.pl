#!/usr/bin/perl
#
# Copyright (C) 2007 - 2011 Niccolo Rigacci
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
# Author:       Niccolo Rigacci <niccolo@rigacci.org>
#
# Version:      0.8.3.1   2011-07-24
#
# Control program for GPS units using the MediaTek (MTK) chipset.
# Tested to work with i-Blue 747 and Holux M-241 GPS data loggers.
#

use strict;
# Use the basename() function.
use File::Basename;
# Use the getopts() function.
use Getopt::Std;
use vars qw($opt_a $opt_b $opt_c $opt_d $opt_E $opt_f $opt_h $opt_i $opt_I $opt_l $opt_m $opt_o $opt_p $opt_R $opt_r $opt_s $opt_t $opt_v $opt_w $opt_x);
# Install the libdevice-serialport-perl Debian package.
use Device::SerialPort;
# Install the libtimedate-perl Debian package.
use Date::Format;

my $NAME = basename($0);

# Debug levels.
my $LOG_EMERG   = 0;
my $LOG_ALERT   = 1;
my $LOG_CRIT    = 2;
my $LOG_ERR     = 3;
my $LOG_WARNING = 4;
my $LOG_NOTICE  = 5;
my $LOG_INFO    = 6;
my $LOG_DEBUG   = 7;

# Size in bytes of data types.
my $SIZEOF_BYTE   = 1;
my $SIZEOF_WORD   = 2;
my $SIZEOF_LONG   = 4;
my $SIZEOF_FLOAT3 = 3;
my $SIZEOF_FLOAT  = 4;
my $SIZEOF_DOUBLE = 8;

# Log format is stored as a bitmask field.
my $LOG_FORMAT_UTC         = 0x00000001;
my $LOG_FORMAT_VALID       = 0x00000002;
my $LOG_FORMAT_LATITUDE    = 0x00000004;
my $LOG_FORMAT_LONGITUDE   = 0x00000008;
my $LOG_FORMAT_HEIGHT      = 0x00000010;
my $LOG_FORMAT_SPEED       = 0x00000020;
my $LOG_FORMAT_HEADING     = 0x00000040;
my $LOG_FORMAT_DSTA        = 0x00000080;
my $LOG_FORMAT_DAGE        = 0x00000100;
my $LOG_FORMAT_PDOP        = 0x00000200;
my $LOG_FORMAT_HDOP        = 0x00000400;
my $LOG_FORMAT_VDOP        = 0x00000800;
my $LOG_FORMAT_NSAT        = 0x00001000;
my $LOG_FORMAT_SID         = 0x00002000;
my $LOG_FORMAT_ELEVATION   = 0x00004000;
my $LOG_FORMAT_AZIMUTH     = 0x00008000;
my $LOG_FORMAT_SNR         = 0x00010000;
my $LOG_FORMAT_RCR         = 0x00020000;
my $LOG_FORMAT_MILLISECOND = 0x00040000;
my $LOG_FORMAT_DISTANCE    = 0x00080000;

# Log status is stored as a bitmask field;
my $LOG_STATUS_AUTOLOG        = 0x0002; # AUTO_LOG mode (by criteria) ON/OFF
my $LOG_STATUS_STOP_WHEN_FULL = 0x0004; # STOP/OVERLAP method when memory full 
my $LOG_STATUS_ENABLE         = 0x0100; # Device is in ENABLE_LOG (normal) state
my $LOG_STATUS_DISABLE        = 0x0200; # Device entered DISABLE_LOG state (fail sectors >= 16)
my $LOG_STATUS_NEED_FORMAT    = 0x0400; # Flash memory need format
my $LOG_STATUS_FULL           = 0x0800; # Flash memory is full

# Default data types.
my $SIZEOF_LOG_UTC         = $SIZEOF_LONG;
my $SIZEOF_LOG_VALID       = $SIZEOF_WORD;
my $SIZEOF_LOG_LATITUDE    = $SIZEOF_DOUBLE;
my $SIZEOF_LOG_LONGITUDE   = $SIZEOF_DOUBLE;
my $SIZEOF_LOG_HEIGHT      = $SIZEOF_FLOAT;
my $SIZEOF_LOG_SPEED       = $SIZEOF_FLOAT;
my $SIZEOF_LOG_HEADING     = $SIZEOF_FLOAT;
my $SIZEOF_LOG_DSTA        = $SIZEOF_WORD;
my $SIZEOF_LOG_DAGE        = $SIZEOF_LONG;
my $SIZEOF_LOG_PDOP        = $SIZEOF_WORD;
my $SIZEOF_LOG_HDOP        = $SIZEOF_WORD;
my $SIZEOF_LOG_VDOP        = $SIZEOF_WORD;
my $SIZEOF_LOG_NSAT        = $SIZEOF_BYTE * 2;
my $SIZEOF_LOG_SID         = $SIZEOF_BYTE;
my $SIZEOF_LOG_SIDINUSE    = $SIZEOF_BYTE;
my $SIZEOF_LOG_SATSINVIEW  = $SIZEOF_WORD;
my $SIZEOF_LOG_ELEVATION   = $SIZEOF_WORD;
my $SIZEOF_LOG_AZIMUTH     = $SIZEOF_WORD;
my $SIZEOF_LOG_SNR         = $SIZEOF_WORD;
my $SIZEOF_LOG_RCR         = $SIZEOF_WORD;
my $SIZEOF_LOG_MILLISECOND = $SIZEOF_WORD;
my $SIZEOF_LOG_DISTANCE    = $SIZEOF_DOUBLE;

# A record separator has one of the following types.
my $SEP_TYPE_CHANGE_LOG_BITMASK    = 0x02;
my $SEP_TYPE_CHANGE_LOG_PERIOD     = 0x03;
my $SEP_TYPE_CHANGE_LOG_DISTANCE   = 0x04;
my $SEP_TYPE_CHANGE_LOG_SPEED      = 0x05;
my $SEP_TYPE_CHANGE_OVERLAP_STOP   = 0x06;
my $SEP_TYPE_CHANGE_START_STOP_LOG = 0x07;

# Values for the VALID field.
my $VALID_NOFIX     = 0x0001;
my $VALID_SPS       = 0x0002;
my $VALID_DGPS      = 0x0004;
my $VALID_PPS       = 0x0008;
my $VALID_RTK       = 0x0010;
my $VALID_FRTK      = 0x0020;
my $VALID_ESTIMATED = 0x0040;
my $VALID_MANUAL    = 0x0080;
my $VALID_SIMULATOR = 0x0100;

# Values for the RCR field.
my $RCR_TIME     = 0x01;
my $RCR_SPEED    = 0x02;
my $RCR_DISTANCE = 0x04;
my $RCR_BUTTON   = 0x08;

# Recording method: OVERLAP or STOP.
my $RCD_METHOD_OVP = 1;
my $RCD_METHOD_STP = 2;

# Log data is retrieved in chunks of this size.
my $SIZEOF_CHUNK         = 0x800;
my $SIZEOF_SECTOR        = 0x10000;
my $SIZEOF_SECTOR_HEADER = 0x200;
my $SIZEOF_SEPARATOR     = 0x10;
   
# End Of Line for generate GPX files.
my $GPX_EOL = "\n";

# Default timeout for packet wait (sec).
my $TIMEOUT = 5;
# Timeout waiting for char "$" which is MTK packet start (sec).
my $TIMEOUT_PKT_PREAMBLE = 20;
# Timeout for activity on device port (msec).
my $TIMEOUT_IDLE_PORT = 5000;

# Is there a char "*" separator between data and checksum?
my $LOG_HAS_CHECKSUM_SEPARATOR = 1;

#-------------------------------------------------------------------------
# Global variablee.
#-------------------------------------------------------------------------
my $debug    = $LOG_ERR;         # Default loggin level.
#my $port     = '/dev/ttyUSB0';   # Default communication port.
my $port     = '/dev/ttyACM0';   	# Win7
#my $port = '/dev/tty.usbmodem1410';	# OSX
my $baudrate = 115200;           # Default port speed.

# GPX global values.
my $gpx_trk_minlat =   90.0;
my $gpx_trk_minlon =  180.0;
my $gpx_trk_maxlat =  -90.0;
my $gpx_trk_maxlon = -180.0;
my $gpx_wpt_minlat =   90.0;
my $gpx_wpt_minlon =  180.0;
my $gpx_wpt_maxlat =  -90.0;
my $gpx_wpt_maxlon = -180.0;
my $gpx_trk_number = 0;
my $gpx_wpt_number = 0;

my $device;
my $version;
my $release;
my $model_id;
my $ret;
my $log_format;
my $next_write_address;
my $expected_records_total;
my $log_status;
my $rec_method;
my $fail_sectors;
my $sectors;
my $bytes_to_read;
my $fp;
my $fp_log;
my $offset;
my $non_written_sector_found;
my $next_data_force_waypoint;

# Record values.
my $record_utc;
my $record_valid;
my $record_latitude;
my $record_longitude;
my $record_height;
my $record_speed;
my $record_heading;
my $record_dsta;
my $record_dage;
my $record_pdop;
my $record_hdop;
my $record_vdop;
my $record_nsat_in_use;
my $record_nsat_in_view;
my $record_satdata;
my $record_rcr;
my $record_millisecond;
my $record_distance;

#-------------------------------------------------------------------------
# Get options from command line.
#-------------------------------------------------------------------------
if (! getopts('ab:cd:Ef:hiIl:m:o:p:Rr:s:tvwx') or $opt_h) {
    my $str1 = describe_log_format(0x00fff);
    my $str2 = describe_log_format(0xff000);
    print <<HELP;
Usage: $NAME [options]
Options:
    -a                       Read all the log memory (overlapped data)
    -b filename.bin          Don't read data from a GPS device, but rather read
                             a previously saved .bin file. Ignore -f option
    -c                       Create a GPX file with tracks and waypoints
    -d debug_level           Debug level: 0..7
    -E                       Erase data log memory
    -f filename              Base name for saved files (.bin and .gpx)
    -h                       Print this message and exit
    -i                       Ignore some error conditions, try to extract data
    -I                       Same as -i, but also write bad records in GPX
    -l {on|off}              Turn logging ON/OFF
    -m {stop|overlap}        Set STOP/OVERLAP recording method on memory full
    -o log_format            Enable or disable log fields (FIELD1,-FIELD2,...),
                             available fields:
                             $str1
                             $str2
    -p port                  Communication port, default: $port
    -R                       Recover from disabled log: erase data and reset
                             recording criteria
    -r time:distance:speed   Set logging criteria (zero to disable):
                             every 0.10-9999999.90 seconds, every 0.10-9999999.90
                             meters, over 0.10-9999999.90 km/h
    -s speed                 Serial port speed, default $baudrate baud
    -t                       Create a GPX file with tracks
    -v                       Print MTKBabel version and exit
    -w                       Create a GPX file with waypoints
    -x                       Force reading Holux format

Example:
    Download traks and waypoints from GPS device, creating the
    following files: gpsdata.bin, gpsdata_trk.gpx and gpsdata_wpt.gpx.

    mtkbabel -s 115200 -f gpsdata -t -w

HELP
    exit(1)
}

#-------------------------------------------------------------------------
# Check command line options.
#-------------------------------------------------------------------------
if ($opt_v) { print "\nMTKBabel Version 0.8.3.1\n\n"; exit }
if ($opt_I) { $opt_i = 1; }
$debug    = $opt_d if (defined($opt_d) and ($opt_d >= $LOG_EMERG) and ($opt_d <= $LOG_DEBUG));
$port     = $opt_p if (defined($opt_p));
$baudrate = int($opt_s) if (defined($opt_s));
$opt_f    = substr($opt_f, 0, -4) if (substr($opt_f, -4) eq '.bin');
$opt_b    = substr($opt_b, 0, -4) if (substr($opt_b, -4) eq '.bin');

#-------------------------------------------------------------------------
# Force Holux format. Required to read a binary file because some
# Holux devices does not embed Holux identifier into the data.
#-------------------------------------------------------------------------
if ($opt_x) {
    $model_id = '0000';
    set_data_types($model_id);
}

#-------------------------------------------------------------------------
# Do not open the device, read instead an existing binary log file.
#-------------------------------------------------------------------------
if ($opt_b) {
    if ($opt_t or $opt_w or $opt_c) {
        # Parse binary data and save GPX files.
        $opt_f = $opt_b;
        # Total number of records is unknown: we will exit on End Of File.
        $expected_records_total = 0xffffffff;
        parse_log_data();
    }
    exit;
}

#-------------------------------------------------------------------------
# Initialize the device port.
#-------------------------------------------------------------------------
serial_port_open($port, $baudrate);

# Do not write the log file yet.
undef($fp_log);

# Send test packet (PMTK_TEST).
packet_send('PMTK000');
$ret = packet_wait('PMTK001,0,');
die("MTK Test command FAILED\n") if (!($ret =~ m/PMTK001,0,/));
print "MTK Test OK\n";

# Query firmware version (PMTK_Q_VERSION).
packet_send('PMTK604');
$ret = packet_wait('PMTK001,604,');
if ($ret =~ m/PMTK001,604,([0-9A-Za-z]+)\*/) {
    $version = $1;
}

# Query firmware release (PMTK_Q_RELEASE). Examples:
#
# Holux M-1200E         PMTK705,AXN_1.30-B_1.3_C01,0000,01035-01A,1.0
# Holux GPSport 245     PMTK705,M-core_2.12,0000,GR-245,1.0
# ???                   PMTK705,M-core_1.8,0001
# 747 ???               PMTK705,B-core_1.1,0002,TSI_747CD,1.0
# Qstarz BT-Q1000X      PMTK705,AXN_1.0-B_1.3_C01,0003,QST1000,1.0
# 747 ???               PMTK705,B-core_1.1,0004,TSI747CD,1.0
# Qstarz BT-Q1000P      PMTK705,B-core_1.1,0005,QST1000P,1.0
# 747 A+ Trip Recorder  PMTK705,AXN_1.0-B_1.3_C01,0006
# iBlue 747A+           PMTK705,AXN_1.30-B_1.3_C01,000F,TSI_747A+,1.0
# ???                   PMTK705,M-core_1.85,0008
# i-Blue 747            PMTK705,M-core_1.94,001B
# Holux M-241           PMTK705,B-core_1.1,0021,01017-00C,1.0
# Holux M-241           PMTK705,B-core_1.20,0023,01017-00D,1.0
# Holux M-241 fw 1.13   PMTK705,B-core_1.20,0043,01017-00F,1.0
# Holux M-1000C         PMTK705,AXN_1.30-B_1.3_C01,005C
# Qstarz BT-Q1300
# Qstarz BT-Q1000
#
packet_send('PMTK605');
$ret = packet_wait('PMTK705,');
if ($ret =~ m/PMTK705,([\.0-9A-Za-z_-]+),([0-9A-Za-z]+)[,\*]/) {
    $release  = $1;
    $model_id = $2;
}

printf "MTK Firmware: Version: $version, Release: $release, Model ID: $model_id\n";
if (! $opt_x) {
    set_data_types($model_id);
}

#-------------------------------------------------------------------------
# Erase memory.
#-------------------------------------------------------------------------
if ($opt_E) {
    printf(">> Erasing log memory...\n");
    packet_send('PMTK182,6,1');
    packet_wait('PMTK001,182,6,3', 120);
}

#-------------------------------------------------------------------------
# Recover from disable log: ENABLE LOG and FORMAT LOG ALL.
# Also reset recording criteria.
#-------------------------------------------------------------------------
if ($opt_R) {
    printf(">> Recover from disable log: ENABLE LOG and FORMAT LOG ALL...\n");
    packet_send('PMTK182,10');
    packet_wait('PMTK001,182,16,3', 120);
    packet_send('PMTK182,6,1');
    packet_wait('PMTK001,182,6,3', 120);
}

#-------------------------------------------------------------------------
# Turn ON or OFF data logging.
#-------------------------------------------------------------------------
if ($opt_l eq 'on') {
    printf(">> Switch recording to ON\n");
    # Send PMTK_LOG_ON.
    packet_send('PMTK182,4');
    packet_wait('PMTK001,182,4,3');
}
if ($opt_l eq 'off') {
    printf(">> Switch recording to OFF\n");
    # Send PMTK_LOG_OFF.
    packet_send('PMTK182,5');
    packet_wait('PMTK001,182,5,3');
}

#-------------------------------------------------------------------------
# Set recording criteria: TIME, DISTANCE, SPEED.
#-------------------------------------------------------------------------
if ($opt_r) {
    printf(">> Setting recording criteria: time, distance, speed\n");
    my ($time, $distance, $speed) = split(/:/, $opt_r);
    undef($time)     if ($time     eq '');
    undef($distance) if ($distance eq '');
    undef($speed)    if ($speed    eq '');
    # Expect floating point values.
    $time     = eval(1.0 * $time    ) if (defined($time));
    $distance = eval(1.0 * $distance) if (defined($distance));
    $speed    = eval(1.0 * $speed   ) if (defined($speed));
    if (defined($time) and (($time >= 0.10 and $time <= 9999999.90) or ($time == 0))) {
        packet_send(sprintf('PMTK182,1,3,%u', int($time * 10)));
        packet_wait('PMTK001,182,1,3');
    }
    if (defined($distance) and (($distance >= 0.10 and $distance <= 9999999.90) or ($distance == 0))) {
        packet_send(sprintf('PMTK182,1,4,%u', int($distance * 10)));
        packet_wait('PMTK001,182,1,3');
    }
    if (defined($speed) and (($speed >= 0.10 and $speed <= 9999999.90) or ($speed == 0))) {
        packet_send(sprintf('PMTK182,1,5,%u', int($speed * 10)));
        packet_wait('PMTK001,182,1,3');
    }
}

#-------------------------------------------------------------------------
# Set recording method: OVERLAP or STOP (PMTK_LOG_REC_METHOD).
#-------------------------------------------------------------------------
if ((lc($opt_m) eq 'overlap') or (lc($opt_m) eq 'stop')) {
    if (lc($opt_m) eq 'overlap') {
        printf(">> Setting method OVERLAP on memory full\n");
        packet_send('PMTK182,1,6,1');
    } else {
        printf(">> Setting method STOP on memory full\n");
        packet_send('PMTK182,1,6,2');
    }
    $ret = packet_wait('PMTK001,182,1,');
    if ($ret =~ m/PMTK001,182,1,(\d)/) {
        if ($1 ne '3') {
            printf(">> ERROR: Cannot set recording method\n");
        }
    }
}

#-------------------------------------------------------------------------
# Set log format (PMTK_LOG_SETFORMAT).
#-------------------------------------------------------------------------
if ($opt_o) {
    printf(">> Setting log format\n");
    # Get current log format.
    packet_send('PMTK182,2,2');
    $ret = packet_wait('PMTK182,3,2,');
    packet_wait('PMTK001,182,2,3');
    if ($ret =~ m/PMTK182,3,2,([0-9A-Za-z]+)\*/) {
        $log_format = hex($1);
        $log_format = encode_log_format($log_format, $opt_o);
        packet_send(sprintf('PMTK182,1,2,%08X', $log_format));
        $ret = packet_wait('PMTK001,182,1,3');
    }
}


# Query log format (PMTK_LOG_QUERY).
packet_send('PMTK182,2,2');
$ret = packet_wait('PMTK182,3,2,');
packet_wait('PMTK001,182,2,3');
if ($ret =~ m/PMTK182,3,2,([0-9A-Za-z]+)\*/) {
    $log_format = hex($1);
    my ($size_wpt, $size_sat) = sizeof_log_format($log_format);
    printf("Log format: (%s) %s\n", $1, describe_log_format($log_format));
    printf("Size in bytes of each log record: %u + (%u * sats_in_view)\n", $size_wpt + 2, $size_sat);
}

# Query recording criteria: time, distance, speed (PMTK_LOG_QUERY).
packet_send('PMTK182,2,3');
$ret = packet_wait('PMTK182,3,3,');
if ($ret =~ m/PMTK182,3,3,([0-9]+)\*/) {
    printf("Logging TIME interval:     %6.2f s\n", $1 / 10);
}
packet_send('PMTK182,2,4');
$ret = packet_wait('PMTK182,3,4,');
if ($ret =~ m/PMTK182,3,4,([0-9]+)\*/) {
    printf("Logging DISTANCE interval: %6.2f m\n", $1 / 10);
}
packet_send('PMTK182,2,5');
$ret = packet_wait('PMTK182,3,5,');
if ($ret =~ m/PMTK182,3,5,([0-9]+)\*/) {
    printf("Logging SPEED limit:       %6.2f km/h\n", $1 / 10);
}

# Query recording method when full (OVERLAP/STOP).
packet_send('PMTK182,2,6');
packet_wait('PMTK001,182,2,3');
$ret = packet_wait('PMTK182,3,6,');
if ($ret =~ m/PMTK182,3,6,([0-9]+)\*/) {
    $rec_method = $1;
    printf("Recording method on memory full: (%u) %s\n", $rec_method, describe_recording_method($rec_method));
}

# Query LOG_STATUS (recording ON/OFF, ...).
packet_send('PMTK182,2,7');
packet_wait('PMTK001,182,2,3');
$ret = packet_wait('PMTK182,3,7,');
if ($ret =~ m/PMTK182,3,7,([0-9A-Za-z]+)\*/) {
    $log_status = $1;
    printf("Log status: (%012b) %s\n", $log_status, describe_log_status($log_status));
    if ($log_status & $LOG_STATUS_NEED_FORMAT) {
        printf("WARNING! Log status NEED_FORMAT, log data is not valid!\n");
    }
    if ($log_status & $LOG_STATUS_DISABLE) {
        printf("WARNING! Log status DISABLE_LOG, may too many failed sectors!\n");
    }
}

# Query the RCD_ADDR (data log Next Write Address).
# If device is in STOP mode, this is also the total memory used.
# If it is in OVERLAP mode, there is old data beyond the NWA.
packet_send('PMTK182,2,8');
$ret = packet_wait('PMTK182,3,8,');
packet_wait('PMTK001,182,2,3');
if ($ret =~ m/PMTK182,3,8,([0-9A-Za-z]+)\*/) {
    $next_write_address = hex($1);
    printf("Next write address: %u (0x%08X)\n", $next_write_address, $next_write_address);
}

# Query number of records stored in the log.
packet_send('PMTK182,2,10');
$ret = packet_wait('PMTK182,3,10,');
packet_wait('PMTK001,182,2,3');
if ($ret =~ m/PMTK182,3,10,([0-9A-Za-z]+)\*/) {
    $expected_records_total = hex($1);
    printf("Number of records: %u\n", $expected_records_total);
}

# Query failed sectors (PMTK_RCD_FSECTOR).
packet_send('PMTK182,2,11');
$ret = packet_wait('PMTK182,3,11,');
packet_wait('PMTK001,182,2,3');
if ($ret =~ m/PMTK182,3,11,([0-9A-Za-z]+)\*/) {
    $fail_sectors = $1;
    printf("Memory health status (failed sectors mask): %s\n", $fail_sectors);
}


#-------------------------------------------------------------------------
# Get binary data from the device and save to a file.
#-------------------------------------------------------------------------
if ($opt_f and ($opt_t or $opt_w or $opt_c)) {

    # Compute the memory used by data log, round-up to the entire sector.
    if (($rec_method == $RCD_METHOD_OVP) or $opt_a) {
        # In OVERLAP mode we don't know where data ends; read the entire memory.
        $bytes_to_read = flash_memory_size($model_id);
    } else {
        # In STOP mode read from zero to Next Write Address.
        $sectors  = int($next_write_address / $SIZEOF_SECTOR);
        $sectors += 1 if (($next_write_address % $SIZEOF_SECTOR) != 0);
        $bytes_to_read = $sectors * $SIZEOF_SECTOR;
    }

    printf(">> Retrieving %u (0x%08X) bytes of log data from device...\n", $bytes_to_read, $bytes_to_read);
    open($fp_log, ">${opt_f}.bin") or die("Cannot open file ${opt_f}.bin: $!");
    binmode($fp_log);

    # Avoid reading the entire memory if we find a non-written sector.
    $non_written_sector_found = 0;

    # NOTE: On a slow machine there was some problem getting the entire log data
    # via USB port with a single PMTK_LOG_REQ_DATA request.
    # The GPS device eventually begins to send packets longer than $SIZEOF_CHUNK,
    # apparently with corrupted data (failed checksum).

    # To be safe we iterate requesting $SIZEOF_CHUNK bytes at time.
    for ($offset = 0; $offset < $bytes_to_read; $offset += $SIZEOF_CHUNK) {
        # Request log data (PMTK_LOG_REQ_DATA) from $offset to $bytes_to_read.
        packet_send(sprintf('PMTK182,7,%08X,%08X', $offset, $SIZEOF_CHUNK));
        # Start writing binary data to file and wait the final PMTK_ACK packet.
        packet_wait('PMTK001,182,7,3', 10);
        last if ($non_written_sector_found);
    }
    close($fp_log);
    undef($fp_log);

    # Parse binary data and save GPX files.
    parse_log_data();

}

serial_port_close();

exit;

#-------------------------------------------------------------------------
# Calculate the packet checksum: bitwise XOR of string's bytes.
#-------------------------------------------------------------------------
sub packet_checksum {

    my $pkt   = shift;
    my $len   = length($pkt);
    my $check = 0;
    my $i;

    for ($i = 0; $i < $len; $i++) { $check ^= ord(substr($pkt, $i, 1)); }
    #printf("0x%02X\n", $check);
    return($check);
}

#-------------------------------------------------------------------------
# Send NMEA packet to the device.
#-------------------------------------------------------------------------
sub packet_send {

    my $pkt = shift;
    my $n;

    # Add the checksum to the packet.
    $pkt = $pkt . '*' . sprintf('%02X', packet_checksum($pkt));
    printf("%s TX packet => %s\n", log_time(), $pkt) if ($debug >= $LOG_NOTICE);
    # Add the preamble and <CR><LF>.
    $pkt = '$' . $pkt . "\r\n";

    $n = serial_port_write($pkt);
    printf("Writing %u bytes to device; actually written %u bytes\n", length($pkt), $n) if ($debug >= $LOG_DEBUG);
    die("ERROR: Writing to device: $!") if ($n != length($pkt));
}

#-------------------------------------------------------------------------
# Read a packet from the device.
# Return the packet with PktType, DataField, "*" and Checksum.
#
#   Example: PMTK182,3,8,0004E69C*13
#
# The packet received has a leading Preample and a trailing <CR><LF>,
# example: $PMTK182,3,8,0004E69C*13<CR><LF>
#-------------------------------------------------------------------------
sub packet_read {

    my $timeout = shift;
    my $c;
    my $n;
    my $t;
    my $pkt;
    my $previous_c;
    my $payload;
    my $checksum;

    # Timeout (in milliseconds) for activity on the port.
    $timeout = $TIMEOUT_IDLE_PORT if (!defined($timeout));
    serial_port_set_read_timeout($timeout);

    # Wait packet preamble.
    $c = '';
    $t = time();
    while ($c ne '$' and (time() - $t) < $TIMEOUT_PKT_PREAMBLE) {
        ($n, $c) = serial_port_getch();
        die("ERROR: Reading from device (may be switched OFF): $!") if ($n != 1);
    }
    die("ERROR: Packet preamble not found (wrong serial speed or output from device not flushed)\n") if ($c ne '$');

    # Read until End Of Packet.
    $pkt = '';
    $previous_c = '';
    while (1) {
        ($n, $c) = serial_port_getch();
        die("ERROR: Reading from device (may be switched OFF): $!") if ($n != 1);
        if ($c eq '$') {
            $pkt = '';
        } else {
            $pkt .= $c;
        }
        if (($c eq "\n") and ($previous_c eq "\r")) {
            last;
        }
        $previous_c = $c;
    }

    # Remove trailing <CR><LF>.
    $pkt = substr($pkt, 0, -2);
    printf("%s RX packet <= %s\n", log_time(), $pkt) if ($debug >= $LOG_NOTICE);

    # Extract packet payload and checksum.
    $payload  = substr($pkt,  0, -3);
    $checksum = hex(substr($pkt, -2,  2));

    # Verify packet checksum.
    if ($checksum ne packet_checksum($payload)) {
        printf("Packet checksum error: expected 0x%02X, computed 0x%02X\n", $checksum, packet_checksum($payload)) if ($debug >= $LOG_ERR);
        return('');
    } else {
        return($pkt);
    }
}

#-------------------------------------------------------------------------
# Read packets from the device, untill we get the one we want.
#-------------------------------------------------------------------------
sub packet_wait {

    my $pkt_type = shift;
    my $timeout  = shift;
    my $max_time;
    my $pkt;
    my $len;
    my $i;

    $len = length($pkt_type);

    # Timeout (in seconds) for packet wait.
    $timeout = $TIMEOUT if (!defined($timeout));
    $max_time = time() + $timeout;

    while(1) {
        $pkt = packet_read($timeout * 1000);
        return($pkt) if (substr($pkt, 0, $len) eq $pkt_type);
        write_log_packet($pkt) if (defined($fp_log));
        last if (time() > $max_time);
    }
    printf("%s ERROR: packet_wait() failed for packet %s\n", log_time(), $pkt_type) if ($debug >= $LOG_ERR);
    return(undef);
}

#-------------------------------------------------------------------------
# Append received packets of log data to a file.
#
# Packet format is PMTK182,8,SSSSSSSS,PacketData*CC where:
#   SSSSSSSS   = Offset of first byte (hex value, 8 chars)
#   PacketData = Packet data (hex values, 4096 chars)
#   *          = Separator (1 char)
#   CC         = Checksum of data (hex value, 2 chars)
#-------------------------------------------------------------------------
sub write_log_packet {

    my $pkt = shift;
    my $pkt_len;
    my $pkt_offset;
    my $log_offset;
    my $percent;
    my $i;

    # Save only datalog packets (PMTK_LOG_RESP_DATA).
    return if (substr($pkt, 0, 10) ne 'PMTK182,8,');

    # Check if packet length is OK and if chunk is in sequence order.
    $log_offset = tell($fp_log);
    $pkt_offset = hex(substr($pkt, 10, 8));
    $pkt_len    = ($SIZEOF_CHUNK * 2) + 22;

    # Check sector headers for non-written pattern, set a flag to avoid reading unused memory.
    if (($log_offset % $SIZEOF_SECTOR) == 0) {
        if (uc(substr($pkt, 19, $SIZEOF_SEPARATOR * 2)) eq ('FF' x $SIZEOF_SEPARATOR)) {
            printf("WARNING: Sector header at offset 0x%08X is non-written data\n", $log_offset);
            $non_written_sector_found = 1;
        }
    }

    # We request $pkt_len at time, check how much we got.
    if (length($pkt) != $pkt_len) {
        printf("WARNING: Packet size error: expected %04X, got %04X\n", $pkt_len, length($pkt)) if ($debug >= $LOG_WARNING);
    }

    if ($pkt_offset != $log_offset) {
        printf("ERROR: Chunk out of sequence: expected %08X, got %08X\n", $log_offset, $pkt_offset) if ($debug >= $LOG_ERR);
    } else {
        printf("Saving log data, offset: 0x%08X\n", $log_offset) if ($debug >= $LOG_INFO);
        # Convert the string of hex values into binary data.
        for ($i = 19; $i < ($pkt_len - 3); $i += 2) {
            printf $fp_log chr(hex(substr($pkt, $i, 2)));
        }
        $percent = ($log_offset / $bytes_to_read) * 100;
        printf("Saved log data: %6.2f%%\n", $percent);
    }
}

#-------------------------------------------------------------------------
# Parse raw binary log data and write GPX files.
#-------------------------------------------------------------------------
sub parse_log_data {

    my $i;
    my $fp;
    my $fp_gpx;
    my $fp_gpx_trk;
    my $fp_gpx_wpt;
    my $buffer;
    my $log_len;
    my $new_offset;
    my $log_format;
    my $checksum;
    my $trailing_spaces;
    my $sep_model;

    my $expected_records_sector  = 0;
    my $record_count_sector      = 0;
    my $record_count_total       = 0;
    my $gpx_in_trk               = 0;
    my $next_data_force_waypoint = 0;

    my $gpx_fname         = "${opt_f}.gpx";
    my $gpx_trk_fname     = "${opt_f}_trk.gpx";
    my $gpx_wpt_fname     = "${opt_f}_wpt.gpx";
    my $gpx_trk_tmp_fname = "${opt_f}_trk.gpx.$$.tmp";
    my $gpx_wpt_tmp_fname = "${opt_f}_wpt.gpx.$$.tmp";

    # Open the binary log file for reading.
    open($fp, "${opt_f}.bin") or die("ERROR reading ${opt_f}.bin: $!");
    binmode($fp);
    seek($fp, 0, 2) or die;
    $log_len = tell($fp);
    seek($fp, 0, 0) or die;

    # Open GPX temporary files for writing tracks and waypoints.
    open($fp_gpx_trk, ">$gpx_trk_tmp_fname") or die("ERROR writing $gpx_trk_tmp_fname: $!") if ($opt_t or $opt_c);
    open($fp_gpx_wpt, ">$gpx_wpt_tmp_fname") or die("ERROR writing $gpx_wpt_tmp_fname: $!") if ($opt_w or $opt_c);

    while (1) {

        # Print current file position.
        printf("Reading offset %08X\n", tell($fp)) if ($debug >= $LOG_INFO);

        #-----------------------------------------
        # Process the begin of a sector (header).
        #-----------------------------------------
        if ((tell($fp) % $SIZEOF_SECTOR) == 0) {
            # Reached the begin of a log sector (every 0x10000 bytes). Get the header (0x200 bytes).
            $buffer = my_read($fp, $SIZEOF_SECTOR_HEADER);
            # What we will find in this log sector:
            ($expected_records_sector, $log_format) = parse_sector_header($buffer);
            last if (!defined($expected_records_sector) or !defined($log_format));
            $record_count_sector = 0;
        }

        #-----------------------------------------
        # Check if all the records has been read.
        #-----------------------------------------
        if ($record_count_total >= $expected_records_total) {
            printf("Total record count: %u\n", $record_count_total);
            last;
        }
        if ($record_count_sector >= $expected_records_sector) {
            $new_offset = $SIZEOF_SECTOR * (int(tell($fp) / $SIZEOF_SECTOR) + 1);
            if ($new_offset < $log_len) {
                # Log file contains more data (that is old data, being overwritten).
                seek($fp, $new_offset, 0);
                next;
            } else {
                # End Of File.
                if (!defined($opt_b)) {
                    printf("ERROR: End of log file! Total record count: %u, expected %u\n", $record_count_total, $expected_records_total);
                } else {
                    printf("Total record count: %u\n", $record_count_total);
                }
                last;
            } 
        }

        #------------------------------------------------------------------
        # Check for:
        # - record separator:          "AAAAAAAAAAAAAAXXYYYYYYYYBBBBBBBB"
        # - non written space:         "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
        # - Holux M-241 separators:    "HOLUXGR241LOGGER"
        #                              "HOLUXGR241WAYPNT"
        # - Holux M-241 fw 1.13 sep.:  "HOLUXGR241LOGGER    "
        #                              "HOLUXGR241WAYPNT    "
        # - Holux Holux M-1200E sep.:  "HOLUXM1200WAYPNT    "
        #
        # - Holux GPSport GR-245 sep.: "HOLUXGR245LOGGER    "
        #                              "HOLUXGR245WAYPNT    "
        #------------------------------------------------------------------
        if (($log_len - tell($fp)) >= $SIZEOF_SEPARATOR) {

            $buffer = my_read($fp, $SIZEOF_SEPARATOR);

            if ((substr($buffer, 0, 7) eq (chr(0xaa) x 7)) and (substr($buffer, -4) eq (chr(0xbb) x 4))) {
                #----------------------------------------------------------
                # Found a record separator.
                #----------------------------------------------------------
                # Close the current <trk> in GPX.
                if ($gpx_in_trk) {
                    gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
                    $gpx_in_trk = 0;
                }
                my $separator_type = ord(substr($buffer, 7, $SIZEOF_BYTE));
                my $separator_arg  = mtk2long(substr($buffer, 8, $SIZEOF_LONG));
                printf("Separator: %s, type: %s\n", uc(unpack('H*', $buffer)), describe_separator_type($separator_type)) if ($debug >= $LOG_INFO);
                if ($separator_type == $SEP_TYPE_CHANGE_LOG_BITMASK) {
                    $log_format = $separator_arg;
                    printf("New log bitmask: %s (0x%08X = %s)\n", $separator_arg, $log_format, describe_log_format($log_format)) if ($debug >= $LOG_INFO);
                }
                next; # Search for the next record or record separator.

            } elsif (substr($buffer, 0, 5) eq 'HOLUX') {
                #----------------------------------------------------------
                # Found Holux separator.
                #----------------------------------------------------------
                printf("Separator: %s\n", $buffer) if ($debug >= $LOG_INFO);

                # Sarch for trailig four spaces after the separator.
                $trailing_spaces = 0;
                if (($log_len - tell($fp)) >= 4) {
                    if (my_read($fp, 4) eq '    ') {
                        $trailing_spaces = 1;
                    } else {
                        seek($fp, -4, 1);
                    }
                }

                # Define $model_id upon separator string.
                $sep_model = substr($buffer, 5,  5);
                if ($sep_model eq 'GR241' and $trailing_spaces) {
                    $model_id = '0043'; # M-241 fw 1.13
                } elsif ($sep_model eq 'GR241') {
                    $model_id = '0021'; # M-241
                } elsif ($sep_model eq 'GR245' or $sep_model eq 'M1200') {
                    $model_id = '0000'; # M-1200E or GPSport 245
                } else {
                    $model_id = '0021'; # Unknown Holux model, assume M-241
                }
                set_data_types($model_id);

                # Check if the following data is a waypoint.
                if (substr($buffer, 10, 6) eq 'WAYPNT') {
                    $next_data_force_waypoint = 1;
                }
                next;

            } elsif ($buffer eq (chr(0xff) x $SIZEOF_SEPARATOR)) {
                #----------------------------------------------------------
                # Found non-written space.
                #----------------------------------------------------------
                # Close the current <trk> in GPX.
                if ($gpx_in_trk) {
                    gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
                    $gpx_in_trk = 0;
                }
                if ($expected_records_sector == 0xffff) {
                    # Sector record count = 0xffff means this is the currently writing sector.
                    # We found empty space, so we skip to sector end.
                    $new_offset = $SIZEOF_SECTOR * (int(tell($fp) / $SIZEOF_SECTOR) + 1);
                    if ($new_offset < $log_len) {
                        # Log file contains more data (that is old data, being overwritten).
                        seek($fp, $new_offset, 0);
                        next; # Search for the next record or record separator.
                    } else {
                        # End Of File.
                        if (!defined($opt_b)) {
                            printf("ERROR: End of log file! Total record count: %u, expected %u\n", $record_count_total, $expected_records_total);
                        } else {
                            printf("Total record count: %u\n", $record_count_total); 
                        }
                        last;
                    } 
                } else {
                    # ERROR! Non written space, but this is not the writing sector.
                    printf("ERROR: Non written space! Read %u records, expected %u\n", $record_count_sector, $expected_records_sector);
                    next if ($opt_i);
                    last;
                }

            } else {
                # None of above, should be record data: rewind the file pointer so we can read it.
                seek($fp, -$SIZEOF_SEPARATOR, 1);
            }
        }

        #-----------------------------------------
        # Read a log record.
        #-----------------------------------------
        $record_count_sector++;
        $record_count_total++;
        $checksum = 0;
        printf("Reading log sector: record %u (%u/%u total)\n", $record_count_sector, $record_count_total, $expected_records_total) if ($debug >= $LOG_INFO);

        # Read each record field.
        undef($record_utc);
        if ($log_format & $LOG_FORMAT_UTC) {
            $buffer = my_read($fp, $SIZEOF_LOG_UTC);
            $checksum ^= packet_checksum($buffer);
            $record_utc = utc_time(mtk2long($buffer));
            printf("Record UTC: %s %s\n", uc(unpack('H*', $buffer)), $record_utc) if ($debug >= $LOG_DEBUG);
        }

        undef($record_valid);
        if ($log_format & $LOG_FORMAT_VALID) {
            $buffer = my_read($fp, $SIZEOF_LOG_VALID);
            $checksum ^= packet_checksum($buffer);
            $record_valid = mtk2unsignedword($buffer);
            printf("Record VALID: %s (0x%04X = %s)\n", uc(unpack('H*', $buffer)), $record_valid, describe_valid_mtk($record_valid)) if ($debug >= $LOG_DEBUG);
         }

        undef($record_latitude);
        if ($log_format & $LOG_FORMAT_LATITUDE) {
            $buffer = my_read($fp, $SIZEOF_LOG_LATITUDE);
            $checksum ^= packet_checksum($buffer);
            $record_latitude = mtk2number($buffer, $SIZEOF_LOG_LATITUDE);
            printf("Record LATITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_latitude) if ($debug >= $LOG_DEBUG);
        }

        undef($record_longitude);
        if ($log_format & $LOG_FORMAT_LONGITUDE) {
            $buffer = my_read($fp, $SIZEOF_LOG_LONGITUDE);
            $checksum ^= packet_checksum($buffer);
            $record_longitude = mtk2number($buffer, $SIZEOF_LOG_LONGITUDE);
            printf("Record LONGITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_longitude) if ($debug >= $LOG_DEBUG);
        }

        undef($record_height);
        if ($log_format & $LOG_FORMAT_HEIGHT) {
            $buffer = my_read($fp, $SIZEOF_LOG_HEIGHT);
            $checksum ^= packet_checksum($buffer);
            $record_height = mtk2number($buffer, $SIZEOF_LOG_HEIGHT);
            printf("Record HEIGHT: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_height) if ($debug >= $LOG_DEBUG);
        }

        undef($record_speed);
        if ($log_format & $LOG_FORMAT_SPEED) {
            $buffer = my_read($fp, $SIZEOF_LOG_SPEED);
            $checksum ^= packet_checksum($buffer);
            $record_speed = mtk2number($buffer, $SIZEOF_LOG_SPEED);
            printf("Record SPEED: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_speed) if ($debug >= $LOG_DEBUG);
        }

        undef($record_heading);
        if ($log_format & $LOG_FORMAT_HEADING) {
            $buffer = my_read($fp, $SIZEOF_LOG_HEADING);
            $checksum ^= packet_checksum($buffer);
            $record_heading = mtk2number($buffer, $SIZEOF_LOG_HEADING);
            printf("Record HEADING: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_heading) if ($debug >= $LOG_DEBUG);
        }

        undef($record_dsta);
        if ($log_format & $LOG_FORMAT_DSTA) {
            $buffer = my_read($fp, $SIZEOF_LOG_DSTA);
            $checksum ^= packet_checksum($buffer);
            $record_dsta = mtk2unsignedword($buffer);
            printf("Record DSTA: %s (%u)\n", uc(unpack('H*', $buffer)), $record_dsta) if ($debug >= $LOG_DEBUG);
        }

        undef($record_dage);
        if ($log_format & $LOG_FORMAT_DAGE) {
            $buffer = my_read($fp, $SIZEOF_LOG_DAGE);
            $checksum ^= packet_checksum($buffer);
            $record_dage = mtk2long($buffer);
            printf("Record DAGE: %s (%u)\n", uc(unpack('H*', $buffer)), $record_dage) if ($debug >= $LOG_DEBUG);
        }

        undef($record_pdop);
        if ($log_format & $LOG_FORMAT_PDOP) {
            $buffer = my_read($fp, $SIZEOF_LOG_PDOP);
            $checksum ^= packet_checksum($buffer);
            $record_pdop = mtk2unsignedword($buffer) / 100;
            printf("Record PDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_pdop) if ($debug >= $LOG_DEBUG);
        }

        undef($record_hdop);
        if ($log_format & $LOG_FORMAT_HDOP) {
            $buffer = my_read($fp, $SIZEOF_LOG_HDOP);
            $checksum ^= packet_checksum($buffer);
            $record_hdop = mtk2unsignedword($buffer) / 100;
            printf("Record HDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_hdop) if ($debug >= $LOG_DEBUG);
        }

        undef($record_vdop);
        if ($log_format & $LOG_FORMAT_VDOP) {
            $buffer = my_read($fp, $SIZEOF_LOG_VDOP);
            $checksum ^= packet_checksum($buffer);
            $record_vdop = mtk2unsignedword($buffer) / 100;
            printf("Record VDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_vdop) if ($debug >= $LOG_DEBUG);
        }

        undef($record_nsat_in_use);
        undef($record_nsat_in_view);
        if ($log_format & $LOG_FORMAT_NSAT) {
            $buffer = my_read($fp, $SIZEOF_BYTE);
            $checksum ^= packet_checksum($buffer);
            $record_nsat_in_view = mtk2byte($buffer);
            $buffer = my_read($fp, $SIZEOF_BYTE);
            $checksum ^= packet_checksum($buffer);
            $record_nsat_in_use = mtk2byte($buffer);
            printf("Record NSAT: in view %u, in use %u\n", $record_nsat_in_view, $record_nsat_in_use) if ($debug >= $LOG_DEBUG);
        }

        undef($record_satdata);
        if ($log_format & $LOG_FORMAT_SID) {
            my $satdata_count = 0;
            my $satdata_sid;
            my $satdata_inuse;
            my $satdata_inview;
            my $satdata_elevation;
            my $satdata_azimuth;
            my $satdata_snr;
            while (1) {
                # Even with zero satellites in view, we have at least one bunch of data.
                $buffer = my_read($fp, $SIZEOF_LOG_SID);
                $checksum ^= packet_checksum($buffer);
                $satdata_sid = mtk2byte($buffer);
                $buffer = my_read($fp, $SIZEOF_LOG_SIDINUSE);
                $checksum ^= packet_checksum($buffer);
                $satdata_inuse = mtk2byte($buffer);
                $buffer = my_read($fp, $SIZEOF_LOG_SATSINVIEW);
                $checksum ^= packet_checksum($buffer);
                $satdata_inview = mtk2unsignedword($buffer);
                if ($satdata_inview == 0) {
                    printf("No satellites in view\n") if ($debug >= $LOG_DEBUG);
                } else {
                    # Read data for this satellite.
                    printf("Sats in view: %u, SatID %u in use: %u\n", $satdata_inview, $satdata_sid, $satdata_inuse) if ($debug >= $LOG_DEBUG);
                    undef($satdata_elevation);
                    if ($log_format & $LOG_FORMAT_ELEVATION) {
                        $buffer = my_read($fp, $SIZEOF_LOG_ELEVATION);
                        $checksum ^= packet_checksum($buffer);
                        $satdata_elevation = mtk2signedword($buffer);
                        printf("Satellite ELEVATION: %s (%d)\n", uc(unpack('H*', $buffer)), $satdata_elevation) if ($debug >= $LOG_DEBUG);
                    }
                    undef($satdata_azimuth);
                    if ($log_format & $LOG_FORMAT_AZIMUTH) {
                        $buffer = my_read($fp, $SIZEOF_LOG_AZIMUTH);
                        $checksum ^= packet_checksum($buffer);
                        $satdata_azimuth = mtk2unsignedword($buffer);
                        printf("Satellite AZIMUTH:   %s (%u)\n", uc(unpack('H*', $buffer)), $satdata_azimuth) if ($debug >= $LOG_DEBUG);
                    }
                    undef($satdata_snr);
                    if ($log_format & $LOG_FORMAT_SNR) {
                        $buffer = my_read($fp, $SIZEOF_LOG_SNR);
                        $checksum ^= packet_checksum($buffer);
                        $satdata_snr = mtk2unsignedword($buffer);
                        printf("Satellite SNR:       %s (%u)\n", uc(unpack('H*', $buffer)), $satdata_snr) if ($debug >= $LOG_DEBUG);
                    }
                    $record_satdata .= "\n" if ($record_satdata ne '');
                    $record_satdata .= $satdata_sid       . "\t";
                    $record_satdata .= $satdata_inuse     . "\t";
                    $record_satdata .= $satdata_elevation . "\t";
                    $record_satdata .= $satdata_azimuth   . "\t";
                    $record_satdata .= $satdata_snr;
                    $satdata_count++;
                }
                last if ($satdata_count >= $satdata_inview);
            }
        }

        undef($record_rcr);
        if ($log_format & $LOG_FORMAT_RCR) {
            $buffer = my_read($fp, $SIZEOF_LOG_RCR);
            $checksum ^= packet_checksum($buffer);
            $record_rcr = mtk2unsignedword($buffer);
            printf("Record RCR: %s (%s)\n", uc(unpack('H*', $buffer)), describe_rcr_mtk($record_rcr)) if ($debug >= $LOG_DEBUG);
        }

        undef($record_millisecond);
        if ($log_format & $LOG_FORMAT_MILLISECOND) {
            $buffer = my_read($fp, $SIZEOF_LOG_MILLISECOND);
            $checksum ^= packet_checksum($buffer);
            $record_millisecond = mtk2unsignedword($buffer);
            printf("Record MILLISECOND: %s (%u)\n", uc(unpack('H*', $buffer)), $record_millisecond) if ($debug >= $LOG_DEBUG);
        }

        undef($record_distance);
        if ($log_format & $LOG_FORMAT_DISTANCE) {
            $buffer = my_read($fp, $SIZEOF_LOG_DISTANCE);
            $checksum ^= packet_checksum($buffer);
            $record_distance = mtk2number($buffer, $SIZEOF_LOG_DISTANCE);
            printf("Record DISTANCE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_distance) if ($debug >= $LOG_DEBUG);
        }

        if ($LOG_HAS_CHECKSUM_SEPARATOR) {
            # Read separator between data and checksum.
            $buffer = my_read($fp, $SIZEOF_BYTE);
            if ($buffer ne '*') {
                printf("ERROR: Checksum separator error: expected char 0x%02X, found 0x%02X\n", ord('*'), ord($buffer));
                last if (! $opt_i);
                next if (! $opt_I);
            }
        }
        # Read and verify checksum.
        $buffer = my_read($fp, $SIZEOF_BYTE);
        if ($checksum != ord($buffer)) {
            printf("ERROR: Record checksum error: expected 0x%02X, computed 0x%02X\n", ord($buffer), $checksum);
            last if (! $opt_i);
            next if (! $opt_I);
        }

        # Start a new GPX <trkseg> on satellite lost.
        if (($record_valid == $VALID_NOFIX) and $gpx_in_trk) {
            gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
            $gpx_in_trk = 0;
        }

        if (defined($record_latitude) and defined($record_longitude)) {
            if ($next_data_force_waypoint) {
                # Write <wpt> data in GPX file.
                gpx_print_wpt($fp_gpx_wpt) if ($opt_w or $opt_c);
                $next_data_force_waypoint = 0;
            } else {
                # Write <trkpt> data in GPX file.
                if (($record_valid != $VALID_NOFIX) and !($record_rcr & $RCR_BUTTON)) {
                    if (! $gpx_in_trk) {
                        gpx_print_trk_begin($fp_gpx_trk) if ($opt_t or $opt_c);
                        $gpx_in_trk = 1;
                    }
                    gpx_print_trkpt($fp_gpx_trk) if ($opt_t or $opt_c);
                }
                # Write <wpt> data in GPX file.
                if (($record_rcr & $RCR_BUTTON) and ($record_valid != $VALID_NOFIX)) {
                    gpx_print_wpt($fp_gpx_wpt) if ($opt_w or $opt_c);
                }
            }
        }

    }
    close($fp);

    # Eventually close the <trk> GPX tags.
    if ($gpx_in_trk) {
        gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
        $gpx_in_trk = 0;
    }

    # Close temporary files.
    close($fp_gpx_trk) if ($opt_t or $opt_c);
    close($fp_gpx_wpt) if ($opt_w or $opt_c);

    # Write GPX combined tracks and waypoint file.
    if ($opt_c) {
        my $gpx_minlat = $gpx_trk_minlat < $gpx_wpt_minlat ? $gpx_trk_minlat : $gpx_wpt_minlat;
        my $gpx_maxlat = $gpx_trk_maxlat > $gpx_wpt_maxlat ? $gpx_trk_maxlat : $gpx_wpt_maxlat;
        my $gpx_minlon = $gpx_trk_minlon < $gpx_wpt_minlon ? $gpx_trk_minlon : $gpx_wpt_minlon;
        my $gpx_maxlon = $gpx_trk_maxlon > $gpx_wpt_maxlon ? $gpx_trk_maxlon : $gpx_wpt_maxlon;
        open($fp_gpx, ">$gpx_fname") or die("ERROR writing $gpx_fname: $!");
        gpx_print_gpx_begin($fp_gpx, time(), $gpx_minlat, $gpx_minlon, $gpx_maxlat, $gpx_maxlon);
        open($fp_gpx_trk, "$gpx_trk_tmp_fname") or die;
        while (<$fp_gpx_trk>) { print $fp_gpx $_; }
        close($fp_gpx_trk);
        open($fp_gpx_wpt, "$gpx_wpt_tmp_fname") or die;
        while (<$fp_gpx_wpt>) { print $fp_gpx $_; }
        close($fp_gpx_wpt);
        gpx_print_gpx_end($fp_gpx);
        close($fp_gpx);
    }

    # Write GPX tracks file.
    if ($opt_t) {
        open($fp_gpx, ">$gpx_trk_fname") or die("ERROR writing $gpx_trk_fname: $!");
        gpx_print_gpx_begin($fp_gpx, time(), $gpx_trk_minlat, $gpx_trk_minlon, $gpx_trk_maxlat, $gpx_trk_maxlon);
        open($fp_gpx_trk, "$gpx_trk_tmp_fname") or die;
        while (<$fp_gpx_trk>) { print $fp_gpx $_; }
        close($fp_gpx_trk);
        gpx_print_gpx_end($fp_gpx);
        close($fp_gpx);
    }

    # Write GPX waypoints file.
    if ($opt_w) {
        open($fp_gpx, ">$gpx_wpt_fname") or die("ERROR writing $gpx_wpt_fname: $!");
        gpx_print_gpx_begin($fp_gpx, time(), $gpx_wpt_minlat, $gpx_wpt_minlon, $gpx_wpt_maxlat, $gpx_wpt_maxlon);
        open($fp_gpx_wpt, "$gpx_wpt_tmp_fname") or die;
        while (<$fp_gpx_wpt>) { print $fp_gpx $_; }
        close($fp_gpx_wpt);
        gpx_print_gpx_end($fp_gpx);
        close($fp_gpx);
    }

    # Remove temporary files.
    unlink($gpx_trk_tmp_fname) if ($opt_t or $opt_c);
    unlink($gpx_wpt_tmp_fname) if ($opt_w or $opt_c);

}

#-------------------------------------------------------------------------
# Read some bytes from the device and return them.
#-------------------------------------------------------------------------
sub my_read {
    my $handle   = shift;
    my $length   = shift;
    my $variable;
    my $n = read($handle, $variable, $length);
    printf("ERROR: Reading file: read %u bytes, expected %u\n", $n, $length) if ($n != $length);
    return($variable);
}

#-------------------------------------------------------------------------
# Parse the header (0x200 bytes) of a datalog sector (every 0x10000 bytes).
# Return the number of records in the sector and the log format.
#-------------------------------------------------------------------------
sub parse_sector_header {

    my $sector_header = shift;

    if ($debug >= $LOG_NOTICE) {
        printf("\n");
        printf("Sector header:      %s\n", uc(unpack('H*', $sector_header))) if ($debug >= $LOG_DEBUG);
    }

    # Check validity of sector header.
    my $separator   =     substr($sector_header, -6, 1);              # Should be '*'
    my $checksum    = ord(substr($sector_header, -5, $SIZEOF_BYTE));  # WARNING: It's not the XOR checksum!!!
    my $header_tail =     substr($sector_header, -4, 4);              # Should be 0xBBBBBBBB
    if (($separator ne '*') or ($header_tail ne (chr(0xBB) x 4))) {
        printf("ERROR: Invalid datalog sector header\n");
        return(undef, undef);
    }

    # Settings of this log sector (hex values, LSB first).
    my $log_count    = substr($sector_header,  0, $SIZEOF_WORD);      # Record count in this sector. 0xFFFF if the sector is not filled.
    my $log_format   = substr($sector_header,  2, $SIZEOF_LONG);      # Log format bitmask
    my $log_status   = substr($sector_header,  6, $SIZEOF_WORD);      # Log mode bitmask
    my $log_period   = substr($sector_header,  8, $SIZEOF_LONG);      # Log period   in 10ths of s
    my $log_distance = substr($sector_header, 12, $SIZEOF_LONG);      # Log distance in 10ths of m
    my $log_speed    = substr($sector_header, 16, $SIZEOF_LONG);      # Log speed    in 10ths ok km/h
    my $log_failsect = substr($sector_header, 20, $SIZEOF_BYTE * 32); # Failed sectors bitmask

    my $count    = unpack('S', $log_count);
    my $format   = unpack('L', $log_format);
    my $status   = unpack('S', $log_status);
    my $period   = unpack('L', $log_period);
    my $distance = unpack('L', $log_distance);
    my $speed    = unpack('L', $log_speed);

    if ($debug >= $LOG_NOTICE) {
        printf("Record count:        %s %s %u records\n",  uc(unpack('H*', $log_count)),    ' 'x6, $count);
        printf("Log format mask:     %s %s %032bb (%s)\n", uc(unpack('H*', $log_format)),   ' 'x2, $format, describe_log_format($format));
        printf("Log mode mask:       %s %s %016bb (%s)\n", uc(unpack('H*', $log_status)),   ' 'x6, $status, describe_log_status($status));
        printf("Log period:          %s %s %6.2f s\n",     uc(unpack('H*', $log_period)),   ' 'x2, $period   / 10);
        printf("Log distance:        %s %s %6.2f m\n",     uc(unpack('H*', $log_distance)), ' 'x2, $distance / 10);
        printf("Log speed:           %s %s %6.2f km/h\n",  uc(unpack('H*', $log_speed)),    ' 'x2, $speed    / 10);
        printf("Failed sectors mask: %s\n",                uc(unpack('H*', $log_failsect)));
        printf("\n");
    }

    return($count, $format);
}

#-------------------------------------------------------------------------
# Given a log format value (bitmask), return a description string.
#-------------------------------------------------------------------------
sub describe_log_format {

    my $log_format = shift;
    my $str = '';

    $str .= ',UTC'         if ($log_format & $LOG_FORMAT_UTC);
    $str .= ',VALID'       if ($log_format & $LOG_FORMAT_VALID);
    $str .= ',LATITUDE'    if ($log_format & $LOG_FORMAT_LATITUDE);
    $str .= ',LONGITUDE'   if ($log_format & $LOG_FORMAT_LONGITUDE);
    $str .= ',HEIGHT'      if ($log_format & $LOG_FORMAT_HEIGHT);
    $str .= ',SPEED'       if ($log_format & $LOG_FORMAT_SPEED);
    $str .= ',HEADING'     if ($log_format & $LOG_FORMAT_HEADING);
    $str .= ',DSTA'        if ($log_format & $LOG_FORMAT_DSTA);
    $str .= ',DAGE'        if ($log_format & $LOG_FORMAT_DAGE);
    $str .= ',PDOP'        if ($log_format & $LOG_FORMAT_PDOP);
    $str .= ',HDOP'        if ($log_format & $LOG_FORMAT_HDOP);
    $str .= ',VDOP'        if ($log_format & $LOG_FORMAT_VDOP);
    $str .= ',NSAT'        if ($log_format & $LOG_FORMAT_NSAT);
    $str .= ',SID'         if ($log_format & $LOG_FORMAT_SID);
    $str .= ',ELEVATION'   if ($log_format & $LOG_FORMAT_ELEVATION);
    $str .= ',AZIMUTH'     if ($log_format & $LOG_FORMAT_AZIMUTH);
    $str .= ',SNR'         if ($log_format & $LOG_FORMAT_SNR);
    $str .= ',RCR'         if ($log_format & $LOG_FORMAT_RCR);
    $str .= ',MILLISECOND' if ($log_format & $LOG_FORMAT_MILLISECOND);
    $str .= ',DISTANCE'    if ($log_format & $LOG_FORMAT_DISTANCE);

    return(substr($str, 1));
}

#-------------------------------------------------------------------------
#
#-------------------------------------------------------------------------
sub encode_log_format {

    my $log_format = shift;
    my $changes = shift;

    $log_format |= $LOG_FORMAT_UTC          if ($changes =~ m/\bUTC\b/);
    $log_format |= $LOG_FORMAT_VALID        if ($changes =~ m/\bVALID\b/);
    $log_format |= $LOG_FORMAT_LATITUDE     if ($changes =~ m/\bLATITUDE\b/);
    $log_format |= $LOG_FORMAT_LONGITUDE    if ($changes =~ m/\bLONGITUDE\b/);
    $log_format |= $LOG_FORMAT_HEIGHT       if ($changes =~ m/\bHEIGHT\b/);
    $log_format |= $LOG_FORMAT_SPEED        if ($changes =~ m/\bSPEED\b/);
    $log_format |= $LOG_FORMAT_HEADING      if ($changes =~ m/\bHEADING\b/);
    $log_format |= $LOG_FORMAT_DSTA         if ($changes =~ m/\bDSTA\b/);
    $log_format |= $LOG_FORMAT_DAGE         if ($changes =~ m/\bDAGE\b/);
    $log_format |= $LOG_FORMAT_PDOP         if ($changes =~ m/\bPDOP\b/);
    $log_format |= $LOG_FORMAT_HDOP         if ($changes =~ m/\bHDOP\b/);
    $log_format |= $LOG_FORMAT_VDOP         if ($changes =~ m/\bVDOP\b/);
    $log_format |= $LOG_FORMAT_NSAT         if ($changes =~ m/\bNSAT\b/);
    $log_format |= $LOG_FORMAT_SID          if ($changes =~ m/\bSID\b/);
    $log_format |= $LOG_FORMAT_ELEVATION    if ($changes =~ m/\bELEVATION\b/);
    $log_format |= $LOG_FORMAT_AZIMUTH      if ($changes =~ m/\bAZIMUTH\b/);
    $log_format |= $LOG_FORMAT_SNR          if ($changes =~ m/\bSNR\b/);
    $log_format |= $LOG_FORMAT_RCR          if ($changes =~ m/\bRCR\b/);
    $log_format |= $LOG_FORMAT_MILLISECOND  if ($changes =~ m/\bMILLISECOND\b/);
    $log_format |= $LOG_FORMAT_DISTANCE     if ($changes =~ m/\bDISTANCE\b/);

    $log_format &= ~$LOG_FORMAT_UTC         if ($changes =~ m/-UTC\b/);
    $log_format &= ~$LOG_FORMAT_VALID       if ($changes =~ m/-VALID\b/);
    $log_format &= ~$LOG_FORMAT_LATITUDE    if ($changes =~ m/-LATITUDE\b/);
    $log_format &= ~$LOG_FORMAT_LONGITUDE   if ($changes =~ m/-LONGITUDE\b/);
    $log_format &= ~$LOG_FORMAT_HEIGHT      if ($changes =~ m/-HEIGHT\b/);
    $log_format &= ~$LOG_FORMAT_SPEED       if ($changes =~ m/-SPEED\b/);
    $log_format &= ~$LOG_FORMAT_HEADING     if ($changes =~ m/-HEADING\b/);
    $log_format &= ~$LOG_FORMAT_DSTA        if ($changes =~ m/-DSTA\b/);
    $log_format &= ~$LOG_FORMAT_DAGE        if ($changes =~ m/-DAGE\b/);
    $log_format &= ~$LOG_FORMAT_PDOP        if ($changes =~ m/-PDOP\b/);
    $log_format &= ~$LOG_FORMAT_HDOP        if ($changes =~ m/-HDOP\b/);
    $log_format &= ~$LOG_FORMAT_VDOP        if ($changes =~ m/-VDOP\b/);
    $log_format &= ~$LOG_FORMAT_NSAT        if ($changes =~ m/-NSAT\b/);
    $log_format &= ~$LOG_FORMAT_SID         if ($changes =~ m/-SID\b/);
    $log_format &= ~$LOG_FORMAT_ELEVATION   if ($changes =~ m/-ELEVATION\b/);
    $log_format &= ~$LOG_FORMAT_AZIMUTH     if ($changes =~ m/-AZIMUTH\b/);
    $log_format &= ~$LOG_FORMAT_SNR         if ($changes =~ m/-SNR\b/);
    $log_format &= ~$LOG_FORMAT_RCR         if ($changes =~ m/-RCR\b/);
    $log_format &= ~$LOG_FORMAT_MILLISECOND if ($changes =~ m/-MILLISECOND\b/);
    $log_format &= ~$LOG_FORMAT_DISTANCE    if ($changes =~ m/-DISTANCE\b/);

    return($log_format);
}

#-------------------------------------------------------------------------
# Given a log format value (bitmask), return the record size in bytes.
#-------------------------------------------------------------------------
sub sizeof_log_format {

    my $log_format = shift;
    my $size_wpt = 0;
    my $size_sat = 0;

    $size_wpt += $SIZEOF_LOG_UTC         if ($log_format & $LOG_FORMAT_UTC);
    $size_wpt += $SIZEOF_LOG_VALID       if ($log_format & $LOG_FORMAT_VALID);
    $size_wpt += $SIZEOF_LOG_LATITUDE    if ($log_format & $LOG_FORMAT_LATITUDE);
    $size_wpt += $SIZEOF_LOG_LONGITUDE   if ($log_format & $LOG_FORMAT_LONGITUDE);
    $size_wpt += $SIZEOF_LOG_HEIGHT      if ($log_format & $LOG_FORMAT_HEIGHT);
    $size_wpt += $SIZEOF_LOG_SPEED       if ($log_format & $LOG_FORMAT_SPEED);
    $size_wpt += $SIZEOF_LOG_HEADING     if ($log_format & $LOG_FORMAT_HEADING);
    $size_wpt += $SIZEOF_LOG_DSTA        if ($log_format & $LOG_FORMAT_DSTA);
    $size_wpt += $SIZEOF_LOG_DAGE        if ($log_format & $LOG_FORMAT_DAGE);
    $size_wpt += $SIZEOF_LOG_PDOP        if ($log_format & $LOG_FORMAT_PDOP);
    $size_wpt += $SIZEOF_LOG_HDOP        if ($log_format & $LOG_FORMAT_HDOP);
    $size_wpt += $SIZEOF_LOG_VDOP        if ($log_format & $LOG_FORMAT_VDOP);
    $size_wpt += $SIZEOF_LOG_NSAT        if ($log_format & $LOG_FORMAT_NSAT);

    # Variable part, for each satellite:
    if ($log_format & $LOG_FORMAT_SID) {
        $size_sat += $SIZEOF_LOG_SID;
        $size_sat += $SIZEOF_LOG_SIDINUSE;
        $size_sat += $SIZEOF_LOG_SATSINVIEW;
        $size_sat += $SIZEOF_LOG_ELEVATION   if ($log_format & $LOG_FORMAT_ELEVATION);
        $size_sat += $SIZEOF_LOG_AZIMUTH     if ($log_format & $LOG_FORMAT_AZIMUTH);
        $size_sat += $SIZEOF_LOG_SNR         if ($log_format & $LOG_FORMAT_SNR);
    }

    $size_wpt += $SIZEOF_LOG_RCR         if ($log_format & $LOG_FORMAT_RCR);
    $size_wpt += $SIZEOF_LOG_MILLISECOND if ($log_format & $LOG_FORMAT_MILLISECOND);
    $size_wpt += $SIZEOF_LOG_DISTANCE    if ($log_format & $LOG_FORMAT_DISTANCE);

    return($size_wpt, $size_sat);

}

#-------------------------------------------------------------------------
# Return a string describing the log record separator type.
#-------------------------------------------------------------------------
sub describe_separator_type {
    my $sep_type = shift;
    return('CHANGE_LOG_BITMASK')    if ($sep_type == $SEP_TYPE_CHANGE_LOG_BITMASK);
    return('CHANGE_LOG_PERIOD')     if ($sep_type == $SEP_TYPE_CHANGE_LOG_PERIOD);
    return('CHANGE_LOG_DISTANCE')   if ($sep_type == $SEP_TYPE_CHANGE_LOG_DISTANCE);
    return('CHANGE_LOG_SPEED')      if ($sep_type == $SEP_TYPE_CHANGE_LOG_SPEED);
    return('CHANGE_OVERLAP_STOP')   if ($sep_type == $SEP_TYPE_CHANGE_OVERLAP_STOP);
    return('CHANGE_START_STOP_LOG') if ($sep_type == $SEP_TYPE_CHANGE_START_STOP_LOG);
    return('Unknown');
}

#-------------------------------------------------------------------------
# Return a string describing some field.
#-------------------------------------------------------------------------
sub describe_valid_mtk {
    my $valid = shift;
    return('nofix')     if ($valid == $VALID_NOFIX);
    return('sps')       if ($valid == $VALID_SPS);
    return('dgps')      if ($valid == $VALID_DGPS);
    return('pps')       if ($valid == $VALID_PPS);
    return('rtk')       if ($valid == $VALID_RTK);
    return('frtk')      if ($valid == $VALID_FRTK);
    return('estimated') if ($valid == $VALID_ESTIMATED);
    return('manual')    if ($valid == $VALID_MANUAL);
    return('simulator') if ($valid == $VALID_SIMULATOR);
    return('Unknown');
}

# Description suitable for GPX <trkpt> <fix> element.
sub describe_valid_gpx {
    my $valid = shift;
    return('none')         if ($valid == $VALID_NOFIX);
    return('3d')           if ($valid == $VALID_SPS);
    return('dgps')         if ($valid == $VALID_DGPS);
    return('pps')          if ($valid == $VALID_PPS);
    return(undef);
}

sub describe_rcr_mtk {
    my $rcr = shift;
    my $str = '';

    $str .= ',TIME'     if ($rcr & $RCR_TIME);
    $str .= ',SPEED'    if ($rcr & $RCR_SPEED);
    $str .= ',DISTANCE' if ($rcr & $RCR_DISTANCE);
    $str .= ',BUTTON'   if ($rcr & $RCR_BUTTON);

    return('Unknown') if ($str eq '');
    return(substr($str, 1));
}

# Description suitable for GPX <trkpt> <type> element.
sub describe_rcr_gpx {
    my $rcr = shift;
    my $str = '';

    $str .= ',TIME'     if ($rcr & $RCR_TIME);
    $str .= ',SPEED'    if ($rcr & $RCR_SPEED);
    $str .= ',DISTANCE' if ($rcr & $RCR_DISTANCE);
    $str .= ',BUTTON'   if ($rcr & $RCR_BUTTON);

    return(undef) if ($str eq '');
    return(substr($str, 1));
}

sub describe_log_status {
    my $log_status = shift;
    my $str = '';
    if ($log_status & $LOG_STATUS_AUTOLOG)        { $str .= ',AUTOLOG_ON'; }     else { $str .= ',AUTOLOG_OFF'; }
    if ($log_status & $LOG_STATUS_STOP_WHEN_FULL) { $str .= ',STOP_WHEN_FULL'; } else { $str .= ',OVERLAP_WHEN_FULL'; }
    if ($log_status & $LOG_STATUS_ENABLE)         { $str .= ',ENABLE_LOG'; }
    if ($log_status & $LOG_STATUS_DISABLE)        { $str .= ',DISABLE_LOG'; }
    if ($log_status & $LOG_STATUS_NEED_FORMAT)    { $str .= ',NEED_FORMAT'; }
    if ($log_status & $LOG_STATUS_FULL)           { $str .= ',FULL'; }
    return(substr($str, 1));
}

sub describe_recording_method {
    my $val = shift;
    return('OVERLAP') if ($val == $RCD_METHOD_OVP);
    return('STOP')      if ($val == $RCD_METHOD_STP);
    return('Unknown');
}

#-------------------------------------------------------------------------
# Silly function: return a byte!
#-------------------------------------------------------------------------
sub mtk2byte {
    ord(shift);
}

#-------------------------------------------------------------------------
# Convert a 16 bit binary data into an integer.
#-------------------------------------------------------------------------
sub mtk2unsignedword {
    unpack('S', shift);
}
sub mtk2signedword{
    unpack('s', shift);
}

#-------------------------------------------------------------------------
# Convert a 32 bit binary data into a long integer number.
#-------------------------------------------------------------------------
sub mtk2long {
    unpack('L', shift);
}

#-------------------------------------------------------------------------
# Convert 24, 32 or 64 bit binary data into a number.
#-------------------------------------------------------------------------
sub mtk2number {
    my $buffer = shift;
    my $size = shift;
    if    ($size == $SIZEOF_FLOAT3) { return unpack('f', chr(0) . $buffer); }
    elsif ($size == $SIZEOF_FLOAT)  { return unpack('f', $buffer); }
    elsif ($size == $SIZEOF_DOUBLE) { return unpack('d', $buffer); }
}

#-------------------------------------------------------------------------
# Convert seconds from epoch (long int) to UTC timestamp. 
#-------------------------------------------------------------------------
sub utc_time {
    time2str('%Y-%m-%dT%H:%M:%SZ', shift, 'GMT');
}

#-------------------------------------------------------------------------
# Print the header of a GPX file.
#-------------------------------------------------------------------------
sub gpx_print_gpx_begin {
    my $fp     = shift;
    my $time   = shift;
    my $minlat = shift;
    my $minlon = shift;
    my $maxlat = shift;
    my $maxlon = shift;
    print $fp sprintf('<?xml version="1.0" encoding="UTF-8"?>%s', $GPX_EOL);
    print $fp '<gpx' . $GPX_EOL;
    print $fp '  version="1.1"' . $GPX_EOL;
    print $fp '  creator="MTKBabel - http://www.rigacci.org/"' . $GPX_EOL;
    print $fp '  xmlns="http://www.topografix.com/GPX/1/1"' . $GPX_EOL;
    print $fp '  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"' . $GPX_EOL;
    print $fp '  xmlns:mtk="http://www.rigacci.org/gpx/MtkExtensions/v1"' . $GPX_EOL;
    print $fp '  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd' . $GPX_EOL;
    print $fp '                      http://www.rigacci.org/gpx/MtkExtensions/v1 http://www.rigacci.org/gpx/MtkExtensions/v1/MtkExtensionsv1.xsd">' . $GPX_EOL;
    print $fp sprintf('<metadata>%s', $GPX_EOL);
    print $fp sprintf('  <time>%s</time>%s', utc_time($time), $GPX_EOL);
    print $fp sprintf('  <bounds minlat="%.9f" minlon="%.9f" maxlat="%.9f" maxlon="%.9f"/>%s', $minlat, $minlon, $maxlat, $maxlon, $GPX_EOL);
    print $fp sprintf('</metadata>%s', $GPX_EOL);
}
sub gpx_print_gpx_end {
    my $fp = shift;
    print $fp sprintf('</gpx>%s', $GPX_EOL);
}

#-------------------------------------------------------------------------
# Open and close the GPX <trk> tag.
#-------------------------------------------------------------------------
sub gpx_print_trk_begin {
    my $fp = shift;
    print $fp sprintf('<trk>%s', $GPX_EOL);
    print $fp sprintf('  <name>%s</name>%s', $record_utc, $GPX_EOL);
    print $fp sprintf('  <number>%u</number>%s', $gpx_trk_number, $GPX_EOL) if ($gpx_trk_number > 0);;
    print $fp sprintf('<trkseg>%s', $GPX_EOL);
    $gpx_trk_number++;
}
sub gpx_print_trk_end {
    my $fp = shift;
    print $fp sprintf('</trkseg>%s', $GPX_EOL);
    print $fp sprintf('</trk>%s', $GPX_EOL);
}

#-------------------------------------------------------------------------
# Print a GPX <trkpt>.
#-------------------------------------------------------------------------
sub gpx_print_trkpt {
    my $fp = shift;
    print $fp sprintf('<trkpt lat="%.9f" lon="%.9f">%s', $record_latitude, $record_longitude, $GPX_EOL);
    print $fp sprintf('  <ele>%.6f</ele>%s', $record_height, $GPX_EOL) if (defined($record_height));
    print $fp sprintf('  <time>%s</time>%s', $record_utc,    $GPX_EOL) if (defined($record_utc));
    gpx_print_pt_attributes($fp);
    print $fp sprintf('</trkpt>%s', $GPX_EOL);
    $gpx_trk_minlat = $record_latitude  if ($record_latitude  < $gpx_trk_minlat);
    $gpx_trk_maxlat = $record_latitude  if ($record_latitude  > $gpx_trk_maxlat);
    $gpx_trk_minlon = $record_longitude if ($record_longitude < $gpx_trk_minlon);
    $gpx_trk_maxlon = $record_longitude if ($record_longitude > $gpx_trk_maxlon);
}

#-------------------------------------------------------------------------
# Print a GPX <wpt>.
#-------------------------------------------------------------------------
sub gpx_print_wpt {
    my $fp = shift;
    $gpx_wpt_number++;
    print $fp sprintf('<wpt lat="%.9f" lon="%.9f">%s', $record_latitude, $record_longitude, $GPX_EOL);
    print $fp sprintf('  <ele>%.6f</ele>%s',   $record_height, $GPX_EOL) if (defined($record_height));
    print $fp sprintf('  <time>%s</time>%s',   $record_utc,    $GPX_EOL) if (defined($record_utc));
    print $fp sprintf('  <name>%03d</name>%s', $gpx_wpt_number, $GPX_EOL);
    print $fp sprintf('  <cmt>%03d</cmt>%s',   $gpx_wpt_number, $GPX_EOL);
    print $fp sprintf('  <desc>%s</desc>%s',   $record_utc,     $GPX_EOL) if (defined($record_utc));
    print $fp sprintf('  <sym>Flag</sym>%s',                    $GPX_EOL);
    gpx_print_pt_attributes($fp);
    print $fp sprintf('</wpt>%s', $GPX_EOL);
    $gpx_wpt_minlat = $record_latitude  if ($record_latitude  < $gpx_wpt_minlat);
    $gpx_wpt_maxlat = $record_latitude  if ($record_latitude  > $gpx_wpt_maxlat);
    $gpx_wpt_minlon = $record_longitude if ($record_longitude < $gpx_wpt_minlon);
    $gpx_wpt_maxlon = $record_longitude if ($record_longitude > $gpx_wpt_maxlon);
}

#-------------------------------------------------------------------------
# Print <trkpt> and <wpt> common attributes.
#-------------------------------------------------------------------------
sub gpx_print_pt_attributes {
    my $fp = shift;
    print $fp sprintf('  <type>%s</type>%s',  describe_rcr_gpx($record_rcr),         $GPX_EOL) if (describe_rcr_gpx($record_rcr));
    print $fp sprintf('  <fix>%s</fix>%s',  describe_valid_gpx($record_valid),       $GPX_EOL) if (describe_valid_gpx($record_valid));
    print $fp sprintf('  <sat>%u</sat>%s',                     $record_nsat_in_use,  $GPX_EOL) if (defined($record_nsat_in_use));
    print $fp sprintf('  <hdop>%.2f</hdop>%s',                 $record_hdop,         $GPX_EOL) if (defined($record_hdop));
    print $fp sprintf('  <vdop>%.2f</vdop>%s',                 $record_vdop,         $GPX_EOL) if (defined($record_vdop));
    print $fp sprintf('  <pdop>%.2f</pdop>%s',                 $record_pdop,         $GPX_EOL) if (defined($record_pdop));
    print $fp sprintf('  <ageofdgpsdata>%u</ageofdgpsdata>%s', $record_dage,         $GPX_EOL) if (defined($record_dage));
    print $fp sprintf('  <dgpsid>%u</dgpsid>%s',               $record_dsta,         $GPX_EOL) if (defined($record_dsta));

    if (defined($record_speed) or
        defined($record_heading) or
        defined($record_nsat_in_view) or
        defined($record_millisecond) or
        defined($record_distance) or
        defined($record_satdata)) {

    print $fp sprintf('  <extensions>%s', $GPX_EOL);
    print $fp sprintf('    <mtk:wptExtension>%s', $GPX_EOL);
    print $fp sprintf('      <mtk:valid>%s</mtk:valid>%s',  describe_valid_mtk($record_valid),       $GPX_EOL) if (defined($record_valid));
    print $fp sprintf('      <mtk:speed>%.6f</mtk:speed>%s',                   $record_speed,        $GPX_EOL) if (defined($record_speed));
    print $fp sprintf('      <mtk:heading>%.6f</mtk:heading>%s',               $record_heading,      $GPX_EOL) if (defined($record_heading));
    print $fp sprintf('      <mtk:satinview>%u</mtk:satinview>%s',             $record_nsat_in_view, $GPX_EOL) if (defined($record_nsat_in_view));

    if (defined($record_satdata)) {
        my $sat;
        foreach $sat (split(/\n/, $record_satdata)) {
            my ($sid, $inuse, $elevation, $azimuth, $snr) = split(/\t/, $sat);
            print $fp sprintf('      <mtk:satdata sid="%u" inuse="%u">%s',   $sid, $inuse, $GPX_EOL);
            print $fp sprintf('        <mtk:elevation>%d</mtk:elevation>%s', $elevation,   $GPX_EOL) if ($elevation ne '');
            print $fp sprintf('        <mtk:azimuth>%u</mtk:azimuth>%s',     $azimuth,     $GPX_EOL) if ($azimuth   ne '');
            print $fp sprintf('        <mtk:snr>%u</mtk:snr>%s',             $snr,         $GPX_EOL) if ($snr       ne '');
            print $fp sprintf('      </mtk:satdata>%s', $GPX_EOL);
        }
    }

    print $fp sprintf('      <mtk:msec>%u</mtk:msec>%s',           $record_millisecond,  $GPX_EOL) if (defined($record_millisecond));
    print $fp sprintf('      <mtk:distance>%.9f</mtk:distance>%s', $record_distance,     $GPX_EOL) if (defined($record_distance));
    print $fp sprintf('    </mtk:wptExtension>%s', $GPX_EOL);
    print $fp sprintf('  </extensions>%s', $GPX_EOL);

    }
}

#-------------------------------------------------------------------------
# Log time.
#-------------------------------------------------------------------------
sub log_time {
    time2str('%H:%M:%S', time());
}

#-------------------------------------------------------------------------
# Return the flash memory size upon model ID.
#-------------------------------------------------------------------------
sub flash_memory_size {
    my $string = shift;
    my $model = hex('0x' . $string);
    # 8 Mbit = 1 Mb
    return( 8 * 1024 * 1024 / 8) if ($model == 0x1388); # 757/ZI v1
    return( 8 * 1024 * 1024 / 8) if ($model == 0x5202); # 757/ZI v2
    # 32 Mbit = 4 Mb
    return(32 * 1024 * 1024 / 8) if ($model == 0x0000); # Holux M-1200E
    return(32 * 1024 * 1024 / 8) if ($model == 0x0005); # Qstarz BT-Q1000P
    return(32 * 1024 * 1024 / 8) if ($model == 0x0006); # 747 A+ GPS Trip Recorder
    return(32 * 1024 * 1024 / 8) if ($model == 0x0008); # Pentagram PathFinder P 3106
    return(32 * 1024 * 1024 / 8) if ($model == 0x000F); # iBlue 747A+
    return(32 * 1024 * 1024 / 8) if ($model == 0x005C); # Holux M-1000C
    return(32 * 1024 * 1024 / 8) if ($model == 0x8300); # Qstarz BT-1200
    # 16Mbit -> 2Mb
    # 0x0051    i-Blue 737, Qstarz 810, Polaris iBT-GPS, Holux M1000
    # 0x0002    Qstarz 815
    # 0x001b    i-Blue 747
    # 0x001d    Qstarz BT-Q1000 / BGL-32
    # 0x0131    EB-85A
    return(16 * 1024 * 1024 / 8);
}

#-------------------------------------------------------------------------
# Functions for serial communication. Use the global variable $device.
#-------------------------------------------------------------------------
# Open the serial port.
sub serial_port_open {
    my $port = shift;
    my $baudrate = shift;
    die("Cannot open $port. Did you switch ON the GPS device?\n") if (! -c $port);
    $device = Device::SerialPort->new($port)
        || die "ERROR: Opening serial device $port: $!";
    $device->baudrate($baudrate) || die "fail setting baud rate";
    $device->parity('none')      || die "fail setting parity";
    $device->databits(8)         || die "fail setting databits";
    $device->stopbits(1)         || die "fail setting stopbits";
    $device->handshake('none')   || die "fail setting handshake";
    $device->write_settings      || die "no settings";
}

# Close the port.
sub serial_port_close {
    $device->close || warn "close failed";
}

# Write the received string to the port.
# Return the bytes actually written.
sub serial_port_write {
    return($device->write(shift));
}

# Set read timeout (in milliseconds) on the port.
sub serial_port_set_read_timeout {
    $device->read_const_time(shift);
}

# Get a character from the port.
# Return the number of characters read (1 if success) and the character itself.
sub serial_port_getch {
    return($device->read(1));
}

#-------------------------------------------------------------------------
# Set data type according to GPS model ID.
#-------------------------------------------------------------------------
sub set_data_types {

    my $model_id = shift;

    if ($model_id eq '0000' or
        $model_id eq '0021' or
        $model_id eq '0023' or
        $model_id eq '0043' or
        $model_id eq '005C')
    {
        printf("Setting log format and data types for Holux, model ID %s\n", $model_id) if ($debug >= $LOG_INFO);
        $LOG_HAS_CHECKSUM_SEPARATOR = 0;
        $SIZEOF_LOG_LATITUDE  = $SIZEOF_FLOAT;
        $SIZEOF_LOG_LONGITUDE = $SIZEOF_FLOAT;
        $SIZEOF_LOG_HEIGHT    = $SIZEOF_FLOAT3;
    }
}
