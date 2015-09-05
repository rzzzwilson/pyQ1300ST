#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import struct
import binascii


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

SEP_TYPE_CHANGE_LOG_BITMASK = 0x02
SEP_TYPE_CHANGE_LOG_PERIOD = 0x03
SEP_TYPE_CHANGE_LOG_DISTANCE = 0x04
SEP_TYPE_CHANGE_LOG_SPEED = 0x05
SEP_TYPE_CHANGE_OVERLAP_STOP = 0x06
SEP_TYPE_CHANGE_START_STOP_LOG = 0x07


def abort(msg):
    print(msg)
    sys.exit(10)

def unpack(byte_array):
    """Unpack byte array into binary (LSB first)."""

    result = 0
    for val in reversed(byte_array):
        result = result*256 + ord(val)

    return result

def parse_sector_header(hdr):
    """Parse a log sector header.
   
    hdr  a string containing binary bytes of sector header

    A sector header is 512 binary bytes organized:
        name      offset   size    type   comment
        ---------+--------+-------+------+-----------------------------------
        count    | 0      | 2     | uint | number of records in the sector
        fmt      | 2      | 4     | uint | bitmask of log format
        mode     | 6      | 2     | uint | log mode (STOP or OVERWRITE)
        period   | 8      | 4     | uint | logging period (tenths of a second)
        distance | 12     | 4     | uint | logging distance (tenths of a metre)
        speed    | 16     | 4     | uint | logging speed (tenths of km/hr)
        failsect | 20     | 32    | bits | bitmask of failed sectors (0 == bad)
        unused   | 52     | 454   | ?    | unused, for later expansion
        separator| 506    | 1     | str  | separator character
        checksum | 507    | 1     | uint | checksum byte
        tail     | 508    | 4     | str  | ????
    """

    (count, fmt, mode, period, distance, speed, failsect, unused,
            separator, checksum, tail) = struct.unpack('<HIHIII32s454s1sB4s', hdr)

    hex_tail = binascii.b2a_hex(tail)
    if separator != '*' or hex_tail != 'bbbbbbbb':
        abort("ERROR: Invalid sector header, separator='%s', hex_tail='%s'"
                (separator, hex_tail))

    return (count, fmt)

def parse_log_data(data):
    """Parse log data.

    data  bytearray of log data
    """

    fp = 0
    log_len = len(data)
    record_count_total = 0

    print('parse_log_data: log_len=0x%06x (%d)' % (log_len, log_len))

    while True:
        print('>> Reading offset %08x' % fp)

        print('fp %% SIZEOF_SECTOR=%d' % (fp % SIZEOF_SECTOR))
        if (fp % SIZEOF_SECTOR) == 0:
            # reached the beginning of a log sector (every 0x10000 bytes),
            # get header (0x200 bytes)
            header = data[fp:fp + SIZEOF_SECTOR_HEADER]
            (expected_records_sector, log_format) = parse_sector_header(header)
            print('expected_records_sector=0x%06x' % expected_records_sector)

            record_count_sector = 0

#        if record_count_total >= self.expected_records_total:
#            print('Total record count: %d' % record_count_total)
#            break

        print('???: record_count_sector=%d, expected_records_sector=%d' % (record_count_sector, expected_records_sector))
        if record_count_sector >= expected_records_sector:
            print('Calculating new offset')
            new_offset = SIZEOF_SECTOR * (fp/SIZEOF_SECTOR + 1)
            if new_offset < log_len:
                fp = new_offset
                continue
            else:
                # end of file
                print('Total record count: %d' % record_count_total)
                print('Total record count: %d' % record_count_total)
                break

#        #------------------------------------------------------------------
#        # Check for:
#        # - record separator:          "AAAAAAAAAAAAAAXXYYYYYYYYBBBBBBBB"
#        # - non written space:         "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
#        # - Holux M-241 separators:    "HOLUXGR241LOGGER"
#        #                              "HOLUXGR241WAYPNT"
#        # - Holux M-241 fw 1.13 sep.:  "HOLUXGR241LOGGER    "
#        #                              "HOLUXGR241WAYPNT    "
#        # - Holux Holux M-1200E sep.:  "HOLUXM1200WAYPNT    "
#        #
#        # - Holux GPSport GR-245 sep.: "HOLUXGR245LOGGER    "
#        #                              "HOLUXGR245WAYPNT    "
#        #------------------------------------------------------------------
        if (log_len - fp) >= SIZEOF_SEPARATOR:
#        if (($log_len - tell($fp)) >= $SIZEOF_SEPARATOR) {
#
#            $buffer = my_read($fp, $SIZEOF_SEPARATOR);
            buffer = data[fp:fp+SIZEOF_SEPARATOR]
            print('len(buffer)=%d' % len(buffer))
            print('buffer:\n%s' % str(buffer))
            separator = buffer[:32]
            print('len(separator)=%d' % len(separator))
            print('binascii.b2a_hex(separator):\n%s' % binascii.b2a_hex(separator))
            (buff_hdr, _, buff_tail) = struct.unpack('7s5s4s', buffer)
            print('buff_hdr=%s' % str(buff_hdr))
#
#            if ((substr($buffer, 0, 7) eq (chr(0xaa) x 7)) and (substr($buffer, -4) eq (chr(0xbb) x 4))) {
            if buff_hdr == chr(0xaa) * 7 and buff_tail == chr(0xbb) * 4:
                # Found a record separator.
#                #----------------------------------------------------------
#                # Found a record separator.
#                #----------------------------------------------------------
#                # Close the current <trk> in GPX.
#                if ($gpx_in_trk) {
#                    gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
#                    $gpx_in_trk = 0;
#                }
#                my $separator_type = ord(substr($buffer, 7, $SIZEOF_BYTE));
#                my $separator_arg  = mtk2long(substr($buffer, 8, $SIZEOF_LONG));
#                printf("Separator: %s, type: %s\n", uc(unpack('H*', $buffer)), describe_separator_type($separator_type)) if ($debug >= $LOG_INFO);
                separator_type = ord(buffer[7])
                print('separator_type=%s' % str(separator_type))
#                if ($separator_type == $SEP_TYPE_CHANGE_LOG_BITMASK) {
#                    $log_format = $separator_arg;
#                    printf("New log bitmask: %s (0x%08X = %s)\n", $separator_arg, $log_format, describe_log_format($log_format)) if ($debug >= $LOG_INFO);
#                }
#                next; # Search for the next record or record separator.
#
#            } elsif (substr($buffer, 0, 5) eq 'HOLUX') {
#                #----------------------------------------------------------
#                # Found Holux separator.
#                #----------------------------------------------------------
#                printf("Separator: %s\n", $buffer) if ($debug >= $LOG_INFO);
#
#                # Sarch for trailig four spaces after the separator.
#                $trailing_spaces = 0;
#                if (($log_len - tell($fp)) >= 4) {
#                    if (my_read($fp, 4) eq '    ') {
#                        $trailing_spaces = 1;
#                    } else {
#                        seek($fp, -4, 1);
#                    }
#                }
#
#                # Define $model_id upon separator string.
#                $sep_model = substr($buffer, 5,  5);
#                if ($sep_model eq 'GR241' and $trailing_spaces) {
#                    $model_id = '0043'; # M-241 fw 1.13
#                } elsif ($sep_model eq 'GR241') {
#                    $model_id = '0021'; # M-241
#                } elsif ($sep_model eq 'GR245' or $sep_model eq 'M1200') {
#                    $model_id = '0000'; # M-1200E or GPSport 245
#                } else {
#                    $model_id = '0021'; # Unknown Holux model, assume M-241
#                }
#                set_data_types($model_id);
#
#                # Check if the following data is a waypoint.
#                if (substr($buffer, 10, 6) eq 'WAYPNT') {
#                    $next_data_force_waypoint = 1;
#                }
#                next;
#
#            } elsif ($buffer eq (chr(0xff) x $SIZEOF_SEPARATOR)) {
#                #----------------------------------------------------------
#                # Found non-written space.
#                #----------------------------------------------------------
#                # Close the current <trk> in GPX.
#                if ($gpx_in_trk) {
#                    gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
#                    $gpx_in_trk = 0;
#                }
#                if ($expected_records_sector == 0xffff) {
#                    # Sector record count = 0xffff means this is the currently writing sector.
#                    # We found empty space, so we skip to sector end.
#                    $new_offset = $SIZEOF_SECTOR * (int(tell($fp) / $SIZEOF_SECTOR) + 1);
#                    if ($new_offset < $log_len) {
#                        # Log file contains more data (that is old data, being overwritten).
#                        seek($fp, $new_offset, 0);
#                        next; # Search for the next record or record separator.
#                    } else {
#                        # End Of File.
#                        if (!defined($opt_b)) {
#                            printf("ERROR: End of log file! Total record count: %u, expected %u\n", $record_count_total, $expected_records_total);
#                        } else {
#                            printf("Total record count: %u\n", $record_count_total); 
#                        }
#                        last;
#                    } 
#                } else {
#                    # ERROR! Non written space, but this is not the writing sector.
#                    printf("ERROR: Non written space! Read %u records, expected %u\n", $record_count_sector, $expected_records_sector);
#                    next if ($opt_i);
#                    last;
#                }
#
#            } else {
#                # None of above, should be record data: rewind the file pointer so we can read it.
#                seek($fp, -$SIZEOF_SEPARATOR, 1);
#            }
#        }
#
#        #-----------------------------------------
#        # Read a log record.
#        #-----------------------------------------
#        $record_count_sector++;
#        $record_count_total++;
#        $checksum = 0;
#        printf("Reading log sector: record %u (%u/%u total)\n", $record_count_sector, $record_count_total, $expected_records_total) if ($debug >= $LOG_INFO);
#
#        # Read each record field.
#        undef($record_utc);
#        if ($log_format & $LOG_FORMAT_UTC) {
#            $buffer = my_read($fp, $SIZEOF_LOG_UTC);
#            $checksum ^= packet_checksum($buffer);
#            $record_utc = utc_time(mtk2long($buffer));
#            printf("Record UTC: %s %s\n", uc(unpack('H*', $buffer)), $record_utc) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_valid);
#        if ($log_format & $LOG_FORMAT_VALID) {
#            $buffer = my_read($fp, $SIZEOF_LOG_VALID);
#            $checksum ^= packet_checksum($buffer);
#            $record_valid = mtk2unsignedword($buffer);
#            printf("Record VALID: %s (0x%04X = %s)\n", uc(unpack('H*', $buffer)), $record_valid, describe_valid_mtk($record_valid)) if ($debug >= $LOG_DEBUG);
#         }
#
#        undef($record_latitude);
#        if ($log_format & $LOG_FORMAT_LATITUDE) {
#            $buffer = my_read($fp, $SIZEOF_LOG_LATITUDE);
#            $checksum ^= packet_checksum($buffer);
#            $record_latitude = mtk2number($buffer, $SIZEOF_LOG_LATITUDE);
#            printf("Record LATITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_latitude) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_longitude);
#        if ($log_format & $LOG_FORMAT_LONGITUDE) {
#            $buffer = my_read($fp, $SIZEOF_LOG_LONGITUDE);
#            $checksum ^= packet_checksum($buffer);
#            $record_longitude = mtk2number($buffer, $SIZEOF_LOG_LONGITUDE);
#            printf("Record LONGITUDE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_longitude) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_height);
#        if ($log_format & $LOG_FORMAT_HEIGHT) {
#            $buffer = my_read($fp, $SIZEOF_LOG_HEIGHT);
#            $checksum ^= packet_checksum($buffer);
#            $record_height = mtk2number($buffer, $SIZEOF_LOG_HEIGHT);
#            printf("Record HEIGHT: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_height) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_speed);
#        if ($log_format & $LOG_FORMAT_SPEED) {
#            $buffer = my_read($fp, $SIZEOF_LOG_SPEED);
#            $checksum ^= packet_checksum($buffer);
#            $record_speed = mtk2number($buffer, $SIZEOF_LOG_SPEED);
#            printf("Record SPEED: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_speed) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_heading);
#        if ($log_format & $LOG_FORMAT_HEADING) {
#            $buffer = my_read($fp, $SIZEOF_LOG_HEADING);
#            $checksum ^= packet_checksum($buffer);
#            $record_heading = mtk2number($buffer, $SIZEOF_LOG_HEADING);
#            printf("Record HEADING: %s (%.6f)\n", uc(unpack('H*', $buffer)), $record_heading) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_dsta);
#        if ($log_format & $LOG_FORMAT_DSTA) {
#            $buffer = my_read($fp, $SIZEOF_LOG_DSTA);
#            $checksum ^= packet_checksum($buffer);
#            $record_dsta = mtk2unsignedword($buffer);
#            printf("Record DSTA: %s (%u)\n", uc(unpack('H*', $buffer)), $record_dsta) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_dage);
#        if ($log_format & $LOG_FORMAT_DAGE) {
#            $buffer = my_read($fp, $SIZEOF_LOG_DAGE);
#            $checksum ^= packet_checksum($buffer);
#            $record_dage = mtk2long($buffer);
#            printf("Record DAGE: %s (%u)\n", uc(unpack('H*', $buffer)), $record_dage) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_pdop);
#        if ($log_format & $LOG_FORMAT_PDOP) {
#            $buffer = my_read($fp, $SIZEOF_LOG_PDOP);
#            $checksum ^= packet_checksum($buffer);
#            $record_pdop = mtk2unsignedword($buffer) / 100;
#            printf("Record PDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_pdop) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_hdop);
#        if ($log_format & $LOG_FORMAT_HDOP) {
#            $buffer = my_read($fp, $SIZEOF_LOG_HDOP);
#            $checksum ^= packet_checksum($buffer);
#            $record_hdop = mtk2unsignedword($buffer) / 100;
#            printf("Record HDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_hdop) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_vdop);
#        if ($log_format & $LOG_FORMAT_VDOP) {
#            $buffer = my_read($fp, $SIZEOF_LOG_VDOP);
#            $checksum ^= packet_checksum($buffer);
#            $record_vdop = mtk2unsignedword($buffer) / 100;
#            printf("Record VDOP: %s (%.2f)\n", uc(unpack('H*', $buffer)), $record_vdop) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_nsat_in_use);
#        undef($record_nsat_in_view);
#        if ($log_format & $LOG_FORMAT_NSAT) {
#            $buffer = my_read($fp, $SIZEOF_BYTE);
#            $checksum ^= packet_checksum($buffer);
#            $record_nsat_in_view = mtk2byte($buffer);
#            $buffer = my_read($fp, $SIZEOF_BYTE);
#            $checksum ^= packet_checksum($buffer);
#            $record_nsat_in_use = mtk2byte($buffer);
#            printf("Record NSAT: in view %u, in use %u\n", $record_nsat_in_view, $record_nsat_in_use) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_satdata);
#        if ($log_format & $LOG_FORMAT_SID) {
#            my $satdata_count = 0;
#            my $satdata_sid;
#            my $satdata_inuse;
#            my $satdata_inview;
#            my $satdata_elevation;
#            my $satdata_azimuth;
#            my $satdata_snr;
#            while (1) {
#                # Even with zero satellites in view, we have at least one bunch of data.
#                $buffer = my_read($fp, $SIZEOF_LOG_SID);
#                $checksum ^= packet_checksum($buffer);
#                $satdata_sid = mtk2byte($buffer);
#                $buffer = my_read($fp, $SIZEOF_LOG_SIDINUSE);
#                $checksum ^= packet_checksum($buffer);
#                $satdata_inuse = mtk2byte($buffer);
#                $buffer = my_read($fp, $SIZEOF_LOG_SATSINVIEW);
#                $checksum ^= packet_checksum($buffer);
#                $satdata_inview = mtk2unsignedword($buffer);
#                if ($satdata_inview == 0) {
#                    printf("No satellites in view\n") if ($debug >= $LOG_DEBUG);
#                } else {
#                    # Read data for this satellite.
#                    printf("Sats in view: %u, SatID %u in use: %u\n", $satdata_inview, $satdata_sid, $satdata_inuse) if ($debug >= $LOG_DEBUG);
#                    undef($satdata_elevation);
#                    if ($log_format & $LOG_FORMAT_ELEVATION) {
#                        $buffer = my_read($fp, $SIZEOF_LOG_ELEVATION);
#                        $checksum ^= packet_checksum($buffer);
#                        $satdata_elevation = mtk2signedword($buffer);
#                        printf("Satellite ELEVATION: %s (%d)\n", uc(unpack('H*', $buffer)), $satdata_elevation) if ($debug >= $LOG_DEBUG);
#                    }
#                    undef($satdata_azimuth);
#                    if ($log_format & $LOG_FORMAT_AZIMUTH) {
#                        $buffer = my_read($fp, $SIZEOF_LOG_AZIMUTH);
#                        $checksum ^= packet_checksum($buffer);
#                        $satdata_azimuth = mtk2unsignedword($buffer);
#                        printf("Satellite AZIMUTH:   %s (%u)\n", uc(unpack('H*', $buffer)), $satdata_azimuth) if ($debug >= $LOG_DEBUG);
#                    }
#                    undef($satdata_snr);
#                    if ($log_format & $LOG_FORMAT_SNR) {
#                        $buffer = my_read($fp, $SIZEOF_LOG_SNR);
#                        $checksum ^= packet_checksum($buffer);
#                        $satdata_snr = mtk2unsignedword($buffer);
#                        printf("Satellite SNR:       %s (%u)\n", uc(unpack('H*', $buffer)), $satdata_snr) if ($debug >= $LOG_DEBUG);
#                    }
#                    $record_satdata .= "\n" if ($record_satdata ne '');
#                    $record_satdata .= $satdata_sid       . "\t";
#                    $record_satdata .= $satdata_inuse     . "\t";
#                    $record_satdata .= $satdata_elevation . "\t";
#                    $record_satdata .= $satdata_azimuth   . "\t";
#                    $record_satdata .= $satdata_snr;
#                    $satdata_count++;
#                }
#                last if ($satdata_count >= $satdata_inview);
#            }
#        }
#
#        undef($record_rcr);
#        if ($log_format & $LOG_FORMAT_RCR) {
#            $buffer = my_read($fp, $SIZEOF_LOG_RCR);
#            $checksum ^= packet_checksum($buffer);
#            $record_rcr = mtk2unsignedword($buffer);
#            printf("Record RCR: %s (%s)\n", uc(unpack('H*', $buffer)), describe_rcr_mtk($record_rcr)) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_millisecond);
#        if ($log_format & $LOG_FORMAT_MILLISECOND) {
#            $buffer = my_read($fp, $SIZEOF_LOG_MILLISECOND);
#            $checksum ^= packet_checksum($buffer);
#            $record_millisecond = mtk2unsignedword($buffer);
#            printf("Record MILLISECOND: %s (%u)\n", uc(unpack('H*', $buffer)), $record_millisecond) if ($debug >= $LOG_DEBUG);
#        }
#
#        undef($record_distance);
#        if ($log_format & $LOG_FORMAT_DISTANCE) {
#            $buffer = my_read($fp, $SIZEOF_LOG_DISTANCE);
#            $checksum ^= packet_checksum($buffer);
#            $record_distance = mtk2number($buffer, $SIZEOF_LOG_DISTANCE);
#            printf("Record DISTANCE: %s (%.9f)\n", uc(unpack('H*', $buffer)), $record_distance) if ($debug >= $LOG_DEBUG);
#        }
#
#        if ($LOG_HAS_CHECKSUM_SEPARATOR) {
#            # Read separator between data and checksum.
#            $buffer = my_read($fp, $SIZEOF_BYTE);
#            if ($buffer ne '*') {
#                printf("ERROR: Checksum separator error: expected char 0x%02X, found 0x%02X\n", ord('*'), ord($buffer));
#                last if (! $opt_i);
#                next if (! $opt_I);
#            }
#        }
#        # Read and verify checksum.
#        $buffer = my_read($fp, $SIZEOF_BYTE);
#        if ($checksum != ord($buffer)) {
#            printf("ERROR: Record checksum error: expected 0x%02X, computed 0x%02X\n", ord($buffer), $checksum);
#            last if (! $opt_i);
#            next if (! $opt_I);
#        }
#
#        # Start a new GPX <trkseg> on satellite lost.
#        if (($record_valid == $VALID_NOFIX) and $gpx_in_trk) {
#            gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
#            $gpx_in_trk = 0;
#        }
#
#        if (defined($record_latitude) and defined($record_longitude)) {
#            if ($next_data_force_waypoint) {
#                # Write <wpt> data in GPX file.
#                gpx_print_wpt($fp_gpx_wpt) if ($opt_w or $opt_c);
#                $next_data_force_waypoint = 0;
#            } else {
#                # Write <trkpt> data in GPX file.
#                if (($record_valid != $VALID_NOFIX) and !($record_rcr & $RCR_BUTTON)) {
#                    if (! $gpx_in_trk) {
#                        gpx_print_trk_begin($fp_gpx_trk) if ($opt_t or $opt_c);
#                        $gpx_in_trk = 1;
#                    }
#                    gpx_print_trkpt($fp_gpx_trk) if ($opt_t or $opt_c);
#                }
#                # Write <wpt> data in GPX file.
#                if (($record_rcr & $RCR_BUTTON) and ($record_valid != $VALID_NOFIX)) {
#                    gpx_print_wpt($fp_gpx_wpt) if ($opt_w or $opt_c);
#                }
#            }
#        }
#
#    }
#    close($fp);
#
#    # Eventually close the <trk> GPX tags.
#    if ($gpx_in_trk) {
#        gpx_print_trk_end($fp_gpx_trk) if ($opt_t or $opt_c);
#        $gpx_in_trk = 0;
#    }
#
#    # Close temporary files.
#    close($fp_gpx_trk) if ($opt_t or $opt_c);
#    close($fp_gpx_wpt) if ($opt_w or $opt_c);
#
#    # Write GPX combined tracks and waypoint file.
#    if ($opt_c) {
#        my $gpx_minlat = $gpx_trk_minlat < $gpx_wpt_minlat ? $gpx_trk_minlat : $gpx_wpt_minlat;
#        my $gpx_maxlat = $gpx_trk_maxlat > $gpx_wpt_maxlat ? $gpx_trk_maxlat : $gpx_wpt_maxlat;
#        my $gpx_minlon = $gpx_trk_minlon < $gpx_wpt_minlon ? $gpx_trk_minlon : $gpx_wpt_minlon;
#        my $gpx_maxlon = $gpx_trk_maxlon > $gpx_wpt_maxlon ? $gpx_trk_maxlon : $gpx_wpt_maxlon;
#        open($fp_gpx, ">$gpx_fname") or die("ERROR writing $gpx_fname: $!");
#        gpx_print_gpx_begin($fp_gpx, time(), $gpx_minlat, $gpx_minlon, $gpx_maxlat, $gpx_maxlon);
#        open($fp_gpx_trk, "$gpx_trk_tmp_fname") or die;
#        while (<$fp_gpx_trk>) { print $fp_gpx $_; }
#        close($fp_gpx_trk);
#        open($fp_gpx_wpt, "$gpx_wpt_tmp_fname") or die;
#        while (<$fp_gpx_wpt>) { print $fp_gpx $_; }
#        close($fp_gpx_wpt);
#        gpx_print_gpx_end($fp_gpx);
#        close($fp_gpx);
#    }
#
#    # Write GPX tracks file.
#    if ($opt_t) {
#        open($fp_gpx, ">$gpx_trk_fname") or die("ERROR writing $gpx_trk_fname: $!");
#        gpx_print_gpx_begin($fp_gpx, time(), $gpx_trk_minlat, $gpx_trk_minlon, $gpx_trk_maxlat, $gpx_trk_maxlon);
#        open($fp_gpx_trk, "$gpx_trk_tmp_fname") or die;
#        while (<$fp_gpx_trk>) { print $fp_gpx $_; }
#        close($fp_gpx_trk);
#        gpx_print_gpx_end($fp_gpx);
#        close($fp_gpx);
#    }
#
#    # Write GPX waypoints file.
#    if ($opt_w) {
#        open($fp_gpx, ">$gpx_wpt_fname") or die("ERROR writing $gpx_wpt_fname: $!");
#        gpx_print_gpx_begin($fp_gpx, time(), $gpx_wpt_minlat, $gpx_wpt_minlon, $gpx_wpt_maxlat, $gpx_wpt_maxlon);
#        open($fp_gpx_wpt, "$gpx_wpt_tmp_fname") or die;
#        while (<$fp_gpx_wpt>) { print $fp_gpx $_; }
#        close($fp_gpx_wpt);
#        gpx_print_gpx_end($fp_gpx);
#        close($fp_gpx);
#    }
#
#    # Remove temporary files.
#    unlink($gpx_trk_tmp_fname) if ($opt_t or $opt_c);
#    unlink($gpx_wpt_tmp_fname) if ($opt_w or $opt_c);


with open('debug.bin', 'rb') as fd:
    memory = fd.read()

parse_log_data(memory)
