#!/usr/bin/env python
#<trkpt lat="32.182141790" lon="76.344743429">

import sys

def usage(msg=None):
    if msg:
        print('\n%s\n' % msg)
    print('Usage: data2kml <input file> [<output file>]')
    sys.exit(1)

def print_preamble(fd):
    fd.write('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
             '<Document>\n'
             '    <name>Paths</name>\n'
             '    <description>DESCRIPTION</description>\n'
             '    <Style id="yellowLineGreenPoly">\n'
             '        <LineStyle>\n'
             '            <color>ffb5c5ff</color>\n'
             '            <width>5</width>\n'
             '        </LineStyle>\n'
             '        <PolyStyle>\n'
             '            <color>ffb5c500</color>\n'
             '        </PolyStyle>\n'
             '    </Style>\n'
             '    <Placemark>\n'
             '        <name>NAME</name>\n'
             '        <description>DESCRIPTION</description>\n'
             '        <styleUrl>#yellowLineGreenPoly</styleUrl>\n'
             '        <LineString>\n'
             '            <altitudeMode>relative</altitudeMode>\n'
             '            <coordinates>\n')

def print_postamble(fd):
    fd.write('            </coordinates>\n'
             '        </LineString>\n'
             '    </Placemark>\n'
             '</Document>\n'
             '</kml>\n')


# check params
if len(sys.argv) < 2:
    usage()
input_filename = sys.argv[1]
if len(sys.argv) > 2:
    output_filename = sys.argv[2]
else:
    output_filename = input_filename + '.kml'

# analyse file
with open(input_filename, 'rb') as fd:
    lines = fd.readlines()

with open(output_filename, 'wb') as fd:
    print_preamble(fd)
    for line in lines:
        line = line.strip()
        if line.startswith('<trkpt '):
            #print(line)
            fields = line.split('=')
            lat = fields[1].split('"')[1]
            lon = fields[2].split('"')[1]
            fd.write('%s,%s,0\n' % (lon, lat))
            #print('%s,%s,0' % (lon, lat))
    print_postamble(fd)
