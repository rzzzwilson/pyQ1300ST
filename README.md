##Python code to manage a BT-Q1300ST GPS logger from QStarz.

This is a rewrite of mtkbabel-0.8.3.1 from [http://sourceforge.net/projects/mtkbabel/].

The file mtkbabel.pl is the base file from the above project.  The mtkbabel.py is the
python rewrite.  The log.py file is a logging module for debug.

The file mtkbabel.bin is a binary dump of the device memory.  Reading this makes
debugging quicker.

The file data2kml.py converts a GPX file output by mtkbabel.pl into a Google Earth
KML file.

The files *PinkBus2.kml* and *Walking_21August2014.kml* are two KML files produced
by data2kml.py from real data.  *PinkBus2.kml* is the track followed by a Phuket
pink bus (number 2).  *Walking_21August2014.kml* is just a simple file from a test
walk around my area.

NOTE: the original development of this was done on OSX.  Unfortunately, since
Yosemite (10.10) the QStarz BT-Q1300ST device isn't mounted under OSX.  Development
was switched to Linux (XUbuntu) at that point.

NOTE: Product web page: http://www.qstarz.com/Products/GPS%20Products/BT-Q1300ST-F.htm
