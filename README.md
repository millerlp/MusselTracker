This repository contains hardware designs and software to create a low-cost datalogger that can be used to track mussel (bivalve molluscs, not muscles) behavior  and temperature over long periods of time. The basic design includes a thermocouple temperature sensor, a hall effect magnetic sensor to sense valve gape, and a 3-D acclerometer/magnetometer to record mussel shell orientation.

Arduino-style code to run the datalogger is available the MusselTracker2 directory. The code requires the use of the MusselTrackerlib library available separately at https://github.com/millerlp/MusselTrackerlib 

Various testing code for individual sensors and the overall circuit board are available in the test_sketches directory. 

Circuit board designs are available in the Eagle_files directory.


Orginally a fork of https://github.com/andrewpetersen/MusselTracker/, now with a wholesale rewrite of the software and hardware. 

******************
Copyright Luke Miller 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
