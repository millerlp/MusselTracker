This repository contains hardware designs and software to create a low-cost datalogger that can be used to track mussel (bivalve molluscs, not muscles) behavior  and temperature over long periods of time. The basic design includes a thermocouple temperature sensor, a hall effect magnetic sensor to sense valve gape, and a 3-D acclerometer/magnetometer to record mussel shell orientation.

Arduino-style code to run the datalogger is available the MusselTracker2 directory. The code requires the use of the MusselTrackerlib library available separately at https://github.com/millerlp/MusselTrackerlib 

Various testing code for individual sensors and the overall circuit board are available in the test_sketches directory. 

Circuit board designs are available in the Eagle_files directory.


Orginally a fork of https://github.com/andrewpetersen/MusselTracker/, now with a wholesale rewrite of the software and hardware. 
