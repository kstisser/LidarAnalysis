Two files: (python 2.7)

1. code.py:
Runs directly on the Circuit Playground Express and prints out the light value

Expected: Circuit Playground Express with loaded uf2 file. This script was tested with version 5.3.1, and had the full library package loaded for this version as well. The code.py was loaded through Mu. Check Adafruit website for instructions on how to properly load and flash this board. 

2. piGetLightData.py:
This runs on the pi and publishes the light as a ros topic

Needs prior to running: sudo chmod 666 /dev/ttyACM0
