# LidarAnalysis

Components included:

SenseHat/sensehat.py- This is a python script that publishes the room temperature and humidity each as a ros topic

CircuitPlaygroundExpress/piGetLightData.py- This receives from the Circuit Playground Express the room light reading and publishes it as a ros topic

DataCollection:
scenarioEnums.py- assigns an enum number to each classifiable test scenario (size of object, color of object, temperature of object, and distance away object is from the lidar)
dataCollection.py- this takes in arguments for each the size, color, temperature, and distance as specified in the line above, and publishes them out each as their own ros topic


Experiment procedures (each command in a different terminal):
roscore
python sensehat.py
python piGetLightData.py
python dataCollection.py

cd to the location you want to store the rosbag
rosbag collect -a


Post processing:
1. Time synchronize the data

2. Remove the extra point cloud data

3. Plot data
