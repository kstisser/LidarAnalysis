#! /usr/bin/env python

import scenarioEnums
import rospy
from std_msgs.msg import Float32
import sys
from std_msgs.msg import Int8

def publishData():
    print 'Number of arguments: ', len(sys.argv), 'arguments.'
    print 'Expecting order: size, object temperature, object color, distance'

    if len(sys.argv) != 5:
        print('Error! Wrong # of arguments!')
        exit()

    size = int(sys.argv[1])
    objColor = int(sys.argv[2])
    objTemp = int(sys.argv[4])
    distance = int(sys.argv[3])

    testCase = scenarioEnums.TestCase(size, objTemp, objColor, distance)
    
    sizePub = rospy.Publisher('size', Int8, queue_size=10, latch=True)
    colorPub = rospy.Publisher('color', Int8, queue_size=10, latch=True)
    tempPub = rospy.Publisher('objectTemp', Int8, queue_size=10, latch=True)
    distancePub = rospy.Publisher('distance', Int8, queue_size=10, latch=True)

    rospy.init_node('scenario', anonymous=True)

    sizePub.publish(size)
    colorPub.publish(objColor)
    tempPub.publish(objTemp)
    distancePub.publish(distance)

    r = rospy.Rate(1) # 1 hz, just so it doesn't end

    while not rospy.is_shutdown():
         r.sleep()

if __name__ == '__main__':
    try:
        publishData()
    except rospy.ROSInterruptException:
        pass    
