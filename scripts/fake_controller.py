#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

current_setpoint_value = Float64()

# Topic callback function.
def setpointValueListenerCallback(setpoint_value):
    rospy.loginfo('%s', setpoint_value.data)
    current_setpoint_value.data = setpoint_value.data

def fakeControllerHandler():
    current_setpoint_value.data = 0

    # Create a new SensorInformation object and fill in its contents.
    actual_value = Float64()
    actual_value.data = 0

    av_publisher = rospy.Publisher('my_controller/actual_value', Float64, queue_size = 10)
    rate = rospy.Rate(10)

    rospy.Subscriber('my_controller/setpoint_value', Float64, setpointValueListenerCallback)

    while not rospy.is_shutdown():
        # Read the sensor data from a simulated sensor.
        # Publish the sensor information on the /actual_value topic.
        error = current_setpoint_value.data - actual_value.data
        if error > 0.099999999999:
            actual_value.data = actual_value.data + 0.1
        if error < -0.099999999999:
            actual_value.data = actual_value.data - 0.1
        av_publisher.publish(actual_value)
        # Print log message if all went well.
        #rospy.loginfo("All went well. Publishing topic ")
        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('fake_controller', anonymous=False)

    try:
        fakeControllerHandler()
    except rospy.ROSInterruptException:
        pass
