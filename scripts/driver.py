#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from mvp_msgs.msg import Float64Stamped
import ms5837

# Node example class.
class BlueRoboticsPressure():
    def __init__(self):
        self.node_name = rospy.get_name()

        # ros parameters
        self.get_params()

        # ros subscribers and publishers
        self.setup_ros()

        # setup pressure sensor
        self.setup_sensor()


        # Main while loop.
        while not rospy.is_shutdown():
            
            # get header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_id

            # get pressure
            self.pressure_msg.header = header
            self.pressure_msg.data = self.sensor.pressure(ms5837.UNITS_psi)
            self.pressure_pub.publish(self.pressure_msg)

            # get temperature
            self.temperature_msg.header = header
            self.temperature_msg.data = self.sensor.temperature(ms5837.UNITS_Centigrade)
            self.temperature_pub.publish(self.temperature_msg)

            # get depth
            self.depth_msg.header = header
            self.depth_msg.data = self.sensor.depth()
            self.depth_pub.publish(self.depth_msg)

            # Sleep for a while before publishing new messages. Division is so rate != period.
            rospy.sleep(1.0/self.rate)

    def get_params(self):
        # ros configureation
        self.topic_pressure = rospy.get_param('~ros/topic_pressure', '/bluerobotics_pressure/pressure')
        self.topic_temperature = rospy.get_param('~ros/topic_temperature', '/bluerobotics_pressure/temperature')
        self.topic_depth = rospy.get_param('~ros/topic_depth', '/bluerobotics_pressure/depth')
        self.frame_id = rospy.get_param('~ros/frame_id', '/pressure')

        # sensor configureation
        self.sensor_model = int(rospy.get_param('~sensor/model', '1'))
        self.sensor_bus = int(rospy.get_param('~sensor/bus', '1'))
        self.sensor_fluid_density = float(rospy.get_param('~sensor/fluid_density', '997.0474'))

        # driver system configuration
        self.rate = int(rospy.get_param('~system/rate', '1'))

    def setup_ros(self):
        # presure pub
        self.pressure_pub = rospy.Publisher(self.topic_pressure, Float64Stamped, queue_size=10)
        self.pressure_msg = Float64Stamped()

        # temperature pub
        self.temperature_pub = rospy.Publisher(self.topic_temperature, Float64Stamped, queue_size=10)
        self.temperature_msg = Float64Stamped()

        # depth pub
        self.depth_pub = rospy.Publisher(self.topic_depth, Float64Stamped, queue_size=10)
        self.depth_msg = Float64Stamped()

    def setup_sensor(self):
        # start the sensor
        self.sensor = ms5837.MS5837(model=self.sensor_model, bus=self.sensor_bus)

        # We must initialize the sensor before reading it
        if not self.sensor.init():
            rospy.logerr('%s: Sensor could not be initialized', self.node_name)
            exit(1)

        # check read
        if not self.sensor.read():
            rospy.logerr('%s: Sensor read failed!', self.node_name)
            exit(1)

        # setup fluid density
        self.sensor.setFluidDensity(self.sensor_fluid_density)

        rospy.sleep(1)


# Main function.
if __name__ == '__main__':
    rospy.init_node('blueRobotics_pressure_node')

    try:
        node = BlueRoboticsPressure()
    except rospy.ROSInterruptException: pass