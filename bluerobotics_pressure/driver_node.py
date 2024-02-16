import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Header
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from mvp_msgs.msg import Float64Stamped

from .ms5837 import ms5837 
from time import sleep

class BlueRoboticsPressure(Node):

    def __init__(self):
        super().__init__('bluerobotics_pressure_node')

        # setup ros parameters
        self.setup_params()

        # ros subscribers and publishers
        self.setup_ros()
  
        # setup pressure sensor
        self.setup_sensor()

    def setup_params(self):

        # ros related
        frame_id_descriptor = ParameterDescriptor(description='The frame_id assign in the message!')
        self.declare_parameter('ros.frame_id', '/pressure', frame_id_descriptor)

        # sensor related
        model_descriptor = ParameterDescriptor(description='The sensor model !')
        self.declare_parameter('sensor.model', 1, model_descriptor)

        bus_descriptor = ParameterDescriptor(description='The bus of I2C !')
        self.declare_parameter('sensor.bus', 1, bus_descriptor)

        fluid_density_descriptor = ParameterDescriptor(description='The fluid density used in Pressure sensor!')
        self.declare_parameter('sensor.fluid_density', 1000.0, fluid_density_descriptor)

        # system related
        rate_descriptor = ParameterDescriptor(description='The system rate to publish the messages!')
        self.declare_parameter('system.rate', 1, rate_descriptor)

    def setup_ros(self):
        # pressure publisher
        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.pressure_msg = FluidPressure()

        # temperature publisher
        self.temperature_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.temperature_msg = Temperature()

        # depth publisher
        # TODO: change this to standard ros message, i.e., geometry_msgs/msg/Vector3Stamped
        self.depth_pub = self.create_publisher(Float64Stamped, 'depth', 10)
        self.depth_msg = Float64Stamped()        

        # timer callback
        rate = self.get_parameter('system.rate')
        timer_period = 1.0 / rate.value
        self.timer = self.create_timer(timer_period, self.timer_callback)        

    def setup_sensor(self):

        # get parameters
        sensor_model = self.get_parameter('sensor.model')
        sensor_bus = self.get_parameter('sensor.bus')
        sensor_fluid_density = self.get_parameter('sensor.fluid_density')
        
        # start the sensor
        self.sensor = ms5837.MS5837(model=sensor_model, bus=sensor_bus)

        # We must initialize the sensor before reading it
        if not self.sensor.init():
            self.get_logger().error('Sensor could not be initialized !')
            exit(1)

        # check read
        if not self.sensor.read():
            self.get_logger().error('Sensor read failed !')
            exit(1)

        # setup fluid density
        self.sensor.setFluidDensity(sensor_fluid_density)

        # sleep some time
        self.get_logger().info('Sleep a small time to init the Pressure sensor !')
        sleep(1)

    def timer_callback(self):
        # get parameters
        frame_id = self.get_parameter('ros.frame_id')

        # get header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        # get absolute pressure
        self.pressure_msg.header = header
        self.pressure_msg.fluid_pressure = self.sensor.pressure(ms5837.UNITS_psi)
        self.pressure_pub.publish(self.pressure_msg)

        # get temperature in Degrees Celsius
        self.temperature_msg.header = header
        self.temperature_msg.temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)
        self.temperature_pub.publish(self.temperature_msg)

        # get depth
        self.depth_msg.header = header
        self.depth_msg.data = self.sensor.depth()
        self.depth_pub.publish(self.depth_msg)

        # DEBUG:
        self.get_logger().info('The Pressure is "%f" Pa' % self.pressure_msg.fluid_pressure)

def main(args=None):
    rclpy.init(args=args)

    bluerobotics_pressure = BlueRoboticsPressure()

    rclpy.spin(bluerobotics_pressure)

    bluerobotics_pressure.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()  