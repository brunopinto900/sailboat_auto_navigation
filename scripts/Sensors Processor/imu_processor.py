import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
import math

class IMU_processor():

    def __init__(self):

	rospy.init_node("IMU_processor", anonymous=True)
	self.freq = rospy.get_param("config/rate")
        self.rate = rospy.Rate(self.freq)
	
	self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size = 10)
	
	self.orientation_covar = rospy.get_param("simulation/imu/orientation_covariance")
	self.gyro_covar = rospy.get_param("simulation/imu/angular_velocity_covariance")
	self.acc_covar = rospy.get_param("simulation/imu/linear_acceleration_covariance")


        self.pitch_pub = rospy.Publisher('pitch', Float32, queue_size = 10)
	self.roll_pub = rospy.Publisher('roll', Float32, queue_size = 10)
	self.yaw_pub = rospy.Publisher('yaw', Float32, queue_size = 10)

	self.pitch_rate_pub = rospy.Publisher('pitch_rate', Float32, queue_size = 10)
	self.roll_rate_pub = rospy.Publisher('roll_rate', Float32, queue_size = 10)
	self.yaw_rate_pub = rospy.Publisher('yaw_rate', Float32, queue_size = 10)

	self.gyro_pitch_pub = rospy.Publisher('gyro_pitch', Float32, queue_size = 10)
	self.gyro_roll_pub = rospy.Publisher('gyro_roll', Float32, queue_size = 10)
	self.gyro_yaw_pub = rospy.Publisher('gyro_yaw', Float32, queue_size = 10)

	self.acc_velX_pub = rospy.Publisher('acc_velX', Float32, queue_size = 10)
	self.acc_velY_pub = rospy.Publisher('acc_velY', Float32, queue_size = 10)

	self.gyro_pitch = 0
	self.gyro_roll = 0
	self.gyro_yaw = 0
	self._last_time_gyro = 0

	self._last_time_acc = 0
	self.acc_velX = 0
	self.acc_velY = 0

	rospy.Subscriber('minimu/mag', Vector3, self.get_mag)
        self.MagX = 0
	self.MagY = 0
	self.MagZ = 0
	self.heading = 0

	rospy.Subscriber('minimu/acceleration', Vector3, self.get_acc)
	self.AccX = 0
	self.AccY = 0
	self.AccZ = 0
	self.pitch = 0
	self.roll = 0

	rospy.Subscriber('minimu/gyroscope', Vector3, self.get_gyro)
        self.GyroX = 0
	self.GyroY = 0
	self.GyroZ = 0


    def get_mag(self, msg):
	MagX = msg.x
	MagY = msg.y
	MagZ = msg.z
	self.heading = math.degrees(math.atan2(-MagY, MagX))
	self.heading = (heading + offset_true_north) % 360


    def get_acc(self, msg):
	AccX = msg.x
	AccY = msg.y
	AccZ = msg.z
	self.pitch = math.atan2(AccX, math.sqrt(AccY**2 + AccZ**2))
	self.roll = math.atan2(-AccY, -AccZ)


    def get_gyro(self, msg):
	GyroX = msg.x
	GyroY = msg.y
	GyroZ = msg.z


    def imu_data_publisher(self):
	
	while not rospy.is_shutdown():

	    imu = Imu();
	    imu.header.stamp = rospy.Time.now()
	    imu.header.frame_id = "sailboat"
	
	    [imu_raw_msg.orientation.x,
             imu_raw_msg.orientation.y,
             imu_raw_msg.orientation.z,
             imu_raw_msg.orientation.w]= tf.transformations.quaternion_from_euler(self.roll, self.pitch, math.radians(self.heading))
	
	    imu.angular_velocity = Vector3(self.GyroX, self.GyroY, self.GyroZ)
            imu.linear_acceleration = Vector3(self.AccX, self.AccY, self.AccZ)
	    imu.orientation_covariance = self.orientation_covar
	    imu.angular_velocity_covariance = self.gyro_covar
	    imu.linear_velocity_covariance = self.acc_covar
	
            imu_pub.publish(imu)

	    self.rate.sleep()

    def pitch_yaw_roll_publisher(self):

	while not rospy.is_shutdown():
	
	    
	    pitch_pub.publish(math.degrees(pitch))

	    
	    roll_pub.publish(math.degrees(pitch))

	 
	    yaw_pub.publish(math.degrees(heading))

	    self.rate.sleep()


    def pitch_yaw_roll_rate_publisher(self):

	while not rospy.is_shutdown():
	
	    pitch_rate = GyroY
	    pitch_rate_pub.publish(pitch_rate)

	    roll_rate = GyroX
	    roll_rate_pub.publish(roll_rate)

	    yaw_rate = GyroZ
	    yaw_rate_pub.publish(yaw_rate)

	    self.rate.sleep()


    def gyro_publisher(self):

	while not rospy.is_shutdown():

	    cur_time = time.time()
	    dt = cur_time - self._last_time
            self._last_time_gyro = cur_time

	    gyro_pitch += pitch_rate*dt
 	    gyro_roll += roll_rate*dt
 	    gyro_yaw += yaw_rate*dt

	    gyro_pitch_pub.publish(gyro_pitch)
	    gyro_roll_pub.publish(gyro_roll)
	    gyro_yaw_pub.publish(gyro_yaw)

	    self.rate.sleep()


    def acc_vel_publisher(self):
	
	while not rospy.is_shutdown():

	    cur_time = time.time()
	    dt = cur_time - self._last_time
            self._last_time_acc = cur_time

	    acc_velX += AccX*dt
 	    acc_velY += AccY*dt

	    acc_velX_pub.publish(acc_velX)
	    acc_velY_pub.publish(acc_velY)

	    self.rate.sleep()


if __name__ == '__main__':
    try:
        IMU_processor()
    except rospy.ROSInterruptException:
        pass

