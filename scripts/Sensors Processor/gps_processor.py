import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from sensor_msgs.msg import Odometry
import math

class GPS_processor():

    def __init__(self):
	
	rospy.init_node("GPS_processor", anonymous=True)
	self.freq = rospy.get_param("config/rate")
        self.rate = rospy.Rate(self.freq)
	
	self.gps_odom_pub = rospy.Publisher('odom', Odometry, queue_size = 10)
	rospy.Subscriber('position', NavSatFix, self.get_GPS)
        self.gps_x = 0
        self.gps_y = 0
        self.gps_z = 0
        self.pos_covar = rospy.get_param("simulation/gps/position_covariance")
	self.cox_x = pos_covar[0]
	self.cox_y = pos_covar[4]
	self.cox_z = pos_covar[8]
    
        rospy.Subscriber('gps_fix', gpswtime, self.get_Time)


    def get_Time(self,msg):

	self.gps_time = gpswtime()
	self.gps_time = msg

    def get_GPS(self,msg):

	gps_x = msg.longitude
	gps_y = msg.latitude
	gps_z = msg.altitude
	
    def gps_odom_publisher(self):
	
	while not rospy.is_shutdown():
	
	    odom = Odometry()
	    odom.header.stamp = self.gps_time                   // time of gps measurement
 	    odom.header.frame_id = "gps_measurements"      // the tracked robot frame
 	    odom.pose.pose.position.x = self.gps_x              // x measurement GPS.
 	    odom.pose.pose.position.y = self.gps_y              // y measurement GPS.
 	    odom.pose.pose.position.z = self.gps_z              // z measurement GPS.
 	    odom.pose.pose.orientation.x = 1               // identity quaternion
 	    odom.pose.pose.orientation.y = 0               // identity quaternion
 	    odom.pose.pose.orientation.z = 0               // identity quaternion
 	    odom.pose.pose.orientation.w = 0               // identity quaternion
 	    odom.pose.covariance = {self.cox_x, 0, 0, 0, 0, 0,  // covariance on gps_x
                        0, self.cov_y, 0, 0, 0, 0,  // covariance on gps_y
                        0, 0, self.cov_z, 0, 0, 0,  // covariance on gps_z
                        0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                        0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                        0, 0, 0, 0, 0, 99999}  // large covariance on rot z


	    gps_odom_pub.publish(odom)

	    self.rate.sleep()	

if __name__ == '__main__':
    try:
        GPS_processor()
    except rospy.ROSInterruptException:
        pass

