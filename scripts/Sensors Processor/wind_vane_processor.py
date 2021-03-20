import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
import math

class Wind_processor():

    def __init__(self):
	
	apparent_wind_direction_pub = rospy.Publisher('wind_direction_apparent', Float64, queue_size=10)
	
	rospy.Subscriber('wind/mag', Vector3, self.get_mag)
	self.MagX = 0
	self.MagY = 0
	self.MagZ = 0

	i = 0
        average_time = rospy.get_param("wind/sensor_average_time")
        sensor_rate = rospy.get_param("config/rate")
        AVE_SIZE = int(average_time * sensor_rate)   # averaging over the last AVE_SIZE values
	self.average_list = [0] * AVE_SIZE 
       

    def get_mag(self,msg)
	
	self.MagX = msg.x
	self.MagY = msg.y
	self.MagZ = msg.z


    def apparent_wind_publisher(self):

	while not rospy.is_shutdown():

	    wind_direction = math.atan2(self.MagX, self.MagY)*(180/math.pi)
            wind_direction = (wind_direction - ANGLEOFFSET) % 360

            i = (i+1) % AVE_SIZE
            self.average_list[i] = wind_direction
            average_wind_direction = math.atan2(sum([ math.sin(x*math.pi/180) for x in average_list]),
            sum([ math.cos(x*math.pi/180) for x in average_list]))*180/math.pi % 360

            apparent_wind_direction_pub.publish(average_wind_direction)

	    self.rate.sleep()


if __name__ == '__main__':
    try:
	rospy.init_node("wind_processor", anonymous=True)
        Wind_processor()
    except rospy.ROSInterruptException:
        pass

