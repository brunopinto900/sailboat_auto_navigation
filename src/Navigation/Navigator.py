# Module to compute distances, load waypoints, etc

import math,mpmath, rospy
from LatLon import LatLon
from pyproj import Proj

class Navigator(object):

    def __init__(self):
	utm_zone = rospy.get_param('navigation/utm_zone')
	self.converter = Proj(proj='utm', zone=utm_zone, ellps='WGS84') # for debgug purposes only

    def latlon_to_utm(self, lat, lon):
        """Returns (x, y) coordinates in metres"""
        return self.converter(lon, lat)

    def utm_to_latlon(self, x, y):
        """Returns a LatLon object"""
        lon, lat = self.converter(x, y, inverse=True)
        return LatLon(lat, lon)

    def angle_subtract(self, goal, current): # to keep the angle between 180 and -180

	angle = (goal-current) % 360
        
        if angle > 180:
            angle = angle - 360

	if angle < -180:
	    angle = angle + 360

        return angle

    def angle_sum(self, x,y):
    
        return (x+y) % 360


    def get_point_of_sail(self, goal_heading, current_heading, wind_angle):

	angle = self.heading_relative_to_wind(goal_heading,wind_angle)
	
	if( (angle >= 0) and (angle <= 45) ):
	    return True, "tacking_to_starboard"
	if( (angle >= -45) and (angle < 0) ):
	    return True, "tacking_to_port"

	if( (angle <= 180) and (angle >= 135) ):
	    return True, "jibing_to_starboard"
	if( (angle <= -135) and (angle >= -180) ):
	    return True, "jibing_to_port"
	
	else:
	    return False, "normal"


    def angle_is_upwind(self, goal_heading, wind_angle):

	angle = self.heading_relative_to_wind(goal_heading,wind_angle)
	
	if( (angle >= 0) and (angle <= 45) ):
	    return True, "tacking_to_starboard"
	elif( (angle >= -45) and (angle < 0) ):
	    return True, "tacking_to_port"
	else:
	    return False, "normal"

    def angle_is_downwind(self, goal_heading, wind_angle):

	angle = self.heading_relative_to_wind(goal_heading,wind_angle)
        
	if( (angle <= 180) and (angle >= 135) ):
	    return True, "jibing_to_starboard"
	if( (angle <= -135) and (angle >= -180) ):
	    return True, "jibing_to_port"
	else:
	    return False, "normal"


    def load_gps_coordinates(self, gps_list, gps_coords):

        try:
            dictionary = dict(gps_coords)
	    coordinates = []
	    for i in gps_list:
		coordinates.append(dictionary[i])
            
        except KeyError:
	    print("Can't load gps")
            coordinates = []

	return coordinates

    def check_end_condition(self, goal_position, current_position, radius):
	
	if( self.calculate_distance(current_position, goal_position) < radius):
	    return True
	else:
	    return False


    def heading_relative_to_wind(self,compass_angle,wind_angle):

	angle = compass_angle - 180
	return self.angle_subtract(angle, wind_angle)

    def heading_opposite_to_wind(self, compass_relative,wind_angle):
	angle = self.angle_sum(compass_relative,180)
	if angle < 0:
	    angle = angle + 360
	return angle
	

    def convert_apparent_to_true(self, heading, AWA):
	TWA = self.angle_sum(heading, AWA)

	if TWA > 180:
            TWA -= 360
	    
	return TWA


    def calculate_goal_heading(self, pointA, pointB): #point is a vector
        angle = math.degrees(math.atan2( (pointB[1] - pointA[1]), (pointB[0] - pointA[0]) ) ) % 360
	return angle

    def calculate_distance(self,pointA, pointB):
	
	
	lon1,lat1=pointA
        lon2,lat2=pointB
        
        R=6371000                               # radius of Earth in meters
        phi_1=math.radians(lat1)
        phi_2=math.radians(lat2)

        delta_phi=math.radians(lat2-lat1)
        delta_lambda=math.radians(lon2-lon1)

        a=math.sin(delta_phi/2.0)**2 + math.cos(phi_1)*math.cos(phi_2) * math.sin(delta_lambda/2.0)**2
        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))
        
        meters=R*c                         # distance in meters
        
        return meters
