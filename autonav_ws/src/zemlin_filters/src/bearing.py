from autonav_msgs.msg import GPSFeedback
import math


class BearingFilter:
    def __init__(self, latitudeLength, longitudeLength) -> None:
        self.last_gps = None
        self.last_bearing = None
        self.first_gps = None
        self.latitudeLength = latitudeLength
        self.longitudeLength = longitudeLength

    def reset(self):
        self.last_gps = None
        self.last_bearing = None
        self.first_gps = None
     
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # use the latitude and longitude lengths to calculate the distance
        # north_to_gps = (next_waypoint[0] - self.position.latitude) * self.config.latitude_length
        # west_to_gps = (self.position.longitude - next_waypoint[1]) * self.config.longitude_length
        north_to_gps = (lat2 - lat1) * self.latitudeLength
        west_to_gps = (lon1 - lon2) * self.longitudeLength
        return math.sqrt(north_to_gps**2 + west_to_gps**2)
     
    def gps(self, gps: GPSFeedback) -> tuple:
        if self.first_gps is None:
            self.first_gps = gps
        
        if self.last_gps is None:
            self.last_gps = gps
            return None, None, None  # Could also raise an exception or return None

        lat1 = self.last_gps.latitude
        lon1 = self.last_gps.longitude
        lat2 = gps.latitude
        lon2 = gps.longitude

        distance = self.calculate_distance(lat1, lon1, lat2, lon2)
        
        # Calculate the bearing
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (
            math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        )
        bearing = math.atan2(x, y) % (2 * math.pi)
        
        # calculate the distance in meters we have moved in x and y since the first gps
        # we are using the latitude and longitude lengths to calculate the distance
        north_to_gps = (gps.latitude - self.first_gps.latitude) * self.latitudeLength
        west_to_gps = (self.first_gps.longitude - gps.longitude) * self.longitudeLength
        x = north_to_gps
        y = west_to_gps
        
        if distance <= 0.05:
            # we barely moved so we don't want to update the bearing
            self.last_gps = gps
            return x, y, self.last_bearing
        
        self.last_gps = gps  # Update the last GPS
        self.last_bearing = bearing
        return x, y, bearing
