from autonav_msgs.msg import MotorFeedback, GPSFeedback, ParticleFilterDebug, Particle as ParticleMsg
import numpy as np
import math
import random

class Particle:
    def __init__(self, x = 0, y = 0, theta = 0, weight = 1) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class ParticleFilter:
    def __init__(self, latitudeLength, longitudeLength) -> None:
        self.num_particles = 750
        self.gps_noise = [0.45]
        self.particles = []
        self.odom_noise = [0, 0, 0.1]
        self.init_particles()
        self.first_gps = None
        self.last_feedback = None
        
        self.latitudeLength = latitudeLength
        self.longitudeLength = longitudeLength

    def init_particles(self, seedHeading: float = 0.0, useSeedHeading: bool = False):
        if useSeedHeading:
            self.particles = [Particle(0, 0, seedHeading + np.random.normal(0, 0.1)) for i in range(self.num_particles)]
        else:
            self.particles = [Particle(0, 0, i / self.num_particles * 2 * math.pi) for i in range(self.num_particles)]

    def feedback(self, feedback: MotorFeedback) -> list[float]:
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0
        
        for particle in self.particles:
            # particle.x += feedback.delta_x * 0.95 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            # particle.y += feedback.delta_x * 0.95 * math.sin(particle.theta) + feedback.delta_y * math.cos(particle.theta)
            particle.x += feedback.delta_x
            particle.y += feedback.delta_y
            particle.theta += feedback.delta_theta
            particle.theta = particle.theta % (2 * math.pi)
            weight = particle.weight ** 2
            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += math.cos(particle.theta) * weight
            sum_theta_y += math.sin(particle.theta) * weight
            sum_weight += weight
            
        if sum_weight < 0.000001:
            sum_weight = 0.000001
            
        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = math.atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * math.pi)
        
        self.last_feedback = [avg_x, avg_y, avg_theta]
        return [avg_x, avg_y, avg_theta]
    
    def clamp_between(self, value: float, min_value: float, max_value: float) -> float:
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value
    
    def get_particle_feedback(self) -> ParticleFilterDebug:
        if self.last_feedback is None:
            return None
        
        msg = ParticleFilterDebug()
        msg.x = self.last_feedback[0]
        msg.y = self.last_feedback[1]
        msg.theta = self.last_feedback[2]
        msg.particles = []
        for particle in self.particles:
            msg.particles.append(ParticleMsg())
            msg.particles[-1].x = self.clamp_between(particle.x, -1000, 1000)
            msg.particles[-1].y = self.clamp_between(particle.y, -1000, 1000)
            msg.particles[-1].theta = self.clamp_between(particle.theta, -1000, 1000)
            msg.particles[-1].weight = self.clamp_between(particle.weight, -1000, 1000)

        return msg
     
    def gps(self, gps: GPSFeedback) -> list[float]:
        if self.first_gps is None:
            self.first_gps = gps
            
        gps_x = (gps.latitude - self.first_gps.latitude) * self.latitudeLength
        gps_y = (self.first_gps.longitude - gps.longitude) * self.longitudeLength
    
        for particle in self.particles:
            dist_sqrt = np.sqrt((particle.x - gps_x) ** 2 + (particle.y - gps_y) ** 2)
            particle.weight = math.exp(-dist_sqrt / (2 * self.gps_noise[0] ** 2))
            
        self.resample()
        return [gps_x, gps_y]
    
    def resample(self) -> None:
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        if weights_sum <= 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]
        
        new_particles = random.choices(self.particles, weights, k = self.num_particles)
        self.particles = []
        
        for particle in new_particles:
            rand_x = np.random.normal(0, self.odom_noise[0])
            rand_y = np.random.normal(0, self.odom_noise[1])
            x = particle.x + rand_x * math.cos(particle.theta) + rand_y * math.sin(particle.theta)
            y = particle.y + rand_x * math.sin(particle.theta) + rand_y * math.cos(particle.theta)
            theta = np.random.normal(particle.theta, self.odom_noise[2]) % (2 * math.pi)
            self.particles.append(Particle(x, y, theta, particle.weight))