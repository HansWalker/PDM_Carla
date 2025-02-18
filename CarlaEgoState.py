import carla
import numpy as np

class CarlaEgoState:
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()

    def get_center(self):
        return self.vehicle.get_transform().location

    def get_distance_to(self, other_vehicle):
        return self.get_center().distance(other_vehicle.get_transform().location)

    def get_velocity(self):
        velocity = self.vehicle.get_velocity()
        return np.array([velocity.x, velocity.y])

    def get_acceleration(self):
        acceleration = self.vehicle.get_acceleration()
        return np.array([acceleration.x, acceleration.y])

    def get_angular_velocity(self):
        return self.vehicle.get_angular_velocity().z  # Only yaw rotation

    def get_angular_acceleration(self, prev_angular_velocity, dt):
        current_angular_velocity = self.get_angular_velocity()
        return (current_angular_velocity - prev_angular_velocity) / dt

    def get_rear_axle(self):
        return self.get_center()  # CARLA doesn’t model the rear axle

    def get_heading(self):
        return self.vehicle.get_transform().rotation.yaw

    def get_time_point(self):
        return self.world.get_snapshot().timestamp.elapsed_seconds

    def get_vehicle_parameters(self):
        physics = self.vehicle.get_physics_control()
        return {
            "mass": physics.mass,
            "wheelbase": physics.wheels[0].position.x - physics.wheels[2].position.x,
            "width": physics.wheels[0].position.y - physics.wheels[1].position.y,
            "height": self.vehicle.bounding_box.extent.z * 2
        }

    def serialize(self):
        return {
            "center": self.get_center(),
            "velocity": self.get_velocity(),
            "acceleration": self.get_acceleration(),
            "angular_velocity": self.get_angular_velocity(),
            "angular_acceleration": self.get_angular_acceleration(0, 0.1), 
            "rear_axle": self.get_rear_axle(),
            "heading": self.get_heading(),
            "time_point": self.get_time_point(),
            "vehicle_parameters": self.get_vehicle_parameters()
        }
