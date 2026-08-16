
from importer import *

from behavior import EmptyBehavior #type:ignore
from path_tracker import Balancer, ReferencePointBehavior, Rotate #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import DeadMotors #type:ignore


if True: 
    # this behavior allows you to manually handle the robot
    # move it around in the air in all direction to gather data
    constants.set_iteration_per_second(10)
    def EmptyBehavior(robot):
        robot.motors = DeadMotors(robot.balboa)
        return Balancer()
    create_behavior = EmptyBehavior
else:
    # this behavior will make the robot balance and slowly turn on itself
    constants.max_rotational_speed = 2*pi / 60 # 1 rotation per 60 seconds
    constants.strafing_impacts_theta = False
    create_behavior = lambda robot: ReferencePointBehavior(Rotate(0, 4*pi))

def main(robot, file):
    behavior = create_behavior(robot)
    clock = Clock()
    set_columns(file, "phase,raw_mag_x,raw_mag_y,raw_mag_z,raw_acc_x,raw_acc_y,raw_acc_z,delta_encoder,mag_orient,theta")
    gather["delta_encoder"] = lambda robot: robot.encoders.right.value - robot.encoders.left.value
    gather["mag_orient"] = lambda robot: robot.magnetometer.get_orientation() / pi * 180
    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        robot.lis3mdl.read()
        write_columns(file, robot, True)
        #print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, True)
    robot.position.magnet_trust = 0
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()