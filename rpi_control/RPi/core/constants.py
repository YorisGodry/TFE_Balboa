"""
A set of constants that are used throughout the code
e.g.: the size of the robot, the size of its wheels, the relative position of the decawave tag, 
    the number of iterations per second of the main control loop, the reference balancing angle 
    when stationary, some stabilisation delays, ...
"""
from math import pi

gyro_calibration_iterations = 100

# robot specific
robot_width = 0
    #mm (measured from the middle of one wheel to the middle of the other wheel)
wheel_diameter = 80 
    #mm
counts_per_wheel_revolution = 1012
    #count

# calculated using formulas
#wheel_circumference = pi * wheel_diameter 
    #mm
mm_per_count = pi * wheel_diameter / counts_per_wheel_revolution
    #mm/count
count_per_mm = 1 / mm_per_count
    #count/mm (for more intuitive conversion)
#robot_revolution_circumference = 2 * pi * robot_width 
    #mm (imagine a circle traced by one wheel when the other wheel is fixed to the ground)
counts_per_robot_revolution = 2 * robot_width * counts_per_wheel_revolution / wheel_diameter 
    #counts (= robot_revolution_circumference [mm] / mm_per_count [mm/count])


def set_robot_width(width):
    global robot_width, counts_per_robot_revolution
    robot_width = width
    counts_per_robot_revolution = 2 * robot_width * counts_per_wheel_revolution / wheel_diameter 


radian_to_count = 1/2/pi*counts_per_robot_revolution

max_speed = 400 #mm/sec
max_acceleration = 600 #mm/sec**2
max_ref_alpha = 6 / 180 * pi 
max_jerk = 4000 #mm/sec**3
max_rotational_speed = 4 #rad/sec
max_rotational_acceleration = 20 #rad/sec**2

iteration_per_second = 70
iteration_duration =  1 / iteration_per_second
def set_iteration_per_second(hz):
    global iteration_per_second, iteration_duration
    iteration_per_second = hz
    iteration_duration = 1 / hz

#init_delay_time = 2
#aim_to_go_delay = 0.5
#go_to_rotate_delay = 1
#rotate_to_aim_delay = 0.5
after_rotate_delay = 1 # 0.5
after_forward_delay = 3 # 1

activation_angle = 30 / 180 * pi
deactivation_angle = 70 / 180 * pi

dwm_position = (0, -50)

# alpha_offset represent the balancing angle of the robot
# this constant only makes it so the robot can stabilize faster
#   it depends on the angle of the floor (assuming it's completely flat)
#   and the position of the center of gravity of the robot 
#   (everything you put on the robot changes that)
# In the controllers I have, there is an integral term on the distance error
#   so the robot can dynamically (and implicitely) find its alpha_offset
alpha_offset = 11.2 / 180 * pi
stabilization_phase_delay = 6

strafing_impacts_theta = True # set to False if the robot is going to stay stationary for a while (e.g.: balancer)

def counts_to_radians(counts):
    return counts / (robot_width * count_per_mm)
