from importer import *

import constants #type:ignore
constants.after_rotate_delay = 0.5 # 1
constants.after_forward_delay = 1.5 # 3
constants.stabilization_phase_delay = 3 # 6
#constants.distance_integral_factor = ... #TODO remove sse from distance very slowly

from path_tracker import ReferencePointBehavior, PathTracker, Circle, SegmentedPlanner, OneSegment, Rose, SegmentedPath #type:ignore
from path_tracker import ReferencePointMovement, create_forward_profile, create_infinite_forward_profile #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors, DeadMotors #type:ignore


size = 400 if True else 1000

type_ = 5
do_circle, do_square, do_rose, do_note = tuple([i == type_ for i in range(4)])
do_catmull_rom = type_ == 4
do_a_to_b = type_ == 5



if do_circle:
    circle_path = Circle(0, size, size, -pi/2, 2*pi)
    circle = ReferencePointMovement(circle_path, create_forward_profile, True)
    planner = circle

if do_square:
    square = PathTracker([(size, 0), (size, size), (0, size), (0, 0)], None, True)
    planner = square

#pointy_circle = ...
#two_circles = ...

if do_rose:
    rose_path = Rose(size)
    #looping_rose_path = SegmentedPath([rose_path], True)
    #from path_tracker import set_forward_smoothness #type:ignore
    #set_forward_smoothness(2)
    #looping_rose_movement = ReferencePointMovement(looping_rose_path, create_infinite_forward_profile, True)
    rose_movement = ReferencePointMovement(rose_path, create_forward_profile, True)
    rose = SegmentedPlanner([lambda ref_point: OneSegment(size, 0, pi/2), lambda ref_point: rose_movement], False)
    #looping_rose = SegmentedPlanner([lambda ref_point: OneSegment(size, 0, pi/2), lambda ref_point: looping_rose_movement], False)
    planner = rose

if do_note:
    from path_tracker import Note, ReferencePoint #type:ignore
    constants.max_speed = 120
    note_path = Note(size)
    ref_point = ReferencePoint()
    minx, miny, maxx, maxy = 0, 0, 0, 0
    for i in range(1000):
        x, y = note_path.sample(i / 1000)
        minx, miny = min(x, minx), min(y, miny)
        maxx, maxy = max(x, maxx), max(y, maxy)
    print(minx, miny, maxx, maxy, sep=", ")
    note_path.update(ref_point, 0)
    note_x, note_y, note_theta = ref_point.x, ref_point.y, ref_point.theta
    print(ref_point.x, ref_point.y, ref_point.theta)
    note_movement = ReferencePointMovement(note_path, create_forward_profile, True)
    note = SegmentedPlanner([lambda ref_point: OneSegment(note_x, note_y, note_theta), lambda ref_point: note_movement], False)
    planner = note

if do_catmull_rom:
    from splines import SplinePath #type:ignore
    points = [(2, 1), (2, 2), (0, 3), (-0.5, 1), (-1, 5), (0, 4), (1, 4)]
    path, loop = SplinePath((0, 0), 0, [(300*x, 300*y) for x, y in points], None, False)
    #for segment in path:
    #    print(segment.points)
    #raise Exception
    constants.max_speed = 120
    path = SegmentedPath(path)
    planner = ReferencePointMovement(path, create_forward_profile, True)

if do_a_to_b:
    from splines import SplinePath #type:ignore
    path, loop = SplinePath((0, 0), 0, [(-300, 600)], None, False)
    path = SegmentedPath(path)
    planner = ReferencePointMovement(path, create_forward_profile, True)




def main(robot, file):
    behavior = ReferencePointBehavior(planner)
    clock = Clock()
    set_columns("phase,x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
    gather["ref_x"] = lambda robot: behavior.ref_point.x
    gather["ref_y"] = lambda robot: behavior.ref_point.y
    gather["ref_theta"] = lambda robot: behavior.ref_point.theta / pi * 180
    gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha / pi * 180

    def print_pretty(columns):
        columns = columns.split(",")
        for column in columns:
            print(column, gather[column](robot))

    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        write_columns(file, robot)
        #print_pretty("x,y,ref_x,ref_y,ref_alpha")
        print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, False)
    #robot.motors = LinRegMotors(robot.balboa)
    #robot.angle.gyro_trust = 0.97
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()