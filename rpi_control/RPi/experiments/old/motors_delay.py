import importer
from main import Robot, EmptyBehavior #type:ignore
from utils import Clock #type:ignore
import constants #type:ignore

current_command = 0
def set_speeds(robot, command):
    global current_command
    robot.motors.accelerate(-current_command, -current_command)
    robot.motors.accelerate(command, command)
    current_command = command
    robot.motors.update()

behavior = EmptyBehavior()
robot = Robot(104)
clock = Clock()
with open("motors_delay.csv", "w") as file:
    file.write("command,delta_left,delta_right\n")
    iteration = 0
    while True:
        robot.loop(behavior, 1/100)
        ld = round(robot.encoders.left.delta / constants.mm_per_count)
        rd = round(robot.encoders.right.delta / constants.mm_per_count)
        text = f"{current_command},{ld},{rd}"
        print(text)
        file.write(text + "\n")
        if iteration == 100:
            set_speeds(robot, 100)
        elif iteration == 200:
            set_speeds(robot, 120)
        elif iteration == 300:
            set_speeds(robot, 140)
        elif iteration == 400:
            set_speeds(robot, 0)
        elif iteration == 500:
            set_speeds(robot, -100)
        elif iteration == 600:
            set_speeds(robot, -140)
        elif iteration == 700:
            set_speeds(robot, 0)
        iteration += 1
        clock.add_wait_time(1/100)
        clock.wait()