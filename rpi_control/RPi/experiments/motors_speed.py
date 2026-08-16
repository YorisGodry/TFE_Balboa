import importer
from main import Robot #type:ignore
from behavior import EmptyBehavior #type:ignore
from utils import Clock #type:ignore
import constants #type:ignore
from motors import LinRegMotors #type:ignore

current_command = 0
def set_speeds(robot, command):
    global current_command
    robot.motors.accelerate(-current_command, -current_command)
    robot.motors.accelerate(command, command)
    current_command = command
    robot.motors.update()

def save_data(file, robot):
    ld = round(robot.encoders.left.delta / constants.mm_per_count)
    rd = round(robot.encoders.right.delta / constants.mm_per_count)
    text = f"{current_command},{ld},{rd}"
    print(text)
    file.write(text + "\n")

behavior = EmptyBehavior()
robot = Robot(104, False, False)
#robot.motors = LinRegMotors(robot.balboa)
clock = Clock()
with open("lin_reg_motors_speed.csv", "w") as file:
    file.write("command,delta_left,delta_right\n")
    iteration = 0
    speeds = [100, 120, 140, 160, 180, 200, -100, -120, -140, -160, -180, -200]
    while True:
        robot.loop(behavior, 1)
        if iteration % 5 == 0:
            save_data(file, robot)
            set_speeds(robot, 0)
        elif iteration % 5 == 1:
            i = iteration // 5
            if i < len(speeds):
                set_speeds(robot, speeds[i])
        elif iteration % 5 == 2:
            pass
        else:
            save_data(file, robot)
        iteration += 1
        clock.add_wait_time(1)
        clock.wait()