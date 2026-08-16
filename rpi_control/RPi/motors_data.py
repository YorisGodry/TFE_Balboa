from main import *
from robot import AdjustedMotors

if __name__ == "__main__":
    balboa = Balboa()
    encoders = Encoders(balboa)
    motors = AdjustedMotors(balboa)

    iteration_number = 0

    try:
        with open("data.csv", "w") as file:
            file.write("iteration,motor,encoder_left,encoder_right\n")
            clock = Clock()
            motor = 0
            increasing = True
            while True:
                encoders.update(3)
                file.write(f"{iteration_number},{motor},{encoders.left.delta},{encoders.right.delta}\n")
                motors.accelerate(-motor, -motor)

                if increasing:
                    motor += 20
                else:
                    motor -= 20
                if abs(motor) == 100:
                    increasing = not increasing
                iteration_number += 1
                
                motors.accelerate(motor, motor)
                motors.update()
                clock.wait(3)
    except:
        motors.reset()