from math import sin, cos
import constants


class Position:
    """ Only uses relative position (start at 0,0,0 on robot startup) """

    def __init__(self):
        self.x = 0.0 # mm
        self.y = 0.0 # mm
        self.theta = 0.0 # radians
    
    def reset(self):
        self.x = 0.0 # mm
        self.y = 0.0 # mm
        self.theta = 0.0 # radians
    
    def get_relative(self):
        return self.x, self.y, self.theta

    def update(self, robot, delta_time):
        self.update_encoders(robot.encoders.left.delta, robot.encoders.right.delta)

    def update_encoders(self, left, right):
        """
        Updates the position of the robot based on left and right
        'left' and 'right' have to be the number of counts each wheel 
            has traveled since the last update
        [reset = set x, y and theta to 0]
        """
        _, _, theta = self.get_relative()
        dx, dy, dtheta = 0, 0, 0
        if left == right:
            dx = left * cos(theta)
            dy = left * sin(theta)
        else:
            alpha = (right - left) / constants.robot_width
            D = left / alpha
            d = D + constants.robot_width / 2

            dx = d * (-sin(theta) + sin(theta + alpha))
            dy = d * (cos(theta) - cos(theta + alpha))
            dtheta = alpha
        self.x += dx
        self.y += dy
        self.theta += dtheta


class AbsolutePosition(Position):
    """ Adds the posibility of setting an absolute initial position
        Adds an initialisation delay (to be used however you want) """

    def __init__(self):
        super().__init__()
        self.init_x, self.init_y = 0, 0
        self.init_theta = 0
        self.absolute_offset_set = False
    
    def set_absolute_offset(self, x, y, theta):
        # keep the relative position unchanged
        self.x += x
        self.y += y
        self.theta += theta
        self.init_x, self.init_y = x, y
        self.init_theta = theta
        self.absolute_offset_set = True
    
    def get_absolute(self):
        return (self.x, self.y, self.theta)
    
    def get_relative(self):
        return (self.x - self.init_x, self.y - self.init_y, self.theta - self.init_theta)
    
    def reset(self):
        super().reset()
        self.init_x, self.init_y = 0, 0
        self.init_theta = 0
        self.absolute_offset_set = False
    
    def update(self, robot, delta_time):
        self.update_encoders(robot.encoders.left.delta, robot.encoders.right.delta)
        if not self.is_initialized():
            self.initialization_update(robot, delta_time)
    
    def initialization_update(self, robot, delta_time):
        pass

    def is_initialized(self):
        return True
