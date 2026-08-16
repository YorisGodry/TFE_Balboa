
from balboa import Balboa
from utils import clamp, sign

class Motors:
    def __init__(self, balboa: Balboa):
        self.left_speed = 0
        self.right_speed = 0
        self.left_temp = 0
        self.right_temp = 0
        self.balboa = balboa

        self.previous_left_speed = 0
        self.previous_right_speed = 0
    
    def accelerate(self, left, right):
        self.left_speed = clamp(self.left_speed + left, -300, 300)
        self.right_speed = clamp(self.right_speed + right, -300, 300)
    
    def accelerate_no_memory(self, left, right):
        self.left_temp += left
        self.right_temp += right
    
    def get_speeds(self):
        left = self.left_speed + self.left_temp
        right = self.right_speed + self.right_temp
        left, right = round(left), round(right)
        self.left_temp = 0
        self.right_temp = 0
        return clamp(left, -300, 300), clamp(right, -300, 300)
    
    def update(self):
        """
        Write the data from accelerate functions to balboa
        [reset = motors set to 0]
        """
        left, right = self.get_speeds()
        self.previous_left_speed, self.previous_right_speed = left, right
        self.balboa.update_motors(left, right)
    
    def reset(self):
        self.__init__(self.balboa)
        self.update()


class AdjustedMotors(Motors):
    adjustment = 16.5115453
    def get_speeds(self):
        left = self.left_speed + self.left_temp
        right = self.right_speed + self.right_temp

        left += sign(left) * self.adjustment
        right += sign(right) * self.adjustment

        left, right = round(left), round(right)
        self.left_temp = 0
        self.right_temp = 0
        return clamp(left, -300, 300), clamp(right, -300, 300)


class LinRegMotors(Motors):
    def __init__(self, balboa, deadzone=10):
        self.wdw = deadzone
        super().__init__(balboa)
    def adjust_commands(self, left, right):
        aw, arp, brp, alp, blp, arn, brn, aln, bln = 21.427619047619054, 21.05952380952381, -845.9841269841272, 21.795714285714297, -1064.4126984127001, 21.50761904761905, 789.5873015873017, 19.727142857142862, 527.126984126985
        aw, arp, brp, alp, blp, arn, brn, aln, bln = 23.6552380952381, 23.49666666666667, -605.666666666667, 23.81380952380953, -970.4603174603185, 25.392857142857157, 650.7619047619069, 20.966190476190487, 532.7619047619064

        wdw = self.wdw # wanted deadzone width
        if abs(left) < wdw:
            left = 0
        else:
            if left > 0:
                left -= wdw
            elif left < wdw:
                left += wdw
        if abs(right) < wdw:
            right = 0
        else:
            if right > wdw:
                right -= wdw
            elif right < wdw:
                right += wdw

        if left > 0:
            left = (aw * left - blp) / alp
            if left < 0:
                left = 0
        elif left < 0:
            left = (aw * left - bln) / aln
            if left > 0:
                left = 0
        if right > 0:
            right = (aw * right - brp) / arp
            if right < 0:
                right = 0
        elif right < 0:
            right = (aw * right - brn) / arn
            if right > 0:
                right = 0
        return left, right
    
    def get_speeds(self):
        left = self.left_speed + self.left_temp
        right = self.right_speed + self.right_temp

        left, right = self.adjust_commands(left, right)

        left, right = round(left), round(right)
        self.left_temp = 0
        self.right_temp = 0
        return clamp(left, -300, 300), clamp(right, -300, 300)


class DeadMotors(Motors):
    def get_speeds(self):
        return 0, 0