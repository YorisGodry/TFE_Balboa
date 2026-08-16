class Behavior:
    def __init__(self):
        raise NotImplementedError("Behavior should be considered as an interface/abstract class")
    def update(self, robot, delta_time):
        raise NotImplementedError()
    def reset(self):
        raise NotImplementedError()

class EmptyBehavior(Behavior):
    def __init__(self):
        pass
    def update(self, robot, delta_time):
        pass
    def reset(self):
        pass