import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from path_tracker import *

from utils import find_closest #type:ignore



# vectors

def add(p, q):
    return tuple([a + b for a, b in zip(p, q)])
def mult(p, a):
    return tuple([a * v for v in p])
def sub(p, q):
    return add(p, mult(q, -1))
def length(p):
    return sum([v ** 2 for v in p]) ** 0.5
def distance(p, q):
    return length(sub(p, q))
def vector(theta, length=1):
    return (length * cos(theta), length * sin(theta))
def angle(p):
    x, y = p
    return atan2(y, x)

def poll_(coefs, value):
    s = 0
    for i in range(len(coefs)):
        s += coefs[i] * (value ** i)
    return s
def poll(points, value):
    xs = [x for x, y in points]
    ys = [y for x, y in points]
    return (poll_(xs, value), poll_(ys, value))

# bezier spline

class Bezier(ParametricCurvePath):
    def __init__(self, p0, p1, p2, p3):
        self.points = [p0, p1, p2, p3]
        self.matrix = [[-1, 3, -3, 1],
                       [3, -6, 3, 0],
                       [-3, 3, 0, 0],
                       [1, 0, 0, 0]]

        self.coefs = []
        for i in range(4):
            coef_def = False
            for p, v in zip(self.points, self.matrix[3 - i]):
                current = mult(p, v)
                if not coef_def:
                    coef = current
                else:
                    coef = add(coef, current)
                coef_def = True
            self.coefs.append(coef)

        #self.derivative_matrix = [[-1*3, 3*3, -3*3, 1*3],
        #                          [3*2, -6*2, 3*2, 0],
        #                          [-3, 3, 0, 0]]
        #self.derivative_coefs = []
        #for i in range(3):
        #    coef_def = False
        #    for p, v in zip(self.points, self.derivative_matrix[2 - i]):
        #        current = mult(p, v)
        #        if not coef_def:
        #            coef = current
        #        else:
        #            coef = add(coef, current)
        #        coef_def = True
        #    self.derivative_coefs.append(coef)
        #print(self.derivative_coefs)

        self.derivative_coefs = [mult(self.coefs[i + 1], i + 1) for i in range(len(self.coefs) - 1)]
        #print(self.derivative_coefs)

        super().__init__()
    def sample(self, ratio):
        return poll(self.coefs, ratio)
    def sample_derivative(self, ratio):
        return poll(self.derivative_coefs, ratio)

    
# conversions

def Hermite(p0, t0, p3, t3):
    return Bezier(p0, add(p0, mult(t0, 1/3)), add(p3, mult(t3, -1/3)), p3)
def CatmullRom(p_minus, p0, p3, p_plus, alpha):
    # see paper https://people.engr.tamu.edu/schaefer/research/catmull_rom.pdf 
    # ("On the Parameterization of Catmull-Rom Curves" by Cem Yuksel et al.)
    d1 = distance(p_minus, p0)
    d2 = distance(p0, p3)
    d3 = distance(p3, p_plus)
    t1 = pow(d1, alpha)
    ts1 = t1*t1
    t2 = pow(d2, alpha)
    ts2 = t2*t2
    t3 = pow(d3, alpha)
    ts3 = t3*t3
    b0 = p0
    b1 = mult(p3, ts1)
    b1 = add(b1, mult(p_minus, -ts2))
    b1 = add(b1, mult(p0, 2*ts1 + 3*t1*t2 + ts2))
    b1 = mult(b1, 1/( 3*t1*(t1 + t2) ))
    b2 = mult(p0, ts3)
    b2 = add(b2, mult(p_plus, -ts2))
    b2 = add(b2, mult(p3, 2*ts3 + 3*t3*t2 + ts2))
    b2 = mult(b2, 1/( 3*t3*(t3 + t2) ))
    b3 = p3
    return Bezier(b0, b1, b2, b3)
def UniformCatmullRom(p_minus, p0, p3, p_plus):
    return CatmullRom(p_minus, p0, p3, p_plus, 0.0)
def CentripetalCatmullRom(p_minus, p0, p3, p_plus):
    return CatmullRom(p_minus, p0, p3, p_plus, 0.5)
def CordalCatmullRom(p_minus, p0, p3, p_plus):
    return CatmullRom(p_minus, p0, p3, p_plus, 1.0)

def Spline(p0, theta0, p3, theta3):
    d = distance(p3, p0)
    return Hermite(p0, vector(theta0, d/2), p3, vector(theta3, d/2))
def SplinePath(p0, theta0, points, thetas, on_loop):
    assert len(points) >= 1
    assert not on_loop or len(points) > 1, "Impossible to loop over a single point"
    assert thetas is None or len(thetas) == len(points)
    # return (init path segments, looping segments)
    init_path, looping_path = [], []
    if thetas is None: 

        ### Centripedal Catmull-Rom splines
        if on_loop:
            path12 = CentripetalCatmullRom(p0, points[0], points[1], points[2 % len(points)])
            path01 = Hermite(p0, vector(theta0, distance(points[0], p0) / 2), points[0], path12.sample_derivative(0))
            init_path.append(path01)
            init_path.append(path12) # initpath: go to points[1]
            for i in range(1, len(points) + 1): # loopingpath: go from points[1] back to points[1]
                p_minus = points[i - 1]
                p0 = points[i % len(points)]
                p3 = points[(i + 1) % len(points)]
                p_plus = points[(i + 2) % len(points)]
                looping_path.append(CentripetalCatmullRom(p_minus, p0, p3, p_plus))
            return init_path, looping_path
        else:
            if len(points) <= 2:
                # not enough points for even a single segment of catmullrom
                return SplinePath(p0, theta0, points, [None for p in points], on_loop)
            path12 = CentripetalCatmullRom(p0, points[0], points[1], points[2])
            path01 = Hermite(p0, vector(theta0, distance(points[0], p0) / 2), points[0], path12.sample_derivative(0))
            init_path.append(path01)
            init_path.append(path12)
            
            for i in range(1, len(points) - 2):
                path = CentripetalCatmullRom(points[i - 1], points[i], points[i + 1], points[i + 2])
                init_path.append(path)

            onetolastpoint = points[len(points) - 2]
            lastpoint = points[len(points) - 1]
            theta = angle(sub(lastpoint, onetolastpoint))
            derivative = vector(theta, distance(onetolastpoint, lastpoint) / 2)
            lastpath = Hermite(onetolastpoint, path.sample_derivative(1), lastpoint, derivative)
            init_path.append(lastpath)
            return init_path, looping_path

    else: 

        ### Create thetas for Hermite splines
        # use thetas given but fill in the None values if any
        def make_theta(p0, p1, p2=None):
            if p2 is not None:
                first = angle(sub(p1, p0))
                second = angle(sub(p2, p1))
                second = find_closest(first, second, 2*pi)
                delta = second - first
                theta = first + delta/2 # perpendicular to bissector of 01 and 12
            else:
                theta = angle(sub(p1, p0)) # straight line angle of 01
            return theta
        
        if on_loop:
            first_theta = thetas[0]
            if first_theta is None:
                first_theta = make_theta(p0, points[0], points[1])
            init_path.append(Spline(p0, theta0, points[0], first_theta)) # p0 to points[0]
            second_theta = thetas[1]
            if second_theta is None:
                second_theta = make_theta(points[0], points[1], points[2 % len(points)])
            init_path.append(Spline(points[0], first_theta, points[1], second_theta)) # points[0] to points[1]
            for i in range(1, len(points) + 1): # points[1] to points[1]
                p0 = points[i % len(points)]
                p3 = points[(i + 1) % len(points)]
                theta0 = thetas[i % len(points)]
                if theta0 is None:
                    theta0 = make_theta(points[i - 1], p0, p3)
                theta3 = thetas[(i + 1) % len(points)]
                if theta3 is None:
                    theta3 = make_theta(p0, p3, points[(i + 2) % len(points)])
                looping_path.append(Spline(p0, theta0, p3, theta3))
            return init_path, looping_path
        else:
            # from p0 to points[0]
            first_theta = thetas[0]
            if first_theta is None:
                if len(points) == 1:
                    first_theta = make_theta(p0, points[0])
                else:
                    first_theta = make_theta(p0, points[0], points[1])
            init_path.append(Spline(p0, theta0, points[0], first_theta))
            if len(points) == 1:
                return init_path, looping_path
            # from points[0] to points[1]
            second_theta = thetas[1]
            if second_theta is None:
                if len(points) == 2:
                    second_theta = make_theta(points[0], points[1])
                else:
                    second_theta = make_theta(points[0], points[1], points[2])
            init_path.append(Spline(points[0], first_theta, points[1], second_theta))
            if len(points) == 2:
                return init_path, looping_path
            # from points[1] to points[len(points) - 2] # can't be last segment
            for i in range(1, len(points) - 2):
                # from i to i+1
                theta0 = thetas[i]
                if theta0 is None:
                    theta0 = make_theta(points[i - 1], points[i], points[i + 1])
                theta3 = thetas[i + 1]
                if theta3 is None:
                    theta3 = make_theta(points[i], points[i + 1], points[i + 2])
                init_path.append(Spline(points[i], theta0, points[i + 1], theta3))
            # from points[len(points) - 2] to points[len(points) - 1] # is last segment
            theta0 = thetas[-2]
            if theta0 is None:
                theta0 = make_theta(points[-3], points[-2], points[-1])
            theta3 = thetas[-1]
            if theta3 is None:
                theta3 = make_theta(points[-2], points[-1])
            init_path.append(Spline(points[-2], theta0, points[-1], theta3))
            return init_path, looping_path