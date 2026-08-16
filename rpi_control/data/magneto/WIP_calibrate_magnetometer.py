import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *

from utils import find_closest #type:ignore
from math import sqrt


# to (more or less) align with my phone's compass (just for convenience)
mag_offset = -25

def mean(l):
    return sum(l) / len(l)
def mean_or_zero(l):
    return sum(l) / max(len(l), 1)

def variance(l):
    m = mean(l)
    r = []
    for v in l:
        r.append((v - m) ** 2)
    return mean(r)

def standard_deviation(l):
    return sqrt(variance(l))

def get_data(filename, mag_offset):
    input, output = get_filenames(filename, __file__)
    columns, length = read_csv(input)
    data = columns_to_dict(columns)

    phase = list(map(int, data["phase"]))
    mag = list(map(float, data["mag_orient"]))
    theta = list(map(float, data["theta"]))

    # to (more or less) align with my phone's compass (just for convenience)
    mag = [m + mag_offset for m in mag]

    def smooth_modulo(l, modulo):
        r = [l[0]]
        for value in l[1:]:
            value = find_closest(r[-1], value, modulo)
            r.append(value)
        return r
    
    def moving_average(l, window_width):
        padding = [0 for i in range(window_width)]
        r = []
        for i in range(window_width, len(l) - window_width):
            sum_ = 0
            for j in range(-window_width, window_width + 1):
                sum_ += l[i + j]
            r.append(sum_ / (window_width * 2 + 1))
        return padding + r + padding

    mag = smooth_modulo(mag, 360)
    mag = moving_average(mag, 20)
    theta = moving_average(theta, 20)

    for start_index in range(length):
        if phase[start_index] == 3:
            break
    s, e = start_index + 30*70, start_index + 90*70

    time = [1/70 * i for i in range(s, e)]
    mag = mag[s:e]
    theta = theta[s:e]

    return time, mag, theta

def to_bucket_index(m, n_buckets):
    m = (m % 360) / 360
    m = int(m * n_buckets)
    return m

def create_buckets(n_buckets):
    return [[] for i in range(n_buckets)]

def add_to_buckets(buckets, mag, errors):
    for m, e in zip(mag, errors):
        m = to_bucket_index(m, len(buckets))
        buckets[m].append(e)

def correct(m, buckets):
    bucket = buckets[to_bucket_index(m, len(buckets))]
    return m - mean_or_zero(bucket)

def show_stats(errors):
    print(f"mean: {round(mean(errors), 3)},",
          f"std.dev.: {round(standard_deviation(errors), 3)},",
          f"mean of abs: {round(mean([abs(e) for e in errors]), 3)}")
    
fig = plt.figure()
ax = fig.add_subplot()

def main(filename, buckets, training, verbose):
    time, mag, theta = get_data(filename, mag_offset)
    if not training:
        mag = [correct(m, buckets) for m in mag]
    mean_mag, mean_theta = mean(mag), mean(theta)
    theta = [t - mean_theta + mean_mag for t in theta]
    errors = [m - t for m, t in zip(mag, theta)]
    if training:
        add_to_buckets(buckets, mag, errors)
    if verbose:
        show_stats(errors)
    return time, mag, theta
    
def train(filename, buckets, verbose=True):
    return main(filename, buckets, True, verbose)

def test(filename, buckets, verbose=True):
    return main(filename, buckets, False, verbose)

def cross_validation(test_filename, filenames):
    buckets = create_buckets(int(get_arg()))
    print("\t" + test_filename)
    print("Before:", end="\t\t")
    test(test_filename, buckets)
    for filename in filenames:
        if filename == test_filename:
            continue
        train(filename, buckets, False)
    print("After:", end="\t\t")
    test(test_filename, buckets)

filenames = ["ccw_south", "ccw_north", "ccw_west"]
for filename in filenames:
    cross_validation(filename, filenames)


if False:
    for testing_filename in filenames:
        
        buckets = create_buckets(int(get_arg()))
        print(f"Stats for {testing_filename} without training:")
        test(testing_filename, buckets)

        for filename in filenames:
            if filename == testing_filename:
                continue
            time, mag, theta = train(filename, buckets)
            ax.plot(time, mag, label="mag "+filename)
            ax.plot(time, theta, label="theta "+filename)
        for filename in filenames:
            if filename == testing_filename:
                print(f"Stats for {filename} without using it in training:")
            time, mag, theta = test(filename, buckets)
            ax.plot(time, mag, label="corrected mag "+filename)

    ax.legend()
    plt.show()