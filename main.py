import time
import matplotlib as mpl
import matplotlib.pyplot as plt
import math
import numpy as np
import pyglet
import ipdb
import time
import Arm
import seaborn as sns

np.random.seed(sum(map(ord, "aesthetics")))

def generate_random_xy_point(xlim,ylim):
    x = np.random.uniform(xlim[0],xlim[1],1)
    y = np.random.uniform(ylim[0],ylim[1],1)
    return((x,y))
def generate_n_random_xy_points(n):
    return([generate_random_xy_point(xlim=[-500,500], ylim = [0,500]) for x in range(n)])

def line_segment(joint_positions, i):
        first_point = (joint_positions[0][i], joint_positions[1][i])
        second_point = (joint_positions[0][i+1], joint_positions[1][i+1])
        return(first_point, second_point)
def extract_line_segments(joint_positions):
        list_of_line_segments = [line_segment(joint_positions,i) for i in range(3)]
        return(list_of_line_segments)

def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return(A, B, -C)

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return(x,y)
    else:
        return(False)

def test_line_intersection():
    L1 = line([0,1], [2,3])
    L2 = line([2,3], [0,4])

    R = intersection(L1, L2)
    if R:
        print("Intersection detected:", R)
    else:
        print("No single intersection point detected")


def plot_one_position(line_segments):
    plt.figure(time.time() * 1000)
    x_vals = [320] + [x[1][0] for x in line_segments]
    y_vals = [0]   + [x[1][1] for x in line_segments]
    plt.plot(x_vals, y_vals)
    print("Just plotted ")
    plt.xlim([0,640])
    plt.ylim([0,640])
    plt.savefig('output/file' + str(time.time() * 1000) + '.png')
    plt.close()


def test_with_one_arm():
    arm1 = Arm.Arm3Link(L = np.array([300,200,100]))
    for xy_pair in generate_n_random_xy_points(100):
        arm1.snap_arm_to_endpoint_position(xy_pair)
        list_of_line_segments = extract_line_segments(arm1.get_joint_positions())
        plot_one_position(list_of_line_segments)
        print(list_of_line_segments)
   



test_with_one_arm()
print('done')



