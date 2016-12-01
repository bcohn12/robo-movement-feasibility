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
    x = np.random.randint(xlim[0],xlim[1],1)
    y = np.random.randint(ylim[0],ylim[1],1)
    return((x,y))
def generate_n_random_xy_points(n):
    return([generate_random_xy_point(xlim=[-600,600], ylim = [-600,600]) for x in range(n)])

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


def plot_one_position(line_segments, xy_pair, x_displacement, y_displacement):
    plt.figure(time.time() * 1000)
    x_vals = [x_displacement] + [x[1][0] for x in line_segments]
    y_vals = [y_displacement]   + [x[1][1] for x in line_segments]
    x_target = x_displacement + xy_pair[0]
    y_target = y_displacement + xy_pair[1]
    plt.plot(x_vals, y_vals)
    plt.scatter(x_target,y_target, s=80, marker='+')
    print("Just plotted ")
    plt.xlim([-640,640])
    plt.ylim([-640,640])
    plt.savefig('output/file' + str(time.time() * 1000) + '.pdf')
    plt.close()

def test_with_one_arm():
    arm1 = Arm.Arm3Link(L = np.array([300,200,100]), x_displacement = 320, y_displacement = 0)
    for xy_pair in generate_n_random_xy_points(160):
        arm1.snap_arm_to_endpoint_position(xy_pair)
        list_of_line_segments = extract_line_segments(
            arm1.get_joint_positions()
            )
        plot_one_position(list_of_line_segments, xy_pair, arm1.x_displacement, arm1.y_displacement)
        print(xy_pair)
        print(list_of_line_segments)

#put arms into position before calling this one
def intersection_between_arms(arm1,arm2):
    a1_lines = arm1.get_lines_with_C()
    a2_lines = arm2.get_lines_with_C()
    segment_indices = [0,1,2]
    res = [intersection(arm1_segments[i], arm2_segments[j]) for i in segment_indices for j in segment_indices]
    return(res)    

def arms_intersecting_test():
    arm1 = Arm.Arm3Link(L = np.array([300,200,100]),x_displacement=320,y_displacement=0)
    arm2 = Arm.Arm3Link(L = np.array([300,200,100]),x_displacement=320,y_displacement=0)
    arm1_taskpoint = generate_n_random_xy_points(1)[0]
    arm2_taskpoint = generate_n_random_xy_points(1)[0]
    arm1.snap_arm_to_endpoint_position(arm1_taskpoint)
    arm2.snap_arm_to_endpoint_position(arm2_taskpoint)
    intersection_between_arms(arm1,arm2)
    ipdb.set_trace()


arms_intersecting_test()
test_with_one_arm()
print('done')



