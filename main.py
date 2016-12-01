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



def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return(True)
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
    arm1 = Arm.Arm3Link(L = np.array([3,2,1]), x_displacement = 0, y_displacement = 0)
    for xy_pair in generate_n_random_xy_points(160):
        arm1.snap_arm_to_endpoint_position(xy_pair)
        list_of_line_segments = arm1.extract_line_segments()
        plot_one_position(list_of_line_segments, xy_pair, arm1.x_displacement, arm1.y_displacement)
        print(xy_pair)
        print(list_of_line_segments)

#put arms into position before calling this one
def intersection_between_arms(arm1,arm2):
    a1_lines = arm1.get_lines_with_C()
    a2_lines = arm2.get_lines_with_C()
    segment_indices = [0,1,2]
    res = [intersection(a1_lines[i], a2_lines[j]) for i in segment_indices for j in segment_indices]
    return(res)


def arms_intersecting_test():
    arm1 = Arm.Arm3Link(L = np.array([3,2,1]),x_displacement=0,y_displacement=0)
    arm1_taskpoint = arm1.snap_arm_to_new_XY_target()

    arm2 = Arm.Arm3Link(L = np.array([3,2,1]),x_displacement=0,y_displacement=0)
    arm2_taskpoint = arm2.snap_arm_to_new_XY_target()

    arm3 = Arm.Arm3Link(L = np.array([3,2,1]),x_displacement=0,y_displacement=0)
    arm3_taskpoint = arm3.snap_arm_to_new_XY_target()
    

    arm4 = Arm.Arm3Link(L = np.array([3,2,1]),x_displacement=0,y_displacement=0)
    arm4_taskpoint = arm4.snap_arm_to_new_XY_target()


    intersection_report = intersection_between_arms(arm1,arm2)
    print("intersection_report")
    print(intersection_report)
    print("arm1.extract_line_segments")
    print(arm1.extract_line_segments())
    print("arm2.extract_line_segments")
    print(arm2.extract_line_segments())
    plot_multiple_arms([
        (arm1,arm1_taskpoint,"black"),
        (arm2, arm2_taskpoint, "green"),
        (arm3, arm3_taskpoint, "grey"),
        (arm4, arm4_taskpoint, "purple")
        ])

#returns a tuple constructed with:
# (arm1_joint_x_vals,arm1_joint_y_vals)
# (arm1_x_target,arm1_y_target)
def get_vals_with_target(arm, arm_xy):
    arm_joint_x_vals = [arm.x_displacement] + [x[1][0] for x in arm.extract_line_segments()]
    arm_joint_y_vals = [arm.y_displacement] + [x[1][1] for x in arm.extract_line_segments()]
    arm_x_target = arm.x_displacement + arm_xy[0]
    arm_y_target = arm.y_displacement + arm_xy[1]
    return((arm_joint_x_vals,arm_joint_y_vals), (arm_x_target,arm_y_target))
    
def apply_arm_and_target_to_plt(arm, arm_xy, col):
    arm_joint_xy, arm_xy  = get_vals_with_target(arm,arm_xy)
    plt.plot(arm_joint_xy[0], arm_joint_xy[1], c=col)
    plt.scatter(arm_xy[0],arm_xy[1], s=80, marker='+')

def plot_multiple_arms(list_of_triples_of_arm_and_XY_and_col):
    plt.figure(time.time() * 1000)
    [apply_arm_and_target_to_plt(arm, arm_xy, col) for arm,arm_xy,col in list_of_triples_of_arm_and_XY_and_col]

    print("Just plotted ")
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    plt.savefig('output/file' + str(time.time() * 1000) + '.png')
    plt.close()



[arms_intersecting_test() for x in range(1000)]
# test_with_one_arm()
print('done')



