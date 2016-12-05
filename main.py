import datetime
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


from numpy import ones,vstack
from numpy.linalg import lstsq
# takes in the list of two points (as tuples)
# returns a tuple of the slope m and the intercept b s.t. y=mx+b
def get_slope_and_intercept(list_of_points = [(1,5),(3,4)]):
    x_coords, y_coords = zip(*list_of_points)
    A = vstack([x_coords,ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]
    return((m,c))

#via http://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
#modified so elements are put in as lists of 2 elements, not dictionaries.
def isBetween(a, b, c, epsilon=0.0001):
    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])
    if abs(crossproduct) > epsilon : return False   # (or != 0 if using integers)

    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
    if dotproduct < 0 : return False

    squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
    if dotproduct > squaredlengthba: return False

    return True

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    point1 = L1[3]
    point2 = L1[4]
    #if there is an intersection ever
    if D != 0:
        x = Dx / D
        y = Dy / D

        if isBetween(point1,point2,[x,y]):
            return(True)
        else:
            return(False)
    #totally co-linear or parallel
    else:
        return(False)

def test_line_intersection():
    p1 = [0,1], [2,3]
    p2 = [2,3], [0,4]
    L1 = line(p1)
    L2 = line(p2)

    R = intersection(L1, L2, p1, p2)
    if R!=False:
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
    for xy_pair in generate_n_random_xy_points(160, xlim=[-6,6], ylim=[-6,6]):
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



def arms_intersecting_test(plot=True, spacing = 100):

    x_start_displacement= -200
    x_center_vector = np.arange(5)*spacing + x_start_displacement

    arm1 = Arm.Arm3Link(L = np.array([50,10,10]),x_displacement=x_center_vector[0],y_displacement=0)
    arm1_taskpoint = arm1.snap_arm_to_new_XY_target(x_center=x_center_vector[0],y_center=0, outer_radius=70, inner_radius=0)

    arm2 = Arm.Arm3Link(L = np.array([50,10,10]),x_displacement=x_center_vector[1],y_displacement=0)
    arm2_taskpoint = arm2.snap_arm_to_new_XY_target(x_center=x_center_vector[1],y_center=0, outer_radius=70, inner_radius=0)

    arm3 = Arm.Arm3Link(L = np.array([50,10,10]),x_displacement=x_center_vector[2],y_displacement=0)
    arm3_taskpoint = arm3.snap_arm_to_new_XY_target(x_center=x_center_vector[2],y_center=0, outer_radius=70, inner_radius=0)

    arm4 = Arm.Arm3Link(L = np.array([50,10,10]),x_displacement=x_center_vector[3],y_displacement=0)
    arm4_taskpoint = arm4.snap_arm_to_new_XY_target(x_center=x_center_vector[3],y_center=0, outer_radius=70, inner_radius=0)

    arm5 = Arm.Arm3Link(L = np.array([50,10,10]),x_displacement=x_center_vector[4],y_displacement=0)
    arm5_taskpoint = arm5.snap_arm_to_new_XY_target(x_center=x_center_vector[4],y_center=0, outer_radius=70, inner_radius=0)

    num_intersections = sum([sum(intersection_between_arms(arm1,arm2)),
    sum(intersection_between_arms(arm1,arm3)),
    sum(intersection_between_arms(arm1,arm4)),
    sum(intersection_between_arms(arm1,arm5)),
    sum(intersection_between_arms(arm2,arm3)),
    sum(intersection_between_arms(arm2,arm4)),
    sum(intersection_between_arms(arm2,arm5)),
    sum(intersection_between_arms(arm3,arm4)),
    sum(intersection_between_arms(arm3,arm5)),
    sum(intersection_between_arms(arm4,arm5))])
    if plot==True:
        plot_multiple_arms([
            (arm1, arm1_taskpoint,"black"),
            (arm2, arm2_taskpoint, "green"),
            (arm3, arm3_taskpoint, "grey"),
            (arm4, arm4_taskpoint, "purple"),
            (arm5, arm5_taskpoint, "yellow")
            ])
    return(num_intersections)

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

def print_time_elapsed_from(initial_time):
    print(str(datetime.timedelta(seconds=time.time() - initial_time)))

def plot_multiple_arms(list_of_triples_of_arm_and_XY_and_col):
    plt.figure(time.time() * 1000)
    [apply_arm_and_target_to_plt(arm, arm_xy, col) for arm,arm_xy,col in list_of_triples_of_arm_and_XY_and_col]
    plt.xlim([-400,400])
    plt.ylim([-400,400])
    plt.savefig('output/file' + str(time.time() * 1000) + '.png')
    plt.close()


def timing_analysis():
    tic = time.time()
    intersection_values = [arms_intersecting_test(plot=False) for x in range(1)]
    print(intersection_values)
    print_time_elapsed_from(tic)

    tic = time.time()
    intersection_values = [arms_intersecting_test(plot=False) for x in range(10)]
    print(intersection_values)
    print_time_elapsed_from(tic)

    tic = time.time()
    intersection_values = [arms_intersecting_test(plot=False) for x in range(100)]
    print(intersection_values)
    print_time_elapsed_from(tic)

    tic = time.time()
    intersection_values = [arms_intersecting_test(plot=False) for x in range(1000)]
    print(intersection_values)
    print_time_elapsed_from(tic)
##



def collision_counts_per_anchor_distance(replicates):
    a500 = [arms_intersecting_test(plot=False, spacing = 500) for x in range(replicates)]
    a200 = [arms_intersecting_test(plot=False, spacing = 200) for x in range(replicates)]
    a100 = [arms_intersecting_test(plot=False, spacing = 100) for x in range(replicates)]
    a50  = [arms_intersecting_test(plot=False, spacing = 50) for x in range(replicates)]
    a20  = [arms_intersecting_test(plot=False, spacing = 20) for x in range(replicates)]
    a10  = [arms_intersecting_test(plot=False, spacing = 10) for x in range(replicates)]
    a5   = [arms_intersecting_test(plot=False, spacing = 5) for x in range(replicates)]
    a2   = [arms_intersecting_test(plot=False, spacing = 2) for x in range(replicates)]
    print("a500")    ; print(a500)
    print("a200")    ; print(a200)
    print("a100")    ; print(a100)
    print("a50 ")   ; print(a50 )
    print("a20 ")   ; print(a20 )
    print("a10 ")   ; print(a10 )
    print("a5  ")   ; print(a5  )
    print("a2  ")   ; print(a2  )
# collision_counts_per_anchor_distance(30)
timing_analysis()

# test_with_one_arm()
print('done')
print(intersection_values)



