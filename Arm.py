#via http://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
from __future__ import division 
import ipdb
'''
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import math
import numpy as np
import scipy.optimize


##helper functions for random point generation.
def generate_random_xy_point(xlim,ylim):
    x = np.random.randint(xlim[0],xlim[1],1)
    y = np.random.randint(ylim[0],ylim[1],1)
    return((x,y))

def generate_n_random_xy_points(n, xlim, ylim):
    return([generate_random_xy_point(xlim=xlim, ylim = ylim) for x in range(n)])

def point_is_in_circle(prospective_xy_point, outer_radius):
    x = prospective_xy_point[0]
    y = prospective_xy_point[1]
    result = x**2 + y**2 < outer_radius**2
    return(result)

def uar_sample_from_circle(outer_radius):
    while True:            
        x = np.random.uniform(-outer_radius,outer_radius)
        y = np.random.uniform(-outer_radius,outer_radius)
        if point_is_in_circle([x,y], outer_radius):
            return(x,y)

def xy_not_in_circle(x,y,radius):
    if point_is_in_circle((x,y),radius):
        return(False)
    else:
        return(True)

# warning - if the outer and inner are very close together, it will get incredibly slow. if you want to make it faster, reimplement with something like this:
# https://bl.ocks.org/mbostock/d8b1e0a25467e6034bb9
def uar_sample_from_annulus(outer_radius,inner_radius):
    while 1==1:
        x,y = uar_sample_from_circle(outer_radius)
        if xy_not_in_circle(x,y,inner_radius):
            return(x,y)
        else:
            pass #keep collecting points!


class Arm3Link:

    def __init__(self, q=None, q0=None, L=None, x_displacement=None, y_displacement=None):
        """Set up the basic parameters of the arm.
        All lists are in order [shoulder, elbow, wrist].

        q : np.array
            the initial joint angles of the arm
        q0 : np.array
            the default (resting state) joint configuration
        L : np.array
            the arm segment lengths

        x_displacement: 
        y_displacement
        """
        # initial joint angles
        self.q = [math.pi/4, math.pi/4, 0] if q is None else q
        # some default arm positions
        self.q0 = np.array([math.pi/4, math.pi/4, 0]) if q0 is None else q0
        # arm segment lengths
        self.L = np.array([1, 1, 1]) if L is None else L
        self.max_angles = [2*math.pi, 2*math.pi, 2*math.pi]
        self.min_angles = [-2*math.pi, -2*math.pi, -2*math.pi]
        self.x_displacement = 0 if x_displacement is None else x_displacement
        self.y_displacement = 0 if y_displacement is None else y_displacement


    def snap_arm_to_endpoint_position(self, xy_endpoint_position_tuple):
        self.q = self.inv_kin([xy_endpoint_position_tuple[0], xy_endpoint_position_tuple[1]])
    
    def snap_arm_to_new_XY_target(self, x_center,y_center, outer_radius, inner_radius):
        xy = uar_sample_from_annulus(outer_radius, inner_radius)
        self.snap_arm_to_endpoint_position(xy)
        return(xy)

    def get_joint_positions(self):
        """This method finds the (x,y) coordinates of each joint"""
         #set to match original frames
        x = np.array([ 0, 
            self.L[0]*np.cos(self.q[0]),
            self.L[0]*np.cos(self.q[0]) + self.L[1]*np.cos(self.q[0]+self.q[1]),
            self.L[0]*np.cos(self.q[0]) + self.L[1]*np.cos(self.q[0]+self.q[1]) + 
                self.L[2]*np.cos(np.sum(self.q)) ]) + self.x_displacement
        y = np.array([ 0, 
            self.L[0]*np.sin(self.q[0]),
            self.L[0]*np.sin(self.q[0]) + self.L[1]*np.sin(self.q[0]+self.q[1]),
            self.L[0]*np.sin(self.q[0]) + self.L[1]*np.sin(self.q[0]+self.q[1]) + 
                self.L[2]*np.sin(np.sum(self.q)) ]) + self.y_displacement
        return(np.array([x, y]).astype('int'))


    def extract_line_segments(self):

        def line_segment(joint_positions, i):
            first_point = (joint_positions[0][i], joint_positions[1][i])
            second_point = (joint_positions[0][i+1], joint_positions[1][i+1])
            return(first_point, second_point)

        list_of_line_segments = [line_segment(self.get_joint_positions(),i) for i in range(3)]
        return(list_of_line_segments)


    def get_lines_with_C(self):

        def line(p1, p2):
            A = (p1[1] - p2[1])
            B = (p2[0] - p1[0])
            C = (p1[0]*p2[1] - p2[0]*p1[1])
            return(A, B, -C, p1, p2)

        lines = [line(x[0],x[1]) for x in self.extract_line_segments()]
        return(lines)

    def get_xy(self, q=None):
        """Returns the corresponding hand xy coordinates for
        a given set of joint angle values [shoulder, elbow, wrist],
        and the above defined arm segment lengths, L

        q : np.array
            the list of current joint angles

        returns : list
            the [x,y] position of the arm
        """
        if q is None:
            q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, xy):
        """This is just a quick write up to find the inverse kinematics
        for a 3-link arm, using the SciPy optimize package minimization
        function.

        Given an (x,y) position of the hand, return a set of joint angles (q)
        using constraint based minimization, constraint is to match hand (x,y),
        minimize the distance of each joint from it's default position (q0).

        xy : tuple
            the desired xy position of the arm

        returns : list
            the optimal [shoulder, elbow, wrist] angle configuration
        """

        def distance_to_default(q, *args):
            """Objective function to minimize
            Calculates the euclidean distance through joint space to the
            default arm configuration. The weight list allows the penalty of
            each joint being away from the resting position to be scaled
            differently, such that the arm tries to stay closer to resting
            state more for higher weighted joints than those with a lower
            weight.

            q : np.array
                the list of current joint angles

            returns : scalar
                euclidean distance to the default arm position
            """
            # weights found with trial and error,
            # get some wrist bend, but not much
            weight = [1, 1, 1.3]
            return np.sqrt(np.sum([(qi - q0i)**2 * wi
                           for qi, q0i, wi in zip(q, self.q0, weight)]))

        def x_constraint(q, xy):
            """Returns the corresponding hand xy coordinates for
            a given set of joint angle values [shoulder, elbow, wrist],
            and the above defined arm segment lengths, L

            q : np.array
                the list of current joint angles
            xy : np.array
                current xy position (not used)

            returns : np.array
                the difference between current and desired x position
            """
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - xy[0]
            return x

        def y_constraint(q, xy):
            """Returns the corresponding hand xy coordinates for
            a given set of joint angle values [shoulder, elbow, wrist],
            and the above defined arm segment lengths, L

            q : np.array
                the list of current joint angles
            xy : np.array
                current xy position (not used)
            returns : np.array
                the difference between current and desired y position
            """
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q))) - xy[1]
            return y

        def joint_limits_upper_constraint(q, xy):
            """Used in the function minimization such that the output from
            this function must be greater than 0 to be successfully passed.

            q : np.array
                the current joint angles
            xy : np.array
                current xy position (not used)

            returns : np.array
                all > 0 if constraint matched
            """
            return self.max_angles - q

        def joint_limits_lower_constraint(q, xy):
            """Used in the function minimization such that the output from
            this function must be greater than 0 to be successfully passed.

            q : np.array
                the current joint angles
            xy : np.array
                current xy position (not used)

            returns : np.array
                all > 0 if constraint matched
            """
            return q - self.min_angles

        return scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.q,
            eqcons=[x_constraint,
                    y_constraint],
            ieqcons=[joint_limits_upper_constraint,
                     joint_limits_lower_constraint],
            args=(xy,),
            iprint=0)  # iprint=0 suppresses output


def test():
    # ###########Test it!##################

    arm = Arm3Link()

    # create a grid of desired (x,y) hand positions
    x = np.arange(-.75, .75, .05)
    y = np.arange(0, .75, .05)

    # threshold for printing out information, to find trouble spots
    thresh = .025

    count = 0
    total_error = 0
    # test it across the range of specified x and y values
    def PointsInCircum(r,n=100):
        return [[math.cos(2*np.pi/n*x)*r,math.sin(2*np.pi/n*x)*r] for x in range(0,n+1)]
    PointsInCircum(1) 
    # ipdb.set_trace()
    # xv, yv = npmeshgrid(x, y)
    
    # [Arm3Link().inv_kin(xy) for xy in xy_grid]
    # [Arm3Link().inv_kin(xy) for xy in xy_grid]
    # [Arm3Link().inv_kin(xy) for xy in xy_grid]
    for xi in range(len(x)):
        for yi in range(len(y)):
            # test the inv_kin function on a range of different targets
            xy = [x[xi], y[yi]]
            # run the inv_kin function, get the optimal joint angles
            q = arm.inv_kin(xy=xy)
            # find the (x,y) position of the hand given these angles
            actual_xy = arm.get_xy(q)
            # calculate the root squared error
            error = np.sqrt((np.array(xy) - np.array(actual_xy))**2)
            # total the error
            total_error += error

            # if the error was high, print out more information
            if np.sum(error) > thresh:
                print('-------------------------')
                print('Initial joint angles', arm.q)
                print('Final joint angles: ', q)
                print('Desired hand position: ', xy)
                print('Actual hand position: ', actual_xy)
                print('Error: ', error)
                print('-------------------------')

            count += 1

    print('\n---------Results---------')
    print('Total number of trials: ', count)
    print('Total error: ', total_error)
    print('-------------------------')





test()

