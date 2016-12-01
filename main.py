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
import pyglet
import ipdb

import time

import Arm

def generate_random_xy_point(xlim,ylim):
    x = np.random.uniform(xlim[0],xlim[1],1)
    y = np.random.uniform(ylim[0],ylim[1],1)
    return((x,y))
def randomly_pick_samples_of_xy_endpoints(arm,n):
    generate_random_xy_point(xlim=[0,500], ylim = [0,500])
        

def plot(): 
    """A function for plotting an arm, and having it calculate the 
    inverse kinematics such that given the mouse (x, y) position it 
    finds the appropriate joint angles to reach that point."""
    def get_joint_positions(arm):
        """This method finds the (x,y) coordinates of each joint"""

        x = np.array([ 0, 
            arm.L[0]*np.cos(arm.q[0]),
            arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]),
            arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]) + 
                arm.L[2]*np.cos(np.sum(arm.q)) ]) + window.width/2

        y = np.array([ 0, 
            arm.L[0]*np.sin(arm.q[0]),
            arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]),
            arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]) + 
                arm.L[2]*np.sin(np.sum(arm.q)) ])

        return np.array([x, y]).astype('int')

    # create an instance of the arm
    firstArm = Arm.Arm3Link(L = np.array([300,200,100]))
    arm2 = Arm.Arm3Link(L = np.array([200,50,50]), q = [math.pi/2, math.pi/2, 0])
    arm3 = Arm.Arm3Link(L = np.array([50,100,10]))
    firstArm_xy_database = randomly_pick_samples_of_xy_endpoints(firstArm, 10)
    # make our window for drawin'
    window = pyglet.window.Window()
    label = pyglet.text.Label('Mouse (x,y)', font_name='Futura', 
        font_size=18, x=window.width//2, y=window.height//2,
        anchor_x='left', anchor_y='center')

    
    window.firstArm = get_joint_positions(firstArm)
    window.secondArm = get_joint_positions(arm2)
    window.thirdArm = get_joint_positions(arm3)

    @window.event
    def on_draw():
        window.clear()
        label.draw()
        draw_joint_positions(window.firstArm)
        draw_joint_positions(window.secondArm)
        draw_joint_positions(window.thirdArm)
    def draw_joint_positions(joint_positions):
        for i in range(3): 
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
                (joint_positions[0][i], joint_positions[1][i], 
                 joint_positions[0][i+1], joint_positions[1][i+1])))

    def get_line_segment_points(joint_positions):
        for i in range(3): 
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
                (joint_positions[0][i], joint_positions[1][i], 
                 joint_positions[0][i+1], joint_positions[1][i+1])))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        # call the inverse kinematics function of the arm
        # to find the joint angles optimal for pointing at 
        # this position of the mouse 
        label.text = '(x,y) = (%.3f, %.3f)'%(x,y)
        firstArm.q = firstArm.inv_kin([x - window.width/2, y]) # get new arm angles
        window.firstArm = get_joint_positions(firstArm) # get new joint (x,y) positions

    pyglet.app.run()

plot()
