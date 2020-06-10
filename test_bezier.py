import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import os
from mpl_toolkits.mplot3d import Axes3D

# import cpp lib
# from build import libBezierTest
from extending import *
from build.libBezierTest import *

def SetBezierCurve(order, start_pt, end_pt, ctrl_pts):
    hi = hello('California')
    print(hi.greet())
    test = BezierCurve(order)
    test.start_ = start_pt
    test.end_ = end_pt
    test.SetCtrlPts(to_std_vector(ctrl_pts))
    return test

def TestDrawBezierCurve(order, bz_class):
    bezier_curve = to_std_vector([BezierPoint(list_to_vector([0.0, 0.0]))] * order)
    bz_class.PtsOnCurve(bezier_curve, order)
    plt.scatter(bz_class.start_.coordinate[0], bz_class.start_.coordinate[1], s = 8, c= 'tab:pink')
    plt.scatter(bz_class.end_.coordinate[0], bz_class.end_.coordinate[1], s = 8, c= 'tab:brown')
    print('start_pos is', bz_class.start_.coordinate[0], bz_class.start_.coordinate[1])
    print('end_pos is', bz_class.end_.coordinate[0], bz_class.end_.coordinate[1])
    for i in range (0, bz_class.order_ - 1):
        plt.scatter(bz_class.ctrl_pts_[i].coordinate[0], bz_class.ctrl_pts_[i].coordinate[1], s = 8, c= 'tab:orange')
        print('ctrl_pos is', bz_class.ctrl_pts_[i].coordinate[0], bz_class.ctrl_pts_[i].coordinate[1])
    for i in range(1, order - 1):
        plt.scatter(bezier_curve[i].coordinate[0], bezier_curve[i].coordinate[1], s = 4, c='c')

# ls_st_pt = [(line2_points[0][1] + line3_points[0][1])/2, (line2_points[0][0] + line3_points[0][0])/2]
ls_st_pt = [0.0, 0.0]
start_pt = BezierPoint(list_to_vector(ls_st_pt))
# end_pt = BezierPoint(list_to_vector([(line4_points[0][1] + line5_points[0][1])/2, (line4_points[0][0] + line5_points[0][0])/2]), 0.05)
end_pt = BezierPoint(list_to_vector([48.0, 98.0]))
# ctrl_pt = BezierPoint(list_to_vector([(line4_points[len_ln4 - 1][1] + line5_points[len_ln5 - 1][1])/2, (line4_points[len_ln4 - 1][0] + line5_points[len_ln5 - 1][0])/2 + 20]), 0.05)
ctrl_pt = BezierPoint(list_to_vector([13.0, 12.0]))
ctrl_pt2 = BezierPoint(list_to_vector([25.6, 59.3]))
ctrl_pt3 = BezierPoint(list_to_vector([38.6, 89.3]))
ctrl_pt4 = BezierPoint(list_to_vector([30.1, 66.3]))
ctrl_pts = [ctrl_pt, ctrl_pt2, ctrl_pt3, ctrl_pt4]

test = SetBezierCurve(5, start_pt, end_pt, ctrl_pts)
TestDrawBezierCurve(30, test)

def SetBezierCurve3d(order, start_pt, end_pt, ctrl_pts):
    hi = hello('Beijing')
    print(hi.greet())
    test = BezierCurve3d(order)
    test.start_3d = start_pt
    test.end_3d = end_pt
    test.SetCtrlPts3d(to_std_vector3d(ctrl_pts))
    return test

def TestDrawBezierCurve3d(order, bz_class):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_ylabel('Z')
    bezier_curve = to_std_vector3d([BezierPoint3d(list_to_vector([0.0, 0.0, 0.0]))] * order)
    bz_class.PtsOnCurve3d(bezier_curve, order)
    ax.scatter(bz_class.start_3d.coordinate3d[0], bz_class.start_3d.coordinate3d[1], bz_class.start_3d.coordinate3d[2], c= 'tab:pink')
    ax.scatter(bz_class.end_3d.coordinate3d[0], bz_class.end_3d.coordinate3d[1], bz_class.end_3d.coordinate3d[2], c= 'tab:brown')
    print('start_pos is', bz_class.start_3d.coordinate3d[0], bz_class.start_3d.coordinate3d[1], bz_class.start_3d.coordinate3d[2])
    print('end_pos is', bz_class.end_3d.coordinate3d[0], bz_class.end_3d.coordinate3d[1], bz_class.end_3d.coordinate3d[2])
    for i in range (0, bz_class.order_3d - 1):
        ax.scatter(bz_class.ctrl_pts_3d[i].coordinate3d[0], bz_class.ctrl_pts_3d[i].coordinate3d[1], c= 'tab:orange')
        print('ctrl_pos is', bz_class.ctrl_pts_3d[i].coordinate3d[0], bz_class.ctrl_pts_3d[i].coordinate3d[1], bz_class.ctrl_pts_3d[i].coordinate3d[2])
    for i in range(1, order - 1):
        ax.scatter(bezier_curve[i].coordinate3d[0], bezier_curve[i].coordinate3d[1], bezier_curve[i].coordinate3d[2], c='c')

start_pt_3d = BezierPoint3d(list_to_vector([0.0, 0.0, 0.0]))
end_pt_3d = BezierPoint3d(list_to_vector([10.0, 100.0, 100.0]))
ctrl_pt_3d = BezierPoint3d(list_to_vector([15.0, 72.0, 18.0]))
ctrl_pt_3d = BezierPoint3d(list_to_vector([15.0, 72.0, 18.0]))
ctrl_pts_3d = [ctrl_pt_3d]
test2 = SetBezierCurve3d(2, start_pt_3d, end_pt_3d, ctrl_pts_3d)
TestDrawBezierCurve3d(30, test2)

plt.show()
