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

def ROI(img, vertices):
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def ProcessImage(img, vertices):
    img_copy = np.copy(img)
    gray = cv2.cvtColor(img_copy, cv2.COLOR_RGB2GRAY)
    # Gaussian smoothing/blurring
    kernel_size = 7
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    # Define Canny Edge parameters
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
    # masked image
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    return masked_edges

# image_original = cv2.imread('test_image/ramp2ramps.png')
# print('Original image is:', type(image_original), '. with dimensions: ', image_original.shape)
# #changing color space
# image_RGB = cv2.cvtColor(image_original, cv2.COLOR_RGBA2RGB)
# print('After Convertion: ', type(image_RGB), '. with dimensions: ', image_RGB.shape)
#
# img_shape = image_RGB.shape
# # in the form of y and x
# # left_upper/ left_lower/ right_lower/ right_upper in order
# vertices1 = np.array([[(150, 150), (150, img_shape[1]- 150), (img_shape[0] - 130, img_shape[1] - 150), (img_shape[0] - 130, 150)]], dtype=np.int32)
# print('vertices are: ', vertices1)
# image_result = ProcessImage(image_RGB, vertices1)
#
# cv2.imwrite('./test_image/out_01.jpg', image_result)
#
# # stupid lines
# silly_ln1 = cv2.imread('test_image/stupid_line1.jpg')
# silly_ln1 = cv2.cvtColor(silly_ln1, cv2.COLOR_RGB2GRAY)
#
# silly_ln2 = cv2.imread('test_image/stupid_line2.jpg')
# silly_ln2 = cv2.cvtColor(silly_ln2, cv2.COLOR_RGB2GRAY)
#
# silly_ln3 = cv2.imread('test_image/stupid_line3.jpg')
# silly_ln3 = cv2.cvtColor(silly_ln3, cv2.COLOR_RGB2GRAY)
#
# silly_ln4 = cv2.imread('test_image/stupid_line4.jpg')
# silly_ln4 = cv2.cvtColor(silly_ln4, cv2.COLOR_RGB2GRAY)
#
# silly_ln5 = cv2.imread('test_image/stupid_line5.jpg')
# silly_ln5 = cv2.cvtColor(silly_ln5, cv2.COLOR_RGB2GRAY)
# print('Gray Silly Line Picture Shape is: ', silly_ln5.shape)
# # change stupid lines into points
# silly_ln_shape = silly_ln5.shape
# line_array = np.array([silly_ln1, silly_ln2, silly_ln3, silly_ln4, silly_ln5])
# l_array_shape = line_array.shape
# print('line array shape is:', l_array_shape)
#
# line1_points = []
# for i in range(0, silly_ln_shape[0]):
#     for j in range(0, silly_ln_shape[1]):
#         if silly_ln1[i][j] == 255:
#             temp_point = np.array([i, j])
#             line1_points.append(temp_point)
#             break
#
# line2_points = []
# for i in range(0, silly_ln_shape[0]):
#     for j in range(0, silly_ln_shape[1]):
#         if silly_ln2[i][j] == 255:
#             temp_point = np.array([i, j])
#             line2_points.append(temp_point)
#             break
#
# line3_points = []
# for i in range(0, silly_ln_shape[0]):
#     for j in range(0, silly_ln_shape[1]):
#         if silly_ln3[i][j] == 255:
#             temp_point = np.array([i, j])
#             line3_points.append(temp_point)
#             break
#
# line4_points = []
# for i in range(0, silly_ln_shape[0]):
#     for j in range(0, silly_ln_shape[1]):
#         if silly_ln4[i][j] == 255:
#             temp_point = np.array([i, j])
#             line4_points.append(temp_point)
#             break
#
# line5_points = []
# for i in range(0, silly_ln_shape[0]):
#     for j in range(0, silly_ln_shape[1]):
#         if silly_ln5[i][j] == 255:
#             temp_point = np.array([i, j])
#             line5_points.append(temp_point)
#
# len_ln1 = len(line1_points)
# len_ln2 = len(line2_points)
# len_ln3 = len(line3_points)
# len_ln4 = len(line4_points)
# len_ln5 = len(line5_points)
# print('points number in silly line1: ', len_ln1)
# # plot the lines
# for i in range(0, len_ln1):
#     plt.scatter(line1_points[i][1], line1_points[i][0], s = 2, c='b')
# for i in range(0, len_ln2):
#     plt.scatter(line2_points[i][1], line2_points[i][0], s = 2, c='r')
# for i in range(0, len_ln3):
#     plt.scatter(line3_points[i][1], line3_points[i][0], s = 2, c='y')
# for i in range(0, len_ln4):
#     plt.scatter(line4_points[i][1], line4_points[i][0], s = 2, c='g')
# for i in range(0, len_ln5):
#     plt.scatter(line5_points[i][1], line5_points[i][0], s = 2, c='m')

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