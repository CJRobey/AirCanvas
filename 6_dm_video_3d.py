# Copyright (C) 2019 Eugene Pomazov, <stereopi.com>, virt2real team
#
# This file is part of StereoPi tutorial scripts.
#
# StereoPi tutorial is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# StereoPi tutorial is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with StereoPi tutorial.  
# If not, see <http://www.gnu.org/licenses/>.
#
# Most of this code is updated version of 3dberry.org project by virt2real
# 
# Thanks to Adrian and http://pyimagesearch.com, as there are lot of
# code in this tutorial was taken from his lessons.
# 


from picamera import PiCamera
import time
import cv2
import numpy as np
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration
from datetime import datetime
from detect_finger import *
from mpl_toolkits.mplot3d import Axes3D
from dust import *
import gui_coords_lib
import uart 
# Depth map default preset
SWS = 5
PFS = 5
PFC = 29
MDS = -30
NOD = 160
TTH = 100
UR = 10
SR = 14
SPWS = 100

# Camera settimgs
cam_width = 1280
cam_height = 480

# Final image capture settings
scale_ratio = 0.5

# Camera resolution height must be dividable by 16, and width by 32
cam_width = int((cam_width+31)/32)*32
cam_height = int((cam_height+15)/16)*16
print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

# Initialize the camera
camera = PiCamera(stereo_mode='side-by-side',stereo_decimate=False)
camera.resolution=(cam_width, cam_height)
camera.framerate = 20
camera.hflip = True

in_folder = 'in_box_calib_result'

# Implementing calibration data
print('Read calibration data and rectifying stereo pair...')
calibration = StereoCalibration(input_folder=in_folder)

# Initialize interface windows
cv2.namedWindow("Image")
cv2.moveWindow("Image", 50,100)
cv2.namedWindow("left")
cv2.moveWindow("left", 450,100)
cv2.namedWindow("right")
cv2.moveWindow("right", 850,100)


disparity = np.zeros((img_width, img_height), np.uint8)
sbm = cv2.StereoBM_create(numDisparities=0, blockSize=21)


def stereo_depth_map(rectified_pair):
    dmLeft = rectified_pair[0]
    dmRight = rectified_pair[1]
    disparity = sbm.compute(dmLeft, dmRight)
    local_max = disparity.max()
    local_min = disparity.min()
    disparity_grayscale = (disparity-local_min)*(65535.0/(local_max-local_min))
    disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0/65535.0))
    disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
    #print('Disparity color size:', disparity_color.shape, '\nDisparity fixtype:', disparity_fixtype.shape,
    #      '\nDisparity grayscae:', disparity_grayscale.shape, '\n')
    cv2.imshow("Image", disparity_color)
    key = cv2.waitKey(1) & 0xFF   
    if key == ord("q"):
        quit();
    return disparity_color, disparity_fixtype, (disparity-local_min)/(local_max-local_min)


def load_map_settings(fName):
    global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
    print('Loading parameters from file...')
    f=open(fName, 'r')
    data = json.load(f)
    SWS=data['SADWindowSize']
    PFS=data['preFilterSize']
    PFC=data['preFilterCap']
    MDS=data['minDisparity']
    NOD=data['numberOfDisparities']
    TTH=data['textureThreshold']
    UR=data['uniquenessRatio']
    SR=data['speckleRange']
    SPWS=data['speckleWindowSize']    
    #sbm.setSADWindowSize(SWS)
    sbm.setPreFilterType(1)
    sbm.setPreFilterSize(PFS)
    sbm.setPreFilterCap(PFC)
    sbm.setMinDisparity(MDS)
    sbm.setNumDisparities(NOD)
    sbm.setTextureThreshold(TTH)
    sbm.setUniquenessRatio(UR)
    sbm.setSpeckleRange(SR)
    sbm.setSpeckleWindowSize(SPWS)
    f.close()
    print('Parameters loaded from file '+fName)


def main():
    load_map_settings(in_folder + '/3dmap_set.txt')
    is_hand_hist_created = False
    x = 0
    final_scale = 1 
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    model = dust('10.138.232.207')
    model.clear()
    model.reply()
    lNode = 0
    count = 0
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
       
        uart_val = uart.get_value()    
        t1 = datetime.now()
        pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
        imgLeft_gr = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
        imgRight_gr = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
        imgLeft = frame[0:img_height,0:int(img_width/2)]
        imgRight = frame[0:img_height,int(img_width/2):img_width] #Y+H and X+W
        rectified_pair_gr = calibration.rectify((imgLeft_gr, imgRight_gr))
        rectified_pair = calibration.rectify((imgLeft, imgRight))
        # capture frames from the camera
        pressed_key = cv2.waitKey(1)
        disparity, ft, gs = stereo_depth_map(rectified_pair_gr)
        
        if ((uart_val & 0x01) == 0x01) or (pressed_key & 0xFF == ord('z')) and (not is_hand_hist_created):
            is_hand_hist_created = True
            hand_hist = hand_histogram(rectified_pair[0])
        if is_hand_hist_created:
            far_point = manage_image_opr(rectified_pair[0], hand_hist)
            if far_point is None:
                far_point = (100000, 100000)
            show_frame = rectified_pair[0]
            
            draw_point = gui_coords_lib.get_normed_3d_coord(far_point, rectified_pair, gs, 1)
         
            count, lNode = gui_coords_lib.render_drawing(model, draw_point, uart_val, count,lNode)
            # show the frame
            #cv2.imshow("left", imgLeft_gr)
            #cv2.imshow("right", imgRight_gr)
            t2 = datetime.now()
            print("DM build time: " + str(t2-t1))

        else:
            show_frame = draw_rect(rectified_pair[0])
            #show_frame = ft
        if (uart_val & 0x08) == 0x08:
            break
        cv2.imshow("Livestream Finger Tracking", show_frame)
    model.close()
if __name__ == "__main__":
    main()
