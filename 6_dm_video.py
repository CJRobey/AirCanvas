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

in_folder = 'newcams_calib_result'

# Implementing calibration data
print('Read calibration data and rectifying stereo pair...')
calibration = StereoCalibration(input_folder=in_folder)

# Initialize interface windows
cv2.namedWindow("Image")
cv2.moveWindow("Image", 50,100)
#cv2.namedWindow("left")
#cv2.moveWindow("left", 450,100)
#cv2.namedWindow("right")
#cv2.moveWindow("right", 850,100)


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
    final_scale = 2
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    color = 'b'
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
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
        
        uart_val = uart.get_value()
        if ((uart_val & 0x01) == 0x01) or (pressed_key == ord('z')):
            is_hand_hist_created = True
            hand_hist = hand_histogram(rectified_pair[0])
        if is_hand_hist_created:
            far_point = manage_image_opr(rectified_pair[0], hand_hist)
            if far_point is None:
                far_point = (100000, 100000)
                continue
            #show_frame = imgLeft
            show_frame = rectified_pair[0]
            if far_point[0] < gs.shape[0] and far_point[1] < gs.shape[1]:
                print("Disparity at finger is, ", gs[far_point])
            
                # draw point is a 3d point normalized between -final_scale and final_scale
                draw_point = np.zeros(3)
                draw_point[0] = far_point[0]/rectified_pair[0].shape[0]*final_scale
                draw_point[1] = far_point[1]/rectified_pair[0].shape[1]*final_scale
                draw_point[2] = gs[far_point]*final_scale
                if (uart_val & 0x01) == 0x01:
                    color = 'y'
                elif (uart_val & 0x02) == 0x02:
                    color = 'r'
                elif (uart_val & 0x04) == 0x04:
                    color = 'g'
                ax.scatter(draw_point[0], draw_point[1], draw_point[2], c=color)
                plt.pause(0.005)
                #plt.scatter(x, gs[far_point])
                #x += 1
                #plt.pause(0.05)
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
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
