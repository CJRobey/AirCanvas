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
import threading
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration
from datetime import datetime
from detect_finger import *
from mpl_toolkits.mplot3d import Axes3D
from dust import *
import gui_coords_lib
import uart 
import loadOBJ
import os
global camera
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
print ("DM: Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
print ("DM: Scaled image resolution: "+str(img_width)+" x "+str(img_height))

# Initialize the camera
#camera = PiCamera(stereo_mode='side-by-side',stereo_decimate=False)
#camera.resolution=(cam_width, cam_height)
#camera.framerate = 20
#camera.hflip = True

in_folder = 'newcams_calib_result'

# Implementing calibration data
print('DM: Read calibration data and rectifying stereo pair...')
calibration = StereoCalibration(input_folder=in_folder)

# Initialize interface windows
cv2.namedWindow("Image")
cv2.moveWindow("Image", 0,0)
#cv2.resizeWindow("Image", cam_width, cam_height)
cv2.namedWindow("Livestream Finger Tracking")
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
    view_img = cv2.resize(disparity_color, None, fx=2, fy=2)
    cv2.imshow("Image", view_img)
    #cv2.imshow("Image", disparity_color)
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



def get_grayscale_lr_images(frame, img_height, img_width):
    # get the left and right images in the context of the frame
    pair_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imgLeft_gr = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
    imgRight_gr = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
    return imgLeft_gr, imgRight_gr


def get_grayscale_lr_crop(imgLeft, imgRight, boundaries):
    b=boundaries
    crop_l = imgLeft[b[0]:b[1], b[2]:b[3]]
    crop_r = imgRight[b[0]:b[1], b[2]:b[3]]
    return crop_l, crop_r


def get_local_dm(frame, img_height, img_width, calibration, touch_point, pad_val = 100):
    # figure out the boundaries
    boundaries = np.zeros(4)
    boundaries[0] = touch_point[0] - pad_val
    boundaries[1] = touch_point[0] + pad_val
    boundaries[2] = touch_point[1] - pad_val
    boundaries[3] = touch_point[1] + pad_val
    boundaries[boundaries < 0] = 0
    for num in range(2):
        if boundaries[num] >= img_width:
            boundaries[num] = img_width-1
    for num in range(2,4):
        if boundaries[num] >= img_height:
            boundaries[num] = img_height-1
    left_im, right_im = get_grayscale_lr_images(frame, img_height, img_width)
    l_crop, r_crop  = get_grayscale_lr_crop(left_im, right_im, boundaries.astype(int))

    # create a relative touch point to the new window
    rel_tp = np.zeros(2)
    rel_tp[0] = touch_point[0] - boundaries[0]
    rel_tp[1] = touch_point[1] - boundaries[2]
    
    rectified = rect_images(l_crop, r_crop, calibration)
    disparity, ft, gs = stereo_depth_map(rectified)
    return gs, rel_tp.astype(int)



def get_grayscale_lr_images(frame, img_height, img_width):
    # get the left and right images in the context of the frame
    pair_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imgLeft_gr = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
    imgRight_gr = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
    return imgLeft_gr, imgRight_gr


def get_rgb_lr_images(frame, img_height, img_width):
    imgLeft = frame[0:img_height,0:int(img_width/2)]
    imgRight = frame[0:img_height,int(img_width/2):img_width] #Y+H and X+W
    return imgLeft, imgRight


def rect_images(im_l, im_r, calibration):
    # rectify the left and right pairs (this can be in grayscale or RGB)
    return calibration.rectify((im_l, im_r))


def get_dm(frame, img_height, img_width, calibration):
    imgLeft_gr, imgRight_gr = get_grayscale_lr_images(frame, img_height, img_width)
    rectified = rect_images(imgLeft_gr, imgRight_gr, calibration)
    disparity, ft, gs = stereo_depth_map(rectified)
    return disparity, ft, gs

uart_val =[0]
loadOBJ.uart_val[:] = uart_val
def main():
    global uart_val
    load_map_settings(in_folder + '/3dmap_set.txt')
    is_hand_hist_created = False
    x = 0
    final_scale = 1 
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    # these are the more intuitive ways of perspective labeling
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    ax.set_xlim3d(0, final_scale)
    ax.set_ylim3d(0, final_scale)
    ax.set_zlim3d(0, final_scale)
    local = False
    ax = plt.axes(projection='3d')
    
   #UART thread
   
    uart_t = threading.Thread(target=uart.cont_get_value, args=(uart_val,)) 
    uart_t.daemon = True
    uart_t.start()
    
    model = dust('10.138.50.226')
    model.clear()
    lNode = 0
    count = 0
    model_t = threading.Thread(target=model.reply)
    model_t.daemon = True
    model_t.start()
    obj_t = threading.Thread(target=loadOBJ.pyglet.app.run)
    obj_t.daemon = True
    obj_t.start()
    if os.path.exists('./hand_hist.npy'):
        hand_hist = np.load('./hand_hist.npy')
        is_hand_hist_created = True
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
       
        t1 = datetime.now()
#        imgLeft_gr, imgRight_gr = get_grayscale_lr_images(frame, img_height, img_width)
        imgRight, imgLeft = get_rgb_lr_images(frame, img_height, img_width)
       # uart_val[0] = uart.get_value()
#        pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
#        imgLeft_gr = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
#        imgRight_gr = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
#        imgLeft = frame[0:img_height,0:int(img_width/2)]
#        imgRight = frame[0:img_height,int(img_width/2):img_width] #Y+H and X+W
#        rectified_pair_gr = calibration.rectify((imgLeft_gr, imgRight_gr))
#        rectified_pair = calibration.rectify((imgLeft, imgRight))
        # capture frames from the camera
        pressed_key = cv2.waitKey(1)
    
        #disparity, ft, gs = stereo_depth_map(rectified_pair_gr)
        
        if ((uart_val[0] & 0x01) == 0x01) or (pressed_key & 0xFF == ord('z')) and (not is_hand_hist_created):
            is_hand_hist_created = True
            hand_hist = hand_histogram(imgLeft)
        if is_hand_hist_created:
            far_point = manage_image_opr(imgLeft, hand_hist)
            if far_point is None:
                far_point = (100000, 100000)
                continue
            show_frame = imgLeft 
            rel_tp = None
        
            if local:
                gs, rel_tp = get_local_dm(frame, img_height, img_width, calibration, far_point)
            else:
                disparity, ft, gs = get_dm(frame, img_height, img_width, calibration)
            
            draw_point = gui_coords_lib.get_normed_3d_coord(far_point, imgLeft.shape, gs, 1, rel_tp)
         
            count, lNode = gui_coords_lib.render_drawing(model, draw_point, uart_val[0], count,lNode, pressed_key)
            uart_val[:] = [0x00]
            # show the frame
            #cv2.imshow("left", imgLeft_gr)
            #cv2.imshow("right", imgRight_gr)
            t2 = datetime.now()
            print("DM build time: " + str(t2-t1))

        else:
            show_frame = draw_rect(imgLeft)
            #show_frame = ft
        if (uart_val[0] & 0x08) == 0x08:
            break
        if(not obj_t.is_alive()):
            obj_t = threading.Thread(target=loadOBJ.pyglet.app.run)
            obj_t.daemon = True
            obj_t.start()
        cv2.imshow("Livestream Finger Tracking", show_frame)
    model.close()
if __name__ == "__main__":
    main()
