import threading
from multiprocessing import Process, Queue, Lock
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
from DM import *
#

def hardcorestuff():
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
       
        t1 = datetime.now()
#        imgLeft_gr, imgRight_gr = get_grayscale_lr_images(frame, img_height, img_width)
        imgRight, imgLeft = get_rgb_lr_images(frame, img_height, img_width)
        
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
        if (uart_val[0] & 0x08) == 0x08:
            break
        t2 = datetime.now()
        print("DM build time: " + str(t2-t1))
     # show the frame
        #cv2.imshow("left", imgLeft_gr)
        #cv2.imshow("right", imgRight_gr)
        if (uart_val[0] & 0x08) == 0x08:
            break
        cv2.imshow("Livestream Finger Tracking", show_frame)       
i

def soft():
    while((uart_val[0] & 0x08) != 0x08):    
        draw_point = gui_coords_lib.get_normed_3d_coord(far_point, imgLeft.shape, gs, 1, rel_tp)
        count, lNode = gui_coords_lib.render_drawing(model, draw_point, uart_val[0], count,lNode, pressed_key)
        if (uart_val[0] & 0x08) == 0x08:
            break
         

def main():
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
    local = True
    ax = plt.axes(projection='3d')
    
   #UART thread
    uart_val =[0]
    uart_t = threading.Thread(target=uart.cont_get_value, args=(uart_val,)) 
    uart_t.daemon = True
    uart_t.start()
    
    model = dust('10.228.5.88')
    model.clear()
    #model.reply()
    lNode = 0
    count = 0
    model_t = threading.Thread(target=model.reply)
    model_t.daemon = True
    model_t.start()
    loadOBJ.obj_t.start()
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
       
#        imgLeft_gr, imgRight_gr = get_grayscale_lr_images(frame, img_height, img_width)
        imgRight, imgLeft = get_rgb_lr_images(frame, img_height, img_width)
        
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
            break  
        else:
            show_frame = draw_rect(imgLeft)
            #show_frame = ft



  if __name__ == '__main__':
    main()
    print("done") 
