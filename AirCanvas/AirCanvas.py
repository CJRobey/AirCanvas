import threading
from multiprocessing import Process, Queue, Lock, Array 
from detect_finger import *
from dust import *
import gui_coords_lib
import uart 
import loadOBJ
from DM import *
#from DM import load_map_settings, get_rgb_lr_images, get_local_dm, get_dm

pressed_key = cv2.waitKey(1)
def producer(uart_val, q):
    global camera
    global pressed_key
    load_map_settings(in_folder + '/3dmap_set.txt')
    camera = PiCamera(stereo_mode='side-by-side',stereo_decimate=False)
    camera.resolution=(cam_width, cam_height)
    camera.framerate = 20
    camera.hflip = True

    hand_hist = calib() 
    print("starting producer: ", os.getpid()) 
    for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
        t1 = datetime.now()
        imgRight, imgLeft = get_rgb_lr_images(frame, img_height, img_width)
        pressed_key = cv2.waitKey(1)
        far_point = manage_image_opr(imgLeft, hand_hist)
        if far_point is None:
            far_point = (100000, 100000)
            continue
        show_frame = imgLeft 
        rel_tp = None
         
        disparity, ft, gs = get_dm(frame, img_height, img_width, calibration)
        #gs, rel_tp = get_local_dm(frame, img_height, img_width, calibration, far_point)
        draw_point = gui_coords_lib.get_normed_3d_coord(far_point, imgLeft.shape, gs, 1, rel_tp)
        q.put(draw_point)
        if (uart_val[0] & 0x08) == 0x08:
            break
        t2 = datetime.now()
        print("DM build time: " + str(t2-t1))
        # show the frame
        #cv2.imshow("left", imgLeft_gr)
        #cv2.imshow("right", imgRight_gr)
        if (uart_val[0] & 0x08) == 0x08 or (pressed_key & 0xFF == ord('q')):
            break
        show_frame = cv2.resize(show_frame, None, fx=2, fy=2)
        cv2.imshow("Livestream Finger Tracking", show_frame)       


def consumer(uart_val,q, uart_lock):
    global pressed_key 
    model = dust('10.138.255.159')
    model.clear()
    #model.reply()
    lNode = 0
    count = 0
    model_t = threading.Thread(target=model.reply)
    model_t.daemon = True
    model_t.start()
    
    while((uart_val[0] & 0x08) != 0x08):
        draw_point = q.get()
        count, lNode = gui_coords_lib.render_drawing(model, draw_point, uart_val[0], count,lNode, pressed_key)
        if (uart_val[0] & 0x08) == 0x08 or (pressed_key & 0xFF == ord('q')):
            break
        uart_lock.acquire()
        uart_val[:] = [0x00]
        uart_lock.release()

 
def calib():
    global hand_hist
    is_hand_hist_created= False
    hand_hist = np.load('hand_hist.npy')
    return hand_hist


def main():
    uart_val = Array('i',[0])
    loadOBJ.uart_val[:] = uart_val
    is_hand_hist_created = False
    x = 0
    final_scale = 1 
    local = False
    #UART thread
    uart_lock = Lock()
    uart_t = Process(target=uart.cont_get_value, args=(uart_val,uart_lock)) 
    uart_t.daemon = True
    uart_t.start()
    
    obj_t = threading.Thread(target=loadOBJ.start)
    obj_t.daemon = True
    obj_t.start()
    
 
    #calib_t = threading.Thread(target=calib)
    #calib_t.start()
    q = Queue() 
    #hardcore(hand_hist, q)
    p = Process(target=producer, args=(uart_val,q))
    #p.daemon = True
    p.start()
    #print("producer started")
    c = Process(target=consumer, args=(uart_val, q, uart_lock)) 
    c.start()
    time.sleep(10)
    while((uart_val[0] & 0x08) != 0x08):
        loadOBJ.uart_val[:] = uart_val
        if(not obj_t.is_alive()):
            print("restarting")
            obj_t = threading.Thread(target=loadOBJ.start)
            obj_t.daemon = True
            obj_t.start()
            print("restart")
        
    p.join(timeout = 1.0)    
    c.join(timeout = 1.0)
    uart_t.join(timeout = 1.0)
    q.close()
if __name__ == '__main__':
    main()
    time.sleep(2)
    print("done") 
