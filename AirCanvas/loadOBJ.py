import pywavefront
from pywavefront import visualization
import pyglet
from pyglet.gl import *
#import logging
import ctypes
import threading

#pywavefront.configure_logging(
    #logging.DEBUG,
    #formatter=logging.Formatter('%(name)s-%(levelname)s: %(message)s')
#)

rotation = 0
meshes = pywavefront.Wavefront("model.obj")
window = pyglet.window.Window()
lightfv = ctypes.c_float *4
uart_val= [0]

@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60., float(width)/height, 1., 100.)
    glMatrixMode(GL_MODELVIEW)
    return True

@window.event
def on_draw():
    global meshes
    window.clear()
    glLoadIdentity()

    glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0, 1.0, 0.0))
    glEnable(GL_LIGHT0)

    glTranslated(0.0, 0.0, -3.0)
    glRotatef(rotation, 0.0, 1.0, 0.0)
    glRotatef(0.0, 1.0, 0.0, 0.0)
    glRotatef(0.0, 0.0, 0.0, 1.0)

    glEnable(GL_LIGHTING)
    meshes = pywavefront.Wavefront("model.obj")
    visualization.draw(meshes)


def update(dt):
    global rotation
    global meshes
    global uart_val
    #print(uart_val[0])
    if ((uart_val[0] & 0x10) == 0x10):
        rotation += 30.0
    if((uart_val[0] & 0x20) == 0x20):
        rotation -= 30.0
    if rotation > 720.0:
        rotation = 0.0
    if rotation < 0:
        rotation = 720
pyglet.clock.schedule(update)
#obj_t = threading.Thread(target=pyglet.app.run)
#obj_t.daemon = True

def start():
    pyglet.app.run()

if __name__ == '__main__':
    #obj_t.start()
   # while(True):
   #     print('h')
    pyglet.app.run()
