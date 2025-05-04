import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import dearpygui.dearpygui as dpg
import threading

cube_size = 1.0

vertices = [
    [1, 1, -1],
    [1, -1, -1],
    [-1, -1, -1],
    [-1, 1, -1],
    [1, 1, 1],
    [1, -1, 1],
    [-1, -1, 1],
    [-1, 1, 1]
]

edges = [
    (0,1), (1,2), (2,3), (3,0),
    (4,5), (5,6), (6,7), (7,4),
    (0,4), (1,5), (2,6), (3,7)
]

def Cube(scale):
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv([x * scale for x in vertices[vertex]])
    glEnd()

def slider_callback(sender, app_data):
    global cube_size
    cube_size = app_data

def gui_thread():
    dpg.create_context()
    dpg.create_viewport(title='Controls', width=300, height=100)
    with dpg.window(label="Cube Controls"):
        dpg.add_slider_float(label="Cube Size", default_value=1.0, min_value=0.1, max_value=3.0, callback=slider_callback)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

threading.Thread(target=gui_thread, daemon=True).start()

# Pygame + OpenGL init
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)

# Main loop
angle = 0
clock = pygame.time.Clock()
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glPushMatrix()
    glRotatef(angle, 1, 1, 0)
    Cube(cube_size)
    glPopMatrix()

    pygame.display.flip()
    clock.tick(60)
    angle += 1
