#!/usr/bin/python3

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os



class link_controller():

    def __init__(self):
    
        self.xml_path = 'link.xml' #xml file (assumes this is in the same folder as this file)
        self.simend = 50 #simulation time
        self.print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

        # For callback functions
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0

        #get the full path
        self.dirname = os.path.dirname(__file__)
        self.abspath = os.path.join(self.dirname + "/" + self.xml_path)
        #xml_path = abspath

        # MuJoCo data structures
        self.model = mj.MjModel.from_xml_path(self.xml_path)  # MuJoCo model
        self.data = mj.MjData(self.model)                # MuJoCo data
        self.cam = mj.MjvCamera()                        # Abstract camera
        self.opt = mj.MjvOption()                        # visualization options

        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # initialize visualization data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(self.window, self.keyboard) 
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)

        # Example on how to set camera configuration
        # cam.azimuth = 90
        # cam.elevation = -45
        # cam.distance = 2
        # cam.lookat = np.array([0.0, 0.0, 0])

        self.cam.azimuth = 89.8 ; self.cam.elevation = -88.0000000000001 ; self.cam.distance =  13.332486107176425
        self.cam.lookat = np.array([ 0.0 , 0.0 , 0.0 ])


        #initialize the controller
        self.init_controller(self.model,self.data)

        #set the controller
        mj.set_mjcb_control(self.controller)


        self.N = 1000
        q0_start = 0
        q0_end = 1.57
        q1_start = 0
        q1_end = -2*3.14

        self.q0 = np.linspace(q0_start,q0_end,self.N)
        self.q1 = np.linspace(q1_start,q1_end,self.N)

        self.data.qpos[0] = q0_start
        self.data.qpos[1] = q1_start
        self.time = 0
        self.dt = 0.001
        self.i = 0

    def init_controller(self,model,data):
        #initialize the controller here. This function is called once, in the beginning
        pass

    def controller(self,model, data):
        #put the controller here. This function is called inside the simulation.
        pass

    def keyboard(self,window, key, scancode, act, mods):
        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)
            print("key pressed++++++++++++++++++++++++++++++++++++++++++=")
            # Reset time
            self.time = 0

            # Reset the index used for generating joint angles
            self.i = 0

            print(self.i, self.time)

            print("Simulation reset")

    def mouse_button(self, window, button, act, mods):
   

        self.button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self,window, xpos, ypos):
        # compute mouse displacement, save
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if self.button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.model, action, dx/height,
                        dy/height, self.scene, self.cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 *
                        yoffset, self.scene, self.cam)



    def main(self):       

        while not glfw.window_should_close(self.window):


            print(f"after {self.i}, the time is {self.time}")

            time_prev = self.time

            while (self.time - time_prev < 1.0/60.0):
                self.data.qpos[0] = self.q0[self.i]
                self.data.qpos[1] = self.q1[self.i]
                mj.mj_forward(self.model,self.data)
                self.time += self.dt
                #mj.mj_step(model, data)

            print(self.data.site_xpos[0])         # Cartesian site position                          (nsite x 3)
            self.i += 1
            if (self.i>=self.N):
                break;

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            #print camera configuration (help to initialize the view)
            if (self.print_camera_config==1):
                print('cam.azimuth =',self.cam.azimuth,';','cam.elevation =',self.cam.elevation,';','cam.distance = ',self.cam.distance)
                print('cam.lookat =np.array([',self.cam.lookat[0],',',self.cam.lookat[1],',',self.cam.lookat[2],'])')

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                            mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            #glfw.set_key_callback(window, keyboard)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()


if __name__ == "__main__":
    LC = link_controller()
    LC.main()
