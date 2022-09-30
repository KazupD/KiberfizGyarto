import tinyik
import math
import numpy as np
from tkinter import *
import time
import pygame

class robot_arm():
    def __init__(self, angle, distance, tcp_shifted):
        self.angle = angle
        self.rotation = self.angle+90
        self.transform = [distance*math.cos(self.deg2rad(angle)), distance*math.sin(self.deg2rad(angle)), self.deg2rad(self.rotation)]
        self.distance = distance
        self.own_tcp = None

        self.htm_rot1 = np.array([[math.cos(self.transform[2]), math.sin(self.transform[2]), 0],[-math.sin(self.transform[2]), math.cos(self.transform[2]), 0],[0, 0, 1]])
        self.htm_trans1 = np.array([[1, 0, 0],[0, 1, distance],[0, 0, 1]])
        self.htm1 = np.matmul(self.htm_trans1, self.htm_rot1)

        self.htm_rot2 = np.array([[math.cos(-self.transform[2]), math.sin(-self.transform[2]), 0],[-math.sin(-self.transform[2]), math.cos(-self.transform[2]), 0],[0, 0, 1]])
        self.htm_trans2 = np.array([[1, 0, 0],[0, 1, -distance],[0, 0, 1]])
        self.htm2 = np.matmul(self.htm_rot2, self.htm_trans2)

        self.rod_length = [0.75*distance, 0.75*distance]
        self.arm = tinyik.Actuator(['z', [self.rod_length[0], 0, 0], 'z', [self.rod_length[1], 0, 0]])
        self.convert_to_own_frame(coords=tcp_shifted)
        self.own_bearing_coordinates = None
        self.world_bearing_coordinates = None
        self.theta = None

    def convert_to_own_frame(self, coords):
        vect = np.array([coords[0], coords[1], 1])
        product = self.htm1.dot(vect)
        self.own_tcp = [product[0], product[1]]

    def calculate_ik(self):
        self.arm.ee = self.own_tcp + [0]
        self.theta =self.arm.angles[0]
        self.own_bearing_coordinates = [self.rod_length[0]*math.cos(self.arm.angles[0]), self.rod_length[0]*math.sin(self.arm.angles[0])]
        self.convert_to_world_frame(self.own_bearing_coordinates)

    def convert_to_world_frame(self, coords):
        vect = np.array([coords[0], coords[1], 1])
        product = self.htm2.dot(vect)
        self.world_bearing_coordinates = [product[0], product[1]]

    def deg2rad(self, deg):
        return (deg/180)*math.pi

class manipulator():
    def __init__(self, angle1=90, angle2=210, angle3=330, radius=50):
        self.buffer = []
        self.angles = [angle1, angle2, angle3]
        self.tcp = [0, 0, 0]
        self.tripod_radius = radius*0.2
        self.shifted_tcps = None
        self.motor_coords = []
        self.motor_coords.append([radius*math.cos(self.deg2rad(self.angles[0])), radius*math.sin(self.deg2rad(self.angles[0]))])
        self.motor_coords.append([radius*math.cos(self.deg2rad(self.angles[1])), radius*math.sin(self.deg2rad(self.angles[1]))])
        self.motor_coords.append([radius*math.cos(self.deg2rad(self.angles[2])), radius*math.sin(self.deg2rad(self.angles[2]))])
        self.bearing_coords = []
        self.thetas = []
        self.calc_shifted_tcps()
        self.arms = [robot_arm(angle=angle1, distance=radius, tcp_shifted=self.shifted_tcps[0]), robot_arm(angle=angle2, distance=radius, tcp_shifted=self.shifted_tcps[1]), robot_arm(angle=angle3, distance=radius, tcp_shifted=self.shifted_tcps[2])]

    def calc_shifted_tcps(self):
        self.shifted_tcps=[]
        self.shifted_tcps.append([self.tcp[0]+self.tripod_radius*math.cos(self.tcp[2]+self.deg2rad(self.angles[0])), self.tcp[1]+self.tripod_radius*math.sin(self.tcp[2]+self.deg2rad(self.angles[0]))])
        self.shifted_tcps.append([self.tcp[0]+self.tripod_radius*math.cos(self.tcp[2]+self.deg2rad(self.angles[1])), self.tcp[1]+self.tripod_radius*math.sin(self.tcp[2]+self.deg2rad(self.angles[1]))])
        self.shifted_tcps.append([self.tcp[0]+self.tripod_radius*math.cos(self.tcp[2]+self.deg2rad(self.angles[2])), self.tcp[1]+self.tripod_radius*math.sin(self.tcp[2]+self.deg2rad(self.angles[2]))])

    def new_tcp(self, x, y, phi):
        tcp_backup = self.tcp
        self.tcp=[x, y, phi]
        self.calc_shifted_tcps()
        [self.arms[i].convert_to_own_frame(coords=self.shifted_tcps[i]) for i in range(3)]
        [self.arms[i].calculate_ik() for i in range(3)]
        self.bearing_coords = [self.arms[i].world_bearing_coordinates for i in range(3)]
        self.thetas = [self.arms[i].theta for i in range(3)]
        if(not all([-math.pi/4 <= self.thetas[i] <= math.pi/2 for i in range(3)]) or not(-math.pi/2<=phi<=math.pi/2)):
            self.new_tcp(tcp_backup[0], tcp_backup[1], tcp_backup[2])

    def deg2rad(self, deg):
        return (deg/180)*math.pi

    def buffer_len(self):
        return len(self.buffer)

    def execute_shape(self):
        if(len(self.buffer) > 0):
            if(len(self.buffer[0].points)>0):
                time.sleep(0.03)
                self.new_tcp(self.buffer[0].points[0][0], self.buffer[0].points[0][1], self.buffer[0].points[0][2])
                self.buffer[0].points.pop(0)
            else:
                self.buffer.pop(0)

class graphics():
    def __init__(self):
        self.root = Tk()
        self.root.title("Parallel Inverse Kinematics")
        self.root.configure(bg='lightgrey')
        self.width = 640
        self.height = 640
        self.root.minsize(width=self.width+250, height=self.height)
        self.root.maxsize(width=self.width+250, height=self.height)
        self.canvas = Canvas(self.root, bg='white', height=self.height, width=self.width)
        self.canvas.pack(side='left')
        self.x_slider = Scale(self.root, from_=25, to=-25, orient=VERTICAL, label='X', length=200, resolution=0.1)
        self.y_slider = Scale(self.root, from_=25, to=-25, orient=VERTICAL, label ='Y', length=200, resolution=0.1)
        self.phi_slider = Scale(self.root, from_=round(math.pi, 2), to=round(-math.pi, 2), orient=VERTICAL, label ='PHI', length=200, resolution=0.02)
        self.x_slider.place(relx=0.75, rely=0.1)
        self.y_slider.place(relx=0.82, rely=0.1)
        self.phi_slider.place(relx=0.89, rely=0.1)
        self.shape_center_coords = Label(self.root, text="Circle Center X:Y")
        self.cx = Entry(self.root, text="CX", width=10)
        self.cy = Entry(self.root, text="CY", width=10)
        self.r = Entry(self.root, text="R", width=10)
        self.shape_radius = Label(self.root, text="Circle Radius R")
        self.clockwise = IntVar()
        self.radiobutton_cw = Radiobutton(self.root, text="CW", variable=self.clockwise, value=1)
        self.radiobutton_ccw = Radiobutton(self.root, text="CCW", variable=self.clockwise, value=0)
        self.shape_execute = Button(self.root, text="DRAW!", command=self.draw)

        self.shape_center_coords.place(relx=0.82, rely=0.5)
        self.cx.place(relx=0.83, rely=0.55)
        self.cy.place(relx=0.83, rely=0.6)

        self.shape_radius.place(relx=0.82, rely=0.65)
        self.r.place(relx=0.83, rely=0.7)

        self.radiobutton_cw.place(relx=0.81, rely=0.75)
        self.radiobutton_ccw.place(relx=0.86, rely=0.75)
        self.shape_execute.place(relx=0.84, rely=0.82)

        self.r_value = None
        self.cx_value = None
        self.cy_value = None
        
    def draw(self):
        try:
            self.r_value = float(self.r.get())
            self.cx_value = float(self.cx.get())
            self.cy_value = float(self.cy.get())
        except:
            self.r_value = None
            self.cx_value = None
            self.cy_value = None
    
    def clear_shape_params(self):
        self.r_value = None
        self.cx_value = None
        self.cy_value = None

    def scale_to_screen(self, coords):
        return [round(coords[0]*5+self.width/2), round(self.height-(coords[1]*5+self.height/2))]

    def circle(self, r=5, color='black', coords = None):
        return self.canvas.create_oval(coords[0]-r, coords[1]-r, coords[0]+r, coords[1]+r, fill=color)

    def triangle(self, color='blue', coords1 = None, coords2 = None, coords3 = None):
        return self.canvas.create_polygon([coords1[0], coords1[1], coords2[0], coords2[1], coords3[0], coords3[1]], fill=color)

    def line(self, color='blue', coords1=None, coords2=None):
        return self.canvas.create_line(coords1[0], coords1[1], coords2[0], coords2[1], fill=color, width=10)

    def update_data(self, x, y, phi, theta1, theta2, theta3):
        self.canvas.create_text(80, 20, font=("Arial",16), text="X: " +str(round(x, 2)), fill="black")
        self.canvas.create_text(80, 40, font=("Arial",16), text="Y: " +str(round(y, 2)), fill="black")
        self.canvas.create_text(80, 60, font=("Arial",16), text="PHI: " +str(round(phi, 2)), fill="black")
        self.canvas.create_text(self.width-100, 20, font=("Arial",16), text="MOTOR 1: " +str(round(theta1, 2)), fill="black")
        self.canvas.create_text(self.width-100, 40, font=("Arial",16), text="MOTOR 2: " +str(round(theta2, 2)), fill="black")
        self.canvas.create_text(self.width-100, 60, font=("Arial",16), text="MOTOR 3: " +str(round(theta3, 2)), fill="black")
        self.canvas.create_text(320, self.height-25, font=("Arial",10), text="L5 : Kazup Dániel / Devecser Boglárka / Rutkai Tamás / Juhász Bence", fill="black")


class smartpad():
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.clock.tick(60)
        self.joystick = None
        for i in range(0, pygame.joystick.get_count()):
            self.joystick = pygame.joystick.Joystick(i)
            self.joystick.init()

    def get_input(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis2 = self.joystick.get_axis(2)
                axis3 = self.joystick.get_axis(3)
                leftaxis = self.joystick.get_axis(4)
                rightaxis = self.joystick.get_axis(5)
                return [round(1.5*axis2), -round(1.5*axis3), round(1.5*leftaxis-1.5*rightaxis)]
            
            if event.type == pygame.JOYBUTTONDOWN:
                a = self.joystick.get_button(0)
                b = self.joystick.get_button(1)
                x = self.joystick.get_button(2)
                y = self.joystick.get_button(3)
                return [b-x, y-a, 0]

class circle():
    def __init__(self, r, c, phi, resolution, clockwise):
        self.r = r
        self.c = c
        self.phi=phi
        self.resolution = resolution
        self.clockwise = clockwise
        self.points = []
        if(not clockwise): self.unit_phi = 2*math.pi/self.resolution
        if(clockwise): self.unit_phi = -2*math.pi/self.resolution
        self.phis = [i*self.unit_phi for i in range(resolution+1)]
        for angle in self.phis:
            x = c[0]+self.r*math.cos(angle)
            y = c[1]+self.r*math.sin(angle)
            if (abs(x) > 25 or abs(y) > 25 or abs(self.phi)>math.pi):
                self.points = []
                break
            self.points.append([x, y, self.phi])

def main():
    robot = manipulator()
    run_flag = True
    robot.new_tcp(0, 0, 0.0)
    screen = graphics()
    teachpedant = smartpad()
    slider_values = [0,0,0]

    while(run_flag):
        increment = teachpedant.get_input()

        try:
            screen.root.update_idletasks()
            screen.root.update()
            screen.canvas.delete('all')
            robot.execute_shape()

            if(screen.cx_value is not None and screen.cy_value is not None and screen.r_value is not None):
                robot.buffer.append(circle(screen.r_value, [screen.cx_value, screen.cy_value], robot.tcp[2], 150, bool(screen.clockwise.get())))
                screen.clear_shape_params()
            
            if(increment is not None and robot.buffer_len() ==0):
                tcp_now = robot.tcp
                robot.new_tcp(tcp_now[0]+increment[0], tcp_now[1]+increment[1], tcp_now[2]+0.01*increment[2])

            if(robot.buffer_len() ==0):
                new_slider_values = [screen.x_slider.get(), screen.y_slider.get(), screen.phi_slider.get()]
                if(slider_values[0] != new_slider_values[0] or slider_values[1] != new_slider_values[1] or slider_values[2] != new_slider_values[2]):
                    slider_values = new_slider_values
                    robot.new_tcp(slider_values[0], slider_values[1], slider_values[2])
            else:
                screen.x_slider.set(robot.tcp[0])
                screen.y_slider.set(robot.tcp[1])
                screen.phi_slider.set(robot.tcp[2])

            screen.triangle(coords1=screen.scale_to_screen(robot.shifted_tcps[0]),
            coords2=screen.scale_to_screen(robot.shifted_tcps[1]),
            coords3=screen.scale_to_screen(robot.shifted_tcps[2]))

            [screen.line(coords1=screen.scale_to_screen(robot.shifted_tcps[i]), coords2=screen.scale_to_screen(robot.bearing_coords[i])) for i in range(3)]

            [screen.line(coords1=screen.scale_to_screen(robot.motor_coords[i]), coords2=screen.scale_to_screen(robot.bearing_coords[i])) for i in range(3)]

            [screen.circle(r=8, coords=screen.scale_to_screen(robot.shifted_tcps[i])) for i in range(3)]

            [screen.circle(r=8, coords=screen.scale_to_screen(robot.bearing_coords[i])) for i in range(3)]

            [screen.circle(r=10, coords=screen.scale_to_screen(robot.motor_coords[i])) for i in range(3)]

            screen.update_data(x=robot.tcp[0], y = robot.tcp[1], phi=robot.tcp[2], theta1=robot.thetas[0], theta2=robot.thetas[1], theta3=robot.thetas[2])

        except:
            run_flag = False


if __name__=='__main__':
    main()