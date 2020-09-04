



# Simulation of a quadcopter autopilot

# Author: Rufus Fraanje, p.r.fraanje@hhs.nl
# Date:   17/04/2017

# This code is to serve the UAVREG tutorial in the 
# 2nd year Mechatronics program of the Hague University 
# of Applied Sciences.
# The code can be used for free, but comes with no warrantees.
# The author will not take any responsibility for the correctness
# of completeness of this code.
# In fact parts of the code are deleted for the purpose of the
# UAVREG tutrial, and meant to be finished by students.

# import pyqtgraph for the fast 3D graphics using these lines:
# please first install pyqtgraph and OpenGL using
# pip install OpenGL OpenGL_accelerate pyqtgraph
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, BatteryState, Range
import signal

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
# import numpy for vector calculations
import numpy as np
# import trigonometric functions from math
# for single-value evaluation these are faster than
# the ones from numpy
from math import atan2, acos, asin
# import pybullet for physics simulation
import pybullet as p

# import Thread from threading to create threads
# threads are parts of a program than run parallel to eachother
# and are allowed to share data
# using threads we can run the simulation in a separate thread
# and use the python command line to interact with the simulation
# in this way we can change references (setpoints) and controller
# parameters while the simulation is still running and we see the
# effect directly
from threading import Thread

# import the time module, mainly to use the sleep function
# and some time evaluation functions
import time











# load some utility functions
# bullet2pyqtgraph is a function to convert objects from bullet to 
# pyqtgraph, when using this function for meshes of 3D CAD models 
# make sure you have the python module trimesh as well
# install it using the command: pip install trimesh
# quaternion2axis_angle and quaternion2rotation_matrix are two
# functions to convert a quaternion to an axis-angle and a rotation 
# matrix, which are all different ways to describe an orientation.
from util import bullet2pyqtgraph, quaternion2axis_angle, quaternion2rotation_matrix

######################################################################################
# definition of PID controller class
######################################################################################
class PIDcontroller:
    #construction of the object PIDcontroller:
    def __init__(self,Kp,Ki,Kd,dT,e_prev,e_int):
        self.Kp = Kp         # proportional gain
        self.Ki = Ki         # integral gain
        self.Kd = Kd         # derivative gain

        self.dT = dT         # sampling time
        self.inv_dT = 1/dT   # for speeding up calculations (multiplying is
                             # faster than dividing)
        self.e_prev = e_prev # previous error signal
        self.e_int  = e_int  # integral of error up to now

    # calculation of the PID control signal u and update of the items in memory
    # CHANGE THIS: the current definition of the function calc_control
    # is not correct, and should be changed to calculate a PID control action
    def calc_control(self,e_now):
        # PID control signal: Kp * e_now + Ki * e_integral + Kd * e_derivative
        u     = 0 # FIX THIS HERE AKM

        # P
        u += self.Kp * e_now
        # I
        e_int = self.Ki * e_now * self.dT
        u += e_int
        # D
        de = (e_now - self.e_prev) * self.Kd * self.inv_dT
        u += de
        # update memory (state) of PIDcontroller:
        self.e_prev = e_now   # next time, now is previous
        self.e_int  = e_int   # keep the integration of the error up to now
        return u

######################################################################################
# definition of quadcopter control (autopilot) class which makes use of
# PIDcontroller class
######################################################################################
main_time = time.time()
class quadcopter_control:
    # function used to contruct quadcopter_control objects, PID controller
    # parameters Kp, Ki and Kd are 6-dimensional vectors for 
    # x-, y-, z-, pitch-, roll-, and yaw-controllers.
    def __init__(self,Kp,Ki,Kd):
        global Tsample_control
        # controller along global x-, y- and z-axis
        self.x_ctrl = PIDcontroller(Kp[0],Ki[0],Kd[0],Tsample_control,0,0)
        self.y_ctrl = PIDcontroller(Kp[1],Ki[1],Kd[1],Tsample_control,0,0)
        self.z_ctrl = PIDcontroller(Kp[2],Ki[2],Kd[2],Tsample_control,0,0)
        # roll is rotation about quadcopters x-axis
        # pitch is rotation about quadcopters y-axis
        # yaw is rotation about quadcopters z-axis
        self.roll_ctrl  = PIDcontroller(Kp[3],Ki[3],Kd[3],Tsample_control,0,0)
        self.pitch_ctrl = PIDcontroller(Kp[4],Ki[4],Kd[4],Tsample_control,0,0)
        self.yaw_ctrl   = PIDcontroller(Kp[5],Ki[5],Kd[5],Tsample_control,0,0)

        # position (x, y, z), and rpy (roll, pitch, yaw) reference:
        self.ref_pos = np.zeros(3)
        self.ref_rpy = np.zeros(3)
        # offset op roll,pitch,yaw referentie ten behoeve van tunen:
        self.ref_rpy_offset = np.zeros(3)

        self.error_pos = np.zeros(3)
        self.error_rpy = np.zeros(3)

        # krachten voor de 4 actuatoren
        self.force_act1 = 0.
        self.force_act2 = 0.
        self.force_act3 = 0.
        self.force_act4 = 0.
        # moment ten behoeve van yaw-control:
        self.moment_yaw = 0.

        # if true continue simulations
        self.sim    = True
        self.sample = 0
        self.hold   = True

        self.start_time = time.time()

    ##################################################################################
    # main function for quadcopter autopilot
    # inputs are pos_meas (measured position) and quaternion_meas (measured
    # orientation), the function returns the forces for the 4 actuators and
    # the moment for the yaw-control
    # In this function you can include gravity compensation, but no need to
    # change the function in other ways
    ##################################################################################
    def update_control(self,pos_meas,quaternion_meas):
        global quadcopterId, gravity, mass
        R_meas   = quaternion2rotation_matrix(quaternion_meas)
        rpy_meas = p.getEulerFromQuaternion(quaternion_meas)

        if time.time() - self.start_time > 2.0:
            self.ref_pos[2] = 1

        self.error_pos  = self.ref_pos - pos_meas

        # calc. desired force in global x,y,z coordinates:
        force_pos = np.zeros(3)
        force_pos[0] = self.x_ctrl.calc_control(self.error_pos[0])
        force_pos[1] = self.y_ctrl.calc_control(self.error_pos[1])
        force_pos[2] = self.z_ctrl.calc_control(self.error_pos[2])
        # gravity compensation
        force_pos[2] += gravity*mass

        #############################
        # math1 block               #
        #############################
        sign_z =  np.sign(force_pos[2])

        # transform force to quadcopters coordinate frame
        force_pos_local = np.dot(R_meas.T,force_pos.reshape(3,1)).reshape(3)
        #  force_pos_local[2] = thrust, so for each actuator one quarter
        quarter_thrust = 0.25*force_pos_local[2]

        if sign_z == 0: sign_z = 1
        norm_F = np.linalg.norm(force_pos)
        # following equations only hold for yaw = 0!
        self.ref_rpy[0] = asin(-sign_z*force_pos[1]/norm_F)
        self.ref_rpy[1] = atan2(sign_z*force_pos[0],sign_z*force_pos[2])
        # setpoint for yaw = 0:
        self.ref_rpy[2] = 0.
        # to enhance robustness, do not let absolute reference angle be greater
        # than 30 degrees (pi/6 radians)
        if self.ref_rpy[0] < -np.pi/6: self.ref_rpy[0] = -np.pi/6
        if self.ref_rpy[1] < -np.pi/6: self.ref_rpy[1] = -np.pi/6
        if self.ref_rpy[0] > np.pi/6: self.ref_rpy[0] = np.pi/6
        if self.ref_rpy[1] > np.pi/6: self.ref_rpy[1] = np.pi/6

        #############################
        # end of math1 block        #
        #############################

        self.error_rpy = self.ref_rpy_offset + self.ref_rpy - rpy_meas

        moments = np.zeros(3)
        # roll pitch yaw control:
        moments[0] = self.roll_ctrl.calc_control(self.error_rpy[0])
        moments[1] = self.pitch_ctrl.calc_control(self.error_rpy[1])
        # though we will not change the yaw, we need a yaw-controller to stabilize
        # the yaw at zero radians
        moments[2] = self.yaw_ctrl.calc_control(self.error_rpy[2])


        #############################
        # math2 block               #
        #############################
        factor         = .5/arm_length
        self.force_act1 = quarter_thrust - factor*moments[1]
        self.force_act3 = quarter_thrust + factor*moments[1]
        self.force_act2 = quarter_thrust + factor*moments[0]
        self.force_act4 = quarter_thrust - factor*moments[0]
        self.moment_yaw = moments[2]

        # apparantly the forces for applyExternalForce are not in LINK_FRAME, but the positions are
        # so transform forces in right direction (in world-space coordinates)
        force_act1 = self.force_act1*R_meas[:,2]
        force_act2 = self.force_act2*R_meas[:,2]
        force_act3 = self.force_act3*R_meas[:,2]
        force_act4 = self.force_act4*R_meas[:,2]

        #############################
        # end of math2 block        #
        #############################

        return force_act1,force_act2,force_act3,force_act4,moments[2]


# this function is repeatedly evaluated in a separate thread
# and evualates the control-law and updates the physics
# (no need to change this)

class command_subscriber:
    
    def __init__(self):
        self.r = 0
        self.p = 0
        self.y = 0
        self.t = 0
        self.t_fact = 2.0
        self.y_fact = 2.0 #1
        self.rp_fact = 2.0
        self.old_esc_vals = np.zeros(5)
        
    def fact_update(self, in_msg):
        print("HELLO from fact update")
        
        print("TRYING UPDATRE")
        
        msg = in_msg.data.split(" ")
        self.t_fact = float(msg[0])
        self.y_fact = float(msg[1])
        self.rp_fact = float(msg[2])
        print(self.t_fact, self.y_fact, self.rp_fact)
        
        
    def command_update(self, msg):
        #print("HELLO FROM COMMAND UPDATRE")
        msg = msg.data.split(" ")
        self.r = (float(msg[0]) - 1500)/1000.0
        self.p = (float(msg[1]) - 1500)/1000.0
        self.y = (float(msg[2]) - 1500)/1000.0
        self.t = (float(msg[3]) - 1000)/1000.0

        thrust_multiplier = self.t_fact
        yaw_multiplier = self.y_fact
        roll_pitch_multiplier = self.rp_fact
        if self.t < 0:
            thrust_multiplier = 0
            yaw_multiplier = 0
            roll_pitch_multiplier = 0

        self.t *= thrust_multiplier
        self.y *= yaw_multiplier
        self.r *= -roll_pitch_multiplier
        self.p *= roll_pitch_multiplier
        #print("CMD UPDATED ON SIMULTOR", msg)
        
    def get_cmd(self):
        return np.array([self.r, self.p, self.y, self.t])
    
    def get_control(self):
        # order of escs are front-right, rear-right, rear-left, front-left
        # and the fifth is the imaginary yaw which we do not simulate physcially
        esc_vals = np.zeros(5)
        
        
        esc_vals[0] = self.t - self.p + self.r# - self.y
        esc_vals[1] = self.t + self.p + self.r# + self.y
        esc_vals[2] = self.t + self.p - self.r# - self.y
        esc_vals[3] = self.t - self.p - self.r# + self.y
        esc_vals[4] = self.y

        #normalize values
        if not np.array_equal(
                esc_vals, esc_vals):
            #print("CMD UPDATE ON SIMULATOR", esc_vals)
            self.old_esc_vals = esc_vals
        return esc_vals


class physics_sim():
    def __init__(self):
        print("Physcis sim started")
        
    def start(self, delay, quadcopterId, quadcopter_controller):
        imu_pub = rospy.Publisher("/simulation/imu", Imu, queue_size=1, tcp_nodelay=False)
        range_pub = rospy.Publisher('/simulation/range', Range, queue_size=1)
        cmd_sub = command_subscriber()
        rospy.Subscriber("/simulation/simCommands", String, cmd_sub.command_update)
        rospy.Subscriber("/simulation/simfact", String, cmd_sub.fact_update)
        
        self.update_physics(delay, quadcopterId, quadcopter_controller, cmd_sub, imu_pub, range_pub)
    
    def ctrl_c_reset(self, signal,frame):
        p.resetBaseVelocity(quadcopterId, [0.0,0.0,0.0],[0.0,0.0,0.0])
        p.resetBasePositionAndOrientation(quadcopterId, [0.0,0.0,1.0],
                                          [0.0,0.0,0.0, 1.0])
    
    def update_physics(self, delay,quadcopterId,quadcopter_controller, cmd_sub, imu_pub, range_pub):
        vol_meas = p.getBaseVelocity(quadcopterId)
        one_over_sample_rate = 1/delay
        previous_publish = time.perf_counter()
        try:
            while not rospy.is_shutdown():
                start = time.perf_counter();
                # the controllers are evaluated at a slower rate, only once in the
                # control_subsample times the controller is evaluated
                pos_meas, quaternion_meas = p.getBasePositionAndOrientation(quadcopterId)
                previous_vol_meas = np.array(vol_meas)
                vol_meas = np.array(p.getBaseVelocity(quadcopterId))
                #print(vol_meas, type(vol_meas))
                acc = (vol_meas - previous_vol_meas) * one_over_sample_rate
                #print("VOL MEAS", previous_vol_meas, acc)
                
                if quadcopter_controller.sample == 0:
                    quadcopter_controller.sample = control_subsample
                    
                    #force_act1,force_act2,force_act3,force_act4,moment_yaw = quadcopter_controller.update_control(pos_meas,quaternion_meas)
                    actual_values = cmd_sub.get_control()
                    vectorizer = np.zeros([5,3])
                    vectorizer[:,2] = actual_values
                    #print("input forces ", actual_values)
                    force_act1, force_act2, force_act3, force_act4, moment_yaw = vectorizer
                    #print("Well well well\t\t", vectorizer, force_act1, moment_yaw)
                else:
                    quadcopter_controller.sample -= 1
        
        
                # apply forces/moments from controls etc:
                # (do this each time, because forces and moments are reset to zero after a stepSimulation())
        
                #assuming right is -y
                
                p.applyExternalForce(quadcopterId, -1, force_act1, [arm_length, -arm_length, 0.], p.LINK_FRAME)
                p.applyExternalForce(quadcopterId,-1,force_act2,[-arm_length,-arm_length,0.], p.LINK_FRAME)
                p.applyExternalForce(quadcopterId,-1,force_act3,[-arm_length,arm_length,0.],p.LINK_FRAME)
                p.applyExternalForce(quadcopterId,-1,force_act4,[arm_length,arm_length,0.],p.LINK_FRAME)
        
                # for the yaw-control:
                p.applyExternalTorque(quadcopterId,-1,moment_yaw,p.LINK_FRAME)
            
                # do simulation with pybullet:
        
                p.stepSimulation()
    
                header = Header()
                header.frame_id = 'Body'
                header.stamp = rospy.Time.now()
                Tsample_physics
    
                imu_message = Imu()
                imu_message.header = header
                imu_message.orientation.x = quaternion_meas[0]
                imu_message.orientation.y = quaternion_meas[1]
                imu_message.orientation.z = quaternion_meas[2]
                imu_message.orientation.w = quaternion_meas[3]
                imu_message.linear_acceleration.x = acc[0,0]
                imu_message.linear_acceleration.y = acc[0,1]
                imu_message.linear_acceleration.z = acc[0,2]
        
                
                
                msg = Range()
                msg.header = header
                msg.max_range = 200
                msg.min_range = 0
                msg.range = pos_meas[2]
                
                if start - previous_publish > 1/100:
                    imu_pub.publish(imu_message)
                    range_pub.publish(msg)
                    previous_publish = start
    
                #print("BUBLISHED RANON /simulation/infrared_sensor_node", msg.range)
                # delay than repeat
                calc_time = time.perf_counter()-start
                if ( calc_time > delay ):
                    #print("Time to update physics is {} and is more than 20% of the desired update time ({}).".format(calc_time,delay))
                    pass
                else:
                    # print("calc_time = ",calc_time)
                    while (time.perf_counter()-start < delay):
                        time.sleep(delay/10)
                        
        except KeyboardInterrupt:
            
            self.ctrl_c_reset()


# this function is to update the window (no need to change this)
def update_window():
    try:
        global quadcopterId, quadcopterMesh, w
        pos_meas,quat_meas = p.getBasePositionAndOrientation(quadcopterId)
        angle,x,y,z = quaternion2axis_angle(quat_meas)
        ll = np.eye(4).flatten()
        quadcopterMesh.setTransform([np.eye(4).flatten()])
        quadcopterMesh.rotate(np.degrees(angle),x,y,z,local=True)
        quadcopterMesh.translate(pos_meas[0],pos_meas[1],pos_meas[2])
        
        window.update()
    except KeyboardInterrupt:
        import sys
        print ("SYS EXIT IN update window")
        sys.exit()
        

################################################################################
# The above is just definition of classes and functions
# Here actual python script for the simulation starts
################################################################################

# Definition of update times (in sec.) for quadcopter physics, controller and 
# window refreshing
Tsample_physics    = 0.0001
control_subsample  = 50
Tsample_control    = control_subsample * Tsample_physics
Tsample_window     = 0.02

# definition of number of constants: 
gravity = 9.8
# arm length, mass and moments of inertia of quadcopter
arm_length       = 0.1
mass             = 0.5
Ixx = Iyy        = 0.0023
Izz              = 0.004

# creation of pyqtgraph 3D graphics window
# with a ground plane and coordinate frame (global axis)


app = QtGui.QApplication([])
# PROBLEM IS THIS LINE
window = gl.GLViewWidget()

window.show()
print("HELLO")
window.setWindowTitle('Bullet Physics example')
grid = gl.GLGridItem()
window.addItem(grid)
global_axis = gl.GLAxisItem()
global_axis.updateGLOptions({'glLineWidth':(4,)})
window.addItem(global_axis)
print("WINDOW not updated")
window.update()

# configure pybullet and load plane.urdf and quadcopter.urdf
physicsClient = p.connect(p.DIRECT)  # pybullet only for computations no visualisation
p.setGravity(0,0,-gravity)
p.setTimeStep(Tsample_physics)
# disable real-time simulation, we want to step through the
# physics ourselves with p.stepSimulation()
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]))
quadcopterId = p.loadURDF("quadrotor.urdf",[0,0,1],p.getQuaternionFromEuler([0,0,0]))
# do a few steps to start simulation and let the quadcopter land safely

for i in range(int(2/Tsample_physics)):
    p.stepSimulation()

# create a pyqtgraph mesh from the quadcopter to visualize
# the quadcopter in the 3D pyqtgraph window
quadcopterMesh = bullet2pyqtgraph(quadcopterId)[0]
window.addItem(quadcopterMesh)
window.update()

# Initialize PID controller gains:
Kp = np.zeros(6)
Ki = np.zeros(6)
Kd = np.zeros(6)

# give them values:
# x-y-z controlers:
xy = 0.05
z = 0.5
Kp[0] = xy*0.5
Kp[1] = xy*0.5
Kp[2] = z*0.5

Kd[0] = xy*1.0
Kd[1] = xy*1.0
Kd[2] = 1.0*z

Ki[0] = xy*0.25
Ki[1] = xy*0.25
Ki[2] = 0.25*z

# roll-pitch-yaw controlers (yaw is already prefilled):
Kp[3] = 2.0
Kp[4] = 2.0
Kp[5] = 25.6

Kd[3] = 0.1
Kd[4] = 0.1
Kd[5] = 1.28

Ki[3] = 1.0
Ki[4] = 1.0
Ki[5] = 0

# create the quadcopter control (i.e. the autopilot) object
# which is named qcc

qcc = quadcopter_control(Kp,Ki,Kd)

# start the update_physics that updates both physics and control
# in a separate thread
# this allows us to let the physics+control simulation run 
# completely in the background and we keep a python command line
# which can be used to interact with the quadcopter
# e.g. we can manually change setpoints and change control parameters
app.setQuitOnLastWindowClosed(False)

print('before physics')
rospy.init_node("simulator")
max_range = 200.0
rospy.set_param("/carbon/maxrange", str(max_range))
psim = physics_sim()
signal.signal(signal.SIGINT, psim.ctrl_c_reset)
thread_physics = Thread(target=psim.start,args=(Tsample_physics,quadcopterId,qcc))
# start the thread:
thread_physics.start()
#print('after physics')
# the graphics window is updated every Tsample_window seconds
# using a timer function from the Qt GUI part of pyqtgraph
# this also runs in the background, but at a much lower speed than
# the physics and control updates.
#print('a')
timer_window = QtCore.QTimer()
#print('b')
timer_window.timeout.connect(update_window)
#print('c')
timer_window.start()
#print('d')
# END
# Tips for using:
print('qcc errorpos', qcc.error_pos, ' dk ', qcc.z_ctrl.Kp)
#qcc.ref_pos = np.array([0,0,1])

QtGui.QApplication.instance().exec_()
