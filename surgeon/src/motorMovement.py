#!/usr/bin/env python

# --- Import libraries

import os
import rospy

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from surgeon.msg import *

from dynamixel_sdk import *

import copy


# ----- Serial setup

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# ----------- Classes

class Robot(object):

    # ------------------- Joint Variables

    qChain = [0,np.pi,np.pi,np.pi]
    qControl = [2048,2048,2048] # --- Desired Position
    qDXL = [2048,2048,2048] # --- Actual Position
    qError = [0,0,0]

    # ------------------- General Variables
    
    DXL_ID = [0,1,2]

    ADDR_TORQUE_EN = 64
    ADDR_LED_EN = 65

    ADDR_GOAL_POSITION = 116
    ADDR_PRO_PRESENT_POSITION = 132

    
    ADDR_POS_D_GAIN = 80
    ADDR_POS_I_GAIN = 82
    ADDR_POS_P_GAIN = 84

    b_max = 4095 #12-bit Max Value

    # ------------------- General Settings

    PROT_VR = 2.0
    BRATE = 57600
    DEVICE = '/dev/ttyUSB0'


    portHandler = PortHandler(DEVICE)
    packetHandler = PacketHandler(PROT_VR)
    


    def __init__(self):


        # ------------------------------------- Open port
        try:
            self.portHandler.openPort()
            print("\nSucceeded to open the port.\n")

        except:
            print("Failed to open the port. Shutting down...\n")
            getch()
            quit()

        # ------------------------------------- Set Baudrate

        try:
            self.portHandler.setBaudRate(self.BRATE)
            print("Baudrate changed to " + str(self.BRATE) + "\n")
        except:
            print("Failed to change the baudrate. Shutting down...\n")
            getch()
            quit()


        # ------------------------------------- Create Joint_State subscriber

        jointSubscriber = rospy.Subscriber("joint_states",JointState,self.Joints)
        
        rospy.sleep(0.005)


        # ------------------------------------- Set up Servomotors 

        for ID in self.DXL_ID:

            print(ID)

            # ------------------------------------- Identify

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, True)
            

            # ------------------------------------- Set PID

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_D_GAIN, 0)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_I_GAIN, 30)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_P_GAIN, 300)

            # ------------------------------------- Turn on

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, True)
            

            # ------------------------------------- Verify errors

            if dxl_comm_result != COMM_SUCCESS:

                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print("Failed setup for DYNAMIXEL Motor ID: " + str(ID) + "\n")
                self.Shutdown()

            elif dxl_error != 0: 
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                print("Failed setup for DYNAMIXEL Motor ID: " + str(ID) + "\n")
                self.Shutdown()

            else:
                print("DYNAMIXEL Motor ID: " + str(ID) + " ready to use!\n")



    def Norm(self,x):

        # --- Normalize radians to digital

        y = self.b_max/(2*np.pi)*x + self.b_max/2

        return int(y)


    def Joints(self,msg):

        # -------------------------- Recieved angles from subscriber

        data = msg.position

        self.qChain = data

        self.qControl = [self.Norm(self.Radii(self.qChain[1],1.5,1)),
                         self.Norm(self.Radii(self.qChain[2],1.5,1)),
                         self.Norm(self.Radii(self.qChain[3],1.5,1))]

    def Radii(self,x,r1,r2):

        # -------------------------- Radius Ratio
        y = x*r1/r2
        return y



    def SetPosition(self):

        # -------------------------- Set positions to servomotors

        for ID in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_GOAL_POSITION, self.qControl[ID])

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("Timeout for DYNAMIXEL Motor ID: " + str(ID) + "\n")
                self.Shutdown()


    def GetPosition(self):

        # -------------------------- Get positions from servomotors

        for ID in self.DXL_ID:
            self.qDXL[ID], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PRESENT_POSITION)

            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("Timeout for DYNAMIXEL Motor ID: " + str(ID) + "\n")
                self.Shutdown()

        # -------------------------- Calculate position error

        for i in range(len(self.qDXL)):
            self.qError[i] = self.qDXL[i] - self.qControl[i]

        # -------------------------- Return values

        return self.qControl, self.qDXL, self.qError


    def Shutdown(self):

        for ID in self.DXL_ID:

            # -------------------------- Turn off

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, False)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, False)

            print("Shutdown DYNAMIXEL Motor ID: " + str(ID) + "\n")
        
        getch()
        quit()


def main():

    rospy.init_node("motor_movement_node")
    freq = 10
    rate = rospy.Rate(freq)

    robot = Robot()

    while not rospy.is_shutdown():

        robot.SetPosition()

        rate.sleep()

    rospy.on_shutdown(robot.Shutdown())


if __name__ == '__main__':
    main()


