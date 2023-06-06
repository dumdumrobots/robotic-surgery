#!/usr/bin/env python

import os
import rospy

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from surgeon.msg import *

from dynamixel_sdk import *


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

#Address

ADDR_TORQUE_EN = 64
ADDR_LED_EN = 65

ADDR_GOAL_POSITION = 116

#PID Address
ADDR_POS_P_GAIN = 400
ADDR_POS_D_GAIN = 0


#Protocol
PROT_VR = 2.0

                                     
#Settings
DXL_ID = [0,1,2]
BRATE = 57600
DEVICE = '/dev/ttyUSB0'


#12-bit 
b_zero = 0
b_max = 4095

portHandler = PortHandler(DEVICE)
packetHandler = PacketHandler(PROT_VR)


q_recieved = [0,0,0,0]
q_norm = [b_max/2,b_max/2,b_max/2]

def norm_angles(x):

    y = b_max/(2*np.pi)*x + b_max/2

    return int(y)


def robotAngles(data):
    global q_recieved
    global q_norm

    q_recieved = data.position
    q_norm = [norm_angles(q_recieved[1]), norm_angles(q_recieved[2]), norm_angles(q_recieved[3])]


def set_goal_pos(ID, position):

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, position)

    #if dxl_comm_result != COMM_SUCCESS:
    #    print("Timeout for Dynamixel ID: " + str(ID) + "\n")
     #   getch()
      #  quit()

    #elif dxl_error != 0:
     #   print("Timeout setup for Dynamixel ID: " + str(ID) + "\n")
      #  getch()
       # quit()

def motor_movement_node():

    rospy.init_node("motorMovement")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.Subscriber("joint_states",JointState,robotAngles)

        for ID in DXL_ID:
            set_goal_pos(ID,q_norm[ID])

        rate.sleep()

    rospy.on_shutdown(shutdown())

def shutdown():

    print("Shutting down DYNAMIXEL motors!\n")

    # Disable Dynamixel

    for ID in DXL_ID:

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_LED_EN, False)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_EN, False)


def main():
    # Open port
    try:
       portHandler.openPort()
       print("\nSucceeded to open the port\n")
    except:
        print("Failed to open the port\n")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BRATE)
        print("Succeeded to change the baudrate\n")
    except:
        print("Failed to change the baudrate\n")
        getch()
        quit()

    # Enable Dynamixel
    for ID in DXL_ID:

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_LED_EN, True)

        #Settings PID
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_POS_P_GAIN, 175)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_POS_D_GAIN, 75)

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_EN, True)
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("Failed setup for Dynamixel ID: " + str(ID) + "\n")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("Failed setup for Dynamixel ID: " + str(ID) + "\n")
            getch()
            quit()
        else:
            print("Dynamixel ID: " + str(ID) + " ready to use!\n")

        
        

    motor_movement_node()


if __name__ == '__main__':
    main()


