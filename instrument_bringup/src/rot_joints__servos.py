#!/usr/bin/env python3

# --- Import libraries
import os,rospy
import numpy as np

from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import *


# --- Serial setup
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



# --- Define a Robot with multiple DYNAMIXEL motors. 
class Servos(object):

    def __init__(self):

         # --- Joint Variables
        self.goal_position = [2048,2048,2048] # --- Desired Position
        self.present_position = [2048,2048,2048] # --- Present Position

        # --- General Variables
        self.DXL_ID = np.array([0,1,2])

        # --- Addresses EEPROM
        self.ADDR_OP_MODE = 11
        self.ADDR_MAX_POS_LIMIT = 48
        self.ADDR_MIN_POS_LIMIT = 52
        #125 Current 

        # --- Addresses RAM
        self.ADDR_TORQUE_EN = 64
        self.ADDR_LED_EN = 65 
        self.ADDR_GOAL_POS = 116
        self.ADDR_PRESENT_POS = 132
        self.ADDR_POS_D_GAIN = 80
        self.ADDR_POS_I_GAIN = 82
        self.ADDR_POS_P_GAIN = 84

        # --- Byte Length
        self.LEN_GOAL_POS = 4
        self.LEN_PRESENT_POS = 4


        # --- General Settings
        self.PROT_VR = 2.0
        self.BRATE = 57600
        self.DEVICE = '/dev/ttyUSB1'

        self.portHandler = PortHandler(self.DEVICE)
        self.packetHandler = PacketHandler(self.PROT_VR)

        self.pos_groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POS, self.LEN_GOAL_POS)
        self.pos_groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POS, self.LEN_PRESENT_POS)

        # --- Configure port
        try:
            self.portHandler.openPort()
            rospy.loginfo("Succeeded to open the port.")

        except:
            rospy.logfatal("Failed to open the port.")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BRATE)
            rospy.loginfo("Baudrate changed to " + str(self.BRATE))
        except:
            rospy.logfatal("Failed to change the baudrate.")
            getch()
            quit()


        # --- ROS COMMS
        self.posSubscriber = rospy.Subscriber("rot_joints__goal_position",Int32MultiArray, self.update_goal_position)
        rospy.sleep(0.005) #pwm_goal_value

        self.posPublisher = rospy.Publisher("rot_joints__present_position",Int32MultiArray, queue_size=10)
        rospy.sleep(0.005) #pos_goal_value


        # --- Set up Servos 
        for ID in self.DXL_ID:

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, True)
            
            # --- Define Position Control / 4095 to 0 at 0.088 / 512 (45deg)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_OP_MODE, 3)

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_D_GAIN, 0)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_I_GAIN, 30)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_POS_P_GAIN, 400)

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_MAX_POS_LIMIT, 2560)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_MIN_POS_LIMIT, 1536)

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, True)

            # --- Verify errors
            if dxl_comm_result != COMM_SUCCESS:

                rospy.logfatal("%s" % self.packetHandler.getTxRxResult(dxl_comm_result) + 
                               "\nFailed setup for Motor ID: " + str(ID))
                self.shutdown()

            elif dxl_error != 0:

                rospy.logfatal("%s" % self.packetHandler.getTxRxResult(dxl_comm_result) + 
                               "\nFailed setup for Motor ID: " + str(ID))
                self.shutdown()

        rospy.loginfo("Servos enabled and ready to use.")


    def update_goal_position(self,msg):

        data_array = msg.data # --- Int32MultiArray data

        if len(data_array) == len(self.DXL_ID):
            self.goal_position = data_array

        else:
            rospy.logerr("Invalid array recieved from callback {callback}.".format(callback="update_goal_position"))


    def publish_present_position(self):

        pub_array = Int32MultiArray()
        pub_array.data = self.present_position

        self.posPublisher.publish(pub_array)


    def set_goal_position(self):

        for ID in self.DXL_ID:

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.goal_position[ID])), DXL_HIBYTE(DXL_LOWORD(self.goal_position[ID])),
                                   DXL_LOBYTE(DXL_HIWORD(self.goal_position[ID])), DXL_HIBYTE(DXL_HIWORD(self.goal_position[ID]))]

            dxl_addparam_result = self.pos_groupSyncWrite.addParam(ID, param_goal_position)

            if dxl_addparam_result != True:
                rospy.logerr("Failed to set Goal Position for Motor ID: " + str(ID) + ". groupSyncWrite addparam failed.\n")
                self.shutdown()
            else: 
                pass
                #rospy.loginfo("Goal Position set to Motor ID: " + str(ID) + " to " + str(self.goal_position[ID]) + " units." + "\n")


        dxl_comm_result = self.pos_groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:

            rospy.logfatal("Failed to group write Goal Position.")
            self.shutdown()

        self.pos_groupSyncWrite.clearParam()


    def read_present_position(self):

        for ID in self.DXL_ID:

            dxl_addparam_result = self.pos_groupSyncRead.addParam(ID)

            if dxl_addparam_result != True:
                rospy.logerr("Failed setup for Motor ID: " + str(ID) + ". groupSyncRead addparam failed.")
                self.shutdown()

        dxl_comm_result = self.pos_groupSyncRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            rospy.logfatal("Failed to group read Present Position." + "%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.shutdown()

        for ID in self.DXL_ID:

            dxl_getdata_result = self.pos_groupSyncRead.isAvailable(ID, self.ADDR_PRESENT_POS, self.LEN_PRESENT_POS)

            if dxl_getdata_result != True:
                rospy.logerr("Failed to read Present Position from Motor ID: " + str(ID) + ". groupSyncRead getdata failed.")
                self.shutdown()

            self.present_position[ID] = self.pos_groupSyncRead.getData(ID, self.ADDR_PRESENT_POS, self.LEN_PRESENT_POS)

        self.pos_groupSyncRead.clearParam()


    def shutdown(self):

        rospy.logwarn("Shutting down all motors.")

        for ID in self.DXL_ID:
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, False)
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, False)
        
        self.portHandler.closePort()
        getch()
        quit()


def main():

    rospy.init_node("rot_motors_node")
    freq = 200
    rate = rospy.Rate(freq)

    robot = Servos()

    while not rospy.is_shutdown():

        robot.set_goal_position()
        robot.read_present_position()
        robot.publish_present_position()

        rate.sleep()

    rospy.on_shutdown(robot.shutdown())


if __name__ == '__main__':
    main()