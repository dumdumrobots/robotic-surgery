#!/usr/bin/env python3

# --- Import libraries
import os,rospy
import numpy as np

from sensor_msgs.msg import JointState
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
class Robot(object):

    # --- Joint Variables
    q_complete = np.array([0,np.pi,np.pi,np.pi]) # --- 4 DOF
    q_desired = np.array([2048,2048,2048])

    range_position = np.array([0,4095])

    pulley_radii = np.array([0.5,0.5,0.5])

    DXL_q_present = np.array([2048,2048,2048])
    DXL_q_error = np.array([0,0,0])

    # --- General Variables
    DXL_ID = np.array([0,1,2])

    # --- Addresses EEPROM
    ADDR_OP_MODE = 11

    # --- Addresses RAM
    ADDR_POS_D_GAIN = 80
    ADDR_POS_I_GAIN = 82
    ADDR_POS_P_GAIN = 84

    ADDR_TORQUE_EN = 64
    ADDR_LED_EN = 65 
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    # --- Byte Length
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4

    # --- General Settings
    PROT_VR = 2.0
    BRATE = 57600
    DEVICE = '/dev/ttyUSB0'

    portHandler = PortHandler(DEVICE)
    packetHandler = PacketHandler(PROT_VR)

    # --- Sync write and read instances
    position_groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    position_groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)



    def __init__(self):

        # --- Configure port
        try:
            self.portHandler.openPort()
            print("\nSucceeded to open the port.\n")

        except:
            print("\nFailed to open the port.\n")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BRATE)
            print("Baudrate changed to " + str(self.BRATE) + "\n")
        except:
            print("Failed to change the baudrate.\n")
            getch()
            quit()


        # --- ROS COMMS
        self.positionSubscriber = rospy.Subscriber("joint_position_goal",JointState, self.update_goal_position)
        rospy.sleep(0.005)

        self.positionPublisher = rospy.Publisher("dxl_position_present",Int32MultiArray, queue_size=10)
        rospy.sleep(0.005)


        # --- Set up Servomotors 
        for ID in self.DXL_ID:

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, True)
            
            # --- Define Position Control / Range 0 ~ 0,4095, 0.088 per unit
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_OP_MODE, 3)
            
            # --- Define PID gains, with a low I gain to compensate tension
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_POS_P_GAIN, 300)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_POS_I_GAIN, 30)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_POS_D_GAIN, 0)

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, True)

            # --- Verify errors
            if dxl_comm_result != COMM_SUCCESS:

                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print("Failed setup for Motor ID: " + str(ID) + "\n")
                self.shutdown()

            elif dxl_error != 0:

                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                print("Failed setup for Motor ID: " + str(ID) + "\n")
                self.shutdown()

            else:
                print("Motor ID: " + str(ID) + " ready to use!\n")
            


    def radian_to_digital(self,radian):
        return int(radian*self.range_position[1]/(2*np.pi) 
                   + self.range_position[1]/2)
    
    def radii_ratio(self,radian,r1,r2):
        return radian*r1/r2



    def update_goal_position(self,msg):

        data_array = msg.position

        if len(data_array) == len(self.q_complete):
            self.q_desired = np.array(self.radian_to_digital(self.radii_ratio(data_array[1])),
                                      self.radian_to_digital(self.radii_ratio(data_array[2])),
                                      self.radian_to_digital(self.radii_ratio(data_array[3])))

        else:
            print("Invalid Goal Position array, no changes made.\n")



    def publish_present_position(self):

        pub_array = Int32MultiArray()
        pub_array.data = self.DXL_q_present.tolist()

        self.positionPublisher.publish(pub_array)



    def set_goal_position(self):

        for ID in self.DXL_ID:

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.q_desired[ID])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.q_desired[ID])),
                                   DXL_LOBYTE(DXL_HIWORD(self.q_desired[ID])),
                                   DXL_HIBYTE(DXL_HIWORD(self.q_desired[ID]))]

            dxl_addparam_result = self.position_groupSyncWrite.addParam(ID, param_goal_position)

            if dxl_addparam_result != True:

                print("Failed to set Goal Position to Motor ID: " + str(ID) + ". groupSyncWrite addparam failed.\n")
                self.shutdown()

            else: 
                print("Goal Position set to Motor ID: " + str(ID) + " to " + str(self.pwm_desired[ID] * 0.113) + "%." + "\n")


        dxl_comm_result = self.position_groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:

            print("Failed to group write Goal Position.\n")
            self.shutdown()

        self.position_groupSyncWrite.clearParam()



    def read_present_position(self):

        for ID in self.DXL_ID:

            dxl_addparam_result = self.position_groupSyncRead.addParam(ID)

            if dxl_addparam_result != True:
                print("Failed setup for Motor ID: " + str(ID) + ". groupSyncRead addparam failed.\n")
                self.shutdown()

        dxl_comm_result = self.position_groupSyncRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print("Failed to group read Present PWM.\n")
            self.shutdown()

        for ID in self.DXL_ID:

            dxl_getdata_result = self.position_groupSyncRead.isAvailable(ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

            if dxl_getdata_result != True:
                print("Failed to read Present Position from Motor ID: " + str(ID) + ". groupSyncRead getdata failed.\n")
                self.shutdown()

            self.DXL_q_present[ID] = self.position_groupSyncRead.getData(ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        self.position_groupSyncRead.clearParam()



    def shutdown(self):

        for ID in self.DXL_ID:
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, False)
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, False)
            print("Shutting down Motor ID: " + str(ID) + "\n")
        
        self.portHandler.closePort()
        getch()
        quit()



def main():

    rospy.init_node("pwm_node")
    freq = 10
    rate = rospy.Rate(freq)

    robot = Robot()

    while not rospy.is_shutdown():

        robot.set_goal_position()
        robot.read_present_position()
        robot.publish_present_position()

    rospy.on_shutdown(robot.shutdown())


if __name__ == '__main__':
    main()