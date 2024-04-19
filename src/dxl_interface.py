#!/usr/bin/env python

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_robot dxl_interface.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_robot/servo_position "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_robot/servo_position "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import os
import rospy
import time
from dynamixel_sdk import *
from dynamixel_sdk_robot.srv import *
from dynamixel_sdk_robot.msg import *

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

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


def ping_devices(port, packetHandler, cand_ids):
    """Ping to candidate devices which port and ids are specified
    Args:
        port:PortHandler
        cand_ids:Tuple[int] (each id must be in range [0,255])
    Return:
        respond_ids:Tuple[int] 
    """
    respond_ids = []
    for cand_id in cand_ids:
        model_number, result, error = packetHandler.ping(port, cand_id)
        if result == COMM_SUCCESS:
            respond_ids.append(cand_id)

    return respond_ids

def check_connection(target_ids, connected_ids):
    success = True
    for target_id in target_ids:
        if target_id not in connected_ids:
            print(f"id:{target_id} not connected")
            success = False
    return success

int32_max_value = 2**31
int32_ceil_value = 2**32

def round_to_int32(data):
    if data >= int32_max_value:
        data -= int32_ceil_value
    return data

    

class DxlInterface:
    def __init__(self, cand_ids=None):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self._init_port()
        
        self.last_port_used_time = time.time()

        self.groupSyncReader = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, 4)
        self.groupSyncWriter = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, 4)
        self.connected_ids = []
        self.init_connection(cand_ids)

        rospy.init_node('dxl_interface_node')
        rospy.Subscriber('set_position_array', servo_position_array, self.set_position_array_callback)
        # rospy.spin()
        servo_pos_publisher = rospy.Publisher('servo_position_array', servo_position_array, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            servo_pos_array = self.get_position_array()
            try:
                servo_pos_publisher.publish(servo_pos_array)
            except rospy.exceptions.ROSSerializationException as e:
                print("servo_pos_array=", servo_pos_array)
                print(e)
                quit()
            rate.sleep()

    
    def _init_port(self):
        # Open port
        try:
           self.portHandler.openPort()
           print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        print("initialize port successfully")



    def init_connection(self, cand_ids=None):
        if cand_ids is None:
            # ping to [1, 252] (id_max=252 according to 
            cand_ids = [i for i in range(1,253)]
            self.connected_ids = ping_devices(self.portHandler, cand_ids)
        else:
            # ping to given ids
            self.connected_ids = ping_devices(self.portHandler,self.packetHandler, cand_ids)
            if not check_connection(cand_ids, self.connected_ids):
                # if some devices are missing, quit
                print("[ERROR]: failed to establish all servos")
                quit()
        # Enable torque for each servo
        for connected_id in self.connected_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, connected_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print("dxl_comm_result=", dxl_comm_result)
                print("Press any key to terminate...")
                getch()
                quit()

    def set_position_array_callback(self, servo_position_array):
        for servo_position in servo_position_array.data:
            if servo_position.id not in self.connected_ids:
                print(f"specified id: {servo_position.id} is not connected. skip")
                continue
            # print(f"id:{servo_position.id} try to set position: {servo_position.position}")
            pos_byte_arr = [ DXL_LOBYTE(DXL_LOWORD(servo_position.position)),
                            DXL_HIBYTE(DXL_LOWORD(servo_position.position)),
                            DXL_LOBYTE(DXL_HIWORD(servo_position.position)),
                            DXL_HIBYTE(DXL_HIWORD(servo_position.position))]

            dxl_add_param_result = self.groupSyncWriter.addParam(servo_position.id, pos_byte_arr)
            if not dxl_add_param_result:
                print(f"Failed to addparam to groupSyncWrite for Dynamixel ID {servo_position.id}");

        dxl_comm_result = self.groupSyncWriter.txPacket()
        self.last_port_used_time = time.time()
        print(f"[SET]: {self.last_port_used_time}")

        if dxl_comm_result is not COMM_SUCCESS:
            print(f"Failed to set position! Result: {dxl_comm_result}")
        self.groupSyncWriter.clearParam()

    def get_position_array(self):
        for connected_id in self.connected_ids:
            dxl_add_param_result = self.groupSyncReader.addParam(connected_id)
            if not dxl_add_param_result:
                print(f"Failed to addparam to groupSyncWrite for Dynamixel ID {servo_position.id}");

        dxl_comm_result = self.groupSyncReader.txRxPacket()
        self.last_port_used_time = time.time()
        print(f"[GET]: {self.last_port_used_time}")

        if dxl_comm_result is not COMM_SUCCESS:
            print(f"Failed to set position! Result: {dxl_comm_result}")
        pos_array = []
        for connected_id in self.connected_ids:
            pos = servo_position()
            pos.id = connected_id
            pos.position = round_to_int32(self.groupSyncReader.getData(connected_id, ADDR_PRESENT_POSITION, 4))
            pos_array.append(pos)
        self.groupSyncReader.clearParam()
        return servo_position_array(data=pos_array)
        



def main():
    dxl_interface = DxlInterface([1, 2, 3, 4, 5, 6])

if __name__ == '__main__':
    main()
