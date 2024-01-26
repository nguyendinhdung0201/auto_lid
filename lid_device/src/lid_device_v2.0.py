#!/usr/bin/env python3
import can
import rospy
from cargobot_msgs.msg import LidCmd, LidStatus


interface = 'socketcan'
channel = 'can0'

ID_DEVICE = 0x1A
ID_CMD_PKG = 0x01
ID_CONF_PKG = 0x02
ID_STATUS_PKG_1 = 0x11
ID_STATUS_PKG_2 = 0x12

def id_extended(id_device,id_pkg):
    return (id_pkg << 8) | id_device
def id_device_pkg(id_extended):
    return id_extended & 0xFF, id_extended >> 8

class LidDevice:
    def __init__(self):
        rospy.init_node("lid_device")
        rospy.Subscriber("/lid/cmd",LidCmd,self.cmdCallback,queue_size=10)
        # rospy.Service("/lid/config",LidCmd,self.send_config)
        self.status_pub = rospy.Publisher("/lid/status",LidStatus,queue_size=10)
        self.close_speed = rospy.get_param('/lid/close_speed')
        self.open_speed = rospy.get_param('/lid/open_speed')
        self.threshold = rospy.get_param('/lid/threshold')
        try:
            self.bus = can.Bus(channel=channel, interface=interface)
        except can.CanError:
            rospy.logerr("CAN error")
            rospy.signal_shutdown("CAN bus init error")
        self.msg_status = LidStatus()
        self.config()
    def config(self):
        close_speed = self.close_speed
        open_speed = self.open_speed
        threshold_upper = (self.threshold >> 8) & 0xFF
        threshold_lower = self.threshold & 0xFF
        checksum = close_speed ^ open_speed ^ threshold_upper ^ threshold_lower
        data_config = [close_speed, open_speed, threshold_upper, threshold_lower, checksum]
        message = can.Message(arbitration_id=id_extended(ID_DEVICE,ID_CONF_PKG), data=data_config, is_extended_id=True)
        print(message)
        try:
            self.bus.send(message)
            rospy.loginfo('Succesfully')
        except can.CanError:
            rospy.logerr('Error sending data config')
  
    def cmdCallback(self,msg):
        if msg.lid_cmd == LidCmd.OPEN:
            data_=[1]
            message = can.Message(arbitration_id=id_extended(ID_DEVICE,ID_CMD_PKG), data=data_, is_extended_id=True)
            print(message)
            try:
                self.bus.send(message)
                rospy.loginfo('Succesfully')
            except can.CanError:
                rospy.logerr('Error sending data open')
        elif msg.lid_cmd == LidCmd.CLOSE:
            data_=[0]
            message = can.Message(arbitration_id=id_extended(ID_DEVICE,ID_CMD_PKG), data=data_, is_extended_id=True)
            print(message)
            try:
                self.bus.send(message)
                rospy.loginfo('Succesfully')
            except can.CanError:
                rospy.logerr('Error sending data close')
    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg != None:
                    id_device, id_pkg = id_device_pkg(msg.arbitration_id)
                    if id_device == ID_DEVICE:
                        if id_pkg == ID_STATUS_PKG_1:
                            status = msg.data[0]
                            current_threshold = int.from_bytes(msg.data[1:3], byteorder='big', signed=False)
                            button_status = msg.data[3]
                            current = int.from_bytes(msg.data[4:6],byteorder='big',signed=False)
                            voltage = int.from_bytes(msg.data[6:8],byteorder='big',signed=False)

                            self.msg_status.status = status
                            self.msg_status.current_threshold = current_threshold
                            self.msg_status.button_state = button_status
                            self.msg_status.current = current/1000
                            self.msg_status.voltage = float(voltage)/10
                            self.msg_status.header.seq += 1
                            self.msg_status.header.stamp = rospy.Time.now()
                            self.msg_status.header.frame_id = "LID"
                            self.status_pub.publish(self.msg_status)
                        elif id_pkg == ID_STATUS_PKG_2:
                            self.msg_status.header.seq += 1
                            error = msg.data[0]
                            self.msg_status.error = error
                rate.sleep()
            except can.CanError:
                rospy.logwarn('Error receive CAN data')
if __name__ == "__main__":
    try:
        a = LidDevice()
        a.main()
    except Exception as e:
        pass


