#!/usr/bin/env python3
import can
import rospy
from std_srvs.srv import Empty, EmptyResponse
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
class CANsend:
    def __init__(self):
        rospy.init_node('can_send')
        self.service = rospy.Service('can_send_config',Empty,self.can_send_config)
        rospy.loginfo('CAN Send is ready')
        # self.close_speed = rospy.get_param('/lid/close_speed')
        # self.open_speed = rospy.get_param('/lid/open_speed')
        # self.threshold = rospy.get_param('/lid/threshold')
        self.close_speed = 210
        self.open_speed = 210
        self.threshold = 650
        try:
            self.bus = can.Bus(channel=channel, interface=interface)
        except can.CanError:
            rospy.logerr("CAN error")
            rospy.signal_shutdown("CAN bus init error")
    def can_send_config(self, request):
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
        return EmptyResponse()
    def run(self):
        rospy.spin()
if __name__ == "__main__":
    try:
        a = CANsend()
        a.run()
    except Exception as e:
        pass

        