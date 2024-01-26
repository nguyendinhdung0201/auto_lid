#!/usr/bin/env python3
import rospy
import can
from cargobot_msgs.msg import LidCmd,LidStatus

interface = 'socketcan'
channel = 'can0'

ID_DEVICE = 0x1A
ID_CMD_PKG = 0x01
ID_CONF_PKG = 0x02
ID_STATUS_PKG_1 = 0x11
ID_STATUS_PKG_2 = 0x12


def id_encode(id_device,id_pkg):
    return (id_pkg << 8) | id_device
def id_decode(id_msg):
    return id_msg & 0xff, id_msg >> 8

class LidDevice:
    def __init__(self):
        rospy.init_node("lid_device")
        rospy.Subscriber("/lid/cmd",LidCmd,self.cmdCallback,queue_size=10)
        self.status_pub = rospy.Publisher("/lid/status",LidStatus,queue_size=10)
        # params = rospy.get_param("~", {})
        # print(params)
        # self.speed = int(params["speed"])
        # self.threshold = int(params["threshold"])
        # print(params)
        self.speed=rospy.get_param('/lid/speed')
        self.threshold=rospy.get_param('/lid/threshold')
        # print(self.speed)
        try:
            self.bus = can.Bus(channel=channel, interface=interface)
        except can.CanError:
            rospy.logerr("CAN error")
            rospy.signal_shutdown("CAN bus init error")
        self.msg_status = LidStatus()
        self.config()
       

    def config(self):
        data_config = self.threshold.to_bytes(2,byteorder='big',signed=False)
        message = can.Message(arbitration_id=id_encode(ID_DEVICE,ID_CONF_PKG), data = data_config, is_extended_id=True, dlc=8)
        print(message)
        try:
            self.bus.send(message)
            rospy.loginfo('Succesfully')
        except can.CanError:
            rospy.logerr('Error sending config')
    
    def cmdCallback(self,msg):
        if msg.lid_cmd == LidCmd.OPEN:
            val = self.speed
            data_ = val.to_bytes(2,byteorder='big',signed=True)
            message = can.Message(arbitration_id=id_encode(ID_DEVICE,ID_CMD_PKG), data = data_, is_extended_id=True)
            print(message)
            try:
                self.bus.send(message)
                rospy.loginfo('Succesfully')
            except can.CanError:
                rospy.logerr('Error sending data')
        
        elif msg.lid_cmd == LidCmd.CLOSE :
            val = - self.speed
            data_ = val.to_bytes(2,byteorder='big',signed=True)
            message = can.Message(arbitration_id=id_encode(ID_DEVICE,ID_CMD_PKG), data = data_, is_extended_id=True)
            print(message)
            try:
                self.bus.send(message)
                rospy.loginfo('Succesfully')
            except can.CanError:
                rospy.logerr('Error sending data')
    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try: 
                msg = self.bus.recv(timeout=0.1)
                if msg != None:
                    id_device, id_pkg = id_decode(msg.arbitration_id)
                    # print(id_device)
                    if id_device == ID_DEVICE:
                        if id_pkg == ID_STATUS_PKG_1:
                            status = msg.data[0]
                            threshold_current = int.from_bytes(msg.data[1:3],byteorder='big',signed=False)
                            current = int.from_bytes(msg.data[3:5],byteorder='big',signed=False)
                            voltage = int.from_bytes(msg.data[5:7],byteorder='big',signed=False)
                            button_state = msg.data[7]
                            self.msg_status.status = status
                            self.msg_status.current = float(current)/1000
                            self.msg_status.voltage = float(voltage)/10
                            self.msg_status.button_state = button_state
                            self.msg_status.header.seq += 1
                            self.msg_status.header.stamp = rospy.Time.now()
                            self.status_pub.publish(self.msg_status)
                rate.sleep()
            except can.CanError:
                rospy.logwarn('Error receive CAN data')
                            
if __name__ == "__main__" :
    try: 
        a = LidDevice()
        a.main()
    except Exception as e:
        pass

                            
                             






