#!/usr/bin/env python3

import rospy
import rostopic
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Topic:
    def __init__(self, config):
        self.name = list(config.keys())[0]
        self.topic_path = config['name']
        self.hz = float(config['hz'])
        self.meas_hz = 0
        self.last_time = rospy.Time.now()

        self.have_sub = False
        self.create_sub()

    def create_sub(self):
        if self.have_sub:
            return
        #Find the topic message type
        topic_class,_,_ = rostopic.get_topic_class(self.topic_path, blocking=False)
        if topic_class is None:
            return

        #Subscribe to the topic
        self.sub = rospy.Subscriber(self.topic_path, topic_class, self.cb)
        self.have_sub = True

    def cb(self, msg):
        dt = rospy.Time.now() - self.last_time
        self.last_time = rospy.Time.now()
        self.meas_hz = 1./dt.to_sec()

    def is_published(self):
        self.meas_hz = min(self.meas_hz, 1/(rospy.Time.now() - self.last_time).to_sec())
        if not self.have_sub:
            self.create_sub()
            return False
        else:
            return self.meas_hz > self.hz

    def __str__(self):
        return f"{self.name}: {self.topic_path}"
    __repr__ = __str__

class TopicMonitor:
    def __init__(self):
        config_path = rospy.get_param("~config_path", "")
        if len(config_path) == 0:
            rospy.logfatal("No config path specified")
            quit()
        config = yaml.safe_load(open(config_path, 'r'))    

        self.topic_monitors_ = []
        for topic in config:
            self.topic_monitors_.append(Topic(topic))

        self.diagnostics_pub_ = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self.report_data_timer_ = rospy.Timer(rospy.Duration(1), self.report_data_cb)

    def report_data_cb(self, timer):
        rospy.loginfo("===================================================")
        diag_arr_msg = DiagnosticArray()
        diag_arr_msg.header.stamp = rospy.Time.now()

        for topic in self.topic_monitors_:
            diag_msg = DiagnosticStatus()
            diag_msg.name = str(topic)
            diag_msg.hardware_id = "topic_monitor"
            if topic.is_published():
                diag_msg.message = "OK"
                diag_msg.level = DiagnosticStatus.OK
                rospy.loginfo("\033[92m(x) "+str(topic)+"\033[0m")
            else:
                diag_msg.message = f"Actual: {topic.hz} Hz, Desired: {topic.meas_hz} Hz"
                diag_msg.level = DiagnosticStatus.WARN
                rospy.loginfo("\033[31;1m( ) "+str(topic)+"\033[0m")
            diag_arr_msg.status.append(diag_msg)

        self.diagnostics_pub_.publish(diag_arr_msg)

if __name__ == '__main__':
    rospy.init_node("topic_monitor")
    tm = TopicMonitor()
    rospy.spin()
