#!/usr/bin/env python3

import rospy
import rostopic
import yaml

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

        self.print_data_timer_ = rospy.Timer(rospy.Duration(1), self.print_data_cb)

    def print_data_cb(self, timer):
        rospy.loginfo("===================================================")
        for topic in self.topic_monitors_:
            if topic.is_published():
                rospy.loginfo(f"✔️  {topic}")
            else:
                rospy.loginfo(f"❌  {topic}")

if __name__ == '__main__':
    rospy.init_node("topic_monitor")
    tm = TopicMonitor()
    rospy.spin()
