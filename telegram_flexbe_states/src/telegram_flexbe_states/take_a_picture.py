#!/usr/bin/python3

import rospy
import time
import rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TakePicture(EventState):
    def __init__(self, camera_topic, image_path, blocking=True, clear=False):
        super(TakePicture, self).__init__(outcomes=['complete'],
                                          input_keys=['pose_name'])
        self._topic = camera_topic
        self._image_path = image_path
        self._blocking = blocking
        self._clear = clear
        self._connected = False
        self._image_data = None
        self.bridge = CvBridge()
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topic, self.name))

    def on_enter(self, userdata):
        time.sleep(1)
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        if self._connected and self._clear and self._sub.has_msg(self._topic):
            self._sub.remove_last_msg(self._topic)

    def execute(self, userdata):
        if self._sub.has_msg(self._topic) or not self._blocking:
            self._image_data = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self._image_data, "bgr8")
            except Exception as e:
                rospy.logerr(e)
            image_filename = f"{self._image_path}{userdata.pose_name}.jpg"
            cv2.imwrite(image_filename, cv_image, [cv2.IMWRITE_PNG_COMPRESSION, 9])
            rospy.loginfo(f"Saved image: {image_filename}")
            return 'complete'

    def _connect(self):
        global_topic = self._topic
        if global_topic[0] != '/':
            global_topic = rospy.get_namespace() + global_topic
        msg_type, msg_topic, _ = rostopic.get_topic_class(global_topic)
        if msg_topic == global_topic:
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            return True
        return False