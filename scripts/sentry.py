#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: 
Version:
"""
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kobuki_msgs.msg import Sound
import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        self.sound_publish = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
        self.prev = None
        self.average = 0
        self.threshold = 15  
        self.intruder = False  


        #rospy.spin()
        

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """

        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        
        current = depth[:, depth.shape[1] / 2]
        #current = current[~np.isnan(current)]

        
        if self.prev is None:
            self.prev = current
        
        
        d = np.abs(current - self.prev)
        
        d = d[~np.isnan(d)]
        d = np.sum(d)
        
        
        
        self.average = self.average * .5 + d * (1 - .5)
        
        self.prev = current
        if(self.average > self.threshold):
            
            self.intruder = True
        
    def run(self):
        
        while(True and not rospy.is_shutdown()):
            beep = Sound()
            
            if(self.intruder):
                print self.average
                beep.value = 0
                self.sound_publish.publish(beep)
                self.intruder = False

if __name__ == "__main__":
    x = SentryNode()
    x.run()
