#!/usr/bin/env python
import numpy as np
import rospy
import tf
import tf.transformations as tr
from pcl_msgs.msg import ModelCoefficients
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from math import pi

INIT_OFFSET = [0,0,0]
INIT_ROLL = 0.0
PARENT_FRAME = "camera_depth_optical_frame"
CHILD_FRAME = "table_frame"
PLANE_OFFSET = 0.01


class TableSegmenter( object ):
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.trans = INIT_OFFSET
        self.rot = tr.quaternion_from_euler(0, 0, INIT_ROLL, 'szyx')
        # create a timer that sends the current estimated transform repeatedly:
        self.tf_timer = rospy.Timer(rospy.Duration(0.01), self.tf_pub_cb)

        # create a subscriber that subscribes to the table model:
        self.model_sub = rospy.Subscriber("/normal_segmentation/segmentation/model", ModelCoefficients,
                                          self.modelcb)
        self.model_coeffs = None
        # now create a service provider, that, when called, will update the
        # transform data based off of the model
        self.model_srv = rospy.Service("update_table_model", Empty, self.update_model)
        
        return

    def tf_pub_cb(self, te):
        self.br.sendTransform(self.trans, self.rot, rospy.Time.now(), CHILD_FRAME, PARENT_FRAME)
        return

    def modelcb(self, model):
        self.model_coeffs = model.values
        return

    def update_model(self, srv):
        if self.model_coeffs is None:
            return EmptyResponse()
        # we need to convert the table model into useful transform data. the
        # plane coefficients are [a,b,c,d] where the equation of the plane is
        # given by `ax + by + cz + d = 0`
        # from the plane normal, we can calculate the orientation:
        model = np.array(self.model_coeffs)
        # we want to make sure this normal is pointed "down"
        if np.dot(model[0:3], [0,1,0]) < 0:
            model = -1.0*model
        normal = np.array(model[0:3])
        # now we can build a rotation matrix from this normal, and assuming the
        # table frame's x-axis is aligned with the sensor's x axis:
        z = np.cross([1,0,0], normal)
        R = np.vstack(([1,0,0], normal, z)).T
        self.rot = tr.quaternion_from_matrix(np.vstack((np.hstack((R,np.zeros((3,1)))),[0,0,0,1])))
        # now if we assume that the center of the plane has the following
        # coordinates (0, y0, 1), we can solve for the height of the plane (y0)
        # http://mathworld.wolfram.com/Plane.html
        a,b,c,d = model
        x = 0
        z = 1
        y0 = (-d - c*z - a*x)/b
        self.trans = [0, y0 - PLANE_OFFSET, 1]
        return EmptyResponse()


def main():
    rospy.init_node("table_cutoff_settings", log_level=rospy.INFO)
    try:
        segmenter = TableSegmenter()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()



if __name__ == '__main__':
    main()
