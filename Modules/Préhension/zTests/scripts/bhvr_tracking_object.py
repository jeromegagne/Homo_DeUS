#! /usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
from head_control.scripts.headController import headController
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import HomoDeUS_common_py.HomoDeUS_common_py as common  

BASE_MVMT_LIMIT = ""
MAX_REACHEABLE_RIGHT = -1.24
MAX_REACHEABLE_LEFT = 1.24
MAX_REACHEABLE_HEIGHT = 0.79
MAX_REACHEABLE_DOWN = -0.98

PIXEL_TO_RAD_X = 0.00158
PIXEL_TO_RAD_Y = 0.00146


class ObjectTracking:
    def __init__(self, mode):
        self.desired_object = None
        rospy.Subscriber('bounding_boxes',BoundingBoxes, self._head_cb,queue_size= 1)
        #rospy.Subscriber('/proc_output_face_positions', FacePositions, self._head_callback, queue_size=5)
        rospy.Subscriber('/desired_object', String, self._desired_obj_cb, queue_size=5)
        rospy.Subscriber('/bhvr_tracking_object_interrupt', Empty, self._interrupt_cb, queue_size=5)

        if mode == "remote":
            camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        else:
            camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
    
        self.img_center_x = camera_info.height // 2
        self.img_center_y = camera_info.width // 2

        self.perception_pub = rospy.Publisher('bhvr_output_trackingObject_boxes',BoundingBoxes, queue_size=1)
        self.detect_pub =  rospy.Publisher('bhvr_output_detect_object',Bool, queue_size=1)
        #self.pub = rospy.Publisher('head_command', JointTrajectory, queue_size=2)
        self.head_controller = headController(rate=0.05, head_control_topic='head_command')

        self.distance_threshold = 20
        

    def _desired_obj_cb(self, d_object):
        rospy.loginfo("desired object receive: " + str(d_object.data))
        self.desired_object = d_object.data

    def _head_cb(self, boxes):
        self.current_boxes = boxes
        boxes = boxes.bounding_boxes
        # Find the closest face to the image center (main face)
        if self.desired_object is not None:
            desired_obj_position = self._detect_desired_object(boxes)
            if desired_obj_position:
                self.detect_pub.publish(True)
                self._center_desired_object(desired_obj_position)


    def _detect_desired_object(self,boxes):
        for box in boxes:
            if box.Class == self.desired_object:
                object_x_center = box.xmin + abs((box.xmax-box.xmin) // 2)
                object_y_center = box.ymin + abs((box.ymax-box.ymin) // 2)
                return object_x_center, object_y_center
    
    def _center_desired_object(self, obj_position):
        if not self._object_centered(obj_position):
            self.result_sent = False
            commands = self.convert_pixel_to_command(obj_position[0],obj_position[1])
            self.head_controller.goto_position(repeat=False,x=commands[0],y= commands[1],duration=1.)
        elif not self.head_controller.is_base_align():
            self.result_sent = False
            self.head_controller.center_base()
        elif not self.result_sent:
            commands = self.head_controller.get_head_angles() # stay at that position
            self.head_controller.goto_position(repeat=True,x=commands[0],y= commands[1],duration=1.)
            self.perception_pub.publish(self.current_boxes)
            self.result_sent = True


    def _object_centered(self,obj_position):
        if self.distance_threshold > self._distance_from_img_center(obj_position):
            return True
        else:
            return False

    def convert_pixel_to_angle(self,pixel_position_x, pixel_position_y):
        pixel_mvmt_x = -(pixel_position_x - self.img_center_x) # negative since to turn right needs negative commands
        pixel_mvmt_y = -(pixel_position_y - self.img_center_y) # negative since to going down needs negative commands
        angle_mvmt_x = pixel_mvmt_x*PIXEL_TO_RAD_X
        angle_mvmt_y = pixel_mvmt_y*PIXEL_TO_RAD_Y
        return [angle_mvmt_x, angle_mvmt_y]

    def convert_pixel_to_command(self,pixel_position_x,pixel_position_y):
        mvmt_angles = self.convert_pixel_to_angle(pixel_position_x, pixel_position_y)
        command_x = self.head_controller.get_head_angles()[0] + mvmt_angles[0]
        command_y = self.head_controller.get_head_angles()[1] + mvmt_angles[1]
        return self.apply_max_values(command_x, command_y)


    def apply_max_values(self,command_x, command_y):
        if command_x < MAX_REACHEABLE_RIGHT:
            rospy.logwarn("Can not reach the desired x position")
            command_x = MAX_REACHEABLE_RIGHT
        elif command_x > MAX_REACHEABLE_LEFT:
            rospy.logwarn("Can not reach the desired x position")
            command_x = MAX_REACHEABLE_LEFT

        if command_y < MAX_REACHEABLE_DOWN:
            rospy.logwarn("Can not reach the desired y position")
            command_y = MAX_REACHEABLE_DOWN
        elif command_y > MAX_REACHEABLE_HEIGHT:
            rospy.logwarn("Can not reach the desired y position")
            command_y = MAX_REACHEABLE_HEIGHT
        return [command_x,command_y]


    def _distance_from_img_center(self, obj_position):
        answer = math.sqrt((self.img_center_x - obj_position[0])**2 + (self.img_center_y - obj_position[1])**2)
        return answer

    def _interrupt_cb(self, data):
        self.head_controller.stop_repeated_sending()
        self.head_controller.home_cb('nothing')


if __name__ == "__main__":

    try:
        rospy.init_node('ObjectTracking', anonymous=False)
        mode = rospy.get_param('camera_mode', 'simul')
        ObjectTracking = ObjectTracking(mode)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
