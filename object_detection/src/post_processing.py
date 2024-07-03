#!/usr/bin/env python3

import rospy
from object_detection_msgs.msg import ObjectDetectionInfo, ObjectDetectionInfoArray
from geometry_msgs.msg import Point, PointStamped
import tf2_ros as tf
import tf2_geometry_msgs.tf2_geometry_msgs
import numpy as np


class our_node:
    def __init__(self):
        rospy.init_node('post_processor_node')
        self.detected_objects = []
        self.confidence_threshold = 0.6
        self.num_detections_threshold = 2
        self.object_radius = 1
        self.tf_buffer = tf.Buffer()
        tf_listener = tf.TransformListener(self.tf_buffer)
        post_processor = rospy.Subscriber("/object_detector/detection_info", ObjectDetectionInfoArray ,self.callback, queue_size=2)
        rospy.spin()

    def callback(self, data):

        for info in data.info:
            
            rospy.loginfo("Detected object: %s", info.class_id)
            rospy.loginfo("Position: [%f, %f, %f]", 
            info.position.x, info.position.y, info.position.z)
            rospy.loginfo("Uncertainty: %f", info.confidence)
            object_point = PointStamped()
            object_point.header.frame_id = 'rgb_camera_optical_link'
            object_point.header.stamp = data.header.stamp
            object_point.point.x = info.position.x
            object_point.point.y = info.position.y
            object_point.point.z = info.position.z
            
            converted_point = self.object_detections_to_world_frame(object_point)
            if converted_point:
                rospy.loginfo("Converted point: [%f, %f, %f]", 
                              converted_point.point.x, converted_point.point.y, converted_point.point.z)
                self.detected_objects.append((info.class_id, [info.position.x, info.position.y, info.position.z], info.confidence))
                self.confident_detected_objects = self.filter_real_detections(self, self.detected_objects, self.confidence_threshold, self.num_detections_threshold, self.object_radius)
            
            else:
                rospy.logerr("Failed to convert point for object: %s", info.class_id)

            

    def object_detections_to_world_frame(self, point1:Point):
        """ Function that transforms the detected object location from the camera frame to the world frame.
        
        Args:
            point1 (Point): Detected object location in the camera frame.
        """
        try:
            trans = self.tf_buffer.lookup_transform('map_o3d', 'rgb_camera_optical_link', rospy.Time()) 
            point2 = tf2_geometry_msgs.do_transform_point(point1, trans)

        except Exception as e:
            point2 = None
        return point2


    def filter_real_detections(self, detected_objects:list, confidence_threshold:float, num_detections_threshold:int, object_radius:float):
        """ Function that filters out real object detections and outputs thea averge location of the detected objec
        
        Args:
            detected_objects (list): List of tuples containing the detected object name, object location (x, y, z), and object confidence score.
            confidence_threshold (float): Threshold for filtering out low confidence detections.
            num_detections_threshold (int): Minimum number of detections to consider an object as real.
            object_radius (float): Radius to consider for merging nearby detections.
        
        Returns:
            list: List of tuples containing the real detected object name and its average location (x, y, z).
        """

        # Filter out low confidence detections
        confident_detected_objects = [obj for obj in detected_objects if obj[2] > confidence_threshold]
        if not confident_detected_objects:
            return []
        
        same_detections = 0
        real_detection_indices = []
        i = 0
        
        # set base object location (first detection)
        base_object_location = confident_detected_objects[i][1]

        # Identify the real detections and add their indices to a list
        # Calculate the sequence length (# of sequential detections of the same object around the same location)
        for j in range(len(confident_detected_objects)):

            # Calculate the distance between the base object (detected first) and the current object
            base_object_distance = np.linalg.norm(np.array(base_object_location) - np.array(confident_detected_objects[j][1]))

            if confident_detected_objects[i][2] == confident_detected_objects[j][2] and base_object_distance < object_radius:
                same_detections += 1

            else:
                if same_detections >= num_detections_threshold:
                    real_detection_indices.extend(range(i, j))

                # update variables
                i = j
                same_detections = 0
                
                # update base object location
                base_object_location = confident_detected_objects[i][1]

        return confident_detected_objects[real_detection_indices]


# def publish_low_confidence_detections(detected_objects:list, confidence_threshold:float):
            
    
#     # Create a publisher if not provided
#     if publisher is None:
#         publisher = rospy.Publisher('/low_confidence_objects', Point, queue_size=10)

if __name__=='__main__':
    node = our_node()