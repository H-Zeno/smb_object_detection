#!/usr/bin/env python3

import rospy
from object_detection_msgs.msg import ObjectDetectionInfo, ObjectDetectionInfoArray
from geometry_msgs.msg import Point, PointStamped
import tf2_ros as tf
import tf2_geometry_msgs.tf2_geometry_msgs
import numpy as np
import csv
from std_msgs.msg import Bool
from datetime import datetime


class our_node:
    def __init__(self):
        rospy.init_node('post_processor_node')
        self.detected_objects = []

        # Create a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # backpack, umbrella, bottle, sign, clock
        # self.classes = ["backpack", "umbrella", "bottle", "stop sign", "clock"]
        self.classes = ["backpack", "person", "bottle", "laptop"]

        self.rads = [2,2,2,2]
        # File path with timestamp
        self.file_path_detected_objects = '/workspaces/rss_workspace/src/object_detection/object_detection/src/detected_objects.csv'
        
        #f'~/detected_objects_{timestamp}.csv'

        self.confidence_threshold = 0.65
        self.num_detections_threshold = 1
        self.object_radius = 1
        self.tf_buffer = tf.Buffer()
        tf_listener = tf.TransformListener(self.tf_buffer)
        post_processor = rospy.Subscriber("/object_detector/detection_info", ObjectDetectionInfoArray,self.callback, queue_size=2)
        rospy.Subscriber("/sensor_coverage_planner/exploration_finish", Bool, self.filter_real_detections, queue_size=10)
        rospy.spin()


    def callback(self, data):

        for info in data.info:
            print(' ///// Callback started /////')
            # rospy.loginfo("Detected object: %s", info.class_id)
            # rospy.loginfo("Position: [%f, %f, %f]", 
            # info.position.x, info.position.y, info
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

            else:
                rospy.logerr("Failed to convert point for object: %s", info.class_id)


    def filter_real_detections(self, data):
        """ Function that filters out real object detections and outputs thea averge location of the detected objec
        
        Args:
            detected_objects (list): List of tuples containing the detected object name, object location (x, y, z), and object confidence score.
            confidence_threshold (float): Threshold for filtering out low confidence detections.
            num_detections_threshold (int): Minimum number of detections to consider an object as real.
            object_radius (float): Radius to consider for merging nearby detections.
        
        Returns:
            list: List of tuples containing the real detected object name and its average location (x, y, z).
        """
        print(data)
        if not data.data:
            return

        detected_objects = self.detected_objects
        confidence_threshold = self.confidence_threshold
        num_detections_threshold = self.num_detections_threshold
        object_radius = self.object_radius
        
        # Filter out low confidence detections
        confident_detected_objects = [obj for obj in detected_objects if obj[2] > confidence_threshold]
        if not confident_detected_objects:
            return

        print('Confident detected objects:', confident_detected_objects)
        

        # print('Objects with a high confidence score:', confident_detected_objects
        clusters = {name: [] for name in self.classes}
        print(f'{clusters=}')
        print(confident_detected_objects)
        for i in range(len(confident_detected_objects)):
            print(i)
            curr_name = confident_detected_objects[i][0]
            curr_pos = np.array(confident_detected_objects[i][1])
            if clusters[curr_name] == []:
                clusters[curr_name].append((curr_pos, 1))
                continue
            for cluster_center, count in clusters[curr_name]:
                if np.linalg.norm(cluster_center - curr_pos) < self.rads[self.classes.index(curr_name)]:
                    
                    clusters[curr_name].remove((cluster_center, count))
                    clusters[curr_name].append(((count/(count+1))*cluster_center + (1/(count+1))*curr_pos, count+1))
                    break
                else: 
                    # new
                    clusters[curr_name].append((curr_pos, 1))
                    break

        print(clusters)
        # Write the confidently detected objects to a CSV file
        file_path = self.file_path_detected_objects
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Object Class', 'Position (x,y,z)'])
            for name, centroids in clusters.items(): 
                for centroid, count in centroids:
                    writer.writerow([name, f'({centroid[0]},{centroid[1]},{centroid[2]})'])

        print(f'Detected objects have been written to {file_path}')



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





# def publish_low_confidence_detections(detected_objects:list, confidence_threshold:float):
            
    
#     # Create a publisher if not provided
#     if publisher is None:
#         publisher = rospy.Publisher('/low_confidence_objects', Point, queue_size=10)

if __name__=='__main__':
    node = our_node()
    
