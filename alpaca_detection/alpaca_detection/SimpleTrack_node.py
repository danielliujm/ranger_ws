# from pcdet.datasets.kitti.kitti_object_eval_python.kitti_common import iou
from rclpy.node import Node
import rclpy
import ros2_numpy as rnp
import numpy as np
from alpaca_detection.SimpleTrack.mot_3d.mot import MOTModel
from alpaca_detection.SimpleTrack.mot_3d.frame_data import FrameData
from alpaca_detection.SimpleTrack.mot_3d.data_protos import BBox
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
import math

class SimpleTrackNode(Node):
    def __init__ (self, configs=None):
        super().__init__('SimpleTrack_node')
        self.pc_numpy = None
        # provide sensible default configs when none provided
        if not configs:
            configs = {
                'running': {
                    'match_type': 'bipartite',
                    'score_threshold': 0.8,
                    'asso': 'giou',
                    'asso_thres': {'giou': 1.5, 'iou': 0.9},
                    'motion_model': 'kf',
                    'max_age_since_update': 2,
                    'min_hits_to_birth': 20,
                    'covariance': {},
                    # 'asso_thres':{
                    #     'iou': 0.9,                  
                    #     'giou': 1.5 
                    #  }
                },
                'redundancy': {
                    'mode': 'mm',
                    'det_score_threshold': {'giou': 0.1, 'iou': 0.1, 'euler': 0.1},
                    'det_dist_threshold': {'giou': -0.5, 'iou': 0.1, 'euler': 4}
                }
            }
        self.mot_model = MOTModel(configs)
        self.detection_subscriber = self.create_subscription (Detection3DArray, '/detection_results', self.detection_cb, 10)
        self.tracker_marker_publisher = self.create_publisher(MarkerArray, '/SimpleTrack_trackers', 10)
        self.result_publisher = self.create_publisher(Detection3DArray, '/SimpleTrack_results', 10)

        # keep track of marker ids we've published so we can delete ones that disappear
        self._published_tracker_ids = set()
        self.tracker = MOTModel (configs)
    
    def detection_cb (self,msg):
        dets = []
        det_time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        det_types = []
        for det in msg.detections: 
            position = det.bbox.center.position
            size = det.bbox.size
            orientation = 2*math.atan2(det.bbox.center.orientation.z, det.bbox.center.orientation.w)
            score = det.results[0].hypothesis.score
           

            detection = np.array ([position.x, position.y, position.z , orientation, size.x, size.y , size.z, score] )
            dets.append (detection)
            det_types.append ('person')

        aux_info = {'is_key_frame': True}
        frame_data = FrameData (dets = dets, ego = np.eye(4), time_stamp=det_time_stamp, aux_info=aux_info, det_types=det_types)
        tracks = self.mot_model.frame_mot (frame_data)
        # print ("Tracks: ", tracks[0][0])
        # self.get_logger().info (f'Tracks: {tracks[0][2]}')
        self.draw_tracks (tracks)
        self.publish_results (tracks)

    
    def draw_tracks(self, tracks):
        tracker_markers = MarkerArray()
        current_ids = set()
        # Add/Update markers for currently tracked objects
        for track in tracks:
            tid = track[1]
            current_ids.add(tid)
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.ns = "trackers"
            marker.id = tid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = track[0].l
            marker.scale.y = track[0].w
            marker.scale.z = track[0].h
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = track[0].x
            marker.pose.position.y = track[0].y
            marker.pose.position.z = track[0].z
            qz = math.sin(track[0].o / 2)
            qw = math.cos(track[0].o / 2)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            tracker_markers.markers.append(marker)

        # Any previously published markers that are NOT in current_ids should be removed immediately
        removed_ids = self._published_tracker_ids - current_ids
        for rid in removed_ids:
            del_marker = Marker()
            del_marker.header.frame_id = "velodyne"
            del_marker.ns = "trackers"
            del_marker.id = rid
            del_marker.action = Marker.DELETE
            tracker_markers.markers.append(del_marker)

        # publish and update the published ids set
        self.tracker_marker_publisher.publish(tracker_markers)
        self._published_tracker_ids = current_ids
    
    def publish_results (self, tracks):
        detection_msg = Detection3DArray()
        detection_msg.header.frame_id = 'velodyne'
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        for track in tracks:
            bbox = track[0]
            tid = track[1]
            score = bbox.s if bbox.s is not None else 0.0

            det_msg = Detection3D()
            det_msg.header.frame_id = 'velodyne'
            det_msg.header.stamp = self.get_clock().now().to_msg()

            # Fill in bbox
            det_msg.bbox.center.position.x = bbox.x
            det_msg.bbox.center.position.y = bbox.y
            det_msg.bbox.center.position.z = bbox.z
            qz = math.sin(bbox.o / 2)
            qw = math.cos(bbox.o / 2)
            det_msg.bbox.center.orientation.x = 0.0
            det_msg.bbox.center.orientation.y = 0.0
            det_msg.bbox.center.orientation.z = qz
            det_msg.bbox.center.orientation.w = qw
            det_msg.bbox.size.x = bbox.l
            det_msg.bbox.size.y = bbox.w
            det_msg.bbox.size.z = bbox.h

            # Fill in result with track ID and score
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(tid)
            hypothesis.hypothesis.score = float(score)
            det_msg.results.append(hypothesis)

           

            detection_msg.detections.append(det_msg)
        
        self.result_publisher.publish(detection_msg)
        




def main():
    rclpy.init()
    node = SimpleTrackNode(configs={})
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





            


