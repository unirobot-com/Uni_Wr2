;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::StartTrajectory)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'StartTrajectory (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORY")
  (make-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORY"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORYREQUEST")
  (make-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORYREQUEST"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORYRESPONSE")
  (make-package "CARTOGRAPHER_ROS_MSGS::STARTTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass cartographer_ros_msgs::StartTrajectoryRequest
  :super ros::object
  :slots (_configuration_directory _configuration_basename _use_initial_pose _initial_pose _relative_to_trajectory_id ))

(defmethod cartographer_ros_msgs::StartTrajectoryRequest
  (:init
   (&key
    ((:configuration_directory __configuration_directory) "")
    ((:configuration_basename __configuration_basename) "")
    ((:use_initial_pose __use_initial_pose) nil)
    ((:initial_pose __initial_pose) (instance geometry_msgs::Pose :init))
    ((:relative_to_trajectory_id __relative_to_trajectory_id) 0)
    )
   (send-super :init)
   (setq _configuration_directory (string __configuration_directory))
   (setq _configuration_basename (string __configuration_basename))
   (setq _use_initial_pose __use_initial_pose)
   (setq _initial_pose __initial_pose)
   (setq _relative_to_trajectory_id (round __relative_to_trajectory_id))
   self)
  (:configuration_directory
   (&optional __configuration_directory)
   (if __configuration_directory (setq _configuration_directory __configuration_directory)) _configuration_directory)
  (:configuration_basename
   (&optional __configuration_basename)
   (if __configuration_basename (setq _configuration_basename __configuration_basename)) _configuration_basename)
  (:use_initial_pose
   (&optional (__use_initial_pose :null))
   (if (not (eq __use_initial_pose :null)) (setq _use_initial_pose __use_initial_pose)) _use_initial_pose)
  (:initial_pose
   (&rest __initial_pose)
   (if (keywordp (car __initial_pose))
       (send* _initial_pose __initial_pose)
     (progn
       (if __initial_pose (setq _initial_pose (car __initial_pose)))
       _initial_pose)))
  (:relative_to_trajectory_id
   (&optional __relative_to_trajectory_id)
   (if __relative_to_trajectory_id (setq _relative_to_trajectory_id __relative_to_trajectory_id)) _relative_to_trajectory_id)
  (:serialization-length
   ()
   (+
    ;; string _configuration_directory
    4 (length _configuration_directory)
    ;; string _configuration_basename
    4 (length _configuration_basename)
    ;; bool _use_initial_pose
    1
    ;; geometry_msgs/Pose _initial_pose
    (send _initial_pose :serialization-length)
    ;; int32 _relative_to_trajectory_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _configuration_directory
       (write-long (length _configuration_directory) s) (princ _configuration_directory s)
     ;; string _configuration_basename
       (write-long (length _configuration_basename) s) (princ _configuration_basename s)
     ;; bool _use_initial_pose
       (if _use_initial_pose (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Pose _initial_pose
       (send _initial_pose :serialize s)
     ;; int32 _relative_to_trajectory_id
       (write-long _relative_to_trajectory_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _configuration_directory
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _configuration_directory (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _configuration_basename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _configuration_basename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _use_initial_pose
     (setq _use_initial_pose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Pose _initial_pose
     (send _initial_pose :deserialize buf ptr-) (incf ptr- (send _initial_pose :serialization-length))
   ;; int32 _relative_to_trajectory_id
     (setq _relative_to_trajectory_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cartographer_ros_msgs::StartTrajectoryResponse
  :super ros::object
  :slots (_status _trajectory_id ))

(defmethod cartographer_ros_msgs::StartTrajectoryResponse
  (:init
   (&key
    ((:status __status) (instance cartographer_ros_msgs::StatusResponse :init))
    ((:trajectory_id __trajectory_id) 0)
    )
   (send-super :init)
   (setq _status __status)
   (setq _trajectory_id (round __trajectory_id))
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:trajectory_id
   (&optional __trajectory_id)
   (if __trajectory_id (setq _trajectory_id __trajectory_id)) _trajectory_id)
  (:serialization-length
   ()
   (+
    ;; cartographer_ros_msgs/StatusResponse _status
    (send _status :serialization-length)
    ;; int32 _trajectory_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartographer_ros_msgs/StatusResponse _status
       (send _status :serialize s)
     ;; int32 _trajectory_id
       (write-long _trajectory_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartographer_ros_msgs/StatusResponse _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; int32 _trajectory_id
     (setq _trajectory_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cartographer_ros_msgs::StartTrajectory
  :super ros::object
  :slots ())

(setf (get cartographer_ros_msgs::StartTrajectory :md5sum-) "dcc000df748d283ba7bf678a47ffa491")
(setf (get cartographer_ros_msgs::StartTrajectory :datatype-) "cartographer_ros_msgs/StartTrajectory")
(setf (get cartographer_ros_msgs::StartTrajectory :request) cartographer_ros_msgs::StartTrajectoryRequest)
(setf (get cartographer_ros_msgs::StartTrajectory :response) cartographer_ros_msgs::StartTrajectoryResponse)

(defmethod cartographer_ros_msgs::StartTrajectoryRequest
  (:response () (instance cartographer_ros_msgs::StartTrajectoryResponse :init)))

(setf (get cartographer_ros_msgs::StartTrajectoryRequest :md5sum-) "dcc000df748d283ba7bf678a47ffa491")
(setf (get cartographer_ros_msgs::StartTrajectoryRequest :datatype-) "cartographer_ros_msgs/StartTrajectoryRequest")
(setf (get cartographer_ros_msgs::StartTrajectoryRequest :definition-)
      "# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

string configuration_directory
string configuration_basename
bool use_initial_pose
geometry_msgs/Pose initial_pose
int32 relative_to_trajectory_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
cartographer_ros_msgs/StatusResponse status
int32 trajectory_id


================================================================================
MSG: cartographer_ros_msgs/StatusResponse
# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# A common message type to indicate the outcome of a service call.
uint8 code
string message
")

(setf (get cartographer_ros_msgs::StartTrajectoryResponse :md5sum-) "dcc000df748d283ba7bf678a47ffa491")
(setf (get cartographer_ros_msgs::StartTrajectoryResponse :datatype-) "cartographer_ros_msgs/StartTrajectoryResponse")
(setf (get cartographer_ros_msgs::StartTrajectoryResponse :definition-)
      "# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

string configuration_directory
string configuration_basename
bool use_initial_pose
geometry_msgs/Pose initial_pose
int32 relative_to_trajectory_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
cartographer_ros_msgs/StatusResponse status
int32 trajectory_id


================================================================================
MSG: cartographer_ros_msgs/StatusResponse
# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# A common message type to indicate the outcome of a service call.
uint8 code
string message
")



(provide :cartographer_ros_msgs/StartTrajectory "dcc000df748d283ba7bf678a47ffa491")


