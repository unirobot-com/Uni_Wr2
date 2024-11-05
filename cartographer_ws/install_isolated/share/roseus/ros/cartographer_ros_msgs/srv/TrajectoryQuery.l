;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::TrajectoryQuery)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'TrajectoryQuery (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERY")
  (make-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERY"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERYREQUEST")
  (make-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERYREQUEST"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERYRESPONSE")
  (make-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYQUERYRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cartographer_ros_msgs::TrajectoryQueryRequest
  :super ros::object
  :slots (_trajectory_id ))

(defmethod cartographer_ros_msgs::TrajectoryQueryRequest
  (:init
   (&key
    ((:trajectory_id __trajectory_id) 0)
    )
   (send-super :init)
   (setq _trajectory_id (round __trajectory_id))
   self)
  (:trajectory_id
   (&optional __trajectory_id)
   (if __trajectory_id (setq _trajectory_id __trajectory_id)) _trajectory_id)
  (:serialization-length
   ()
   (+
    ;; int32 _trajectory_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _trajectory_id
       (write-long _trajectory_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _trajectory_id
     (setq _trajectory_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cartographer_ros_msgs::TrajectoryQueryResponse
  :super ros::object
  :slots (_status _trajectory ))

(defmethod cartographer_ros_msgs::TrajectoryQueryResponse
  (:init
   (&key
    ((:status __status) (instance cartographer_ros_msgs::StatusResponse :init))
    ((:trajectory __trajectory) ())
    )
   (send-super :init)
   (setq _status __status)
   (setq _trajectory __trajectory)
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; cartographer_ros_msgs/StatusResponse _status
    (send _status :serialization-length)
    ;; geometry_msgs/PoseStamped[] _trajectory
    (apply #'+ (send-all _trajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartographer_ros_msgs/StatusResponse _status
       (send _status :serialize s)
     ;; geometry_msgs/PoseStamped[] _trajectory
     (write-long (length _trajectory) s)
     (dolist (elem _trajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartographer_ros_msgs/StatusResponse _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; geometry_msgs/PoseStamped[] _trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass cartographer_ros_msgs::TrajectoryQuery
  :super ros::object
  :slots ())

(setf (get cartographer_ros_msgs::TrajectoryQuery :md5sum-) "a2f9ca51998db8f538b44b11375e5916")
(setf (get cartographer_ros_msgs::TrajectoryQuery :datatype-) "cartographer_ros_msgs/TrajectoryQuery")
(setf (get cartographer_ros_msgs::TrajectoryQuery :request) cartographer_ros_msgs::TrajectoryQueryRequest)
(setf (get cartographer_ros_msgs::TrajectoryQuery :response) cartographer_ros_msgs::TrajectoryQueryResponse)

(defmethod cartographer_ros_msgs::TrajectoryQueryRequest
  (:response () (instance cartographer_ros_msgs::TrajectoryQueryResponse :init)))

(setf (get cartographer_ros_msgs::TrajectoryQueryRequest :md5sum-) "a2f9ca51998db8f538b44b11375e5916")
(setf (get cartographer_ros_msgs::TrajectoryQueryRequest :datatype-) "cartographer_ros_msgs/TrajectoryQueryRequest")
(setf (get cartographer_ros_msgs::TrajectoryQueryRequest :definition-)
      "# Copyright 2019 The Cartographer Authors
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

int32 trajectory_id
---
cartographer_ros_msgs/StatusResponse status
geometry_msgs/PoseStamped[] trajectory


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

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
")

(setf (get cartographer_ros_msgs::TrajectoryQueryResponse :md5sum-) "a2f9ca51998db8f538b44b11375e5916")
(setf (get cartographer_ros_msgs::TrajectoryQueryResponse :datatype-) "cartographer_ros_msgs/TrajectoryQueryResponse")
(setf (get cartographer_ros_msgs::TrajectoryQueryResponse :definition-)
      "# Copyright 2019 The Cartographer Authors
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

int32 trajectory_id
---
cartographer_ros_msgs/StatusResponse status
geometry_msgs/PoseStamped[] trajectory


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

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
")



(provide :cartographer_ros_msgs/TrajectoryQuery "a2f9ca51998db8f538b44b11375e5916")


