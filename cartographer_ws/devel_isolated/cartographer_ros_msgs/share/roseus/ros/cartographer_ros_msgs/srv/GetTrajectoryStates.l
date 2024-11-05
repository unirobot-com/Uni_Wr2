;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::GetTrajectoryStates)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'GetTrajectoryStates (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATES")
  (make-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATES"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATESREQUEST")
  (make-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATESREQUEST"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATESRESPONSE")
  (make-package "CARTOGRAPHER_ROS_MSGS::GETTRAJECTORYSTATESRESPONSE"))

(in-package "ROS")





(defclass cartographer_ros_msgs::GetTrajectoryStatesRequest
  :super ros::object
  :slots ())

(defmethod cartographer_ros_msgs::GetTrajectoryStatesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cartographer_ros_msgs::GetTrajectoryStatesResponse
  :super ros::object
  :slots (_status _trajectory_states ))

(defmethod cartographer_ros_msgs::GetTrajectoryStatesResponse
  (:init
   (&key
    ((:status __status) (instance cartographer_ros_msgs::StatusResponse :init))
    ((:trajectory_states __trajectory_states) (instance cartographer_ros_msgs::TrajectoryStates :init))
    )
   (send-super :init)
   (setq _status __status)
   (setq _trajectory_states __trajectory_states)
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:trajectory_states
   (&rest __trajectory_states)
   (if (keywordp (car __trajectory_states))
       (send* _trajectory_states __trajectory_states)
     (progn
       (if __trajectory_states (setq _trajectory_states (car __trajectory_states)))
       _trajectory_states)))
  (:serialization-length
   ()
   (+
    ;; cartographer_ros_msgs/StatusResponse _status
    (send _status :serialization-length)
    ;; cartographer_ros_msgs/TrajectoryStates _trajectory_states
    (send _trajectory_states :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartographer_ros_msgs/StatusResponse _status
       (send _status :serialize s)
     ;; cartographer_ros_msgs/TrajectoryStates _trajectory_states
       (send _trajectory_states :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartographer_ros_msgs/StatusResponse _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; cartographer_ros_msgs/TrajectoryStates _trajectory_states
     (send _trajectory_states :deserialize buf ptr-) (incf ptr- (send _trajectory_states :serialization-length))
   ;;
   self)
  )

(defclass cartographer_ros_msgs::GetTrajectoryStates
  :super ros::object
  :slots ())

(setf (get cartographer_ros_msgs::GetTrajectoryStates :md5sum-) "b9e3b373f17df088ee6dcd817b79dff0")
(setf (get cartographer_ros_msgs::GetTrajectoryStates :datatype-) "cartographer_ros_msgs/GetTrajectoryStates")
(setf (get cartographer_ros_msgs::GetTrajectoryStates :request) cartographer_ros_msgs::GetTrajectoryStatesRequest)
(setf (get cartographer_ros_msgs::GetTrajectoryStates :response) cartographer_ros_msgs::GetTrajectoryStatesResponse)

(defmethod cartographer_ros_msgs::GetTrajectoryStatesRequest
  (:response () (instance cartographer_ros_msgs::GetTrajectoryStatesResponse :init)))

(setf (get cartographer_ros_msgs::GetTrajectoryStatesRequest :md5sum-) "b9e3b373f17df088ee6dcd817b79dff0")
(setf (get cartographer_ros_msgs::GetTrajectoryStatesRequest :datatype-) "cartographer_ros_msgs/GetTrajectoryStatesRequest")
(setf (get cartographer_ros_msgs::GetTrajectoryStatesRequest :definition-)
      "# Copyright 2018 The Cartographer Authors
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


---
cartographer_ros_msgs/StatusResponse status
cartographer_ros_msgs/TrajectoryStates trajectory_states


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
MSG: cartographer_ros_msgs/TrajectoryStates
# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

uint8 ACTIVE = 0
uint8 FINISHED = 1
uint8 FROZEN = 2
uint8 DELETED = 3

std_msgs/Header header
int32[] trajectory_id
uint8[] trajectory_state

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
")

(setf (get cartographer_ros_msgs::GetTrajectoryStatesResponse :md5sum-) "b9e3b373f17df088ee6dcd817b79dff0")
(setf (get cartographer_ros_msgs::GetTrajectoryStatesResponse :datatype-) "cartographer_ros_msgs/GetTrajectoryStatesResponse")
(setf (get cartographer_ros_msgs::GetTrajectoryStatesResponse :definition-)
      "# Copyright 2018 The Cartographer Authors
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


---
cartographer_ros_msgs/StatusResponse status
cartographer_ros_msgs/TrajectoryStates trajectory_states


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
MSG: cartographer_ros_msgs/TrajectoryStates
# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

uint8 ACTIVE = 0
uint8 FINISHED = 1
uint8 FROZEN = 2
uint8 DELETED = 3

std_msgs/Header header
int32[] trajectory_id
uint8[] trajectory_state

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
")



(provide :cartographer_ros_msgs/GetTrajectoryStates "b9e3b373f17df088ee6dcd817b79dff0")


