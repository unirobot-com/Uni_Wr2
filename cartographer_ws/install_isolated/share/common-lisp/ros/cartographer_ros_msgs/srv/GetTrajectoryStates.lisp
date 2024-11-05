; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-srv)


;//! \htmlinclude GetTrajectoryStates-request.msg.html

(cl:defclass <GetTrajectoryStates-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetTrajectoryStates-request (<GetTrajectoryStates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTrajectoryStates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTrajectoryStates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<GetTrajectoryStates-request> is deprecated: use cartographer_ros_msgs-srv:GetTrajectoryStates-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTrajectoryStates-request>) ostream)
  "Serializes a message object of type '<GetTrajectoryStates-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTrajectoryStates-request>) istream)
  "Deserializes a message object of type '<GetTrajectoryStates-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTrajectoryStates-request>)))
  "Returns string type for a service object of type '<GetTrajectoryStates-request>"
  "cartographer_ros_msgs/GetTrajectoryStatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryStates-request)))
  "Returns string type for a service object of type 'GetTrajectoryStates-request"
  "cartographer_ros_msgs/GetTrajectoryStatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTrajectoryStates-request>)))
  "Returns md5sum for a message object of type '<GetTrajectoryStates-request>"
  "b9e3b373f17df088ee6dcd817b79dff0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTrajectoryStates-request)))
  "Returns md5sum for a message object of type 'GetTrajectoryStates-request"
  "b9e3b373f17df088ee6dcd817b79dff0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTrajectoryStates-request>)))
  "Returns full string definition for message of type '<GetTrajectoryStates-request>"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTrajectoryStates-request)))
  "Returns full string definition for message of type 'GetTrajectoryStates-request"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTrajectoryStates-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTrajectoryStates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTrajectoryStates-request
))
;//! \htmlinclude GetTrajectoryStates-response.msg.html

(cl:defclass <GetTrajectoryStates-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cartographer_ros_msgs-msg:StatusResponse
    :initform (cl:make-instance 'cartographer_ros_msgs-msg:StatusResponse))
   (trajectory_states
    :reader trajectory_states
    :initarg :trajectory_states
    :type cartographer_ros_msgs-msg:TrajectoryStates
    :initform (cl:make-instance 'cartographer_ros_msgs-msg:TrajectoryStates)))
)

(cl:defclass GetTrajectoryStates-response (<GetTrajectoryStates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTrajectoryStates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTrajectoryStates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<GetTrajectoryStates-response> is deprecated: use cartographer_ros_msgs-srv:GetTrajectoryStates-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GetTrajectoryStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:status-val is deprecated.  Use cartographer_ros_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'trajectory_states-val :lambda-list '(m))
(cl:defmethod trajectory_states-val ((m <GetTrajectoryStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:trajectory_states-val is deprecated.  Use cartographer_ros_msgs-srv:trajectory_states instead.")
  (trajectory_states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTrajectoryStates-response>) ostream)
  "Serializes a message object of type '<GetTrajectoryStates-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory_states) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTrajectoryStates-response>) istream)
  "Deserializes a message object of type '<GetTrajectoryStates-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory_states) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTrajectoryStates-response>)))
  "Returns string type for a service object of type '<GetTrajectoryStates-response>"
  "cartographer_ros_msgs/GetTrajectoryStatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryStates-response)))
  "Returns string type for a service object of type 'GetTrajectoryStates-response"
  "cartographer_ros_msgs/GetTrajectoryStatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTrajectoryStates-response>)))
  "Returns md5sum for a message object of type '<GetTrajectoryStates-response>"
  "b9e3b373f17df088ee6dcd817b79dff0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTrajectoryStates-response)))
  "Returns md5sum for a message object of type 'GetTrajectoryStates-response"
  "b9e3b373f17df088ee6dcd817b79dff0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTrajectoryStates-response>)))
  "Returns full string definition for message of type '<GetTrajectoryStates-response>"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%cartographer_ros_msgs/TrajectoryStates trajectory_states~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: cartographer_ros_msgs/TrajectoryStates~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 ACTIVE = 0~%uint8 FINISHED = 1~%uint8 FROZEN = 2~%uint8 DELETED = 3~%~%std_msgs/Header header~%int32[] trajectory_id~%uint8[] trajectory_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTrajectoryStates-response)))
  "Returns full string definition for message of type 'GetTrajectoryStates-response"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%cartographer_ros_msgs/TrajectoryStates trajectory_states~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: cartographer_ros_msgs/TrajectoryStates~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 ACTIVE = 0~%uint8 FINISHED = 1~%uint8 FROZEN = 2~%uint8 DELETED = 3~%~%std_msgs/Header header~%int32[] trajectory_id~%uint8[] trajectory_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTrajectoryStates-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory_states))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTrajectoryStates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTrajectoryStates-response
    (cl:cons ':status (status msg))
    (cl:cons ':trajectory_states (trajectory_states msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTrajectoryStates)))
  'GetTrajectoryStates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTrajectoryStates)))
  'GetTrajectoryStates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTrajectoryStates)))
  "Returns string type for a service object of type '<GetTrajectoryStates>"
  "cartographer_ros_msgs/GetTrajectoryStates")