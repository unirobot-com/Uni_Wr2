; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-srv)


;//! \htmlinclude TrajectoryQuery-request.msg.html

(cl:defclass <TrajectoryQuery-request> (roslisp-msg-protocol:ros-message)
  ((trajectory_id
    :reader trajectory_id
    :initarg :trajectory_id
    :type cl:integer
    :initform 0))
)

(cl:defclass TrajectoryQuery-request (<TrajectoryQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<TrajectoryQuery-request> is deprecated: use cartographer_ros_msgs-srv:TrajectoryQuery-request instead.")))

(cl:ensure-generic-function 'trajectory_id-val :lambda-list '(m))
(cl:defmethod trajectory_id-val ((m <TrajectoryQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:trajectory_id-val is deprecated.  Use cartographer_ros_msgs-srv:trajectory_id instead.")
  (trajectory_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryQuery-request>) ostream)
  "Serializes a message object of type '<TrajectoryQuery-request>"
  (cl:let* ((signed (cl:slot-value msg 'trajectory_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryQuery-request>) istream)
  "Deserializes a message object of type '<TrajectoryQuery-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trajectory_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryQuery-request>)))
  "Returns string type for a service object of type '<TrajectoryQuery-request>"
  "cartographer_ros_msgs/TrajectoryQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryQuery-request)))
  "Returns string type for a service object of type 'TrajectoryQuery-request"
  "cartographer_ros_msgs/TrajectoryQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryQuery-request>)))
  "Returns md5sum for a message object of type '<TrajectoryQuery-request>"
  "a2f9ca51998db8f538b44b11375e5916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryQuery-request)))
  "Returns md5sum for a message object of type 'TrajectoryQuery-request"
  "a2f9ca51998db8f538b44b11375e5916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryQuery-request>)))
  "Returns full string definition for message of type '<TrajectoryQuery-request>"
  (cl:format cl:nil "# Copyright 2019 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%int32 trajectory_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryQuery-request)))
  "Returns full string definition for message of type 'TrajectoryQuery-request"
  (cl:format cl:nil "# Copyright 2019 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%int32 trajectory_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryQuery-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryQuery-request
    (cl:cons ':trajectory_id (trajectory_id msg))
))
;//! \htmlinclude TrajectoryQuery-response.msg.html

(cl:defclass <TrajectoryQuery-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cartographer_ros_msgs-msg:StatusResponse
    :initform (cl:make-instance 'cartographer_ros_msgs-msg:StatusResponse))
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass TrajectoryQuery-response (<TrajectoryQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<TrajectoryQuery-response> is deprecated: use cartographer_ros_msgs-srv:TrajectoryQuery-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <TrajectoryQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:status-val is deprecated.  Use cartographer_ros_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <TrajectoryQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:trajectory-val is deprecated.  Use cartographer_ros_msgs-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryQuery-response>) ostream)
  "Serializes a message object of type '<TrajectoryQuery-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryQuery-response>) istream)
  "Deserializes a message object of type '<TrajectoryQuery-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryQuery-response>)))
  "Returns string type for a service object of type '<TrajectoryQuery-response>"
  "cartographer_ros_msgs/TrajectoryQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryQuery-response)))
  "Returns string type for a service object of type 'TrajectoryQuery-response"
  "cartographer_ros_msgs/TrajectoryQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryQuery-response>)))
  "Returns md5sum for a message object of type '<TrajectoryQuery-response>"
  "a2f9ca51998db8f538b44b11375e5916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryQuery-response)))
  "Returns md5sum for a message object of type 'TrajectoryQuery-response"
  "a2f9ca51998db8f538b44b11375e5916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryQuery-response>)))
  "Returns full string definition for message of type '<TrajectoryQuery-response>"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%geometry_msgs/PoseStamped[] trajectory~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryQuery-response)))
  "Returns full string definition for message of type 'TrajectoryQuery-response"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%geometry_msgs/PoseStamped[] trajectory~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryQuery-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryQuery-response
    (cl:cons ':status (status msg))
    (cl:cons ':trajectory (trajectory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrajectoryQuery)))
  'TrajectoryQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrajectoryQuery)))
  'TrajectoryQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryQuery)))
  "Returns string type for a service object of type '<TrajectoryQuery>"
  "cartographer_ros_msgs/TrajectoryQuery")