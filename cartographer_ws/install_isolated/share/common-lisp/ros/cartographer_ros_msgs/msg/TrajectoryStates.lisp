; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-msg)


;//! \htmlinclude TrajectoryStates.msg.html

(cl:defclass <TrajectoryStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (trajectory_id
    :reader trajectory_id
    :initarg :trajectory_id
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (trajectory_state
    :reader trajectory_state
    :initarg :trajectory_state
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass TrajectoryStates (<TrajectoryStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-msg:<TrajectoryStates> is deprecated: use cartographer_ros_msgs-msg:TrajectoryStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajectoryStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:header-val is deprecated.  Use cartographer_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'trajectory_id-val :lambda-list '(m))
(cl:defmethod trajectory_id-val ((m <TrajectoryStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:trajectory_id-val is deprecated.  Use cartographer_ros_msgs-msg:trajectory_id instead.")
  (trajectory_id m))

(cl:ensure-generic-function 'trajectory_state-val :lambda-list '(m))
(cl:defmethod trajectory_state-val ((m <TrajectoryStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:trajectory_state-val is deprecated.  Use cartographer_ros_msgs-msg:trajectory_state instead.")
  (trajectory_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryStates>)))
    "Constants for message type '<TrajectoryStates>"
  '((:ACTIVE . 0)
    (:FINISHED . 1)
    (:FROZEN . 2)
    (:DELETED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryStates)))
    "Constants for message type 'TrajectoryStates"
  '((:ACTIVE . 0)
    (:FINISHED . 1)
    (:FROZEN . 2)
    (:DELETED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryStates>) ostream)
  "Serializes a message object of type '<TrajectoryStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'trajectory_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'trajectory_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryStates>) istream)
  "Deserializes a message object of type '<TrajectoryStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryStates>)))
  "Returns string type for a message object of type '<TrajectoryStates>"
  "cartographer_ros_msgs/TrajectoryStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryStates)))
  "Returns string type for a message object of type 'TrajectoryStates"
  "cartographer_ros_msgs/TrajectoryStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryStates>)))
  "Returns md5sum for a message object of type '<TrajectoryStates>"
  "85efdd795e95b57a59cb785ecb152345")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryStates)))
  "Returns md5sum for a message object of type 'TrajectoryStates"
  "85efdd795e95b57a59cb785ecb152345")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryStates>)))
  "Returns full string definition for message of type '<TrajectoryStates>"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 ACTIVE = 0~%uint8 FINISHED = 1~%uint8 FROZEN = 2~%uint8 DELETED = 3~%~%std_msgs/Header header~%int32[] trajectory_id~%uint8[] trajectory_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryStates)))
  "Returns full string definition for message of type 'TrajectoryStates"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 ACTIVE = 0~%uint8 FINISHED = 1~%uint8 FROZEN = 2~%uint8 DELETED = 3~%~%std_msgs/Header header~%int32[] trajectory_id~%uint8[] trajectory_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryStates>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryStates
    (cl:cons ':header (header msg))
    (cl:cons ':trajectory_id (trajectory_id msg))
    (cl:cons ':trajectory_state (trajectory_state msg))
))
