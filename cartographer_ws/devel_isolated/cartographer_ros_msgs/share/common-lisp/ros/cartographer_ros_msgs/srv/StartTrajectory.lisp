; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-srv)


;//! \htmlinclude StartTrajectory-request.msg.html

(cl:defclass <StartTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((configuration_directory
    :reader configuration_directory
    :initarg :configuration_directory
    :type cl:string
    :initform "")
   (configuration_basename
    :reader configuration_basename
    :initarg :configuration_basename
    :type cl:string
    :initform "")
   (use_initial_pose
    :reader use_initial_pose
    :initarg :use_initial_pose
    :type cl:boolean
    :initform cl:nil)
   (initial_pose
    :reader initial_pose
    :initarg :initial_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (relative_to_trajectory_id
    :reader relative_to_trajectory_id
    :initarg :relative_to_trajectory_id
    :type cl:integer
    :initform 0))
)

(cl:defclass StartTrajectory-request (<StartTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<StartTrajectory-request> is deprecated: use cartographer_ros_msgs-srv:StartTrajectory-request instead.")))

(cl:ensure-generic-function 'configuration_directory-val :lambda-list '(m))
(cl:defmethod configuration_directory-val ((m <StartTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:configuration_directory-val is deprecated.  Use cartographer_ros_msgs-srv:configuration_directory instead.")
  (configuration_directory m))

(cl:ensure-generic-function 'configuration_basename-val :lambda-list '(m))
(cl:defmethod configuration_basename-val ((m <StartTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:configuration_basename-val is deprecated.  Use cartographer_ros_msgs-srv:configuration_basename instead.")
  (configuration_basename m))

(cl:ensure-generic-function 'use_initial_pose-val :lambda-list '(m))
(cl:defmethod use_initial_pose-val ((m <StartTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:use_initial_pose-val is deprecated.  Use cartographer_ros_msgs-srv:use_initial_pose instead.")
  (use_initial_pose m))

(cl:ensure-generic-function 'initial_pose-val :lambda-list '(m))
(cl:defmethod initial_pose-val ((m <StartTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:initial_pose-val is deprecated.  Use cartographer_ros_msgs-srv:initial_pose instead.")
  (initial_pose m))

(cl:ensure-generic-function 'relative_to_trajectory_id-val :lambda-list '(m))
(cl:defmethod relative_to_trajectory_id-val ((m <StartTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:relative_to_trajectory_id-val is deprecated.  Use cartographer_ros_msgs-srv:relative_to_trajectory_id instead.")
  (relative_to_trajectory_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTrajectory-request>) ostream)
  "Serializes a message object of type '<StartTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'configuration_directory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'configuration_directory))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'configuration_basename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'configuration_basename))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_initial_pose) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initial_pose) ostream)
  (cl:let* ((signed (cl:slot-value msg 'relative_to_trajectory_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTrajectory-request>) istream)
  "Deserializes a message object of type '<StartTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'configuration_directory) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'configuration_directory) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'configuration_basename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'configuration_basename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'use_initial_pose) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initial_pose) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'relative_to_trajectory_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTrajectory-request>)))
  "Returns string type for a service object of type '<StartTrajectory-request>"
  "cartographer_ros_msgs/StartTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory-request)))
  "Returns string type for a service object of type 'StartTrajectory-request"
  "cartographer_ros_msgs/StartTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTrajectory-request>)))
  "Returns md5sum for a message object of type '<StartTrajectory-request>"
  "dcc000df748d283ba7bf678a47ffa491")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTrajectory-request)))
  "Returns md5sum for a message object of type 'StartTrajectory-request"
  "dcc000df748d283ba7bf678a47ffa491")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTrajectory-request>)))
  "Returns full string definition for message of type '<StartTrajectory-request>"
  (cl:format cl:nil "# Copyright 2016 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string configuration_directory~%string configuration_basename~%bool use_initial_pose~%geometry_msgs/Pose initial_pose~%int32 relative_to_trajectory_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTrajectory-request)))
  "Returns full string definition for message of type 'StartTrajectory-request"
  (cl:format cl:nil "# Copyright 2016 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string configuration_directory~%string configuration_basename~%bool use_initial_pose~%geometry_msgs/Pose initial_pose~%int32 relative_to_trajectory_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'configuration_directory))
     4 (cl:length (cl:slot-value msg 'configuration_basename))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initial_pose))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTrajectory-request
    (cl:cons ':configuration_directory (configuration_directory msg))
    (cl:cons ':configuration_basename (configuration_basename msg))
    (cl:cons ':use_initial_pose (use_initial_pose msg))
    (cl:cons ':initial_pose (initial_pose msg))
    (cl:cons ':relative_to_trajectory_id (relative_to_trajectory_id msg))
))
;//! \htmlinclude StartTrajectory-response.msg.html

(cl:defclass <StartTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cartographer_ros_msgs-msg:StatusResponse
    :initform (cl:make-instance 'cartographer_ros_msgs-msg:StatusResponse))
   (trajectory_id
    :reader trajectory_id
    :initarg :trajectory_id
    :type cl:integer
    :initform 0))
)

(cl:defclass StartTrajectory-response (<StartTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<StartTrajectory-response> is deprecated: use cartographer_ros_msgs-srv:StartTrajectory-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <StartTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:status-val is deprecated.  Use cartographer_ros_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'trajectory_id-val :lambda-list '(m))
(cl:defmethod trajectory_id-val ((m <StartTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:trajectory_id-val is deprecated.  Use cartographer_ros_msgs-srv:trajectory_id instead.")
  (trajectory_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTrajectory-response>) ostream)
  "Serializes a message object of type '<StartTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (cl:let* ((signed (cl:slot-value msg 'trajectory_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTrajectory-response>) istream)
  "Deserializes a message object of type '<StartTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trajectory_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTrajectory-response>)))
  "Returns string type for a service object of type '<StartTrajectory-response>"
  "cartographer_ros_msgs/StartTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory-response)))
  "Returns string type for a service object of type 'StartTrajectory-response"
  "cartographer_ros_msgs/StartTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTrajectory-response>)))
  "Returns md5sum for a message object of type '<StartTrajectory-response>"
  "dcc000df748d283ba7bf678a47ffa491")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTrajectory-response)))
  "Returns md5sum for a message object of type 'StartTrajectory-response"
  "dcc000df748d283ba7bf678a47ffa491")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTrajectory-response>)))
  "Returns full string definition for message of type '<StartTrajectory-response>"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%int32 trajectory_id~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTrajectory-response)))
  "Returns full string definition for message of type 'StartTrajectory-response"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%int32 trajectory_id~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTrajectory-response
    (cl:cons ':status (status msg))
    (cl:cons ':trajectory_id (trajectory_id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartTrajectory)))
  'StartTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartTrajectory)))
  'StartTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory)))
  "Returns string type for a service object of type '<StartTrajectory>"
  "cartographer_ros_msgs/StartTrajectory")