; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-msg)


;//! \htmlinclude BagfileProgress.msg.html

(cl:defclass <BagfileProgress> (roslisp-msg-protocol:ros-message)
  ((current_bagfile_name
    :reader current_bagfile_name
    :initarg :current_bagfile_name
    :type cl:string
    :initform "")
   (current_bagfile_id
    :reader current_bagfile_id
    :initarg :current_bagfile_id
    :type cl:integer
    :initform 0)
   (total_bagfiles
    :reader total_bagfiles
    :initarg :total_bagfiles
    :type cl:integer
    :initform 0)
   (total_messages
    :reader total_messages
    :initarg :total_messages
    :type cl:integer
    :initform 0)
   (processed_messages
    :reader processed_messages
    :initarg :processed_messages
    :type cl:integer
    :initform 0)
   (total_seconds
    :reader total_seconds
    :initarg :total_seconds
    :type cl:float
    :initform 0.0)
   (processed_seconds
    :reader processed_seconds
    :initarg :processed_seconds
    :type cl:float
    :initform 0.0))
)

(cl:defclass BagfileProgress (<BagfileProgress>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BagfileProgress>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BagfileProgress)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-msg:<BagfileProgress> is deprecated: use cartographer_ros_msgs-msg:BagfileProgress instead.")))

(cl:ensure-generic-function 'current_bagfile_name-val :lambda-list '(m))
(cl:defmethod current_bagfile_name-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:current_bagfile_name-val is deprecated.  Use cartographer_ros_msgs-msg:current_bagfile_name instead.")
  (current_bagfile_name m))

(cl:ensure-generic-function 'current_bagfile_id-val :lambda-list '(m))
(cl:defmethod current_bagfile_id-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:current_bagfile_id-val is deprecated.  Use cartographer_ros_msgs-msg:current_bagfile_id instead.")
  (current_bagfile_id m))

(cl:ensure-generic-function 'total_bagfiles-val :lambda-list '(m))
(cl:defmethod total_bagfiles-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:total_bagfiles-val is deprecated.  Use cartographer_ros_msgs-msg:total_bagfiles instead.")
  (total_bagfiles m))

(cl:ensure-generic-function 'total_messages-val :lambda-list '(m))
(cl:defmethod total_messages-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:total_messages-val is deprecated.  Use cartographer_ros_msgs-msg:total_messages instead.")
  (total_messages m))

(cl:ensure-generic-function 'processed_messages-val :lambda-list '(m))
(cl:defmethod processed_messages-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:processed_messages-val is deprecated.  Use cartographer_ros_msgs-msg:processed_messages instead.")
  (processed_messages m))

(cl:ensure-generic-function 'total_seconds-val :lambda-list '(m))
(cl:defmethod total_seconds-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:total_seconds-val is deprecated.  Use cartographer_ros_msgs-msg:total_seconds instead.")
  (total_seconds m))

(cl:ensure-generic-function 'processed_seconds-val :lambda-list '(m))
(cl:defmethod processed_seconds-val ((m <BagfileProgress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:processed_seconds-val is deprecated.  Use cartographer_ros_msgs-msg:processed_seconds instead.")
  (processed_seconds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BagfileProgress>) ostream)
  "Serializes a message object of type '<BagfileProgress>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'current_bagfile_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'current_bagfile_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_bagfile_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current_bagfile_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current_bagfile_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current_bagfile_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_bagfiles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_bagfiles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_bagfiles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_bagfiles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'processed_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'processed_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'processed_messages)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'processed_messages)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_seconds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'processed_seconds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BagfileProgress>) istream)
  "Deserializes a message object of type '<BagfileProgress>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_bagfile_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'current_bagfile_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_bagfile_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current_bagfile_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current_bagfile_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current_bagfile_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_bagfiles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_bagfiles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_bagfiles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_bagfiles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'processed_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'processed_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'processed_messages)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'processed_messages)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_seconds) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'processed_seconds) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BagfileProgress>)))
  "Returns string type for a message object of type '<BagfileProgress>"
  "cartographer_ros_msgs/BagfileProgress")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BagfileProgress)))
  "Returns string type for a message object of type 'BagfileProgress"
  "cartographer_ros_msgs/BagfileProgress")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BagfileProgress>)))
  "Returns md5sum for a message object of type '<BagfileProgress>"
  "2a36f93b13e2b297d45098a38cb00510")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BagfileProgress)))
  "Returns md5sum for a message object of type 'BagfileProgress"
  "2a36f93b13e2b297d45098a38cb00510")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BagfileProgress>)))
  "Returns full string definition for message of type '<BagfileProgress>"
  (cl:format cl:nil "#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%# Contains general information about the bagfiles processing progress~%~%string current_bagfile_name~%uint32 current_bagfile_id~%uint32 total_bagfiles~%uint32 total_messages~%uint32 processed_messages~%float32 total_seconds~%float32 processed_seconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BagfileProgress)))
  "Returns full string definition for message of type 'BagfileProgress"
  (cl:format cl:nil "#~%# Licensed under the Apache License, Version 2.0 (the 'License');~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an 'AS IS' BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%# Contains general information about the bagfiles processing progress~%~%string current_bagfile_name~%uint32 current_bagfile_id~%uint32 total_bagfiles~%uint32 total_messages~%uint32 processed_messages~%float32 total_seconds~%float32 processed_seconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BagfileProgress>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'current_bagfile_name))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BagfileProgress>))
  "Converts a ROS message object to a list"
  (cl:list 'BagfileProgress
    (cl:cons ':current_bagfile_name (current_bagfile_name msg))
    (cl:cons ':current_bagfile_id (current_bagfile_id msg))
    (cl:cons ':total_bagfiles (total_bagfiles msg))
    (cl:cons ':total_messages (total_messages msg))
    (cl:cons ':processed_messages (processed_messages msg))
    (cl:cons ':total_seconds (total_seconds msg))
    (cl:cons ':processed_seconds (processed_seconds msg))
))
