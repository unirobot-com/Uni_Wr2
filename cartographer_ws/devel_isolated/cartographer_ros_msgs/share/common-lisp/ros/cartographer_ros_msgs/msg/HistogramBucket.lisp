; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-msg)


;//! \htmlinclude HistogramBucket.msg.html

(cl:defclass <HistogramBucket> (roslisp-msg-protocol:ros-message)
  ((bucket_boundary
    :reader bucket_boundary
    :initarg :bucket_boundary
    :type cl:float
    :initform 0.0)
   (count
    :reader count
    :initarg :count
    :type cl:float
    :initform 0.0))
)

(cl:defclass HistogramBucket (<HistogramBucket>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HistogramBucket>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HistogramBucket)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-msg:<HistogramBucket> is deprecated: use cartographer_ros_msgs-msg:HistogramBucket instead.")))

(cl:ensure-generic-function 'bucket_boundary-val :lambda-list '(m))
(cl:defmethod bucket_boundary-val ((m <HistogramBucket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:bucket_boundary-val is deprecated.  Use cartographer_ros_msgs-msg:bucket_boundary instead.")
  (bucket_boundary m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <HistogramBucket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:count-val is deprecated.  Use cartographer_ros_msgs-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HistogramBucket>) ostream)
  "Serializes a message object of type '<HistogramBucket>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bucket_boundary))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'count))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HistogramBucket>) istream)
  "Deserializes a message object of type '<HistogramBucket>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bucket_boundary) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'count) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HistogramBucket>)))
  "Returns string type for a message object of type '<HistogramBucket>"
  "cartographer_ros_msgs/HistogramBucket")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HistogramBucket)))
  "Returns string type for a message object of type 'HistogramBucket"
  "cartographer_ros_msgs/HistogramBucket")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HistogramBucket>)))
  "Returns md5sum for a message object of type '<HistogramBucket>"
  "b579df35b32758cf09f65ae223aea0ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HistogramBucket)))
  "Returns md5sum for a message object of type 'HistogramBucket"
  "b579df35b32758cf09f65ae223aea0ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HistogramBucket>)))
  "Returns full string definition for message of type '<HistogramBucket>"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HistogramBucket)))
  "Returns full string definition for message of type 'HistogramBucket"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HistogramBucket>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HistogramBucket>))
  "Converts a ROS message object to a list"
  (cl:list 'HistogramBucket
    (cl:cons ':bucket_boundary (bucket_boundary msg))
    (cl:cons ':count (count msg))
))
