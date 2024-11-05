; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-msg)


;//! \htmlinclude Metric.msg.html

(cl:defclass <Metric> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (labels
    :reader labels
    :initarg :labels
    :type (cl:vector cartographer_ros_msgs-msg:MetricLabel)
   :initform (cl:make-array 0 :element-type 'cartographer_ros_msgs-msg:MetricLabel :initial-element (cl:make-instance 'cartographer_ros_msgs-msg:MetricLabel)))
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (counts_by_bucket
    :reader counts_by_bucket
    :initarg :counts_by_bucket
    :type (cl:vector cartographer_ros_msgs-msg:HistogramBucket)
   :initform (cl:make-array 0 :element-type 'cartographer_ros_msgs-msg:HistogramBucket :initial-element (cl:make-instance 'cartographer_ros_msgs-msg:HistogramBucket))))
)

(cl:defclass Metric (<Metric>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Metric>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Metric)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-msg:<Metric> is deprecated: use cartographer_ros_msgs-msg:Metric instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Metric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:type-val is deprecated.  Use cartographer_ros_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'labels-val :lambda-list '(m))
(cl:defmethod labels-val ((m <Metric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:labels-val is deprecated.  Use cartographer_ros_msgs-msg:labels instead.")
  (labels m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Metric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:value-val is deprecated.  Use cartographer_ros_msgs-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'counts_by_bucket-val :lambda-list '(m))
(cl:defmethod counts_by_bucket-val ((m <Metric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:counts_by_bucket-val is deprecated.  Use cartographer_ros_msgs-msg:counts_by_bucket instead.")
  (counts_by_bucket m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Metric>)))
    "Constants for message type '<Metric>"
  '((:TYPE_COUNTER . 0)
    (:TYPE_GAUGE . 1)
    (:TYPE_HISTOGRAM . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Metric)))
    "Constants for message type 'Metric"
  '((:TYPE_COUNTER . 0)
    (:TYPE_GAUGE . 1)
    (:TYPE_HISTOGRAM . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Metric>) ostream)
  "Serializes a message object of type '<Metric>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'labels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'labels))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'counts_by_bucket))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'counts_by_bucket))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Metric>) istream)
  "Deserializes a message object of type '<Metric>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'labels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'labels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartographer_ros_msgs-msg:MetricLabel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'counts_by_bucket) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'counts_by_bucket)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartographer_ros_msgs-msg:HistogramBucket))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Metric>)))
  "Returns string type for a message object of type '<Metric>"
  "cartographer_ros_msgs/Metric")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Metric)))
  "Returns string type for a message object of type 'Metric"
  "cartographer_ros_msgs/Metric")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Metric>)))
  "Returns md5sum for a message object of type '<Metric>"
  "94a6ea1c6ef245b483a220f6769c8e36")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Metric)))
  "Returns md5sum for a message object of type 'Metric"
  "94a6ea1c6ef245b483a220f6769c8e36")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Metric>)))
  "Returns full string definition for message of type '<Metric>"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Metric)))
  "Returns full string definition for message of type 'Metric"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Metric>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'labels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'counts_by_bucket) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Metric>))
  "Converts a ROS message object to a list"
  (cl:list 'Metric
    (cl:cons ':type (type msg))
    (cl:cons ':labels (labels msg))
    (cl:cons ':value (value msg))
    (cl:cons ':counts_by_bucket (counts_by_bucket msg))
))
