; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-msg)


;//! \htmlinclude MetricFamily.msg.html

(cl:defclass <MetricFamily> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (metrics
    :reader metrics
    :initarg :metrics
    :type (cl:vector cartographer_ros_msgs-msg:Metric)
   :initform (cl:make-array 0 :element-type 'cartographer_ros_msgs-msg:Metric :initial-element (cl:make-instance 'cartographer_ros_msgs-msg:Metric))))
)

(cl:defclass MetricFamily (<MetricFamily>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MetricFamily>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MetricFamily)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-msg:<MetricFamily> is deprecated: use cartographer_ros_msgs-msg:MetricFamily instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MetricFamily>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:name-val is deprecated.  Use cartographer_ros_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <MetricFamily>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:description-val is deprecated.  Use cartographer_ros_msgs-msg:description instead.")
  (description m))

(cl:ensure-generic-function 'metrics-val :lambda-list '(m))
(cl:defmethod metrics-val ((m <MetricFamily>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-msg:metrics-val is deprecated.  Use cartographer_ros_msgs-msg:metrics instead.")
  (metrics m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MetricFamily>) ostream)
  "Serializes a message object of type '<MetricFamily>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'metrics))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'metrics))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MetricFamily>) istream)
  "Deserializes a message object of type '<MetricFamily>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'metrics) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'metrics)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartographer_ros_msgs-msg:Metric))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MetricFamily>)))
  "Returns string type for a message object of type '<MetricFamily>"
  "cartographer_ros_msgs/MetricFamily")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MetricFamily)))
  "Returns string type for a message object of type 'MetricFamily"
  "cartographer_ros_msgs/MetricFamily")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MetricFamily>)))
  "Returns md5sum for a message object of type '<MetricFamily>"
  "583a11b161bb4a70f5df274715bcaf10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MetricFamily)))
  "Returns md5sum for a message object of type 'MetricFamily"
  "583a11b161bb4a70f5df274715bcaf10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MetricFamily>)))
  "Returns full string definition for message of type '<MetricFamily>"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string name~%string description~%cartographer_ros_msgs/Metric[] metrics~%~%================================================================================~%MSG: cartographer_ros_msgs/Metric~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MetricFamily)))
  "Returns full string definition for message of type 'MetricFamily"
  (cl:format cl:nil "# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string name~%string description~%cartographer_ros_msgs/Metric[] metrics~%~%================================================================================~%MSG: cartographer_ros_msgs/Metric~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MetricFamily>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'description))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'metrics) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MetricFamily>))
  "Converts a ROS message object to a list"
  (cl:list 'MetricFamily
    (cl:cons ':name (name msg))
    (cl:cons ':description (description msg))
    (cl:cons ':metrics (metrics msg))
))
