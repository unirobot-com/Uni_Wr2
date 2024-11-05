; Auto-generated. Do not edit!


(cl:in-package cartographer_ros_msgs-srv)


;//! \htmlinclude ReadMetrics-request.msg.html

(cl:defclass <ReadMetrics-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReadMetrics-request (<ReadMetrics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMetrics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMetrics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<ReadMetrics-request> is deprecated: use cartographer_ros_msgs-srv:ReadMetrics-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMetrics-request>) ostream)
  "Serializes a message object of type '<ReadMetrics-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMetrics-request>) istream)
  "Deserializes a message object of type '<ReadMetrics-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMetrics-request>)))
  "Returns string type for a service object of type '<ReadMetrics-request>"
  "cartographer_ros_msgs/ReadMetricsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMetrics-request)))
  "Returns string type for a service object of type 'ReadMetrics-request"
  "cartographer_ros_msgs/ReadMetricsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMetrics-request>)))
  "Returns md5sum for a message object of type '<ReadMetrics-request>"
  "a1fe8d7dcf3708e96e015774b1df470e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMetrics-request)))
  "Returns md5sum for a message object of type 'ReadMetrics-request"
  "a1fe8d7dcf3708e96e015774b1df470e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMetrics-request>)))
  "Returns full string definition for message of type '<ReadMetrics-request>"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMetrics-request)))
  "Returns full string definition for message of type 'ReadMetrics-request"
  (cl:format cl:nil "# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMetrics-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMetrics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMetrics-request
))
;//! \htmlinclude ReadMetrics-response.msg.html

(cl:defclass <ReadMetrics-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cartographer_ros_msgs-msg:StatusResponse
    :initform (cl:make-instance 'cartographer_ros_msgs-msg:StatusResponse))
   (metric_families
    :reader metric_families
    :initarg :metric_families
    :type (cl:vector cartographer_ros_msgs-msg:MetricFamily)
   :initform (cl:make-array 0 :element-type 'cartographer_ros_msgs-msg:MetricFamily :initial-element (cl:make-instance 'cartographer_ros_msgs-msg:MetricFamily)))
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0))
)

(cl:defclass ReadMetrics-response (<ReadMetrics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMetrics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMetrics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartographer_ros_msgs-srv:<ReadMetrics-response> is deprecated: use cartographer_ros_msgs-srv:ReadMetrics-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ReadMetrics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:status-val is deprecated.  Use cartographer_ros_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'metric_families-val :lambda-list '(m))
(cl:defmethod metric_families-val ((m <ReadMetrics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:metric_families-val is deprecated.  Use cartographer_ros_msgs-srv:metric_families instead.")
  (metric_families m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ReadMetrics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartographer_ros_msgs-srv:timestamp-val is deprecated.  Use cartographer_ros_msgs-srv:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMetrics-response>) ostream)
  "Serializes a message object of type '<ReadMetrics-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'metric_families))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'metric_families))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMetrics-response>) istream)
  "Deserializes a message object of type '<ReadMetrics-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'metric_families) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'metric_families)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartographer_ros_msgs-msg:MetricFamily))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMetrics-response>)))
  "Returns string type for a service object of type '<ReadMetrics-response>"
  "cartographer_ros_msgs/ReadMetricsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMetrics-response)))
  "Returns string type for a service object of type 'ReadMetrics-response"
  "cartographer_ros_msgs/ReadMetricsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMetrics-response>)))
  "Returns md5sum for a message object of type '<ReadMetrics-response>"
  "a1fe8d7dcf3708e96e015774b1df470e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMetrics-response)))
  "Returns md5sum for a message object of type 'ReadMetrics-response"
  "a1fe8d7dcf3708e96e015774b1df470e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMetrics-response>)))
  "Returns full string definition for message of type '<ReadMetrics-response>"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%cartographer_ros_msgs/MetricFamily[] metric_families~%time timestamp~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricFamily~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string name~%string description~%cartographer_ros_msgs/Metric[] metrics~%~%================================================================================~%MSG: cartographer_ros_msgs/Metric~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMetrics-response)))
  "Returns full string definition for message of type 'ReadMetrics-response"
  (cl:format cl:nil "cartographer_ros_msgs/StatusResponse status~%cartographer_ros_msgs/MetricFamily[] metric_families~%time timestamp~%~%~%================================================================================~%MSG: cartographer_ros_msgs/StatusResponse~%# Copyright 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A common message type to indicate the outcome of a service call.~%uint8 code~%string message~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricFamily~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string name~%string description~%cartographer_ros_msgs/Metric[] metrics~%~%================================================================================~%MSG: cartographer_ros_msgs/Metric~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%uint8 TYPE_COUNTER=0~%uint8 TYPE_GAUGE=1~%uint8 TYPE_HISTOGRAM=2~%~%uint8 type~%cartographer_ros_msgs/MetricLabel[] labels~%~%# TYPE_COUNTER or TYPE_GAUGE~%float64 value~%~%# TYPE_HISTOGRAM~%cartographer_ros_msgs/HistogramBucket[] counts_by_bucket~%~%================================================================================~%MSG: cartographer_ros_msgs/MetricLabel~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string key~%string value~%~%================================================================================~%MSG: cartographer_ros_msgs/HistogramBucket~%# 2018 The Cartographer Authors~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#      http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# A histogram bucket counts values x for which:~%#   previous_boundary < x <= bucket_boundary~%# holds.~%float64 bucket_boundary~%float64 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMetrics-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'metric_families) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMetrics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMetrics-response
    (cl:cons ':status (status msg))
    (cl:cons ':metric_families (metric_families msg))
    (cl:cons ':timestamp (timestamp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadMetrics)))
  'ReadMetrics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadMetrics)))
  'ReadMetrics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMetrics)))
  "Returns string type for a service object of type '<ReadMetrics>"
  "cartographer_ros_msgs/ReadMetrics")