;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::ReadMetrics)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'ReadMetrics (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::READMETRICS")
  (make-package "CARTOGRAPHER_ROS_MSGS::READMETRICS"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::READMETRICSREQUEST")
  (make-package "CARTOGRAPHER_ROS_MSGS::READMETRICSREQUEST"))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::READMETRICSRESPONSE")
  (make-package "CARTOGRAPHER_ROS_MSGS::READMETRICSRESPONSE"))

(in-package "ROS")





(defclass cartographer_ros_msgs::ReadMetricsRequest
  :super ros::object
  :slots ())

(defmethod cartographer_ros_msgs::ReadMetricsRequest
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

(defclass cartographer_ros_msgs::ReadMetricsResponse
  :super ros::object
  :slots (_status _metric_families _timestamp ))

(defmethod cartographer_ros_msgs::ReadMetricsResponse
  (:init
   (&key
    ((:status __status) (instance cartographer_ros_msgs::StatusResponse :init))
    ((:metric_families __metric_families) ())
    ((:timestamp __timestamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _status __status)
   (setq _metric_families __metric_families)
   (setq _timestamp __timestamp)
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:metric_families
   (&rest __metric_families)
   (if (keywordp (car __metric_families))
       (send* _metric_families __metric_families)
     (progn
       (if __metric_families (setq _metric_families (car __metric_families)))
       _metric_families)))
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; cartographer_ros_msgs/StatusResponse _status
    (send _status :serialization-length)
    ;; cartographer_ros_msgs/MetricFamily[] _metric_families
    (apply #'+ (send-all _metric_families :serialization-length)) 4
    ;; time _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartographer_ros_msgs/StatusResponse _status
       (send _status :serialize s)
     ;; cartographer_ros_msgs/MetricFamily[] _metric_families
     (write-long (length _metric_families) s)
     (dolist (elem _metric_families)
       (send elem :serialize s)
       )
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartographer_ros_msgs/StatusResponse _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; cartographer_ros_msgs/MetricFamily[] _metric_families
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _metric_families (let (r) (dotimes (i n) (push (instance cartographer_ros_msgs::MetricFamily :init) r)) r))
     (dolist (elem- _metric_families)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cartographer_ros_msgs::ReadMetrics
  :super ros::object
  :slots ())

(setf (get cartographer_ros_msgs::ReadMetrics :md5sum-) "a1fe8d7dcf3708e96e015774b1df470e")
(setf (get cartographer_ros_msgs::ReadMetrics :datatype-) "cartographer_ros_msgs/ReadMetrics")
(setf (get cartographer_ros_msgs::ReadMetrics :request) cartographer_ros_msgs::ReadMetricsRequest)
(setf (get cartographer_ros_msgs::ReadMetrics :response) cartographer_ros_msgs::ReadMetricsResponse)

(defmethod cartographer_ros_msgs::ReadMetricsRequest
  (:response () (instance cartographer_ros_msgs::ReadMetricsResponse :init)))

(setf (get cartographer_ros_msgs::ReadMetricsRequest :md5sum-) "a1fe8d7dcf3708e96e015774b1df470e")
(setf (get cartographer_ros_msgs::ReadMetricsRequest :datatype-) "cartographer_ros_msgs/ReadMetricsRequest")
(setf (get cartographer_ros_msgs::ReadMetricsRequest :definition-)
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
cartographer_ros_msgs/MetricFamily[] metric_families
time timestamp


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
MSG: cartographer_ros_msgs/MetricFamily
# 2018 The Cartographer Authors
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

string name
string description
cartographer_ros_msgs/Metric[] metrics

================================================================================
MSG: cartographer_ros_msgs/Metric
# 2018 The Cartographer Authors
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

uint8 TYPE_COUNTER=0
uint8 TYPE_GAUGE=1
uint8 TYPE_HISTOGRAM=2

uint8 type
cartographer_ros_msgs/MetricLabel[] labels

# TYPE_COUNTER or TYPE_GAUGE
float64 value

# TYPE_HISTOGRAM
cartographer_ros_msgs/HistogramBucket[] counts_by_bucket

================================================================================
MSG: cartographer_ros_msgs/MetricLabel
# 2018 The Cartographer Authors
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

string key
string value

================================================================================
MSG: cartographer_ros_msgs/HistogramBucket
# 2018 The Cartographer Authors
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

# A histogram bucket counts values x for which:
#   previous_boundary < x <= bucket_boundary
# holds.
float64 bucket_boundary
float64 count
")

(setf (get cartographer_ros_msgs::ReadMetricsResponse :md5sum-) "a1fe8d7dcf3708e96e015774b1df470e")
(setf (get cartographer_ros_msgs::ReadMetricsResponse :datatype-) "cartographer_ros_msgs/ReadMetricsResponse")
(setf (get cartographer_ros_msgs::ReadMetricsResponse :definition-)
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
cartographer_ros_msgs/MetricFamily[] metric_families
time timestamp


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
MSG: cartographer_ros_msgs/MetricFamily
# 2018 The Cartographer Authors
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

string name
string description
cartographer_ros_msgs/Metric[] metrics

================================================================================
MSG: cartographer_ros_msgs/Metric
# 2018 The Cartographer Authors
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

uint8 TYPE_COUNTER=0
uint8 TYPE_GAUGE=1
uint8 TYPE_HISTOGRAM=2

uint8 type
cartographer_ros_msgs/MetricLabel[] labels

# TYPE_COUNTER or TYPE_GAUGE
float64 value

# TYPE_HISTOGRAM
cartographer_ros_msgs/HistogramBucket[] counts_by_bucket

================================================================================
MSG: cartographer_ros_msgs/MetricLabel
# 2018 The Cartographer Authors
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

string key
string value

================================================================================
MSG: cartographer_ros_msgs/HistogramBucket
# 2018 The Cartographer Authors
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

# A histogram bucket counts values x for which:
#   previous_boundary < x <= bucket_boundary
# holds.
float64 bucket_boundary
float64 count
")



(provide :cartographer_ros_msgs/ReadMetrics "a1fe8d7dcf3708e96e015774b1df470e")


