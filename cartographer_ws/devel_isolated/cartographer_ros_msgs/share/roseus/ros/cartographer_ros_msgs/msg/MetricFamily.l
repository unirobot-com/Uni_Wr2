;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::MetricFamily)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'MetricFamily (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::METRICFAMILY")
  (make-package "CARTOGRAPHER_ROS_MSGS::METRICFAMILY"))

(in-package "ROS")
;;//! \htmlinclude MetricFamily.msg.html


(defclass cartographer_ros_msgs::MetricFamily
  :super ros::object
  :slots (_name _description _metrics ))

(defmethod cartographer_ros_msgs::MetricFamily
  (:init
   (&key
    ((:name __name) "")
    ((:description __description) "")
    ((:metrics __metrics) ())
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _description (string __description))
   (setq _metrics __metrics)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:metrics
   (&rest __metrics)
   (if (keywordp (car __metrics))
       (send* _metrics __metrics)
     (progn
       (if __metrics (setq _metrics (car __metrics)))
       _metrics)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _description
    4 (length _description)
    ;; cartographer_ros_msgs/Metric[] _metrics
    (apply #'+ (send-all _metrics :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;; cartographer_ros_msgs/Metric[] _metrics
     (write-long (length _metrics) s)
     (dolist (elem _metrics)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; cartographer_ros_msgs/Metric[] _metrics
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _metrics (let (r) (dotimes (i n) (push (instance cartographer_ros_msgs::Metric :init) r)) r))
     (dolist (elem- _metrics)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cartographer_ros_msgs::MetricFamily :md5sum-) "583a11b161bb4a70f5df274715bcaf10")
(setf (get cartographer_ros_msgs::MetricFamily :datatype-) "cartographer_ros_msgs/MetricFamily")
(setf (get cartographer_ros_msgs::MetricFamily :definition-)
      "# 2018 The Cartographer Authors
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



(provide :cartographer_ros_msgs/MetricFamily "583a11b161bb4a70f5df274715bcaf10")


