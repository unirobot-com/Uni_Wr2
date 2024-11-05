;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::Metric)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'Metric (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::METRIC")
  (make-package "CARTOGRAPHER_ROS_MSGS::METRIC"))

(in-package "ROS")
;;//! \htmlinclude Metric.msg.html


(intern "*TYPE_COUNTER*" (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(shadow '*TYPE_COUNTER* (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(defconstant cartographer_ros_msgs::Metric::*TYPE_COUNTER* 0)
(intern "*TYPE_GAUGE*" (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(shadow '*TYPE_GAUGE* (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(defconstant cartographer_ros_msgs::Metric::*TYPE_GAUGE* 1)
(intern "*TYPE_HISTOGRAM*" (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(shadow '*TYPE_HISTOGRAM* (find-package "CARTOGRAPHER_ROS_MSGS::METRIC"))
(defconstant cartographer_ros_msgs::Metric::*TYPE_HISTOGRAM* 2)

(defun cartographer_ros_msgs::Metric-to-symbol (const)
  (cond
        ((= const 0) 'cartographer_ros_msgs::Metric::*TYPE_COUNTER*)
        ((= const 1) 'cartographer_ros_msgs::Metric::*TYPE_GAUGE*)
        ((= const 2) 'cartographer_ros_msgs::Metric::*TYPE_HISTOGRAM*)
        (t nil)))

(defclass cartographer_ros_msgs::Metric
  :super ros::object
  :slots (_type _labels _value _counts_by_bucket ))

(defmethod cartographer_ros_msgs::Metric
  (:init
   (&key
    ((:type __type) 0)
    ((:labels __labels) ())
    ((:value __value) 0.0)
    ((:counts_by_bucket __counts_by_bucket) ())
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _labels __labels)
   (setq _value (float __value))
   (setq _counts_by_bucket __counts_by_bucket)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:labels
   (&rest __labels)
   (if (keywordp (car __labels))
       (send* _labels __labels)
     (progn
       (if __labels (setq _labels (car __labels)))
       _labels)))
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:counts_by_bucket
   (&rest __counts_by_bucket)
   (if (keywordp (car __counts_by_bucket))
       (send* _counts_by_bucket __counts_by_bucket)
     (progn
       (if __counts_by_bucket (setq _counts_by_bucket (car __counts_by_bucket)))
       _counts_by_bucket)))
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; cartographer_ros_msgs/MetricLabel[] _labels
    (apply #'+ (send-all _labels :serialization-length)) 4
    ;; float64 _value
    8
    ;; cartographer_ros_msgs/HistogramBucket[] _counts_by_bucket
    (apply #'+ (send-all _counts_by_bucket :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; cartographer_ros_msgs/MetricLabel[] _labels
     (write-long (length _labels) s)
     (dolist (elem _labels)
       (send elem :serialize s)
       )
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; cartographer_ros_msgs/HistogramBucket[] _counts_by_bucket
     (write-long (length _counts_by_bucket) s)
     (dolist (elem _counts_by_bucket)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; cartographer_ros_msgs/MetricLabel[] _labels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _labels (let (r) (dotimes (i n) (push (instance cartographer_ros_msgs::MetricLabel :init) r)) r))
     (dolist (elem- _labels)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; cartographer_ros_msgs/HistogramBucket[] _counts_by_bucket
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _counts_by_bucket (let (r) (dotimes (i n) (push (instance cartographer_ros_msgs::HistogramBucket :init) r)) r))
     (dolist (elem- _counts_by_bucket)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cartographer_ros_msgs::Metric :md5sum-) "94a6ea1c6ef245b483a220f6769c8e36")
(setf (get cartographer_ros_msgs::Metric :datatype-) "cartographer_ros_msgs/Metric")
(setf (get cartographer_ros_msgs::Metric :definition-)
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



(provide :cartographer_ros_msgs/Metric "94a6ea1c6ef245b483a220f6769c8e36")


