;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::MetricLabel)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'MetricLabel (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::METRICLABEL")
  (make-package "CARTOGRAPHER_ROS_MSGS::METRICLABEL"))

(in-package "ROS")
;;//! \htmlinclude MetricLabel.msg.html


(defclass cartographer_ros_msgs::MetricLabel
  :super ros::object
  :slots (_key _value ))

(defmethod cartographer_ros_msgs::MetricLabel
  (:init
   (&key
    ((:key __key) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _key (string __key))
   (setq _value (string __value))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cartographer_ros_msgs::MetricLabel :md5sum-) "cf57fdc6617a881a88c16e768132149c")
(setf (get cartographer_ros_msgs::MetricLabel :datatype-) "cartographer_ros_msgs/MetricLabel")
(setf (get cartographer_ros_msgs::MetricLabel :definition-)
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

string key
string value

")



(provide :cartographer_ros_msgs/MetricLabel "cf57fdc6617a881a88c16e768132149c")


