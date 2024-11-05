;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::TrajectoryStates)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'TrajectoryStates (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES")
  (make-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ACTIVE*" (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(shadow '*ACTIVE* (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(defconstant cartographer_ros_msgs::TrajectoryStates::*ACTIVE* 0)
(intern "*FINISHED*" (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(shadow '*FINISHED* (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(defconstant cartographer_ros_msgs::TrajectoryStates::*FINISHED* 1)
(intern "*FROZEN*" (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(shadow '*FROZEN* (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(defconstant cartographer_ros_msgs::TrajectoryStates::*FROZEN* 2)
(intern "*DELETED*" (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(shadow '*DELETED* (find-package "CARTOGRAPHER_ROS_MSGS::TRAJECTORYSTATES"))
(defconstant cartographer_ros_msgs::TrajectoryStates::*DELETED* 3)

(defun cartographer_ros_msgs::TrajectoryStates-to-symbol (const)
  (cond
        ((= const 0) 'cartographer_ros_msgs::TrajectoryStates::*ACTIVE*)
        ((= const 1) 'cartographer_ros_msgs::TrajectoryStates::*FINISHED*)
        ((= const 2) 'cartographer_ros_msgs::TrajectoryStates::*FROZEN*)
        ((= const 3) 'cartographer_ros_msgs::TrajectoryStates::*DELETED*)
        (t nil)))

(defclass cartographer_ros_msgs::TrajectoryStates
  :super ros::object
  :slots (_header _trajectory_id _trajectory_state ))

(defmethod cartographer_ros_msgs::TrajectoryStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trajectory_id __trajectory_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:trajectory_state __trajectory_state) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _trajectory_id __trajectory_id)
   (setq _trajectory_state __trajectory_state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trajectory_id
   (&optional __trajectory_id)
   (if __trajectory_id (setq _trajectory_id __trajectory_id)) _trajectory_id)
  (:trajectory_state
   (&optional __trajectory_state)
   (if __trajectory_state (setq _trajectory_state __trajectory_state)) _trajectory_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _trajectory_id
    (* 4    (length _trajectory_id)) 4
    ;; uint8[] _trajectory_state
    (* 1    (length _trajectory_state)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _trajectory_id
     (write-long (length _trajectory_id) s)
     (dotimes (i (length _trajectory_id))
       (write-long (elt _trajectory_id i) s)
       )
     ;; uint8[] _trajectory_state
     (write-long (length _trajectory_state) s)
     (princ _trajectory_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _trajectory_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _trajectory_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint8[] _trajectory_state
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _trajectory_state (make-array n :element-type :char))
     (replace _trajectory_state buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get cartographer_ros_msgs::TrajectoryStates :md5sum-) "85efdd795e95b57a59cb785ecb152345")
(setf (get cartographer_ros_msgs::TrajectoryStates :datatype-) "cartographer_ros_msgs/TrajectoryStates")
(setf (get cartographer_ros_msgs::TrajectoryStates :definition-)
      "# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

uint8 ACTIVE = 0
uint8 FINISHED = 1
uint8 FROZEN = 2
uint8 DELETED = 3

std_msgs/Header header
int32[] trajectory_id
uint8[] trajectory_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :cartographer_ros_msgs/TrajectoryStates "85efdd795e95b57a59cb785ecb152345")


