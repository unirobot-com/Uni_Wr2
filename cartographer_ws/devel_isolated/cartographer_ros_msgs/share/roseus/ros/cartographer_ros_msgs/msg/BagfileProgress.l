;; Auto-generated. Do not edit!


(when (boundp 'cartographer_ros_msgs::BagfileProgress)
  (if (not (find-package "CARTOGRAPHER_ROS_MSGS"))
    (make-package "CARTOGRAPHER_ROS_MSGS"))
  (shadow 'BagfileProgress (find-package "CARTOGRAPHER_ROS_MSGS")))
(unless (find-package "CARTOGRAPHER_ROS_MSGS::BAGFILEPROGRESS")
  (make-package "CARTOGRAPHER_ROS_MSGS::BAGFILEPROGRESS"))

(in-package "ROS")
;;//! \htmlinclude BagfileProgress.msg.html


(defclass cartographer_ros_msgs::BagfileProgress
  :super ros::object
  :slots (_current_bagfile_name _current_bagfile_id _total_bagfiles _total_messages _processed_messages _total_seconds _processed_seconds ))

(defmethod cartographer_ros_msgs::BagfileProgress
  (:init
   (&key
    ((:current_bagfile_name __current_bagfile_name) "")
    ((:current_bagfile_id __current_bagfile_id) 0)
    ((:total_bagfiles __total_bagfiles) 0)
    ((:total_messages __total_messages) 0)
    ((:processed_messages __processed_messages) 0)
    ((:total_seconds __total_seconds) 0.0)
    ((:processed_seconds __processed_seconds) 0.0)
    )
   (send-super :init)
   (setq _current_bagfile_name (string __current_bagfile_name))
   (setq _current_bagfile_id (round __current_bagfile_id))
   (setq _total_bagfiles (round __total_bagfiles))
   (setq _total_messages (round __total_messages))
   (setq _processed_messages (round __processed_messages))
   (setq _total_seconds (float __total_seconds))
   (setq _processed_seconds (float __processed_seconds))
   self)
  (:current_bagfile_name
   (&optional __current_bagfile_name)
   (if __current_bagfile_name (setq _current_bagfile_name __current_bagfile_name)) _current_bagfile_name)
  (:current_bagfile_id
   (&optional __current_bagfile_id)
   (if __current_bagfile_id (setq _current_bagfile_id __current_bagfile_id)) _current_bagfile_id)
  (:total_bagfiles
   (&optional __total_bagfiles)
   (if __total_bagfiles (setq _total_bagfiles __total_bagfiles)) _total_bagfiles)
  (:total_messages
   (&optional __total_messages)
   (if __total_messages (setq _total_messages __total_messages)) _total_messages)
  (:processed_messages
   (&optional __processed_messages)
   (if __processed_messages (setq _processed_messages __processed_messages)) _processed_messages)
  (:total_seconds
   (&optional __total_seconds)
   (if __total_seconds (setq _total_seconds __total_seconds)) _total_seconds)
  (:processed_seconds
   (&optional __processed_seconds)
   (if __processed_seconds (setq _processed_seconds __processed_seconds)) _processed_seconds)
  (:serialization-length
   ()
   (+
    ;; string _current_bagfile_name
    4 (length _current_bagfile_name)
    ;; uint32 _current_bagfile_id
    4
    ;; uint32 _total_bagfiles
    4
    ;; uint32 _total_messages
    4
    ;; uint32 _processed_messages
    4
    ;; float32 _total_seconds
    4
    ;; float32 _processed_seconds
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _current_bagfile_name
       (write-long (length _current_bagfile_name) s) (princ _current_bagfile_name s)
     ;; uint32 _current_bagfile_id
       (write-long _current_bagfile_id s)
     ;; uint32 _total_bagfiles
       (write-long _total_bagfiles s)
     ;; uint32 _total_messages
       (write-long _total_messages s)
     ;; uint32 _processed_messages
       (write-long _processed_messages s)
     ;; float32 _total_seconds
       (sys::poke _total_seconds (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _processed_seconds
       (sys::poke _processed_seconds (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _current_bagfile_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _current_bagfile_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _current_bagfile_id
     (setq _current_bagfile_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _total_bagfiles
     (setq _total_bagfiles (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _total_messages
     (setq _total_messages (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _processed_messages
     (setq _processed_messages (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _total_seconds
     (setq _total_seconds (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _processed_seconds
     (setq _processed_seconds (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cartographer_ros_msgs::BagfileProgress :md5sum-) "2a36f93b13e2b297d45098a38cb00510")
(setf (get cartographer_ros_msgs::BagfileProgress :datatype-) "cartographer_ros_msgs/BagfileProgress")
(setf (get cartographer_ros_msgs::BagfileProgress :definition-)
      "#
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


# Contains general information about the bagfiles processing progress

string current_bagfile_name
uint32 current_bagfile_id
uint32 total_bagfiles
uint32 total_messages
uint32 processed_messages
float32 total_seconds
float32 processed_seconds

")



(provide :cartographer_ros_msgs/BagfileProgress "2a36f93b13e2b297d45098a38cb00510")


