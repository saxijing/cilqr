;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::ObstacleState)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'ObstacleState (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::OBSTACLESTATE")
  (make-package "SATURN_MSGS::OBSTACLESTATE"))

(in-package "ROS")
;;//! \htmlinclude ObstacleState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::ObstacleState
  :super ros::object
  :slots (_header _id _name _predicted_states ))

(defmethod saturn_msgs::ObstacleState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:name __name) "")
    ((:predicted_states __predicted_states) (let (r) (dotimes (i 0) (push (instance saturn_msgs::StateLite :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _name (string __name))
   (setq _predicted_states __predicted_states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:predicted_states
   (&rest __predicted_states)
   (if (keywordp (car __predicted_states))
       (send* _predicted_states __predicted_states)
     (progn
       (if __predicted_states (setq _predicted_states (car __predicted_states)))
       _predicted_states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; string _name
    4 (length _name)
    ;; saturn_msgs/StateLite[] _predicted_states
    (apply #'+ (send-all _predicted_states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; saturn_msgs/StateLite[] _predicted_states
     (write-long (length _predicted_states) s)
     (dolist (elem _predicted_states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; saturn_msgs/StateLite[] _predicted_states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _predicted_states (let (r) (dotimes (i n) (push (instance saturn_msgs::StateLite :init) r)) r))
     (dolist (elem- _predicted_states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get saturn_msgs::ObstacleState :md5sum-) "d45e6ca6695835598d9ff9ee662af118")
(setf (get saturn_msgs::ObstacleState :datatype-) "saturn_msgs/ObstacleState")
(setf (get saturn_msgs::ObstacleState :definition-)
      "std_msgs/Header header

int32 id
string name
StateLite[] predicted_states

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

================================================================================
MSG: saturn_msgs/StateLite
std_msgs/Header header

float64 x
float64 y
float64 theta
float64 v
float64 accel
float64 yawrate

")



(provide :saturn_msgs/ObstacleState "d45e6ca6695835598d9ff9ee662af118")


