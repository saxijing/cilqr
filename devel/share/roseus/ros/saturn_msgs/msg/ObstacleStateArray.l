;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::ObstacleStateArray)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'ObstacleStateArray (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::OBSTACLESTATEARRAY")
  (make-package "SATURN_MSGS::OBSTACLESTATEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObstacleStateArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::ObstacleStateArray
  :super ros::object
  :slots (_header _obstacles ))

(defmethod saturn_msgs::ObstacleStateArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance saturn_msgs::ObstacleState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; saturn_msgs/ObstacleState[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; saturn_msgs/ObstacleState[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; saturn_msgs/ObstacleState[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance saturn_msgs::ObstacleState :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get saturn_msgs::ObstacleStateArray :md5sum-) "c25d44797a7f0424f6e9ee3a9b560ae5")
(setf (get saturn_msgs::ObstacleStateArray :datatype-) "saturn_msgs/ObstacleStateArray")
(setf (get saturn_msgs::ObstacleStateArray :definition-)
      "std_msgs/Header header

ObstacleState[] obstacles

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
MSG: saturn_msgs/ObstacleState
std_msgs/Header header

int32 id
string name
StateLite[] predicted_states

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



(provide :saturn_msgs/ObstacleStateArray "c25d44797a7f0424f6e9ee3a9b560ae5")


