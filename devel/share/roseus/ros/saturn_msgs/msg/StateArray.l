;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::StateArray)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'StateArray (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::STATEARRAY")
  (make-package "SATURN_MSGS::STATEARRAY"))

(in-package "ROS")
;;//! \htmlinclude StateArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::StateArray
  :super ros::object
  :slots (_header _states ))

(defmethod saturn_msgs::StateArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:states __states) (let (r) (dotimes (i 0) (push (instance saturn_msgs::State :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _states __states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; saturn_msgs/State[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; saturn_msgs/State[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; saturn_msgs/State[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance saturn_msgs::State :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get saturn_msgs::StateArray :md5sum-) "75ced35a227c4e9c4cf1357d48b4ee7e")
(setf (get saturn_msgs::StateArray :datatype-) "saturn_msgs/StateArray")
(setf (get saturn_msgs::StateArray :definition-)
      "std_msgs/Header header

State[] states

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
MSG: saturn_msgs/State
std_msgs/Header header

int32 id
string name
float64 x
float64 y
float64 theta
float64 v
float64 accel
float64 yawrate

")



(provide :saturn_msgs/StateArray "75ced35a227c4e9c4cf1357d48b4ee7e")


