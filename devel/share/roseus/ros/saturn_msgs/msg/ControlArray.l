;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::ControlArray)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'ControlArray (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::CONTROLARRAY")
  (make-package "SATURN_MSGS::CONTROLARRAY"))

(in-package "ROS")
;;//! \htmlinclude ControlArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::ControlArray
  :super ros::object
  :slots (_header _control_lst ))

(defmethod saturn_msgs::ControlArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:control_lst __control_lst) (let (r) (dotimes (i 0) (push (instance saturn_msgs::Control :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _control_lst __control_lst)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:control_lst
   (&rest __control_lst)
   (if (keywordp (car __control_lst))
       (send* _control_lst __control_lst)
     (progn
       (if __control_lst (setq _control_lst (car __control_lst)))
       _control_lst)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; saturn_msgs/Control[] _control_lst
    (apply #'+ (send-all _control_lst :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; saturn_msgs/Control[] _control_lst
     (write-long (length _control_lst) s)
     (dolist (elem _control_lst)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; saturn_msgs/Control[] _control_lst
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _control_lst (let (r) (dotimes (i n) (push (instance saturn_msgs::Control :init) r)) r))
     (dolist (elem- _control_lst)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get saturn_msgs::ControlArray :md5sum-) "26a69d081791f88c3703af6f5549edf4")
(setf (get saturn_msgs::ControlArray :datatype-) "saturn_msgs/ControlArray")
(setf (get saturn_msgs::ControlArray :definition-)
      "std_msgs/Header header

Control[] control_lst

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
MSG: saturn_msgs/Control
std_msgs/Header header

float64 u_accel
float64 u_yawrate

")



(provide :saturn_msgs/ControlArray "26a69d081791f88c3703af6f5549edf4")


