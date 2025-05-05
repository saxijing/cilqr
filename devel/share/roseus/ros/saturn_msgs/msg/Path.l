;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::Path)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'Path (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::PATH")
  (make-package "SATURN_MSGS::PATH"))

(in-package "ROS")
;;//! \htmlinclude Path.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::Path
  :super ros::object
  :slots (_header _path ))

(defmethod saturn_msgs::Path
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:path __path) (let (r) (dotimes (i 0) (push (instance saturn_msgs::StateLite :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _path __path)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; saturn_msgs/StateLite[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; saturn_msgs/StateLite[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; saturn_msgs/StateLite[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance saturn_msgs::StateLite :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get saturn_msgs::Path :md5sum-) "5329b3c8e4d0d10f7d64d970ba2d2048")
(setf (get saturn_msgs::Path :datatype-) "saturn_msgs/Path")
(setf (get saturn_msgs::Path :definition-)
      "std_msgs/Header header

StateLite[] path

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



(provide :saturn_msgs/Path "5329b3c8e4d0d10f7d64d970ba2d2048")


