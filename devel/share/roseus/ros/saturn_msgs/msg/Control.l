;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::Control)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'Control (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::CONTROL")
  (make-package "SATURN_MSGS::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::Control
  :super ros::object
  :slots (_header _u_accel _u_yawrate ))

(defmethod saturn_msgs::Control
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:u_accel __u_accel) 0.0)
    ((:u_yawrate __u_yawrate) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _u_accel (float __u_accel))
   (setq _u_yawrate (float __u_yawrate))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:u_accel
   (&optional __u_accel)
   (if __u_accel (setq _u_accel __u_accel)) _u_accel)
  (:u_yawrate
   (&optional __u_yawrate)
   (if __u_yawrate (setq _u_yawrate __u_yawrate)) _u_yawrate)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _u_accel
    8
    ;; float64 _u_yawrate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _u_accel
       (sys::poke _u_accel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u_yawrate
       (sys::poke _u_yawrate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _u_accel
     (setq _u_accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u_yawrate
     (setq _u_yawrate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get saturn_msgs::Control :md5sum-) "f42f5551e89d8349e570aae4222e3836")
(setf (get saturn_msgs::Control :datatype-) "saturn_msgs/Control")
(setf (get saturn_msgs::Control :definition-)
      "std_msgs/Header header

float64 u_accel
float64 u_yawrate

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



(provide :saturn_msgs/Control "f42f5551e89d8349e570aae4222e3836")


