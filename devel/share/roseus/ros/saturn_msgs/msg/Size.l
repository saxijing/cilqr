;; Auto-generated. Do not edit!


(when (boundp 'saturn_msgs::Size)
  (if (not (find-package "SATURN_MSGS"))
    (make-package "SATURN_MSGS"))
  (shadow 'Size (find-package "SATURN_MSGS")))
(unless (find-package "SATURN_MSGS::SIZE")
  (make-package "SATURN_MSGS::SIZE"))

(in-package "ROS")
;;//! \htmlinclude Size.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass saturn_msgs::Size
  :super ros::object
  :slots (_header _length _width _height _wheel_base _wheel_track ))

(defmethod saturn_msgs::Size
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:length __length) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    ((:wheel_base __wheel_base) 0.0)
    ((:wheel_track __wheel_track) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _length (float __length))
   (setq _width (float __width))
   (setq _height (float __height))
   (setq _wheel_base (float __wheel_base))
   (setq _wheel_track (float __wheel_track))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:wheel_base
   (&optional __wheel_base)
   (if __wheel_base (setq _wheel_base __wheel_base)) _wheel_base)
  (:wheel_track
   (&optional __wheel_track)
   (if __wheel_track (setq _wheel_track __wheel_track)) _wheel_track)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _length
    8
    ;; float64 _width
    8
    ;; float64 _height
    8
    ;; float64 _wheel_base
    8
    ;; float64 _wheel_track
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _length
       (sys::poke _length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wheel_base
       (sys::poke _wheel_base (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wheel_track
       (sys::poke _wheel_track (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _length
     (setq _length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wheel_base
     (setq _wheel_base (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wheel_track
     (setq _wheel_track (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get saturn_msgs::Size :md5sum-) "0122303aa1e5e98fff82adf25a489b8a")
(setf (get saturn_msgs::Size :datatype-) "saturn_msgs/Size")
(setf (get saturn_msgs::Size :definition-)
      "std_msgs/Header header

float64 length
float64 width
float64 height
float64 wheel_base
float64 wheel_track

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



(provide :saturn_msgs/Size "0122303aa1e5e98fff82adf25a489b8a")


