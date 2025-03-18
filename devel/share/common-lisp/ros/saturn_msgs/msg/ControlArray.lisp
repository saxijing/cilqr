; Auto-generated. Do not edit!


(cl:in-package saturn_msgs-msg)


;//! \htmlinclude ControlArray.msg.html

(cl:defclass <ControlArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (control_lst
    :reader control_lst
    :initarg :control_lst
    :type (cl:vector saturn_msgs-msg:Control)
   :initform (cl:make-array 0 :element-type 'saturn_msgs-msg:Control :initial-element (cl:make-instance 'saturn_msgs-msg:Control))))
)

(cl:defclass ControlArray (<ControlArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name saturn_msgs-msg:<ControlArray> is deprecated: use saturn_msgs-msg:ControlArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:header-val is deprecated.  Use saturn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'control_lst-val :lambda-list '(m))
(cl:defmethod control_lst-val ((m <ControlArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:control_lst-val is deprecated.  Use saturn_msgs-msg:control_lst instead.")
  (control_lst m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlArray>) ostream)
  "Serializes a message object of type '<ControlArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'control_lst))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'control_lst))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlArray>) istream)
  "Deserializes a message object of type '<ControlArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'control_lst) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'control_lst)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'saturn_msgs-msg:Control))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlArray>)))
  "Returns string type for a message object of type '<ControlArray>"
  "saturn_msgs/ControlArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlArray)))
  "Returns string type for a message object of type 'ControlArray"
  "saturn_msgs/ControlArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlArray>)))
  "Returns md5sum for a message object of type '<ControlArray>"
  "26a69d081791f88c3703af6f5549edf4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlArray)))
  "Returns md5sum for a message object of type 'ControlArray"
  "26a69d081791f88c3703af6f5549edf4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlArray>)))
  "Returns full string definition for message of type '<ControlArray>"
  (cl:format cl:nil "std_msgs/Header header~%~%Control[] control_lst~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/Control~%std_msgs/Header header~%~%float64 u_accel~%float64 u_yawrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlArray)))
  "Returns full string definition for message of type 'ControlArray"
  (cl:format cl:nil "std_msgs/Header header~%~%Control[] control_lst~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/Control~%std_msgs/Header header~%~%float64 u_accel~%float64 u_yawrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'control_lst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlArray
    (cl:cons ':header (header msg))
    (cl:cons ':control_lst (control_lst msg))
))
