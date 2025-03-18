; Auto-generated. Do not edit!


(cl:in-package saturn_msgs-msg)


;//! \htmlinclude StateArray.msg.html

(cl:defclass <StateArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (states
    :reader states
    :initarg :states
    :type (cl:vector saturn_msgs-msg:State)
   :initform (cl:make-array 0 :element-type 'saturn_msgs-msg:State :initial-element (cl:make-instance 'saturn_msgs-msg:State))))
)

(cl:defclass StateArray (<StateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name saturn_msgs-msg:<StateArray> is deprecated: use saturn_msgs-msg:StateArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:header-val is deprecated.  Use saturn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:states-val is deprecated.  Use saturn_msgs-msg:states instead.")
  (states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateArray>) ostream)
  "Serializes a message object of type '<StateArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateArray>) istream)
  "Deserializes a message object of type '<StateArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'saturn_msgs-msg:State))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateArray>)))
  "Returns string type for a message object of type '<StateArray>"
  "saturn_msgs/StateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateArray)))
  "Returns string type for a message object of type 'StateArray"
  "saturn_msgs/StateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateArray>)))
  "Returns md5sum for a message object of type '<StateArray>"
  "75ced35a227c4e9c4cf1357d48b4ee7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateArray)))
  "Returns md5sum for a message object of type 'StateArray"
  "75ced35a227c4e9c4cf1357d48b4ee7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateArray>)))
  "Returns full string definition for message of type '<StateArray>"
  (cl:format cl:nil "std_msgs/Header header~%~%State[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/State~%std_msgs/Header header~%~%int32 id~%string name~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateArray)))
  "Returns full string definition for message of type 'StateArray"
  (cl:format cl:nil "std_msgs/Header header~%~%State[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/State~%std_msgs/Header header~%~%int32 id~%string name~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'StateArray
    (cl:cons ':header (header msg))
    (cl:cons ':states (states msg))
))
