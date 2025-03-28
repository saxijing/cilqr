; Auto-generated. Do not edit!


(cl:in-package saturn_msgs-msg)


;//! \htmlinclude ObstacleStateArray.msg.html

(cl:defclass <ObstacleStateArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector saturn_msgs-msg:ObstacleState)
   :initform (cl:make-array 0 :element-type 'saturn_msgs-msg:ObstacleState :initial-element (cl:make-instance 'saturn_msgs-msg:ObstacleState))))
)

(cl:defclass ObstacleStateArray (<ObstacleStateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleStateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleStateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name saturn_msgs-msg:<ObstacleStateArray> is deprecated: use saturn_msgs-msg:ObstacleStateArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleStateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:header-val is deprecated.  Use saturn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <ObstacleStateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:obstacles-val is deprecated.  Use saturn_msgs-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleStateArray>) ostream)
  "Serializes a message object of type '<ObstacleStateArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleStateArray>) istream)
  "Deserializes a message object of type '<ObstacleStateArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'saturn_msgs-msg:ObstacleState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleStateArray>)))
  "Returns string type for a message object of type '<ObstacleStateArray>"
  "saturn_msgs/ObstacleStateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleStateArray)))
  "Returns string type for a message object of type 'ObstacleStateArray"
  "saturn_msgs/ObstacleStateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleStateArray>)))
  "Returns md5sum for a message object of type '<ObstacleStateArray>"
  "49f890b8acf390d2ac08943005e834a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleStateArray)))
  "Returns md5sum for a message object of type 'ObstacleStateArray"
  "49f890b8acf390d2ac08943005e834a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleStateArray>)))
  "Returns full string definition for message of type '<ObstacleStateArray>"
  (cl:format cl:nil "std_msgs/Header header~%~%ObstacleState[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/ObstacleState~%std_msgs/Header header~%~%int32 id~%string name~%StateLite[] predicted_states~%Size size~%~%================================================================================~%MSG: saturn_msgs/StateLite~%std_msgs/Header header~%~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%================================================================================~%MSG: saturn_msgs/Size~%std_msgs/Header header~%~%float64 length~%float64 width~%float64 height~%float64 wheel_base~%float64 wheel_track~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleStateArray)))
  "Returns full string definition for message of type 'ObstacleStateArray"
  (cl:format cl:nil "std_msgs/Header header~%~%ObstacleState[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/ObstacleState~%std_msgs/Header header~%~%int32 id~%string name~%StateLite[] predicted_states~%Size size~%~%================================================================================~%MSG: saturn_msgs/StateLite~%std_msgs/Header header~%~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%================================================================================~%MSG: saturn_msgs/Size~%std_msgs/Header header~%~%float64 length~%float64 width~%float64 height~%float64 wheel_base~%float64 wheel_track~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleStateArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleStateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleStateArray
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
