; Auto-generated. Do not edit!


(cl:in-package saturn_msgs-msg)


;//! \htmlinclude ObstacleState.msg.html

(cl:defclass <ObstacleState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (predicted_states
    :reader predicted_states
    :initarg :predicted_states
    :type (cl:vector saturn_msgs-msg:StateLite)
   :initform (cl:make-array 0 :element-type 'saturn_msgs-msg:StateLite :initial-element (cl:make-instance 'saturn_msgs-msg:StateLite)))
   (size
    :reader size
    :initarg :size
    :type saturn_msgs-msg:Size
    :initform (cl:make-instance 'saturn_msgs-msg:Size)))
)

(cl:defclass ObstacleState (<ObstacleState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name saturn_msgs-msg:<ObstacleState> is deprecated: use saturn_msgs-msg:ObstacleState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:header-val is deprecated.  Use saturn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:id-val is deprecated.  Use saturn_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:name-val is deprecated.  Use saturn_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'predicted_states-val :lambda-list '(m))
(cl:defmethod predicted_states-val ((m <ObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:predicted_states-val is deprecated.  Use saturn_msgs-msg:predicted_states instead.")
  (predicted_states m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <ObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader saturn_msgs-msg:size-val is deprecated.  Use saturn_msgs-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleState>) ostream)
  "Serializes a message object of type '<ObstacleState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'predicted_states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'predicted_states))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleState>) istream)
  "Deserializes a message object of type '<ObstacleState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'predicted_states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'predicted_states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'saturn_msgs-msg:StateLite))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleState>)))
  "Returns string type for a message object of type '<ObstacleState>"
  "saturn_msgs/ObstacleState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleState)))
  "Returns string type for a message object of type 'ObstacleState"
  "saturn_msgs/ObstacleState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleState>)))
  "Returns md5sum for a message object of type '<ObstacleState>"
  "206a275054569710e712258c18396ed6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleState)))
  "Returns md5sum for a message object of type 'ObstacleState"
  "206a275054569710e712258c18396ed6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleState>)))
  "Returns full string definition for message of type '<ObstacleState>"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 id~%string name~%StateLite[] predicted_states~%Size size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/StateLite~%std_msgs/Header header~%~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%================================================================================~%MSG: saturn_msgs/Size~%std_msgs/Header header~%~%float64 length~%float64 width~%float64 height~%float64 wheel_base~%float64 wheel_track~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleState)))
  "Returns full string definition for message of type 'ObstacleState"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 id~%string name~%StateLite[] predicted_states~%Size size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: saturn_msgs/StateLite~%std_msgs/Header header~%~%float64 x~%float64 y~%float64 theta~%float64 v~%float64 accel~%float64 yawrate~%~%================================================================================~%MSG: saturn_msgs/Size~%std_msgs/Header header~%~%float64 length~%float64 width~%float64 height~%float64 wheel_base~%float64 wheel_track~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'predicted_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleState>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleState
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':name (name msg))
    (cl:cons ':predicted_states (predicted_states msg))
    (cl:cons ':size (size msg))
))
