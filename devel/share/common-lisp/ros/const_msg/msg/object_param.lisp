; Auto-generated. Do not edit!


(cl:in-package const_msg-msg)


;//! \htmlinclude object_param.msg.html

(cl:defclass <object_param> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obj_angle
    :reader obj_angle
    :initarg :obj_angle
    :type cl:float
    :initform 0.0)
   (obj_distance
    :reader obj_distance
    :initarg :obj_distance
    :type cl:integer
    :initform 0))
)

(cl:defclass object_param (<object_param>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_param>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_param)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name const_msg-msg:<object_param> is deprecated: use const_msg-msg:object_param instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <object_param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:Header-val is deprecated.  Use const_msg-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'obj_angle-val :lambda-list '(m))
(cl:defmethod obj_angle-val ((m <object_param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:obj_angle-val is deprecated.  Use const_msg-msg:obj_angle instead.")
  (obj_angle m))

(cl:ensure-generic-function 'obj_distance-val :lambda-list '(m))
(cl:defmethod obj_distance-val ((m <object_param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:obj_distance-val is deprecated.  Use const_msg-msg:obj_distance instead.")
  (obj_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_param>) ostream)
  "Serializes a message object of type '<object_param>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obj_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'obj_distance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_param>) istream)
  "Deserializes a message object of type '<object_param>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obj_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'obj_distance) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_param>)))
  "Returns string type for a message object of type '<object_param>"
  "const_msg/object_param")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_param)))
  "Returns string type for a message object of type 'object_param"
  "const_msg/object_param")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_param>)))
  "Returns md5sum for a message object of type '<object_param>"
  "6cb701956f8e3917f84ab2b8286c0409")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_param)))
  "Returns md5sum for a message object of type 'object_param"
  "6cb701956f8e3917f84ab2b8286c0409")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_param>)))
  "Returns full string definition for message of type '<object_param>"
  (cl:format cl:nil "Header Header~%float64 obj_angle~%int64 obj_distance~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_param)))
  "Returns full string definition for message of type 'object_param"
  (cl:format cl:nil "Header Header~%float64 obj_angle~%int64 obj_distance~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_param>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_param>))
  "Converts a ROS message object to a list"
  (cl:list 'object_param
    (cl:cons ':Header (Header msg))
    (cl:cons ':obj_angle (obj_angle msg))
    (cl:cons ':obj_distance (obj_distance msg))
))
