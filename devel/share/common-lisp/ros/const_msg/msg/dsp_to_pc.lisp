; Auto-generated. Do not edit!


(cl:in-package const_msg-msg)


;//! \htmlinclude dsp_to_pc.msg.html

(cl:defclass <dsp_to_pc> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (XDist
    :reader XDist
    :initarg :XDist
    :type cl:integer
    :initform 0)
   (YDist
    :reader YDist
    :initarg :YDist
    :type cl:integer
    :initform 0)
   (fAngle
    :reader fAngle
    :initarg :fAngle
    :type cl:float
    :initform 0.0)
   (Vx
    :reader Vx
    :initarg :Vx
    :type cl:fixnum
    :initform 0)
   (Vy
    :reader Vy
    :initarg :Vy
    :type cl:fixnum
    :initform 0)
   (dw
    :reader dw
    :initarg :dw
    :type cl:fixnum
    :initform 0)
   (RecData_State
    :reader RecData_State
    :initarg :RecData_State
    :type cl:fixnum
    :initform 0))
)

(cl:defclass dsp_to_pc (<dsp_to_pc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dsp_to_pc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dsp_to_pc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name const_msg-msg:<dsp_to_pc> is deprecated: use const_msg-msg:dsp_to_pc instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:Header-val is deprecated.  Use const_msg-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'XDist-val :lambda-list '(m))
(cl:defmethod XDist-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:XDist-val is deprecated.  Use const_msg-msg:XDist instead.")
  (XDist m))

(cl:ensure-generic-function 'YDist-val :lambda-list '(m))
(cl:defmethod YDist-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:YDist-val is deprecated.  Use const_msg-msg:YDist instead.")
  (YDist m))

(cl:ensure-generic-function 'fAngle-val :lambda-list '(m))
(cl:defmethod fAngle-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:fAngle-val is deprecated.  Use const_msg-msg:fAngle instead.")
  (fAngle m))

(cl:ensure-generic-function 'Vx-val :lambda-list '(m))
(cl:defmethod Vx-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:Vx-val is deprecated.  Use const_msg-msg:Vx instead.")
  (Vx m))

(cl:ensure-generic-function 'Vy-val :lambda-list '(m))
(cl:defmethod Vy-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:Vy-val is deprecated.  Use const_msg-msg:Vy instead.")
  (Vy m))

(cl:ensure-generic-function 'dw-val :lambda-list '(m))
(cl:defmethod dw-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:dw-val is deprecated.  Use const_msg-msg:dw instead.")
  (dw m))

(cl:ensure-generic-function 'RecData_State-val :lambda-list '(m))
(cl:defmethod RecData_State-val ((m <dsp_to_pc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:RecData_State-val is deprecated.  Use const_msg-msg:RecData_State instead.")
  (RecData_State m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dsp_to_pc>) ostream)
  "Serializes a message object of type '<dsp_to_pc>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'XDist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'YDist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'Vx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Vy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RecData_State)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dsp_to_pc>) istream)
  "Deserializes a message object of type '<dsp_to_pc>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'XDist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'YDist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fAngle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Vx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Vy) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dw) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RecData_State)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dsp_to_pc>)))
  "Returns string type for a message object of type '<dsp_to_pc>"
  "const_msg/dsp_to_pc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dsp_to_pc)))
  "Returns string type for a message object of type 'dsp_to_pc"
  "const_msg/dsp_to_pc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dsp_to_pc>)))
  "Returns md5sum for a message object of type '<dsp_to_pc>"
  "d5bfc4d9d173fac3e9f429b7263af1d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dsp_to_pc)))
  "Returns md5sum for a message object of type 'dsp_to_pc"
  "d5bfc4d9d173fac3e9f429b7263af1d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dsp_to_pc>)))
  "Returns full string definition for message of type '<dsp_to_pc>"
  (cl:format cl:nil "Header Header~%int32 XDist~%int32 YDist~%float32 fAngle~%int16 Vx~%int16 Vy~%int16 dw~%uint8 RecData_State~% ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dsp_to_pc)))
  "Returns full string definition for message of type 'dsp_to_pc"
  (cl:format cl:nil "Header Header~%int32 XDist~%int32 YDist~%float32 fAngle~%int16 Vx~%int16 Vy~%int16 dw~%uint8 RecData_State~% ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dsp_to_pc>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     4
     4
     4
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dsp_to_pc>))
  "Converts a ROS message object to a list"
  (cl:list 'dsp_to_pc
    (cl:cons ':Header (Header msg))
    (cl:cons ':XDist (XDist msg))
    (cl:cons ':YDist (YDist msg))
    (cl:cons ':fAngle (fAngle msg))
    (cl:cons ':Vx (Vx msg))
    (cl:cons ':Vy (Vy msg))
    (cl:cons ':dw (dw msg))
    (cl:cons ':RecData_State (RecData_State msg))
))
