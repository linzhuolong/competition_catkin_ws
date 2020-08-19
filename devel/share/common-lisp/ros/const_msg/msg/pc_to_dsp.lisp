; Auto-generated. Do not edit!


(cl:in-package const_msg-msg)


;//! \htmlinclude pc_to_dsp.msg.html

(cl:defclass <pc_to_dsp> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (flag_start_stop
    :reader flag_start_stop
    :initarg :flag_start_stop
    :type cl:fixnum
    :initform 0)
   (V1
    :reader V1
    :initarg :V1
    :type cl:fixnum
    :initform 0)
   (V2
    :reader V2
    :initarg :V2
    :type cl:fixnum
    :initform 0)
   (V3
    :reader V3
    :initarg :V3
    :type cl:fixnum
    :initform 0)
   (V4
    :reader V4
    :initarg :V4
    :type cl:fixnum
    :initform 0)
   (SendData_State
    :reader SendData_State
    :initarg :SendData_State
    :type cl:fixnum
    :initform 0))
)

(cl:defclass pc_to_dsp (<pc_to_dsp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pc_to_dsp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pc_to_dsp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name const_msg-msg:<pc_to_dsp> is deprecated: use const_msg-msg:pc_to_dsp instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:Header-val is deprecated.  Use const_msg-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'flag_start_stop-val :lambda-list '(m))
(cl:defmethod flag_start_stop-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:flag_start_stop-val is deprecated.  Use const_msg-msg:flag_start_stop instead.")
  (flag_start_stop m))

(cl:ensure-generic-function 'V1-val :lambda-list '(m))
(cl:defmethod V1-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:V1-val is deprecated.  Use const_msg-msg:V1 instead.")
  (V1 m))

(cl:ensure-generic-function 'V2-val :lambda-list '(m))
(cl:defmethod V2-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:V2-val is deprecated.  Use const_msg-msg:V2 instead.")
  (V2 m))

(cl:ensure-generic-function 'V3-val :lambda-list '(m))
(cl:defmethod V3-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:V3-val is deprecated.  Use const_msg-msg:V3 instead.")
  (V3 m))

(cl:ensure-generic-function 'V4-val :lambda-list '(m))
(cl:defmethod V4-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:V4-val is deprecated.  Use const_msg-msg:V4 instead.")
  (V4 m))

(cl:ensure-generic-function 'SendData_State-val :lambda-list '(m))
(cl:defmethod SendData_State-val ((m <pc_to_dsp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader const_msg-msg:SendData_State-val is deprecated.  Use const_msg-msg:SendData_State instead.")
  (SendData_State m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pc_to_dsp>) ostream)
  "Serializes a message object of type '<pc_to_dsp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag_start_stop)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'V1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'V2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'V3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'V4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SendData_State)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pc_to_dsp>) istream)
  "Deserializes a message object of type '<pc_to_dsp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag_start_stop)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'V1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'V2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'V3) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'V4) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SendData_State)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pc_to_dsp>)))
  "Returns string type for a message object of type '<pc_to_dsp>"
  "const_msg/pc_to_dsp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pc_to_dsp)))
  "Returns string type for a message object of type 'pc_to_dsp"
  "const_msg/pc_to_dsp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pc_to_dsp>)))
  "Returns md5sum for a message object of type '<pc_to_dsp>"
  "72744243899e1bab7337fbf15a335ac5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pc_to_dsp)))
  "Returns md5sum for a message object of type 'pc_to_dsp"
  "72744243899e1bab7337fbf15a335ac5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pc_to_dsp>)))
  "Returns full string definition for message of type '<pc_to_dsp>"
  (cl:format cl:nil "Header Header~%uint8 flag_start_stop~%int16 V1~%int16 V2~%int16 V3~%int16 V4~%uint8 SendData_State~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pc_to_dsp)))
  "Returns full string definition for message of type 'pc_to_dsp"
  (cl:format cl:nil "Header Header~%uint8 flag_start_stop~%int16 V1~%int16 V2~%int16 V3~%int16 V4~%uint8 SendData_State~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pc_to_dsp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     1
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pc_to_dsp>))
  "Converts a ROS message object to a list"
  (cl:list 'pc_to_dsp
    (cl:cons ':Header (Header msg))
    (cl:cons ':flag_start_stop (flag_start_stop msg))
    (cl:cons ':V1 (V1 msg))
    (cl:cons ':V2 (V2 msg))
    (cl:cons ':V3 (V3 msg))
    (cl:cons ':V4 (V4 msg))
    (cl:cons ':SendData_State (SendData_State msg))
))
