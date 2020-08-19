;; Auto-generated. Do not edit!


(when (boundp 'const_msg::object_param)
  (if (not (find-package "CONST_MSG"))
    (make-package "CONST_MSG"))
  (shadow 'object_param (find-package "CONST_MSG")))
(unless (find-package "CONST_MSG::OBJECT_PARAM")
  (make-package "CONST_MSG::OBJECT_PARAM"))

(in-package "ROS")
;;//! \htmlinclude object_param.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass const_msg::object_param
  :super ros::object
  :slots (_Header _obj_angle _obj_distance ))

(defmethod const_msg::object_param
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:obj_angle __obj_angle) 0.0)
    ((:obj_distance __obj_distance) 0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _obj_angle (float __obj_angle))
   (setq _obj_distance (round __obj_distance))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:obj_angle
   (&optional __obj_angle)
   (if __obj_angle (setq _obj_angle __obj_angle)) _obj_angle)
  (:obj_distance
   (&optional __obj_distance)
   (if __obj_distance (setq _obj_distance __obj_distance)) _obj_distance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; float64 _obj_angle
    8
    ;; int64 _obj_distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; float64 _obj_angle
       (sys::poke _obj_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _obj_distance
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _obj_distance (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _obj_distance) (= (length (_obj_distance . bv)) 2)) ;; bignum
              (write-long (ash (elt (_obj_distance . bv) 0) 0) s)
              (write-long (ash (elt (_obj_distance . bv) 1) -1) s))
             ((and (class _obj_distance) (= (length (_obj_distance . bv)) 1)) ;; big1
              (write-long (elt (_obj_distance . bv) 0) s)
              (write-long (if (>= _obj_distance 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _obj_distance s)(write-long (if (>= _obj_distance 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; float64 _obj_angle
     (setq _obj_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _obj_distance
#+(or :alpha :irix6 :x86_64)
      (setf _obj_distance (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _obj_distance (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get const_msg::object_param :md5sum-) "6cb701956f8e3917f84ab2b8286c0409")
(setf (get const_msg::object_param :datatype-) "const_msg/object_param")
(setf (get const_msg::object_param :definition-)
      "Header Header
float64 obj_angle
int64 obj_distance
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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :const_msg/object_param "6cb701956f8e3917f84ab2b8286c0409")


