;; Auto-generated. Do not edit!


(when (boundp 'const_msg::dsp_to_pc)
  (if (not (find-package "CONST_MSG"))
    (make-package "CONST_MSG"))
  (shadow 'dsp_to_pc (find-package "CONST_MSG")))
(unless (find-package "CONST_MSG::DSP_TO_PC")
  (make-package "CONST_MSG::DSP_TO_PC"))

(in-package "ROS")
;;//! \htmlinclude dsp_to_pc.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass const_msg::dsp_to_pc
  :super ros::object
  :slots (_Header _XDist _YDist _fAngle _Vx _Vy _dw _RecData_State ))

(defmethod const_msg::dsp_to_pc
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:XDist __XDist) 0)
    ((:YDist __YDist) 0)
    ((:fAngle __fAngle) 0.0)
    ((:Vx __Vx) 0)
    ((:Vy __Vy) 0)
    ((:dw __dw) 0)
    ((:RecData_State __RecData_State) 0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _XDist (round __XDist))
   (setq _YDist (round __YDist))
   (setq _fAngle (float __fAngle))
   (setq _Vx (round __Vx))
   (setq _Vy (round __Vy))
   (setq _dw (round __dw))
   (setq _RecData_State (round __RecData_State))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:XDist
   (&optional __XDist)
   (if __XDist (setq _XDist __XDist)) _XDist)
  (:YDist
   (&optional __YDist)
   (if __YDist (setq _YDist __YDist)) _YDist)
  (:fAngle
   (&optional __fAngle)
   (if __fAngle (setq _fAngle __fAngle)) _fAngle)
  (:Vx
   (&optional __Vx)
   (if __Vx (setq _Vx __Vx)) _Vx)
  (:Vy
   (&optional __Vy)
   (if __Vy (setq _Vy __Vy)) _Vy)
  (:dw
   (&optional __dw)
   (if __dw (setq _dw __dw)) _dw)
  (:RecData_State
   (&optional __RecData_State)
   (if __RecData_State (setq _RecData_State __RecData_State)) _RecData_State)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; int32 _XDist
    4
    ;; int32 _YDist
    4
    ;; float32 _fAngle
    4
    ;; int16 _Vx
    2
    ;; int16 _Vy
    2
    ;; int16 _dw
    2
    ;; uint8 _RecData_State
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; int32 _XDist
       (write-long _XDist s)
     ;; int32 _YDist
       (write-long _YDist s)
     ;; float32 _fAngle
       (sys::poke _fAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _Vx
       (write-word _Vx s)
     ;; int16 _Vy
       (write-word _Vy s)
     ;; int16 _dw
       (write-word _dw s)
     ;; uint8 _RecData_State
       (write-byte _RecData_State s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; int32 _XDist
     (setq _XDist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _YDist
     (setq _YDist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _fAngle
     (setq _fAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _Vx
     (setq _Vx (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _Vy
     (setq _Vy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dw
     (setq _dw (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _RecData_State
     (setq _RecData_State (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get const_msg::dsp_to_pc :md5sum-) "d5bfc4d9d173fac3e9f429b7263af1d7")
(setf (get const_msg::dsp_to_pc :datatype-) "const_msg/dsp_to_pc")
(setf (get const_msg::dsp_to_pc :definition-)
      "Header Header
int32 XDist
int32 YDist
float32 fAngle
int16 Vx
int16 Vy
int16 dw
uint8 RecData_State
 
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



(provide :const_msg/dsp_to_pc "d5bfc4d9d173fac3e9f429b7263af1d7")


