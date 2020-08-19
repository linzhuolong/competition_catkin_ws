;; Auto-generated. Do not edit!


(when (boundp 'const_msg::pc_to_dsp)
  (if (not (find-package "CONST_MSG"))
    (make-package "CONST_MSG"))
  (shadow 'pc_to_dsp (find-package "CONST_MSG")))
(unless (find-package "CONST_MSG::PC_TO_DSP")
  (make-package "CONST_MSG::PC_TO_DSP"))

(in-package "ROS")
;;//! \htmlinclude pc_to_dsp.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass const_msg::pc_to_dsp
  :super ros::object
  :slots (_Header _flag_start_stop _V1 _V2 _V3 _V4 _SendData_State ))

(defmethod const_msg::pc_to_dsp
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:flag_start_stop __flag_start_stop) 0)
    ((:V1 __V1) 0)
    ((:V2 __V2) 0)
    ((:V3 __V3) 0)
    ((:V4 __V4) 0)
    ((:SendData_State __SendData_State) 0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _flag_start_stop (round __flag_start_stop))
   (setq _V1 (round __V1))
   (setq _V2 (round __V2))
   (setq _V3 (round __V3))
   (setq _V4 (round __V4))
   (setq _SendData_State (round __SendData_State))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:flag_start_stop
   (&optional __flag_start_stop)
   (if __flag_start_stop (setq _flag_start_stop __flag_start_stop)) _flag_start_stop)
  (:V1
   (&optional __V1)
   (if __V1 (setq _V1 __V1)) _V1)
  (:V2
   (&optional __V2)
   (if __V2 (setq _V2 __V2)) _V2)
  (:V3
   (&optional __V3)
   (if __V3 (setq _V3 __V3)) _V3)
  (:V4
   (&optional __V4)
   (if __V4 (setq _V4 __V4)) _V4)
  (:SendData_State
   (&optional __SendData_State)
   (if __SendData_State (setq _SendData_State __SendData_State)) _SendData_State)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; uint8 _flag_start_stop
    1
    ;; int16 _V1
    2
    ;; int16 _V2
    2
    ;; int16 _V3
    2
    ;; int16 _V4
    2
    ;; uint8 _SendData_State
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; uint8 _flag_start_stop
       (write-byte _flag_start_stop s)
     ;; int16 _V1
       (write-word _V1 s)
     ;; int16 _V2
       (write-word _V2 s)
     ;; int16 _V3
       (write-word _V3 s)
     ;; int16 _V4
       (write-word _V4 s)
     ;; uint8 _SendData_State
       (write-byte _SendData_State s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; uint8 _flag_start_stop
     (setq _flag_start_stop (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _V1
     (setq _V1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _V2
     (setq _V2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _V3
     (setq _V3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _V4
     (setq _V4 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _SendData_State
     (setq _SendData_State (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get const_msg::pc_to_dsp :md5sum-) "72744243899e1bab7337fbf15a335ac5")
(setf (get const_msg::pc_to_dsp :datatype-) "const_msg/pc_to_dsp")
(setf (get const_msg::pc_to_dsp :definition-)
      "Header Header
uint8 flag_start_stop
int16 V1
int16 V2
int16 V3
int16 V4
uint8 SendData_State

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



(provide :const_msg/pc_to_dsp "72744243899e1bab7337fbf15a335ac5")


