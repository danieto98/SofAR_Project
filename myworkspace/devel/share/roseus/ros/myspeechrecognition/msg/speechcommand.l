;; Auto-generated. Do not edit!


(when (boundp 'myspeechrecognition::speechcommand)
  (if (not (find-package "MYSPEECHRECOGNITION"))
    (make-package "MYSPEECHRECOGNITION"))
  (shadow 'speechcommand (find-package "MYSPEECHRECOGNITION")))
(unless (find-package "MYSPEECHRECOGNITION::SPEECHCOMMAND")
  (make-package "MYSPEECHRECOGNITION::SPEECHCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude speechcommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass myspeechrecognition::speechcommand
  :super ros::object
  :slots (_header _mode _argument ))

(defmethod myspeechrecognition::speechcommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) "")
    ((:argument __argument) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode (string __mode))
   (setq _argument (string __argument))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:argument
   (&optional __argument)
   (if __argument (setq _argument __argument)) _argument)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _mode
    4 (length _mode)
    ;; string _argument
    4 (length _argument)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; string _argument
       (write-long (length _argument) s) (princ _argument s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _argument
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _argument (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get myspeechrecognition::speechcommand :md5sum-) "4f47b16802a5c9e8b62fe293298cf8f4")
(setf (get myspeechrecognition::speechcommand :datatype-) "myspeechrecognition/speechcommand")
(setf (get myspeechrecognition::speechcommand :definition-)
      "Header header
string mode
string argument

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
string frame_id

")



(provide :myspeechrecognition/speechcommand "4f47b16802a5c9e8b62fe293298cf8f4")


