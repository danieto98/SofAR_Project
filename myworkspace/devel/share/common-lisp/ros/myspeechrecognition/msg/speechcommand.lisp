; Auto-generated. Do not edit!


(cl:in-package myspeechrecognition-msg)


;//! \htmlinclude speechcommand.msg.html

(cl:defclass <speechcommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (argument
    :reader argument
    :initarg :argument
    :type cl:string
    :initform ""))
)

(cl:defclass speechcommand (<speechcommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speechcommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speechcommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name myspeechrecognition-msg:<speechcommand> is deprecated: use myspeechrecognition-msg:speechcommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <speechcommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader myspeechrecognition-msg:header-val is deprecated.  Use myspeechrecognition-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <speechcommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader myspeechrecognition-msg:mode-val is deprecated.  Use myspeechrecognition-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'argument-val :lambda-list '(m))
(cl:defmethod argument-val ((m <speechcommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader myspeechrecognition-msg:argument-val is deprecated.  Use myspeechrecognition-msg:argument instead.")
  (argument m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speechcommand>) ostream)
  "Serializes a message object of type '<speechcommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'argument))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'argument))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speechcommand>) istream)
  "Deserializes a message object of type '<speechcommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'argument) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'argument) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speechcommand>)))
  "Returns string type for a message object of type '<speechcommand>"
  "myspeechrecognition/speechcommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speechcommand)))
  "Returns string type for a message object of type 'speechcommand"
  "myspeechrecognition/speechcommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speechcommand>)))
  "Returns md5sum for a message object of type '<speechcommand>"
  "4f47b16802a5c9e8b62fe293298cf8f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speechcommand)))
  "Returns md5sum for a message object of type 'speechcommand"
  "4f47b16802a5c9e8b62fe293298cf8f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speechcommand>)))
  "Returns full string definition for message of type '<speechcommand>"
  (cl:format cl:nil "Header header~%string mode~%string argument~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speechcommand)))
  "Returns full string definition for message of type 'speechcommand"
  (cl:format cl:nil "Header header~%string mode~%string argument~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speechcommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'mode))
     4 (cl:length (cl:slot-value msg 'argument))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speechcommand>))
  "Converts a ROS message object to a list"
  (cl:list 'speechcommand
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':argument (argument msg))
))
