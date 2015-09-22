; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude roboteq_msg.msg.html

(cl:defclass <roboteq_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rpm_1
    :reader rpm_1
    :initarg :rpm_1
    :type cl:integer
    :initform 0)
   (rpm_2
    :reader rpm_2
    :initarg :rpm_2
    :type cl:integer
    :initform 0)
   (encoder_1
    :reader encoder_1
    :initarg :encoder_1
    :type cl:integer
    :initform 0)
   (encoder_2
    :reader encoder_2
    :initarg :encoder_2
    :type cl:integer
    :initform 0)
   (mxrpm
    :reader mxrpm
    :initarg :mxrpm
    :type cl:fixnum
    :initform 0))
)

(cl:defclass roboteq_msg (<roboteq_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <roboteq_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'roboteq_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<roboteq_msg> is deprecated: use localization-msg:roboteq_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rpm_1-val :lambda-list '(m))
(cl:defmethod rpm_1-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:rpm_1-val is deprecated.  Use localization-msg:rpm_1 instead.")
  (rpm_1 m))

(cl:ensure-generic-function 'rpm_2-val :lambda-list '(m))
(cl:defmethod rpm_2-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:rpm_2-val is deprecated.  Use localization-msg:rpm_2 instead.")
  (rpm_2 m))

(cl:ensure-generic-function 'encoder_1-val :lambda-list '(m))
(cl:defmethod encoder_1-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:encoder_1-val is deprecated.  Use localization-msg:encoder_1 instead.")
  (encoder_1 m))

(cl:ensure-generic-function 'encoder_2-val :lambda-list '(m))
(cl:defmethod encoder_2-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:encoder_2-val is deprecated.  Use localization-msg:encoder_2 instead.")
  (encoder_2 m))

(cl:ensure-generic-function 'mxrpm-val :lambda-list '(m))
(cl:defmethod mxrpm-val ((m <roboteq_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:mxrpm-val is deprecated.  Use localization-msg:mxrpm instead.")
  (mxrpm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <roboteq_msg>) ostream)
  "Serializes a message object of type '<roboteq_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rpm_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rpm_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'encoder_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'encoder_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mxrpm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <roboteq_msg>) istream)
  "Deserializes a message object of type '<roboteq_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'encoder_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'encoder_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mxrpm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<roboteq_msg>)))
  "Returns string type for a message object of type '<roboteq_msg>"
  "localization/roboteq_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'roboteq_msg)))
  "Returns string type for a message object of type 'roboteq_msg"
  "localization/roboteq_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<roboteq_msg>)))
  "Returns md5sum for a message object of type '<roboteq_msg>"
  "a71ff46624c4d40ab790d3ae256f719d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'roboteq_msg)))
  "Returns md5sum for a message object of type 'roboteq_msg"
  "a71ff46624c4d40ab790d3ae256f719d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<roboteq_msg>)))
  "Returns full string definition for message of type '<roboteq_msg>"
  (cl:format cl:nil "# For Timestamp and Seq~%~%Header header~%~%~%# The readings of rpm of both wheels, and their rates of change~%~%int32 rpm_1~%int32 rpm_2~%~%~%# The encoder values coming from both wheels~%~%int32 encoder_1~%int32 encoder_2~%~%~%# MaxRPM~%~%int16 mxrpm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'roboteq_msg)))
  "Returns full string definition for message of type 'roboteq_msg"
  (cl:format cl:nil "# For Timestamp and Seq~%~%Header header~%~%~%# The readings of rpm of both wheels, and their rates of change~%~%int32 rpm_1~%int32 rpm_2~%~%~%# The encoder values coming from both wheels~%~%int32 encoder_1~%int32 encoder_2~%~%~%# MaxRPM~%~%int16 mxrpm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <roboteq_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <roboteq_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'roboteq_msg
    (cl:cons ':header (header msg))
    (cl:cons ':rpm_1 (rpm_1 msg))
    (cl:cons ':rpm_2 (rpm_2 msg))
    (cl:cons ':encoder_1 (encoder_1 msg))
    (cl:cons ':encoder_2 (encoder_2 msg))
    (cl:cons ':mxrpm (mxrpm msg))
))
