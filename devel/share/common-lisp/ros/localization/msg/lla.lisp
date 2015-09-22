; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude lla.msg.html

(cl:defclass <lla> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (LLA
    :reader LLA
    :initarg :LLA
    :type geographic_msgs-msg:GeoPoint
    :initform (cl:make-instance 'geographic_msgs-msg:GeoPoint)))
)

(cl:defclass lla (<lla>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lla>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lla)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<lla> is deprecated: use localization-msg:lla instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <lla>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'LLA-val :lambda-list '(m))
(cl:defmethod LLA-val ((m <lla>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:LLA-val is deprecated.  Use localization-msg:LLA instead.")
  (LLA m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lla>) ostream)
  "Serializes a message object of type '<lla>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'LLA) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lla>) istream)
  "Deserializes a message object of type '<lla>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'LLA) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lla>)))
  "Returns string type for a message object of type '<lla>"
  "localization/lla")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lla)))
  "Returns string type for a message object of type 'lla"
  "localization/lla")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lla>)))
  "Returns md5sum for a message object of type '<lla>"
  "eae2c6a941ba3cf090b75f129f755c94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lla)))
  "Returns md5sum for a message object of type 'lla"
  "eae2c6a941ba3cf090b75f129f755c94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lla>)))
  "Returns full string definition for message of type '<lla>"
  (cl:format cl:nil "std_msgs/Header header~%geographic_msgs/GeoPoint LLA~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lla)))
  "Returns full string definition for message of type 'lla"
  (cl:format cl:nil "std_msgs/Header header~%geographic_msgs/GeoPoint LLA~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lla>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'LLA))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lla>))
  "Converts a ROS message object to a list"
  (cl:list 'lla
    (cl:cons ':header (header msg))
    (cl:cons ':LLA (LLA msg))
))
