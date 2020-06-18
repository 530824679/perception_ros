; Auto-generated. Do not edit!


(cl:in-package perception_ros-msg)


;//! \htmlinclude ObjectInfoArray.msg.html

(cl:defclass <ObjectInfoArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object_info
    :reader object_info
    :initarg :object_info
    :type (cl:vector perception_ros-msg:ObjectInfo)
   :initform (cl:make-array 0 :element-type 'perception_ros-msg:ObjectInfo :initial-element (cl:make-instance 'perception_ros-msg:ObjectInfo)))
   (object_num
    :reader object_num
    :initarg :object_num
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ObjectInfoArray (<ObjectInfoArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectInfoArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectInfoArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name perception_ros-msg:<ObjectInfoArray> is deprecated: use perception_ros-msg:ObjectInfoArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:header-val is deprecated.  Use perception_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object_info-val :lambda-list '(m))
(cl:defmethod object_info-val ((m <ObjectInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:object_info-val is deprecated.  Use perception_ros-msg:object_info instead.")
  (object_info m))

(cl:ensure-generic-function 'object_num-val :lambda-list '(m))
(cl:defmethod object_num-val ((m <ObjectInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:object_num-val is deprecated.  Use perception_ros-msg:object_num instead.")
  (object_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectInfoArray>) ostream)
  "Serializes a message object of type '<ObjectInfoArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'object_info))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_num)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectInfoArray>) istream)
  "Deserializes a message object of type '<ObjectInfoArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'perception_ros-msg:ObjectInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_num)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectInfoArray>)))
  "Returns string type for a message object of type '<ObjectInfoArray>"
  "perception_ros/ObjectInfoArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectInfoArray)))
  "Returns string type for a message object of type 'ObjectInfoArray"
  "perception_ros/ObjectInfoArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectInfoArray>)))
  "Returns md5sum for a message object of type '<ObjectInfoArray>"
  "37eb51ebc8f73aad80204a4eb7f1c89d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectInfoArray)))
  "Returns md5sum for a message object of type 'ObjectInfoArray"
  "37eb51ebc8f73aad80204a4eb7f1c89d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectInfoArray>)))
  "Returns full string definition for message of type '<ObjectInfoArray>"
  (cl:format cl:nil "Header                       header                 # The message of header~%~%ObjectInfo[]                 object_info            # The information of object~%uint16                       object_num             # The number of object~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: perception_ros/ObjectInfo~%uint16                       id                 # The No. of object~%uint8                        type 		        # The category of object~%# unknow     0~%# pedestrain 1~%# motor      2~%# car        3~%# truck      4~%~%uint8                        yaw                # The orientation angle of object~%uint8                        confidence         # The confidence of object~%uint16                       height             # The height of object~%uint16                       width              # The width of object~%uint16                       length             # The length of object~%int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate~%int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate~%int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate~%int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate~%int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate~%int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate~%uint8                        motion_state       # The motion status of object~%# unknow     0~%# moving     1~%# stationary 2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectInfoArray)))
  "Returns full string definition for message of type 'ObjectInfoArray"
  (cl:format cl:nil "Header                       header                 # The message of header~%~%ObjectInfo[]                 object_info            # The information of object~%uint16                       object_num             # The number of object~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: perception_ros/ObjectInfo~%uint16                       id                 # The No. of object~%uint8                        type 		        # The category of object~%# unknow     0~%# pedestrain 1~%# motor      2~%# car        3~%# truck      4~%~%uint8                        yaw                # The orientation angle of object~%uint8                        confidence         # The confidence of object~%uint16                       height             # The height of object~%uint16                       width              # The width of object~%uint16                       length             # The length of object~%int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate~%int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate~%int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate~%int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate~%int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate~%int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate~%uint8                        motion_state       # The motion status of object~%# unknow     0~%# moving     1~%# stationary 2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectInfoArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectInfoArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectInfoArray
    (cl:cons ':header (header msg))
    (cl:cons ':object_info (object_info msg))
    (cl:cons ':object_num (object_num msg))
))
