; Auto-generated. Do not edit!


(cl:in-package perception_ros-msg)


;//! \htmlinclude ObjectInfo.msg.html

(cl:defclass <ObjectInfo> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (distance_xv
    :reader distance_xv
    :initarg :distance_xv
    :type cl:fixnum
    :initform 0)
   (distance_yv
    :reader distance_yv
    :initarg :distance_yv
    :type cl:fixnum
    :initform 0)
   (velocity_xv
    :reader velocity_xv
    :initarg :velocity_xv
    :type cl:fixnum
    :initform 0)
   (velocity_yv
    :reader velocity_yv
    :initarg :velocity_yv
    :type cl:fixnum
    :initform 0)
   (accelerate_xv
    :reader accelerate_xv
    :initarg :accelerate_xv
    :type cl:fixnum
    :initform 0)
   (accelerate_yv
    :reader accelerate_yv
    :initarg :accelerate_yv
    :type cl:fixnum
    :initform 0)
   (motion_state
    :reader motion_state
    :initarg :motion_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ObjectInfo (<ObjectInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name perception_ros-msg:<ObjectInfo> is deprecated: use perception_ros-msg:ObjectInfo instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:id-val is deprecated.  Use perception_ros-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:type-val is deprecated.  Use perception_ros-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:yaw-val is deprecated.  Use perception_ros-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:confidence-val is deprecated.  Use perception_ros-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:height-val is deprecated.  Use perception_ros-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:width-val is deprecated.  Use perception_ros-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:length-val is deprecated.  Use perception_ros-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'distance_xv-val :lambda-list '(m))
(cl:defmethod distance_xv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:distance_xv-val is deprecated.  Use perception_ros-msg:distance_xv instead.")
  (distance_xv m))

(cl:ensure-generic-function 'distance_yv-val :lambda-list '(m))
(cl:defmethod distance_yv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:distance_yv-val is deprecated.  Use perception_ros-msg:distance_yv instead.")
  (distance_yv m))

(cl:ensure-generic-function 'velocity_xv-val :lambda-list '(m))
(cl:defmethod velocity_xv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:velocity_xv-val is deprecated.  Use perception_ros-msg:velocity_xv instead.")
  (velocity_xv m))

(cl:ensure-generic-function 'velocity_yv-val :lambda-list '(m))
(cl:defmethod velocity_yv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:velocity_yv-val is deprecated.  Use perception_ros-msg:velocity_yv instead.")
  (velocity_yv m))

(cl:ensure-generic-function 'accelerate_xv-val :lambda-list '(m))
(cl:defmethod accelerate_xv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:accelerate_xv-val is deprecated.  Use perception_ros-msg:accelerate_xv instead.")
  (accelerate_xv m))

(cl:ensure-generic-function 'accelerate_yv-val :lambda-list '(m))
(cl:defmethod accelerate_yv-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:accelerate_yv-val is deprecated.  Use perception_ros-msg:accelerate_yv instead.")
  (accelerate_yv m))

(cl:ensure-generic-function 'motion_state-val :lambda-list '(m))
(cl:defmethod motion_state-val ((m <ObjectInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perception_ros-msg:motion_state-val is deprecated.  Use perception_ros-msg:motion_state instead.")
  (motion_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectInfo>) ostream)
  "Serializes a message object of type '<ObjectInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yaw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'distance_xv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance_yv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity_xv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity_yv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'accelerate_xv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'accelerate_yv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motion_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectInfo>) istream)
  "Deserializes a message object of type '<ObjectInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yaw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_xv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance_yv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity_xv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity_yv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'accelerate_xv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'accelerate_yv) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motion_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectInfo>)))
  "Returns string type for a message object of type '<ObjectInfo>"
  "perception_ros/ObjectInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectInfo)))
  "Returns string type for a message object of type 'ObjectInfo"
  "perception_ros/ObjectInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectInfo>)))
  "Returns md5sum for a message object of type '<ObjectInfo>"
  "a84a83544761a25875cca8bb70b78846")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectInfo)))
  "Returns md5sum for a message object of type 'ObjectInfo"
  "a84a83544761a25875cca8bb70b78846")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectInfo>)))
  "Returns full string definition for message of type '<ObjectInfo>"
  (cl:format cl:nil "uint16                       id                 # The No. of object~%uint8                        type 		        # The category of object~%# unknow     0~%# pedestrain 1~%# motor      2~%# car        3~%# truck      4~%~%uint8                        yaw                # The orientation angle of object~%uint8                        confidence         # The confidence of object~%uint16                       height             # The height of object~%uint16                       width              # The width of object~%uint16                       length             # The length of object~%int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate~%int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate~%int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate~%int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate~%int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate~%int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate~%uint8                        motion_state       # The motion status of object~%# unknow     0~%# moving     1~%# stationary 2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectInfo)))
  "Returns full string definition for message of type 'ObjectInfo"
  (cl:format cl:nil "uint16                       id                 # The No. of object~%uint8                        type 		        # The category of object~%# unknow     0~%# pedestrain 1~%# motor      2~%# car        3~%# truck      4~%~%uint8                        yaw                # The orientation angle of object~%uint8                        confidence         # The confidence of object~%uint16                       height             # The height of object~%uint16                       width              # The width of object~%uint16                       length             # The length of object~%int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate~%int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate~%int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate~%int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate~%int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate~%int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate~%uint8                        motion_state       # The motion status of object~%# unknow     0~%# moving     1~%# stationary 2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectInfo>))
  (cl:+ 0
     2
     1
     1
     1
     2
     2
     2
     2
     2
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectInfo
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':length (length msg))
    (cl:cons ':distance_xv (distance_xv msg))
    (cl:cons ':distance_yv (distance_yv msg))
    (cl:cons ':velocity_xv (velocity_xv msg))
    (cl:cons ':velocity_yv (velocity_yv msg))
    (cl:cons ':accelerate_xv (accelerate_xv msg))
    (cl:cons ':accelerate_yv (accelerate_yv msg))
    (cl:cons ':motion_state (motion_state msg))
))
