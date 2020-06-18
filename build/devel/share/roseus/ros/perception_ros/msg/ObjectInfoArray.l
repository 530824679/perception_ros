;; Auto-generated. Do not edit!


(when (boundp 'perception_ros::ObjectInfoArray)
  (if (not (find-package "PERCEPTION_ROS"))
    (make-package "PERCEPTION_ROS"))
  (shadow 'ObjectInfoArray (find-package "PERCEPTION_ROS")))
(unless (find-package "PERCEPTION_ROS::OBJECTINFOARRAY")
  (make-package "PERCEPTION_ROS::OBJECTINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectInfoArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass perception_ros::ObjectInfoArray
  :super ros::object
  :slots (_header _object_info _object_num ))

(defmethod perception_ros::ObjectInfoArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:object_info __object_info) (let (r) (dotimes (i 0) (push (instance perception_ros::ObjectInfo :init) r)) r))
    ((:object_num __object_num) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _object_info __object_info)
   (setq _object_num (round __object_num))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:object_info
   (&rest __object_info)
   (if (keywordp (car __object_info))
       (send* _object_info __object_info)
     (progn
       (if __object_info (setq _object_info (car __object_info)))
       _object_info)))
  (:object_num
   (&optional __object_num)
   (if __object_num (setq _object_num __object_num)) _object_num)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; perception_ros/ObjectInfo[] _object_info
    (apply #'+ (send-all _object_info :serialization-length)) 4
    ;; uint16 _object_num
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; perception_ros/ObjectInfo[] _object_info
     (write-long (length _object_info) s)
     (dolist (elem _object_info)
       (send elem :serialize s)
       )
     ;; uint16 _object_num
       (write-word _object_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; perception_ros/ObjectInfo[] _object_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_info (let (r) (dotimes (i n) (push (instance perception_ros::ObjectInfo :init) r)) r))
     (dolist (elem- _object_info)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16 _object_num
     (setq _object_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get perception_ros::ObjectInfoArray :md5sum-) "37eb51ebc8f73aad80204a4eb7f1c89d")
(setf (get perception_ros::ObjectInfoArray :datatype-) "perception_ros/ObjectInfoArray")
(setf (get perception_ros::ObjectInfoArray :definition-)
      "Header                       header                 # The message of header

ObjectInfo[]                 object_info            # The information of object
uint16                       object_num             # The number of object
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

================================================================================
MSG: perception_ros/ObjectInfo
uint16                       id                 # The No. of object
uint8                        type 		        # The category of object
# unknow     0
# pedestrain 1
# motor      2
# car        3
# truck      4

uint8                        yaw                # The orientation angle of object
uint8                        confidence         # The confidence of object
uint16                       height             # The height of object
uint16                       width              # The width of object
uint16                       length             # The length of object
int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate
int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate
int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate
int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate
int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate
int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate
uint8                        motion_state       # The motion status of object
# unknow     0
# moving     1
# stationary 2




")



(provide :perception_ros/ObjectInfoArray "37eb51ebc8f73aad80204a4eb7f1c89d")


