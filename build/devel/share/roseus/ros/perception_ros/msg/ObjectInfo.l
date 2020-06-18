;; Auto-generated. Do not edit!


(when (boundp 'perception_ros::ObjectInfo)
  (if (not (find-package "PERCEPTION_ROS"))
    (make-package "PERCEPTION_ROS"))
  (shadow 'ObjectInfo (find-package "PERCEPTION_ROS")))
(unless (find-package "PERCEPTION_ROS::OBJECTINFO")
  (make-package "PERCEPTION_ROS::OBJECTINFO"))

(in-package "ROS")
;;//! \htmlinclude ObjectInfo.msg.html


(defclass perception_ros::ObjectInfo
  :super ros::object
  :slots (_id _type _yaw _confidence _height _width _length _distance_xv _distance_yv _velocity_xv _velocity_yv _accelerate_xv _accelerate_yv _motion_state ))

(defmethod perception_ros::ObjectInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:type __type) 0)
    ((:yaw __yaw) 0)
    ((:confidence __confidence) 0)
    ((:height __height) 0)
    ((:width __width) 0)
    ((:length __length) 0)
    ((:distance_xv __distance_xv) 0)
    ((:distance_yv __distance_yv) 0)
    ((:velocity_xv __velocity_xv) 0)
    ((:velocity_yv __velocity_yv) 0)
    ((:accelerate_xv __accelerate_xv) 0)
    ((:accelerate_yv __accelerate_yv) 0)
    ((:motion_state __motion_state) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _yaw (round __yaw))
   (setq _confidence (round __confidence))
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _length (round __length))
   (setq _distance_xv (round __distance_xv))
   (setq _distance_yv (round __distance_yv))
   (setq _velocity_xv (round __velocity_xv))
   (setq _velocity_yv (round __velocity_yv))
   (setq _accelerate_xv (round __accelerate_xv))
   (setq _accelerate_yv (round __accelerate_yv))
   (setq _motion_state (round __motion_state))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:distance_xv
   (&optional __distance_xv)
   (if __distance_xv (setq _distance_xv __distance_xv)) _distance_xv)
  (:distance_yv
   (&optional __distance_yv)
   (if __distance_yv (setq _distance_yv __distance_yv)) _distance_yv)
  (:velocity_xv
   (&optional __velocity_xv)
   (if __velocity_xv (setq _velocity_xv __velocity_xv)) _velocity_xv)
  (:velocity_yv
   (&optional __velocity_yv)
   (if __velocity_yv (setq _velocity_yv __velocity_yv)) _velocity_yv)
  (:accelerate_xv
   (&optional __accelerate_xv)
   (if __accelerate_xv (setq _accelerate_xv __accelerate_xv)) _accelerate_xv)
  (:accelerate_yv
   (&optional __accelerate_yv)
   (if __accelerate_yv (setq _accelerate_yv __accelerate_yv)) _accelerate_yv)
  (:motion_state
   (&optional __motion_state)
   (if __motion_state (setq _motion_state __motion_state)) _motion_state)
  (:serialization-length
   ()
   (+
    ;; uint16 _id
    2
    ;; uint8 _type
    1
    ;; uint8 _yaw
    1
    ;; uint8 _confidence
    1
    ;; uint16 _height
    2
    ;; uint16 _width
    2
    ;; uint16 _length
    2
    ;; int16 _distance_xv
    2
    ;; int16 _distance_yv
    2
    ;; int16 _velocity_xv
    2
    ;; int16 _velocity_yv
    2
    ;; int16 _accelerate_xv
    2
    ;; int16 _accelerate_yv
    2
    ;; uint8 _motion_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _id
       (write-word _id s)
     ;; uint8 _type
       (write-byte _type s)
     ;; uint8 _yaw
       (write-byte _yaw s)
     ;; uint8 _confidence
       (write-byte _confidence s)
     ;; uint16 _height
       (write-word _height s)
     ;; uint16 _width
       (write-word _width s)
     ;; uint16 _length
       (write-word _length s)
     ;; int16 _distance_xv
       (write-word _distance_xv s)
     ;; int16 _distance_yv
       (write-word _distance_yv s)
     ;; int16 _velocity_xv
       (write-word _velocity_xv s)
     ;; int16 _velocity_yv
       (write-word _velocity_yv s)
     ;; int16 _accelerate_xv
       (write-word _accelerate_xv s)
     ;; int16 _accelerate_yv
       (write-word _accelerate_yv s)
     ;; uint8 _motion_state
       (write-byte _motion_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _yaw
     (setq _yaw (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _confidence
     (setq _confidence (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _height
     (setq _height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _length
     (setq _length (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _distance_xv
     (setq _distance_xv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _distance_yv
     (setq _distance_yv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _velocity_xv
     (setq _velocity_xv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _velocity_yv
     (setq _velocity_yv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _accelerate_xv
     (setq _accelerate_xv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _accelerate_yv
     (setq _accelerate_yv (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _motion_state
     (setq _motion_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get perception_ros::ObjectInfo :md5sum-) "a84a83544761a25875cca8bb70b78846")
(setf (get perception_ros::ObjectInfo :datatype-) "perception_ros/ObjectInfo")
(setf (get perception_ros::ObjectInfo :definition-)
      "uint16                       id                 # The No. of object
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



(provide :perception_ros/ObjectInfo "a84a83544761a25875cca8bb70b78846")


