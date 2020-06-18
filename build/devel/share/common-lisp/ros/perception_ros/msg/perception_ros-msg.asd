
(cl:in-package :asdf)

(defsystem "perception_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectInfo" :depends-on ("_package_ObjectInfo"))
    (:file "_package_ObjectInfo" :depends-on ("_package"))
    (:file "ObjectInfoArray" :depends-on ("_package_ObjectInfoArray"))
    (:file "_package_ObjectInfoArray" :depends-on ("_package"))
  ))