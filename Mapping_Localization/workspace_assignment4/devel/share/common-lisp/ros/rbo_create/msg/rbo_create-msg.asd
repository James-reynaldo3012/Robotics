
(cl:in-package :asdf)

(defsystem "rbo_create-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SensorPacket" :depends-on ("_package_SensorPacket"))
    (:file "_package_SensorPacket" :depends-on ("_package"))
  ))