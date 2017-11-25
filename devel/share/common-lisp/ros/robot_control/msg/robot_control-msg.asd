
(cl:in-package :asdf)

(defsystem "robot_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelSpeeds" :depends-on ("_package_WheelSpeeds"))
    (:file "_package_WheelSpeeds" :depends-on ("_package"))
  ))