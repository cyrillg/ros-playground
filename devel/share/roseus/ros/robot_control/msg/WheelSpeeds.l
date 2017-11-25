;; Auto-generated. Do not edit!


(when (boundp 'robot_control::WheelSpeeds)
  (if (not (find-package "ROBOT_CONTROL"))
    (make-package "ROBOT_CONTROL"))
  (shadow 'WheelSpeeds (find-package "ROBOT_CONTROL")))
(unless (find-package "ROBOT_CONTROL::WHEELSPEEDS")
  (make-package "ROBOT_CONTROL::WHEELSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude WheelSpeeds.msg.html


(defclass robot_control::WheelSpeeds
  :super ros::object
  :slots (_left _right ))

(defmethod robot_control::WheelSpeeds
  (:init
   (&key
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; float64 _left
    8
    ;; float64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robot_control::WheelSpeeds :md5sum-) "50c2436c38cded221d061b57126c4e40")
(setf (get robot_control::WheelSpeeds :datatype-) "robot_control/WheelSpeeds")
(setf (get robot_control::WheelSpeeds :definition-)
      "float64 left
float64 right

")



(provide :robot_control/WheelSpeeds "50c2436c38cded221d061b57126c4e40")


