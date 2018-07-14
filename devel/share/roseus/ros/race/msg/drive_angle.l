;; Auto-generated. Do not edit!


(when (boundp 'race::drive_angle)
  (if (not (find-package "RACE"))
    (make-package "RACE"))
  (shadow 'drive_angle (find-package "RACE")))
(unless (find-package "RACE::DRIVE_ANGLE")
  (make-package "RACE::DRIVE_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude drive_angle.msg.html


(defclass race::drive_angle
  :super ros::object
  :slots (_angle ))

(defmethod race::drive_angle
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get race::drive_angle :md5sum-) "2d11dcdbe5a6f73dd324353dc52315ab")
(setf (get race::drive_angle :datatype-) "race/drive_angle")
(setf (get race::drive_angle :definition-)
      "float32 angle
")



(provide :race/drive_angle "2d11dcdbe5a6f73dd324353dc52315ab")


