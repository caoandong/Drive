;; Auto-generated. Do not edit!


(when (boundp 'race::drive_speed)
  (if (not (find-package "RACE"))
    (make-package "RACE"))
  (shadow 'drive_speed (find-package "RACE")))
(unless (find-package "RACE::DRIVE_SPEED")
  (make-package "RACE::DRIVE_SPEED"))

(in-package "ROS")
;;//! \htmlinclude drive_speed.msg.html


(defclass race::drive_speed
  :super ros::object
  :slots (_speed ))

(defmethod race::drive_speed
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get race::drive_speed :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get race::drive_speed :datatype-) "race/drive_speed")
(setf (get race::drive_speed :definition-)
      "float32 speed
")



(provide :race/drive_speed "ca65bba734a79b4a6707341d829f4d5c")


