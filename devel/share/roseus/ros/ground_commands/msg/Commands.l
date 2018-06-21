;; Auto-generated. Do not edit!


(when (boundp 'ground_commands::Commands)
  (if (not (find-package "GROUND_COMMANDS"))
    (make-package "GROUND_COMMANDS"))
  (shadow 'Commands (find-package "GROUND_COMMANDS")))
(unless (find-package "GROUND_COMMANDS::COMMANDS")
  (make-package "GROUND_COMMANDS::COMMANDS"))

(in-package "ROS")
;;//! \htmlinclude Commands.msg.html


(defclass ground_commands::Commands
  :super ros::object
  :slots (_X _Y _Z _X_Dot _Y_Dot _Z_Dot _Psi _Theta _Phi _Psi_Dot _Theta_Dot _Phi_Dot _Mag_X _Mag_Y _Mag_Z ))

(defmethod ground_commands::Commands
  (:init
   (&key
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:Z __Z) 0.0)
    ((:X_Dot __X_Dot) 0.0)
    ((:Y_Dot __Y_Dot) 0.0)
    ((:Z_Dot __Z_Dot) 0.0)
    ((:Psi __Psi) 0.0)
    ((:Theta __Theta) 0.0)
    ((:Phi __Phi) 0.0)
    ((:Psi_Dot __Psi_Dot) 0.0)
    ((:Theta_Dot __Theta_Dot) 0.0)
    ((:Phi_Dot __Phi_Dot) 0.0)
    ((:Mag_X __Mag_X) 0.0)
    ((:Mag_Y __Mag_Y) 0.0)
    ((:Mag_Z __Mag_Z) 0.0)
    )
   (send-super :init)
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _Z (float __Z))
   (setq _X_Dot (float __X_Dot))
   (setq _Y_Dot (float __Y_Dot))
   (setq _Z_Dot (float __Z_Dot))
   (setq _Psi (float __Psi))
   (setq _Theta (float __Theta))
   (setq _Phi (float __Phi))
   (setq _Psi_Dot (float __Psi_Dot))
   (setq _Theta_Dot (float __Theta_Dot))
   (setq _Phi_Dot (float __Phi_Dot))
   (setq _Mag_X (float __Mag_X))
   (setq _Mag_Y (float __Mag_Y))
   (setq _Mag_Z (float __Mag_Z))
   self)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:Z
   (&optional __Z)
   (if __Z (setq _Z __Z)) _Z)
  (:X_Dot
   (&optional __X_Dot)
   (if __X_Dot (setq _X_Dot __X_Dot)) _X_Dot)
  (:Y_Dot
   (&optional __Y_Dot)
   (if __Y_Dot (setq _Y_Dot __Y_Dot)) _Y_Dot)
  (:Z_Dot
   (&optional __Z_Dot)
   (if __Z_Dot (setq _Z_Dot __Z_Dot)) _Z_Dot)
  (:Psi
   (&optional __Psi)
   (if __Psi (setq _Psi __Psi)) _Psi)
  (:Theta
   (&optional __Theta)
   (if __Theta (setq _Theta __Theta)) _Theta)
  (:Phi
   (&optional __Phi)
   (if __Phi (setq _Phi __Phi)) _Phi)
  (:Psi_Dot
   (&optional __Psi_Dot)
   (if __Psi_Dot (setq _Psi_Dot __Psi_Dot)) _Psi_Dot)
  (:Theta_Dot
   (&optional __Theta_Dot)
   (if __Theta_Dot (setq _Theta_Dot __Theta_Dot)) _Theta_Dot)
  (:Phi_Dot
   (&optional __Phi_Dot)
   (if __Phi_Dot (setq _Phi_Dot __Phi_Dot)) _Phi_Dot)
  (:Mag_X
   (&optional __Mag_X)
   (if __Mag_X (setq _Mag_X __Mag_X)) _Mag_X)
  (:Mag_Y
   (&optional __Mag_Y)
   (if __Mag_Y (setq _Mag_Y __Mag_Y)) _Mag_Y)
  (:Mag_Z
   (&optional __Mag_Z)
   (if __Mag_Z (setq _Mag_Z __Mag_Z)) _Mag_Z)
  (:serialization-length
   ()
   (+
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float64 _Z
    8
    ;; float64 _X_Dot
    8
    ;; float64 _Y_Dot
    8
    ;; float64 _Z_Dot
    8
    ;; float64 _Psi
    8
    ;; float64 _Theta
    8
    ;; float64 _Phi
    8
    ;; float64 _Psi_Dot
    8
    ;; float64 _Theta_Dot
    8
    ;; float64 _Phi_Dot
    8
    ;; float64 _Mag_X
    8
    ;; float64 _Mag_Y
    8
    ;; float64 _Mag_Z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z
       (sys::poke _Z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _X_Dot
       (sys::poke _X_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y_Dot
       (sys::poke _Y_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z_Dot
       (sys::poke _Z_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Psi
       (sys::poke _Psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Theta
       (sys::poke _Theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Phi
       (sys::poke _Phi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Psi_Dot
       (sys::poke _Psi_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Theta_Dot
       (sys::poke _Theta_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Phi_Dot
       (sys::poke _Phi_Dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Mag_X
       (sys::poke _Mag_X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Mag_Y
       (sys::poke _Mag_Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Mag_Z
       (sys::poke _Mag_Z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z
     (setq _Z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _X_Dot
     (setq _X_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y_Dot
     (setq _Y_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z_Dot
     (setq _Z_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Psi
     (setq _Psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Theta
     (setq _Theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Phi
     (setq _Phi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Psi_Dot
     (setq _Psi_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Theta_Dot
     (setq _Theta_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Phi_Dot
     (setq _Phi_Dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Mag_X
     (setq _Mag_X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Mag_Y
     (setq _Mag_Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Mag_Z
     (setq _Mag_Z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ground_commands::Commands :md5sum-) "4d98a8c001152cf3f66becd92df597c9")
(setf (get ground_commands::Commands :datatype-) "ground_commands/Commands")
(setf (get ground_commands::Commands :definition-)
      "float64 X
float64 Y
float64 Z
float64 X_Dot
float64 Y_Dot
float64 Z_Dot
float64 Psi
float64 Theta
float64 Phi
float64 Psi_Dot
float64 Theta_Dot
float64 Phi_Dot
float64 Mag_X
float64 Mag_Y
float64 Mag_Z
")



(provide :ground_commands/Commands "4d98a8c001152cf3f66becd92df597c9")


