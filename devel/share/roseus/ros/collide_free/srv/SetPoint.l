;; Auto-generated. Do not edit!


(when (boundp 'collide_free::SetPoint)
  (if (not (find-package "COLLIDE_FREE"))
    (make-package "COLLIDE_FREE"))
  (shadow 'SetPoint (find-package "COLLIDE_FREE")))
(unless (find-package "COLLIDE_FREE::SETPOINT")
  (make-package "COLLIDE_FREE::SETPOINT"))
(unless (find-package "COLLIDE_FREE::SETPOINTREQUEST")
  (make-package "COLLIDE_FREE::SETPOINTREQUEST"))
(unless (find-package "COLLIDE_FREE::SETPOINTRESPONSE")
  (make-package "COLLIDE_FREE::SETPOINTRESPONSE"))

(in-package "ROS")





(defclass collide_free::SetPointRequest
  :super ros::object
  :slots (_pose ))

(defmethod collide_free::SetPointRequest
  (:init
   (&key
    ((:pose __pose) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:serialization-length
   ()
   (+
    ;; float64[] _pose
    (* 8    (length _pose)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _pose
     (write-long (length _pose) s)
     (dotimes (i (length _pose))
       (sys::poke (elt _pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass collide_free::SetPointResponse
  :super ros::object
  :slots (_wm _wh _t ))

(defmethod collide_free::SetPointResponse
  (:init
   (&key
    ((:wm __wm) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:wh __wh) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:t __t) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _wm __wm)
   (setq _wh __wh)
   (setq _t __t)
   self)
  (:wm
   (&optional __wm)
   (if __wm (setq _wm __wm)) _wm)
  (:wh
   (&optional __wh)
   (if __wh (setq _wh __wh)) _wh)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; float64[] _wm
    (* 8    (length _wm)) 4
    ;; float64[] _wh
    (* 8    (length _wh)) 4
    ;; float64[] _t
    (* 8    (length _t)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _wm
     (write-long (length _wm) s)
     (dotimes (i (length _wm))
       (sys::poke (elt _wm i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _wh
     (write-long (length _wh) s)
     (dotimes (i (length _wh))
       (sys::poke (elt _wh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _t
     (write-long (length _t) s)
     (dotimes (i (length _t))
       (sys::poke (elt _t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _wm
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wm (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wm i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _wh
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wh (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass collide_free::SetPoint
  :super ros::object
  :slots ())

(setf (get collide_free::SetPoint :md5sum-) "1d719385decc796c11ed1d75b56878fc")
(setf (get collide_free::SetPoint :datatype-) "collide_free/SetPoint")
(setf (get collide_free::SetPoint :request) collide_free::SetPointRequest)
(setf (get collide_free::SetPoint :response) collide_free::SetPointResponse)

(defmethod collide_free::SetPointRequest
  (:response () (instance collide_free::SetPointResponse :init)))

(setf (get collide_free::SetPointRequest :md5sum-) "1d719385decc796c11ed1d75b56878fc")
(setf (get collide_free::SetPointRequest :datatype-) "collide_free/SetPointRequest")
(setf (get collide_free::SetPointRequest :definition-)
      "float64[] pose
---
float64[] wm
float64[] wh
float64[] t

")

(setf (get collide_free::SetPointResponse :md5sum-) "1d719385decc796c11ed1d75b56878fc")
(setf (get collide_free::SetPointResponse :datatype-) "collide_free/SetPointResponse")
(setf (get collide_free::SetPointResponse :definition-)
      "float64[] pose
---
float64[] wm
float64[] wh
float64[] t

")



(provide :collide_free/SetPoint "1d719385decc796c11ed1d75b56878fc")


