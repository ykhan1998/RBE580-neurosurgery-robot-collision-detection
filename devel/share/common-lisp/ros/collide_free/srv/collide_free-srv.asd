
(cl:in-package :asdf)

(defsystem "collide_free-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetPoint" :depends-on ("_package_SetPoint"))
    (:file "_package_SetPoint" :depends-on ("_package"))
  ))