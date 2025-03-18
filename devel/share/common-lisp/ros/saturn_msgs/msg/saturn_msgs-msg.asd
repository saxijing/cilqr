
(cl:in-package :asdf)

(defsystem "saturn_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "ControlArray" :depends-on ("_package_ControlArray"))
    (:file "_package_ControlArray" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
    (:file "StateArray" :depends-on ("_package_StateArray"))
    (:file "_package_StateArray" :depends-on ("_package"))
  ))