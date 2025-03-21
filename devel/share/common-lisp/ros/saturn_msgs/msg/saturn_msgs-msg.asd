
(cl:in-package :asdf)

(defsystem "saturn_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "ControlArray" :depends-on ("_package_ControlArray"))
    (:file "_package_ControlArray" :depends-on ("_package"))
    (:file "ObstacleState" :depends-on ("_package_ObstacleState"))
    (:file "_package_ObstacleState" :depends-on ("_package"))
    (:file "ObstacleStateArray" :depends-on ("_package_ObstacleStateArray"))
    (:file "_package_ObstacleStateArray" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
    (:file "StateLite" :depends-on ("_package_StateLite"))
    (:file "_package_StateLite" :depends-on ("_package"))
  ))