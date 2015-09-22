
(cl:in-package :asdf)

(defsystem "localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geographic_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "roboteq_msg" :depends-on ("_package_roboteq_msg"))
    (:file "_package_roboteq_msg" :depends-on ("_package"))
    (:file "lp" :depends-on ("_package_lp"))
    (:file "_package_lp" :depends-on ("_package"))
    (:file "lla" :depends-on ("_package_lla"))
    (:file "_package_lla" :depends-on ("_package"))
  ))