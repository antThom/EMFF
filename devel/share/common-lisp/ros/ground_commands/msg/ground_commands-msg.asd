
(cl:in-package :asdf)

(defsystem "ground_commands-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Commands" :depends-on ("_package_Commands"))
    (:file "_package_Commands" :depends-on ("_package"))
  ))