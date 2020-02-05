
(cl:in-package :asdf)

(defsystem "myspeechrecognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "speechcommand" :depends-on ("_package_speechcommand"))
    (:file "_package_speechcommand" :depends-on ("_package"))
  ))