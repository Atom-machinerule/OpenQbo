
(cl:in-package :asdf)

(defsystem "qbo_talk-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Text2Speach" :depends-on ("_package_Text2Speach"))
    (:file "_package_Text2Speach" :depends-on ("_package"))
  ))
