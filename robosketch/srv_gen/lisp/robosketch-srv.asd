
(cl:in-package :asdf)

(defsystem "robosketch-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ConvertXYSrv" :depends-on ("_package_ConvertXYSrv"))
    (:file "_package_ConvertXYSrv" :depends-on ("_package"))
  ))