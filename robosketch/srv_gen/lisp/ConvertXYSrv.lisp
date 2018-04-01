; Auto-generated. Do not edit!


(cl:in-package robosketch-srv)


;//! \htmlinclude ConvertXYSrv-request.msg.html

(cl:defclass <ConvertXYSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConvertXYSrv-request (<ConvertXYSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertXYSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertXYSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robosketch-srv:<ConvertXYSrv-request> is deprecated: use robosketch-srv:ConvertXYSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertXYSrv-request>) ostream)
  "Serializes a message object of type '<ConvertXYSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertXYSrv-request>) istream)
  "Deserializes a message object of type '<ConvertXYSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertXYSrv-request>)))
  "Returns string type for a service object of type '<ConvertXYSrv-request>"
  "robosketch/ConvertXYSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertXYSrv-request)))
  "Returns string type for a service object of type 'ConvertXYSrv-request"
  "robosketch/ConvertXYSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertXYSrv-request>)))
  "Returns md5sum for a message object of type '<ConvertXYSrv-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertXYSrv-request)))
  "Returns md5sum for a message object of type 'ConvertXYSrv-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertXYSrv-request>)))
  "Returns full string definition for message of type '<ConvertXYSrv-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertXYSrv-request)))
  "Returns full string definition for message of type 'ConvertXYSrv-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertXYSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertXYSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertXYSrv-request
))
;//! \htmlinclude ConvertXYSrv-response.msg.html

(cl:defclass <ConvertXYSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConvertXYSrv-response (<ConvertXYSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertXYSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertXYSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robosketch-srv:<ConvertXYSrv-response> is deprecated: use robosketch-srv:ConvertXYSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertXYSrv-response>) ostream)
  "Serializes a message object of type '<ConvertXYSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertXYSrv-response>) istream)
  "Deserializes a message object of type '<ConvertXYSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertXYSrv-response>)))
  "Returns string type for a service object of type '<ConvertXYSrv-response>"
  "robosketch/ConvertXYSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertXYSrv-response)))
  "Returns string type for a service object of type 'ConvertXYSrv-response"
  "robosketch/ConvertXYSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertXYSrv-response>)))
  "Returns md5sum for a message object of type '<ConvertXYSrv-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertXYSrv-response)))
  "Returns md5sum for a message object of type 'ConvertXYSrv-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertXYSrv-response>)))
  "Returns full string definition for message of type '<ConvertXYSrv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertXYSrv-response)))
  "Returns full string definition for message of type 'ConvertXYSrv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertXYSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertXYSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertXYSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConvertXYSrv)))
  'ConvertXYSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConvertXYSrv)))
  'ConvertXYSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertXYSrv)))
  "Returns string type for a service object of type '<ConvertXYSrv>"
  "robosketch/ConvertXYSrv")