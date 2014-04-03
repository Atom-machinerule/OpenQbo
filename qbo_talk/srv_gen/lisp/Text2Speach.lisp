; Auto-generated. Do not edit!


(cl:in-package qbo_talk-srv)


;//! \htmlinclude Text2Speach-request.msg.html

(cl:defclass <Text2Speach-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass Text2Speach-request (<Text2Speach-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Text2Speach-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Text2Speach-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qbo_talk-srv:<Text2Speach-request> is deprecated: use qbo_talk-srv:Text2Speach-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Text2Speach-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qbo_talk-srv:command-val is deprecated.  Use qbo_talk-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Text2Speach-request>) ostream)
  "Serializes a message object of type '<Text2Speach-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Text2Speach-request>) istream)
  "Deserializes a message object of type '<Text2Speach-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Text2Speach-request>)))
  "Returns string type for a service object of type '<Text2Speach-request>"
  "qbo_talk/Text2SpeachRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speach-request)))
  "Returns string type for a service object of type 'Text2Speach-request"
  "qbo_talk/Text2SpeachRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Text2Speach-request>)))
  "Returns md5sum for a message object of type '<Text2Speach-request>"
  "2fb3aa2736d70ecbfc297d3d9b879661")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Text2Speach-request)))
  "Returns md5sum for a message object of type 'Text2Speach-request"
  "2fb3aa2736d70ecbfc297d3d9b879661")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Text2Speach-request>)))
  "Returns full string definition for message of type '<Text2Speach-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Text2Speach-request)))
  "Returns full string definition for message of type 'Text2Speach-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Text2Speach-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Text2Speach-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Text2Speach-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude Text2Speach-response.msg.html

(cl:defclass <Text2Speach-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Text2Speach-response (<Text2Speach-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Text2Speach-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Text2Speach-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qbo_talk-srv:<Text2Speach-response> is deprecated: use qbo_talk-srv:Text2Speach-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Text2Speach-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qbo_talk-srv:result-val is deprecated.  Use qbo_talk-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Text2Speach-response>) ostream)
  "Serializes a message object of type '<Text2Speach-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Text2Speach-response>) istream)
  "Deserializes a message object of type '<Text2Speach-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Text2Speach-response>)))
  "Returns string type for a service object of type '<Text2Speach-response>"
  "qbo_talk/Text2SpeachResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speach-response)))
  "Returns string type for a service object of type 'Text2Speach-response"
  "qbo_talk/Text2SpeachResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Text2Speach-response>)))
  "Returns md5sum for a message object of type '<Text2Speach-response>"
  "2fb3aa2736d70ecbfc297d3d9b879661")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Text2Speach-response)))
  "Returns md5sum for a message object of type 'Text2Speach-response"
  "2fb3aa2736d70ecbfc297d3d9b879661")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Text2Speach-response>)))
  "Returns full string definition for message of type '<Text2Speach-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Text2Speach-response)))
  "Returns full string definition for message of type 'Text2Speach-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Text2Speach-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Text2Speach-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Text2Speach-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Text2Speach)))
  'Text2Speach-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Text2Speach)))
  'Text2Speach-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speach)))
  "Returns string type for a service object of type '<Text2Speach>"
  "qbo_talk/Text2Speach")
