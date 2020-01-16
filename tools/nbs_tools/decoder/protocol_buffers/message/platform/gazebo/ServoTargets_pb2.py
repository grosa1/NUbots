# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/platform/gazebo/ServoTargets.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2
from message.motion import ServoTarget_pb2 as message_dot_motion_dot_ServoTarget__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/platform/gazebo/ServoTargets.proto',
  package='message.platform.gazebo',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n*message/platform/gazebo/ServoTargets.proto\x12\x17message.platform.gazebo\x1a\x1fgoogle/protobuf/timestamp.proto\x1a message/motion/ServoTarget.proto\"v\n\x0cServoTargets\x12\r\n\x05model\x18\x01 \x01(\t\x12(\n\x04time\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12-\n\x07targets\x18\x03 \x01(\x0b\x32\x1c.message.motion.ServoTargetsb\x06proto3')
  ,
  dependencies=[google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,message_dot_motion_dot_ServoTarget__pb2.DESCRIPTOR,])




_SERVOTARGETS = _descriptor.Descriptor(
  name='ServoTargets',
  full_name='message.platform.gazebo.ServoTargets',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='model', full_name='message.platform.gazebo.ServoTargets.model', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='time', full_name='message.platform.gazebo.ServoTargets.time', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='targets', full_name='message.platform.gazebo.ServoTargets.targets', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=138,
  serialized_end=256,
)

_SERVOTARGETS.fields_by_name['time'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_SERVOTARGETS.fields_by_name['targets'].message_type = message_dot_motion_dot_ServoTarget__pb2._SERVOTARGETS
DESCRIPTOR.message_types_by_name['ServoTargets'] = _SERVOTARGETS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ServoTargets = _reflection.GeneratedProtocolMessageType('ServoTargets', (_message.Message,), dict(
  DESCRIPTOR = _SERVOTARGETS,
  __module__ = 'message.platform.gazebo.ServoTargets_pb2'
  # @@protoc_insertion_point(class_scope:message.platform.gazebo.ServoTargets)
  ))
_sym_db.RegisterMessage(ServoTargets)


# @@protoc_insertion_point(module_scope)
