# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/motion/ServoTarget.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/motion/ServoTarget.proto',
  package='message.motion',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n message/motion/ServoTarget.proto\x12\x0emessage.motion\x1a\x1fgoogle/protobuf/timestamp.proto\"s\n\x0bServoTarget\x12(\n\x04time\x18\x01 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\n\n\x02id\x18\x02 \x01(\r\x12\x10\n\x08position\x18\x03 \x01(\x02\x12\x0c\n\x04gain\x18\x04 \x01(\x02\x12\x0e\n\x06torque\x18\x05 \x01(\x02\"<\n\x0cServoTargets\x12,\n\x07targets\x18\x01 \x03(\x0b\x32\x1b.message.motion.ServoTargetb\x06proto3')
  ,
  dependencies=[google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,])




_SERVOTARGET = _descriptor.Descriptor(
  name='ServoTarget',
  full_name='message.motion.ServoTarget',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time', full_name='message.motion.ServoTarget.time', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='message.motion.ServoTarget.id', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='position', full_name='message.motion.ServoTarget.position', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gain', full_name='message.motion.ServoTarget.gain', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='torque', full_name='message.motion.ServoTarget.torque', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=85,
  serialized_end=200,
)


_SERVOTARGETS = _descriptor.Descriptor(
  name='ServoTargets',
  full_name='message.motion.ServoTargets',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='targets', full_name='message.motion.ServoTargets.targets', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=202,
  serialized_end=262,
)

_SERVOTARGET.fields_by_name['time'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_SERVOTARGETS.fields_by_name['targets'].message_type = _SERVOTARGET
DESCRIPTOR.message_types_by_name['ServoTarget'] = _SERVOTARGET
DESCRIPTOR.message_types_by_name['ServoTargets'] = _SERVOTARGETS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ServoTarget = _reflection.GeneratedProtocolMessageType('ServoTarget', (_message.Message,), dict(
  DESCRIPTOR = _SERVOTARGET,
  __module__ = 'message.motion.ServoTarget_pb2'
  # @@protoc_insertion_point(class_scope:message.motion.ServoTarget)
  ))
_sym_db.RegisterMessage(ServoTarget)

ServoTargets = _reflection.GeneratedProtocolMessageType('ServoTargets', (_message.Message,), dict(
  DESCRIPTOR = _SERVOTARGETS,
  __module__ = 'message.motion.ServoTarget_pb2'
  # @@protoc_insertion_point(class_scope:message.motion.ServoTargets)
  ))
_sym_db.RegisterMessage(ServoTargets)


# @@protoc_insertion_point(module_scope)
