# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/behaviour/FieldTarget.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/behaviour/FieldTarget.proto',
  package='message.behaviour',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n#message/behaviour/FieldTarget.proto\x12\x11message.behaviour\"l\n\x0b\x46ieldTarget\x12\x35\n\x06target\x18\x01 \x01(\x0e\x32%.message.behaviour.FieldTarget.Target\"&\n\x06Target\x12\x08\n\x04SELF\x10\x00\x12\x08\n\x04\x42\x41LL\x10\x01\x12\x08\n\x04GOAL\x10\x02\x62\x06proto3')
)



_FIELDTARGET_TARGET = _descriptor.EnumDescriptor(
  name='Target',
  full_name='message.behaviour.FieldTarget.Target',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SELF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BALL', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GOAL', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=128,
  serialized_end=166,
)
_sym_db.RegisterEnumDescriptor(_FIELDTARGET_TARGET)


_FIELDTARGET = _descriptor.Descriptor(
  name='FieldTarget',
  full_name='message.behaviour.FieldTarget',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='target', full_name='message.behaviour.FieldTarget.target', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _FIELDTARGET_TARGET,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=58,
  serialized_end=166,
)

_FIELDTARGET.fields_by_name['target'].enum_type = _FIELDTARGET_TARGET
_FIELDTARGET_TARGET.containing_type = _FIELDTARGET
DESCRIPTOR.message_types_by_name['FieldTarget'] = _FIELDTARGET
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FieldTarget = _reflection.GeneratedProtocolMessageType('FieldTarget', (_message.Message,), dict(
  DESCRIPTOR = _FIELDTARGET,
  __module__ = 'message.behaviour.FieldTarget_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.FieldTarget)
  ))
_sym_db.RegisterMessage(FieldTarget)


# @@protoc_insertion_point(module_scope)
