# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/behaviour/Nod.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/behaviour/Nod.proto',
  package='message.behaviour',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x1bmessage/behaviour/Nod.proto\x12\x11message.behaviour\"\x14\n\x03Nod\x12\r\n\x05value\x18\x01 \x01(\x08\x62\x06proto3')
)




_NOD = _descriptor.Descriptor(
  name='Nod',
  full_name='message.behaviour.Nod',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='value', full_name='message.behaviour.Nod.value', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=50,
  serialized_end=70,
)

DESCRIPTOR.message_types_by_name['Nod'] = _NOD
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Nod = _reflection.GeneratedProtocolMessageType('Nod', (_message.Message,), dict(
  DESCRIPTOR = _NOD,
  __module__ = 'message.behaviour.Nod_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.Nod)
  ))
_sym_db.RegisterMessage(Nod)


# @@protoc_insertion_point(module_scope)
