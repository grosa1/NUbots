# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/behaviour/FixedWalkCommand.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import duration_pb2 as google_dot_protobuf_dot_duration__pb2
import Vector_pb2 as Vector__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/behaviour/FixedWalkCommand.proto',
  package='message.behaviour',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n(message/behaviour/FixedWalkCommand.proto\x12\x11message.behaviour\x1a\x1egoogle/protobuf/duration.proto\x1a\x0cVector.proto\"\x13\n\x11\x46ixedWalkFinished\"\x11\n\x0fWalkConfigSaved\"\x11\n\x0f\x43\x61ncelFixedWalk\"*\n\x14WalkOptimiserCommand\x12\x12\n\nwalkConfig\x18\x01 \x01(\t\"\x80\x02\n\x10\x46ixedWalkCommand\x12\x41\n\x08segments\x18\x01 \x03(\x0b\x32/.message.behaviour.FixedWalkCommand.WalkSegment\x1a\xa8\x01\n\x0bWalkSegment\x12\x18\n\tdirection\x18\x01 \x01(\x0b\x32\x05.vec2\x12\x13\n\x0b\x63urvePeriod\x18\x02 \x01(\x01\x12\x1a\n\x12normalisedVelocity\x18\x03 \x01(\x01\x12!\n\x19normalisedAngularVelocity\x18\x04 \x01(\x01\x12+\n\x08\x64uration\x18\x05 \x01(\x0b\x32\x19.google.protobuf.Durationb\x06proto3')
  ,
  dependencies=[google_dot_protobuf_dot_duration__pb2.DESCRIPTOR,Vector__pb2.DESCRIPTOR,])




_FIXEDWALKFINISHED = _descriptor.Descriptor(
  name='FixedWalkFinished',
  full_name='message.behaviour.FixedWalkFinished',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=109,
  serialized_end=128,
)


_WALKCONFIGSAVED = _descriptor.Descriptor(
  name='WalkConfigSaved',
  full_name='message.behaviour.WalkConfigSaved',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=130,
  serialized_end=147,
)


_CANCELFIXEDWALK = _descriptor.Descriptor(
  name='CancelFixedWalk',
  full_name='message.behaviour.CancelFixedWalk',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=149,
  serialized_end=166,
)


_WALKOPTIMISERCOMMAND = _descriptor.Descriptor(
  name='WalkOptimiserCommand',
  full_name='message.behaviour.WalkOptimiserCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='walkConfig', full_name='message.behaviour.WalkOptimiserCommand.walkConfig', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=168,
  serialized_end=210,
)


_FIXEDWALKCOMMAND_WALKSEGMENT = _descriptor.Descriptor(
  name='WalkSegment',
  full_name='message.behaviour.FixedWalkCommand.WalkSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='direction', full_name='message.behaviour.FixedWalkCommand.WalkSegment.direction', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='curvePeriod', full_name='message.behaviour.FixedWalkCommand.WalkSegment.curvePeriod', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='normalisedVelocity', full_name='message.behaviour.FixedWalkCommand.WalkSegment.normalisedVelocity', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='normalisedAngularVelocity', full_name='message.behaviour.FixedWalkCommand.WalkSegment.normalisedAngularVelocity', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='duration', full_name='message.behaviour.FixedWalkCommand.WalkSegment.duration', index=4,
      number=5, type=11, cpp_type=10, label=1,
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
  serialized_start=301,
  serialized_end=469,
)

_FIXEDWALKCOMMAND = _descriptor.Descriptor(
  name='FixedWalkCommand',
  full_name='message.behaviour.FixedWalkCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='segments', full_name='message.behaviour.FixedWalkCommand.segments', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_FIXEDWALKCOMMAND_WALKSEGMENT, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=213,
  serialized_end=469,
)

_FIXEDWALKCOMMAND_WALKSEGMENT.fields_by_name['direction'].message_type = Vector__pb2._VEC2
_FIXEDWALKCOMMAND_WALKSEGMENT.fields_by_name['duration'].message_type = google_dot_protobuf_dot_duration__pb2._DURATION
_FIXEDWALKCOMMAND_WALKSEGMENT.containing_type = _FIXEDWALKCOMMAND
_FIXEDWALKCOMMAND.fields_by_name['segments'].message_type = _FIXEDWALKCOMMAND_WALKSEGMENT
DESCRIPTOR.message_types_by_name['FixedWalkFinished'] = _FIXEDWALKFINISHED
DESCRIPTOR.message_types_by_name['WalkConfigSaved'] = _WALKCONFIGSAVED
DESCRIPTOR.message_types_by_name['CancelFixedWalk'] = _CANCELFIXEDWALK
DESCRIPTOR.message_types_by_name['WalkOptimiserCommand'] = _WALKOPTIMISERCOMMAND
DESCRIPTOR.message_types_by_name['FixedWalkCommand'] = _FIXEDWALKCOMMAND
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FixedWalkFinished = _reflection.GeneratedProtocolMessageType('FixedWalkFinished', (_message.Message,), dict(
  DESCRIPTOR = _FIXEDWALKFINISHED,
  __module__ = 'message.behaviour.FixedWalkCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.FixedWalkFinished)
  ))
_sym_db.RegisterMessage(FixedWalkFinished)

WalkConfigSaved = _reflection.GeneratedProtocolMessageType('WalkConfigSaved', (_message.Message,), dict(
  DESCRIPTOR = _WALKCONFIGSAVED,
  __module__ = 'message.behaviour.FixedWalkCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.WalkConfigSaved)
  ))
_sym_db.RegisterMessage(WalkConfigSaved)

CancelFixedWalk = _reflection.GeneratedProtocolMessageType('CancelFixedWalk', (_message.Message,), dict(
  DESCRIPTOR = _CANCELFIXEDWALK,
  __module__ = 'message.behaviour.FixedWalkCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.CancelFixedWalk)
  ))
_sym_db.RegisterMessage(CancelFixedWalk)

WalkOptimiserCommand = _reflection.GeneratedProtocolMessageType('WalkOptimiserCommand', (_message.Message,), dict(
  DESCRIPTOR = _WALKOPTIMISERCOMMAND,
  __module__ = 'message.behaviour.FixedWalkCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.WalkOptimiserCommand)
  ))
_sym_db.RegisterMessage(WalkOptimiserCommand)

FixedWalkCommand = _reflection.GeneratedProtocolMessageType('FixedWalkCommand', (_message.Message,), dict(

  WalkSegment = _reflection.GeneratedProtocolMessageType('WalkSegment', (_message.Message,), dict(
    DESCRIPTOR = _FIXEDWALKCOMMAND_WALKSEGMENT,
    __module__ = 'message.behaviour.FixedWalkCommand_pb2'
    # @@protoc_insertion_point(class_scope:message.behaviour.FixedWalkCommand.WalkSegment)
    ))
  ,
  DESCRIPTOR = _FIXEDWALKCOMMAND,
  __module__ = 'message.behaviour.FixedWalkCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.FixedWalkCommand)
  ))
_sym_db.RegisterMessage(FixedWalkCommand)
_sym_db.RegisterMessage(FixedWalkCommand.WalkSegment)


# @@protoc_insertion_point(module_scope)
