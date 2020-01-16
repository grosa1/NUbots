# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/vision/Goal.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2
import Neutron_pb2 as Neutron__pb2
import Vector_pb2 as Vector__pb2
import Matrix_pb2 as Matrix__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/vision/Goal.proto',
  package='message.vision',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x19message/vision/Goal.proto\x12\x0emessage.vision\x1a\x1fgoogle/protobuf/timestamp.proto\x1a\rNeutron.proto\x1a\x0cVector.proto\x1a\x0cMatrix.proto\"\x93\x05\n\x04Goal\x12\'\n\x04side\x18\x01 \x01(\x0e\x32\x19.message.vision.Goal.Side\x12\'\n\x04post\x18\x03 \x01(\x0b\x32\x19.message.vision.Goal.Post\x12\x36\n\x0cmeasurements\x18\x04 \x03(\x0b\x32 .message.vision.Goal.Measurement\x12\x1e\n\x0escreen_angular\x18\x05 \x01(\x0b\x32\x06.fvec2\x12\x1c\n\x0c\x61ngular_size\x18\x06 \x01(\x0b\x32\x06.fvec2\x12\'\n\x04team\x18\x07 \x01(\x0e\x32\x19.message.vision.Goal.Team\x1aw\n\x0bMeasurement\x12\x32\n\x04type\x18\x01 \x01(\x0e\x32$.message.vision.Goal.MeasurementType\x12\x18\n\x08position\x18\x02 \x01(\x0b\x32\x06.fvec3\x12\x1a\n\ncovariance\x18\x03 \x01(\x0b\x32\x06.fmat3\x1a\x45\n\x04Post\x12\x13\n\x03top\x18\x01 \x01(\x0b\x32\x06.fvec3\x12\x16\n\x06\x62ottom\x18\x02 \x01(\x0b\x32\x06.fvec3\x12\x10\n\x08\x64istance\x18\x03 \x01(\x02\"-\n\x04Side\x12\x10\n\x0cUNKNOWN_SIDE\x10\x00\x12\x08\n\x04LEFT\x10\x01\x12\t\n\x05RIGHT\x10\x02\"/\n\x04Team\x12\x10\n\x0cUNKNOWN_TEAM\x10\x00\x12\x07\n\x03OWN\x10\x01\x12\x0c\n\x08OPPONENT\x10\x02\"z\n\x0fMeasurementType\x12\x17\n\x13UNKNOWN_MEASUREMENT\x10\x00\x12\n\n\x06\x43\x45NTRE\x10\x01\x12\x0f\n\x0bLEFT_NORMAL\x10\x02\x12\x10\n\x0cRIGHT_NORMAL\x10\x03\x12\x0e\n\nTOP_NORMAL\x10\x04\x12\x0f\n\x0b\x42\x41SE_NORMAL\x10\x05\"\x82\x01\n\x05Goals\x12\x11\n\tcamera_id\x18\x01 \x01(\r\x12-\n\ttimestamp\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x12\n\x03Hcw\x18\x03 \x01(\x0b\x32\x05.mat4\x12#\n\x05goals\x18\x04 \x03(\x0b\x32\x14.message.vision.Goalb\x06proto3')
  ,
  dependencies=[google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,Neutron__pb2.DESCRIPTOR,Vector__pb2.DESCRIPTOR,Matrix__pb2.DESCRIPTOR,])



_GOAL_SIDE = _descriptor.EnumDescriptor(
  name='Side',
  full_name='message.vision.Goal.Side',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_SIDE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=563,
  serialized_end=608,
)
_sym_db.RegisterEnumDescriptor(_GOAL_SIDE)

_GOAL_TEAM = _descriptor.EnumDescriptor(
  name='Team',
  full_name='message.vision.Goal.Team',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_TEAM', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OWN', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OPPONENT', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=610,
  serialized_end=657,
)
_sym_db.RegisterEnumDescriptor(_GOAL_TEAM)

_GOAL_MEASUREMENTTYPE = _descriptor.EnumDescriptor(
  name='MeasurementType',
  full_name='message.vision.Goal.MeasurementType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_MEASUREMENT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CENTRE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT_NORMAL', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_NORMAL', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TOP_NORMAL', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BASE_NORMAL', index=5, number=5,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=659,
  serialized_end=781,
)
_sym_db.RegisterEnumDescriptor(_GOAL_MEASUREMENTTYPE)


_GOAL_MEASUREMENT = _descriptor.Descriptor(
  name='Measurement',
  full_name='message.vision.Goal.Measurement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='message.vision.Goal.Measurement.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='position', full_name='message.vision.Goal.Measurement.position', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='covariance', full_name='message.vision.Goal.Measurement.covariance', index=2,
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
  serialized_start=371,
  serialized_end=490,
)

_GOAL_POST = _descriptor.Descriptor(
  name='Post',
  full_name='message.vision.Goal.Post',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='top', full_name='message.vision.Goal.Post.top', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bottom', full_name='message.vision.Goal.Post.bottom', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distance', full_name='message.vision.Goal.Post.distance', index=2,
      number=3, type=2, cpp_type=6, label=1,
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
  serialized_start=492,
  serialized_end=561,
)

_GOAL = _descriptor.Descriptor(
  name='Goal',
  full_name='message.vision.Goal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='side', full_name='message.vision.Goal.side', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='post', full_name='message.vision.Goal.post', index=1,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='measurements', full_name='message.vision.Goal.measurements', index=2,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='screen_angular', full_name='message.vision.Goal.screen_angular', index=3,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angular_size', full_name='message.vision.Goal.angular_size', index=4,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='team', full_name='message.vision.Goal.team', index=5,
      number=7, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_GOAL_MEASUREMENT, _GOAL_POST, ],
  enum_types=[
    _GOAL_SIDE,
    _GOAL_TEAM,
    _GOAL_MEASUREMENTTYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=122,
  serialized_end=781,
)


_GOALS = _descriptor.Descriptor(
  name='Goals',
  full_name='message.vision.Goals',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='camera_id', full_name='message.vision.Goals.camera_id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='message.vision.Goals.timestamp', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='Hcw', full_name='message.vision.Goals.Hcw', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='goals', full_name='message.vision.Goals.goals', index=3,
      number=4, type=11, cpp_type=10, label=3,
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
  serialized_start=784,
  serialized_end=914,
)

_GOAL_MEASUREMENT.fields_by_name['type'].enum_type = _GOAL_MEASUREMENTTYPE
_GOAL_MEASUREMENT.fields_by_name['position'].message_type = Vector__pb2._FVEC3
_GOAL_MEASUREMENT.fields_by_name['covariance'].message_type = Matrix__pb2._FMAT3
_GOAL_MEASUREMENT.containing_type = _GOAL
_GOAL_POST.fields_by_name['top'].message_type = Vector__pb2._FVEC3
_GOAL_POST.fields_by_name['bottom'].message_type = Vector__pb2._FVEC3
_GOAL_POST.containing_type = _GOAL
_GOAL.fields_by_name['side'].enum_type = _GOAL_SIDE
_GOAL.fields_by_name['post'].message_type = _GOAL_POST
_GOAL.fields_by_name['measurements'].message_type = _GOAL_MEASUREMENT
_GOAL.fields_by_name['screen_angular'].message_type = Vector__pb2._FVEC2
_GOAL.fields_by_name['angular_size'].message_type = Vector__pb2._FVEC2
_GOAL.fields_by_name['team'].enum_type = _GOAL_TEAM
_GOAL_SIDE.containing_type = _GOAL
_GOAL_TEAM.containing_type = _GOAL
_GOAL_MEASUREMENTTYPE.containing_type = _GOAL
_GOALS.fields_by_name['timestamp'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_GOALS.fields_by_name['Hcw'].message_type = Matrix__pb2._MAT4
_GOALS.fields_by_name['goals'].message_type = _GOAL
DESCRIPTOR.message_types_by_name['Goal'] = _GOAL
DESCRIPTOR.message_types_by_name['Goals'] = _GOALS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Goal = _reflection.GeneratedProtocolMessageType('Goal', (_message.Message,), dict(

  Measurement = _reflection.GeneratedProtocolMessageType('Measurement', (_message.Message,), dict(
    DESCRIPTOR = _GOAL_MEASUREMENT,
    __module__ = 'message.vision.Goal_pb2'
    # @@protoc_insertion_point(class_scope:message.vision.Goal.Measurement)
    ))
  ,

  Post = _reflection.GeneratedProtocolMessageType('Post', (_message.Message,), dict(
    DESCRIPTOR = _GOAL_POST,
    __module__ = 'message.vision.Goal_pb2'
    # @@protoc_insertion_point(class_scope:message.vision.Goal.Post)
    ))
  ,
  DESCRIPTOR = _GOAL,
  __module__ = 'message.vision.Goal_pb2'
  # @@protoc_insertion_point(class_scope:message.vision.Goal)
  ))
_sym_db.RegisterMessage(Goal)
_sym_db.RegisterMessage(Goal.Measurement)
_sym_db.RegisterMessage(Goal.Post)

Goals = _reflection.GeneratedProtocolMessageType('Goals', (_message.Message,), dict(
  DESCRIPTOR = _GOALS,
  __module__ = 'message.vision.Goal_pb2'
  # @@protoc_insertion_point(class_scope:message.vision.Goals)
  ))
_sym_db.RegisterMessage(Goals)


# @@protoc_insertion_point(module_scope)
