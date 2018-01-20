# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: baxter.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import pose_pb2 as pose__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='baxter.proto',
  package='',
  syntax='proto3',
  serialized_pb=_b('\n\x0c\x62\x61xter.proto\x1a\npose.proto\"[\n\x05Joint\x12\n\n\x02s0\x18\x01 \x01(\x02\x12\n\n\x02s1\x18\x02 \x01(\x02\x12\n\n\x02\x65\x30\x18\x03 \x01(\x02\x12\n\n\x02\x65\x31\x18\x04 \x01(\x02\x12\n\n\x02w0\x18\x05 \x01(\x02\x12\n\n\x02w1\x18\x06 \x01(\x02\x12\n\n\x02w2\x18\x07 \x01(\x02\"b\n\x0cGripperState\x12%\n\x05state\x18\x01 \x01(\x0e\x32\x16.GripperState.GripperS\"+\n\x08GripperS\x12\x08\n\x04IDLE\x10\x00\x12\x0b\n\x07SUCCESS\x10\x01\x12\x08\n\x04\x46\x41IL\x10\x02\"f\n\x0eGripperCommand\x12)\n\x07\x63ommand\x18\x01 \x01(\x0e\x32\x18.GripperCommand.GripperC\")\n\x08GripperC\x12\x08\n\x04STAY\x10\x00\x12\x08\n\x04OPEN\x10\x01\x12\t\n\x05\x43LOSE\x10\x02\"F\n\x05Image\x12\x0e\n\x06height\x18\x01 \x01(\x05\x12\r\n\x05width\x18\x02 \x01(\x05\x12\x0f\n\x07\x63hannel\x18\x03 \x01(\x05\x12\r\n\x05image\x18\x04 \x01(\x0c\"\xdc\x01\n\rBaxterCommand\x12\x10\n\x08has_left\x18\x01 \x01(\x08\x12\x11\n\thas_right\x18\x02 \x01(\x08\x12\"\n\x12left_joint_command\x18\x03 \x01(\x0b\x32\x06.Joint\x12#\n\x13right_joint_command\x18\x04 \x01(\x0b\x32\x06.Joint\x12-\n\x14left_gripper_command\x18\x05 \x01(\x0b\x32\x0f.GripperCommand\x12.\n\x15right_gripper_command\x18\x06 \x01(\x0b\x32\x0f.GripperCommand\"\xc7\x02\n\rBaxterObserve\x12!\n\x11left_joint_angles\x18\x01 \x01(\x0b\x32\x06.Joint\x12\"\n\x12right_joint_angles\x18\x02 \x01(\x0b\x32\x06.Joint\x12)\n\x12left_gripper_state\x18\x03 \x01(\x0b\x32\r.GripperState\x12*\n\x13right_gripper_state\x18\x04 \x01(\x0b\x32\r.GripperState\x12\x1b\n\x0cleft_endpose\x18\x05 \x01(\x0b\x32\x05.Pose\x12\x1c\n\rright_endpose\x18\x06 \x01(\x0b\x32\x05.Pose\x12\x1a\n\nhead_image\x18\x07 \x01(\x0b\x32\x06.Image\x12\x1f\n\x0fleft_hand_image\x18\x08 \x01(\x0b\x32\x06.Image\x12 \n\x10right_hand_image\x18\t \x01(\x0b\x32\x06.Imageb\x06proto3')
  ,
  dependencies=[pose__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_GRIPPERSTATE_GRIPPERS = _descriptor.EnumDescriptor(
  name='GripperS',
  full_name='GripperState.GripperS',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='IDLE', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SUCCESS', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FAIL', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=176,
  serialized_end=219,
)
_sym_db.RegisterEnumDescriptor(_GRIPPERSTATE_GRIPPERS)

_GRIPPERCOMMAND_GRIPPERC = _descriptor.EnumDescriptor(
  name='GripperC',
  full_name='GripperCommand.GripperC',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STAY', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OPEN', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CLOSE', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=282,
  serialized_end=323,
)
_sym_db.RegisterEnumDescriptor(_GRIPPERCOMMAND_GRIPPERC)


_JOINT = _descriptor.Descriptor(
  name='Joint',
  full_name='Joint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='s0', full_name='Joint.s0', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s1', full_name='Joint.s1', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='e0', full_name='Joint.e0', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='e1', full_name='Joint.e1', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='w0', full_name='Joint.w0', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='w1', full_name='Joint.w1', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='w2', full_name='Joint.w2', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=28,
  serialized_end=119,
)


_GRIPPERSTATE = _descriptor.Descriptor(
  name='GripperState',
  full_name='GripperState',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='GripperState.state', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _GRIPPERSTATE_GRIPPERS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=121,
  serialized_end=219,
)


_GRIPPERCOMMAND = _descriptor.Descriptor(
  name='GripperCommand',
  full_name='GripperCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='command', full_name='GripperCommand.command', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _GRIPPERCOMMAND_GRIPPERC,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=221,
  serialized_end=323,
)


_IMAGE = _descriptor.Descriptor(
  name='Image',
  full_name='Image',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='height', full_name='Image.height', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='Image.width', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='channel', full_name='Image.channel', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='image', full_name='Image.image', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=325,
  serialized_end=395,
)


_BAXTERCOMMAND = _descriptor.Descriptor(
  name='BaxterCommand',
  full_name='BaxterCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='has_left', full_name='BaxterCommand.has_left', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='has_right', full_name='BaxterCommand.has_right', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_joint_command', full_name='BaxterCommand.left_joint_command', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_joint_command', full_name='BaxterCommand.right_joint_command', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_gripper_command', full_name='BaxterCommand.left_gripper_command', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_gripper_command', full_name='BaxterCommand.right_gripper_command', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=398,
  serialized_end=618,
)


_BAXTEROBSERVE = _descriptor.Descriptor(
  name='BaxterObserve',
  full_name='BaxterObserve',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left_joint_angles', full_name='BaxterObserve.left_joint_angles', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_joint_angles', full_name='BaxterObserve.right_joint_angles', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_gripper_state', full_name='BaxterObserve.left_gripper_state', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_gripper_state', full_name='BaxterObserve.right_gripper_state', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_endpose', full_name='BaxterObserve.left_endpose', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_endpose', full_name='BaxterObserve.right_endpose', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='head_image', full_name='BaxterObserve.head_image', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_hand_image', full_name='BaxterObserve.left_hand_image', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_hand_image', full_name='BaxterObserve.right_hand_image', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=621,
  serialized_end=948,
)

_GRIPPERSTATE.fields_by_name['state'].enum_type = _GRIPPERSTATE_GRIPPERS
_GRIPPERSTATE_GRIPPERS.containing_type = _GRIPPERSTATE
_GRIPPERCOMMAND.fields_by_name['command'].enum_type = _GRIPPERCOMMAND_GRIPPERC
_GRIPPERCOMMAND_GRIPPERC.containing_type = _GRIPPERCOMMAND
_BAXTERCOMMAND.fields_by_name['left_joint_command'].message_type = _JOINT
_BAXTERCOMMAND.fields_by_name['right_joint_command'].message_type = _JOINT
_BAXTERCOMMAND.fields_by_name['left_gripper_command'].message_type = _GRIPPERCOMMAND
_BAXTERCOMMAND.fields_by_name['right_gripper_command'].message_type = _GRIPPERCOMMAND
_BAXTEROBSERVE.fields_by_name['left_joint_angles'].message_type = _JOINT
_BAXTEROBSERVE.fields_by_name['right_joint_angles'].message_type = _JOINT
_BAXTEROBSERVE.fields_by_name['left_gripper_state'].message_type = _GRIPPERSTATE
_BAXTEROBSERVE.fields_by_name['right_gripper_state'].message_type = _GRIPPERSTATE
_BAXTEROBSERVE.fields_by_name['left_endpose'].message_type = pose__pb2._POSE
_BAXTEROBSERVE.fields_by_name['right_endpose'].message_type = pose__pb2._POSE
_BAXTEROBSERVE.fields_by_name['head_image'].message_type = _IMAGE
_BAXTEROBSERVE.fields_by_name['left_hand_image'].message_type = _IMAGE
_BAXTEROBSERVE.fields_by_name['right_hand_image'].message_type = _IMAGE
DESCRIPTOR.message_types_by_name['Joint'] = _JOINT
DESCRIPTOR.message_types_by_name['GripperState'] = _GRIPPERSTATE
DESCRIPTOR.message_types_by_name['GripperCommand'] = _GRIPPERCOMMAND
DESCRIPTOR.message_types_by_name['Image'] = _IMAGE
DESCRIPTOR.message_types_by_name['BaxterCommand'] = _BAXTERCOMMAND
DESCRIPTOR.message_types_by_name['BaxterObserve'] = _BAXTEROBSERVE

Joint = _reflection.GeneratedProtocolMessageType('Joint', (_message.Message,), dict(
  DESCRIPTOR = _JOINT,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:Joint)
  ))
_sym_db.RegisterMessage(Joint)

GripperState = _reflection.GeneratedProtocolMessageType('GripperState', (_message.Message,), dict(
  DESCRIPTOR = _GRIPPERSTATE,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:GripperState)
  ))
_sym_db.RegisterMessage(GripperState)

GripperCommand = _reflection.GeneratedProtocolMessageType('GripperCommand', (_message.Message,), dict(
  DESCRIPTOR = _GRIPPERCOMMAND,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:GripperCommand)
  ))
_sym_db.RegisterMessage(GripperCommand)

Image = _reflection.GeneratedProtocolMessageType('Image', (_message.Message,), dict(
  DESCRIPTOR = _IMAGE,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:Image)
  ))
_sym_db.RegisterMessage(Image)

BaxterCommand = _reflection.GeneratedProtocolMessageType('BaxterCommand', (_message.Message,), dict(
  DESCRIPTOR = _BAXTERCOMMAND,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:BaxterCommand)
  ))
_sym_db.RegisterMessage(BaxterCommand)

BaxterObserve = _reflection.GeneratedProtocolMessageType('BaxterObserve', (_message.Message,), dict(
  DESCRIPTOR = _BAXTEROBSERVE,
  __module__ = 'baxter_pb2'
  # @@protoc_insertion_point(class_scope:BaxterObserve)
  ))
_sym_db.RegisterMessage(BaxterObserve)


# @@protoc_insertion_point(module_scope)