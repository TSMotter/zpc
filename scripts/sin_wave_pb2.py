# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sin_wave.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='sin_wave.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0esin_wave.proto\"p\n\x05\x42\x61tch\x12\x1c\n\x05items\x18\x01 \x03(\x0b\x32\r.Batch.Sample\x1aI\n\x06Sample\x12\x0f\n\x07\x63hannel\x18\x01 \x01(\t\x12\x11\n\tfrequency\x18\x02 \x01(\r\x12\x0c\n\x04time\x18\x03 \x01(\x01\x12\r\n\x05value\x18\x04 \x01(\x01\x62\x06proto3'
)




_BATCH_SAMPLE = _descriptor.Descriptor(
  name='Sample',
  full_name='Batch.Sample',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='channel', full_name='Batch.Sample.channel', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='frequency', full_name='Batch.Sample.frequency', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='time', full_name='Batch.Sample.time', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='Batch.Sample.value', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=57,
  serialized_end=130,
)

_BATCH = _descriptor.Descriptor(
  name='Batch',
  full_name='Batch',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='items', full_name='Batch.items', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_BATCH_SAMPLE, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=18,
  serialized_end=130,
)

_BATCH_SAMPLE.containing_type = _BATCH
_BATCH.fields_by_name['items'].message_type = _BATCH_SAMPLE
DESCRIPTOR.message_types_by_name['Batch'] = _BATCH
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Batch = _reflection.GeneratedProtocolMessageType('Batch', (_message.Message,), {

  'Sample' : _reflection.GeneratedProtocolMessageType('Sample', (_message.Message,), {
    'DESCRIPTOR' : _BATCH_SAMPLE,
    '__module__' : 'sin_wave_pb2'
    # @@protoc_insertion_point(class_scope:Batch.Sample)
    })
  ,
  'DESCRIPTOR' : _BATCH,
  '__module__' : 'sin_wave_pb2'
  # @@protoc_insertion_point(class_scope:Batch)
  })
_sym_db.RegisterMessage(Batch)
_sym_db.RegisterMessage(Batch.Sample)


# @@protoc_insertion_point(module_scope)