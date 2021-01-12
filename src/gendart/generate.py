#
#    Copyright 2016 Rethink Robotics
#
#    Copyright 2016 Chris Smith
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from __future__ import print_function

import sys
import os
import traceback
import re
from os.path import join as pjoin
from os.path import split as psplit
# import roslib.msgs
# import roslib.srvs
# import roslib.packages
# import roslib.gentools
from genmsg import SrvSpec, MsgSpec, MsgContext
from genmsg.msg_loader import load_srv_from_file, load_msg_by_type
import genmsg.gentools
from copy import copy, deepcopy
import glob
import time

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x
GenVersion = "0.0.3"
############################################################
# Built in types
############################################################
generated_packages = set()
DebugGen = False

def is_fixnum(t):
    return t in ['int8', 'uint8', 'int16', 'uint16']


def is_integer(t):
    # t2 byte, char can be fixnum
    return is_fixnum(t) or t in ['byte', 'char', 'int32', 'uint32', 'int64', 'uint64']


def is_signed_int(t):
    return t in ['int8', 'int16', 'int32', 'int64']


def is_unsigned_int(t):
    return t in ['uint8', 'uint16', 'uint32', 'uint64']


def is_bool(t):
    return t == 'bool'


def is_string(t):
    return t == 'string'


def is_float(t):
    return t in ['float32', 'float64']


def is_time(t):
    return t in ['time', 'duration']


def parse_msg_type(f):
    if f.base_type == 'Header':
        return ('std_msgs', 'Header')
    else:
        return f.base_type.split('/')


def get_typed_array(t):
    if t in ['int8', 'byte', 'bool']:
        return 'Int8Array'
    elif t in ['uint8', 'char']:
        return 'UInt8Array'
    elif t == 'uint16':
        return 'UInt16Array'
    elif t == 'int16':
        return 'Int16Array'
    elif t == 'uint32':
        return 'UInt32Array'
    elif t == 'int32':
        return 'Int32Array'
    elif t == 'float32':
        return 'Float32Array'
    elif t == 'float64':
        return 'Float64Array'
    # else
    return None


def has_typed_array(t):
    return is_fixnum(t) or is_float(t) or t in ['byte', 'char', 'bool', 'uint8', 'uint16', 'int8', 'int16', 'uint32', 'int32']


def get_type_size(t):
    """Returns the size in bytes of a builtin type if available. Else None"""
    if t in ['int8', 'uint8', 'byte', 'bool', 'char']:
        return 1
    elif t in ['int16', 'uint16']:
        return 2
    elif t in ['int32', 'uint32', 'float32']:
        return 4
    elif t in ['int64', 'uint64', 'float64', 'time', 'duration']:
        return 8
    return None


def get_type_from_string(t):
    if is_string(t):
        return 'String'
    elif is_time(t):
        return 'RosTime'
    elif is_bool(t):
        return 'bool'
    elif is_float(t):
        return 'double'
    elif is_integer(t):
        return 'int'
    else:
        print(t)
        print('ERROR getting type from string!')
        return 'var'


def get_type(field):
    if field.is_array:
        field_copy = deepcopy(field)
        field_copy.is_array = False
        field_copy.type = field_copy.base_type
        return 'List<{}>'.format(get_type(field_copy))
    elif field.is_builtin:
        if is_string(field.type):
            return 'String'
        elif is_time(field.type):
            return 'RosTime'
        elif is_bool(field.type):
            return 'bool'
        elif is_float(field.type):
            return 'double'
        elif is_integer(field.type):
            return 'int'
        else:
            print(field.type)
            print('ERROR getting type!')
            return 'var'
    (package, msg_type) = field.base_type.split('/')
    return '{}'.format(msg_type)


def get_default_value(field, current_message_package):
    """Return the default value for a message data field"""
    if field.is_array:
        if not field.array_len:
            return '[]'
        else:
            field_copy = deepcopy(field)
            field_copy.is_array = False
            field_default = get_default_value(
                field_copy, current_message_package)
            return 'List.generate({}, (_) => {})'.format(field.array_len, field_default)
    elif field.is_builtin:
        if is_string(field.type):
            return '\'\''
        elif is_time(field.type):
            return 'RosTime(secs: 0, nsecs: 0)'
        elif is_bool(field.type):
            return 'false'
        elif is_float(field.type):
            return '0.0'
        else:
            return '0'
    # else
    (package, msg_type) = field.base_type.split('/')
    return '{}()'.format(msg_type)


def is_message_fixed_size(spec, search_path):
    """Check if a particular message specification has a constant size in bytes"""
    parsed_fields = spec.parsed_fields()
    types = [f.base_type for f in parsed_fields]
    variableLengthArrays = [
        f.is_array and not f.array_len for f in parsed_fields]
    isBuiltin = [f.is_builtin for f in parsed_fields]
    if 'string' in types:
        return False
    elif True in variableLengthArrays:
        return False
    elif False not in isBuiltin:
        return True
    else:
        nonBuiltins = [f for f in parsed_fields if not f.is_builtin]
        # print(nonBuiltins)
        for idx, f in enumerate(nonBuiltins):
            field_msg_context = MsgContext.create_default()
            field_spec = genmsg.msg_loader.load_msg_by_type(
                field_msg_context, f.base_type, search_path)
            if not is_message_fixed_size(field_spec, search_path):
                return False
        return True


def get_message_fixed_size(spec, search_path):
    """
    Return the size of a message.
    If the message does not have a fixed size, returns None
    """
    if not is_message_fixed_size(spec, search_path):
        return None
    # else
    length = 0
    for f in spec.parsed_fields():
        if f.is_builtin:
            type_size = get_type_size(f.base_type)
            if type_size is None:
                raise Exception(
                    'Field {} has a non-constant size'.format(f.base_type))
            if not f.is_array:
                length += type_size
            elif not f.array_len:
                raise Exception(
                    'Array field {} has a variable length'.format(f.base_type))
            else:
                length += (f.array_len * type_size)
        else:
            field_msg_context = MsgContext.create_default()
            field_spec = genmsg.msg_loader.load_msg_by_type(
                field_msg_context, f.base_type, search_path)
            field_size = get_message_fixed_size(field_spec, search_path)
            if field_size is None:
                raise Exception(
                    'Field {} has a non-constant size'.format(f.base_type))
            length += field_size
    return length

############################################################
# Indented writer
############################################################


class IndentedWriter():

    def __init__(self, s):
        self.str = s
        self.indentation = 0
        self.block_indent = False

    def write(self, s, indent=True, newline=True):
        if not indent:
            newline = False
        if self.block_indent:
            self.block_indent = False
        else:
            if newline:
                self.str.write('\n')
            if indent:
                for i in range(self.indentation):
                    self.str.write(' ')
        self.str.write(s)

    def newline(self):
        self.str.write('\n')

    def inc_indent(self, inc=2):
        self.indentation += inc

    def dec_indent(self, dec=2):
        self.indentation -= dec

    def reset_indent(self):
        self.indentation = 0

    def block_next_indent(self):
        self.block_indent = True


class Indent():

    def __init__(self, w, inc=2, indent_first=True):
        self.writer = w
        self.inc = inc
        self.indent_first = indent_first

    def __enter__(self):
        self.writer.inc_indent(self.inc)
        if not self.indent_first:
            self.writer.block_next_indent()

    def __exit__(self, type, val, traceback):
        self.writer.dec_indent(self.inc)


def find_path_from_cmake_path(path):
    cmake_path = os.environ['CMAKE_PREFIX_PATH']
    paths = cmake_path.split(':')
    for search_path in paths:
        test_path = pjoin(search_path, path)
        if os.path.exists(test_path):
            return test_path
    return None


def find_path_for_package(package):
    return find_path_from_cmake_path(pjoin('share/gendart/ros', package))


def find_requires(spec):
    found_packages = []
    local_deps = []
    external_deps = {}
    for field in spec.parsed_fields():
        if not field.is_builtin:
            (field_type_package, msg_type) = field.base_type.split('/')
            if field_type_package != spec.package:
                if field_type_package not in external_deps.keys():
                    external_deps[field_type_package] = []
                external_deps[field_type_package].append(msg_type)
            if field_type_package in found_packages:
                continue
            # else
            if field_type_package == spec.package:
                if msg_type not in local_deps:
                    local_deps.append(msg_type)
            else:
                found_packages.append(field_type_package)

    return found_packages, local_deps, external_deps


def write_begin(s, spec, is_service=False):
    "Writes the beginning of the file: a comment saying it's auto-generated and the in-package form"

    s.write('// Auto-generated. Do not edit!\n\n', newline=False)
    s.write('// Updated: {}\n\n'.format(time.ctime()), newline=False)
    suffix = 'srv' if is_service else 'msg'
    s.write('// (in-package %s.%s)\n\n' %
            (spec.package, suffix), newline=False)
    s.write('// ignore_for_file: unused_import, overridden_fields')

def write_extra_action_requires(s, spec):
    if (spec.short_name.endswith('Action')):
        base_name = spec.short_name.split('Action')[0]
        s.write('import \'{}Goal.dart\';'.format(base_name))
        s.write('import \'{}Feedback.dart\';'.format(base_name))
        s.write('import \'{}Result.dart\';'.format(base_name))
    

def write_requires(s, spec, search_path, output_dir, previous_packages=None, prev_deps=None, isSrv=False):
    "Writes out the require fields"
    if previous_packages is None:
        s.write('import \'dart:convert\';')
        s.write('import \'package:buffer/buffer.dart\';')
        s.write('import \'package:dartros/msg_utils.dart\';')

        previous_packages = {}
    if prev_deps is None:
        prev_deps = []
    # find other message packages and other messages in this packages
    # that this message depends on
    found_packages, local_deps, external_deps = find_requires(spec)
    # print('External Dependencies: {}'.format(external_deps))
    # filter out previously found local deps
    local_deps = [dep for dep in local_deps if dep not in prev_deps]
    # filter out previously found packages
    found_packages = {
        package for package in found_packages if package not in previous_packages}
    for package in found_packages:
        # print('External Package: {}, messages: {}'.format(
        # package, external_deps[package]))
        # TODO: finder is only relevant to node - we should support an option to
        #   create a flat message package directory. The downside is that it requires
        #   copying files between workspaces.
        s.write('import \'package:{}/msgs.dart\';'.format(package))
        (directory, pack) = psplit(output_dir)
        generate_all_msgs_for_package(package, directory, search_path)

    # require mesages from this package
    # messages from this package need to be requried separately
    # so that we don't create a circular requires dependency
    for dep in local_deps:
        if isSrv:
            s.write('import \'../msgs/{}.dart\';'.format(dep))
        else:
            s.write('import \'{}.dart\';'.format(dep))

    s.newline()
    s.write('//-----------------------------------------------------------')
    s.newline()
    return found_packages, local_deps


def write_msg_fields(s, spec, field, action=False):
    if action:
        s.write('@override')
    s.write('{} {};'.format(get_type(field), field.name))


def write_msg_constructor_field(s, spec, field):
    s.write('{} {},'.format(get_type(field), field.name))


def write_msg_constructor_initializers(s, spec, field, last):
    s.write('this.{} = {} ?? {}{}'.format(
            field.name, field.name, get_default_value(field, spec.package), ',' if not last else ';'))

def write_msg_call_initializers(s, spec, field, last):
    s.write('{}: {},'.format(
            field.name, field.name ))


def write_class(s, spec, action=False):
    # s.write('@rosDeserializeable')
    if action:
        base_name = spec.short_name.split('Action')[0]
        if action == 'goal':
            s.write('class {0}ActionGoal extends RosActionGoal<{0}Goal, {0}ActionGoal> {{'.format(base_name))
        elif action == 'feedback':
            s.write('class {0}ActionFeedback extends RosActionFeedback<{0}Feedback, {0}ActionFeedback> {{'.format(base_name))
        elif action == 'result':
            s.write('class {0}ActionResult extends RosActionResult<{0}Result, {0}ActionResult> {{'.format(base_name))
        elif action == 'action':
            s.write('class {0}Action extends RosActionMessage<{0}Goal, {0}ActionGoal, {0}Feedback, {0}ActionFeedback, {0}Result, {0}ActionResult> {{'.format(base_name))
    else:
        s.write('class {0} extends RosMessage<{0}> {{'.format(spec.actual_name))

    action_class = action != False and action != 'action'

    with Indent(s):

        for field in spec.parsed_fields():
            write_msg_fields(s, spec, field, action=action_class)
            s.newline()
        # Constructor
        s.write('static {0} $prototype = {0}();'.format(spec.actual_name))

        num_fields = len(spec.parsed_fields())
        if num_fields > 0:
            s.write('{}({{ '.format(spec.actual_name))
            with Indent(s):
                for field in spec.parsed_fields(): 
                    write_msg_constructor_field(s, spec, field)
            s.write('}):')
            for i, field in enumerate(spec.parsed_fields()):
                write_msg_constructor_initializers(
                    s, spec, field, num_fields-1 == i)
        else:
            s.write('{}();'.format(spec.actual_name))

        s.newline()

        num_fields = len(spec.parsed_fields())
        if num_fields > 0:
            s.write('@override')
            s.write('{} call({{ '.format(spec.actual_name))
            with Indent(s):
                for field in spec.parsed_fields():
                    write_msg_constructor_field(s, spec, field)
            s.write('}}) => {}('.format(spec.actual_name))
            for i, field in enumerate(spec.parsed_fields()):
                write_msg_call_initializers(
                    s, spec, field, num_fields-1 == i)
            s.write(');')
            
        else:
            s.write('@override')
            s.write('{0} call() => {0}();'.format(spec.actual_name))

    s.newline()


def get_message_path_from_field(field, pkg):
    (field_pkg, msg_type) = field.base_type.split('/')
    if field_pkg == pkg:
        return msg_type
    # else
    return '{}.msg.{}'.format(field_pkg, msg_type)


def write_end(s, spec):
    write_constants(s, spec)
    s.write('}')
    s.newline()


def write_serialize_base(s, rest):
    s.write('{};'.format(rest))


def write_serialize_length(s, name):
    # t2
    s.write('// Serialize the length for message field [{}]'.format(name))
    write_serialize_base(
        s, 'writer.writeUint32({}.length)'.format(name))


def write_serialize_length_check(s, field):
    s.write(
        '// Check that the constant length array field [{}] has the right length'.format(field.name))
    s.write('if ({}.length != {}) {{'.format(field.name, field.array_len))
    with Indent(s):
        s.write('throw Exception(\'Unable to serialize array field {} - length must be {}\');'.format(
            field.name, field.array_len))
    s.write('}')

# adds function to serialize builtin types (string, uint8, ...)


def serialize_builtin(t, val):
    if t == 'string':
        return 'writer.writeString({})'.format(val)
    if t == 'time' or t == 'duration':
        return 'writer.writeTime({})'.format(val)
    if t == 'int8':
        return 'writer.writeInt8({})'.format(val)
    if t == 'int16':
        return 'writer.writeInt16({})'.format(val)
    if t == 'int32':
        return 'writer.writeInt32({})'.format(val)
    if t == 'int64':
        return 'writer.writeInt64({})'.format(val)
    if t == 'uint8':
        return 'writer.writeUint8({})'.format(val)
    if t == 'uint16':
        return 'writer.writeUint16({})'.format(val)
    if t == 'uint32':
        return 'writer.writeUint32({})'.format(val)
    if t == 'uint64':
        return 'writer.writeUint64({})'.format(val)
    if t == 'float32':
        return 'writer.writeFloat32({})'.format(val)
    if t == 'float64':
        return 'writer.writeFloat64({})'.format(val)
    if t == 'byte':
        return 'writer.writeUint8({})'.format(val)
    if t == 'char':
        return 'writer.writeUint8({})'.format(val)
    if t == 'bool':
        return 'writer.writeUint8({} == false ? 0 : 1)'.format(val)
    print(t)


def write_serialize_builtin(s, f):
    if (f.is_array):
        write_serialize_base(s, 'writer.writeArray<{}>({}, (val) => {}, specArrayLen: {})'.format(
            get_type_from_string(f.base_type), f.name, serialize_builtin(f.base_type, 'val'), 'null' if f.array_len is None else f.array_len))
    else:
        write_serialize_base(
            s, '{}'.format(serialize_builtin(f.base_type, f.name)))

# adds function to serlialize complex type (geometry_msgs/Pose)


def write_serialize_complex(s, f, thisPackage):
    (package, msg_type) = f.base_type.split('/')
    if (f.is_array):
        if f.array_len is None:
            write_serialize_length(s, f.name)
        s.write('{}.forEach((val) {{'.format(f.name))
        with Indent(s):
            write_serialize_base(
                s, 'val.serialize(writer)')
        s.write('});')
    else:
        write_serialize_base(s, '{}.serialize(writer)'.format(f.name))


# writes serialization for a single field in the message


def write_serialize_field(s, f, package):
    if f.is_array:
        if f.array_len is not None:
            write_serialize_length_check(s, f)

    s.write('// Serialize message field [{}]'.format(f.name))
    if f.is_builtin:
        write_serialize_builtin(s, f)
    else:
        write_serialize_complex(s, f, package)


def write_serialize(s, spec):
    """
    Write the serialize method
    """
    with Indent(s):
        s.write('void serialize(ByteDataWriter writer) {')
        with Indent(s):
            s.write(
                '// Serializes a message object of type {}'.format(spec.short_name))
            for f in spec.parsed_fields():
                write_serialize_field(s, f, spec.package)
        s.write('}')
        s.newline()

# t2 can get rid of is_array


def write_deserialize_length(s, name):
    s.write('// Deserialize array length for message field [{}]'.format(name))
    s.write('final len = reader.readInt32();')


def write_deserialize_complex(s, f, thisPackage):
    (package, msg_type) = f.base_type.split('/')
    if f.is_array:
        s.write('{')
        with Indent(s):
            if f.array_len is None:
                write_deserialize_length(s, f.name)
            else:
                s.write('{\nfinal len = {};'.format(f.array_len))

            s.write(
                'data.{} = List.generate(len, (_) => {}.$prototype.deserialize(reader));'.format(f.name, msg_type))
        s.write('}')
    else:
        s.write('data.{} = {}.$prototype.deserialize(reader);'.format(
            f.name, msg_type))


def deserialize_builtin(t):
    if t == 'string':
        return 'reader.readString()'
    if t == 'time' or t == 'duration':
        return 'reader.readTime()'
    if t == 'int8':
        return 'reader.readInt8()'
    if t == 'int16':
        return 'reader.readInt16()'
    if t == 'int32':
        return 'reader.readInt32()'
    if t == 'int64':
        return 'reader.readInt64()'
    if t == 'uint8':
        return 'reader.readUint8()'
    if t == 'uint16':
        return 'reader.readUint16()'
    if t == 'uint32':
        return 'reader.readUint32()'
    if t == 'uint64':
        return 'reader.readUint64()'
    if t == 'float32':
        return 'reader.readFloat32()'
    if t == 'float64':
        return 'reader.readFloat64()'
    if t == 'byte':
        return 'reader.readUint8()'
    if t == 'char':
        return 'reader.readUint8()'
    if t == 'bool':
        return 'reader.readUint8() != 0'
    print(t)


def write_deserialize_builtin(s, f):
    if f.is_array:
        s.write('data.{} = reader.readArray<{}>(() => {}, arrayLen: {});'.format(
            f.name, get_type_from_string(f.base_type), deserialize_builtin(f.base_type), 'null' if f.array_len is None else f.array_len))
    else:

        s.write('data.{} = {};'.format(
            f.name, deserialize_builtin(f.base_type)))


def write_deserialize_field(s, f, package):
    s.write('// Deserialize message field [{}]'.format(f.name))
    if f.is_builtin:
        write_deserialize_builtin(s, f)
    else:
        write_deserialize_complex(s, f, package)


def write_deserialize(s, spec):
    """
    Write the deserialize method
    """
    with Indent(s):
        s.write('@override')
        s.write('{} deserialize(ByteDataReader reader) {{'.format(
            spec.short_name, spec.short_name))
        with Indent(s):
            s.write(
                '//deserializes a message object of type {}'.format(spec.short_name))
            s.write('final data = {}();'.format(spec.short_name))
            for f in spec.parsed_fields():
                write_deserialize_field(s, f, spec.package)

            s.write('return data;')
        s.write('}')
        s.newline()


def write_get_message_size(s, spec, search_path):
    """
    Write a method to determine the buffer size of a complete message
    """

    with Indent(s):
        s.write('int getMessageSize() {')
        msg_size = get_message_fixed_size(spec, search_path)
        if msg_size is not None:
            with Indent(s):
                s.write('return {};'.format(msg_size))
        else:
            def get_dynamic_field_length_line(field, query):
                if field.is_builtin:
                    if not is_string(field.base_type):
                        raise Exception('Unexpected field {} with type {} has unknown length'.format(
                            field.name, field.base_type))
                    # it's a string array!
                    return 'length += 4 + {}.length;'.format(query)
                # else
                (package, msg_type) = field.base_type.split('/')
                return 'length += {}.getMessageSize();'.format(query)
            with Indent(s):
                s.write('var length = 0;')
                # certain fields will always have the same size
                # calculate that here instead of dynamically every time
                len_constant_length_fields = 0
                for f in spec.parsed_fields():
                    field_size = None
                    if f.is_builtin:
                        field_size = get_type_size(f.base_type)
                    else:
                        field_msg_context = MsgContext.create_default()
                        field_spec = genmsg.msg_loader.load_msg_by_type(
                            field_msg_context, f.base_type, search_path)
                        field_size = get_message_fixed_size(
                            field_spec, search_path)

                    if f.is_array:
                        if f.array_len and field_size is not None:
                            len_constant_length_fields += (
                                field_size * f.array_len)
                            continue
                        elif not f.array_len:
                            len_constant_length_fields += 4

                        if field_size == 1:
                            s.write('length += {}.length;'.format(f.name))
                        elif field_size is not None:
                            s.write(
                                'length += {} * {}.length;'.format(field_size, f.name))
                        else:
                            if f.is_builtin:
                                if not is_string(f.base_type):
                                    raise Exception('Unexpected field {} with type {} has unknown length'.format(
                                        f.name, f.base_type))
                                # it's a string array!
                                line_to_write = 'length += 4 + utf8.encode(val).length;'
                            else:
                                line_to_write = 'length += val.getMessageSize();'
                            s.write(
                                '{}.forEach((val) {{'.format(f.name))
                            with Indent(s):
                                s.write(line_to_write)
                            s.write('});')
                    elif field_size is not None:
                        len_constant_length_fields += field_size
                    else:
                        # field size is dynamic!
                        if f.is_builtin:
                            if not is_string(f.base_type):
                                raise Exception('Unexpected field {} with type {} has unknown length'.format(
                                    f.name, f.base_type))
                            # it's a string array!
                            len_constant_length_fields += 4
                            line_to_write = 'length += utf8.encode({}).length;'.format(
                                f.name)
                        else:
                            line_to_write = 'length += {}.getMessageSize();'.format(f.name)
                        s.write(line_to_write)

                if len_constant_length_fields > 0:
                    s.write(
                        'return length + {};'.format(len_constant_length_fields))
                else:
                    s.write('return length;')
        s.write('}')
        s.newline()





def write_msg_export(s, msgs, pkg, context):
    "Writes an export file for the messages"
    
    s.write('// Auto-generated. Do not edit!\n\n', newline=False)
    s.write('// Updated: {}\n\n'.format(time.ctime()), newline=False)
    for msg in msgs:
        if msg == 'String':
            msg = 'StringMessage'
        s.write('export \'src/msgs/{}.dart\';'.format(msg))
    s.newline()


def write_srv_export(s, srvs, pkg):
    "Writes an export file for the services"

    s.write('// Auto-generated. Do not edit!\n\n', newline=False)
    s.write('// Updated: {}\n\n'.format(time.ctime()), newline=False)
    for srv in srvs:
        s.write('export \'src/srvs/{}.dart\';'.format(srv))
    s.newline()


def write_ros_datatype(s, spec):
    with Indent(s):
        s.write('@override')
        s.write('String get fullType {')
        with Indent(s):
            s.write('// Returns string type for a %s object' %
                    spec.component_type)
            s.write('return \'{}\';'.format(spec.full_name))
        s.write('}')
        s.newline()


def write_md5sum(s, msg_context, spec, parent=None):
    md5sum = genmsg.compute_md5(msg_context, parent or spec)
    with Indent(s):
        s.write('@override')
        s.write('String get md5sum {')
        with Indent(s):
            # t2 this should print 'service' instead of 'message' if it's a service request or response
            s.write('//Returns md5sum for a message object')
            s.write('return \'{}\';'.format(md5sum))
        s.write('}')
        s.newline()


def write_message_definition(s, msg_context, spec):
    with Indent(s):
        s.write('@override')
        s.write('String get messageDefinition {')
        with Indent(s):
            s.write('// Returns full string definition for message')
            definition = genmsg.compute_full_text(msg_context, spec)
            lines = definition.split('\n')
            s.write('return \'\'\'')
            
            for line in lines:
                s.write('{}\n'.format(line), indent=False)
            s.write('\'\'\';', indent=False)
        s.write('}')
        s.newline()
        
def write_action_extras(s, msg_context, spec):
    with Indent(s):
        base_name = spec.short_name.split('Action')[0]
        s.write('@override')
        s.write('{}Goal get goal => {}Goal.$prototype;'.format(base_name, base_name))
        s.newline()
        s.write('@override')
        s.write('{}ActionGoal get actionGoal => {}ActionGoal.$prototype;'.format(base_name, base_name))
        s.newline()
        s.write('@override')
        s.write('{}Feedback get feedback => {}Feedback.$prototype;'.format(base_name, base_name))
        s.newline()
        s.write('@override')
        s.write('{}ActionFeedback get actionFeedback => {}ActionFeedback.$prototype;'.format(base_name, base_name))
        s.newline()
        s.write('@override')
        s.write('{}Result get result => {}Result.$prototype;'.format(base_name, base_name))
        s.newline()
        s.write('@override')
        s.write('{}ActionResult get actionResult => {}ActionResult.$prototype;'.format(base_name, base_name))
        s.newline()

def write_constants(s, spec):
    if spec.constants:
        s.write('// Constants for message')
        with Indent(s):
            for c in spec.constants:
                if is_string(c.type):
                    s.write('static const String {} = \'{}\';'.format(
                        c.name.upper(), c.val.replace("'", "\\'")))
                elif is_bool(c.type):
                    s.write('static const bool {} = {};'.format(c.name.upper(),
                                                                'true' if c.val else 'false'))
                else:
                    s.write('static const int {} = {};'.format(
                        c.name.upper(), c.val))
        s.newline()


def write_srv_component(s, spec, context, parent, search_path):
    spec.component_type = 'service'
    for field in spec.parsed_fields():
        if field.name == spec.actual_name:
            # print('Here')
            field.name = spec.actual_name + 'Value'
    write_class(s, spec)
    write_serialize(s, spec)
    write_deserialize(s, spec)
    write_get_message_size(s, spec, search_path)
    write_ros_datatype(s, spec)
    write_md5sum(s, context, spec)
    write_message_definition(s, context, spec)
    write_constants(s, spec)
    s.write('}')
    s.newline()


def write_srv_end(s, context, spec):
    name = spec.short_name
    s.write('class {} extends RosServiceMessage<{}Request, {}Response> {{'.format(name, name, name))
    with Indent(s):
        s.write('static final $prototype = {}();'.format(name))
        s.write('@override')
        s.write('{}Request get request => {}Request.$prototype;'.format(name, name))
        s.write('@override')
        s.write('{}Response get response => {}Response.$prototype;'.format(name, name))
        md5sum = genmsg.compute_md5(context, spec)
        s.write('@override')
        s.write('String get md5sum => \'{}\';'.format(md5sum))
        s.write('@override')
        s.write('String get fullType => \'{}\';'.format(
            spec.full_name))
    s.write('}')
    s.newline()

def get_all_dependent_pkgs(search_path, context, package, indir):
    msgs = msg_list_full_path(package, search_path, '.msg')
    srvs = msg_list_full_path(package, search_path, '.srv')
    all_pkgs = set()
    for msg in msgs:
        msg_name = os.path.basename(msg)
        full_type = genmsg.gentools.compute_full_type_name(package, msg_name)
        spec = genmsg.msg_loader.load_msg_from_file(context, msg, full_type)
        pkgs, _, _ = find_requires(spec)
        for pkg in pkgs:
            all_pkgs.add(pkg)
    for msg in srvs:
        msg_name = os.path.basename(msg)
        full_type = genmsg.gentools.compute_full_type_name(package, msg_name)
        spec = genmsg.msg_loader.load_srv_from_file(context, msg, full_type)
        pkgs, _, _ = find_requires(spec)
        for pkg in pkgs:
            all_pkgs.add(pkg)
    # print('Package {}'.format(package))
    # print('Package dependencies {}'.format(all_pkgs))
    if package in all_pkgs:
        all_pkgs.remove(package)
    return all_pkgs

def write_pubspec(s, package, search_path, context, indir):
    s.write('# Auto-generated. Do not edit!\n\n', newline=False)
    s.write('# Updated: {}\n\n'.format(time.ctime()), newline=False)
    s.write('name: {}'.format(package))
    s.write('description: A ros {} message package for dartros'.format(package))
    s.write('version: {}'.format(GenVersion))
    if package == 'std_msgs' or package == 'rosgraph_msgs' or package == 'actionlib_msgs' or package == 'sensor_msgs' or package == 'geometry_msgs':
        s.write('repository: https://github.com/TimWhiting/{}_dart'.format(package))
    msgs = msg_list(package, search_path, '.msg')
    for m in msgs:
        genmsg.load_msg_by_type(context, '%s/%s' %
                                (package, m), search_path)
    srvs = msg_list(package, search_path, '.srv')
    deps = get_all_dependent_pkgs(search_path, context, package, indir)
    # msgExists = os.path.exists(pjoin(package_dir, 'lib/msgs.dart'))
    # srvExists = os.path.exists(pjoin(package_dir, 'lib/srvs.dart'))
    s.newline()
    s.write('environment:')
    with Indent(s):
        s.write('sdk: ">=2.7.0 < 3.0.0"')
    s.newline()
    s.write('dependencies:')
    with Indent(s):
        s.write('buffer: ^1.0.6') 
        s.write('dartros: ^0.0.4+3')
        for dep in deps:
            if dep == 'std_msgs':
                s.write('std_msgs: ^{}'.format(GenVersion))
            elif dep == 'actionlib_msgs':
                s.write('actionlib_msgs: ^{}'.format(GenVersion))
            elif dep == 'rosgraph_msgs':
                s.write('rosgraph_msgs: ^{}'.format(GenVersion))
            elif dep == 'sensor_msgs':
                s.write('sensor_msgs: ^{}'.format(GenVersion))
            elif dep == 'geometry_msgs':
                s.write('geometry_msgs: ^{}'.format(GenVersion))
            else:
                s.write('{}:'.format(dep))
                with Indent(s):
                    s.write('path: ../{}'.format(dep))

    s.newline()

def needs_update(infile, outfile):
    if not os.path.exists(outfile):
        return True
    last_modified_input = os.path.getmtime(infile)
    script_updated = os.path.getmtime(__file__.rstrip('c'))
    outexists = os.path.exists(outfile)
    if outexists and not DebugGen:
        last_modified_output = os.path.getmtime(outfile)
        return last_modified_input > last_modified_output or script_updated > last_modified_output
    return True
    
def generate_all_msgs_for_package(package, output_dir, search_path):
    path_package = find_path_from_cmake_path(
            pjoin('share', package, 'msg')) 
    if (path_package is None):
        # print('New package: {}'.format(package))
        return
    msgs = glob.glob(path_package + '/*.msg')
    # print(msgs)
    other_package = '{}/pubspec.yaml'.format(pjoin(output_dir, package))
    # print('Other package {}'.format(other_package))
    if not needs_update(other_package, other_package): # If generation script hasn't changed
        pass
        # print(t)
        # print(time.time())
        # print('Skipping package generation for {}'.format(package))
    elif package not in generated_packages:
        generated_packages.add(package)
        generate_msg(package, msgs, pjoin(output_dir, package), search_path)
    else:
        pass

def generate_msg(pkg, files, out_dir, search_path):
    """
    Generate dart code for all messages in a package
    """
    # print('Generated packages {}'.format(generated_packages))

    msg_context = MsgContext.create_default()
    
    for f in files:
        f = os.path.abspath(f)
        infile = os.path.basename(f)
        full_type = genmsg.gentools.compute_full_type_name(pkg, infile)
        spec = genmsg.msg_loader.load_msg_from_file(msg_context, f, full_type)
        if spec.short_name == 'String':
            spec.short_name = 'StringMessage'
        
        generate_msg_from_spec(msg_context, spec, search_path, out_dir, pkg, f)
    indir = os.path.dirname(files[0])
    
    ########################################
    # 3. Write the package pubspec.yaml.dart file
    ########################################

   
    io = StringIO()
    s = IndentedWriter(io)
    write_pubspec(s, pkg, search_path, msg_context, indir)
    package_update = True
    pubspec = '{}/pubspec.yaml'.format(out_dir)
    mode = 'w+'
    if os.path.isfile(pubspec):
        mode = 'r+'
    with open(pubspec, mode) as f:
        if f.read() == io.getvalue() and time.time() - os.path.getmtime(pubspec) < 5:
            # print('Pubspec identical')
            package_update = False
    
    if package_update:
        with open(pubspec, 'w+') as f:
            f.write(io.getvalue())
        import subprocess
        try:
            # print('running pub upgrade in {}'.format(out_dir))
            subprocess.check_output('which pub', shell=True)
            p = subprocess.Popen(['pub', 'upgrade'], cwd=out_dir, stdout=subprocess.PIPE)
            p.wait()
        except subprocess.CalledProcessError as e:
            pass

    io.close()
    (directory, pack) = psplit(out_dir)
    if len(search_path.keys()) == 0:
        return
    for package in search_path.keys():
        if package != pkg and package is not None:
            # new_search = deepcopy(search_path)
            # new_search.pop(package)
            generate_all_msgs_for_package(package, directory, search_path)
    


def generate_srv(pkg, files, out_dir, search_path):
    """
    Generate dart code for all services in a package
    """
    msg_context = MsgContext.create_default()
    for f in files:
        f = os.path.abspath(f)
        infile = os.path.basename(f)
        full_type = genmsg.gentools.compute_full_type_name(pkg, infile)
        spec = genmsg.msg_loader.load_srv_from_file(msg_context, f, full_type)
        if '.action' in f:
            print('Action class')
            return
        generate_srv_from_spec(msg_context, spec, search_path, out_dir, pkg, f)
    indir = os.path.dirname(files[0])
    ########################################
    # 3. Write the package pubspec.yaml file
    ########################################
    
    io = StringIO()
    s = IndentedWriter(io)
    write_pubspec(s, pkg, search_path, msg_context, indir)
    package_update = True
    pubspec = '{}/pubspec.yaml'.format(out_dir)
    mode = 'w+'
    if os.path.isfile(pubspec):
        mode = 'r+'
    with open(pubspec, mode) as f:
        if f.read() == io.getvalue() and time.time() - os.path.getmtime(pubspec) < 5:
            # print('Pubspec identical')
            package_update = False
    
    if package_update:
        with open(pubspec, 'w+') as f:
            f.write(io.getvalue())
        import subprocess
        try:
            # print('running pub upgrade in {}'.format(out_dir))
            subprocess.check_output('which pub', shell=True)
            p = subprocess.Popen(['pub', 'upgrade'], cwd=out_dir, stdout=subprocess.PIPE)
            p.wait()
        except subprocess.CalledProcessError as e:
            pass
    io.close()

def msg_list(pkg, search_path, ext):
    dir_list = search_path[pkg]
    files = []
    for d in dir_list:
        files.extend([f for f in os.listdir(d) if f.endswith(ext)])
    return [f[:-len(ext)] for f in files]

def msg_list_full_path(pkg, search_path, ext):
    dir_list = search_path[pkg]
    files = []
    for d in dir_list:
        files.extend([pjoin(d,f) for f in os.listdir(d) if f.endswith(ext)])
    return files



def generate_action_from_spec(msg_context, spec, search_path, output_dir, package, action_type='action'):
    io = StringIO()
    s = IndentedWriter(io)
    write_begin(s, spec)
    write_extra_action_requires(s, spec)
    external_deps, _ = write_requires(s, spec, search_path, output_dir)
    write_class(s, spec, action=action_type)
    write_serialize(s, spec)
    write_deserialize(s, spec)
    write_get_message_size(s, spec, search_path)
    write_ros_datatype(s, spec)
    write_md5sum(s, msg_context, spec)
    write_message_definition(s, msg_context, spec)
    if action_type == 'action':
        write_action_extras(s, msg_context, spec)
    write_end(s, spec)
    src_dir = output_dir + '/lib/src/msgs'
    if (not os.path.exists(src_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(src_dir)
        except OSError as e:
            pass

    with open('%s/lib/src/msgs/%s.dart' % (output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue() + "\n")
    io.close()




def generate_msg_from_spec(msg_context, spec, search_path, output_dir, package, infile, msgs=None):
    """
    Generate a message

    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    output_file = '%s/lib/src/msgs/%s.dart' % (output_dir, spec.short_name)
    if not needs_update(infile, output_file):
        return
    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    spec.actual_name = spec.short_name
    for field in spec.parsed_fields():
        if field.name == spec.actual_name:
            field.name = spec.actual_name + 'Value'
    spec.component_type = 'message'
    msgs = msg_list(package, search_path, '.msg')
    for m in msgs:
        genmsg.load_msg_by_type(msg_context, '%s/%s' %
                                (package, m), search_path)
    msg = spec.short_name
    
    if msg + 'Goal' in msgs and msg + 'Feedback' in msgs and msg + 'Result' in msgs:
        return generate_action_from_spec(msg_context, spec, search_path, output_dir, package)
    elif len(msg.split('ActionGoal')) > 1:
        return generate_action_from_spec(msg_context, spec, search_path, output_dir, package, action_type='goal')
    elif len(msg.split('ActionFeedback')) > 1:
        return generate_action_from_spec(msg_context, spec, search_path, output_dir, package, action_type='feedback')
    elif len(msg.split('ActionResult')) > 1:
        return generate_action_from_spec(msg_context, spec, search_path, output_dir, package, action_type='result')


    ########################################
    # 1. Write the .dart file
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_begin(s, spec)
    external_deps, _ = write_requires(s, spec, search_path, output_dir)
    write_class(s, spec)
    write_serialize(s, spec)
    write_deserialize(s, spec)
    write_get_message_size(s, spec, search_path)
    write_ros_datatype(s, spec)
    write_md5sum(s, msg_context, spec)
    write_message_definition(s, msg_context, spec)
    write_end(s, spec)
    src_dir = output_dir + '/lib/src/msgs'
    if (not os.path.exists(src_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(src_dir)
        except OSError as e:
            pass

    with open(output_file, 'w') as f:
        f.write(io.getvalue() + "\n")
    io.close()
    
    ########################################
    # 3. Write the msgs.dart file
    # This is being rewritten once per msg
    # file, which is inefficient
    ########################################
    io = StringIO()
    s = IndentedWriter(io)
    # print(srvs)
    write_msg_export(s, msgs, package, search_path)
    with open('{}/lib/msgs.dart'.format(output_dir), 'w') as f:
        f.write(io.getvalue())
    io.close()
# TODO most of this could probably be refactored into being shared with messages


def generate_srv_from_spec(msg_context, spec, search_path, output_dir, package, path):
    "Generate code from .srv file"
    output_file = '%s/lib/src/srvs/%s.dart' % (output_dir, spec.short_name)
    if not needs_update(path, output_file):
        return
    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    ext = '.srv'
    srv_path = os.path.dirname(path)
    srvs = msg_list(package, {package: [srv_path]}, ext)
    for srv in srvs:
        load_srv_from_file(msg_context, '%s/%s%s' %
                           (srv_path, srv, ext), '%s/%s' % (package, srv))

    src_dir = output_dir + '/lib/src/srvs'
    if (not os.path.exists(src_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(src_dir)
        except OSError as e:
            pass
    ########################################
    # 1. Write the .dart file
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_begin(s, spec, True)
    found_packages, local_deps = write_requires(
        s, spec.request, search_path, output_dir, None, None, True)
    write_requires(s, spec.response, search_path, output_dir,
                   found_packages, local_deps, True)
    spec.request.actual_name = '%sRequest' % spec.short_name
    spec.response.actual_name = '%sResponse' % spec.short_name
    write_srv_component(s, spec.request, msg_context, spec, search_path)
    write_srv_component(s, spec.response, msg_context, spec, search_path)
    write_srv_end(s, msg_context, spec)

    with open('%s/lib/src/srvs/%s.dart' % (output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 3. Write the srvs.dart file
    # This is being rewritten once per msg
    # file, which is inefficient
    ########################################
    io = StringIO()
    s = IndentedWriter(io)
    # print(srvs)
    write_srv_export(s, srvs, package)
    with open('{}/lib/srvs.dart'.format(output_dir), 'w') as f:
        f.write(io.getvalue())
    io.close()
