import sys
import enum
import collections
import re

MSG_TYPES = {'byte': 'int8_t',
             'char': 'uint8_t',
             'bool': 'uint8_t',
             'uint8': 'uint8_t',
             'int8': 'int8_t',
             'uint16': 'uint16_t',
             'int16': 'int16_t',
             'uint32': 'uint32_t',
             'int32': 'int32_t',
             'uint64': 'uint64_t',
             'int64': 'int64_t',
             'float32': 'float',
             'float64': 'double',
             'time': 'float',
             'duration': 'float'}

msg_header_template = """// Copyright 2018 chengzhi-chen
#ifndef ZEROBOT_DRIVER_MSG_H
#define ZEROBOT_DRIVER_MSG_H

#pragma pack(push)
#pragma pack(1)

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <cstring>
#else  // __cplusplus
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#endif  // __cplusplus

#if defined(__GNUC__)
#define MSG_PACKED __attribute__((aligned(1)))
#elif defined(__CC_ARM)
#define MSG_PACKED __packed
#elif defined(_MSC_VER)
#define MSG_PACKED
#endif

STRUCTURE_PLACEHOLDER
BUFFER_PLACEHOLDER
CONSTANTS_PLACEHOLDER

#pragma pack(pop)

#endif  // ZEROBOT_DRIVER_MSG_H
"""


# CamelCase to camel_case
def underscore(text):
    patten = re.compile(r'([a-z]|\d)([A-Z])')
    return re.sub(patten, r'\1_\2', text)


# Add a new line
def line(intend=0, *args):
    text = ''
    for _ in range(intend):
        text += '  '

    if len(args) == 1:
        text += args[0]

    elif len(args) > 1:
        for arg in args[:-1]:
            text += arg + ' '
        text = text.rstrip()
        text += args[-1]

    text += '\n'
    return text


class TopicType(enum.Enum):
    NONE = -1
    SUBSCRIBER = 0
    PUBLISHER = 1


class Msg:
    def __init__(self, name, package, path, fields, constants):
        self.name = name
        self.package = package
        self.path = path
        self.fields = fields
        self.constants = constants

    @staticmethod
    def parse(path_list):
        msg_list = []

        for path in path_list:
            path = path.strip()

            if not path.endswith('msg'):
                print('Unsupported file format: ' + path)
                exit(-1)

            package = re.findall(r'(?:.*)(?<=/)(.*?)(?=/msg/\w+\.msg$)', path)[0].strip()
            name = re.findall(r'(?:.*)(?<=/)(.*?)(?=\.msg$)', path)[0].strip()

            fields = collections.OrderedDict()
            constants = collections.OrderedDict()

            with open(path) as f:
                for rl in f.readlines():
                    # Strip white space
                    rl = rl.strip()

                    # Skipping blank line
                    if len(rl) == 0:
                        continue

                    # Skipping comment
                    if rl.startswith('#'):
                        continue

                    # Constant definition
                    # constant_type1 CONSTANT_NAME1 = constant_value1
                    if re.match(r'^\b\w+\b\s*\b\w+\b\s*=\s*\b\w+\b$', rl):
                        const_type = re.findall(r'(?<=^)(.*?)(?=\s)', rl)[0].strip()

                        if const_type not in MSG_TYPES:
                            print('Error parsing file: ' + path)
                            print('line: ' + rl)
                            print('unsupported type ' + const_type)
                            exit(-1)

                        const_name = re.findall(r'(?<=\s)(.*?)(?==)', rl)[0].strip()

                        const_value = re.findall(r'(?<==)(.*?)(?=$)', rl)[0].strip()

                        constants[const_name] = {'type': const_type, 'value': const_value}

                    # var_type var_name
                    elif re.match(r'^\b\w+\b\s+\b\w+\b$', rl):
                        field_type = re.findall(r'^\w+\b', rl)[0].strip()
                        if field_type not in MSG_TYPES:
                            print('Error parsing file: ' + path)
                            print('line: ' + rl)
                            print('unsupported type ' + field_type)
                            exit(-1)

                        field_name = re.findall(r'\b\w+$', rl)[0].strip()

                        fields[field_name] = {'type': field_type, 'len': '1'}

                    # var_type[count] var_name
                    elif re.match(r'^\b\w+\b\[\d+\]\s+\b\w+\b$', rl):
                        field_len = re.findall(r'(?<=\[)(\d*)(?=\])', rl)[0]

                        field_type = re.findall(r'^\w+\b', rl)[0].strip()
                        if field_type not in MSG_TYPES:
                            print('Error parsing file: ' + path)
                            print('line: ' + rl)
                            print('unsupported type ' + field_type)
                            exit(-1)

                        field_name = re.findall(r'\b\w+$', rl)[0].strip()

                        fields[field_name] = {'type': field_type, 'len': field_len}

                    else:
                        print('Error parsing file: ' + path)
                        print('line: ' + rl)
                        exit(-1)

            msg_list.append(Msg(name, package, path, fields, constants))

        return msg_list

    @staticmethod
    def parse_structure(msg_list):
        text = ''
        for msg in msg_list:
            text += line(0, 'typedef', 'struct', 'MSG_PACKED', '')
            text += line(0, '{', '')

            for field_name, data in msg.fields.items():
                if int(data['len']) > 1:
                    text += line(1, MSG_TYPES[data['type']], field_name + '[' + data['len'] + ']', ';')
                else:
                    text += line(1, MSG_TYPES[data['type']], field_name, ';')

            text += line(0, '}', '')
            text += line(0, 'Msg' + msg.name, ';')
            text += '\n'

        return text

    @staticmethod
    def parse_buffer(msg_list):
        text = ''

        text += line(0, 'typedef', 'union', 'MSG_PACKED', '')
        text += line(0, '{', '')
        for msg in msg_list:
            text += line(1, 'Msg' + msg.name, underscore(msg.name).lower(), ';')

        text += line(0, '}', '')
        text += line(0, 'MsgBuffer', ';')
        text += '\n'
        return text


    @staticmethod
    def parse_constants(msg_list):
        text = ''

        for msg in msg_list:
            for name, data in msg.constants.items():
                data_type = MSG_TYPES[data['type']]
                data_name = underscore('MSG' + '_' + msg.name + '_' + name).upper()
                data_value = data['value']
                text += line(0, 'static', 'const', data_type, data_name, '=', data_value, ';')

        return text


def generate_msg_header(msg_list, template_text):
    text = template_text

    text = text.replace('STRUCTURE_PLACEHOLDER\n', Msg.parse_structure(msg_list))
    text = text.replace('BUFFER_PLACEHOLDER\n', Msg.parse_buffer(msg_list))
    text = text.replace('CONSTANTS_PLACEHOLDER\n', Msg.parse_constants(msg_list))

    return text


def main():
    msg_header_path = sys.argv[1]
    msg_file_list = sys.argv[2:]

    msg_list = Msg.parse(msg_file_list)

    text = generate_msg_header(msg_list, msg_header_template)
    with open(msg_header_path, 'w') as f:
        f.write(text)


if __name__ == '__main__':
    main()
