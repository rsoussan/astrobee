#!/usr/bin/python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
import csv
import os
import shutil
import sys

# TODO: unify these with loc analysis, move to loc utils! (added separator, test param sweep)
def check_and_fill_line(value_map, config_file_line, separator):
    line_strings = config_file_line.split(separator)
    # Overwrite val if config variable is in value map
    if len(line_strings) > 0 and line_strings[0] in value_map:
        return line_strings[0] + separator + str(value_map[line_strings[0]]) + "\n"
    return config_file_line


def fill_in_values(original_config, value_map, new_config, separator):
    original_config_file = open(original_config, "r")
    new_config_file = open(new_config, "w")
    for config_file_line in original_config_file:
        new_config_file.write(check_and_fill_line(value_map, config_file_line, separator))


def make_value_map(values, value_names):
    value_map = {}
    if len(values) != len(value_names):
        print("values and value_names not same length!")
        exit()

    for index, value_name in enumerate(value_names):
        value_map[value_name] = values[index]

    return value_map


def make_config(values, value_names, original_config, new_config, separator):
    value_map = make_value_map(values, value_names)
    fill_in_values(original_config, value_map, new_config, separator)

# TODO: move this to loc common!
def read_value(config_filename, value_name):
    config_file = open(config_filename, "r")
    value = None
    for config_file_line in config_file:
        line_strings = config_file_line.split("=")
        if len(line_strings) > 0 and line_strings[0] == value_name:
            # Remove trailing newline character if it exists
            value = line_strings[1].rstrip("\n")

    return value

# TODO: move to loc common utils!!!
def subdirectories(directory):
    subdirectories = []
    try:
        _, subdirectories, _ = next(os.walk(directory))
    except:
        pass
    return subdirectories

def check_multiple_sequence_numbers(image_directories, sequence_numbers_list):
    for image_directory, sequence_numbers in zip(image_directories, sequence_numbers_list):
        check_sequence_numbers(image_directory, sequence_numbers)

def check_sequence_numbers(image_directory, sequence_numbers):
    for sequence_number in sequence_numbers:
        if not os.path.isdir(os.path.join(image_directory, sequence_number)):
            print("Sequence number " + sequence_number + " subdirectory does not exist.")
            sys.exit()

def get_sequence_numbers(image_directory):
    subdirs = subdirectories(image_directory)
    sequence_numbers = []
    for subdirectory in subdirs:
        if subdirectory.isdigit():
            sequence_numbers.append(subdirectory)
    return sequence_numbers


# Mapping directory name should for example be images_a.0.1.3.images_b.2.4.mapping, for images_a sequences 0,1,3 and images_b sequences 2,4. 
def get_mapping_directory_name(image_directories, sequence_numbers_list):
    name = ""
    for image_directory, sequence_numbers in zip(image_directories, sequence_numbers_list):
        name += image_directory
        sequence_numbers.sort(key=int)
        for sequence_number in sequence_numbers:
            name += "." + sequence_number 
        name += "."
    name += "mapping"
    return name

def copy_image_directories(parent_directory, image_directories_absolute_paths, sequence_numbers_list):
    for image_directory_absolute_path, sequence_numbers in zip(image_directories_absolute_paths, sequence_numbers_list):
        copy_image_directory(parent_directory, image_directory_absolute_path, sequence_numbers)

def copy_image_directory(parent_directory, image_directory_absolute_path, sequence_numbers):
    image_directory = os.path.join(parent_directory, os.path.basename(image_directory_absolute_path))
    os.mkdir(image_directory)
    for sequence_number in sequence_numbers:
        shutil.copytree(os.path.join(image_directory_absolute_path, sequence_number), os.path.join(image_directory, sequence_number))

def save_image_directories_to_sequence_numbers(image_directories_list, sequence_numbers_list, output_file):
    with open(output_file, 'w') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=' ')
        for image_directory, sequence_numbers in zip(image_directories_list, sequence_numbers_list):
            image_directory_to_sequence_numbers = [image_directory] + sequence_numbers
            csv_writer.writerow(image_directory_to_sequence_numbers)

def image_sequences(image_sequences_file):
    image_directories = []
    sequence_numbers_list = []
    with open(image_sequences_file, 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=' ')
        for row in csv_reader:
            image_directories.append(row[0])
            sequence_numbers_list.append(row[1:])
    return image_directories, sequence_numbers_list
            
