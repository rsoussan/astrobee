/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef SPARSE_MAPPING_PROTOBUF_UTILITIES_H_
#define SPARSE_MAPPING_PROTOBUF_UTILITIES_H_

namespace sparse_mapping {
  // save size before protbuf, to save multiple protobufs in one file
  bool WriteProtobufTo(const google::protobuf::MessageLite& message,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput);

  // save size before file, then write file into rawOutput
  bool WriteFileTo(const char* filename,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput);

  // read size before protbuf, to save multiple protobufs in one file
  bool ReadProtobufFrom(google::protobuf::io::ZeroCopyInputStream* rawInput,
                      google::protobuf::MessageLite* message);

  // read a size before protobuf, then read a file and write it to filename
  bool ReadFileFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, const char* filename);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_PROTOBUF_UTILITIES_H_
