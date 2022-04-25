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

#include <sparse_mapping/protobuf_utilities.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message_lite.h>

namespace sparse_mapping {
// save size before protbuf, to save multiple protobufs in one file
bool WriteProtobufTo(const google::protobuf::MessageLite& message,
                     google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
  google::protobuf::io::CodedOutputStream output(rawOutput);
  // Write the size.
  int size = message.ByteSize();
  output.WriteVarint32(size);

  uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
  if (buffer != NULL) {
    // Optimization:  The message fits in one buffer, so use the faster
    // direct-to-array serialization path.
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    // Slightly-slower path when the message is multiple buffers.
    message.SerializeWithCachedSizes(&output);
    if (output.HadError()) return false;
  }

  return true;
}

bool WriteFileTo(const char* filename, google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
  google::protobuf::io::CodedOutputStream output(rawOutput);

  FILE* f = fopen(filename, "r");
  if (f == NULL) return false;
  if (fseek(f, 0L, SEEK_END) != 0) return false;
  uint64_t size = ftell(f);
  rewind(f);

  // Write the size.
  output.WriteVarint32(size);

  char buffer[4096];
  uint64_t count = 0;
  while (true) {
    size_t len = fread(buffer, sizeof(char), 4096, f);
    count += len;
    output.WriteRaw(buffer, len);
    if (len != 4096) break;
  }
  fclose(f);
  if (count != size) {
    LOG(ERROR) << "Failed to write file to protobuf output stream.";
    return false;
  }

  return true;
}

// read size before protbuf, to save multiple protobufs in one file
bool ReadProtobufFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, google::protobuf::MessageLite* message) {
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  auto limit = input.PushLimit(size);

  // Parse the message.
  if (!message->MergePartialFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
}

// read size before protbuf, to save multiple protobufs in one file
bool ReadFileFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, const char* filename) {
  FILE* f = fopen(filename, "w");
  if (f == NULL) return false;
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  auto limit = input.PushLimit(size);

  char buffer[4096];
  unsigned int count = 0;
  while (count < size) {
    unsigned int bytes = std::min(4096u, size - count);
    if (!input.ReadRaw(buffer, bytes)) {
      LOG(ERROR) << "Failed to read file from protobuf.";
      fclose(f);
      return false;
    }
    if (fwrite(buffer, sizeof(char), bytes, f) != bytes) {
      LOG(ERROR) << "Failed to write file from protobuf.";
      fclose(f);
      return false;
    }
    count += bytes;
  }

  // Release the limit.
  input.PopLimit(limit);
  fclose(f);

  return true;
}
}  // namespace sparse_mapping
