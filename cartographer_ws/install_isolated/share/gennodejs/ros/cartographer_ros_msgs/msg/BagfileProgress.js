// Auto-generated. Do not edit!

// (in-package cartographer_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BagfileProgress {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current_bagfile_name = null;
      this.current_bagfile_id = null;
      this.total_bagfiles = null;
      this.total_messages = null;
      this.processed_messages = null;
      this.total_seconds = null;
      this.processed_seconds = null;
    }
    else {
      if (initObj.hasOwnProperty('current_bagfile_name')) {
        this.current_bagfile_name = initObj.current_bagfile_name
      }
      else {
        this.current_bagfile_name = '';
      }
      if (initObj.hasOwnProperty('current_bagfile_id')) {
        this.current_bagfile_id = initObj.current_bagfile_id
      }
      else {
        this.current_bagfile_id = 0;
      }
      if (initObj.hasOwnProperty('total_bagfiles')) {
        this.total_bagfiles = initObj.total_bagfiles
      }
      else {
        this.total_bagfiles = 0;
      }
      if (initObj.hasOwnProperty('total_messages')) {
        this.total_messages = initObj.total_messages
      }
      else {
        this.total_messages = 0;
      }
      if (initObj.hasOwnProperty('processed_messages')) {
        this.processed_messages = initObj.processed_messages
      }
      else {
        this.processed_messages = 0;
      }
      if (initObj.hasOwnProperty('total_seconds')) {
        this.total_seconds = initObj.total_seconds
      }
      else {
        this.total_seconds = 0.0;
      }
      if (initObj.hasOwnProperty('processed_seconds')) {
        this.processed_seconds = initObj.processed_seconds
      }
      else {
        this.processed_seconds = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BagfileProgress
    // Serialize message field [current_bagfile_name]
    bufferOffset = _serializer.string(obj.current_bagfile_name, buffer, bufferOffset);
    // Serialize message field [current_bagfile_id]
    bufferOffset = _serializer.uint32(obj.current_bagfile_id, buffer, bufferOffset);
    // Serialize message field [total_bagfiles]
    bufferOffset = _serializer.uint32(obj.total_bagfiles, buffer, bufferOffset);
    // Serialize message field [total_messages]
    bufferOffset = _serializer.uint32(obj.total_messages, buffer, bufferOffset);
    // Serialize message field [processed_messages]
    bufferOffset = _serializer.uint32(obj.processed_messages, buffer, bufferOffset);
    // Serialize message field [total_seconds]
    bufferOffset = _serializer.float32(obj.total_seconds, buffer, bufferOffset);
    // Serialize message field [processed_seconds]
    bufferOffset = _serializer.float32(obj.processed_seconds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BagfileProgress
    let len;
    let data = new BagfileProgress(null);
    // Deserialize message field [current_bagfile_name]
    data.current_bagfile_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current_bagfile_id]
    data.current_bagfile_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [total_bagfiles]
    data.total_bagfiles = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [total_messages]
    data.total_messages = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [processed_messages]
    data.processed_messages = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [total_seconds]
    data.total_seconds = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [processed_seconds]
    data.processed_seconds = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.current_bagfile_name);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cartographer_ros_msgs/BagfileProgress';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a36f93b13e2b297d45098a38cb00510';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #
    # Licensed under the Apache License, Version 2.0 (the 'License');
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #      http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an 'AS IS' BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    
    # Contains general information about the bagfiles processing progress
    
    string current_bagfile_name
    uint32 current_bagfile_id
    uint32 total_bagfiles
    uint32 total_messages
    uint32 processed_messages
    float32 total_seconds
    float32 processed_seconds
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BagfileProgress(null);
    if (msg.current_bagfile_name !== undefined) {
      resolved.current_bagfile_name = msg.current_bagfile_name;
    }
    else {
      resolved.current_bagfile_name = ''
    }

    if (msg.current_bagfile_id !== undefined) {
      resolved.current_bagfile_id = msg.current_bagfile_id;
    }
    else {
      resolved.current_bagfile_id = 0
    }

    if (msg.total_bagfiles !== undefined) {
      resolved.total_bagfiles = msg.total_bagfiles;
    }
    else {
      resolved.total_bagfiles = 0
    }

    if (msg.total_messages !== undefined) {
      resolved.total_messages = msg.total_messages;
    }
    else {
      resolved.total_messages = 0
    }

    if (msg.processed_messages !== undefined) {
      resolved.processed_messages = msg.processed_messages;
    }
    else {
      resolved.processed_messages = 0
    }

    if (msg.total_seconds !== undefined) {
      resolved.total_seconds = msg.total_seconds;
    }
    else {
      resolved.total_seconds = 0.0
    }

    if (msg.processed_seconds !== undefined) {
      resolved.processed_seconds = msg.processed_seconds;
    }
    else {
      resolved.processed_seconds = 0.0
    }

    return resolved;
    }
};

module.exports = BagfileProgress;
