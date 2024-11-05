// Auto-generated. Do not edit!

// (in-package cartographer_ros_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let StatusResponse = require('../msg/StatusResponse.js');

//-----------------------------------------------------------

class StartTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.configuration_directory = null;
      this.configuration_basename = null;
      this.use_initial_pose = null;
      this.initial_pose = null;
      this.relative_to_trajectory_id = null;
    }
    else {
      if (initObj.hasOwnProperty('configuration_directory')) {
        this.configuration_directory = initObj.configuration_directory
      }
      else {
        this.configuration_directory = '';
      }
      if (initObj.hasOwnProperty('configuration_basename')) {
        this.configuration_basename = initObj.configuration_basename
      }
      else {
        this.configuration_basename = '';
      }
      if (initObj.hasOwnProperty('use_initial_pose')) {
        this.use_initial_pose = initObj.use_initial_pose
      }
      else {
        this.use_initial_pose = false;
      }
      if (initObj.hasOwnProperty('initial_pose')) {
        this.initial_pose = initObj.initial_pose
      }
      else {
        this.initial_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('relative_to_trajectory_id')) {
        this.relative_to_trajectory_id = initObj.relative_to_trajectory_id
      }
      else {
        this.relative_to_trajectory_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartTrajectoryRequest
    // Serialize message field [configuration_directory]
    bufferOffset = _serializer.string(obj.configuration_directory, buffer, bufferOffset);
    // Serialize message field [configuration_basename]
    bufferOffset = _serializer.string(obj.configuration_basename, buffer, bufferOffset);
    // Serialize message field [use_initial_pose]
    bufferOffset = _serializer.bool(obj.use_initial_pose, buffer, bufferOffset);
    // Serialize message field [initial_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.initial_pose, buffer, bufferOffset);
    // Serialize message field [relative_to_trajectory_id]
    bufferOffset = _serializer.int32(obj.relative_to_trajectory_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartTrajectoryRequest
    let len;
    let data = new StartTrajectoryRequest(null);
    // Deserialize message field [configuration_directory]
    data.configuration_directory = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [configuration_basename]
    data.configuration_basename = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [use_initial_pose]
    data.use_initial_pose = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [initial_pose]
    data.initial_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [relative_to_trajectory_id]
    data.relative_to_trajectory_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.configuration_directory);
    length += _getByteLength(object.configuration_basename);
    return length + 69;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cartographer_ros_msgs/StartTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '555a1aa894dfd093eaa13b245b423df8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Copyright 2016 The Cartographer Authors
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #      http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    string configuration_directory
    string configuration_basename
    bool use_initial_pose
    geometry_msgs/Pose initial_pose
    int32 relative_to_trajectory_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartTrajectoryRequest(null);
    if (msg.configuration_directory !== undefined) {
      resolved.configuration_directory = msg.configuration_directory;
    }
    else {
      resolved.configuration_directory = ''
    }

    if (msg.configuration_basename !== undefined) {
      resolved.configuration_basename = msg.configuration_basename;
    }
    else {
      resolved.configuration_basename = ''
    }

    if (msg.use_initial_pose !== undefined) {
      resolved.use_initial_pose = msg.use_initial_pose;
    }
    else {
      resolved.use_initial_pose = false
    }

    if (msg.initial_pose !== undefined) {
      resolved.initial_pose = geometry_msgs.msg.Pose.Resolve(msg.initial_pose)
    }
    else {
      resolved.initial_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.relative_to_trajectory_id !== undefined) {
      resolved.relative_to_trajectory_id = msg.relative_to_trajectory_id;
    }
    else {
      resolved.relative_to_trajectory_id = 0
    }

    return resolved;
    }
};

class StartTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.trajectory_id = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new StatusResponse();
      }
      if (initObj.hasOwnProperty('trajectory_id')) {
        this.trajectory_id = initObj.trajectory_id
      }
      else {
        this.trajectory_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartTrajectoryResponse
    // Serialize message field [status]
    bufferOffset = StatusResponse.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [trajectory_id]
    bufferOffset = _serializer.int32(obj.trajectory_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartTrajectoryResponse
    let len;
    let data = new StartTrajectoryResponse(null);
    // Deserialize message field [status]
    data.status = StatusResponse.deserialize(buffer, bufferOffset);
    // Deserialize message field [trajectory_id]
    data.trajectory_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += StatusResponse.getMessageSize(object.status);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cartographer_ros_msgs/StartTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a14602d76d9b734b374a25be319cdbe9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    cartographer_ros_msgs/StatusResponse status
    int32 trajectory_id
    
    
    ================================================================================
    MSG: cartographer_ros_msgs/StatusResponse
    # Copyright 2018 The Cartographer Authors
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #      http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    # A common message type to indicate the outcome of a service call.
    uint8 code
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartTrajectoryResponse(null);
    if (msg.status !== undefined) {
      resolved.status = StatusResponse.Resolve(msg.status)
    }
    else {
      resolved.status = new StatusResponse()
    }

    if (msg.trajectory_id !== undefined) {
      resolved.trajectory_id = msg.trajectory_id;
    }
    else {
      resolved.trajectory_id = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: StartTrajectoryRequest,
  Response: StartTrajectoryResponse,
  md5sum() { return 'dcc000df748d283ba7bf678a47ffa491'; },
  datatype() { return 'cartographer_ros_msgs/StartTrajectory'; }
};
