// Auto-generated. Do not edit!

// (in-package cartographer_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MetricLabel = require('./MetricLabel.js');
let HistogramBucket = require('./HistogramBucket.js');

//-----------------------------------------------------------

class Metric {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.labels = null;
      this.value = null;
      this.counts_by_bucket = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('labels')) {
        this.labels = initObj.labels
      }
      else {
        this.labels = [];
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
      if (initObj.hasOwnProperty('counts_by_bucket')) {
        this.counts_by_bucket = initObj.counts_by_bucket
      }
      else {
        this.counts_by_bucket = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Metric
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [labels]
    // Serialize the length for message field [labels]
    bufferOffset = _serializer.uint32(obj.labels.length, buffer, bufferOffset);
    obj.labels.forEach((val) => {
      bufferOffset = MetricLabel.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    // Serialize message field [counts_by_bucket]
    // Serialize the length for message field [counts_by_bucket]
    bufferOffset = _serializer.uint32(obj.counts_by_bucket.length, buffer, bufferOffset);
    obj.counts_by_bucket.forEach((val) => {
      bufferOffset = HistogramBucket.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Metric
    let len;
    let data = new Metric(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [labels]
    // Deserialize array length for message field [labels]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.labels = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.labels[i] = MetricLabel.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [counts_by_bucket]
    // Deserialize array length for message field [counts_by_bucket]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.counts_by_bucket = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.counts_by_bucket[i] = HistogramBucket.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.labels.forEach((val) => {
      length += MetricLabel.getMessageSize(val);
    });
    length += 16 * object.counts_by_bucket.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cartographer_ros_msgs/Metric';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94a6ea1c6ef245b483a220f6769c8e36';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 2018 The Cartographer Authors
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
    
    uint8 TYPE_COUNTER=0
    uint8 TYPE_GAUGE=1
    uint8 TYPE_HISTOGRAM=2
    
    uint8 type
    cartographer_ros_msgs/MetricLabel[] labels
    
    # TYPE_COUNTER or TYPE_GAUGE
    float64 value
    
    # TYPE_HISTOGRAM
    cartographer_ros_msgs/HistogramBucket[] counts_by_bucket
    
    ================================================================================
    MSG: cartographer_ros_msgs/MetricLabel
    # 2018 The Cartographer Authors
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
    
    string key
    string value
    
    ================================================================================
    MSG: cartographer_ros_msgs/HistogramBucket
    # 2018 The Cartographer Authors
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
    
    # A histogram bucket counts values x for which:
    #   previous_boundary < x <= bucket_boundary
    # holds.
    float64 bucket_boundary
    float64 count
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Metric(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.labels !== undefined) {
      resolved.labels = new Array(msg.labels.length);
      for (let i = 0; i < resolved.labels.length; ++i) {
        resolved.labels[i] = MetricLabel.Resolve(msg.labels[i]);
      }
    }
    else {
      resolved.labels = []
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    if (msg.counts_by_bucket !== undefined) {
      resolved.counts_by_bucket = new Array(msg.counts_by_bucket.length);
      for (let i = 0; i < resolved.counts_by_bucket.length; ++i) {
        resolved.counts_by_bucket[i] = HistogramBucket.Resolve(msg.counts_by_bucket[i]);
      }
    }
    else {
      resolved.counts_by_bucket = []
    }

    return resolved;
    }
};

// Constants for message
Metric.Constants = {
  TYPE_COUNTER: 0,
  TYPE_GAUGE: 1,
  TYPE_HISTOGRAM: 2,
}

module.exports = Metric;
