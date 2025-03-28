// Auto-generated. Do not edit!

// (in-package saturn_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let StateLite = require('./StateLite.js');
let Size = require('./Size.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObstacleState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.name = null;
      this.predicted_states = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('predicted_states')) {
        this.predicted_states = initObj.predicted_states
      }
      else {
        this.predicted_states = [];
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new Size();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [predicted_states]
    // Serialize the length for message field [predicted_states]
    bufferOffset = _serializer.uint32(obj.predicted_states.length, buffer, bufferOffset);
    obj.predicted_states.forEach((val) => {
      bufferOffset = StateLite.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [size]
    bufferOffset = Size.serialize(obj.size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleState
    let len;
    let data = new ObstacleState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [predicted_states]
    // Deserialize array length for message field [predicted_states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.predicted_states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.predicted_states[i] = StateLite.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [size]
    data.size = Size.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    object.predicted_states.forEach((val) => {
      length += StateLite.getMessageSize(val);
    });
    length += Size.getMessageSize(object.size);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'saturn_msgs/ObstacleState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '206a275054569710e712258c18396ed6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int32 id
    string name
    StateLite[] predicted_states
    Size size
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: saturn_msgs/StateLite
    std_msgs/Header header
    
    float64 x
    float64 y
    float64 theta
    float64 v
    float64 accel
    float64 yawrate
    
    ================================================================================
    MSG: saturn_msgs/Size
    std_msgs/Header header
    
    float64 length
    float64 width
    float64 height
    float64 wheel_base
    float64 wheel_track
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObstacleState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.predicted_states !== undefined) {
      resolved.predicted_states = new Array(msg.predicted_states.length);
      for (let i = 0; i < resolved.predicted_states.length; ++i) {
        resolved.predicted_states[i] = StateLite.Resolve(msg.predicted_states[i]);
      }
    }
    else {
      resolved.predicted_states = []
    }

    if (msg.size !== undefined) {
      resolved.size = Size.Resolve(msg.size)
    }
    else {
      resolved.size = new Size()
    }

    return resolved;
    }
};

module.exports = ObstacleState;
