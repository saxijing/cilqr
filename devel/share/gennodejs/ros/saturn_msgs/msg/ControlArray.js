// Auto-generated. Do not edit!

// (in-package saturn_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Control = require('./Control.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ControlArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.control_lst = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('control_lst')) {
        this.control_lst = initObj.control_lst
      }
      else {
        this.control_lst = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [control_lst]
    // Serialize the length for message field [control_lst]
    bufferOffset = _serializer.uint32(obj.control_lst.length, buffer, bufferOffset);
    obj.control_lst.forEach((val) => {
      bufferOffset = Control.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlArray
    let len;
    let data = new ControlArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [control_lst]
    // Deserialize array length for message field [control_lst]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.control_lst = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.control_lst[i] = Control.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.control_lst.forEach((val) => {
      length += Control.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'saturn_msgs/ControlArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26a69d081791f88c3703af6f5549edf4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    Control[] control_lst
    
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
    MSG: saturn_msgs/Control
    std_msgs/Header header
    
    float64 u_accel
    float64 u_yawrate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.control_lst !== undefined) {
      resolved.control_lst = new Array(msg.control_lst.length);
      for (let i = 0; i < resolved.control_lst.length; ++i) {
        resolved.control_lst[i] = Control.Resolve(msg.control_lst[i]);
      }
    }
    else {
      resolved.control_lst = []
    }

    return resolved;
    }
};

module.exports = ControlArray;
