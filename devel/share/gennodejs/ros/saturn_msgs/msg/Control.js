// Auto-generated. Do not edit!

// (in-package saturn_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.u_accel = null;
      this.u_yawrate = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('u_accel')) {
        this.u_accel = initObj.u_accel
      }
      else {
        this.u_accel = 0.0;
      }
      if (initObj.hasOwnProperty('u_yawrate')) {
        this.u_yawrate = initObj.u_yawrate
      }
      else {
        this.u_yawrate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Control
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [u_accel]
    bufferOffset = _serializer.float64(obj.u_accel, buffer, bufferOffset);
    // Serialize message field [u_yawrate]
    bufferOffset = _serializer.float64(obj.u_yawrate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Control
    let len;
    let data = new Control(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [u_accel]
    data.u_accel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u_yawrate]
    data.u_yawrate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'saturn_msgs/Control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f42f5551e89d8349e570aae4222e3836';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64 u_accel
    float64 u_yawrate
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Control(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.u_accel !== undefined) {
      resolved.u_accel = msg.u_accel;
    }
    else {
      resolved.u_accel = 0.0
    }

    if (msg.u_yawrate !== undefined) {
      resolved.u_yawrate = msg.u_yawrate;
    }
    else {
      resolved.u_yawrate = 0.0
    }

    return resolved;
    }
};

module.exports = Control;
