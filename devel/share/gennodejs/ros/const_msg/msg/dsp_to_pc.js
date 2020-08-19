// Auto-generated. Do not edit!

// (in-package const_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class dsp_to_pc {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.XDist = null;
      this.YDist = null;
      this.fAngle = null;
      this.Vx = null;
      this.Vy = null;
      this.dw = null;
      this.RecData_State = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('XDist')) {
        this.XDist = initObj.XDist
      }
      else {
        this.XDist = 0;
      }
      if (initObj.hasOwnProperty('YDist')) {
        this.YDist = initObj.YDist
      }
      else {
        this.YDist = 0;
      }
      if (initObj.hasOwnProperty('fAngle')) {
        this.fAngle = initObj.fAngle
      }
      else {
        this.fAngle = 0.0;
      }
      if (initObj.hasOwnProperty('Vx')) {
        this.Vx = initObj.Vx
      }
      else {
        this.Vx = 0;
      }
      if (initObj.hasOwnProperty('Vy')) {
        this.Vy = initObj.Vy
      }
      else {
        this.Vy = 0;
      }
      if (initObj.hasOwnProperty('dw')) {
        this.dw = initObj.dw
      }
      else {
        this.dw = 0;
      }
      if (initObj.hasOwnProperty('RecData_State')) {
        this.RecData_State = initObj.RecData_State
      }
      else {
        this.RecData_State = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dsp_to_pc
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [XDist]
    bufferOffset = _serializer.int32(obj.XDist, buffer, bufferOffset);
    // Serialize message field [YDist]
    bufferOffset = _serializer.int32(obj.YDist, buffer, bufferOffset);
    // Serialize message field [fAngle]
    bufferOffset = _serializer.float32(obj.fAngle, buffer, bufferOffset);
    // Serialize message field [Vx]
    bufferOffset = _serializer.int16(obj.Vx, buffer, bufferOffset);
    // Serialize message field [Vy]
    bufferOffset = _serializer.int16(obj.Vy, buffer, bufferOffset);
    // Serialize message field [dw]
    bufferOffset = _serializer.int16(obj.dw, buffer, bufferOffset);
    // Serialize message field [RecData_State]
    bufferOffset = _serializer.uint8(obj.RecData_State, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dsp_to_pc
    let len;
    let data = new dsp_to_pc(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [XDist]
    data.XDist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [YDist]
    data.YDist = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [fAngle]
    data.fAngle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Vx]
    data.Vx = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [Vy]
    data.Vy = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dw]
    data.dw = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [RecData_State]
    data.RecData_State = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'const_msg/dsp_to_pc';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5bfc4d9d173fac3e9f429b7263af1d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header Header
    int32 XDist
    int32 YDist
    float32 fAngle
    int16 Vx
    int16 Vy
    int16 dw
    uint8 RecData_State
     
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dsp_to_pc(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.XDist !== undefined) {
      resolved.XDist = msg.XDist;
    }
    else {
      resolved.XDist = 0
    }

    if (msg.YDist !== undefined) {
      resolved.YDist = msg.YDist;
    }
    else {
      resolved.YDist = 0
    }

    if (msg.fAngle !== undefined) {
      resolved.fAngle = msg.fAngle;
    }
    else {
      resolved.fAngle = 0.0
    }

    if (msg.Vx !== undefined) {
      resolved.Vx = msg.Vx;
    }
    else {
      resolved.Vx = 0
    }

    if (msg.Vy !== undefined) {
      resolved.Vy = msg.Vy;
    }
    else {
      resolved.Vy = 0
    }

    if (msg.dw !== undefined) {
      resolved.dw = msg.dw;
    }
    else {
      resolved.dw = 0
    }

    if (msg.RecData_State !== undefined) {
      resolved.RecData_State = msg.RecData_State;
    }
    else {
      resolved.RecData_State = 0
    }

    return resolved;
    }
};

module.exports = dsp_to_pc;
