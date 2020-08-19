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

class pc_to_dsp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.flag_start_stop = null;
      this.V1 = null;
      this.V2 = null;
      this.V3 = null;
      this.V4 = null;
      this.SendData_State = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('flag_start_stop')) {
        this.flag_start_stop = initObj.flag_start_stop
      }
      else {
        this.flag_start_stop = 0;
      }
      if (initObj.hasOwnProperty('V1')) {
        this.V1 = initObj.V1
      }
      else {
        this.V1 = 0;
      }
      if (initObj.hasOwnProperty('V2')) {
        this.V2 = initObj.V2
      }
      else {
        this.V2 = 0;
      }
      if (initObj.hasOwnProperty('V3')) {
        this.V3 = initObj.V3
      }
      else {
        this.V3 = 0;
      }
      if (initObj.hasOwnProperty('V4')) {
        this.V4 = initObj.V4
      }
      else {
        this.V4 = 0;
      }
      if (initObj.hasOwnProperty('SendData_State')) {
        this.SendData_State = initObj.SendData_State
      }
      else {
        this.SendData_State = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pc_to_dsp
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [flag_start_stop]
    bufferOffset = _serializer.uint8(obj.flag_start_stop, buffer, bufferOffset);
    // Serialize message field [V1]
    bufferOffset = _serializer.int16(obj.V1, buffer, bufferOffset);
    // Serialize message field [V2]
    bufferOffset = _serializer.int16(obj.V2, buffer, bufferOffset);
    // Serialize message field [V3]
    bufferOffset = _serializer.int16(obj.V3, buffer, bufferOffset);
    // Serialize message field [V4]
    bufferOffset = _serializer.int16(obj.V4, buffer, bufferOffset);
    // Serialize message field [SendData_State]
    bufferOffset = _serializer.uint8(obj.SendData_State, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pc_to_dsp
    let len;
    let data = new pc_to_dsp(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [flag_start_stop]
    data.flag_start_stop = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [V1]
    data.V1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [V2]
    data.V2 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [V3]
    data.V3 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [V4]
    data.V4 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [SendData_State]
    data.SendData_State = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'const_msg/pc_to_dsp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '72744243899e1bab7337fbf15a335ac5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header Header
    uint8 flag_start_stop
    int16 V1
    int16 V2
    int16 V3
    int16 V4
    uint8 SendData_State
    
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
    const resolved = new pc_to_dsp(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.flag_start_stop !== undefined) {
      resolved.flag_start_stop = msg.flag_start_stop;
    }
    else {
      resolved.flag_start_stop = 0
    }

    if (msg.V1 !== undefined) {
      resolved.V1 = msg.V1;
    }
    else {
      resolved.V1 = 0
    }

    if (msg.V2 !== undefined) {
      resolved.V2 = msg.V2;
    }
    else {
      resolved.V2 = 0
    }

    if (msg.V3 !== undefined) {
      resolved.V3 = msg.V3;
    }
    else {
      resolved.V3 = 0
    }

    if (msg.V4 !== undefined) {
      resolved.V4 = msg.V4;
    }
    else {
      resolved.V4 = 0
    }

    if (msg.SendData_State !== undefined) {
      resolved.SendData_State = msg.SendData_State;
    }
    else {
      resolved.SendData_State = 0
    }

    return resolved;
    }
};

module.exports = pc_to_dsp;
