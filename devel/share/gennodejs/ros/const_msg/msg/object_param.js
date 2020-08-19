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

class object_param {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.obj_angle = null;
      this.obj_distance = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('obj_angle')) {
        this.obj_angle = initObj.obj_angle
      }
      else {
        this.obj_angle = 0.0;
      }
      if (initObj.hasOwnProperty('obj_distance')) {
        this.obj_distance = initObj.obj_distance
      }
      else {
        this.obj_distance = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type object_param
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [obj_angle]
    bufferOffset = _serializer.float64(obj.obj_angle, buffer, bufferOffset);
    // Serialize message field [obj_distance]
    bufferOffset = _serializer.int64(obj.obj_distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type object_param
    let len;
    let data = new object_param(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [obj_angle]
    data.obj_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [obj_distance]
    data.obj_distance = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'const_msg/object_param';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6cb701956f8e3917f84ab2b8286c0409';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header Header
    float64 obj_angle
    int64 obj_distance
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
    const resolved = new object_param(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.obj_angle !== undefined) {
      resolved.obj_angle = msg.obj_angle;
    }
    else {
      resolved.obj_angle = 0.0
    }

    if (msg.obj_distance !== undefined) {
      resolved.obj_distance = msg.obj_distance;
    }
    else {
      resolved.obj_distance = 0
    }

    return resolved;
    }
};

module.exports = object_param;
