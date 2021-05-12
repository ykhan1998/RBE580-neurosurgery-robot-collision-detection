// Auto-generated. Do not edit!

// (in-package collide_free.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetPointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPointRequest
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float64(obj.pose, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPointRequest
    let len;
    let data = new SetPointRequest(null);
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.pose.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collide_free/SetPointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76249fb45cba333e6a82c7f91cfe1879';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPointRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = []
    }

    return resolved;
    }
};

class SetPointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wm = null;
      this.wh = null;
      this.t = null;
    }
    else {
      if (initObj.hasOwnProperty('wm')) {
        this.wm = initObj.wm
      }
      else {
        this.wm = [];
      }
      if (initObj.hasOwnProperty('wh')) {
        this.wh = initObj.wh
      }
      else {
        this.wh = [];
      }
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPointResponse
    // Serialize message field [wm]
    bufferOffset = _arraySerializer.float64(obj.wm, buffer, bufferOffset, null);
    // Serialize message field [wh]
    bufferOffset = _arraySerializer.float64(obj.wh, buffer, bufferOffset, null);
    // Serialize message field [t]
    bufferOffset = _arraySerializer.float64(obj.t, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPointResponse
    let len;
    let data = new SetPointResponse(null);
    // Deserialize message field [wm]
    data.wm = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [wh]
    data.wh = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [t]
    data.t = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.wm.length;
    length += 8 * object.wh.length;
    length += 8 * object.t.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collide_free/SetPointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '217099dae91577c60de7e73dd2aa8c7c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] wm
    float64[] wh
    float64[] t
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPointResponse(null);
    if (msg.wm !== undefined) {
      resolved.wm = msg.wm;
    }
    else {
      resolved.wm = []
    }

    if (msg.wh !== undefined) {
      resolved.wh = msg.wh;
    }
    else {
      resolved.wh = []
    }

    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SetPointRequest,
  Response: SetPointResponse,
  md5sum() { return '1d719385decc796c11ed1d75b56878fc'; },
  datatype() { return 'collide_free/SetPoint'; }
};
