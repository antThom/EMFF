// Auto-generated. Do not edit!

// (in-package ground_commands.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Commands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.X = null;
      this.Y = null;
      this.Z = null;
      this.X_Dot = null;
      this.Y_Dot = null;
      this.Z_Dot = null;
      this.Psi = null;
      this.Theta = null;
      this.Phi = null;
      this.Psi_Dot = null;
      this.Theta_Dot = null;
      this.Phi_Dot = null;
      this.Mag_X = null;
      this.Mag_Y = null;
      this.Mag_Z = null;
    }
    else {
      if (initObj.hasOwnProperty('X')) {
        this.X = initObj.X
      }
      else {
        this.X = 0.0;
      }
      if (initObj.hasOwnProperty('Y')) {
        this.Y = initObj.Y
      }
      else {
        this.Y = 0.0;
      }
      if (initObj.hasOwnProperty('Z')) {
        this.Z = initObj.Z
      }
      else {
        this.Z = 0.0;
      }
      if (initObj.hasOwnProperty('X_Dot')) {
        this.X_Dot = initObj.X_Dot
      }
      else {
        this.X_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Y_Dot')) {
        this.Y_Dot = initObj.Y_Dot
      }
      else {
        this.Y_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Z_Dot')) {
        this.Z_Dot = initObj.Z_Dot
      }
      else {
        this.Z_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Psi')) {
        this.Psi = initObj.Psi
      }
      else {
        this.Psi = 0.0;
      }
      if (initObj.hasOwnProperty('Theta')) {
        this.Theta = initObj.Theta
      }
      else {
        this.Theta = 0.0;
      }
      if (initObj.hasOwnProperty('Phi')) {
        this.Phi = initObj.Phi
      }
      else {
        this.Phi = 0.0;
      }
      if (initObj.hasOwnProperty('Psi_Dot')) {
        this.Psi_Dot = initObj.Psi_Dot
      }
      else {
        this.Psi_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Theta_Dot')) {
        this.Theta_Dot = initObj.Theta_Dot
      }
      else {
        this.Theta_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Phi_Dot')) {
        this.Phi_Dot = initObj.Phi_Dot
      }
      else {
        this.Phi_Dot = 0.0;
      }
      if (initObj.hasOwnProperty('Mag_X')) {
        this.Mag_X = initObj.Mag_X
      }
      else {
        this.Mag_X = 0.0;
      }
      if (initObj.hasOwnProperty('Mag_Y')) {
        this.Mag_Y = initObj.Mag_Y
      }
      else {
        this.Mag_Y = 0.0;
      }
      if (initObj.hasOwnProperty('Mag_Z')) {
        this.Mag_Z = initObj.Mag_Z
      }
      else {
        this.Mag_Z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Commands
    // Serialize message field [X]
    bufferOffset = _serializer.float64(obj.X, buffer, bufferOffset);
    // Serialize message field [Y]
    bufferOffset = _serializer.float64(obj.Y, buffer, bufferOffset);
    // Serialize message field [Z]
    bufferOffset = _serializer.float64(obj.Z, buffer, bufferOffset);
    // Serialize message field [X_Dot]
    bufferOffset = _serializer.float64(obj.X_Dot, buffer, bufferOffset);
    // Serialize message field [Y_Dot]
    bufferOffset = _serializer.float64(obj.Y_Dot, buffer, bufferOffset);
    // Serialize message field [Z_Dot]
    bufferOffset = _serializer.float64(obj.Z_Dot, buffer, bufferOffset);
    // Serialize message field [Psi]
    bufferOffset = _serializer.float64(obj.Psi, buffer, bufferOffset);
    // Serialize message field [Theta]
    bufferOffset = _serializer.float64(obj.Theta, buffer, bufferOffset);
    // Serialize message field [Phi]
    bufferOffset = _serializer.float64(obj.Phi, buffer, bufferOffset);
    // Serialize message field [Psi_Dot]
    bufferOffset = _serializer.float64(obj.Psi_Dot, buffer, bufferOffset);
    // Serialize message field [Theta_Dot]
    bufferOffset = _serializer.float64(obj.Theta_Dot, buffer, bufferOffset);
    // Serialize message field [Phi_Dot]
    bufferOffset = _serializer.float64(obj.Phi_Dot, buffer, bufferOffset);
    // Serialize message field [Mag_X]
    bufferOffset = _serializer.float64(obj.Mag_X, buffer, bufferOffset);
    // Serialize message field [Mag_Y]
    bufferOffset = _serializer.float64(obj.Mag_Y, buffer, bufferOffset);
    // Serialize message field [Mag_Z]
    bufferOffset = _serializer.float64(obj.Mag_Z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Commands
    let len;
    let data = new Commands(null);
    // Deserialize message field [X]
    data.X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y]
    data.Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Z]
    data.Z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [X_Dot]
    data.X_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y_Dot]
    data.Y_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Z_Dot]
    data.Z_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Psi]
    data.Psi = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Theta]
    data.Theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Phi]
    data.Phi = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Psi_Dot]
    data.Psi_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Theta_Dot]
    data.Theta_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Phi_Dot]
    data.Phi_Dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Mag_X]
    data.Mag_X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Mag_Y]
    data.Mag_Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Mag_Z]
    data.Mag_Z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ground_commands/Commands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d98a8c001152cf3f66becd92df597c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 X
    float64 Y
    float64 Z
    float64 X_Dot
    float64 Y_Dot
    float64 Z_Dot
    float64 Psi
    float64 Theta
    float64 Phi
    float64 Psi_Dot
    float64 Theta_Dot
    float64 Phi_Dot
    float64 Mag_X
    float64 Mag_Y
    float64 Mag_Z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Commands(null);
    if (msg.X !== undefined) {
      resolved.X = msg.X;
    }
    else {
      resolved.X = 0.0
    }

    if (msg.Y !== undefined) {
      resolved.Y = msg.Y;
    }
    else {
      resolved.Y = 0.0
    }

    if (msg.Z !== undefined) {
      resolved.Z = msg.Z;
    }
    else {
      resolved.Z = 0.0
    }

    if (msg.X_Dot !== undefined) {
      resolved.X_Dot = msg.X_Dot;
    }
    else {
      resolved.X_Dot = 0.0
    }

    if (msg.Y_Dot !== undefined) {
      resolved.Y_Dot = msg.Y_Dot;
    }
    else {
      resolved.Y_Dot = 0.0
    }

    if (msg.Z_Dot !== undefined) {
      resolved.Z_Dot = msg.Z_Dot;
    }
    else {
      resolved.Z_Dot = 0.0
    }

    if (msg.Psi !== undefined) {
      resolved.Psi = msg.Psi;
    }
    else {
      resolved.Psi = 0.0
    }

    if (msg.Theta !== undefined) {
      resolved.Theta = msg.Theta;
    }
    else {
      resolved.Theta = 0.0
    }

    if (msg.Phi !== undefined) {
      resolved.Phi = msg.Phi;
    }
    else {
      resolved.Phi = 0.0
    }

    if (msg.Psi_Dot !== undefined) {
      resolved.Psi_Dot = msg.Psi_Dot;
    }
    else {
      resolved.Psi_Dot = 0.0
    }

    if (msg.Theta_Dot !== undefined) {
      resolved.Theta_Dot = msg.Theta_Dot;
    }
    else {
      resolved.Theta_Dot = 0.0
    }

    if (msg.Phi_Dot !== undefined) {
      resolved.Phi_Dot = msg.Phi_Dot;
    }
    else {
      resolved.Phi_Dot = 0.0
    }

    if (msg.Mag_X !== undefined) {
      resolved.Mag_X = msg.Mag_X;
    }
    else {
      resolved.Mag_X = 0.0
    }

    if (msg.Mag_Y !== undefined) {
      resolved.Mag_Y = msg.Mag_Y;
    }
    else {
      resolved.Mag_Y = 0.0
    }

    if (msg.Mag_Z !== undefined) {
      resolved.Mag_Z = msg.Mag_Z;
    }
    else {
      resolved.Mag_Z = 0.0
    }

    return resolved;
    }
};

module.exports = Commands;
