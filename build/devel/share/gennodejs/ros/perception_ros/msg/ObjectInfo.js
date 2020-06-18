// Auto-generated. Do not edit!

// (in-package perception_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObjectInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.type = null;
      this.yaw = null;
      this.confidence = null;
      this.height = null;
      this.width = null;
      this.length = null;
      this.distance_xv = null;
      this.distance_yv = null;
      this.velocity_xv = null;
      this.velocity_yv = null;
      this.accelerate_xv = null;
      this.accelerate_yv = null;
      this.motion_state = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
      }
      if (initObj.hasOwnProperty('distance_xv')) {
        this.distance_xv = initObj.distance_xv
      }
      else {
        this.distance_xv = 0;
      }
      if (initObj.hasOwnProperty('distance_yv')) {
        this.distance_yv = initObj.distance_yv
      }
      else {
        this.distance_yv = 0;
      }
      if (initObj.hasOwnProperty('velocity_xv')) {
        this.velocity_xv = initObj.velocity_xv
      }
      else {
        this.velocity_xv = 0;
      }
      if (initObj.hasOwnProperty('velocity_yv')) {
        this.velocity_yv = initObj.velocity_yv
      }
      else {
        this.velocity_yv = 0;
      }
      if (initObj.hasOwnProperty('accelerate_xv')) {
        this.accelerate_xv = initObj.accelerate_xv
      }
      else {
        this.accelerate_xv = 0;
      }
      if (initObj.hasOwnProperty('accelerate_yv')) {
        this.accelerate_yv = initObj.accelerate_yv
      }
      else {
        this.accelerate_yv = 0;
      }
      if (initObj.hasOwnProperty('motion_state')) {
        this.motion_state = initObj.motion_state
      }
      else {
        this.motion_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectInfo
    // Serialize message field [id]
    bufferOffset = _serializer.uint16(obj.id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.uint8(obj.yaw, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.uint8(obj.confidence, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.uint16(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint16(obj.width, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.uint16(obj.length, buffer, bufferOffset);
    // Serialize message field [distance_xv]
    bufferOffset = _serializer.int16(obj.distance_xv, buffer, bufferOffset);
    // Serialize message field [distance_yv]
    bufferOffset = _serializer.int16(obj.distance_yv, buffer, bufferOffset);
    // Serialize message field [velocity_xv]
    bufferOffset = _serializer.int16(obj.velocity_xv, buffer, bufferOffset);
    // Serialize message field [velocity_yv]
    bufferOffset = _serializer.int16(obj.velocity_yv, buffer, bufferOffset);
    // Serialize message field [accelerate_xv]
    bufferOffset = _serializer.int16(obj.accelerate_xv, buffer, bufferOffset);
    // Serialize message field [accelerate_yv]
    bufferOffset = _serializer.int16(obj.accelerate_yv, buffer, bufferOffset);
    // Serialize message field [motion_state]
    bufferOffset = _serializer.uint8(obj.motion_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectInfo
    let len;
    let data = new ObjectInfo(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [distance_xv]
    data.distance_xv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [distance_yv]
    data.distance_yv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [velocity_xv]
    data.velocity_xv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [velocity_yv]
    data.velocity_yv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [accelerate_xv]
    data.accelerate_xv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [accelerate_yv]
    data.accelerate_yv = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motion_state]
    data.motion_state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'perception_ros/ObjectInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a84a83544761a25875cca8bb70b78846';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16                       id                 # The No. of object
    uint8                        type 		        # The category of object
    # unknow     0
    # pedestrain 1
    # motor      2
    # car        3
    # truck      4
    
    uint8                        yaw                # The orientation angle of object
    uint8                        confidence         # The confidence of object
    uint16                       height             # The height of object
    uint16                       width              # The width of object
    uint16                       length             # The length of object
    int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate
    int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate
    int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate
    int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate
    int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate
    int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate
    uint8                        motion_state       # The motion status of object
    # unknow     0
    # moving     1
    # stationary 2
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectInfo(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
    }

    if (msg.distance_xv !== undefined) {
      resolved.distance_xv = msg.distance_xv;
    }
    else {
      resolved.distance_xv = 0
    }

    if (msg.distance_yv !== undefined) {
      resolved.distance_yv = msg.distance_yv;
    }
    else {
      resolved.distance_yv = 0
    }

    if (msg.velocity_xv !== undefined) {
      resolved.velocity_xv = msg.velocity_xv;
    }
    else {
      resolved.velocity_xv = 0
    }

    if (msg.velocity_yv !== undefined) {
      resolved.velocity_yv = msg.velocity_yv;
    }
    else {
      resolved.velocity_yv = 0
    }

    if (msg.accelerate_xv !== undefined) {
      resolved.accelerate_xv = msg.accelerate_xv;
    }
    else {
      resolved.accelerate_xv = 0
    }

    if (msg.accelerate_yv !== undefined) {
      resolved.accelerate_yv = msg.accelerate_yv;
    }
    else {
      resolved.accelerate_yv = 0
    }

    if (msg.motion_state !== undefined) {
      resolved.motion_state = msg.motion_state;
    }
    else {
      resolved.motion_state = 0
    }

    return resolved;
    }
};

module.exports = ObjectInfo;
