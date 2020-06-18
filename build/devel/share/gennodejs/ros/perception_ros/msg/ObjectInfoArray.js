// Auto-generated. Do not edit!

// (in-package perception_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectInfo = require('./ObjectInfo.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjectInfoArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.object_info = null;
      this.object_num = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('object_info')) {
        this.object_info = initObj.object_info
      }
      else {
        this.object_info = [];
      }
      if (initObj.hasOwnProperty('object_num')) {
        this.object_num = initObj.object_num
      }
      else {
        this.object_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectInfoArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [object_info]
    // Serialize the length for message field [object_info]
    bufferOffset = _serializer.uint32(obj.object_info.length, buffer, bufferOffset);
    obj.object_info.forEach((val) => {
      bufferOffset = ObjectInfo.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [object_num]
    bufferOffset = _serializer.uint16(obj.object_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectInfoArray
    let len;
    let data = new ObjectInfoArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_info]
    // Deserialize array length for message field [object_info]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.object_info = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.object_info[i] = ObjectInfo.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [object_num]
    data.object_num = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.object_info.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'perception_ros/ObjectInfoArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37eb51ebc8f73aad80204a4eb7f1c89d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header                       header                 # The message of header
    
    ObjectInfo[]                 object_info            # The information of object
    uint16                       object_num             # The number of object
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
    
    ================================================================================
    MSG: perception_ros/ObjectInfo
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
    const resolved = new ObjectInfoArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.object_info !== undefined) {
      resolved.object_info = new Array(msg.object_info.length);
      for (let i = 0; i < resolved.object_info.length; ++i) {
        resolved.object_info[i] = ObjectInfo.Resolve(msg.object_info[i]);
      }
    }
    else {
      resolved.object_info = []
    }

    if (msg.object_num !== undefined) {
      resolved.object_num = msg.object_num;
    }
    else {
      resolved.object_num = 0
    }

    return resolved;
    }
};

module.exports = ObjectInfoArray;
