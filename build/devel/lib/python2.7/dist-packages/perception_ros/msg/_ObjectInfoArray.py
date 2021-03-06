# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from perception_ros/ObjectInfoArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import perception_ros.msg
import std_msgs.msg

class ObjectInfoArray(genpy.Message):
  _md5sum = "37eb51ebc8f73aad80204a4eb7f1c89d"
  _type = "perception_ros/ObjectInfoArray"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header                       header                 # The message of header

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



"""
  __slots__ = ['header','object_info','object_num']
  _slot_types = ['std_msgs/Header','perception_ros/ObjectInfo[]','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,object_info,object_num

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObjectInfoArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.object_info is None:
        self.object_info = []
      if self.object_num is None:
        self.object_num = 0
    else:
      self.header = std_msgs.msg.Header()
      self.object_info = []
      self.object_num = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.object_info)
      buff.write(_struct_I.pack(length))
      for val1 in self.object_info:
        _x = val1
        buff.write(_get_struct_H3B3H6hB().pack(_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state))
      buff.write(_get_struct_H().pack(self.object_num))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.object_info is None:
        self.object_info = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object_info = []
      for i in range(0, length):
        val1 = perception_ros.msg.ObjectInfo()
        _x = val1
        start = end
        end += 24
        (_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state,) = _get_struct_H3B3H6hB().unpack(str[start:end])
        self.object_info.append(val1)
      start = end
      end += 2
      (self.object_num,) = _get_struct_H().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.object_info)
      buff.write(_struct_I.pack(length))
      for val1 in self.object_info:
        _x = val1
        buff.write(_get_struct_H3B3H6hB().pack(_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state))
      buff.write(_get_struct_H().pack(self.object_num))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.object_info is None:
        self.object_info = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object_info = []
      for i in range(0, length):
        val1 = perception_ros.msg.ObjectInfo()
        _x = val1
        start = end
        end += 24
        (_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state,) = _get_struct_H3B3H6hB().unpack(str[start:end])
        self.object_info.append(val1)
      start = end
      end += 2
      (self.object_num,) = _get_struct_H().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_H = None
def _get_struct_H():
    global _struct_H
    if _struct_H is None:
        _struct_H = struct.Struct("<H")
    return _struct_H
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_H3B3H6hB = None
def _get_struct_H3B3H6hB():
    global _struct_H3B3H6hB
    if _struct_H3B3H6hB is None:
        _struct_H3B3H6hB = struct.Struct("<H3B3H6hB")
    return _struct_H3B3H6hB
