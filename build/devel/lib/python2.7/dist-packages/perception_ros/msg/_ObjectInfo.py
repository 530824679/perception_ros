# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from perception_ros/ObjectInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ObjectInfo(genpy.Message):
  _md5sum = "a84a83544761a25875cca8bb70b78846"
  _type = "perception_ros/ObjectInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint16                       id                 # The No. of object
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
  __slots__ = ['id','type','yaw','confidence','height','width','length','distance_xv','distance_yv','velocity_xv','velocity_yv','accelerate_xv','accelerate_yv','motion_state']
  _slot_types = ['uint16','uint8','uint8','uint8','uint16','uint16','uint16','int16','int16','int16','int16','int16','int16','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,type,yaw,confidence,height,width,length,distance_xv,distance_yv,velocity_xv,velocity_yv,accelerate_xv,accelerate_yv,motion_state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObjectInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.type is None:
        self.type = 0
      if self.yaw is None:
        self.yaw = 0
      if self.confidence is None:
        self.confidence = 0
      if self.height is None:
        self.height = 0
      if self.width is None:
        self.width = 0
      if self.length is None:
        self.length = 0
      if self.distance_xv is None:
        self.distance_xv = 0
      if self.distance_yv is None:
        self.distance_yv = 0
      if self.velocity_xv is None:
        self.velocity_xv = 0
      if self.velocity_yv is None:
        self.velocity_yv = 0
      if self.accelerate_xv is None:
        self.accelerate_xv = 0
      if self.accelerate_yv is None:
        self.accelerate_yv = 0
      if self.motion_state is None:
        self.motion_state = 0
    else:
      self.id = 0
      self.type = 0
      self.yaw = 0
      self.confidence = 0
      self.height = 0
      self.width = 0
      self.length = 0
      self.distance_xv = 0
      self.distance_yv = 0
      self.velocity_xv = 0
      self.velocity_yv = 0
      self.accelerate_xv = 0
      self.accelerate_yv = 0
      self.motion_state = 0

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
      buff.write(_get_struct_H3B3H6hB().pack(_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state,) = _get_struct_H3B3H6hB().unpack(str[start:end])
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
      buff.write(_get_struct_H3B3H6hB().pack(_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.id, _x.type, _x.yaw, _x.confidence, _x.height, _x.width, _x.length, _x.distance_xv, _x.distance_yv, _x.velocity_xv, _x.velocity_yv, _x.accelerate_xv, _x.accelerate_yv, _x.motion_state,) = _get_struct_H3B3H6hB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_H3B3H6hB = None
def _get_struct_H3B3H6hB():
    global _struct_H3B3H6hB
    if _struct_H3B3H6hB is None:
        _struct_H3B3H6hB = struct.Struct("<H3B3H6hB")
    return _struct_H3B3H6hB
