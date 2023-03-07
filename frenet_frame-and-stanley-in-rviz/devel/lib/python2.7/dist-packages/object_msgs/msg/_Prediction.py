# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from object_msgs/Prediction.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import object_msgs.msg
import std_msgs.msg

class Prediction(genpy.Message):
  _md5sum = "62eb5942cfcf183980a3aa9c64433045"
  _type = "object_msgs/Prediction"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
uint32 id

uint32 n_predictions
float32 dt  # s
object_msgs/Object[] predictions

float32[] sigx
float32[] sigy
float32[] rho
time t0  # time corresponding prediction[0]
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
MSG: object_msgs/Object
std_msgs/Header header
uint32 id

# The type of classification given to this object.
uint8 classification
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_CAR=1
uint8 CLASSIFICATION_PEDESTRIAN=2
uint8 CLASSIFICATION_CYCLIST=3

# The detected position and orientation of the object.
float32 x       # m
float32 y       # m
float32 yaw     # rad

float32 v       # m/s
float32 yawrate # rad/s

float32 a      # m/ss
float32 delta  # radian

# size
float32 L     # m
float32 W     # m
"""
  __slots__ = ['header','id','n_predictions','dt','predictions','sigx','sigy','rho','t0']
  _slot_types = ['std_msgs/Header','uint32','uint32','float32','object_msgs/Object[]','float32[]','float32[]','float32[]','time']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,n_predictions,dt,predictions,sigx,sigy,rho,t0

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Prediction, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = 0
      if self.n_predictions is None:
        self.n_predictions = 0
      if self.dt is None:
        self.dt = 0.
      if self.predictions is None:
        self.predictions = []
      if self.sigx is None:
        self.sigx = []
      if self.sigy is None:
        self.sigy = []
      if self.rho is None:
        self.rho = []
      if self.t0 is None:
        self.t0 = genpy.Time()
    else:
      self.header = std_msgs.msg.Header()
      self.id = 0
      self.n_predictions = 0
      self.dt = 0.
      self.predictions = []
      self.sigx = []
      self.sigy = []
      self.rho = []
      self.t0 = genpy.Time()

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
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2If().pack(_x.id, _x.n_predictions, _x.dt))
      length = len(self.predictions)
      buff.write(_struct_I.pack(length))
      for val1 in self.predictions:
        _v1 = val1.header
        _x = _v1.seq
        buff.write(_get_struct_I().pack(_x))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IB9f().pack(_x.id, _x.classification, _x.x, _x.y, _x.yaw, _x.v, _x.yawrate, _x.a, _x.delta, _x.L, _x.W))
      length = len(self.sigx)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.sigx))
      length = len(self.sigy)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.sigy))
      length = len(self.rho)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.rho))
      _x = self
      buff.write(_get_struct_2I().pack(_x.t0.secs, _x.t0.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.predictions is None:
        self.predictions = None
      if self.t0 is None:
        self.t0 = genpy.Time()
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
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.id, _x.n_predictions, _x.dt,) = _get_struct_2If().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.predictions = []
      for i in range(0, length):
        val1 = object_msgs.msg.Object()
        _v3 = val1.header
        start = end
        end += 4
        (_v3.seq,) = _get_struct_I().unpack(str[start:end])
        _v4 = _v3.stamp
        _x = _v4
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v3.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v3.frame_id = str[start:end]
        _x = val1
        start = end
        end += 41
        (_x.id, _x.classification, _x.x, _x.y, _x.yaw, _x.v, _x.yawrate, _x.a, _x.delta, _x.L, _x.W,) = _get_struct_IB9f().unpack(str[start:end])
        self.predictions.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.sigx = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.sigy = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rho = s.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.t0.secs, _x.t0.nsecs,) = _get_struct_2I().unpack(str[start:end])
      self.t0.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


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
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2If().pack(_x.id, _x.n_predictions, _x.dt))
      length = len(self.predictions)
      buff.write(_struct_I.pack(length))
      for val1 in self.predictions:
        _v5 = val1.header
        _x = _v5.seq
        buff.write(_get_struct_I().pack(_x))
        _v6 = _v5.stamp
        _x = _v6
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v5.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IB9f().pack(_x.id, _x.classification, _x.x, _x.y, _x.yaw, _x.v, _x.yawrate, _x.a, _x.delta, _x.L, _x.W))
      length = len(self.sigx)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.sigx.tostring())
      length = len(self.sigy)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.sigy.tostring())
      length = len(self.rho)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.rho.tostring())
      _x = self
      buff.write(_get_struct_2I().pack(_x.t0.secs, _x.t0.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.predictions is None:
        self.predictions = None
      if self.t0 is None:
        self.t0 = genpy.Time()
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
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.id, _x.n_predictions, _x.dt,) = _get_struct_2If().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.predictions = []
      for i in range(0, length):
        val1 = object_msgs.msg.Object()
        _v7 = val1.header
        start = end
        end += 4
        (_v7.seq,) = _get_struct_I().unpack(str[start:end])
        _v8 = _v7.stamp
        _x = _v8
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v7.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v7.frame_id = str[start:end]
        _x = val1
        start = end
        end += 41
        (_x.id, _x.classification, _x.x, _x.y, _x.yaw, _x.v, _x.yawrate, _x.a, _x.delta, _x.L, _x.W,) = _get_struct_IB9f().unpack(str[start:end])
        self.predictions.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.sigx = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.sigy = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rho = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 8
      (_x.t0.secs, _x.t0.nsecs,) = _get_struct_2I().unpack(str[start:end])
      self.t0.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_2If = None
def _get_struct_2If():
    global _struct_2If
    if _struct_2If is None:
        _struct_2If = struct.Struct("<2If")
    return _struct_2If
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_IB9f = None
def _get_struct_IB9f():
    global _struct_IB9f
    if _struct_IB9f is None:
        _struct_IB9f = struct.Struct("<IB9f")
    return _struct_IB9f
