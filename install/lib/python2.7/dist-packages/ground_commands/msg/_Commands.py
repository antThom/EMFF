# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ground_commands/Commands.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Commands(genpy.Message):
  _md5sum = "4d98a8c001152cf3f66becd92df597c9"
  _type = "ground_commands/Commands"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 X
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
float64 Mag_Z"""
  __slots__ = ['X','Y','Z','X_Dot','Y_Dot','Z_Dot','Psi','Theta','Phi','Psi_Dot','Theta_Dot','Phi_Dot','Mag_X','Mag_Y','Mag_Z']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       X,Y,Z,X_Dot,Y_Dot,Z_Dot,Psi,Theta,Phi,Psi_Dot,Theta_Dot,Phi_Dot,Mag_X,Mag_Y,Mag_Z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Commands, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.X is None:
        self.X = 0.
      if self.Y is None:
        self.Y = 0.
      if self.Z is None:
        self.Z = 0.
      if self.X_Dot is None:
        self.X_Dot = 0.
      if self.Y_Dot is None:
        self.Y_Dot = 0.
      if self.Z_Dot is None:
        self.Z_Dot = 0.
      if self.Psi is None:
        self.Psi = 0.
      if self.Theta is None:
        self.Theta = 0.
      if self.Phi is None:
        self.Phi = 0.
      if self.Psi_Dot is None:
        self.Psi_Dot = 0.
      if self.Theta_Dot is None:
        self.Theta_Dot = 0.
      if self.Phi_Dot is None:
        self.Phi_Dot = 0.
      if self.Mag_X is None:
        self.Mag_X = 0.
      if self.Mag_Y is None:
        self.Mag_Y = 0.
      if self.Mag_Z is None:
        self.Mag_Z = 0.
    else:
      self.X = 0.
      self.Y = 0.
      self.Z = 0.
      self.X_Dot = 0.
      self.Y_Dot = 0.
      self.Z_Dot = 0.
      self.Psi = 0.
      self.Theta = 0.
      self.Phi = 0.
      self.Psi_Dot = 0.
      self.Theta_Dot = 0.
      self.Phi_Dot = 0.
      self.Mag_X = 0.
      self.Mag_Y = 0.
      self.Mag_Z = 0.

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
      buff.write(_get_struct_15d().pack(_x.X, _x.Y, _x.Z, _x.X_Dot, _x.Y_Dot, _x.Z_Dot, _x.Psi, _x.Theta, _x.Phi, _x.Psi_Dot, _x.Theta_Dot, _x.Phi_Dot, _x.Mag_X, _x.Mag_Y, _x.Mag_Z))
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
      end += 120
      (_x.X, _x.Y, _x.Z, _x.X_Dot, _x.Y_Dot, _x.Z_Dot, _x.Psi, _x.Theta, _x.Phi, _x.Psi_Dot, _x.Theta_Dot, _x.Phi_Dot, _x.Mag_X, _x.Mag_Y, _x.Mag_Z,) = _get_struct_15d().unpack(str[start:end])
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
      buff.write(_get_struct_15d().pack(_x.X, _x.Y, _x.Z, _x.X_Dot, _x.Y_Dot, _x.Z_Dot, _x.Psi, _x.Theta, _x.Phi, _x.Psi_Dot, _x.Theta_Dot, _x.Phi_Dot, _x.Mag_X, _x.Mag_Y, _x.Mag_Z))
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
      end += 120
      (_x.X, _x.Y, _x.Z, _x.X_Dot, _x.Y_Dot, _x.Z_Dot, _x.Psi, _x.Theta, _x.Phi, _x.Psi_Dot, _x.Theta_Dot, _x.Phi_Dot, _x.Mag_X, _x.Mag_Y, _x.Mag_Z,) = _get_struct_15d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_15d = None
def _get_struct_15d():
    global _struct_15d
    if _struct_15d is None:
        _struct_15d = struct.Struct("<15d")
    return _struct_15d