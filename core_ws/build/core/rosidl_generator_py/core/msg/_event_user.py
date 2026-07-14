# generated from rosidl_generator_py/resource/_idl.py.em
# with input from core:msg/EventUser.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EventUser(type):
    """Metaclass of message 'EventUser'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('core')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'core.msg.EventUser')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__event_user
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__event_user
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__event_user
            cls._TYPE_SUPPORT = module.type_support_msg__msg__event_user
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__event_user

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EventUser(metaclass=Metaclass_EventUser):
    """Message class 'EventUser'."""

    __slots__ = [
        '_estop',
        '_wake',
        '_sleep',
        '_rough_drive_toggle',
    ]

    _fields_and_field_types = {
        'estop': 'boolean',
        'wake': 'boolean',
        'sleep': 'boolean',
        'rough_drive_toggle': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.estop = kwargs.get('estop', bool())
        self.wake = kwargs.get('wake', bool())
        self.sleep = kwargs.get('sleep', bool())
        self.rough_drive_toggle = kwargs.get('rough_drive_toggle', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.estop != other.estop:
            return False
        if self.wake != other.wake:
            return False
        if self.sleep != other.sleep:
            return False
        if self.rough_drive_toggle != other.rough_drive_toggle:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def estop(self):
        """Message field 'estop'."""
        return self._estop

    @estop.setter
    def estop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'estop' field must be of type 'bool'"
        self._estop = value

    @builtins.property
    def wake(self):
        """Message field 'wake'."""
        return self._wake

    @wake.setter
    def wake(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wake' field must be of type 'bool'"
        self._wake = value

    @builtins.property
    def sleep(self):
        """Message field 'sleep'."""
        return self._sleep

    @sleep.setter
    def sleep(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sleep' field must be of type 'bool'"
        self._sleep = value

    @builtins.property
    def rough_drive_toggle(self):
        """Message field 'rough_drive_toggle'."""
        return self._rough_drive_toggle

    @rough_drive_toggle.setter
    def rough_drive_toggle(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rough_drive_toggle' field must be of type 'bool'"
        self._rough_drive_toggle = value
