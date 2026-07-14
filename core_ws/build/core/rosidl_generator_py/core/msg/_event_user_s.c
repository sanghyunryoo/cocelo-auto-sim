// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from core:msg/EventUser.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "core/msg/detail/event_user__struct.h"
#include "core/msg/detail/event_user__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool core__msg__event_user__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("core.msg._event_user.EventUser", full_classname_dest, 30) == 0);
  }
  core__msg__EventUser * ros_message = _ros_message;
  {  // estop
    PyObject * field = PyObject_GetAttrString(_pymsg, "estop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->estop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // wake
    PyObject * field = PyObject_GetAttrString(_pymsg, "wake");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->wake = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sleep
    PyObject * field = PyObject_GetAttrString(_pymsg, "sleep");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->sleep = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rough_drive_toggle
    PyObject * field = PyObject_GetAttrString(_pymsg, "rough_drive_toggle");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rough_drive_toggle = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * core__msg__event_user__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EventUser */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("core.msg._event_user");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EventUser");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  core__msg__EventUser * ros_message = (core__msg__EventUser *)raw_ros_message;
  {  // estop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->estop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "estop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wake
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->wake ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wake", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sleep
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->sleep ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sleep", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rough_drive_toggle
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rough_drive_toggle ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rough_drive_toggle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
