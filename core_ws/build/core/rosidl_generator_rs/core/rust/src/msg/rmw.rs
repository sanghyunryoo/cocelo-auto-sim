#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "core__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__core__msg__EventUser() -> *const std::ffi::c_void;
}

#[link(name = "core__rosidl_generator_c")]
extern "C" {
    fn core__msg__EventUser__init(msg: *mut EventUser) -> bool;
    fn core__msg__EventUser__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EventUser>, size: usize) -> bool;
    fn core__msg__EventUser__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EventUser>);
    fn core__msg__EventUser__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EventUser>, out_seq: *mut rosidl_runtime_rs::Sequence<EventUser>) -> bool;
}

// Corresponds to core__msg__EventUser
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EventUser {

    // This member is not documented.
    #[allow(missing_docs)]
    pub estop: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wake: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sleep: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rough_drive_toggle: bool,

}



impl Default for EventUser {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !core__msg__EventUser__init(&mut msg as *mut _) {
        panic!("Call to core__msg__EventUser__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EventUser {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__EventUser__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__EventUser__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__EventUser__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EventUser {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EventUser where Self: Sized {
  const TYPE_NAME: &'static str = "core/msg/EventUser";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__core__msg__EventUser() }
  }
}


#[link(name = "core__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__core__msg__CommandUser() -> *const std::ffi::c_void;
}

#[link(name = "core__rosidl_generator_c")]
extern "C" {
    fn core__msg__CommandUser__init(msg: *mut CommandUser) -> bool;
    fn core__msg__CommandUser__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CommandUser>, size: usize) -> bool;
    fn core__msg__CommandUser__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CommandUser>);
    fn core__msg__CommandUser__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CommandUser>, out_seq: *mut rosidl_runtime_rs::Sequence<CommandUser>) -> bool;
}

// Corresponds to core__msg__CommandUser
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CommandUser {

    // This member is not documented.
    #[allow(missing_docs)]
    pub odom: nav_msgs::msg::rmw::Odometry,


    // This member is not documented.
    #[allow(missing_docs)]
    pub event: super::super::msg::rmw::EventUser,

}



impl Default for CommandUser {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !core__msg__CommandUser__init(&mut msg as *mut _) {
        panic!("Call to core__msg__CommandUser__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CommandUser {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__CommandUser__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__CommandUser__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { core__msg__CommandUser__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CommandUser {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CommandUser where Self: Sized {
  const TYPE_NAME: &'static str = "core/msg/CommandUser";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__core__msg__CommandUser() }
  }
}


