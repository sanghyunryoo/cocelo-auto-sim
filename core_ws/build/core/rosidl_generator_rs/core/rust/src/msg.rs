#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to core__msg__EventUser

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EventUser::default())
  }
}

impl rosidl_runtime_rs::Message for EventUser {
  type RmwMsg = super::msg::rmw::EventUser;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        estop: msg.estop,
        wake: msg.wake,
        sleep: msg.sleep,
        rough_drive_toggle: msg.rough_drive_toggle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      estop: msg.estop,
      wake: msg.wake,
      sleep: msg.sleep,
      rough_drive_toggle: msg.rough_drive_toggle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      estop: msg.estop,
      wake: msg.wake,
      sleep: msg.sleep,
      rough_drive_toggle: msg.rough_drive_toggle,
    }
  }
}


// Corresponds to core__msg__CommandUser

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CommandUser {

    // This member is not documented.
    #[allow(missing_docs)]
    pub odom: nav_msgs::msg::Odometry,


    // This member is not documented.
    #[allow(missing_docs)]
    pub event: super::msg::EventUser,

}



impl Default for CommandUser {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CommandUser::default())
  }
}

impl rosidl_runtime_rs::Message for CommandUser {
  type RmwMsg = super::msg::rmw::CommandUser;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        odom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Owned(msg.odom)).into_owned(),
        event: super::msg::EventUser::into_rmw_message(std::borrow::Cow::Owned(msg.event)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        odom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Borrowed(&msg.odom)).into_owned(),
        event: super::msg::EventUser::into_rmw_message(std::borrow::Cow::Borrowed(&msg.event)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      odom: nav_msgs::msg::Odometry::from_rmw_message(msg.odom),
      event: super::msg::EventUser::from_rmw_message(msg.event),
    }
  }
}


