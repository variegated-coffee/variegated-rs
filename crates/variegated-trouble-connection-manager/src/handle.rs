use core::cell::RefCell;
use core::marker::PhantomData;
use trouble_host::prelude::*;
use trouble_host::PacketPool;

use crate::manager::BleConnectionManagerShared;
use crate::types::ConnectionState;

/// Handle to interact with the connection manager
pub struct ManagerHandle<'a, C: Controller, P: PacketPool> {
    pub(crate) shared: &'a RefCell<BleConnectionManagerShared<'a, P>>,
    pub(crate) _phantom: PhantomData<C>,
}

impl<'a, C: Controller, P: PacketPool> ManagerHandle<'a, C, P> {
    /// Create a ManagerHandle from a shared state reference
    ///
    /// This allows creating handles when you have access to the shared state
    /// but the connection manager is borrowed mutably elsewhere (e.g., for run()).
    pub fn from_shared(shared: &'a RefCell<BleConnectionManagerShared<'a, P>>) -> Self {
        Self {
            shared,
            _phantom: PhantomData,
        }
    }

    /// Register a device with the connection manager
    ///
    /// Returns a DeviceHandle that can be used to interact with the specific device.
    pub fn register_device(&self, address: BdAddr) -> DeviceHandle<'a, C, P> {
        DeviceHandle {
            address,
            shared: self.shared,
            _phantom: PhantomData,
        }
    }

    /// Drop a device from the manager entirely: stop maintaining it, disconnect it, and
    /// free its slot in the device table.
    ///
    /// See [`BleConnectionManagerShared::remove_device`] for why the disconnect matters.
    /// Prefer [`DeviceHandle::release`] where a handle is already in scope; this exists
    /// for callers that know an address but never held a handle for it.
    pub fn unregister_device(&self, address: BdAddr) {
        self.shared.borrow_mut().remove_device(address);
    }

    /// Every address currently in the device table, for auditing against the caller's
    /// own idea of what it has registered. See
    /// [`BleConnectionManagerShared::registered_addresses`].
    pub fn registered_addresses(&self) -> heapless::Vec<BdAddr, 8> {
        self.shared.borrow().registered_addresses()
    }
}

impl<'a, C: Controller, P: PacketPool> Clone for ManagerHandle<'a, C, P> {
    fn clone(&self) -> Self {
        Self {
            shared: self.shared,
            _phantom: PhantomData,
        }
    }
}

/// Handle for a specific BLE device
pub struct DeviceHandle<'a, C: Controller, P: PacketPool> {
    pub(crate) address: BdAddr,
    pub(crate) shared: &'a RefCell<BleConnectionManagerShared<'a, P>>,
    pub(crate) _phantom: PhantomData<C>,
}

impl<'a, C: Controller, P: PacketPool> DeviceHandle<'a, C, P> {
    /// Set whether the connection manager should maintain a connection to this device
    ///
    /// When set to `true`, the manager will:
    /// - Attempt to connect when the device is discovered/available
    /// - Automatically reconnect if the connection is lost
    pub async fn set_maintain_connection(&self, maintain: bool) {
        let mut shared = self.shared.borrow_mut();
        shared.set_maintain_connection(self.address, maintain);
    }

    /// Undo the registration: stop maintaining this device, drop its link, and free its
    /// slot in the manager's device table.
    ///
    /// See [`BleConnectionManagerShared::remove_device`] for why this disconnects rather
    /// than merely clearing the flag, and why leaving the entry behind is not harmless.
    ///
    /// A plain `fn`, unlike its `set_maintain_connection` sibling above, which is `async`
    /// without ever awaiting. That distinction is worth keeping: a caller tearing a
    /// device down in response to a cancelled future has no await point to spend, and the
    /// `RefCell` borrow here is confined to the call, so it cannot overlap the manager's
    /// own `borrow_mut` across a yield.
    pub fn release(&self) {
        self.shared.borrow_mut().remove_device(self.address);
    }

    /// Get the current connection state for this device
    pub async fn get_connection_state(&self) -> ConnectionState {
        let shared = self.shared.borrow();
        shared.get_connection_state(self.address)
    }

    /// Check if this device is currently connected
    pub async fn is_connected(&self) -> bool {
        self.get_connection_state().await == ConnectionState::Connected
    }

    /// Get the BLE address of this device
    pub fn address(&self) -> BdAddr {
        self.address
    }

    /// Execute a synchronous operation with the connection for this device
    ///
    /// The provided closure is called with a reference to the Connection if the device is connected.
    /// Returns `Ok(result)` if connected and operation succeeds, `Err(())` if not connected.
    ///
    /// Note: The closure must be synchronous. For async GATT operations, use `clone_connection()`
    /// to get an owned Connection that can be used across await points.
    pub fn with_connection<F, R>(&self, f: F) -> Result<R, ()>
    where
        F: FnOnce(&Connection<'a, P>) -> R,
    {
        let shared = self.shared.borrow();
        match shared.get_connection(self.address) {
            Some(conn) => Ok(f(conn)),
            None => Err(()),
        }
    }

    /// Clone the connection for this device
    ///
    /// Returns a cloned Connection if the device is currently connected, or None if not connected.
    /// The cloned Connection can be used for async GATT operations without holding RefCell borrows.
    ///
    /// Connection implements Clone with reference counting, so cloning is safe and efficient.
    pub fn clone_connection(&self) -> Option<Connection<'a, P>> {
        let shared = self.shared.borrow();
        shared.get_connection(self.address).cloned()
    }
}

impl<'a, C: Controller, P: PacketPool> Clone for DeviceHandle<'a, C, P> {
    fn clone(&self) -> Self {
        Self {
            address: self.address,
            shared: self.shared,
            _phantom: PhantomData,
        }
    }
}
