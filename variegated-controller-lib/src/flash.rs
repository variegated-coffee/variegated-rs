//! Adapter for handing a borrowed flash to sequential-storage.
//!
//! sequential-storage 7.0 replaced its free functions (`fetch_item`,
//! `store_item`, …) with `MapStorage`/`QueueStorage`, which *own* the flash for
//! their lifetime. Our storages all share one flash behind an
//! `embassy_sync::mutex::Mutex`, so what we actually have at each call site is a
//! `&mut T` borrowed from the guard.
//!
//! `embedded-storage-async` implements `ReadNorFlash`/`NorFlash` for `&mut T`,
//! which covers most of it — but not `MultiwriteNorFlash`, and
//! `MapStorage::remove_all_items` requires it. We cannot add that impl ourselves
//! either: it is a foreign marker trait, and `impl<T: MultiwriteNorFlash>
//! MultiwriteNorFlash for &mut T` leaves `T` uncovered, so the orphan rule
//! rejects it. Hence this newtype, which is a local type and can carry the impl.

use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash,
};

/// A `&mut` flash borrow that satisfies the owning sequential-storage API.
///
/// Construct one per call, hand it to `MapStorage::new`, and let it drop with
/// the storage; the mutex guard it borrows from outlives both.
pub struct BorrowedFlash<'a, T>(pub &'a mut T);

impl<T: ErrorType> ErrorType for BorrowedFlash<'_, T> {
    type Error = T::Error;
}

impl<T: ReadNorFlash> ReadNorFlash for BorrowedFlash<'_, T> {
    const READ_SIZE: usize = T::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl<T: NorFlash> NorFlash for BorrowedFlash<'_, T> {
    const WRITE_SIZE: usize = T::WRITE_SIZE;
    const ERASE_SIZE: usize = T::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.0.erase(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(offset, bytes).await
    }
}

impl<T: MultiwriteNorFlash> MultiwriteNorFlash for BorrowedFlash<'_, T> {}
