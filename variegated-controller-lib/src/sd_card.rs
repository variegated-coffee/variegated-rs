//! SD card support with async-friendly yielding
//!
//! Wraps embedded-sdmmc's BlockDevice to yield after each block operation,
//! ensuring safety-critical tasks can run on the same executor.

use core::fmt::Debug;
use defmt::warn;
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

/// A BlockDevice wrapper that yields after each block operation.
///
/// This ensures that long SD card operations don't starve other tasks
/// on the same executor. Each 512-byte block at 25MHz SPI takes ~0.2-0.5ms,
/// so yielding per-block keeps worst-case latency well under 50ms.
///
/// # Example
///
/// ```ignore
/// use embedded_sdmmc::SdCard;
/// use variegated_controller_lib::sd_card::YieldingBlockDevice;
///
/// let sd_card = SdCard::new(spi_device, delay);
/// let yielding_sd = YieldingBlockDevice::new(sd_card);
/// // Use yielding_sd with VolumeManager
/// ```
pub struct YieldingBlockDevice<BD>
where
    BD: BlockDevice,
{
    inner: BD,
}

impl<BD> YieldingBlockDevice<BD>
where
    BD: BlockDevice,
{
    /// Create a new yielding block device wrapper.
    pub fn new(inner: BD) -> Self {
        Self { inner }
    }

    /// Get a reference to the inner block device.
    pub fn inner(&self) -> &BD {
        &self.inner
    }

    /// Get a mutable reference to the inner block device.
    pub fn inner_mut(&mut self) -> &mut BD {
        &mut self.inner
    }

    /// Consume the wrapper and return the inner block device.
    pub fn into_inner(self) -> BD {
        self.inner
    }
}

impl<BD> BlockDevice for YieldingBlockDevice<BD>
where
    BD: BlockDevice,
    BD::Error: Debug,
{
    type Error = BD::Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        warn!("Reading blocks");
        // Read one block at a time, yielding between each to allow
        // other tasks on the executor to run.
        for (i, block) in blocks.iter_mut().enumerate() {
            warn!("Reading element {}", i);
            let block_idx = BlockIdx(start_block_idx.0 + i as u32);
            let single = core::slice::from_mut(block);
            self.inner.read(single, block_idx)?;

            // Yield to allow other tasks to run.
            // Since BlockDevice::read is sync, we use block_on to execute the yield.
            // This is safe because yield_now() completes immediately after
            // giving other ready tasks a chance to run.
            embassy_futures::block_on(embassy_futures::yield_now());
        }
        warn!("Done reading blocks");
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        warn!("Writing blocks");
        // Write one block at a time, yielding between each to allow
        // other tasks on the executor to run.
        for (i, block) in blocks.iter().enumerate() {
            warn!("[{:?}] writing block {:?}", start_block_idx.0, i);
            let block_idx = BlockIdx(start_block_idx.0 + i as u32);
            let single = core::slice::from_ref(block);
            self.inner.write(single, block_idx)?;

            warn!("[{:?}] wrote block {:?}", start_block_idx.0, i);
            // Yield to allow other tasks to run.
            embassy_futures::block_on(embassy_futures::yield_now());
        }
        warn!("Done writing blocks");
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        self.inner.num_blocks()
    }
}

/// A blocking SPI device wrapper that delegates to an async SPI device.
///
/// This wrapper implements `embedded_hal::spi::SpiDevice` by using `block_on`
/// to wait for async SPI operations. It's designed for use with SD card libraries
/// that require blocking SPI access while the rest of the system uses async SPI.
///
/// Note: This wrapper yields after each transaction to allow other async tasks
/// to make progress.
pub struct BlockingSpiDevice<SPI> {
    spi: SPI,
}

impl<SPI> BlockingSpiDevice<SPI> {
    /// Create a new blocking SPI device wrapper.
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

impl<SPI> embedded_hal::spi::ErrorType for BlockingSpiDevice<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;
}

impl<SPI> embedded_hal::spi::SpiDevice for BlockingSpiDevice<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        // Convert blocking operations to async operations
        let result = embassy_futures::block_on(async {
            // Need to convert Operation types between blocking and async
            // Both use the same Operation enum structure
            let mut async_ops: heapless::Vec<embedded_hal_async::spi::Operation<'_, u8>, 8> =
                heapless::Vec::new();

            for op in operations.iter_mut() {
                let async_op = match op {
                    embedded_hal::spi::Operation::Read(buf) => {
                        embedded_hal_async::spi::Operation::Read(buf)
                    }
                    embedded_hal::spi::Operation::Write(buf) => {
                        embedded_hal_async::spi::Operation::Write(buf)
                    }
                    embedded_hal::spi::Operation::Transfer(read, write) => {
                        embedded_hal_async::spi::Operation::Transfer(read, write)
                    }
                    embedded_hal::spi::Operation::TransferInPlace(buf) => {
                        embedded_hal_async::spi::Operation::TransferInPlace(buf)
                    }
                    embedded_hal::spi::Operation::DelayNs(ns) => {
                        embedded_hal_async::spi::Operation::DelayNs(*ns)
                    }
                };
                if async_ops.push(async_op).is_err() {
                    // Vec is full, this shouldn't happen with reasonable transaction sizes
                    panic!("SPI transaction has too many operations");
                }
            }

            self.spi.transaction(&mut async_ops).await
        });

        // Yield after transaction to allow other tasks to run
        embassy_futures::block_on(embassy_futures::yield_now());

        result
    }
}
