use bitflags::bitflags;

bitflags! {
    /// Interrupts
    #[derive(Copy, Clone, Default, PartialEq, Debug)]
    pub struct Irq: u16 {
        const TxDone = 1;
        const RxDone = (1 << 1);
        const SyncwordValid = (1 << 2);
        const SyncwordError = (1 << 3);
        const HeaderValid = (1 << 4);
        const HeaderError = (1 << 5);
        const CrcError = (1 << 6);
        const RangingResponderResponseDone = (1 << 7);
        const RangingResponderRequestDiscarded = (1 << 8);
        const RangingInitiatorResultValid = (1 << 9);
        const RangingInitiatorResultTimeout = (1 << 10);
        const RangingResponderRequestValid = (1 << 11);
        const CadDone = (1 << 12);
        const CadActivityDetected = (1 << 13);
        const RxTxTimeout = (1 << 14);
        const PreambleDetected = (1 << 15);
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Irq {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Irq {{ 0b{0=0..16:016b} }}", self.bits())
    }
}

/// DIO IRQ mask
pub type DioMask = Irq;
