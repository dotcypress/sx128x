use super::*;

/// GFSK operating mode configuration
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GfskModulationParams {
    pub bitrate: BitrateBandwidth,
    pub modulation_index: ModulationIndex,
    pub modulation_shaping: ModulationShaping,
}

impl GfskModulationParams {
    pub(crate) fn as_bytes(&self) -> [u8; 3] {
        [
            self.bitrate as u8,
            self.modulation_index as u8,
            self.modulation_shaping as u8,
        ]
    }
}

/// GFSK sync word length
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GfskSyncWordLength {
    Length1Byte = 0x00,
    Length2Byte = 0x02,
    Length3Byte = 0x04,
    Length4Byte = 0x06,
    #[default]
    Length5Byte = 0x08,
}

/// Crc mode ащк GFSK
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GfskCrcMode {
    CrcOff = 0x00,
    #[default]
    Crc8Bit = 0x10,
    Crc16Bit = 0x20,
}

/// GFSK packet params
#[derive(Clone, Copy, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GfskPacketParams {
    pub preamble_length: PreambleLength,
    pub sync_word_length: GfskSyncWordLength,
    pub sync_word_match: SyncWordRxMatch,
    pub header_type: PacketHeaderType,
    pub payload_length: u8,
    pub crc_mode: GfskCrcMode,
    pub whitening: WhiteningMode,
}

impl GfskPacketParams {
    pub(crate) fn as_bytes(&self) -> [u8; 7] {
        [
            self.preamble_length as u8,
            self.sync_word_length as u8,
            self.sync_word_match as u8,
            self.header_type as u8,
            self.payload_length,
            self.crc_mode as u8,
            self.whitening as u8,
        ]
    }
}

/// GFSK modem params
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GfskModem {
    pub frequency: Frequency,
    pub tx_params: TxParams,
    pub modulation_params: GfskModulationParams,
    pub packet_params: GfskPacketParams,
    pub sync_word: [u8; 15],
}
