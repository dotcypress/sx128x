use super::*;

/// FLRC modulation params
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlrcModulationParams {
    pub bitrate: FlrcBitrate,
    pub coding_rate: FlrcCodingRate,
    pub modulation_shaping: ModulationShaping,
}

impl FlrcModulationParams {
    pub(crate) fn as_bytes(&self) -> [u8; 3] {
        [
            self.bitrate as u8,
            self.coding_rate as u8,
            self.modulation_shaping as u8,
        ]
    }
}

/// FLRC bitrate
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlrcBitrate {
    #[default]
    BR1300 = 0x45,
    BR1040 = 0x69,
    BR650 = 0x86,
    BR520 = 0xAA,
    BR325 = 0xC7,
    BR260 = 0xEB,
}

/// FLRC coding rate
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlrcCodingRate {
    Cr1_2 = 0x00,
    #[default]
    Cr3_4 = 0x02,
    Cr1_1 = 0x04,
}

/// FLRC sync word length
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlrcSyncWordLength {
    None = 0x00,
    #[default]
    Length32 = 0x04,
}

/// FLRC packet params
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlrcPacketParams {
    pub preamble_length: PreambleLength,
    pub sync_word_length: FlrcSyncWordLength,
    pub sync_word_match: SyncWordRxMatch,
    pub header_type: PacketHeaderType,
    pub payload_length: u8,
    pub crc_mode: CrcMode,
    pub whitening: WhiteningMode,
}

impl FlrcPacketParams {
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

/// FLRC modem params
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlrcModem {
    pub frequency: Frequency,
    pub tx_params: TxParams,
    pub modulation_params: FlrcModulationParams,
    pub packet_params: FlrcPacketParams,
}
