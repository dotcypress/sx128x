use super::*;

/// BLE modulation params
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BleModulationParams {
    pub bitrate: BitrateBandwidth,
    pub modulation_index: ModulationIndex,
    pub modulation_shaping: ModulationShaping,
}

impl BleModulationParams {
    pub(crate) fn as_bytes(&self) -> [u8; 3] {
        [
            self.bitrate as u8,
            self.modulation_index as u8,
            self.modulation_shaping as u8,
        ]
    }
}

/// BLE packet params
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BlePacketParams {
    pub connection_state: BleConnectionState,
    pub crc_mode: BleCrcMode,
    pub test_payload: BleTestPayload,
    pub whitening: WhiteningMode,
}

impl BlePacketParams {
    pub(crate) fn as_bytes(&self) -> [u8; 7] {
        [
            self.connection_state as u8,
            self.crc_mode as u8,
            self.test_payload as u8,
            self.whitening as u8,
            0x00,
            0x00,
            0x00,
        ]
    }
}

#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BleConnectionState {
    #[default]
    PayloadLengthMax31Bytes = 0x00,
    PayloadLengthMax37Bytes = 0x20,
    TxTestMode = 0x40,
    PayloadLengthMax255Bytes = 0x80,
}

/// BLE CRC mode
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BleCrcMode {
    #[default]
    Off = 0x00,
    On = 0x10,
}

/// BLE mode packet type
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BleTestPayload {
    #[default]
    Prbs9 = 0x00,
    Eyelong1_0 = 0x04,
    Eyeshort1_0 = 0x08,
    Prbs15 = 0x0C,
    All1 = 0x10,
    All0 = 0x14,
    Eyelong0_1 = 0x18,
    Eyeshort0_1 = 0x1C,
}

/// BLE modem params
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BleModem {
    pub frequency: Frequency,
    pub tx_params: TxParams,
    pub modulation_params: BleModulationParams,
    pub packet_params: BlePacketParams,
}
