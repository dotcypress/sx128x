use bitflags::bitflags;
use ble::*;
use flrc::*;
use gfsk::*;
use lora::*;

pub mod ble;
pub mod flrc;
pub mod gfsk;
pub mod lora;

/// Radio mode
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    Sleep,
    #[default]
    StandbyRc,
    StandbyXosc,
    Fs,
    Rx,
    Tx,
}

/// Configuration parameters
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub busy_timeout_us: u32,
    pub sleep_config: SleepConfig,
    pub tx_timeout: Period,
    pub rx_timeout: Period,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            busy_timeout_us: 100,
            sleep_config: Default::default(),
            tx_timeout: Default::default(),
            rx_timeout: Default::default(),
        }
    }
}

/// Sleep config
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SleepConfig {
    #[default]
    FlushRAM = 0x00,
    KeepRAM = 0x01,
}

/// Period
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Period {
    pub base: u8,
    pub count: u16,
}

impl Period {
    pub const fn from_ms(millis: u32) -> Self {
        Self::from_us(millis * 1000)
    }

    pub const fn from_us(micros: u32) -> Self {
        let (base, count) = if micros < 1_023_984 {
            (0x00, micros * 1_000 / 15_625)
        } else if micros < 4_095_937 {
            (0x01, micros * 10 / 625)
        } else if micros < 65_535_000 {
            (0x02, micros)
        } else {
            (0x03, micros / 4)
        };
        Self {
            base,
            count: count as u16,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Frequency {
    raw: [u8; 3],
}

impl Frequency {
    pub const fn new(freq: u32) -> Self {
        let val = freq as f64 / 198.3642578125;
        let val = val as u32;
        Self {
            raw: [(val >> 16) as u8, (val >> 8) as u8, val as u8],
        }
    }

    pub const fn from_bytes(raw: [u8; 3]) -> Self {
        Self { raw }
    }

    pub fn as_bytes(&self) -> [u8; 3] {
        self.raw
    }
}

impl Default for Frequency {
    fn default() -> Self {
        Self::new(2_400_000_000)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Frequency {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Frequency {{ {:02x} }}", self.raw)
    }
}

/// Packet type
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PacketType {
    #[default]
    GFSK = 0x00,
    LoRa = 0x01,
    Ranging = 0x02,
    FLRC = 0x03,
    BLE = 0x04,
}

impl From<u8> for PacketType {
    fn from(value: u8) -> Self {
        match value {
            0x01 => PacketType::LoRa,
            0x02 => PacketType::Ranging,
            0x03 => PacketType::FLRC,
            0x04 => PacketType::BLE,
            _ => PacketType::GFSK,
        }
    }
}

/// Ranging role
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangingRole {
    #[default]
    Responder = 0x00,
    Initiator = 0x01,
}

/// Ranging role
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CadSymbols {
    #[default]
    Sym1 = 0x00,
    Sym2 = 0x20,
    Sym4 = 0x40,
    Sym8 = 0x60,
    Sym16 = 0x80,
}

/// Regulator operating mode
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegulatorMode {
    #[default]
    Ldo = 0x00,
    Dcdc = 0x01,
}

/// Power amplifier ramp time
#[derive(Copy, Clone, Default, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTime {
    #[default]
    Ramp2Us = 0x00,
    Ramp4Us = 0x20,
    Ramp6Us = 0x40,
    Ramp8Us = 0x60,
    Ramp10Us = 0x80,
    Ramp12Us = 0xA0,
    Ramp16Us = 0xC0,
    Ramp20Us = 0xE0,
}

bitflags! {
    /// Radio calibration parameters
    #[derive(Copy, Clone, Default, PartialEq, Debug)]
    pub struct CalibrationParams: u8 {
        const ADCBulkPEnable = (1 << 5);
        const ADCBulkNEnable = (1 << 4);
        const ADCPulseEnable = (1 << 3);
        const PLLEnable = (1 << 2);
        const RC13MEnable = (1 << 1);
        const RC64KEnable = 1 ;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for CalibrationParams {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "CalibrationParams {{ 0b{0=0..8:08b} }}", self.bits())
    }
}

/// Preamble length
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PreambleLength {
    Sym4 = 0x00,
    Sym8 = 0x10,
    Sym12 = 0x20,
    #[default]
    Sym16 = 0x30,
    Sym20 = 0x40,
    Sym24 = 0x50,
    Sym28 = 0x60,
    Sym32 = 0x70,
}

/// Whitening mode
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WhiteningMode {
    On = 0x00,
    #[default]
    Off = 0x08,
}

/// Sync word RX match
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SyncWordRxMatch {
    SyncwordOff = 0x00,
    #[default]
    Syncword1 = 0x10,
    Syncword2 = 0x20,
    Syncword1_2 = 0x30,
    Syncword3 = 0x40,
    Syncword1_3 = 0x50,
    Syncword2_3 = 0x60,
    Syncword1_2_3 = 0x70,
}

/// Packet headers type
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PacketHeaderType {
    Fixed = 0x00,
    #[default]
    Variable = 0x20,
}

/// Crc mode
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrcMode {
    CrcOff = 0x00,
    #[default]
    Crc16Bit = 0x10,
    Crc24Bit = 0x20,
    Crc32Bit = 0x30,
}

/// Modulation shaping
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModulationShaping {
    #[default]
    Off = 0x00,
    Bt1_0 = 0x10,
    Bt0_5 = 0x20,
}

/// Bitrate and bandwidth
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitrateBandwidth {
    Br2000Bw2_4 = 0x04,
    Br1600Bw2_4 = 0x28,
    Br1000Bw2_4 = 0x4C,
    Br1000Bw1_2 = 0x45,
    Br800Bw2_4 = 0x70,
    Br800Bw1_2 = 0x69,
    Br500Bw1_2 = 0x8D,
    Br500Bw0_6 = 0x86,
    Br400Bw1_2 = 0xB1,
    Br400Bw0_6 = 0xAA,
    Br250Bw0_6 = 0xCE,
    #[default]
    Br250Bw0_3 = 0xC7,
    Br125Bw0_3 = 0xEF,
}

/// Modulation index
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModulationIndex {
    #[default]
    Index0_35 = 0x00,
    Index0_50 = 0x01,
    Index0_75 = 0x02,
    Index1_00 = 0x03,
    Index1_25 = 0x04,
    Index1_50 = 0x05,
    Index1_75 = 0x06,
    Index2_00 = 0x07,
    Index2_25 = 0x08,
    Index2_50 = 0x09,
    Index2_75 = 0x0a,
    Index3_00 = 0x0b,
    Index3_25 = 0x0c,
    Index3_50 = 0x0d,
    Index3_75 = 0x0e,
    Index4_00 = 0x0f,
}

/// Modulation parameters
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModulationParams {
    LoRa(LoRaModulationParams),
    FLRC(FlrcModulationParams),
    GFSK(GfskModulationParams),
    BLE(BleModulationParams),
}

impl ModulationParams {
    pub(crate) fn as_bytes(&self) -> [u8; 3] {
        match self {
            ModulationParams::LoRa(params) => params.as_bytes(),
            ModulationParams::FLRC(params) => params.as_bytes(),
            ModulationParams::GFSK(params) => params.as_bytes(),
            ModulationParams::BLE(params) => params.as_bytes(),
        }
    }
}

/// TX parameters
#[derive(Copy, Clone, PartialEq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxParams {
    pub power: u8,
    pub ramp_time: RampTime,
}

/// Packet parameters
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PacketParams {
    LoRa(LoRaPacketParams),
    FLRC(FlrcPacketParams),
    GFSK(GfskPacketParams),
    BLE(BlePacketParams),
}

impl PacketParams {
    pub(crate) fn as_bytes(&self) -> [u8; 7] {
        match self {
            PacketParams::LoRa(params) => params.as_bytes(),
            PacketParams::FLRC(params) => params.as_bytes(),
            PacketParams::GFSK(params) => params.as_bytes(),
            PacketParams::BLE(params) => params.as_bytes(),
        }
    }
}

/// Modem parameters
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Modem {
    LoRa(LoRaModem),
    Ranging(RangingModem),
    GFSK(GfskModem),
    FLRC(FlrcModem),
    BLE(BleModem),
}

impl From<Modem> for PacketType {
    fn from(value: Modem) -> Self {
        match value {
            Modem::LoRa(_) => PacketType::LoRa,
            Modem::Ranging(_) => PacketType::Ranging,
            Modem::GFSK(_) => PacketType::GFSK,
            Modem::FLRC(_) => PacketType::FLRC,
            Modem::BLE(_) => PacketType::BLE,
        }
    }
}
