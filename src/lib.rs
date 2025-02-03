//! A platform agnostic Rust driver for the SX128x, based on the `embedded-hal` traits.
#![no_std]

use bitflags::bitflags;
use commands::Command;
use config::*;
use embedded_hal::{
    delay::DelayNs,
    digital,
    spi::{self, Operation},
};
use irq::*;
use lora::RangingResultType;
use registers::Register;

pub mod commands;
pub mod config;
pub mod irq;
pub mod registers;

pub const SX128X_MODE: spi::Mode = embedded_hal::spi::MODE_0;

bitflags! {
    /// Packet errors
    #[derive(Copy, Clone, Default, PartialEq, Debug)]
    pub struct PacketErrors: u8 {
        const Unknown = (1 << 7);
        const SyncError = (1 << 6);
        const LengthError = (1 << 5);
        const CrcError = (1 << 4);
        const AbortError = (1 << 3);
        const HeaderReceived = (1 << 2);
        const PacketReceived = (1 << 1);
        const PacketControlerBusy = (1 << 0);
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PacketErrors {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PacketErrors {{ 0b{0=0..8:08b} }}", self.bits())
    }
}

/// SX128x error
pub enum Error<SPI: spi::SpiDevice> {
    PinError,
    Timeout,
    ConfigError,
    TransferError(SPI::Error),
}

impl<SPI: spi::SpiDevice> core::fmt::Debug for Error<SPI> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PinError => write!(f, "GPIO Error"),
            Self::ConfigError => write!(f, "Config Error"),
            Self::Timeout => write!(f, "Timeout"),
            Self::TransferError(err) => write!(f, "SPI Error: {:?}", err),
        }
    }
}

/// Radio status
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Status {
    value: u8,
}

impl Status {
    pub fn new(value: u8) -> Self {
        Self { value }
    }

    pub fn mode(&self) -> Option<Mode> {
        let mode = match self.value >> 5 {
            0x02 => Mode::StandbyRc,
            0x03 => Mode::StandbyXosc,
            0x04 => Mode::Fs,
            0x05 => Mode::Rx,
            0x06 => Mode::Tx,
            _ => return None,
        };
        Some(mode)
    }

    pub fn is_data_available(&self) -> bool {
        self.value >> 2 & 0b111 == 0x02
    }

    pub fn is_command_timeout(&self) -> bool {
        self.value >> 2 & 0b111 == 0x03
    }

    pub fn is_command_processing_failed(&self) -> bool {
        self.value >> 2 & 0b111 == 0x04
    }

    pub fn is_command_execution_failed(&self) -> bool {
        self.value >> 2 & 0b111 == 0x05
    }

    pub fn is_tx_done(&self) -> bool {
        self.value >> 2 & 0b111 == 0x06
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Status mode: {}, tx done: {}, data available: {}, timeout: {}, processing failed: {}, execution failed: {}",
            self.mode(),
            self.is_tx_done(),
            self.is_data_available(),
            self.is_command_timeout(),
            self.is_command_processing_failed(),
            self.is_command_execution_failed()
        )
    }
}

/// Buffer status
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BufferStatus {
    pub length: u8,
    pub pointer: u8,
}

/// Packet status
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PacketStatus {
    status: [u8; 5],
}

impl PacketStatus {
    pub fn new(raw: &[u8]) -> Self {
        let mut status = [0; 5];
        status.copy_from_slice(raw);
        Self { status }
    }

    /// Get RSSI sync
    pub fn rssi_sync(&self, packet_type: PacketType) -> u8 {
        match packet_type {
            PacketType::LoRa | PacketType::Ranging => self.status[0],
            _ => self.status[1],
        }
    }

    /// Get signal to noise ratio
    pub fn snr(&self, packet_type: PacketType) -> u8 {
        match packet_type {
            PacketType::LoRa | PacketType::Ranging => self.status[1],
            _ => 0,
        }
    }

    /// Get RX NO_ACK field of the received packet
    pub fn rx_no_ack(&self) -> bool {
        self.status[3] & 0b1_0000 > 0
    }

    /// Get TX is complete
    pub fn tx_complete(&self) -> bool {
        self.status[3] & 0b1 > 0
    }

    /// Get errors
    pub fn errors(&self) -> PacketErrors {
        PacketErrors::from_bits(self.status[2]).unwrap_or_default()
    }

    /// Code of the sync address detected
    pub fn sync_code(&self) -> u8 {
        self.status[4]
    }
}

/// Driver for the SX128x
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SX128x<SPI, RESET, BUSY, DELAY> {
    spi: SPI,
    reset: RESET,
    busy: BUSY,
    delay: DELAY,
    cfg: Config,
    modem: Option<Modem>,
}

impl<SPI, RESET, BUSY, DELAY> SX128x<SPI, RESET, BUSY, DELAY>
where
    SPI: spi::SpiDevice,
    RESET: digital::OutputPin,
    BUSY: digital::InputPin,
    DELAY: DelayNs,
{
    /// Create a new SX128x driver
    pub fn try_new(
        spi: SPI,
        reset: RESET,
        busy: BUSY,
        delay: DELAY,
        cfg: Config,
    ) -> Result<Self, Error<SPI>> {
        let mut radio = SX128x {
            spi,
            reset,
            busy,
            delay,
            cfg,
            modem: None,
        };
        radio.reset()?;
        radio.calibrate(CalibrationParams::all())?;
        Ok(radio)
    }

    /// Realeses SPI bus and control pins
    pub fn release(self) -> (SPI, RESET, BUSY, DELAY) {
        (self.spi, self.reset, self.busy, self.delay)
    }

    /// Reset radio
    pub fn reset(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("reset()");
        self.reset.set_low().map_err(|_| Error::PinError)?;
        self.delay.delay_ms(10);
        self.reset.set_high().map_err(|_| Error::PinError)?;
        self.delay.delay_ms(100);
        Ok(())
    }

    /// Calibrate
    pub fn calibrate(&mut self, params: CalibrationParams) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("calibrate({})", params);
        self.write_command(Command::Calibrate, &[params.bits()])?;
        self.delay.delay_ms(10);
        Ok(())
    }

    /// Get current modem
    pub fn get_modem(&mut self) -> Option<Modem> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_modem()");
        self.modem
    }

    /// Enable modem
    pub fn enable_modem(&mut self, modem: Modem) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("enable_modem({})", modem);
        self.set_mode(Mode::StandbyRc)?;
        self.set_packet_type(modem.into())?;
        self.modem = Some(modem);
        match modem {
            Modem::LoRa(modem) => {
                self.set_modulation_params(ModulationParams::LoRa(modem.modulation_params))?;
                self.set_packet_params(PacketParams::LoRa(modem.packet_params))?;
                self.set_rf_frequency(modem.frequency)?;
                self.set_tx_params(modem.tx_params)?;
            }
            Modem::Ranging(modem) => {
                self.set_modulation_params(ModulationParams::LoRa(modem.lora.modulation_params))?;
                self.set_packet_params(PacketParams::LoRa(modem.lora.packet_params))?;
                self.set_rf_frequency(modem.lora.frequency)?;
                self.set_tx_params(modem.lora.tx_params)?;
                self.set_ranging_role(modem.role)?;
                match modem.role {
                    RangingRole::Responder => self.write_register(
                        Register::RangingResponderAddress,
                        &modem.address.to_be_bytes(),
                    )?,
                    RangingRole::Initiator => self.write_register(
                        Register::RangingInitiatorAddress,
                        &modem.address.to_be_bytes(),
                    )?,
                };
                self.write_register(
                    Register::RangingAddressCheckLength,
                    &[modem.address_bits as u8],
                )?;
                self.write_register(
                    Register::RangingDelayCalibration,
                    &modem.calibration.to_be_bytes()[1..],
                )?;
            }
            Modem::GFSK(modem) => {
                self.set_modulation_params(ModulationParams::GFSK(modem.modulation_params))?;
                self.set_packet_params(PacketParams::GFSK(modem.packet_params))?;
                self.set_rf_frequency(modem.frequency)?;
                self.set_tx_params(modem.tx_params)?;
                self.write_register(Register::GFSKSyncWord, &modem.sync_word)?
            }
            Modem::FLRC(modem) => {
                self.set_modulation_params(ModulationParams::FLRC(modem.modulation_params))?;
                self.set_packet_params(PacketParams::FLRC(modem.packet_params))?;
                self.set_rf_frequency(modem.frequency)?;
                self.set_tx_params(modem.tx_params)?;
            }
            Modem::BLE(modem) => {
                self.set_modulation_params(ModulationParams::BLE(modem.modulation_params))?;
                self.set_packet_params(PacketParams::BLE(modem.packet_params))?;
                self.set_rf_frequency(modem.frequency)?;
                self.set_tx_params(modem.tx_params)?;
            }
        }
        Ok(())
    }

    /// Set save context
    pub fn set_save_context(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_save_context()");
        self.write_command(Command::SetSaveContext, &[])
    }

    /// Set TX params
    pub fn set_tx_params(&mut self, params: TxParams) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_tx_params({})", params);
        self.write_command(
            Command::SetTxParams,
            &[params.power, params.ramp_time as u8],
        )
    }

    /// Set modulation params
    pub fn set_modulation_params(&mut self, params: ModulationParams) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_modulation_params({})", params);
        self.write_command(Command::SetModulationParams, &params.as_bytes())?;

        if let ModulationParams::LoRa(lora_params) = params {
            let sf_config = match lora_params.spreading_factor {
                lora::LoRaSpreadingFactor::Sf5 | lora::LoRaSpreadingFactor::Sf6 => 0x1E,
                lora::LoRaSpreadingFactor::Sf7 | lora::LoRaSpreadingFactor::Sf8 => 0x37,
                _ => 0x32,
            };
            self.write_register(Register::LoraSpreadingFactorConfiguration, &[sf_config])?;
            self.write_register(Register::FrequencyErrorCompensation, &[0x01])?;
        }

        Ok(())
    }

    /// Set packet params
    pub fn set_packet_params(&mut self, params: PacketParams) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_packet_params({})", params);
        self.write_command(Command::SetPacketParams, &params.as_bytes())?;

        if let PacketParams::LoRa(lora_params) = params {
            let mut scratch = [0x00; 2];
            self.read_register(Register::LoRaSyncWord, &mut scratch)?;
            scratch[0] = scratch[0] & 0x0f | (lora_params.sync_word & 0xf0);
            scratch[1] = scratch[1] & 0x0f | (lora_params.sync_word << 4);
            self.write_register(Register::LoRaSyncWord, &scratch)?;
        }

        Ok(())
    }

    /// Set regulator mode
    pub fn set_regulator_mode(&mut self, mode: RegulatorMode) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_regulator_mode({})", mode);
        self.write_command(Command::SetRegulatorMode, &[mode as u8])
    }

    /// Set common config
    pub fn set_config(&mut self, cfg: Config) {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_config({})", cfg);
        self.cfg = cfg;
    }

    /// Get common config
    pub fn get_config(&mut self) -> Config {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_config()");
        self.cfg
    }

    /// Get status
    pub fn get_status(&mut self) -> Result<Status, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_status()");
        let mut scratch = [0; 1];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Transfer(
                &mut scratch,
                &[Command::GetStatus as u8],
            )])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(Status::new(scratch[0]))
    }

    /// Get SX128x firmware version
    pub fn get_firmware_version(&mut self) -> Result<u16, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_firmware_version()");
        let mut scratch = [0; 2];
        self.read_register(Register::FirmwareVersion, &mut scratch)?;
        Ok(u16::from_le_bytes(scratch))
    }

    /// Set buffer base address
    pub fn set_buffer_base_address(
        &mut self,
        tx_address: u8,
        rx_address: u8,
    ) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "set_buffer_base_address(0x{:02x}, 0x{:02x})",
            tx_address,
            rx_address
        );
        self.write_command(Command::SetBufferBaseAddress, &[tx_address, rx_address])
    }

    /// Read data buffer
    pub fn read_buffer(&mut self, offset: u8, value: &mut [u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read_buffer(0x{:02x})", offset);
        let payload = [Command::ReadBuffer as u8, offset, 0];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Read(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    /// Write data buffer
    pub fn write_buffer(&mut self, offset: u8, value: &[u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("write_buffer(0x{:02x}, {:02x})", offset, value);
        let payload = [Command::WriteBuffer as u8, offset];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Write(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    /// Set radio mode
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_mode({})", mode);
        match mode {
            Mode::StandbyRc => self.write_command(Command::SetStandby, &[0x00]),
            Mode::StandbyXosc => self.write_command(Command::SetStandby, &[0x01]),
            Mode::Fs => self.write_command(Command::SetFs, &[]),
            Mode::Sleep => {
                self.write_command(Command::SetSleep, &[0x01, self.cfg.sleep_config as u8])
            }
            Mode::Rx => {
                let count = self.cfg.rx_timeout.count.to_be_bytes();
                self.write_command(
                    Command::SetRx,
                    &[self.cfg.rx_timeout.base, count[0], count[1]],
                )
            }
            Mode::Tx => {
                let count = self.cfg.tx_timeout.count.to_be_bytes();
                self.write_command(
                    Command::SetTx,
                    &[self.cfg.tx_timeout.base, count[0], count[1]],
                )
            }
        }
    }

    /// Set auto TX time
    pub fn set_auto_tx(&mut self, time: u16) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_auto_tx({:02x})", time);
        self.write_command(Command::SetAutoTx, &time.to_be_bytes())
    }

    /// Set auto FS
    pub fn set_auto_fs(&mut self, enabled: bool) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_auto_fs({})", enabled);
        let payload = if enabled { 0x01 } else { 0x00 };
        self.write_command(Command::SetAutoFs, &[payload])
    }

    /// Set long preamble
    pub fn set_long_preamble(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_long_preamble()");
        self.write_command(Command::SetLongPreamble, &[0x01])
    }

    /// Set TX continuous preamble
    pub fn set_tx_continuous_preamble(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_tx_continuous_preamble()");
        self.write_command(Command::SetTxContinuousPreamble, &[])
    }

    /// Set TX continuous wave
    pub fn set_tx_continuous_wave(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_tx_continuous_wave()");
        self.write_command(Command::SetTxContinuousWave, &[])
    }

    /// Set channel activity detection
    pub fn set_cad(&mut self) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_cad()");
        self.write_command(Command::SetCad, &[])
    }

    /// Set channel activity detection params
    pub fn set_cad_params(&mut self, symbols: CadSymbols) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_cad_params({})", symbols);
        self.write_command(Command::SetCadParams, &[symbols as u8])
    }

    /// Set ranging role
    pub fn set_ranging_role(&mut self, role: RangingRole) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_ranging_role({})", role);
        self.write_command(Command::SetRangingRole, &[role as u8])
    }

    /// Set advanced ranging
    pub fn set_advanced_ranging(&mut self, enable: bool) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_advanced_ranging({})", enable);
        let payload = if enable { 0x01 } else { 0x00 };
        self.write_command(Command::SetAdvancedRanging, &[payload])
    }

    /// Get ranging result
    pub fn get_ranging_result(
        &mut self,
        result_type: RangingResultType,
    ) -> Result<f32, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_ranging_result()");
        self.set_mode(Mode::StandbyXosc)?;
        self.freeze_ranging_result()?;

        let mut scratch = [0];
        self.read_register(Register::RangingResultMUX, &mut scratch)?;
        self.write_register(
            Register::RangingResultMUX,
            &[(scratch[0] & 0xcf) | result_type as u8],
        )?;

        let mut scratch = [0; 4];
        self.read_register(Register::RangingResult, &mut scratch[..3])?;
        self.set_mode(Mode::StandbyRc)?;
        let raw_result = i32::from_be_bytes(scratch) >> 8;
        match self.modem {
            Some(Modem::Ranging(modem)) => {
                let bw = modem.lora.modulation_params.bandwidth.mhz();
                Ok(raw_result as f32 * 150.0 / (4096.0 * bw))
            }
            _ => Err(Error::ConfigError),
        }
    }

    /// Get frequency error indicator level
    pub fn get_fei(&mut self) -> Result<f32, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_fei()");
        let mut scratch = [0; 4];
        self.read_register(Register::LoraEstimatedFrequencyError, &mut scratch[..3])?;
        let aligned = (u32::from_be_bytes(scratch) << 4).to_be_bytes();
        let raw_result = i32::from_be_bytes(aligned) >> 8;
        match self.modem {
            Some(Modem::Ranging(modem)) => {
                let bw = modem.lora.modulation_params.bandwidth.mhz();
                Ok(raw_result as f32 * 1.55 / (1.6 / bw))
            }
            _ => Err(Error::ConfigError),
        }
    }

    /// Freeze ranging result
    pub fn freeze_ranging_result(&mut self) -> Result<u32, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("freeze_ranging_result()");
        let mut scratch = [0];
        self.read_register(Register::RangingResultFreeze, &mut scratch)?;
        self.write_register(Register::RangingResultFreeze, &[scratch[0] | 0x02])?;
        Ok(0)
    }

    /// Get ranging RSSI
    pub fn get_ranging_rssi(&mut self) -> Result<u8, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_ranging_rssi()");
        let mut scratch = [0];
        self.read_register(Register::RangingRssi, &mut scratch)?;
        Ok(scratch[0])
    }

    /// Set RX duty cycle
    pub fn set_rx_duty_cycle(
        &mut self,
        period_base: u8,
        rx_count: u16,
        sleep_count: u16,
    ) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "set_rx_duty_cycle({:02x}, {:02x}, {:02x})",
            period_base,
            rx_count,
            sleep_count
        );
        let rx_count = rx_count.to_be_bytes();
        let sleep_count = sleep_count.to_be_bytes();
        self.write_command(
            Command::SetRxDutyCycle,
            &[
                period_base,
                rx_count[0],
                rx_count[1],
                sleep_count[0],
                sleep_count[1],
            ],
        )
    }

    /// Get RX buffer status
    pub fn get_rx_buffer_status(&mut self) -> Result<BufferStatus, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_rx_buffer_status()");
        let mut value = [0; 3];
        self.read_command(Command::GetRxBufferStatus, &mut value)?;
        Ok(BufferStatus {
            length: value[1],
            pointer: value[2],
        })
    }

    /// Get packet status
    pub fn get_packet_status(&mut self) -> Result<PacketStatus, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_packet_status()");
        let mut value = [0; 6];
        self.read_command(Command::GetPacketStatus, &mut value)?;
        Ok(PacketStatus::new(&value[1..]))
    }

    /// Get instantaneous RSSI
    pub fn get_rssi_inst(&mut self) -> Result<u8, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_rssi_inst()");
        let mut value = [0; 2];
        self.read_command(Command::GetRssiInst, &mut value)?;
        Ok(value[1])
    }

    /// Set DIO IRQ staparamstus
    pub fn set_dio_irq_params(
        &mut self,
        irq_mask: Irq,
        dio1_mask: DioMask,
        dio2_mask: DioMask,
        dio3_mask: DioMask,
    ) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "set_dio_irq_params({}, {}, {}, {})",
            irq_mask,
            dio1_mask,
            dio2_mask,
            dio3_mask
        );
        let mut payload = [0; 8];
        payload[0..2].copy_from_slice(&irq_mask.bits().to_be_bytes());
        payload[2..4].copy_from_slice(&dio1_mask.bits().to_be_bytes());
        payload[4..6].copy_from_slice(&dio2_mask.bits().to_be_bytes());
        payload[6..8].copy_from_slice(&dio3_mask.bits().to_be_bytes());
        self.write_command(Command::SetDioIrqParams, &payload)
    }

    /// Get IRQ status
    pub fn get_irq_status(&mut self) -> Result<Irq, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_irq_status()");
        let mut value = [0; 3];
        self.read_command(Command::GetIrqStatus, &mut value)?;
        let status = Irq::from_bits((value[1] as u16) << 8 | value[2] as u16).unwrap_or_default();
        Ok(status)
    }

    /// Clear IRQ status
    pub fn clear_irq_status(&mut self, mask: DioMask) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("clear_irq_status({})", mask);
        let payload = mask.bits().to_be_bytes();
        self.write_command(Command::ClearIrqStatus, &payload)
    }

    /// Set packet type
    pub fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_packet_type({})", packet_type);
        self.write_command(Command::SetPacketType, &[packet_type as u8])
    }

    /// Get packet type
    pub fn get_packet_type(&mut self) -> Result<PacketType, Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("get_packet_type()");
        let mut value = [0; 2];
        self.read_command(Command::GetPacketType, &mut value)?;
        Ok(value[1].into())
    }

    /// Set RF frequency
    pub fn set_rf_frequency(&mut self, freq: Frequency) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("set_rf_frequency({})", freq);
        self.write_command(Command::SetRfFrequency, &freq.as_bytes())
    }

    pub fn write_register(&mut self, reg: Register, value: &[u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("write_register({}, {:02x})", reg, value);
        let reg = reg as u16;
        let payload = [Command::WiteRegister as u8, (reg >> 8) as u8, reg as u8];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Write(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    pub fn read_register(&mut self, reg: Register, value: &mut [u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read_register({})", reg);
        let reg = reg as u16;
        let payload = [Command::ReadRegister as u8, (reg >> 8) as u8, reg as u8, 0];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Read(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    pub fn read_command(&mut self, cmd: Command, value: &mut [u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read_command({})", cmd);
        let payload = [cmd as u8];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Read(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    pub fn write_command(&mut self, cmd: Command, value: &[u8]) -> Result<(), Error<SPI>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("write_command({}, {:02x})", cmd, value);
        let payload = [cmd as u8];
        self.wait_busy()?;
        self.spi
            .transaction(&mut [Operation::Write(&payload), Operation::Write(value)])
            .map_err(Error::<SPI>::TransferError)?;
        Ok(())
    }

    fn is_busy(&mut self) -> Result<bool, Error<SPI>> {
        self.busy.is_high().map_err(|_| Error::PinError)
    }

    fn wait_busy(&mut self) -> Result<(), Error<SPI>> {
        if !self.is_busy()? {
            return Ok(());
        }
        self.delay.delay_us(self.cfg.busy_timeout_us);
        if self.is_busy()? {
            Err(Error::Timeout)
        } else {
            Ok(())
        }
    }
}
