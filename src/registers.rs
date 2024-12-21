/// Radio registers
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    FirmwareVersion = 0x0153,
    RxGain = 0x0891,
    ManualGainSetting = 0x0895,
    LnaGainValue = 0x089e,
    LnaGainControl = 0x089f,
    SynchPeakAttenuation = 0x08b2,
    PayloadLength = 0x901,
    LoraHeaderMode = 0x903,
    RangingInitiatorAddress = 0x0912,
    RangingResponderAddress = 0x0916,
    RangingFilterWindowSize = 0x091e,
    ResetRangingFilter = 0x0923,
    RangingResultMUX = 0x0924,
    LoraSpreadingFactorConfiguration = 0x0925,
    RangingDelayCalibration = 0x092b,
    RangingAddressCheckLength = 0x0931,
    FrequencyErrorCompensation = 0x093C,
    CADDetectionPeak = 0x0942,
    LoRaSyncWord = 0x0944,
    RangingFilterRssiThresholdOffset = 0x0953,
    LoraEstimatedFrequencyError = 0x0954,
    LoraCodingRate = 0x0950,
    RangingResult = 0x0961,
    RangingRssi = 0x0964,
    RangingResultFreeze = 0x097f,
    GfskBlePreambleLength = 0x09c1,
    LoraWhiteningSeed = 0x09c5,
    CrcPoly = 0x09c6,
    CrcPolySeed = 0x09c7,
    GFSKSyncWordTolerance = 0x09cd,
    GFSKSyncWord = 0x09ce,
    BleAccessAddress = 0x09cf,
}
