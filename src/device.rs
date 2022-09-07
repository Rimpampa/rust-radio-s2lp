//! S2-LP Device definitions
//! 
//! Copyright 2018 Ryan Kurte

// NOTE: the modular-bitfield crate, version 0.11.2, when using
// the bitfield attribute proc-macro, makes rustc throw this warning
// on the proc-macro-generated code
#![allow(unused_braces)]

use core::convert::Infallible;

use modular_bitfield::prelude::*;
use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

/// SPI command modes
#[derive(Copy, Clone, Debug)]
pub enum SpiCommand {
    Write = 0x00,  // SPI Write register command
    Read = 0x01,   // SPI Read register command
    Strobe = 0x80, // SPI Strobe command
}

pub const XO_RCO_CONF0_REFDIV_REGMASK: u8 = 0x08;
pub const XO_RCO_CONF1_PD_CLKDIV_REGMASK: u8 = 0x10;

// TODO: how to represent different enums for Gpio Out and In modes?

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio0Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    _reserved: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio0Conf {
    const ADDRESS: u8 = 0x00;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio1Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    _reserved: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio1Conf {
    const ADDRESS: u8 = 0x01;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio2Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    _reserved: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio2Conf {
    const ADDRESS: u8 = 0x02;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio3Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    _reserved: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio3Conf {
    const ADDRESS: u8 = 0x03;
    type Word = u8;
    type Error = Infallible;
}


/// GPIO Pin Modes
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum GpioMode {
    DigitalInput = 0b01,
    DigitalOutputLowPower = 0b00,
    DigitalOutputHighPower = 0b11,
}

/// GPIO Pin Operation
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 5]
pub enum GpioOutSelect {
    NIrq = 0,   // nIRQ (interrupt request, active low)
    Por = 1,    // POR inverted (active low)
    Wake = 2,   // Wake-up timer expiration: ‘1’ when WUT has expired
    BattLow = 3,    // Low battery detection: ‘1’ when battery is below threshold setting
    TxDataClock = 4,     //TX data internal clock output (TX data are sampled on the rising edge of it)
    TxState = 5,     //TX state outputs a command information coming from the RADIO_TX block
    FifoAlmostEmpty = 6,     //TX/RX FIFO almost empty flag
    FifoAlmostFull = 7,     //TX/RX FIFO almost full flag
    RxData = 8,     //RX data output
    RxClock = 9,     //RX clock output (recovered from received data)
    RxState = 10,       // RX state indication: ‘1’ when the S2-LP is transiting in the RX state
    DeviceAwake = 11,   // Device in a state other than SLEEP or STANDBY: ‘0’ when in SLEEP/STANDBY
    DeviceStandby = 12,   // Device in STANDBY state
    AntennaSw = 13,  // Antenna switch signal used for antenna diversity
    PreambleValid = 14,    // Valid preamble detected flag
    SyncValid = 15, // Sync word detected flag
    RssiAboveThreshold = 16, // RSSI above threshold (same indication of CS register)
    Reserved = 17, // Reserved
    TxRxMode = 18,   // TX or RX mode indicator (to enable an external range extender)
    Vdd = 19,  // VDD (to emulate an additional GPIO of the MCU, programmable by SPI)
    Gnd = 20,  // GND (to emulate an additional GPIO of the MCU, programmable by SPI)
    ExternalSmpsEn = 21, // External SMPS enable signal (active high)
    DeviceSleep = 22,   // Device in SLEEP state
    DeviceReady = 23,   // Device in READY state
    DeviceLock = 24,   // Device in LOCK state
    DeviceWaitLock = 25,   // Device waiting for a high level of the lock-detector output signal
    TxDataOok = 26,  // TX_DATA_OOK signal (internal control signal generated in the OOK analog smooth mode)
    DeviceWaitReady2 = 27,   // Device waiting for a high level of the READY2 signal from XO
    DeviceWaitTimer = 28,   // Device waiting for timer expiration to allow PM block settling
    DeviceWaitVco = 29,   // Device waiting for end of VCO calibration
    DeviceSynthEn = 30,   // Device enables the full circuitry of the SYNTH block
    Reserved2 = 31, // Reserved
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 5]
pub enum GpioInSelect {
    TxCommand = 0,
    RxCommand = 1,
    TxData = 2,
    Wake = 3,
    ExternalClock = 4,
}


#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt3 {
    /// Set the charge pump current according to the XTAL frequency
    pub pll_cp_isel: B3,
    /// Synthesizer band select. This parameter selects the out-of loop divide factor of the synthesizer
    pub band_sel: B1,
    /// MSB bits of the PLL programmable divider
    pub synt_27_24: B4,
}

impl radio::Register for Synt3 {
    const ADDRESS: u8 = 0x05;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt2 {
    /// Intermediate bits of the PLL programmable divider
    pub synt_23_16: u8,
}

impl radio::Register for Synt2 {
    const ADDRESS: u8 = 0x06;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt1 {
    /// Intermediate bits of the PLL programmable divider
    pub synt_15_8: u8,
}

impl radio::Register for Synt1 {
    const ADDRESS: u8 = 0x07;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt0 {
    /// Lower bits of the PLL programmable divider
    pub synt_7_0: u8,
}

impl radio::Register for Synt0 {
    const ADDRESS: u8 = 0x08;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetAna {
    /// Intermediate frequency setting for the analog RF synthesizer
    pub if_ana: u8,
}

impl radio::Register for IfOffsetAna {
    const ADDRESS: u8 = 0x09;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetDig {
    /// Intermediate frequency setting for the digital shift-to-baseband circuits
    pub if_dig: u8,
}

impl radio::Register for IfOffsetDig {
    const ADDRESS: u8 = 0x0A;
    type Word = u8;
    type Error = Infallible;
}


#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChSpace {
    /// Channel spacing setting
    pub ch_space: u8,
}

impl radio::Register for ChSpace {
    const ADDRESS: u8 = 0x0C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChNum {
    /// Channel number. This value is multiplied by the channel spacing and 
    /// added to the synthesizer base frequency to generate the actual RF carrier frequency
    pub ch_num: u8,
}

impl radio::Register for ChNum {
    const ADDRESS: u8 = 0x0D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod4 {
    /// The MSB of the mantissa value of the data rate equation
    pub datarate_m_15_8: u8,
}

impl radio::Register for Mod4 {
    const ADDRESS: u8 = 0x0E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod3 {
    /// The LSB of the mantissa value of the data rate equation
    pub datarate_m_7_0: u8,
}

impl radio::Register for Mod3 {
    const ADDRESS: u8 = 0x0F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod2 {
    /// Modulation type
    pub mod_type: ModType,
    /// Datarate exponent
    pub datarate_e: B4
}

impl radio::Register for Mod2 {
    const ADDRESS: u8 = 0x10;
    type Word = u8;
    type Error = Infallible;
}


/// Modulation modes
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum ModType {
    /// 2-FSK
    Mod2Fsk = 0,
    /// 4-FSK
    Mod4Fsk = 1,
    /// 2-GFSK BT=1.0
    Mod2Gfsk = 2,
    /// 4-GFSK BT=1.0
    Mod4Gfsk = 3,
    /// ASK/OOK
    ModAskOok = 5,
    /// 2-GFSK BT=0.5
    Mod2GfskBt05 = 10,
    /// 4-GFSK BT=0.5
    Mod4GfskBt05 = 11,
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod1 {
    /// enable the PA power interpolator
    pub pa_interp_en: bool,
    /// enable frequency interpolator for the GFSK shaping
    pub mod_interp_en: bool,
    /// Select the constellation map for 4-(G)FSK or 2-(G)FSK modulations
    pub const_map: B2,
    /// The exponent value of the frequency deviation equation
    pub fdev_e: B4,
}

impl radio::Register for Mod1 {
    const ADDRESS: u8 = 0x11;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod0 {
    /// The mantissa value of the frequency deviation equation
    pub fdev_m: u8,
}

impl radio::Register for Mod0 {
    const ADDRESS: u8 = 0x12;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChFlt {
    /// The mantissa value of the receiver channel filter
    pub chflt_m: B4,
    /// The exponent value of the receiver channel filter
    pub chflt_e: B4,
}

impl radio::Register for ChFlt {
    const ADDRESS: u8 = 0x13;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc2 {
    pub raw: u8,
}

impl radio::Register for Afc2 {
    const ADDRESS: u8 = 0x14;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc1 {
    pub raw: u8,
}

impl radio::Register for Afc1 {
    const ADDRESS: u8 = 0x15;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc0 {
    pub raw: u8,
}

impl radio::Register for Afc0 {
    const ADDRESS: u8 = 0x16;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiFlt {
    pub raw: u8,
}

impl radio::Register for RssiFlt {
    const ADDRESS: u8 = 0x17;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiTh {
    pub raw: u8,
}

impl radio::Register for RssiTh {
    const ADDRESS: u8 = 0x18;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl4 {
    pub raw: u8,
}

impl radio::Register for Agcctrl4 {
    const ADDRESS: u8 = 0x1a;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl3 {
    pub raw: u8,
}

impl radio::Register for Agcctrl3 {
    const ADDRESS: u8 = 0x1b;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl2 {
    pub raw: u8,
}

impl radio::Register for Agcctrl2 {
    const ADDRESS: u8 = 0x1c;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl1 {
    pub raw: u8,
}

impl radio::Register for Agcctrl1 {
    const ADDRESS: u8 = 0x1d;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl0 {
    pub raw: u8,
}

impl radio::Register for Agcctrl0 {
    const ADDRESS: u8 = 0x1e;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AntSelectConf {
    pub raw: u8,
}

impl radio::Register for AntSelectConf {
    const ADDRESS: u8 = 0x1f;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec2 {
    pub raw: u8,
}

impl radio::Register for Clockrec2 {
    const ADDRESS: u8 = 0x20;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec1 {
    pub raw: u8,
}

impl radio::Register for Clockrec1 {
    const ADDRESS: u8 = 0x21;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl6 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl6 {
    const ADDRESS: u8 = 0x2B;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl5 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl5 {
    const ADDRESS: u8 = 0x2C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl4 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl4 {
    const ADDRESS: u8 = 0x2D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl3 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl3 {
    const ADDRESS: u8 = 0x2E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl2 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl2 {
    const ADDRESS: u8 = 0x2F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl1 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl1 {
    const ADDRESS: u8 = 0x30;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen1 {
    pub raw: u8,
}

impl radio::Register for Pcktlen1 {
    const ADDRESS: u8 = 0x31;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen0 {
    pub raw: u8,
}

impl radio::Register for Pcktlen0 {
    const ADDRESS: u8 = 0x32;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync3 {
    pub raw: u8,
}

impl radio::Register for Sync3 {
    const ADDRESS: u8 = 0x33;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync2 {
    pub raw: u8,
}

impl radio::Register for Sync2 {
    const ADDRESS: u8 = 0x34;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync1 {
    pub raw: u8,
}

impl radio::Register for Sync1 {
    const ADDRESS: u8 = 0x35;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync0 {
    pub raw: u8,
}

impl radio::Register for Sync0 {
    const ADDRESS: u8 = 0x36;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Qi {
    pub raw: u8,
}

impl radio::Register for Qi {
    const ADDRESS: u8 = 0x37;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktPstmbl {
    pub raw: u8,
}

impl radio::Register for PcktPstmbl {
    const ADDRESS: u8 = 0x38;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol2 {
    pub raw: u8,
}

impl radio::Register for Protocol2 {
    const ADDRESS: u8 = 0x39;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol1 {
    pub raw: u8,
}

impl radio::Register for Protocol1 {
    const ADDRESS: u8 = 0x3A;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol0 {
    pub raw: u8,
}

impl radio::Register for Protocol0 {
    const ADDRESS: u8 = 0x3B;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig3 {
    pub raw: u8,
}

impl radio::Register for FifoConfig3 {
    const ADDRESS: u8 = 0x3C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig2 {
    pub raw: u8,
}

impl radio::Register for FifoConfig2 {
    const ADDRESS: u8 = 0x3D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig1 {
    pub raw: u8,
}

impl radio::Register for FifoConfig1 {
    const ADDRESS: u8 = 0x3E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig0 {
    pub raw: u8,
}

impl radio::Register for FifoConfig0 {
    const ADDRESS: u8 = 0x3F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltOptions {
    pub raw: u8,
}

impl radio::Register for PcktFltOptions {
    const ADDRESS: u8 = 0x40;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals4 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals4 {
    const ADDRESS: u8 = 0x41;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals3 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals3 {
    const ADDRESS: u8 = 0x42;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals2 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals2 {
    const ADDRESS: u8 = 0x43;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals1 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals1 {
    const ADDRESS: u8 = 0x44;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals0 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals0 {
    const ADDRESS: u8 = 0x45;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers5 {
    pub raw: u8,
}

impl radio::Register for Timers5 {
    const ADDRESS: u8 = 0x46;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers4 {
    pub raw: u8,
}

impl radio::Register for Timers4 {
    const ADDRESS: u8 = 0x47;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers3 {
    pub raw: u8,
}

impl radio::Register for Timers3 {
    const ADDRESS: u8 = 0x48;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers2 {
    pub raw: u8,
}

impl radio::Register for Timers2 {
    const ADDRESS: u8 = 0x49;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers1 {
    pub raw: u8,
}

impl radio::Register for Timers1 {
    const ADDRESS: u8 = 0x4A;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers0 {
    pub raw: u8,
}

impl radio::Register for Timers0 {
    const ADDRESS: u8 = 0x4B;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf3 {
    pub raw: u8,
}

impl radio::Register for CsmaConf3 {
    const ADDRESS: u8 = 0x4C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf2 {
    pub raw: u8,
}

impl radio::Register for CsmaConf2 {
    const ADDRESS: u8 = 0x4D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf1 {
    pub raw: u8,
}

impl radio::Register for CsmaConf1 {
    const ADDRESS: u8 = 0x4E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf0 {
    pub raw: u8,
}

impl radio::Register for CsmaConf0 {
    const ADDRESS: u8 = 0x4F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask3 {
    pub raw: u8,
}

impl radio::Register for IrqMask3 {
    const ADDRESS: u8 = 0x50;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask2 {
    pub raw: u8,
}

impl radio::Register for IrqMask2 {
    const ADDRESS: u8 = 0x51;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask1 {
    pub raw: u8,
}

impl radio::Register for IrqMask1 {
    const ADDRESS: u8 = 0x52;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask0 {
    pub raw: u8,
}

impl radio::Register for IrqMask0 {
    const ADDRESS: u8 = 0x53;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FastRxTimer {
    pub raw: u8,
}

impl radio::Register for FastRxTimer {
    const ADDRESS: u8 = 0x54;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower8 {
    pub raw: u8,
}

impl radio::Register for PaPower8 {
    const ADDRESS: u8 = 0x5A;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower7 {
    pub raw: u8,
}

impl radio::Register for PaPower7 {
    const ADDRESS: u8 = 0x5B;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower6 {
    pub raw: u8,
}

impl radio::Register for PaPower6 {
    const ADDRESS: u8 = 0x5C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower5 {
    pub raw: u8,
}

impl radio::Register for PaPower5 {
    const ADDRESS: u8 = 0x5D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower4 {
    pub raw: u8,
}

impl radio::Register for PaPower4 {
    const ADDRESS: u8 = 0x5E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower3 {
    pub raw: u8,
}

impl radio::Register for PaPower3 {
    const ADDRESS: u8 = 0x5F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower2 {
    pub raw: u8,
}

impl radio::Register for PaPower2 {
    const ADDRESS: u8 = 0x60;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower1 {
    pub raw: u8,
}

impl radio::Register for PaPower1 {
    const ADDRESS: u8 = 0x61;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower0 {
    pub raw: u8,
}

impl radio::Register for PaPower0 {
    const ADDRESS: u8 = 0x62;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig1 {
    pub raw: u8,
}

impl radio::Register for PaConfig1 {
    const ADDRESS: u8 = 0x63;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig0 {
    pub raw: u8,
}

impl radio::Register for PaConfig0 {
    const ADDRESS: u8 = 0x64;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct SynthConfig2 {
    pub raw: u8,
}

impl radio::Register for SynthConfig2 {
    const ADDRESS: u8 = 0x65;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoConfig {
    pub raw: u8,
}

impl radio::Register for VcoConfig {
    const ADDRESS: u8 = 0x68;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn2 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn2 {
    const ADDRESS: u8 = 0x69;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn1 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn1 {
    const ADDRESS: u8 = 0x6A;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn0 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn0 {
    const ADDRESS: u8 = 0x6B;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf1 {
    #[skip]
    _reserved: B3,
    /// disable both dividers of digital clock
    pub pd_clkdiv: bool,
    #[skip]
    _reserved: B4,
}

impl radio::Register for XoRcoConf1 {
    const ADDRESS: u8 = 0x6C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf0 {
    /// Set external reference mode
    pub ext_ref: ExtRefMode,
    /// Set the driver gm of the XO at start up
    pub gm_conf: B3,
    /// Enable the the reference clock divider
    pub ref_div: bool,
    #[skip]
    _reserved: B1,
    /// Enable external RCO, the 34.7 kHz signal must be supplied from any GPIO
    pub ext_rco_osc: bool,
    /// Enable the automatic RCO calibratio
    pub rco_calibration: bool,
}

/// External clock reference mode
#[derive(Copy, Clone, Debug, BitfieldSpecifier)]
#[bits = 1]
pub enum ExtRefMode {
    // Reference from XO circuit
    XO = 0,
    /// Reference from XIN pin
    XIN = 1,
}

impl radio::Register for XoRcoConf0 {
    const ADDRESS: u8 = 0x6D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf3 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrConf3 {
    const ADDRESS: u8 = 0x6E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf2 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrConf2 {
    const ADDRESS: u8 = 0x6F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf4 {
    pub raw: u8,
}

impl radio::Register for PmConf4 {
    const ADDRESS: u8 = 0x75;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf3 {
    pub raw: u8,
}

impl radio::Register for PmConf3 {
    const ADDRESS: u8 = 0x76;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf2 {
    pub raw: u8,
}

impl radio::Register for PmConf2 {
    const ADDRESS: u8 = 0x77;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf1 {
    pub raw: u8,
}

impl radio::Register for PmConf1 {
    const ADDRESS: u8 = 0x78;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf0 {
    pub raw: u8,
}

impl radio::Register for PmConf0 {
    const ADDRESS: u8 = 0x79;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState1 {
    pub raw: u8,
}

impl radio::Register for McState1 {
    const ADDRESS: u8 = 0x8D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState0 {
    pub raw: u8,
}

impl radio::Register for McState0 {
    const ADDRESS: u8 = 0x8E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxFifoStatus {
    pub raw: u8,
}

impl radio::Register for TxFifoStatus {
    const ADDRESS: u8 = 0x8F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxFifoStatus {
    pub raw: u8,
}

impl radio::Register for RxFifoStatus {
    const ADDRESS: u8 = 0x90;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut4 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrOut4 {
    const ADDRESS: u8 = 0x94;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut3 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrOut3 {
    const ADDRESS: u8 = 0x95;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrOut1 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrOut1 {
    const ADDRESS: u8 = 0x99;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrOut0 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrOut0 {
    const ADDRESS: u8 = 0x9A;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxPcktInfo {
    pub raw: u8,
}

impl radio::Register for TxPcktInfo {
    const ADDRESS: u8 = 0x9C;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktInfo {
    pub raw: u8,
}

impl radio::Register for RxPcktInfo {
    const ADDRESS: u8 = 0x9D;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AfcCorr {
    pub raw: u8,
}

impl radio::Register for AfcCorr {
    const ADDRESS: u8 = 0x9E;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif2 {
    pub raw: u8,
}

impl radio::Register for LinkQualif2 {
    const ADDRESS: u8 = 0x9F;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif1 {
    pub raw: u8,
}

impl radio::Register for LinkQualif1 {
    const ADDRESS: u8 = 0xA0;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevel {
    pub raw: u8,
}

impl radio::Register for RssiLevel {
    const ADDRESS: u8 = 0xA2;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen1 {
    pub raw: u8,
}

impl radio::Register for RxPcktLen1 {
    const ADDRESS: u8 = 0xA4;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen0 {
    pub raw: u8,
}

impl radio::Register for RxPcktLen0 {
    const ADDRESS: u8 = 0xA5;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField3 {
    pub raw: u8,
}

impl radio::Register for CrcField3 {
    const ADDRESS: u8 = 0xA6;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField2 {
    pub raw: u8,
}

impl radio::Register for CrcField2 {
    const ADDRESS: u8 = 0xA7;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField1 {
    pub raw: u8,
}

impl radio::Register for CrcField1 {
    const ADDRESS: u8 = 0xA8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField0 {
    pub raw: u8,
}

impl radio::Register for CrcField0 {
    const ADDRESS: u8 = 0xA9;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField1 {
    pub raw: u8,
}

impl radio::Register for RxAddreField1 {
    const ADDRESS: u8 = 0xAA;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField0 {
    pub raw: u8,
}

impl radio::Register for RxAddreField0 {
    const ADDRESS: u8 = 0xAB;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevelRun {
    pub raw: u8,
}

impl radio::Register for RssiLevelRun {
    const ADDRESS: u8 = 0xEF;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo1 {
    pub part_no: u8,
}

impl radio::Register for DeviceInfo1 {
    const ADDRESS: u8 = 0xF0;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo0 {
    pub version: u8,
}

impl radio::Register for DeviceInfo0 {
    const ADDRESS: u8 = 0xF1;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus3 {
    pub raw: u8,
}

impl radio::Register for IrqStatus3 {
    const ADDRESS: u8 = 0xFA;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus2 {
    pub raw: u8,
}

impl radio::Register for IrqStatus2 {
    const ADDRESS: u8 = 0xFB;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus1 {
    pub raw: u8,
}

impl radio::Register for IrqStatus1 {
    const ADDRESS: u8 = 0xFC;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus0 {
    pub raw: u8,
}

impl radio::Register for IrqStatus0 {
    const ADDRESS: u8 = 0xFD;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Fifo {
    pub raw: u8,
}

impl radio::Register for Fifo {
    const ADDRESS: u8 = 0xFF;
    type Word = u8;
    type Error = Infallible;
}



//*******************************************************//

/// Switch mode power supply [SMPS] Voltage levels
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum SMPSLevel {
    Smps1v2 = 0b001,
    Smps1v3 = 0b010,
    Smps1v4 = 0b011,
    Smps1v5 = 0b100,
    Smps1v6 = 0b101,
    Smps1v7 = 0b110,
    Smps1v8 = 0b111,
}

/// Clock Frequencies
#[derive(Copy, Clone, Debug)]
pub enum ClockFreq {
    Clock24MHz,
    Clock25MHz,
    Clock26MHz,
    Clock48MHz,
    Clock50MHz,
}

/// Supported frequency bands
pub const BANDS: [(u16, u16); 3] = [(430, 470), (470, 512), (860, 940)];

/// Check if a frequency is in one of the valid bands
pub fn frequency_valid(frequency: u16) -> bool {
    for (l, h) in BANDS.iter() {
        if frequency > *l && frequency < *h {
            return true;
        }
    }
    return false;
}

/// Clock recovery post filter length
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum PostFilterLen {
    FilterLen8 = 0x00,
    FilterLen16 = 0x01,
}

/// Packet handler packet format
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum PacketFormat {
    Basic = 0x00,
    FifteenFour = 0x01,
    UartOTA = 0x02,
    Stack = 0x03,
}

/// Packet handler receive mode
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum RXMode {
    Normal = 0x00,
    DirectFIFO = 0x01,
    DirectGPIO = 0x02,
}



/// External SMPS mode
#[derive(Copy, Clone, Debug)]
pub enum ExtSmpsMode {
    Enable = 0x00,
    Disable = 0x20,
}

pub const PM_CONF0_EXT_SMPS_REGMASK: u8 = 0x20;

/// Radio states (table 48)
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum State {
    Shutdown = 0xFF,
    Standby = 0x02,
    SleepA = 0x01, // Sleep without FIFO retention
    SleepB = 0x03, // Sleep with FIFO retention
    Ready = 0x00,
    Lock = 0x0C,
    Rx = 0x30,
    Tx = 0x5C,
    SynthSetup = 0x50,
}

impl radio::RadioState for State {
    fn idle() -> Self {
        State::Standby
    }

    fn sleep() -> Self {
        State::SleepA
    }
}

/// Radio commands (table 49)
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum Command {
    Tx = 0x60,             // from: READY Send the S2-LP to TX state for transmission
    Rx = 0x61,             // from: READY Send the S2-LP to RX state for reception
    Ready = 0x62,          // from: STANDBY, SLEEP, LOCK Go to READY state
    Standby = 0x63,        // from: READY Go to STANDBY state
    Sleep = 0x64,          // from: READY Go to SLEEP state
    LockRx = 0x65, // from: READY Go to LOCK state by using the RX configuration of the synthesizer
    LockRx2 = 0x66, // from: READY Go to LOCK state by using the TX configuration of the synthesizer
    SAbort = 0x67, // from: TX, RX Exit from TX or RX states and go to READY state
    LDCReload = 0x68, // from: ANY Reload the LDC timer with a pre-programmed value storedin registers
    Reset = 0x70,     // from: ANY Reset the S2-LP state machine and registers values
    FlushRXFifo = 0x71, // from: All Clean the RX FIFO
    FlushTxFifo = 0x72, // from: All Clean the TX FIFO
    SequenceUpdate = 0x73, // from: TE ANY Reload the packet sequence counter with the value stored in register
}
