// NOTE: the modular-bitfield crate, version 0.11.2, when using
// the bitfield attribute proc-macro, makes rustc throw this warning
// on the proc-macro-generated code
#![allow(unused_braces)]

use modular_bitfield::{bitfield, prelude::*};

/// Macro by example used to reduce the boilerplate when implementing
/// [`Default`] and [`radio::Register`] for the types of this module 
macro_rules! derive {
    (Default for $for:ty = bits: $value:expr) => {
        impl Default for $for {
            fn default() -> Self {
                Self::from_bytes([$value])
            }
        }
    };

    (radio::Register for $for:ty = address: $address:expr) => {
        impl radio::Register for $for {
            const ADDRESS: u8 = $address;
            type Word = u8;
            type Error = ::core::convert::Infallible;
        }
    };
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio0Conf {
    /// GPIO0 Mode:
    /// - `00b`: Analog (Hi-Z),
    /// - `01b`: Digital input,
    /// - `10b`: Digital output low power,
    /// - `11b`: Digital output high power
    pub gpio_mode: B2,
    #[skip]
    pub _reserved: B1,
    /// Specify the GPIO0 I/O signal, default setting POR
    /// (see Table 61. GPIO digital output functions).
    pub gpio_select: B5,
}

derive!(Default for Gpio0Conf = bits: 0x0A);
derive!(radio::Register for Gpio0Conf = address: 0x00);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio1Conf {
    /// GPIO1 mode:
    /// - `00b`: Analog (Hi-Z),
    /// - `01b`: Digital input1,
    /// - `10b`: Digital output low power,
    /// - `11b`: Digital Output High Power
    pub gpio_mode: B2,
    #[skip]
    pub _reserved: B1,
    /// Specify the GPIO1 I/O signal, default setting digital GND
    /// (see Table 61. GPIO digital output functions).
    pub gpio_select: B5,
}

derive!(Default for Gpio1Conf = bits: 0xA2);
derive!(radio::Register for Gpio1Conf = address: 0x01);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio2Conf {
    /// GPIO2 mode:
    /// - `00b`: Analog (Hi-Z),
    /// - `01b`: Digital input,
    /// - `10b`: Digital output low power,
    /// - `11b`: Digital output high power
    pub gpio_mode: B2,
    #[skip]
    pub _reserved: B1,
    /// Specify the GPIO2 I/O signal, default setting digital GND
    /// (see Table 61. GPIO digital output functions).
    pub gpio_select: B5,
}

derive!(Default for Gpio2Conf = bits: 0xA2);
derive!(radio::Register for Gpio2Conf = address: 0x02);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio3Conf {
    /// GPIO3 mode:
    /// - `00b`: Analog (Hi-Z),
    /// - `00b`: Analog,
    /// - `01b`: Digital Input,
    /// - `10b`: Digital Output Low Power,
    /// - `11b`: Digital Output High Power
    pub gpio_mode: B2,
    #[skip]
    pub _reserved: B1,
    /// Specify the GPIO3 I/O signal, default setting digital GND
    /// (see Table 61. GPIO digital output functions).
    pub gpio_select: B5,
}

derive!(Default for Gpio3Conf = bits: 0xA2);
derive!(radio::Register for Gpio3Conf = address: 0x03);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt3 {
    /// `[27:24]` MSB bits of the PLL programmable divider
    /// (see Section 5.3.1 RF channel frequency settings).
    pub synt: B4,
    /// Synthesizer band select.
    ///
    /// This parameter selects the out-of loop divide factor of the synthesizer:
    /// - `0`: B1, band select factor for high band,
    /// - `1`: 8, band select factor for middle band
    ///
    /// (see Section 5.3.1 RF channel frequency settings).
    pub bs: B1,
    /// Set the charge pump current according to the XTAL frequency
    /// (see Table 37. Charge pump words).
    pub pll_cp_isel: B3,
}

derive!(Default for Synt3 = bits: 0x42);
derive!(radio::Register for Synt3 = address: 0x05);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt2 {
    /// `[23:16]` Intermediate bits of the PLL programmable divider
    /// (see Section 5.3.1 RF channel frequency settings).
    pub synt: B8,
}

derive!(Default for Synt2 = bits: 0x16);
derive!(radio::Register for Synt2 = address: 0x06);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt1 {
    /// `[15:8]` Intermediate bits of the PLL programmable divider
    /// (see Section 5.3.1 RF channel frequency settings).
    pub synt: B8,
}

derive!(Default for Synt1 = bits: 0x27);
derive!(radio::Register for Synt1 = address: 0x07);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt0 {
    /// `[7:0]` LSB bits of the PLL programmable divider
    /// (see Section 5.3.1 RF channel frequency settings).
    pub synt: B8,
}

derive!(Default for Synt0 = bits: 0x62);
derive!(radio::Register for Synt0 = address: 0x08);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetAna {
    /// Intermediate frequency setting for the analog RF synthesizer,
    /// default: 300 kHz, see Eq. (15).
    pub if_offset_ana: B8,
}

derive!(Default for IfOffsetAna = bits: 0x2A);
derive!(radio::Register for IfOffsetAna = address: 0x09);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetDig {
    /// Intermediate frequency setting for the digital shift-to-baseband circuits,
    /// default: 300 kHz, see Eq. (15).
    pub if_offset_dig: B8,
}

derive!(Default for IfOffsetDig = bits: 0xB8);
derive!(radio::Register for IfOffsetDig = address: 0x0A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Chspace {
    /// Channel spacing setting, see Eq. (16).
    pub ch_space: B8,
}

derive!(Default for Chspace = bits: 0x3F);
derive!(radio::Register for Chspace = address: 0x0C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Chnum {
    /// Channel number.
    ///
    /// This value is multiplied by the channel spacing and added to the synthesizer
    /// base frequency to generate the actual RF carrier frequency, see Eq. (16).
    pub ch_num: B8,
}

derive!(Default for Chnum = bits: 0x00);
derive!(radio::Register for Chnum = address: 0x0D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod4 {
    /// `[15:8]` The MSB of the mantissa value of the data rate equation, see Eq. (14).
    pub datarate_m: B8,
}

derive!(Default for Mod4 = bits: 0x83);
derive!(radio::Register for Mod4 = address: 0x0E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod3 {
    /// `[7:0]` The LSB of the mantissa value of the data rate equation, see Eq. (14).
    pub datarate_m: B8,
}

derive!(Default for Mod3 = bits: 0x2B);
derive!(radio::Register for Mod3 = address: 0x0F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod2 {
    /// The exponent value of the data rate equation (see Eq. (14)).
    pub datarate_e: B4,
    /// Modulation type:
    /// - `0`: 2-FSK,
    /// - `1`: 4-FSK,
    /// - `2`: 2-GFSK BT=1,
    /// - `3`: 4-GFSK BT=1,
    /// - `5`: ASK/OOK,
    /// - `7`: unmodulated,
    /// - `10`: 2-GFSK BT=0.5,
    /// - `11`: 4-GFSK BT=0.5
    pub mod_type: B4,
}

derive!(Default for Mod2 = bits: 0x77);
derive!(radio::Register for Mod2 = address: 0x10);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod1 {
    /// The exponent value of the frequency deviation equation (see Eq. (10)).
    pub fdev_e: B4,
    /// Select the constellation map for 4-(G)FSK or 2-(G)FSK modulations
    /// (see Table 41. Constellation mapping 2-(G)FSK and Table 42. Constellation mapping 4-(G)FSK).
    pub const_map: B2,
    /// - `1`: enable frequency interpolator for the GFSK shaping
    ///
    /// (see Section 5.4.1.1 Gaussian shaping).
    pub mod_interp_en: B1,
    /// - `1`: enable the PA power interpolator
    ///
    /// (see Section 5.6.1 PA configuration).
    pub pa_interp_en: B1,
}

derive!(Default for Mod1 = bits: 0x03);
derive!(radio::Register for Mod1 = address: 0x11);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod0 {
    /// The mantissa value of the frequency deviation equation (see Eq. (10)).
    pub fdev_m: B8,
}

derive!(Default for Mod0 = bits: 0x93);
derive!(radio::Register for Mod0 = address: 0x12);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Chflt {
    /// The exponent value of the receiver channel filter
    /// (see Table 44. Channel filter words).
    pub chflt_e: B4,
    /// The mantissa value of the receiver channel filter
    /// (see Table 44. Channel filter words).
    pub chflt_m: B4,
}

derive!(Default for Chflt = bits: 0x23);
derive!(radio::Register for Chflt = address: 0x13);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc2 {
    #[skip]
    pub _reserved: B5,
    /// Select AFC mode:
    /// - `0`: AFC loop closed on slicer,
    /// - `1`: AFC loop closed on second conversion stage.
    pub afc_mode: B1,
    /// - `1`: enable the AFC correction.
    pub afc_enabled: B1,
    /// - `1`: enable the freeze AFC correction upon sync word detection.
    pub afc_freeze_on_sync: B1,
}

derive!(Default for Afc2 = bits: 0xC8);
derive!(radio::Register for Afc2 = address: 0x14);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc1 {
    /// The length of the AFC fast period.
    pub afc_fast_period: B8,
}

derive!(Default for Afc1 = bits: 0x18);
derive!(radio::Register for Afc1 = address: 0x15);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc0 {
    /// The AFC loop gain in slow mode (2's log).
    pub afc_slow_gain: B4,
    /// The AFC loop gain in fast mode (2's log).
    pub afc_fast_gain: B4,
}

derive!(Default for Afc0 = bits: 0x25);
derive!(radio::Register for Afc0 = address: 0x16);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiFlt {
    #[skip]
    pub _reserved: B2,
    /// Carrier sense mode:
    /// - `00b`: Static CS,
    /// - `01b`: Dynamic CS with 6dB dynamic threshold,
    /// - `10b`: Dynamic CS with 12dB dynamic threshold,
    /// - `11b`: Dynamic CS with 18dB dynamic threshold.
    ///
    /// (see Section 5.5.8.2 Carrier sense)
    pub cs_mode: B2,
    /// Gain of the RSSI filter.
    pub rssi_flt: B4,
}

derive!(Default for RssiFlt = bits: 0xE3);
derive!(radio::Register for RssiFlt = address: 0x17);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiTh {
    /// Signal detect threshold in 1 dB steps.
    /// The `RSSI_TH` can be converted in dBm using the formula `RSSI_TH - 146`.
    pub rssi_th: B8,
}

derive!(Default for RssiTh = bits: 0x28);
derive!(radio::Register for RssiTh = address: 0x18);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl4 {
    /// Low threshold 1 for the AGC
    pub low_threshold_1: B4,
    /// Low threshold 0 for the AGC
    pub low_threshold_0: B4,
}

derive!(Default for Agcctrl4 = bits: 0x54);
derive!(radio::Register for Agcctrl4 = address: 0x1A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl3 {
    /// Low threshold selection (defined in the AGCCTRL4).
    /// Bitmask for each attenuation step.
    pub low_threshold_sel: B8,
}

derive!(Default for Agcctrl3 = bits: 0x10);
derive!(radio::Register for Agcctrl3 = address: 0x1B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl2 {
    /// AGC measurement time
    pub meas_time: B4,
    #[skip]
    pub _reserved: B1,
    /// Enable the AGC algorithm to be frozen on SYNC
    pub freeze_on_sync: B1,
    #[skip]
    pub _reserved: B2,
}

derive!(Default for Agcctrl2 = bits: 0x22);
derive!(radio::Register for Agcctrl2 = address: 0x1C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl1 {
    #[skip]
    pub _reserved: B4,
    /// High threshold for the AGC
    pub high_threshold: B4,
}

derive!(Default for Agcctrl1 = bits: 0x59);
derive!(radio::Register for Agcctrl1 = address: 0x1D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl0 {
    /// Hold time for after gain adjustment for the AGC.
    pub hold_time: B6,
    #[skip]
    pub _reserved: B1,
    /// - `0`: disabled,
    /// - `1`: enabled
    pub agc_enable: B1,
}

derive!(Default for Agcctrl0 = bits: 0x8C);
derive!(radio::Register for Agcctrl0 = address: 0x1E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AntSelectConf {
    /// Set the measurement time.
    pub as_meas_time: B3,
    /// Enable the antenna switching (see Section 5.5.10 Antenna switching).
    pub as_enable: bool,
    /// Do not fill the RX FIFO with data if the CS is threshold
    /// (see Section 5.5.9 CS blanking).
    pub cs_blanking: B1,
    /// ISI cancellation equalizer:
    /// - `00b`: equalization disabled,
    /// - `01b`: single pass equalization,
    /// - `10b`: dual pass equalization.
    ///
    /// (see Section 5.4.1.2 ISI cancellation 4-(G)FSK)
    pub equ_ctrl: B2,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for AntSelectConf = bits: 0x45);
derive!(radio::Register for AntSelectConf = address: 0x1F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec2 {
    /// Set the integral slow gain for symbol timing recovery
    /// (PLL mode only).
    pub clk_rec_i_gain_slow: B4,
    /// Select the symbol timing recovery algorithm:
    /// - `0`: DLL,
    /// - `1`: PLL.
    pub clk_rec_algo_sel: B1,
    /// Clock recovery slow loop gain (log2).
    pub clk_rec_p_gain_slow: B3,
}

derive!(Default for Clockrec2 = bits: 0xC0);
derive!(radio::Register for Clockrec2 = address: 0x20);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec1 {
    /// Set the integral fast gain for symbol timing recovery
    /// (PLL mode only).
    pub clk_rec_i_gain_fast: B4,
    /// Select the post filter length:
    /// - `0`: 8 symbols,
    /// - `1`: 16 symbols.
    pub pstflt_len: B1,
    /// Clock recovery fast loop gain (log2).
    pub clk_rec_p_gain_fast: B3,
}

derive!(Default for Clockrec1 = bits: 0x58);
derive!(radio::Register for Clockrec1 = address: 0x21);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl6 {
    /// `[9:8]` The MSB of the number of '01 or '10' of the preamble of the packet.
    pub preamble_len: B2,
    /// The number of bits used for the SYNC field in the packet.
    pub sync_len: B6,
}

derive!(Default for Pcktctrl6 = bits: 0x80);
derive!(radio::Register for Pcktctrl6 = address: 0x2B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl5 {
    /// `[7:0]` The LSB of the number of '01 or '10' of the preamble of the packet.
    pub preamble_len: B8,
}

derive!(Default for Pcktctrl5 = bits: 0x10);
derive!(radio::Register for Pcktctrl5 = address: 0x2C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl4 {
    #[skip]
    pub _reserved: B3,
    /// Include the ADDRESS field in the packet.
    pub address_len: bool,
    #[skip]
    pub _reserved: B3,
    /// The number of bytes used for the length field:
    /// - `0`: 1 byte,
    /// - `1`: 2 bytes.
    pub len_wid: B1,
}

derive!(Default for Pcktctrl4 = bits: 0x00);
derive!(radio::Register for Pcktctrl4 = address: 0x2D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl3 {
    /// Select the preamble pattern.
    pub preamble_sel: B2,
    /// Select the transmission order between MSB and LSB.
    pub byte_swap: B1,
    /// Select the symbol mapping for 4(G)FSK.
    pub fsk4_sym_swap: B1,
    /// RX mode:
    /// - `0`: normal mode,
    /// - `1`: direct through FIFO,
    /// - `2`: direct through GPIO
    pub rx_mode: B2,
    /// Format of packet:
    /// - `0`: Basic,
    /// - `1`: 802.15.4g,
    /// - `2`: UART OTA,
    /// - `3`: Stack (see Section 7 Packet handler engine)
    pub pckt_frmt: B2,
}

derive!(Default for Pcktctrl3 = bits: 0x20);
derive!(radio::Register for Pcktctrl3 = address: 0x2E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl2 {
    /// Packet length mode:
    /// - `0`: fixed,
    /// - `1`: variable
    ///
    /// (in variable mode the field LEN_WID of PCKTCTRL3 register must be configured)
    pub fix_var_len: B1,
    /// Enable the Manchester encoding/decoding.
    pub manchester_en: bool,
    /// Enable the 3-out-of-6 encoding/decoding.
    pub mbus_3of6_en: bool,
    /// If the 802.15.4 mode is enabled, 1: enable the interleaving of 802.15.4g packet.
    ///
    /// If the UART packet is enabled, this is the value of the START_BIT.
    pub int_en_4g_or_start_bit: B1,
    /// If the 802.15.4 mode is enabled, this is the FCS type in header field of 802.15.4g packet.
    ///
    /// Select the FEC type of 802.15.4g packet:
    /// – 0: NRNSC
    /// – 1: RSC.
    ///
    /// If the UART packet is enabled, this is the value of the STOP_BIT.
    pub fec_type_4g_or_stop_bit: B1,
    /// This is the FCS type in header field of 802.15.4g packet.
    pub fcs_type_4g: B1,
    #[skip]
    pub _reserved: B2,
}

derive!(Default for Pcktctrl2 = bits: 0x00);
derive!(radio::Register for Pcktctrl2 = address: 0x2F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl1 {
    /// Enable the FEC encoding in TX or the Viterbi decoding in RX.
    pub fec_en: bool,
    /// In TX mode:
    /// - 0 select the primary SYNC word,
    /// - 1 select the secondary SYNC word.
    ///
    /// In RX mode, if 1 enable the dual SYNC word detection mode.
    pub second_sync_sel: B1,
    /// Tx source data:
    /// - `0`: normal mode,
    /// - `1`: direct through FIFO,
    /// - `2`: direct through GPIO,
    /// - `3`: PN9
    pub txsource: B2,
    /// - `1`: enable the whitening mode.
    pub whit_en: B1,
    /// CRC field:
    /// - `0`: no CRC field,
    /// - `1`: CRC using poly 0x07
    /// - `2`: CRC using poly 0x8005,
    /// - `3`: CRC using poly 0x1021,
    /// - `4`: CRC using poly 0x864CBF,
    /// - `5`: CRC using poly
    pub crc_mode: B3,
}

derive!(Default for Pcktctrl1 = bits: 0x2C);
derive!(radio::Register for Pcktctrl1 = address: 0x30);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen1 {
    /// MSB of length of packet in bytes.
    pub pcktlen1: B8,
}

derive!(Default for Pcktlen1 = bits: 0x00);
derive!(radio::Register for Pcktlen1 = address: 0x31);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen0 {
    /// LSB of length of packet in bytes.
    pub pcktlen0: B8,
}

derive!(Default for Pcktlen0 = bits: 0x14);
derive!(radio::Register for Pcktlen0 = address: 0x32);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync3 {
    /// SYNC word byte 3.
    pub sync3: B8,
}

derive!(Default for Sync3 = bits: 0x88);
derive!(radio::Register for Sync3 = address: 0x33);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync2 {
    /// SYNC word byte 2.
    pub sync2: B8,
}

derive!(Default for Sync2 = bits: 0x88);
derive!(radio::Register for Sync2 = address: 0x34);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync1 {
    /// SYNC word byte 1.
    pub sync1: B8,
}

derive!(Default for Sync1 = bits: 0x88);
derive!(radio::Register for Sync1 = address: 0x35);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync0 {
    /// SYNC word byte 0.
    pub sync0: B8,
}

derive!(Default for Sync0 = bits: 0x88);
derive!(radio::Register for Sync0 = address: 0x36);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Qi {
    /// Enable the SQI check.
    pub sqi_en: bool,
    /// PQI threshold.
    pub pqi_th: B4,
    /// SQI threshold.
    pub sqi_th: B3,
}

derive!(Default for Qi = bits: 0x01);
derive!(radio::Register for Qi = address: 0x37);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktPstmbl {
    /// Set the packet postamble length.
    pub pckt_pstmbl: B8,
}

derive!(Default for PcktPstmbl = bits: 0x00);
derive!(radio::Register for PcktPstmbl = address: 0x38);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol2 {
    /// Set the LDC timer multiplier factor:
    /// - `00b`: x1,
    /// - `01b`: x2,
    /// - `10b`: x4,
    /// - `11b`: x8.
    pub ldc_timer_mult: B2,
    /// - `0`: select the almost empty/full control for TX FIFO.
    /// - `1`: select the almost empty/full control for RX FIFO.
    pub fifo_gpio_out_mux_sel: B1,
    /// TX sequence number to be used when counting reset is required using the related command.
    pub tx_seq_num_reload: B2,
    /// Enable the PQI value contributes to timeout disabling.
    pub pqi_timeout_mask: bool,
    /// Enable the SQI value contributes to timeout disabling.
    pub sqi_timeout_mask: bool,
    /// Enable the CS value contributes to timeout disabling.
    pub cs_timeout_mask: bool,
}

derive!(Default for Protocol2 = bits: 0x40);
derive!(radio::Register for Protocol2 = address: 0x39);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol1 {
    /// Enable the automatic packet filtering control.
    pub auto_pckt_flt: bool,
    /// Enable the CSMA persistent mode (no back-off cycles).
    pub csma_pers_on: bool,
    /// Enable the CSMA channel access mode.
    pub csma_on: bool,
    /// Enable the reload of the back-off random generator seed using the value written in the BU_COUNTER_SEED.
    pub seed_reload: bool,
    /// Enable the RX sniff timer.
    pub fast_cs_term_en: bool,
    /// Enable the piggybacking.
    pub piggybacking: bool,
    /// Enable the LDC timer reload mode.
    pub ldc_reload_on_sync: bool,
    /// Enable the Low Duty Cycle mode.
    pub ldc_mode: bool,
}

derive!(Default for Protocol1 = bits: 0x00);
derive!(radio::Register for Protocol1 = address: 0x3A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol0 {
    #[skip]
    pub _reserved: B1,
    /// Enable the persistent RX mode.
    pub pers_rx: bool,
    /// Enable the automatic acknowledgment if packet received request.
    pub auto_ack: bool,
    /// Field NO_ACK=1 on transmitted packet.
    pub nack_tx: bool,
    /// Max. number of re-TX (from 0 to 15)(0: re-transmission is not performed).
    pub nmax_retx: B4,
}

derive!(Default for Protocol0 = bits: 0x08);
derive!(radio::Register for Protocol0 = address: 0x3B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig3 {
    /// Set the RX FIFO almost full threshold.
    pub rx_afthr: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for FifoConfig3 = bits: 0x30);
derive!(radio::Register for FifoConfig3 = address: 0x3C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig2 {
    /// Set the RX FIFO almost empty threshold.
    pub rx_aethr: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for FifoConfig2 = bits: 0x30);
derive!(radio::Register for FifoConfig2 = address: 0x3D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig1 {
    /// Set the TX FIFO almost full threshold.
    pub tx_afthr: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for FifoConfig1 = bits: 0x30);
derive!(radio::Register for FifoConfig1 = address: 0x3E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig0 {
    /// Set the TX FIFO almost empty threshold.
    pub tx_aethr: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for FifoConfig0 = bits: 0x30);
derive!(radio::Register for FifoConfig0 = address: 0x3F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltOptions {
    /// Packet discarded if CRC is not valid.
    pub crc_flt: bool,
    /// RX packet accepted if its destination address matches with `RX_SOURCE_ADDR` register.
    pub dest_vs_source_addr: bool,
    /// RX packet accepted if its destination address matches with `MULTICAST_ADDR` register.
    pub dest_vs_multicast_addr: bool,
    /// RX packet accepted if its source field matches with `BROADCAST_ADDR` register.
    pub dest_vs_broadcast_addr: bool,
    /// RX packet accepted if its source field matches with `RX_SOURCE_ADDR` register
    pub source_addr_flt: bool,
    #[skip]
    pub _reserved: B1,
    /// Logical Boolean function applied to CS/SQI/PQI values:
    /// - `1`: OR,
    /// - `0`: AND.
    pub rx_timeout_and_or_sel: B1,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PcktFltOptions = bits: 0x40);
derive!(radio::Register for PcktFltOptions = address: 0x40);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals4 {
    /// Mask register for source address filtering.
    pub rx_source_mask: B8,
}

derive!(Default for PcktFltGoals4 = bits: 0x00);
derive!(radio::Register for PcktFltGoals4 = address: 0x41);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals3 {
    /// If dual sync mode enabled: dual SYNC word byte 3,
    /// otherwise RX packet source or TX packet destination field.
    pub rx_source_addr_or_dual_sync3: B8,
}

derive!(Default for PcktFltGoals3 = bits: 0x00);
derive!(radio::Register for PcktFltGoals3 = address: 0x42);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals2 {
    /// If dual sync mode enabled: dual SYNC word byte 2,
    /// otherwise broadcast address.
    pub broadcast_addr_or_dual_sync2: B8,
}

derive!(Default for PcktFltGoals2 = bits: 0x00);
derive!(radio::Register for PcktFltGoals2 = address: 0x43);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals1 {
    /// If dual sync mode enabled: dual SYNC word byte 1,
    /// otherwise multicast address.
    pub multicast_addr_or_dual_sync1: B8,
}

derive!(Default for PcktFltGoals1 = bits: 0x00);
derive!(radio::Register for PcktFltGoals1 = address: 0x44);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals0 {
    /// If dual sync mode enabled: dual SYNC word byte 0,
    /// othersise TX packet source or RX packet destination field.
    pub tx_source_addr_or_dual_sync0: B8,
}

derive!(Default for PcktFltGoals0 = bits: 0x00);
derive!(radio::Register for PcktFltGoals0 = address: 0x45);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers5 {
    /// Counter for RX timer.
    pub rx_timer_cntr: B8,
}

derive!(Default for Timers5 = bits: 0x01);
derive!(radio::Register for Timers5 = address: 0x46);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers4 {
    /// Prescaler for RX timer.
    pub rx_timer_presc: B8,
}

derive!(Default for Timers4 = bits: 0x00);
derive!(radio::Register for Timers4 = address: 0x47);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers3 {
    /// Prescaler for wake up timer.
    pub ldc_timer_presc: B8,
}

derive!(Default for Timers3 = bits: 0x01);
derive!(radio::Register for Timers3 = address: 0x48);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers2 {
    /// Counter for wake up timer.
    pub ldc_timer_cntr: B8,
}

derive!(Default for Timers2 = bits: 0x00);
derive!(radio::Register for Timers2 = address: 0x49);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers1 {
    /// Prescaler value for reload operation of wake up timer.
    pub ldc_reload_prsc: B8,
}

derive!(Default for Timers1 = bits: 0x01);
derive!(radio::Register for Timers1 = address: 0x4A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers0 {
    /// Counter value for reload operation of wake up timer.
    pub ldc_reload_cntr: B8,
}

derive!(Default for Timers0 = bits: 0x00);
derive!(radio::Register for Timers0 = address: 0x4B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf3 {
    /// `[14:8]` MSB part of the seed for the random generator
    /// used to apply the CSMA algorithm.
    pub bu_cntr_seed: B8,
}

derive!(Default for CsmaConf3 = bits: 0x4C);
derive!(radio::Register for CsmaConf3 = address: 0x4C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf2 {
    /// `[7:0]` LSB part of the seed for the random generator
    /// used to apply the CSMA algorithm.
    pub bu_cntr_seed: B8,
}

derive!(Default for CsmaConf2 = bits: 0x00);
derive!(radio::Register for CsmaConf2 = address: 0x4D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf1 {
    /// Multiplier for the Tcca timer.
    pub cca_period: B2,
    /// Prescaler value for the back-off unit BU.
    pub bu_prsc: B6,
}

derive!(Default for CsmaConf1 = bits: 0x04);
derive!(radio::Register for CsmaConf1 = address: 0x4E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf0 {
    /// Max number of back-off cycles.
    pub nbackoff_max: B3,
    #[skip]
    pub _reserved: B1,
    /// The number of time in which the listen operation is performed.
    pub cca_len: B4,
}

derive!(Default for CsmaConf0 = bits: 0x00);
derive!(radio::Register for CsmaConf0 = address: 0x4F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask3 {
    /// `[31:24]` Enable the routing of the interrupt flag on the configured IRQ GPIO.
    pub int_mask: B8,
}

derive!(Default for IrqMask3 = bits: 0x00);
derive!(radio::Register for IrqMask3 = address: 0x50);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask2 {
    /// `[23:16]` Enable the routing of the interrupt flag on the configured IRQ GPIO.
    pub int_mask: B8,
}

derive!(Default for IrqMask2 = bits: 0x00);
derive!(radio::Register for IrqMask2 = address: 0x51);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask1 {
    /// `[15:8]` Enable the routing of the interrupt flag on the configured IRQ GPIO.
    pub int_mask: B8,
}

derive!(Default for IrqMask1 = bits: 0x00);
derive!(radio::Register for IrqMask1 = address: 0x52);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask0 {
    /// `[7:0]` Enable the routing of the interrupt flag on the configured IRQ GPIO.
    pub int_mask: B8,
}

derive!(Default for IrqMask0 = bits: 0x00);
derive!(radio::Register for IrqMask0 = address: 0x53);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FastRxTimer {
    /// Sniff timer configuration.
    pub rssi_settling_limit: B8,
}

derive!(Default for FastRxTimer = bits: 0x28);
derive!(radio::Register for FastRxTimer = address: 0x54);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower8 {
    /// Output power level for 8th slot.
    pub pa_level8: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower8 = bits: 0x01);
derive!(radio::Register for PaPower8 = address: 0x5A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower7 {
    /// Output power level for 7th slot.
    pub pa_level_7: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower7 = bits: 0x0C);
derive!(radio::Register for PaPower7 = address: 0x5B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower6 {
    /// Output power level for 6th slot.
    pub pa_level_6: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower6 = bits: 0x18);
derive!(radio::Register for PaPower6 = address: 0x5C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower5 {
    /// Output power level for 5th slot.
    pub pa_level_5: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower5 = bits: 0x24);
derive!(radio::Register for PaPower5 = address: 0x5D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower4 {
    /// Output power level for 4th slot.
    pub pa_level_4: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower4 = bits: 0x30);
derive!(radio::Register for PaPower4 = address: 0x5E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower3 {
    /// Output power level for 3rd slot.
    pub pa_level_3: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower3 = bits: 0x48);
derive!(radio::Register for PaPower3 = address: 0x5F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower2 {
    /// Output power level for 2nd slot.
    pub pa_level_2: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower2 = bits: 0x60);
derive!(radio::Register for PaPower2 = address: 0x60);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower1 {
    /// Output power level for 1st slot.
    pub pa_level_1: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PaPower1 = bits: 0x00);
derive!(radio::Register for PaPower1 = address: 0x61);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower0 {
    /// Final level for power ramping or selected output power index.
    pub pa_level_max_idx: B3,
    /// Set the step width (unit: 1/8 of bit period).
    pub pa_ramp_step_len: B2,
    /// Enable the power ramping.
    pub pa_ramp_en: bool,
    /// Configure the PA to send maximum output power.
    ///
    /// Power ramping is disable with this bit set to 1.
    pub pa_maxdbm: bool,
    /// Enable the generation of the internal signal TX_DATA which is the input of the FIR.
    ///
    /// Needed when `FIR_EN = 1`.
    pub dig_smooth_en: bool,
}

derive!(Default for PaPower0 = bits: 0x47);
derive!(radio::Register for PaPower0 = address: 0x62);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig1 {
    #[skip]
    pub _reserved: B1,
    /// Enable FIR (see Section 5.4.2.1 OOK smoothing)
    pub fir_en: bool,
    /// FIR configuration:
    /// - `00b`: filtering
    /// - `01b`: ramping
    /// - `10b`: switching
    ///
    /// (see Section 5.4.2.1 OOK smoothing)
    pub fir_cfg: B2,
    #[skip]
    pub _reserved: B4,
}

derive!(Default for PaConfig1 = bits: 0x03);
derive!(radio::Register for PaConfig1 = address: 0x63);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig0 {
    /// PA bessel filter bandwidth:
    /// - `00b`: 12.5 kHz (data rate 16.2 kbps),
    /// - `01b`: 25 kHz (data rate 32 kbps),
    /// - `10b`: 50 kHz (data rate 62.5 kbps),
    /// - `11b`: 100 kHz (data rate 125 kbps)
    ///
    /// (see Section 5.4.2.1 OOK smoothing).
    pub pa_fc: B2,
    /// During a TX operation, enables and starts the digital ASK calibrator.
    pub safe_ask_cal: bool,
    /// Enables the 'degeneration' mode that introduces a pre-distortion to linearize the power control curve.
    pub pa_degen_on: bool,
    /// - 11xx ® code threshold: 485,
    /// - 10xx ® code threshold: 465,
    /// - 01xx ® code threshold: 439,
    /// - 00xx ® code threshold: 418,
    /// - xx11 ® clamp voltage: 0.55 V,
    /// - xx10 ® clamp voltage: 0.50 V,
    /// - xx01 ® clamp voltage: 0.45 V,
    /// - xx00 ® clamp voltage: 0.40 V.
    pub pa_degen_trim: B4,
}

derive!(Default for PaConfig0 = bits: 0x8A);
derive!(radio::Register for PaConfig0 = address: 0x64);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct SynthConfig2 {
    #[skip]
    pub _reserved: B2,
    /// Enables increased DN current pulses to improve linearization of CP/PFD
    /// (see Table 37. Charge pump words).
    pub pll_pfd_split_en: bool,
    #[skip]
    pub _reserved: B5,
}

derive!(Default for SynthConfig2 = bits: 0xD0);
derive!(radio::Register for SynthConfig2 = address: 0x65);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoConfig {
    #[skip]
    pub _reserved: B4,
    /// VCO frequency calibration is skipped
    ///
    /// (external amplitude word forced on VCO).
    pub vco_calfreq_ext_sel: bool,
    /// VCO amplitude calibration is skipped
    ///
    /// (external amplitude word forced on VCO).
    pub vco_calamp_ext_sel: bool,
    #[skip]
    pub _reserved: B2,
}

derive!(Default for VcoConfig = bits: 0x03);
derive!(radio::Register for VcoConfig = address: 0x68);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn2 {
    #[skip]
    pub _reserved: B8,
}

derive!(Default for VcoCalibrIn2 = bits: 0x88);
derive!(radio::Register for VcoCalibrIn2 = address: 0x69);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn1 {
    #[skip]
    pub _reserved: B8,
}

derive!(Default for VcoCalibrIn1 = bits: 0x40);
derive!(radio::Register for VcoCalibrIn1 = address: 0x6A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn0 {
    #[skip]
    pub _reserved: B8,
}

derive!(Default for VcoCalibrIn0 = bits: 0x40);
derive!(radio::Register for VcoCalibrIn0 = address: 0x6B);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf1 {
    #[skip]
    pub _reserved: B4,
    /// - `1`: disable both dividers of digital clock (and reference clock for the SMPS) and IF-ADC clock.
    pub pd_clkdiv: B1,
    #[skip]
    pub _reserved: B3,
}

derive!(Default for XoRcoConf1 = bits: 0x45);
derive!(radio::Register for XoRcoConf1 = address: 0x6C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf0 {
    /// Enable the automatic RCO calibration.
    pub rco_calibration: bool,
    /// - `1`: the 34.7 kHz signal must be supplied from any GPIO.
    pub ext_rco_osc: B1,
    #[skip]
    pub _reserved: B1,
    /// Enable the the reference clock divider.
    pub refdiv: bool,
    /// Set the driver gm of the XO at start up.
    pub gm_conf: B3,
    /// - `0`: reference signal from XO circuit
    /// - `1`: reference signal from XIN pin.
    pub ext_ref: B1,
}

derive!(Default for XoRcoConf0 = bits: 0x30);
derive!(radio::Register for XoRcoConf0 = address: 0x6D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf3 {
    /// `[4:1]` MSB part of RFB word value for RCO.
    pub rfb_in: B4,
    /// RWT word value for the RCO.
    pub rwt_in: B4,
}

derive!(Default for RcoCalibrConf3 = bits: 0x70);
derive!(radio::Register for RcoCalibrConf3 = address: 0x6E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf2 {
    #[skip]
    pub _reserved: B7,
    /// `[0]` LSB part of RFB word value for RCO.
    pub rfb_in: B1,
}

derive!(Default for RcoCalibrConf2 = bits: 0x4D);
derive!(radio::Register for RcoCalibrConf2 = address: 0x6F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf4 {
    #[skip]
    pub _reserved: B5,
    /// - `1`: disable the internal SMPS.
    pub ext_smps: B1,
    #[skip]
    pub _reserved: B2,
}

derive!(Default for PmConf4 = bits: 0x17);
derive!(radio::Register for PmConf4 = address: 0x75);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf3 {
    /// `[14:8]` Sets the divider ratio (MSB) of the rate multiplier
    /// (default: `Fsw = Fdig / 4`)
    pub krm: B7,
    /// - `0`: divider by 4 enabled (SMPS' switching frequency is `FSW = Fdig / 4`)
    /// - `1`: rate multiplier enabled (SMPS' switching frequency is `FSW = KRM * Fdig / (2^15)`).
    pub krm_en: B1,
}

derive!(Default for PmConf3 = bits: 0x20);
derive!(radio::Register for PmConf3 = address: 0x76);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf2 {
    /// `[7:0]` Sets the divider ratio (LSB) of the rate multiplier
    /// (default: `Fsw = Fdig / 4`)
    pub krm: B8,
}

derive!(Default for PmConf2 = bits: 0x00);
derive!(radio::Register for PmConf2 = address: 0x77);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf1 {
    #[skip]
    pub _reserved: B2,
    /// Set to 0 (default value)
    pub bypass_ldo: B1,
    /// - `0`: SMPS output level depends upon the value written in the `PM_CONFIG0` register (`SET_SMPS_LEVEL` field) both in RX and TX state.
    /// - `1`: SMPS output level depends upon the value in `PM_CONFIG` register just in TX state, while in RX state it is fixed to 1.4 V
    pub smps_lvl_mode: B1,
    /// Set the BLD threshold:
    /// - `00b`: 2.7 V,
    /// - `01b`: 2.5 V,
    /// - `10b`: 2.3 V,
    /// - `11b`: 2.1 V.
    pub set_bld_th: B2,
    /// Enable battery level detector circuit.
    pub battery_lvl_en: bool,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PmConf1 = bits: 0x39);
derive!(radio::Register for PmConf1 = address: 0x78);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf0 {
    /// - `0`: SLEEP without FIFO retention (SLEEP A),
    /// - `1`: SLEEP with FIFO retention (SLEEP B).
    pub sleep_mode_sel: B1,
    #[skip]
    pub _reserved: B3,
    /// SMPS output voltage:
    /// - `000b`: not used,
    /// - `001b`: 1.2 V,
    /// - `010b`: 1.3 V,
    /// - `011b`: 1.4 V,
    /// - `100b`: 1.5 V,
    /// - `101b`: 1.6 V,
    /// - `110b`: 1.7 V,
    /// - `111b`: 1.8 V
    pub set_smps_lvl: B3,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for PmConf0 = bits: 0x42);
derive!(radio::Register for PmConf0 = address: 0x79);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState1 {
    /// - `1`: RCO calibrator error.
    pub error_lock: B1,
    /// - `1`: RX FIFO is empty.
    pub rx_fifo_empty: B1,
    /// - `1`: TX FIFO is full.
    pub tx_fifo_full: B1,
    /// Currently selected antenna.
    pub ant_sel: B1,
    /// RCO calibration successfully terminated.
    pub rco_cal_ok: B1,
    #[skip]
    pub _reserved: B3,
}

derive!(Default for McState1 = bits: 0x52);
derive!(radio::Register for McState1 = address: 0x8D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState0 {
    /// - `1`: XO is operating.
    pub xo_on: B1,
    /// Current state.
    pub state: B7,
}

derive!(Default for McState0 = bits: 0x07);
derive!(radio::Register for McState0 = address: 0x8E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxFifoStatus {
    /// Number of elements in TX FIFO.
    pub nelem_txfifo: B8,
}

derive!(Default for TxFifoStatus = bits: 0x00);
derive!(radio::Register for TxFifoStatus = address: 0x8F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxFifoStatus {
    /// Number of elements in RX FIFO.
    pub nelem_rxfifo: B8,
}

derive!(Default for RxFifoStatus = bits: 0x00);
derive!(radio::Register for RxFifoStatus = address: 0x90);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut4 {
    /// `[4:1]` RFB word (MSB) from internal RCO calibrator.
    pub rfb_out: B4,
    /// RWT word from internal RCO calibrator.
    pub rwt_out: B4,
}

derive!(Default for RcoCalibrOut4 = bits: 0x70);
derive!(radio::Register for RcoCalibrOut4 = address: 0x94);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut3 {
    #[skip]
    pub _reserved: B7,
    /// `[0]` RF word (LSB) from internal RCO calibrator.
    pub rfb_out: B1,
}

derive!(Default for RcoCalibrOut3 = bits: 0x00);
derive!(radio::Register for RcoCalibrOut3 = address: 0x95);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrOut1 {
    /// VCO magnitude calibration output word
    /// (binary coding internally converted from thermometric coding).
    pub vco_cal_amp_out: B4,
    #[skip]
    pub _reserved: B4,
}

derive!(Default for VcoCalibrOut1 = bits: 0x00);
derive!(radio::Register for VcoCalibrOut1 = address: 0x99);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrout0 {
    /// VCO Cbank frequency calibration output word
    /// (binary coding internally converted from thermometric coding).
    pub vco_cal_freq_out: B7,
    #[skip]
    pub _reserved: B1,
}

derive!(Default for VcoCalibrout0 = bits: 0x00);
derive!(radio::Register for VcoCalibrout0 = address: 0x9A);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxPcktInfo {
    /// Number of re-transmissions done for the last TX packet.
    pub n_retx: B4,
    /// Current TX packet sequence number.
    pub tx_seq_num: B2,
    #[skip]
    pub _reserved: B2,
}

derive!(Default for TxPcktInfo = bits: 0x00);
derive!(radio::Register for TxPcktInfo = address: 0x9C);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktInfo {
    /// Sequence number of the received packet.
    pub rx_seq_num: B2,
    /// NACK field of the received packet.
    pub nack_rx: B1,
    #[skip]
    pub _reserved: B5,
}

derive!(Default for RxPcktInfo = bits: 0x00);
derive!(radio::Register for RxPcktInfo = address: 0x9D);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AfcCorr {
    /// AFC corrected value.
    pub afc_corr: B8,
}

derive!(Default for AfcCorr = bits: 0x00);
derive!(radio::Register for AfcCorr = address: 0x9E);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif2 {
    /// PQI value of the received packet.
    pub pqi: B8,
}

derive!(Default for LinkQualif2 = bits: 0x00);
derive!(radio::Register for LinkQualif2 = address: 0x9F);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif1 {
    /// SQI value of the received packet.
    pub sqi: B7,
    /// Carrier sense indication.
    pub cs: B1,
}

derive!(Default for LinkQualif1 = bits: 0x00);
derive!(radio::Register for LinkQualif1 = address: 0xA0);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevel {
    /// RSSI level captured at the end of the SYNC word detection of the received packet.
    pub rssi_level: B8,
}

derive!(Default for RssiLevel = bits: 0x00);
derive!(radio::Register for RssiLevel = address: 0xA2);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen1 {
    /// `[14:8]` MSB value of the length of the packet received.
    pub rx_pckt_len: B8,
}

derive!(Default for RxPcktLen1 = bits: 0x00);
derive!(radio::Register for RxPcktLen1 = address: 0xA4);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen0 {
    /// `[7:0]` LSB value of the length of the packet received.
    pub rx_pckt_len: B8,
}

derive!(Default for RxPcktLen0 = bits: 0x00);
derive!(radio::Register for RxPcktLen0 = address: 0xA5);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField3 {
    /// CRC field 3 of the received packet.
    pub crc_field3: B8,
}

derive!(Default for CrcField3 = bits: 0x00);
derive!(radio::Register for CrcField3 = address: 0xA6);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField2 {
    /// CRC field 2 of the received packet.
    pub crc_field2: B8,
}

derive!(Default for CrcField2 = bits: 0x00);
derive!(radio::Register for CrcField2 = address: 0xA7);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField1 {
    /// CRC field 1 of the received packet.
    pub crc_field1: B8,
}

derive!(Default for CrcField1 = bits: 0x00);
derive!(radio::Register for CrcField1 = address: 0xA8);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField0 {
    /// CRC field 0 of the received packet.
    pub crc_field0: B8,
}

derive!(Default for CrcField0 = bits: 0x00);
derive!(radio::Register for CrcField0 = address: 0xA9);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField1 {
    /// Source address field of the received packet.
    pub rx_addre_field1: B8,
}

derive!(Default for RxAddreField1 = bits: 0x00);
derive!(radio::Register for RxAddreField1 = address: 0xAA);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField0 {
    /// Destination address field of the received packet.
    pub rx_addre_field0: B8,
}

derive!(Default for RxAddreField0 = bits: 0x00);
derive!(radio::Register for RxAddreField0 = address: 0xAB);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevelRun {
    /// RSSI level of the received packet, which supports continuous fast SPI reading.
    pub rssi_level_run: B8,
}

derive!(Default for RssiLevelRun = bits: 0x00);
derive!(radio::Register for RssiLevelRun = address: 0xEF);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo1 {
    /// S2-LP part number
    pub partnum: B8,
}

derive!(Default for DeviceInfo1 = bits: 0x03);
derive!(radio::Register for DeviceInfo1 = address: 0xF0);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo0 {
    /// S2-LP version number
    pub version: B8,
}

derive!(Default for DeviceInfo0 = bits: 0xC1);
derive!(radio::Register for DeviceInfo0 = address: 0xF1);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus3 {
    /// `[31:24]` Interrupt status register 3
    pub int_level: B8,
}

derive!(Default for IrqStatus3 = bits: 0x00);
derive!(radio::Register for IrqStatus3 = address: 0xFA);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus2 {
    /// `[23:16]` Interrupt status register 2
    pub int_level: B8,
}

derive!(Default for IrqStatus2 = bits: 0x09);
derive!(radio::Register for IrqStatus2 = address: 0xFB);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus1 {
    /// `[15:8]` Interrupt status register 1
    pub int_level: B8,
}

derive!(Default for IrqStatus1 = bits: 0x05);
derive!(radio::Register for IrqStatus1 = address: 0xFC);

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus0 {
    /// `[7:0]` Interrupt status register 0
    pub int_level: B8,
}

derive!(Default for IrqStatus0 = bits: 0x00);
derive!(radio::Register for IrqStatus0 = address: 0xFD);
