//! **ADS868x(W) register map**: concrete register tags, selector constants, and typed builders.
//!
//! This module mirrors the device’s register table and provides **typed constructors**
//! for each writable halfword. It builds on the generic SDI primitives in
//! [`crate::sdi`] (which supply the `Command` packing, typed payloads, and selectors).
//!
//! # Usage
//! ```no_run
//! use ads868x::registers::*;
//!
//! // Configure input timing and output interface
//! let cmd1 = SDI_CTL_LO.write(SdiCtlLo::new(SdiMode::Cpol0Cpha0));
//! let cmd2 = SDO_CTL_LO.write(SdoCtlLo::new(SdoMode::Spi, false, Sdo1Config::AlarmOrGpo, false));
//!
//! // Select range and enable parity in data words
//! let cmd3 = RANGE_SEL_LO.write(RangeSelLo::new(RangeSel::Uni1p25xVref, false));
//! let cmd4 = DATAOUT_CTL_LO.write(DataoutCtlLo::new(false,false,false,true,true,DataVal::Conversions));
//!
//! // Optional alarm thresholds
//! let cmd5 = ALARM_H_TH_LO.write(AlarmHighThLo::new(0x8000));
//! let cmd6 = ALARM_H_TH_HI.write(AlarmHighThHi::new(8));
//! let cmd7 = ALARM_L_TH_LO.write(AlarmLowThLo::new(0x0100));
//!
//! // Optional device address (high half of DEVICE_ID)
//! let cmd8 = DEVICE_ID_HI.write(DeviceIdHi::new(0x3));
//! // spi.write(&cmd1.to_be_bytes()).await?; // etc.
//! ```

use crate::sdi::{
    HasHi, HiHWord, LoHWord, RegHi, RegLo, RegisterSpec, WritableHi, WritableLo, hi_hword, lo_hword,
};

#[cfg(feature = "defmt")]
use defmt::Format;

/* -------------------------------------------------------------------------------------------------
Register tags & writability
---------------------------------------------------------------------------------------------- */

/// `DEVICE_ID` (0x00..0x03) — identification / device address nibble in **high** half.
///
/// - Low half (BASE 0x00) is reserved (read-only/undefined).
/// - High half (BASE+2 0x02) exposes the 4-bit device address nibble at bits [3:0].
pub enum DeviceIdReg {}
impl RegisterSpec for DeviceIdReg {
    const BASE: u8 = 0x00;
}
impl HasHi for DeviceIdReg {}
impl WritableHi for DeviceIdReg {} // only high half is writable (via typed builder below)

/// `RST_PWRCTL` (0x04..0x07) — reset / power control. High half reserved; low half writable.
pub enum RstPwrctlReg {}
impl RegisterSpec for RstPwrctlReg {
    const BASE: u8 = 0x04;
}
impl WritableLo for RstPwrctlReg {}

/// `SDI_CTL` (0x08..0x0B) — input SPI CPOL/CPHA selection. High half reserved; low half writable.
pub enum SdiCtlReg {}
impl RegisterSpec for SdiCtlReg {
    const BASE: u8 = 0x08;
}
impl WritableLo for SdiCtlReg {}

/// `SDO_CTL` (0x0C..0x0F) — output protocol / SDO-1 / SRC clock. All active bits are in the low half.
pub enum SdoCtlReg {}
impl RegisterSpec for SdoCtlReg {
    const BASE: u8 = 0x0C;
}
impl WritableLo for SdoCtlReg {}

/// `DATAOUT_CTL` (0x10..0x13) — include flags, parity, test patterns (low half).
pub enum DataoutCtlReg {}
impl RegisterSpec for DataoutCtlReg {
    const BASE: u8 = 0x10;
}
impl WritableLo for DataoutCtlReg {}

/// `RANGE_SEL` (0x14..0x17) — input range & internal reference enable (low half).
pub enum RangeSelReg {}
impl RegisterSpec for RangeSelReg {
    const BASE: u8 = 0x14;
}
impl WritableLo for RangeSelReg {}

/// `ALARM` (0x20..0x23) — alarm flags (read-only; low half).
pub enum AlarmReg {}
impl RegisterSpec for AlarmReg {
    const BASE: u8 = 0x20;
}

/// `ALARM_H_TH` (0x24..0x27) — high threshold (low) + hysteresis [7:2] (high). Both halves writable.
pub enum AlarmHighThReg {}
impl RegisterSpec for AlarmHighThReg {
    const BASE: u8 = 0x24;
}
impl HasHi for AlarmHighThReg {}
impl WritableLo for AlarmHighThReg {}
impl WritableHi for AlarmHighThReg {}

/// `ALARM_L_TH` (0x28..0x2B) — low threshold (high half reserved).
pub enum AlarmLowThReg {}
impl RegisterSpec for AlarmLowThReg {
    const BASE: u8 = 0x28;
}
impl WritableLo for AlarmLowThReg {}

/* -------------------------------------------------------------------------------------------------
Selector constants (datasheet-style names)
---------------------------------------------------------------------------------------------- */

/// Low half of `DEVICE_ID` (reserved / read-only).
pub const DEVICE_ID_LO: RegLo<DeviceIdReg> = RegLo::new();
/// High half of `DEVICE_ID` (contains device address nibble).
pub const DEVICE_ID_HI: RegHi<DeviceIdReg> = RegHi::new();

/// Low half of `RST_PWRCTL` (writable).
pub const RST_PWRCTL_LO: RegLo<RstPwrctlReg> = RegLo::new();

/// Low half of `SDI_CTL` (writable).
pub const SDI_CTL_LO: RegLo<SdiCtlReg> = RegLo::new();

/// Low half of `SDO_CTL` (writable).
pub const SDO_CTL_LO: RegLo<SdoCtlReg> = RegLo::new();

/// Low half of `DATAOUT_CTL` (writable).
pub const DATAOUT_CTL_LO: RegLo<DataoutCtlReg> = RegLo::new();

/// Low half of `RANGE_SEL` (writable).
pub const RANGE_SEL_LO: RegLo<RangeSelReg> = RegLo::new();

/// Low half of `ALARM` (read-only flags).
pub const ALARM_LO: RegLo<AlarmReg> = RegLo::new();

/// Low half of `ALARM_H_TH` (threshold code).
pub const ALARM_H_TH_LO: RegLo<AlarmHighThReg> = RegLo::new();
/// High half of `ALARM_H_TH` (hysteresis [7:2]).
pub const ALARM_H_TH_HI: RegHi<AlarmHighThReg> = RegHi::new();

/// Low half of `ALARM_L_TH` (threshold code).
pub const ALARM_L_TH_LO: RegLo<AlarmLowThReg> = RegLo::new();

/* -------------------------------------------------------------------------------------------------
Field enums & typed builders
---------------------------------------------------------------------------------------------- */

/// SDI input timing: CPOL/CPHA mode for the command interface.
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum SdiMode {
    /// Idle low, capture on rising edge.
    Cpol0Cpha0 = 0b00,
    /// Idle low, capture on falling edge.
    Cpol0Cpha1 = 0b01,
    /// Idle high, capture on rising edge.
    Cpol1Cpha0 = 0b10,
    /// Idle high, capture on falling edge.
    Cpol1Cpha1 = 0b11,
}

/// Builder for the low half of `SDI_CTL`.
pub struct SdiCtlLo;
impl SdiCtlLo {
    /// Construct a typed payload for `SDI_CTL` low half (bits `[1:0]`).
    #[inline]
    pub const fn new(mode: SdiMode) -> LoHWord<SdiCtlReg> {
        lo_hword::<SdiCtlReg>(mode as u16)
    }
}

/// Output interface mode.
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum SdoMode {
    /// Standard SPI: SDO follows external SCLK.
    Spi = 0b00,
    /// Source-synchronous readout (RVS provides read clocking).
    Src = 0b11,
}

/// Function of the ALARM/SDO-1/GPO pin.
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Sdo1Config {
    /// ALARM/GPO function (default).
    AlarmOrGpo = 0b00,
    /// Repurpose as **SDO-1** (dual-SDO).
    DualSdo = 0b11,
}

/// Builder for the low half of `SDO_CTL`.
pub struct SdoCtlLo;
impl SdoCtlLo {
    /// Construct a typed payload for `SDO_CTL` low half:
    ///
    /// - `mode` → `SDO_MODE[1:0]`
    /// - `ssync_clk_internal` → `SSYNC_CLK` (bit 6; `true` = internal in SRC modes)
    /// - `sdo1_cfg` → `SDO1_CONFIG[1:0]` (bits 9:8)
    /// - `gpo_high` → `GPO_VAL` (bit 12)
    #[inline]
    pub const fn new(
        mode: SdoMode,
        ssync_clk_internal: bool,
        sdo1_cfg: Sdo1Config,
        gpo_high: bool,
    ) -> LoHWord<SdoCtlReg> {
        let mut v: u16 = (mode as u16) & 0b11; // [1:0]
        if ssync_clk_internal {
            v |= 1 << 6;
        } // bit 6
        v |= (sdo1_cfg as u16) << 8; // [9:8]
        if gpo_high {
            v |= 1 << 12;
        } // bit 12
        lo_hword::<SdoCtlReg>(v)
    }
}

/// Select test output vs. normal conversions.
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DataVal {
    /// Normal conversion data.
    Conversions = 0b000,
    /// Test pattern: all zeros.
    AllZeros = 0b100,
    /// Test pattern: all ones.
    AllOnes = 0b101,
    /// Test pattern: alternating 01…
    Alt01 = 0b110,
    /// Test pattern: 00 11 00 11…
    Alt0011 = 0b111,
}

/// Builder for the low half of `DATAOUT_CTL`.
pub struct DataoutCtlLo;
impl DataoutCtlLo {
    /// Construct a typed payload for `DATAOUT_CTL` low half.
    ///
    /// Flags map to bits:
    /// - `DEVICE_ADDR_INCL` (14),
    /// - `VDD_ACTIVE_ALARM_INCL` (13),
    /// - `IN_ACTIVE_ALARM_INCL` (12),
    /// - `RANGE_INCL` (8),
    /// - `PAR_EN` (3).
    /// `data_val` selects test output pattern in `[2:0]`.
    #[inline]
    pub const fn new(
        device_addr_incl: bool,
        vdd_active_alarm_incl: bool,
        in_active_alarm_incl: bool,
        range_incl: bool,
        parity_enable: bool,
        data_val: DataVal,
    ) -> LoHWord<DataoutCtlReg> {
        let mut v: u16 = 0;
        if device_addr_incl {
            v |= 1 << 14;
        }
        if vdd_active_alarm_incl {
            v |= 1 << 13;
        }
        if in_active_alarm_incl {
            v |= 1 << 12;
        }
        if range_incl {
            v |= 1 << 8;
        }
        if parity_enable {
            v |= 1 << 3;
        }
        v |= (data_val as u16) & 0b111; // [2:0]
        lo_hword::<DataoutCtlReg>(v)
    }
}

/// Input range code (and internal reference enable).
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum RangeSel {
    Bipo3xVref = 0b0000,
    Bipo2p5xVref = 0b0001,
    Bipo1p5xVref = 0b0010,
    Bipo1p25xVref = 0b0011,
    Bipo0p625xVref = 0b0100,
    Uni3xVref = 0b1000,
    Uni2p5xVref = 0b1001,
    Uni1p5xVref = 0b1010,
    Uni1p25xVref = 0b1011,
}

/// Builder for the low half of `RANGE_SEL`.
pub struct RangeSelLo;
impl RangeSelLo {
    /// Construct a typed payload for `RANGE_SEL` low half.
    ///
    /// - `range` → `RANGE_SEL[3:0]`
    /// - `disable_intref` → `INTREF_DIS` (bit 6; `false` leaves internal 4.096 V ref enabled)
    #[inline]
    pub const fn new(range: RangeSel, disable_intref: bool) -> LoHWord<RangeSelReg> {
        let mut v: u16 = (range as u16) & 0x0F;
        if disable_intref {
            v |= 1 << 6;
        }
        lo_hword::<RangeSelReg>(v)
    }
}

/// Builder for the low half of `RST_PWRCTL`.
///
/// Bits:
/// - `[15:8]` = 0x69 (write-key),
/// - `[5]` `VDD_AL_DIS`, `[4]` `IN_AL_DIS`,
/// - `[2]` `RSTN_APP`, `[1]` `NAP_EN`, `[0]` `PWRDN`.
pub struct RstPwrctlLo;
impl RstPwrctlLo {
    /// Construct a typed payload for `RST_PWRCTL` low half (includes the 0x69 write-key).
    #[inline]
    pub const fn new(
        vdd_alarm_disable: bool,
        in_alarm_disable: bool,
        rstn_app: bool,
        nap_en: bool,
        pwrdn: bool,
    ) -> LoHWord<RstPwrctlReg> {
        const WKEY: u16 = 0x69;
        let mut v: u16 = WKEY << 8;
        if vdd_alarm_disable {
            v |= 1 << 5;
        }
        if in_alarm_disable {
            v |= 1 << 4;
        }
        if rstn_app {
            v |= 1 << 2;
        }
        if nap_en {
            v |= 1 << 1;
        }
        if pwrdn {
            v |= 1 << 0;
        }
        lo_hword::<RstPwrctlReg>(v)
    }
}

/// Builder for the low/high halves of `ALARM_H_TH`.
pub struct AlarmHighThLo;
impl AlarmHighThLo {
    /// Low half = 16-bit high-threshold code.
    #[inline]
    pub const fn new(code: u16) -> LoHWord<AlarmHighThReg> {
        lo_hword::<AlarmHighThReg>(code)
    }
}
/// High half uses bits `[7:2]` for a 6-bit hysteresis value; `[1:0]` must be 0.
pub struct AlarmHighThHi;
impl AlarmHighThHi {
    /// `hyst6` is placed at `[7:2]` (LS two bits zeroed).
    #[inline]
    pub const fn new(hyst6: u8) -> HiHWord<AlarmHighThReg> {
        hi_hword::<AlarmHighThReg>(((hyst6 & 0x3F) as u16) << 2)
    }
}

/// Builder for the low half of `ALARM_L_TH` (16-bit code; high half reserved).
pub struct AlarmLowThLo;
impl AlarmLowThLo {
    #[inline]
    pub const fn new(code: u16) -> LoHWord<AlarmLowThReg> {
        lo_hword::<AlarmLowThReg>(code)
    }
}

/// Builder for the high half of `DEVICE_ID` (device address nibble in bits `[3:0]`).
///
/// All other bits must be written as 0.
pub struct DeviceIdHi;
impl DeviceIdHi {
    /// Build a high-halfword write for `DEVICE_ID`.
    ///
    /// - `addr4`: 4-bit device address (0–15).
    #[inline]
    pub const fn new(addr4: u8) -> HiHWord<DeviceIdReg> {
        hi_hword::<DeviceIdReg>((addr4 & 0x0F) as u16)
    }
}
