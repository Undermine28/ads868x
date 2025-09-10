//! Core **SDI (Serial Data Interface)** primitives used to talk to the ADS868x(W) ADCs.
//!
//! This module is **chip-agnostic**. It provides:
//! - The 32-bit [`Command`] type and helpers to pack/read/write *halfwords*,
//! - Strongly typed, zero-cost **register selectors**: [`RegLo`] and [`RegHi`],
//! - Strongly typed **halfword payloads**: [`LoHWord`] / [`HiHWord`], tied to a specific register,
//! - Marker traits for register **capabilities** (`WritableLo`, `WritableHi`, `HasHi`).
//!
//! The *datasheet-specific* register tags, constants, and field builders live in
//! `crate::registers`, which is built on top of this module.
//!
//! # Design
//! - **Zero runtime cost**: all type tagging is compile-time only (`PhantomData`).
//! - **Invalid states unrepresentable**: you cannot write to a read-only half;
//!   you cannot pass a payload intended for one register/half to a different one.
//!
//! # Quick example
//! ```no_run
//! use ads868x::{sdi::*, registers::*};
//!
//! // Build a typed halfword payload for the RANGE_SEL low half...
//! let word: LoHWord<RangeSelReg> = RangeSelLo::new(RangeSel::Uni1p25xVref, false);
//!
//! // ...and turn it into a WRITE command addressed to RANGE_SEL (low).
//! let cmd: Command = RANGE_SEL_LO.write(word);
//!
//! // Transmit over SPI:
//! // spi.write(&cmd.to_be_bytes()).await?;
//! ```

#![allow(dead_code)]

use core::marker::PhantomData;

#[cfg(feature = "defmt")]
use defmt::Format;

/* -------------------------------------------------------------------------------------------------
32-bit command encoding
---------------------------------------------------------------------------------------------- */

/// A 32-bit SDI command word to send over SPI.
///
/// Use [`Command::to_be_bytes()`] to obtain the 4 transmit bytes (big-endian).
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Command(pub u32);

impl Command {
    /// Convert to big-endian bytes for SPI.
    #[inline]
    pub fn to_be_bytes(self) -> [u8; 4] {
        self.0.to_be_bytes()
    }

    /// `NOP` command (all zeros).
    #[inline]
    pub const fn nop() -> Self {
        Command(0)
    }

    /* ---- Low-half operations (BASE) ---- */

    /// Build a command to **read** the low halfword of register `R` at `BASE`.
    #[inline]
    pub fn read_lo<R: RegisterSpec>(_: RegLo<R>) -> Self {
        pack_cmd(Opcode::ReadHword, WriteMask::Both, R::BASE, 0)
    }

    /// Build a command to **write** the low halfword of register `R` at `BASE`.
    ///
    /// Accepts only a payload created for the **low half of `R`** (see [`LoHWord`]).
    #[inline]
    pub fn write_lo<R: WritableLo>(_: RegLo<R>, data: LoHWord<R>) -> Self {
        pack_cmd(Opcode::Write, WriteMask::Both, R::BASE, data.raw())
    }

    /// Build a command to **SET** bits (bitwise OR) in the low halfword of `R`.
    #[inline]
    pub fn set_lo<R: WritableLo>(_: RegLo<R>, mask_1s: LoHWord<R>) -> Self {
        pack_cmd(Opcode::SetHword, WriteMask::Both, R::BASE, mask_1s.raw())
    }

    /// Build a command to **CLEAR** bits (AND with `!mask`) in the low halfword of `R`.
    #[inline]
    pub fn clear_lo<R: WritableLo>(_: RegLo<R>, mask_1s: LoHWord<R>) -> Self {
        pack_cmd(Opcode::ClearHword, WriteMask::Both, R::BASE, mask_1s.raw())
    }

    /* ---- High-half operations (BASE+2) ---- */

    /// Build a command to **read** the high halfword of register `R` at `BASE+2`.
    #[inline]
    pub fn read_hi<R: RegisterSpec + HasHi>(_: RegHi<R>) -> Self {
        pack_cmd(
            Opcode::ReadHword,
            WriteMask::Both,
            R::BASE.wrapping_add(2),
            0,
        )
    }

    /// Build a command to **write** the high halfword of register `R` at `BASE+2`.
    ///
    /// Accepts only a payload created for the **high half of `R`** (see [`HiHWord`]).
    #[inline]
    pub fn write_hi<R: WritableHi>(_: RegHi<R>, data: HiHWord<R>) -> Self {
        pack_cmd(
            Opcode::Write,
            WriteMask::Both,
            R::BASE.wrapping_add(2),
            data.raw(),
        )
    }

    /// Build a command to **SET** bits in the high halfword of `R`.
    #[inline]
    pub fn set_hi<R: WritableHi>(_: RegHi<R>, mask_1s: HiHWord<R>) -> Self {
        pack_cmd(
            Opcode::SetHword,
            WriteMask::Both,
            R::BASE.wrapping_add(2),
            mask_1s.raw(),
        )
    }

    /// Build a command to **CLEAR** bits in the high halfword of `R`.
    #[inline]
    pub fn clear_hi<R: WritableHi>(_: RegHi<R>, mask_1s: HiHWord<R>) -> Self {
        pack_cmd(
            Opcode::ClearHword,
            WriteMask::Both,
            R::BASE.wrapping_add(2),
            mask_1s.raw(),
        )
    }

    /// Read a single byte at an absolute address.
    ///
    /// Prefer halfword helpers when possible; this is mostly for diagnostics.
    #[inline]
    pub fn read_byte(addr: u8) -> Self {
        pack_cmd(Opcode::ReadByte, WriteMask::Both, addr, 0)
    }
}

/// Convenience: `let bytes: [u8;4] = command.into();`
impl From<Command> for [u8; 4] {
    #[inline]
    fn from(c: Command) -> Self {
        c.to_be_bytes()
    }
}

/// SDI opcode field (5 bits).
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Opcode {
    /// No operation.
    Nop = 0b00000,
    /// Clear bits in the addressed halfword.
    ClearHword = 0b11000,
    /// Read the addressed halfword.
    ReadHword = 0b11001,
    /// Read a single byte at an absolute address.
    ReadByte = 0b01001,
    /// Write the addressed halfword (obeys [`WriteMask`]).
    Write = 0b11010,
    /// Set bits in the addressed halfword.
    SetHword = 0b11011,
}

/// Write-mask subfield (2 bits); only meaningful with [`Opcode::Write`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum WriteMask {
    /// Write both bytes of the halfword.
    Both = 0b00,
    /// Write the most significant byte only.
    MsB = 0b01,
    /// Write the least significant byte only.
    LsB = 0b10,
}

/// Pack a 32-bit SDI command.
///
/// Layout:
/// - bits **[31:27]**: opcode
/// - bits **[26:25]**: write-mask (only meaningful for [`Opcode::Write`])
/// - bits **[24:16]**: 9-bit address (we use 8 bits; MSB must be 0)
//  - bits **[15:0]** : data halfword
#[inline]
pub const fn pack_cmd(op: Opcode, mask: WriteMask, addr: u8, data: u16) -> Command {
    let opc = (op as u32) << 27;
    let msk = (mask as u32) << 25;
    let adr = (addr as u32) << 16;
    let dat = data as u32;
    Command(opc | msk | adr | dat)
}

/* -------------------------------------------------------------------------------------------------
Typed halfwords, markers, traits, and selectors
---------------------------------------------------------------------------------------------- */

/// Marker for **low** halfword payloads (uninhabited).
pub enum Lo {}

/// Marker for **high** halfword payloads (uninhabited).
pub enum Hi {}

/// A 16-bit halfword payload **tied to a specific register and half**.
///
/// The type parameter is a pair `(RegisterTag, Lo|Hi)`, carried via `PhantomData`.
/// There is **no public constructor**; obtain instances from the typed builders in
/// `crate::registers` (e.g., `RangeSelLo::new`, `AlarmHighThHi::new`).
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct RegWord<R>(u16, PhantomData<R>);

impl<R> RegWord<R> {
    /// Raw 16-bit value (for packing into an SDI command).
    #[inline]
    pub const fn raw(&self) -> u16 {
        self.0
    }
}

/// Readability aliases for typed payloads.
///
/// - [`LoHWord<R>`] = payload for the **low** halfword of register `R`.
/// - [`HiHWord<R>`] = payload for the **high** halfword of register `R`.
pub type LoHWord<R> = RegWord<(R, Lo)>;
pub type HiHWord<R> = RegWord<(R, Hi)>;

/// Crate-visible constructors for typed halfword payloads.
///
/// These keep `RegWord`'s fields private while allowing `registers.rs` to build
/// correctly typed payloads in `const` contexts.
#[inline]
pub(crate) const fn lo_hword<R>(val: u16) -> LoHWord<R> {
    RegWord(val, PhantomData)
}
#[inline]
pub(crate) const fn hi_hword<R>(val: u16) -> HiHWord<R> {
    RegWord(val, PhantomData)
}

/// Registers implement this to provide their byte `BASE` address.
pub trait RegisterSpec {
    /// Byte address of the 32-bit register’s **low** halfword.
    const BASE: u8;
}

/// Registers with a meaningful **high** halfword implement this marker.
pub trait HasHi {}

/// Registers whose **low** halfword is writable implement this marker.
pub trait WritableLo: RegisterSpec {}

/// Registers whose **high** halfword is writable implement this marker.
pub trait WritableHi: RegisterSpec + HasHi {}

/// Selector for the **low** halfword (`BASE`) of a register.
///
/// Ergonomics:
/// - `.read()` is always available,
/// - `.write()/.set()/.clear()` are available only when the register implements [`WritableLo`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct RegLo<R: RegisterSpec>(PhantomData<R>);

impl<R: RegisterSpec> RegLo<R> {
    /// Create a low-half selector for register `R`.
    pub const fn new() -> Self {
        Self(PhantomData)
    }

    /// Build a READ command for the low half of `R`.
    #[inline]
    pub fn read(self) -> Command {
        Command::read_lo(self)
    }
}
impl<R: WritableLo> RegLo<R> {
    /// Build a WRITE command for the low half of `R`.
    #[inline]
    pub fn write(self, data: LoHWord<R>) -> Command {
        Command::write_lo(self, data)
    }

    /// Build a SET-bits command for the low half of `R` (bitwise OR with `mask_1s`).
    #[inline]
    pub fn set(self, mask_1s: LoHWord<R>) -> Command {
        Command::set_lo(self, mask_1s)
    }

    /// Build a CLEAR-bits command for the low half of `R` (bitwise AND with `!mask_1s`).
    #[inline]
    pub fn clear(self, mask_1s: LoHWord<R>) -> Command {
        Command::clear_lo(self, mask_1s)
    }
}

/// Selector for the **high** halfword (`BASE+2`) of a register.
///
/// Ergonomics:
/// - `.read()` is available when the register implements [`HasHi`],
/// - `.write()/.set()/.clear()` are available only when it also implements [`WritableHi`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct RegHi<R: RegisterSpec + HasHi>(PhantomData<R>);

impl<R: RegisterSpec + HasHi> RegHi<R> {
    /// Create a high-half selector for register `R`.
    pub const fn new() -> Self {
        Self(PhantomData)
    }

    /// Build a READ command for the high half of `R`.
    #[inline]
    pub fn read(self) -> Command {
        Command::read_hi(self)
    }
}
impl<R: WritableHi> RegHi<R> {
    /// Build a WRITE command for the high half of `R`.
    #[inline]
    pub fn write(self, data: HiHWord<R>) -> Command {
        Command::write_hi(self, data)
    }

    /// Build a SET-bits command for the high half of `R`.
    #[inline]
    pub fn set(self, mask_1s: HiHWord<R>) -> Command {
        Command::set_hi(self, mask_1s)
    }

    /// Build a CLEAR-bits command for the high half of `R`.
    #[inline]
    pub fn clear(self, mask_1s: HiHWord<R>) -> Command {
        Command::clear_hi(self, mask_1s)
    }
}
