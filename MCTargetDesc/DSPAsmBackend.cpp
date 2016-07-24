//===-- DSPASMBackend.cpp - DSP Asm Backend ----------------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the DSPAsmBackend and DSPELFObjectWriter classes.
//
//===----------------------------------------------------------------------===//
//
#include "MCTargetDesc/DSPFixupKinds.h"
#include "MCTargetDesc/DSPAsmBackend.h"
#include "MCTargetDesc/DSPMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <iostream>
using namespace llvm;

// Prepare value for the target space for it
static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
	MCContext *Ctx = nullptr) {
	unsigned Kind = Fixup.getKind();
	std::cout << "value" << Value << std::endl;

	// Add/subtract and shift
	switch (Kind) {
	default:
		return 0;
	case FK_GPRel_4:
	case FK_Data_4:
	case DSP::fixup_DSP_LO16:
		break;
	case DSP::fixup_DSP_HI16:
	case DSP::fixup_DSP_GOT_Local:
		// Get the higher 16-bits. Also add 1 if bit 15 is 1.
		Value = ((Value + 0x8000) >> 16) & 0xffff;
		break;
	case DSP::fixup_DSP_PC16:
		// So far we are only using this type for branches.
		// For branches we start 1 instruction after the branch
		// so the displacement will be one instruction size less.
		Value -= 4;
		// The displacement is then divided by 4 to give us an 18 bit
		// address range. Forcing a signed division because Value can be negative.
		Value = (int64_t)Value / 4;
		// We now check if Value can be encoded as a 16-bit signed immediate.
		if (!isIntN(16, Value) && Ctx)
			Ctx->FatalError(Fixup.getLoc(), "out of range pc 16");
		break;
	case DSP::fixup_DSP_PC24:
		Value -= 4;
		Value = (int64_t)Value / 4;
		if (!isIntN(24,Value)&&Ctx)
			Ctx->FatalError(Fixup.getLoc(), "out of range pc 24");
	case DSP::fixup_DSP_PC21:
		Value -= 4;
		Value = (int64_t)Value / 4;
		if (!isIntN(21, Value) && Ctx)
			Ctx->FatalError(Fixup.getLoc(), "out of range pc 21");
	case DSP::fixup_DSP_PC26:
		//for jmp jnc
		Value -= 4;
		Value = (int64_t)Value / 4;
		if (isIntN(26, Value) && Ctx)
			Ctx->FatalError(Fixup.getLoc(), "out of range pc 26");
	}
	
	return Value;
}

MCObjectWriter *DSPAsmBackend::createObjectWriter(raw_ostream &OS) const {
	return createDSPELFObjectWriter(OS,MCELFObjectTargetWriter::getOSABI(OSType), IsLittle);
}

/// ApplyFixup - Apply the \arg Value for given \arg Fixup into the provided
/// data fragment, at the offset specified by the fixup and following the
/// fixup kind as appropriate.
void DSPAsmBackend::applyFixup(const MCFixup &Fixup, char *Data,
	unsigned DataSize, uint64_t Value,
	bool IsPCRel) const {
	MCFixupKind Kind = Fixup.getKind();
	Value = adjustFixupValue(Fixup, Value);
	if (!Value)
		return; 
	
	// Doesn¡¯t change encoding.
	// Where do we start in the object
	unsigned Offset = Fixup.getOffset();
	

	// Number of bytes we need to fixup
	unsigned NumBytes = (getFixupKindInfo(Kind).TargetSize + 7) / 8;
	// Used to point to big endian bytes
	unsigned FullSize;
	switch ((unsigned)Kind) {
	default:
		FullSize = 4;
		break;
	}
	// Grab current value, if any, from bits.
	uint64_t CurVal = 0;
	for (unsigned i = 0; i != NumBytes; ++i) {
		unsigned Idx = IsLittle ? i : (FullSize - 1 - i);
		CurVal |= (uint64_t)((uint8_t)Data[Offset + Idx]) << (i * 8);
	}


	uint64_t Mask = ((uint64_t)(-1) >> (64 - getFixupKindInfo(Kind).TargetSize));
	Value = Value << 5;
	std::cout << CurVal << std::endl;
	std::cout << Value << std::endl;
	std::cout << "mask" << Mask << std::endl;
	CurVal |= Value & Mask;
	std::cout << CurVal << std::endl;
	// Write out the fixed up bytes back to the code/data bits.
	for (unsigned i = 0; i != NumBytes; ++i) {
		unsigned Idx = IsLittle ? i : (FullSize - 1 - i);
		Data[Offset + Idx] = (uint8_t)((CurVal >> (i * 8)) & 0xff);
	}
}
const MCFixupKindInfo &DSPAsmBackend::
getFixupKindInfo(MCFixupKind Kind) const {
	const static MCFixupKindInfo Infos[DSP::NumTargetFixupKinds] = {
		// This table *must* be in same the order of fixup_* kinds in
		// DSPFixupKinds.h.
		//
		// name offset bits flags
		{ "fixup_DSP_32", 0, 32, 0 },
		{ "fixup_DSP_HI16", 0, 16, 0 },
		{ "fixup_DSP_LO16", 0, 16, 0 },
		{ "fixup_DSP_GPREL16", 0, 16, 0 },
		{ "fixup_DSP_GOT_Global", 0, 16, 0 },
		{ "fixup_DSP_GOT_Local", 0, 16, 0 },
		{ "fixup_DSP_GOT_HI16", 0, 16, 0 },
		{ "fixup_DSP_GOT_LO16", 0, 16, 0 }, 
		{ "fixup_DSP_PC16", 0, 16, MCFixupKindInfo::FKF_IsPCRel },
		{ "fixup_DSP_PC21", 0, 21, MCFixupKindInfo::FKF_IsPCRel },
		{ "fixup_DSP_CALL",0,16, 0 },
		{ "fixup_DSP_PC21_S2", 0, 21, MCFixupKindInfo::FKF_IsPCRel },
		{ "fixup_DSP_PC26_S2", 0, 26, MCFixupKindInfo::FKF_IsPCRel },
		{ "fixup_DSP_PC24", 0, 24, MCFixupKindInfo::FKF_IsPCRel },
		{ "fixup_DSP_PC26", 0, 26, MCFixupKindInfo::FKF_IsPCRel },
	};
	if (Kind < FirstTargetFixupKind)
		return MCAsmBackend::getFixupKindInfo(Kind);
	assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
		"Invalid kind!");
	return Infos[Kind - FirstTargetFixupKind];
}

/// WriteNopData - Write an (optimal) nop sequence of Count bytes
/// to the given output. If the target cannot generate such a sequence,
/// it should return an error.
///
/// \return - True on success.
bool DSPAsmBackend::writeNopData(uint64_t Count, MCObjectWriter *OW) const {
	return true;
}
// MCAsmBackend
MCAsmBackend *llvm::createDSPAsmBackendEL32(const Target &T,
	const MCRegisterInfo &MRI,
	StringRef TT,
	StringRef CPU) {
	return new DSPAsmBackend(T, Triple(TT).getOS(),
		/*IsLittle*/true);
}
MCAsmBackend *llvm::createDSPAsmBackendEB32(const Target &T,
	const MCRegisterInfo &MRI,
	StringRef TT,
	StringRef CPU) {
	return new DSPAsmBackend(T, Triple(TT).getOS(),
		/*IsLittle*/false);
}