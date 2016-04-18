//===-- DSPMCCodeEmitter.cpp - Convert DSP Code to Machine Code ---------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the DSPMCCodeEmitter class.
////===----------------------------------------------------------------------===//
//
#include "DSPMCCodeEmitter.h"
#include "MCTargetDesc/DSPBaseInfo.h"
#include "MCTargetDesc/DSPFixupKinds.h"
#include "MCTargetDesc/DSPMCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/raw_ostream.h"
#include <iostream>
using namespace llvm;


#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "DSPGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

MCCodeEmitter *llvm::createDSPMCCodeEmitterEB(const MCInstrInfo &MCII,
	const MCRegisterInfo &MRI,
	const MCSubtargetInfo &STI,
	MCContext &Ctx)
{
	return new DSPMCCodeEmitter(MCII, Ctx, false);
}
MCCodeEmitter *llvm::createDSPMCCodeEmitterEL(const MCInstrInfo &MCII,
	const MCRegisterInfo &MRI,
	const MCSubtargetInfo &STI,
	MCContext &Ctx)
{
	return new DSPMCCodeEmitter(MCII, Ctx, true);
}

void DSPMCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
	OS << (char)C;
}
void DSPMCCodeEmitter::EmitInstruction(uint64_t Val, unsigned Size, raw_ostream &OS) const {
	// Output the instruction encoding in little endian byte order.
	for (unsigned i = 0; i < Size; ++i) {
		unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
		EmitByte((Val >> Shift) & 0xff, OS);
	}
}

/// EncodeInstruction - Emit the instruction.
/// Size the instruction (currently only 4 bytes)
void DSPMCCodeEmitter::
EncodeInstruction(const MCInst &MI, raw_ostream &OS,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const
{
	//std::cout <<std::hex << getBinaryCodeForInstr(MI, Fixups, STI);
	uint32_t Binary = getBinaryCodeForInstr(MI, Fixups, STI);
	std::cout <<"binary����"<< std::hex <<Binary << std::endl;
	// Check for unimplemented opcodes.
	// Unfortunately in DSP both NOT and SLL will come in with Binary == 0
	// so we have to special check for them.
	unsigned Opcode = MI.getOpcode();


	if ((Opcode != DSP::NOP) && (Opcode != DSP::SHL)&&(Opcode!=DSP::Jmp) && !Binary)
		llvm_unreachable("unimplemented opcode in EncodeInstruction()");
	const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
	uint64_t TSFlags = Desc.TSFlags;
	//std::cout << "TS Flag is" << TSFlags << std::endl;
	// Pseudo instructions don��t get encoded and shouldn��t be here
	// in the first place!

	//if ((TSFlags & DSPII::FrmMask) == DSPII::Pseudo)
		//llvm_unreachable("Pseudo opcode found in EncodeInstruction()");


	// For now all instructions are 4 bytes
	int Size = 4; // FIXME: Have Desc.getSize() return the correct value!
	EmitInstruction(Binary, Size, OS);
}

/// getBranch16TargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned DSPMCCodeEmitter::
getBranch16TargetOpValue(const MCInst &MI, unsigned OpNo,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	const MCOperand &MO = MI.getOperand(OpNo);

	// If the destination is an immediate, we have nothing to do.
	if (MO.isImm()) return MO.getImm();
	assert(MO.isExpr() && "getBranch16TargetOpValue expects only expressions");

	const MCExpr *Expr = MO.getExpr();
	Fixups.push_back(MCFixup::Create(0, Expr,
		MCFixupKind(DSP::fixup_DSP_PC16)));

	return 0;
}
/// getBranch24TargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned DSPMCCodeEmitter::
getBranch24TargetOpValue(const MCInst &MI, unsigned OpNo,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	const MCOperand &MO = MI.getOperand(OpNo);

	// If the destination is an immediate, we have nothing to do.
	if (MO.isImm()) return MO.getImm();
	assert(MO.isExpr() && "getBranch24TargetOpValue expects only expressions");

	const MCExpr *Expr = MO.getExpr();
	Fixups.push_back(MCFixup::Create(0, Expr,
		MCFixupKind()));

	return 0;
}
/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand
/// If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned DSPMCCodeEmitter::
getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	unsigned Opcode = MI.getOpcode();
	const MCOperand &MO = MI.getOperand(OpNo);
	// If the destination is an immediate, we have nothing to do.
	if (MO.isImm()) return MO.getImm();
	assert(MO.isExpr() && "getJumpTargetOpValue expects only expressions");

	const MCExpr *Expr = MO.getExpr();
	std::cout << "kind is" << Expr->getKind();
	//if (Opcode == DSP::CALL || Opcode == DSP::Jmp)

	if (Opcode == DSP::Jmp||Opcode == DSP::CALL)
		Fixups.push_back(MCFixup::Create(0, Expr,
		MCFixupKind(DSP::fixup_DSP_PC24)));
	else
		llvm_unreachable("unexpect opcode in getJumpAbsoluteTargetOpValue()");

	return 0;
}


unsigned DSPMCCodeEmitter::
getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	MCExpr::ExprKind Kind = Expr->getKind();
	if (Kind == MCExpr::Binary) {
		Expr = static_cast<const MCBinaryExpr*>(Expr)->getLHS();
		Kind = Expr->getKind();
	}
	assert(Kind == MCExpr::SymbolRef);
	// All of the information is in the fixup.
	std::cout << "Kind" << Kind << std::endl;

	DSP::Fixups FixupKind = DSP::Fixups(0);

	switch (cast<MCSymbolRefExpr>(Expr)->getKind()) {
	case MCSymbolRefExpr::VK_DSP_GPREL:
		FixupKind = DSP::fixup_DSP_GPREL16;
		break;

	case MCSymbolRefExpr::VK_DSP_GOT_CALL:
		FixupKind = DSP::fixup_DSP_CALL;
		break;

	case MCSymbolRefExpr::VK_DSP_GOT16:
		FixupKind = DSP::fixup_DSP_GOT_Global;
		break;
	case MCSymbolRefExpr::VK_DSP_GOT:
		FixupKind = DSP::fixup_DSP_GOT_Local;
		break;
	case MCSymbolRefExpr::VK_DSP_ABS_HI:
		FixupKind = DSP::fixup_DSP_HI16;
		break;
	case MCSymbolRefExpr::VK_DSP_ABS_LO:
		FixupKind = DSP::fixup_DSP_LO16;
		break;

	case MCSymbolRefExpr::VK_DSP_GOT_HI16:
		FixupKind = DSP::fixup_DSP_GOT_HI16;
		break;
	case MCSymbolRefExpr::VK_DSP_GOT_LO16:
		FixupKind = DSP::fixup_DSP_GOT_LO16;
		break;
	default:
		break;
	} // switch

	Fixups.push_back(MCFixup::Create(0, Expr, MCFixupKind(FixupKind)));
	return 0;
}
/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned DSPMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	if (MO.isReg()) {
		unsigned Reg = MO.getReg();
		unsigned RegNo = getDSPRegisterNumbering(Reg);
		return RegNo;
	}
	else if (MO.isImm()) {
		return static_cast<unsigned>(MO.getImm());
	}
	else if (MO.isFPImm()) {
		return static_cast<unsigned>(APFloat(MO.getFPImm())
			.bitcastToAPInt().getHiBits(32).getLimitedValue());
	}
	// MO must be an Expr.
	assert(MO.isExpr());
	return getExprOpValue(MO.getExpr(), Fixups, STI);
}

/// getMemEncoding - Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
unsigned
DSPMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
SmallVectorImpl<MCFixup> &Fixups,
const MCSubtargetInfo &STI) const {
	// Base register is encoded in bits 19-14, offset is encoded in bits 13-5.
	assert(MI.getOperand(OpNo).isReg());
	unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI)<<9;
	unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo + 1), Fixups, STI)>>2;
	return (OffBits & 0xFFFF) | RegBits;
}
#include "DSPGenMCCodeEmitter.inc"