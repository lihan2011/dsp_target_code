//===-- DSPInstrInfo.cpp - DSP Instruction Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the DSP implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "DSPInstrInfo.h"
#include "DSP.h"
#include "DSPSubtarget.h"
#include "DSPTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
#include <limits>
#include <iostream>
using namespace llvm;

//#define DEBUG_TYPE "dsp-instr-info"

#define GET_INSTRINFO_CTOR_DTOR
#include "DSPGenInstrInfo.inc"
#include "DSPGenDFAPacketizer.inc"

void DSPInstrInfo::anchor(){}


DSPInstrInfo::DSPInstrInfo(const DSPSubtarget &STI) 	
    :DSPGenInstrInfo(DSP::ADJCALLSTACKDOWN, DSP::ADJCALLSTACKUP),
	Subtarget(STI){};
const DSPInstrInfo *DSPInstrInfo::create(const DSPSubtarget &STI){
	return llvm::createDSPSEInstrInfo(STI);

}
MachineInstr*
DSPInstrInfo::emitFrameIndexDebugValue(MachineFunction &MF, int FrameIx,
uint64_t Offset, const MDNode *MDPtr,
DebugLoc DL) const {
	MachineInstrBuilder MIB = BuildMI(MF, DL, get(DSP::DBG_VALUE))
		.addFrameIndex(FrameIx).addImm(0).addImm(Offset).addMetadata(MDPtr);
	return &*MIB;
}


// This returns true in two cases:
// - The OP code itself indicates that this is an extended instruction.
// - One of MOs has been marked with HMOTF_ConstExtended flag.
bool DSPInstrInfo::isExtended(const MachineInstr *MI)const{
	const uint64_t F = MI->getDesc().TSFlags;
	if ((F >> DSPII::ExtendedPos)&DSPII::ExtendedMask)
		return true;
	return false;
}

unsigned getAnalyzableBrOpc(unsigned Opc)  {
	return (Opc == DSP::JC || Opc == DSP::JNC || Opc == DSP::Jmp) ? Opc : 0;
}


unsigned getOppoSiteBrOpc(unsigned Opc){
	if (Opc == DSP::JNC)
		return DSP::JC;
	else if (Opc == DSP::JC)
		return DSP::JNC;
	else return Opc;
}
static void AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
							MachineBasicBlock *&BB, 
							SmallVectorImpl<MachineOperand> &Cond){
	assert(getAnalyzableBrOpc(Opc) && "Not an analyzable branch");
	int NumOp = Inst->getNumExplicitOperands();
	// for both int and fp branches, the last explicit operand is the
	// MBB.
	BB = Inst->getOperand(NumOp - 1).getMBB();
	Cond.push_back(MachineOperand::CreateImm(Opc));

	for (int i = 0; i<NumOp - 1; i++)
		Cond.push_back(Inst->getOperand(i));
}

void DSPInstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
DebugLoc DL,
const SmallVectorImpl<MachineOperand> &Cond) const {
	unsigned Opc = Cond[0].getImm();
	//Opc = getOppoSiteBrOpc(Opc);
	const MCInstrDesc &MCID = get(Opc);
	MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

	for (unsigned i = 1; i < Cond.size(); ++i) {
		if (Cond[i].isReg())
			MIB.addReg(Cond[i].getReg());
		else if (Cond[i].isImm())
			MIB.addImm(Cond[i].getImm());
		else
			assert(true && "Cannot copy operand");
	}
	MIB.addMBB(TBB);
}
/// AnalyzeBranch - Analyze the branching code /* at the end of MBB */ , returning
/// true if it cannot be understood (e.g. it's a switch dispatch or isn't
/// implemented for a target).  Upon success, this returns false and returns
/// with the following information in various cases:
///
/// 1. If this block ends with no branches (it just falls through to its succ)
///    just return false, leaving TBB/FBB null.
/// 2. If this block ends with only an unconditional branch, it sets TBB to be
///    the destination block.
/// 3. If this block ends with a conditional branch and it falls through to a
///    successor block, it sets TBB to be the branch destination block and a
///    list of operands that evaluate the condition. These operands can be
///    passed to other TargetInstrInfo methods to create new branches.
/// 4. If this block ends with a conditional branch followed by an
///    unconditional branch, it returns the 'true' destination in TBB, the
///    'false' destination in FBB, and a list of operands that evaluate the
///    condition.  These operands can be passed to other TargetInstrInfo
///    methods to create new branches.
///
/// Note that RemoveBranch and InsertBranch must be implemented to support
/// cases where this method returns success.
///
/// If AllowModify is true, then this routine is allowed to modify the basic
/// block (e.g. delete instructions after the unconditional branch).
///
bool DSPInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
	SmallVectorImpl<MachineOperand> &Cond,
	bool AllowModify) const {
	//std::cout << "Mbb name" << MBB.getName().data() << std::endl;
	MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
	while (I != REnd&&I->isDebugValue())
		++I;
	if (I == REnd || !isUnpredicatedTerminator(&*I)){
		// This block ends with no branches (it just falls through to its succ).
		// Leave TBB/FBB null. case 1
		TBB = FBB = nullptr;
		return false;
	}

	MachineInstr *LastInst = &*I;
	unsigned LastOpc = LastInst->getOpcode();

	if (!getAnalyzableBrOpc(LastOpc))
		return true;
	unsigned SecondLastOpc = 0;
	MachineInstr *SecondLastInst = nullptr;

	if (++I != REnd){
		SecondLastInst = &*I;
		SecondLastOpc = getAnalyzableBrOpc(SecondLastInst->getOpcode());


		// Not an analyzable branch (must be an indirect jump).
		if (isUnpredicatedTerminator(SecondLastInst) && !SecondLastOpc)
			return false;
	}

	// If there is only one terminator instruction, process it.
	if (!SecondLastOpc){
		//case 2
		if (LastOpc == DSP::Jmp){
			TBB = LastInst->getOperand(0).getMBB();
			return false;
		}

		// Conditional branch
		AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
		return false;
	}

	// If we reached here, there are two branches.
	// If there are three terminators, we don't know what sort of block this is.
	if (++I != REnd && isUnpredicatedTerminator(&*I))
		return true;

	// If second to last instruction is an unconditional branch,
	// analyze it and remove the last instruction.
	if (SecondLastOpc == DSP::Jmp) {
		// Return if the last instruction cannot be removed.
		if (!AllowModify)
			return true;

		TBB = SecondLastInst->getOperand(0).getMBB();
		LastInst->eraseFromParent();
		//BranchInstrs.pop_back();
		return false;
	}


	// Conditional branch followed by an unconditional branch.
	// The last one must be unconditional.
	if (LastOpc != DSP::Jmp)
		return true;

	//std::cout << "?????" << std::endl;

	AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
	FBB = LastInst->getOperand(0).getMBB();



	return false;
}

	bool DSPInstrInfo::ReverseBranchCondition(
		SmallVectorImpl<MachineOperand> &Cond) const {
		assert((Cond.size() && Cond.size() <= 3) &&
			"Invalid Mips branch condition!");
		Cond[0].setImm(getOppoSiteBrOpc(Cond[0].getImm()));
		//std::cout << " revese opcode" << Cond[0].getImm() << std::endl;
		return false;
	}

/// RemoveBranch - Remove the branching code at the end of the specific MBB.
/// This is only invoked in cases where AnalyzeBranch returns success. It
/// returns the number of instructions that were removed.
unsigned DSPInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const{
	MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
	MachineBasicBlock::reverse_iterator FirstBr;
	unsigned removed;

	// Skip all the debug instructions.
	while (I != REnd && I->isDebugValue())
		++I;

	FirstBr = I;

	// Up to 2 branches are removed.
	// Note that indirect branches are not removed.
	for (removed = 0; I != REnd && removed < 2; ++I, ++removed)
		if (!getAnalyzableBrOpc(I->getOpcode()))
			break;

	MBB.erase(I.base(), FirstBr.base());

	return removed;
}



/// InsertBranch - Insert branch code into the end of the specified
/// MachineBasicBlock.  The operands to this method are the same as those
/// returned by AnalyzeBranch.  This is only invoked in cases where
/// AnalyzeBranch returns success. It returns the number of instructions
/// inserted.
///
/// It is also invoked by tail merging to add unconditional branches in
/// cases where AnalyzeBranch doesn't apply because there was no original
/// branch to analyze.  At least this much must be implemented, else tail
/// merging needs to be disabled.
unsigned DSPInstrInfo::InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
	MachineBasicBlock *FBB,
	const SmallVectorImpl<MachineOperand> &Cond,
	DebugLoc DL) const {
	//llvm_unreachable("do not implement InsertBranch");

	// Shouldn't be a fall through.
	assert(TBB && "InsertBranch must not be told to insert a fallthrough");

	// # of condition operands:
	//  Unconditional branches: 0
	//  Floating point branches: 1 (opc)
	//  Int BranchZero: 2 (opc, reg)
	//  Int Branch: 3 (opc, reg0, reg1)
	assert((Cond.size() <= 3) &&
		"# of Mips branch conditions must be <= 3!");

	// Two-way Conditional branch.
	if (FBB) {
		BuildCondBr(MBB, TBB, DL, Cond);
		BuildMI(&MBB, DL, get(DSP::Jmp)).addMBB(FBB);
		return 2;
	}

	// One way branch.
	// Unconditional branch.
	if (Cond.empty())
		BuildMI(&MBB, DL, get(DSP::Jmp)).addMBB(TBB);
	else // Conditional branch.
		BuildCondBr(MBB, TBB, DL, Cond);
	return 1;
	}
bool DSPInstrInfo::isConstExtended(MachineInstr *MI) const {

	const uint64_t F = MI->getDesc().TSFlags;

	unsigned isExtended = (F >> DSPII::ExtendedPos)&DSPII::ExtendedMask;
	if (isExtended)
		return true;
	return false;
}

DFAPacketizer *DSPInstrInfo::
CreateTargetScheduleState(const TargetMachine *TM,
const ScheduleDAG *DAG) const {
	//std::cout << "createTargetScheduleState" << std::endl;
	const InstrItineraryData *II = TM->getInstrItineraryData();
	return TM->getSubtarget<DSPGenSubtargetInfo>().createDFAPacketizer(II);
}
bool DSPInstrInfo::getBaseAndOffset(const MachineInstr *MI, int &Value, unsigned int offset) const {
	return false;
}
/// If the instruction is an increment of a constant value, return the amount.
bool DSPInstrInfo::getIncrementValue(const MachineInstr *MI,
	int &Value) const {
	if (isPostIncrement(MI)) {
		unsigned AccessSize;
		return getBaseAndOffset(MI, Value, AccessSize);
	}
	if (MI->getOpcode() == DSP::ADDiu) {
		Value = MI->getOperand(2).getImm();
		return true;
	}

	return false;
}
