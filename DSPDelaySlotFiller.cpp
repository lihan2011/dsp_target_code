//===-- DSPDelaySlotFiller.cpp - DSP Delay Slot Filler ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Simple pass to fill delay slots with useful instructions.
//
//===----------------------------------------------------------------------===//

#include "DSP.h"


#include "DSPInstrInfo.h"
#include "DSPTargetMachine.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include <iostream>
using namespace llvm;

#define DEBUG_TYPE "delay-slot-filler"

STATISTIC(FilledSlots, "Number of delay slots filled");

namespace {
  typedef MachineBasicBlock::iterator Iter;
  typedef MachineBasicBlock::reverse_iterator ReverseIter;

  class Filler : public MachineFunctionPass {
  public:
    Filler(TargetMachine &tm)
      : MachineFunctionPass(ID), TM(tm) { }

    const char *getPassName() const override {
      return "DSP Delay Slot Filler";
    }

    bool runOnMachineFunction(MachineFunction &F) {
      bool Changed = false;
      for (MachineFunction::iterator FI = F.begin(), FE = F.end();
           FI != FE; ++FI)
        Changed |= runOnMachineBasicBlock(*FI);
      return Changed;
    }
  private:
    bool runOnMachineBasicBlock(MachineBasicBlock &MBB);

    TargetMachine &TM;

    static char ID;
  };
  char Filler::ID = 0;
} // end of anonymous namespace

static bool hasUnoccupiedSlot(const MachineInstr *MI) {
  return MI->hasDelaySlot() && !MI->isBundledWithSucc();
}

static bool isCall(const MachineInstr *MI){
	return MI->isCall();
}

static bool isReturn(const MachineInstr *MI){
	return MI->isReturn();
}
static bool isVLIW(const MachineInstr *MI){
	return false;
}

//read the ld output reg
static bool isUseOutputReg(MachineInstr *I, unsigned Reg){
	bool isPhys = TargetRegisterInfo::isPhysicalRegister(Reg);
	bool Found = false;
	for (unsigned i = 0, e = I->getNumOperands(); i != e; ++i) {
		const MachineOperand &MO = I->getOperand(i);
		if (!MO.isReg() || !MO.isUse())
			continue;
		unsigned MOReg = MO.getReg();
		Found = (MOReg == Reg);
		if (Found) break;
	}
	return Found;
}
static void InsertNopOfNum(MachineBasicBlock &MBB,Iter I, unsigned num, const DSPInstrInfo *TII){
	for (int i = 0; i < num; i++)
	{
		BuildMI(MBB, std::next(I), I->getDebugLoc(), TII->get(DSP::NOP));
	}
}
/// runOnMachineBasicBlock - Fill in delay slots for the given basic block.
/// We assume there is only one delay slot per delayed instruction.
bool Filler::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  bool Changed = false;

  for (Iter I = MBB.begin(); I != MBB.end(); ++I) {
    if (!hasUnoccupiedSlot(&*I))
      continue;

    ++FilledSlots;
    Changed = true;
	const DSPInstrInfo *TII = static_cast<const DSPInstrInfo*>(TM.getInstrInfo());
	if (I->getOpcode() == DSP::LD){
		unsigned reg = I->getOperand(0).getReg();
		auto Next = std::next(I,1);
		auto Next_2 = std::next(I, 2);
		if (Next == MBB.end()){
			InsertNopOfNum(MBB, I, 2, TII);
			continue;
		}
		if (Next_2 == MBB.end())
		{
			InsertNopOfNum(MBB, I, 1, TII);
			continue;
		}
		if (isUseOutputReg(Next, reg)) {
			InsertNopOfNum(MBB, I, 2, TII);
			continue;
		}
		else if (isUseOutputReg(Next_2, reg))
		{
			InsertNopOfNum(MBB, I, 1, TII);
			continue;
		}
		else continue;
	}
    // Bundle the NOP to the instruction with the delay slot.
    
	/*for (int i = 0; i < 2; i++)
	{
		BuildMI(MBB, std::next(I), I->getDebugLoc(), TII->get(DSP::NOP));
	}*/
	InsertNopOfNum(MBB, I, 2, TII);
	//MIBundleBuilder MIB(MBB, I, std::next(I, 2));
	//MIB.append()
	//BuildMI(MBB, std::next(I), I->getDebugLoc(), TII->get(DSP::NOP_S));

  }

  return Changed;
}

/// createDSPDelaySlotFillerPass - Returns a pass that fills in delay
/// slots in DSP MachineFunctions
FunctionPass *llvm::createDSPDelaySlotFillerPass(DSPTargetMachine &tm) {
  return new Filler(tm);
}


