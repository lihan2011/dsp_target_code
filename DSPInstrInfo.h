//===-- DSPInstrInfo.h - DSP Instruction Information ----------*- C++ -*-===//
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

#ifndef DSPINSTRUCTIONINFO_H
#define DSPINSTRUCTIONINFO_H

#include "DSP.h"
#include "DSPAnalyzeImmediate.h"
#include "DSPRegisterInfo.h"
#include "MCTargetDesc\DSPBaseInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "DSPGenInstrInfo.inc"

namespace llvm{

	class DSPInstrInfo : public DSPGenInstrInfo{
		virtual void anchor();

		//const DSPRegisterInfo RI;
	protected:
		const DSPSubtarget &Subtarget;
	public:
		explicit DSPInstrInfo(const DSPSubtarget &STI);

		static const DSPInstrInfo *create(const DSPSubtarget &STI);

		//const DSPRegisterInfo &getRegisterInfo()const{ return RI; }
		virtual const DSPRegisterInfo &getRegisterInfo() const = 0;

		bool AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
			MachineBasicBlock *&FBB,
			SmallVectorImpl<MachineOperand> &Cond,
			bool AllowModify) const override;

		unsigned RemoveBranch(MachineBasicBlock &MBB) const override;

		unsigned InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
			MachineBasicBlock *FBB,
			const SmallVectorImpl<MachineOperand> &Cond,
			DebugLoc DL) const override;
		bool
			ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;

		void BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
			DebugLoc DL,
			const SmallVectorImpl<MachineOperand> &Cond) const;

		void storeRegToStackSlot(MachineBasicBlock &MBB,
			MachineBasicBlock::iterator MBBI,
			unsigned SrcReg, bool isKill, int FrameIndex,
			const TargetRegisterClass *RC,
			const TargetRegisterInfo *TRI) const override {
			storeRegToStack(MBB, MBBI, SrcReg, isKill, FrameIndex, RC, TRI, 0);
		}
		void loadRegFromStackSlot(MachineBasicBlock &MBB,
			MachineBasicBlock::iterator MBBI,
			unsigned DestReg, int FrameIndex,
			const TargetRegisterClass *RC,
			const TargetRegisterInfo *TRI) const override {
			loadRegFromStack(MBB, MBBI, DestReg, FrameIndex, RC, TRI, 0);
		}
		virtual void storeRegToStack(MachineBasicBlock &MBB,
			MachineBasicBlock::iterator MI, unsigned SrcReg, bool isKill, int FrameIndex,
			const TargetRegisterClass *RC,
			const TargetRegisterInfo *TRI,
			int64_t Offset) const = 0;
		virtual void loadRegFromStack(MachineBasicBlock &MBB,
			MachineBasicBlock::iterator MI,
			unsigned DestReg, int FrameIndex,
			const TargetRegisterClass *RC,
			const TargetRegisterInfo *TRI,
			int64_t Offset) const = 0;

		virtual MachineInstr* emitFrameIndexDebugValue(MachineFunction &MF,
			int FrameIx, uint64_t Offset,
			const MDNode *MDPtr,
			DebugLoc DL) const;

		bool isExtended(const MachineInstr* MI) const;

		bool isConstExtended(MachineInstr *MI) const;

		DFAPacketizer *CreateTargetScheduleState(const TargetMachine *TM,
			const ScheduleDAG *DAG) const;

		bool analyzeCompare(const MachineInstr * MI, unsigned & SrcReg, unsigned & SrcReg2, int & Mask, int & Value) const;

		bool AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
			SmallVectorImpl<MachineOperand> &Cond,
			bool AllowModify,
			bool isLoop) const;
		//unsigned CaculateNumOfMemInst(MachineLoop *L);
		//unsigned CaculateNumOfAluInst(MachineLoop *L);
	};

	const DSPInstrInfo *createDSPSEInstrInfo(const DSPSubtarget &STI);
}
#endif