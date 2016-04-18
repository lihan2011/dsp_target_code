//===-- DSPSEISelDAGToDAG.cpp - A Dag to Dag Inst Selector for DSPSE ----===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of DSPDAGToDAGISel specialized for mips32/64.
//
//===----------------------------------------------------------------------===//
#include "DSPSEISelDAGToDAG.h"
#include "MCTargetDesc/DSPBaseInfo.h"
#include "DSPAnalyzeImmediate.h"
#include "DSP.h"
#include "DSPMachineFunction.h"
#include "DSPRegisterInfo.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <iostream>
using namespace llvm;
#define DEBUG_TYPE "dsp-isel"


bool DSPSEDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
	Subtarget = &TM.getSubtarget<DSPSubtarget>();
	return DSPDAGToDAGISel::runOnMachineFunction(MF);
}
void DSPSEDAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
}

SDNode* DSPSEDAGToDAGISel::selectADD_FI(SDNode *Node){
	SDLoc DL(Node);
	SDValue AddNode = Node->getOperand(2);
	FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(AddNode.getOperand(0));
	SDValue Chain = Node->getOperand(0);
	EVT ValTy = Node->getValueType(0);
	
	SDValue StackPtr = CurDAG->getCopyFromReg(Chain, DL, DSP::SP, ValTy);
	SDValue PtrBase = CurDAG->getNode(DSP::ADDu, DL, ValTy, AddNode.getOperand(1), StackPtr);
	std::cout << "begin select node to" << std::endl;
	CurDAG->getNode(ISD::STORE, DL, ValTy, PtrBase, CurDAG->getTargetFrameIndex(FIN->getIndex(),ValTy));
	return Node;
}
std::pair<bool, SDNode*> DSPSEDAGToDAGISel::selectNode(SDNode *Node) {
	unsigned Opcode = Node->getOpcode();
	SDLoc DL(Node);
	SDNode *Result;
	///
	// Instruction Selection not handled by the auto-generated
	// tablegen selection should be handled here.
	///
	EVT NodeTy = Node->getValueType(0);
	
	unsigned MultOpc;
	switch (Opcode) {
	/*case ISD::ADD:
	{
					 if ((Node->getOperand(0).getOpcode()) == ISD::FrameIndex) return std::make_pair(true,selectADD_FI(Node));
					 //break;
	}
	case ISD::STORE:{
						if ((Node->getOperand(2).getOpcode()) == ISD::ADD)  return std::make_pair(true, selectADD_FI(Node));
						break;
	}*/
	default: break;
	}
	return std::make_pair(false, nullptr);
}

FunctionPass *llvm::createDSPSEISelDag(DSPTargetMachine &TM) {
	return new DSPSEDAGToDAGISel(TM);

}