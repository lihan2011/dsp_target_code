//
// An implementation of the Swing Modulo Scheduling (SMS) software pipeliner.
//
/*
The algorithm followed by SMS consists of the following three steps that are described in
detail below:
1. Computation and analysis of the dependence graph.
2.Ordering of the nodes.
3.Scheduling.


*/
#include "DSP.h"
#include "MCTargetDesc/DSPBaseInfo.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Analysis/CodeMetrics.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/RegisterPressure.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/Pass.h"

#include "llvm/PassSupport.h"
#include "llvm/Support/CommandLine.h"
#include <vector>
#include <iostream>
#include <climits>
#include <algorithm>
using namespace llvm;


// Helper function (copied from LoopVectorize.cpp)
static void addInnerLoop(Loop &L, SmallVectorImpl<Loop *> &V) {
	if (L.empty())
		return V.push_back(&L);

	for (Loop *InnerL : L)
		addInnerLoop(*InnerL, V);
}
namespace llvm {
	void initializeDSPSWLoopsPass(PassRegistry&);
}

namespace {
	class DSPSWLoops : public MachineFunctionPass{
	public:
		static char ID;
		explicit DSPSWLoops() :MachineFunctionPass(ID){
			initializeDSPSWLoopsPass(*PassRegistry::getPassRegistry());
		}
		bool runOnMachineFunction(MachineFunction &MF) override{
			this->MF = &MF;
			//CMA = &getAnalysis<CostModelAnalysis>();
			MLI = &getAnalysis<MachineLoopInfo>();
			MDT = &getAnalysis<MachineDominatorTree>();
			AA = &getAnalysis<AliasAnalysis>();
			RegClassInfo.runOnMachineFunction(MF);
			bool Changed;
			for (MachineLoopInfo::iterator I = MLI->begin(),E = MLI->end(); I !=E; I++)
			{
				MachineLoop *L = *I;

				std::cout << "Loop " << (*I)->getLoopDepth() << std::endl;
				Changed = Process(L);
			}
			return true;
		}
		void getAnalysisUsage(AnalysisUsage &AU) const override{
			AU.addRequired<AliasAnalysis>();
			AU.addPreserved<AliasAnalysis>();
			AU.addRequired<MachineLoopInfo>();
			AU.addRequired<MachineDominatorTree>();
			AU.addRequired<LiveIntervals>();
			AU.addPreserved<LiveIntervals>();
			MachineFunctionPass::getAnalysisUsage(AU);
		}
	public:
		MachineFunction *MF = nullptr;
		const MachineLoopInfo *MLI;
		const MachineDominatorTree *MDT;
		DependenceAnalysis *DA;
		AliasAnalysis *AA = nullptr;
		RegisterClassInfo RegClassInfo;
		bool Process(MachineLoop *L);
		bool swingModuloScheduler(MachineLoop *L);
		bool canPipelineLoop(MachineLoop *L);

		class InstructionTrace
		{
		public:
			InstructionTrace() :weight(0){}
			InstructionTrace(const InstructionTrace  &IT) :trace(IT.trace), weight(IT.weight){}
			//~InstructionTrace();

			void add(Instruction *I){
				trace.push_back(I);
				weight += 1;
			}

			unsigned getWeight(){
				return weight;
			}

			const SmallVector<Instruction*, 8> &data() const {
				return trace;
			}

			size_t size() {
				return trace.size();
			}

			const Instruction* find(Instruction *I) const {
				for (auto II = trace.begin(),E = trace.end(); II !=E; II++)
					if (*II == I) return *II;
				return nullptr;
			
			}

			//overload the < operator  I < J == I.<(J) return a bool value

			bool operator < (const InstructionTrace &I)const {
				return weight <= I.weight&&this != &I;
			}
		private:
			SmallVector<Instruction*, 8> trace;
			unsigned weight;

		};

		typedef std::set<InstructionTrace> CycleSet;
		//caculate the resource minimal initial interval
		unsigned calculateResMII(MachineLoop *L);

		//caculate the recursion minimal initial interval
		unsigned calculateRecMII(MachineLoop *L, CycleSet &C);


		// Helper function to find loop dependency cycles through phi nodes
		void getPhiCycles(MachineInstr *I,const PHINode *Phi,InstructionTrace trace,CycleSet &cycles);

		bool getConnectingNodes(Instruction *I,
			const BasicBlock *MBB,
			DenseMap<Instruction*, bool> &VisitedNodes,
			std::vector<Instruction *> &connectionNodes,
			bool direction
			);
	};


	//this class is used to build dependence graph
	class SwingSchedulerDAG : public ScheduleDAGInstrs{
		DSPSWLoops &pass;

		unsigned MII;
		bool isScheduled;
		LiveIntervals *LIS;
		MachineLoop &L;

		/// A toplogical ordering of the SUnits, which is needed for changing
		/// dependences and iterating over the SUnits.
		ScheduleDAGTopologicalSort Topo;

		struct NodeInfo{
			int ASAP;
			int ALAP;
			NodeInfo() :ASAP(0), ALAP(0){};
		};

		SetVector<SUnit*> NodeOrder;
		std::vector<NodeInfo> ScheduleInfo;
	public :
		class NodeSet{
			SetVector<SUnit*> Nodes;
			bool hasRescurrence;
			unsigned RecMII;
			int MaxMov;
			int MaxDepth;
			unsigned Colocate;
			SUnit *ExceedPressure;
		public:
			typedef SetVector<SUnit*>::iterator iterator;
			typedef SetVector<SUnit*>::const_iterator const_iterator;
			NodeSet() :Nodes(), hasRescurrence(false), RecMII(0), MaxMov(0), MaxDepth(0), Colocate(0), ExceedPressure(nullptr){}

			template <typename It>
			NodeSet(It S, It E)
				: Nodes(S, E), HasRecurrence(true), RecMII(0), MaxMOV(0), MaxDepth(0),
				Colocate(0), ExceedPressure(nullptr) {}
		};
	private:
		const RegisterClassInfo &RegClassInfo;
	public:
		SwingSchedulerDAG(DSPSWLoops &P, MachineLoop *L, const RegisterClassInfo &rci, LiveIntervals *LIV)
			:ScheduleDAGInstrs(*P.MF, *P.MLI, *P.MDT, false), pass(P), MII(0), isScheduled(false),
			L(*L), Topo(SUnits, &ExitSU), RegClassInfo(rci), LIS(LIV){}

		void schedule() ;
		int getASAP(SUnit *Node) { return ScheduleInfo[Node->NodeNum].ASAP; }

		/// Return the latest time an instruction my be scheduled.
		int getALAP(SUnit *Node) { return ScheduleInfo[Node->NodeNum].ALAP; }

		/// The mobility function, which the the number of slots in which
		/// an instruction may be scheduled.
		int getMOV(SUnit *Node) { return getALAP(Node) - getASAP(Node); }

		/// The depth, in the dependence graph, for a node.
		int getDepth(SUnit *Node) { return Node->getDepth(); }

		/// The height, in the dependence graph, for a node.
		int getHeight(SUnit *Node) { return Node->getHeight(); }

	};
} // end anonymous namespace

char DSPSWLoops::ID = 0;

INITIALIZE_PASS_BEGIN(DSPSWLoops,"DSPSWLoops", "SoftWare Pipeline",false,false)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(DSPSWLoops, "DSPSWLoops", "SoftWare Pipeline",false,false)
//******************************static help function*********************************************
bool isAluInst(MachineInstr *MI){
	return (MI->getDesc().TSFlags >> DSPII::isAluPos)&DSPII::isAluMask;
}

bool isSlot0_Mov(MachineInstr *MI){
	switch (MI->getOpcode())
	{
	case DSP::MovG2V10:
	case DSP::MovG2V40:
	case DSP::MovV2G10:
	case DSP::MovV2G40:
		return true;
	default:
		return false;
	}
}

bool isSlot01_Mov(MachineInstr *MI){
	switch (MI->getOpcode())
	{
	default:return false;
		break;
	}
}

// Helper function to find loop dependency cycles through phi nodes
/*void DSPSWLoops::getPhiCycles(Instruction *I, const PHINode *Phi,
	InstructionTrace trace,
	CycleSet &cycles) {
	// stay within the loop body
	if (I->getParent() != Phi->getParent())
		return;

	// found a cycle when we end up at our start point
	if (I == Phi && trace.size() != 0) {
		trace.add(I);
		cycles.insert(trace);
		return;
	}

	// found a cycle not passing through the currently considered phi-node
	// for example: a -> b -> c -> b, this can only happen if b is a phi-node
	if (isa<PHINode>(I) && trace.find(I)) {
		return;
	}

	// Add current instruction and check cycles for each operand of the
	// instruction.  Don't add original Phi node until the cycle is completed
	// to preserve (reversed) ordering.
	if (I != Phi)
		trace.add(I);

	if (isa<PHINode>(I)) {
		PHINode *P = cast<PHINode>(I);

		for (unsigned i = 0; i < P->getNumIncomingValues(); i++) {
			Instruction *II = dyn_cast<Instruction>(P->getIncomingValue(i));
			if (II) getPhiCycles(II, Phi, trace, cycles);
		}
	}
	else {
		for (auto &O : I->operands()) {
			Instruction *II = dyn_cast<Instruction>(O);
			if (II) getPhiCycles(II, Phi, trace, cycles);
		}
	}
}*/

// Helper function to find the nodes located at any path between the previous
// and current recurrence.
bool DSPSWLoops::getConnectingNodes(Instruction *I,
	const BasicBlock *B,
	DenseMap<Instruction *, bool> &VisitedNodes,
	std::vector<Instruction *> &connectingNodes,
	bool direction)
{
	// Do not recurse over nodes outside of the current loop body
	if (I->getParent() != B) return false;

	// Recurse until a previously visited node is found
	if (VisitedNodes[I]) return true;

	// Recurse through operands/uses depending on direction
	bool found = false;
	if (direction) {
		// avoid backedges
		if (isa<PHINode>(I)) return false;

		// search upwards
		for (auto &O : I->operands()) {
			Instruction *II = dyn_cast<Instruction>(O);
			if (II)
				found |= getConnectingNodes(II, B, VisitedNodes, connectingNodes, direction);
		}
	}
	else {
		// search downwards
		for (auto U : I->users()) {
			if (isa<PHINode>(U)) continue;
			Instruction *II = dyn_cast<Instruction>(U);
			if (II)
				found |= getConnectingNodes(II, B, VisitedNodes, connectingNodes, direction);
		}
	}

	// Add current node to the visited list and to the connecting nodes if a path was found
	if (found) {
		VisitedNodes[I] = true;
		connectingNodes.push_back(I);
	}

	return found;
}


//********************************
bool DSPSWLoops::canPipelineLoop(MachineLoop *L){
	// Check if loop body has no control flow (single BasicBlock)
	unsigned NumBlocks = L->getNumBlocks();
	if (NumBlocks != 1){
		return false;
	}
	return true;
}



//4 slots
//see DSPSchedule.td
unsigned DSPSWLoops::calculateResMII(MachineLoop *L){
	unsigned NumofSlot2_3Inst = 0;
	unsigned NumOfSlot0Instr = 0;
	unsigned NumOfSlot0_1Instr = 0;
	unsigned NumOfOtherInstr = 0;
	unsigned ResMII = 0;
	std::vector<MachineBasicBlock*>::const_iterator first = L->block_begin();
	unsigned NumofMemInst = 0;
	for (auto MII = (*first)->begin(), MIIe = (*first)->end(); MII != MIIe; MII++)
	{
		//ld/st slot2 or slot3
		if (MII->mayLoad() || MII->mayStore()) NumofSlot2_3Inst++;
		//mul64& some mov instrs  slot0
		else if (MII->getOpcode() == DSP::MUL||isSlot0_Mov(MII)) NumOfSlot0Instr++;
		//alu & some mov instrs  slot0 or slot1
		else if (isAluInst(MII)||isSlot01_Mov(MII)) NumOfSlot0_1Instr++;
		//slot 0 1 2 3
		else NumOfOtherInstr++;

	}
	float Cycle_01 = NumOfSlot0_1Instr + NumOfSlot0Instr / 2;
	float Cycle_23 = NumofSlot2_3Inst / 2;
	unsigned Cycle_tmp = std::max(std::ceil(Cycle_01), std::ceil(Cycle_23));
	unsigned Slot_surplus = Cycle_tmp * 4 - NumOfSlot0Instr - NumOfSlot0_1Instr - NumofSlot2_3Inst;

	if ((Slot_surplus - NumOfOtherInstr > 0)) ResMII = Cycle_tmp;
	else ResMII = Cycle_tmp + std::ceil((NumOfOtherInstr - Slot_surplus) / 4);

	assert(ResMII != 0 && "ResMII could not be zero");
	return ResMII;

}




unsigned DSPSWLoops::calculateRecMII(MachineLoop *L, CycleSet &C){
	MachineBasicBlock *LoopBody = L->getBlocks()[0];
	for (auto I = LoopBody->begin(),E = LoopBody->end(); I != E; I++)
	{
		if (!I->isPHI())
		{
			continue;
		}
		InstructionTrace tr;
		//getPhiCycles(Phi, Phi, tr, C);
	}
	return 2;
}

bool DSPSWLoops::swingModuloScheduler(MachineLoop *L){
	if (L->getBlocks().size() != 1) llvm_unreachable("SMS works on single block only");
	SwingSchedulerDAG SMS(*this, L, RegClassInfo, &getAnalysis<LiveIntervals>());
	MachineBasicBlock *MBB = L->getHeader();
	SMS.startBlock(MBB);
	unsigned size = MBB->size();
	std::cout << "tem" << MBB->getFirstInstrTerminator()->getOpcode() << std::endl;
	std::cout << "instr_end" << MBB->instr_end()->getOpcode() << std::endl;
	for (MachineBasicBlock::iterator I = MBB->getFirstTerminator(),
		E = MBB->instr_end();
		I != E; ++I, --size);
	/*MachineBasicBlock::iterator E = MBB->SkipPHIsAndLabels(MBB->begin());
	for (auto b = MBB->begin(); b != E; b++){
		size++;
		std::cout << "loop instr " << b->getOpcode() << std::endl;
	}*/
	SMS.enterRegion(MBB, MBB->begin(), MBB->getFirstInstrTerminator(), size);
	SMS.schedule();
	SMS.exitRegion();
	SMS.finishBlock();
	return true;
}

void SwingSchedulerDAG::schedule(){
	AliasAnalysis *AA = pass.AA;
	buildSchedGraph(AA);
	//Topo.InitDAGTopologicalSorting();
	std::cout << "schedule" << std::endl;
	std::cout << "name" << getDAGName() << std::endl;
}

bool DSPSWLoops::Process(MachineLoop *L){
	bool Changed = false;
	for (auto &InnerLoop : *L)
	{
		Changed |= Process(InnerLoop);
	}
	if (!canPipelineLoop(L))
		return Changed;
	Changed = swingModuloScheduler(L);
	return Changed;
}
//===----------------------------------------------------------------------===//
//                         Public Constructor Functions
//===----------------------------------------------------------------------===//

FunctionPass *llvm::createLoopPipelinePass() {
	return new DSPSWLoops();
}