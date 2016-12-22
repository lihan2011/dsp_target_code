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
#include "MCTargetDesc/DSPBaseInfo.h"
#include "DSPSubtarget.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Analysis/CodeMetrics.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
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
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetInstrInfo.h"
#include <vector>
#include <deque>
#include <iostream>
#include <climits>
#include <algorithm>
using namespace llvm;

#define DEBUG_TYPE "swpipeline"
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

cl::opt<bool> EnableSwPipeline("sw-pipeline", cl::init(false), cl::NotHidden,
	cl::desc("start software pipeline"));

namespace {
	class NodeSet;
	class SMSchedule;
	typedef SmallVector<NodeSet, 8> NodeSetType;

	class DSPSWLoops : public MachineFunctionPass{
	public:
		static char ID;
		explicit DSPSWLoops() :MachineFunctionPass(ID), MF(nullptr), MLI(nullptr), MDT(nullptr), AA(nullptr),
			TII(nullptr){
			initializeDSPSWLoopsPass(*PassRegistry::getPassRegistry());
		}
		bool runOnMachineFunction(MachineFunction &mf) override{
			this->MF = &mf;
			//CMA = &getAnalysis<CostModelAnalysis>();
			MLI = &getAnalysis<MachineLoopInfo>();
			MDT = &getAnalysis<MachineDominatorTree>();
			AA = &getAnalysis<AliasAnalysis>();
			RegClassInfo.runOnMachineFunction(mf);
			TII = MF->getTarget().getInstrInfo();
			bool Changed;
			for (MachineLoopInfo::iterator I = MLI->begin(),E = MLI->end(); I !=E; I++)
			{
				MachineLoop *L = *I;

				DEBUG(dbgs() << "*****************************Start Perform SoftWare Pipeling****************************" << "\n");
				Changed = Process(L);
			}
			return true;
		}

		const char* getPassName() const override{
			return "DSP SoftWare Pipeline Pass";
		}
		void getAnalysisUsage(AnalysisUsage &AU) const override{
			AU.setPreservesCFG();
			AU.addRequiredID(MachineDominatorsID);
			AU.addRequired<MachineLoopInfo>();
			AU.addRequired<AliasAnalysis>();
			AU.addRequired<LiveIntervals>();
			AU.addPreserved<LiveIntervals>();
			MachineFunctionPass::getAnalysisUsage(AU);
		}
	public:
		MachineFunction *MF;
		const MachineLoopInfo *MLI;
		const MachineDominatorTree *MDT;
		DependenceAnalysis *DA;
		AliasAnalysis *AA;
		RegisterClassInfo RegClassInfo;
		const TargetInstrInfo *TII;
		bool Process(MachineLoop *L);
		bool swingModuloScheduler(MachineLoop *L);
		bool canPipelineLoop(MachineLoop *L);


		
	};


	//this class is used to build dependence graph
	class SwingSchedulerDAG : public ScheduleDAGInstrs{
		DSPSWLoops &pass;
		unsigned MII;
		bool isScheduled;
		MachineLoop &L;
		LiveIntervals &LIS;


		/// A toplogical ordering of the SUnits, which is needed for changing
		/// dependences and iterating over the SUnits.
		ScheduleDAGTopologicalSort Topo;

		struct NodeInfo{
			int ASAP;
			int ALAP;
			NodeInfo() :ASAP(0), ALAP(0){};
		};

		enum OrderKind{ BottomUp = 0, TopDown = 1 };

		SetVector<SUnit*> NodeOrder;
		std::vector<NodeInfo> ScheduleInfo;

		/// Instructions to change when emitting the final schedule.
		DenseMap<SUnit *, std::pair<unsigned, int64_t>> InstrChanges;

		/// We may create a new instruction, so remember it because it
		/// must be deleted when the pass is finished.
		SmallPtrSet<MachineInstr *, 4> NewMIs;
	public :
		
		/// Helper class to implement Johnson's circuit finding algorithm.
		class Circuits {
			std::vector<SUnit> &SUnits;
			SetVector<SUnit *> Stack;
			BitVector Blocked;
			SmallVector<SmallPtrSet<SUnit *, 4>, 10> B;
			SmallVector<SmallVector<int, 4>, 16> AdjK;
			unsigned NumPaths;
			static unsigned MaxPaths;

		public:
			Circuits(std::vector<SUnit> &SUs)
				: SUnits(SUs), Stack(), Blocked(SUs.size()), B(SUs.size()),
				AdjK(SUs.size()) {}
			/// Reset the data structures used in the circuit algorithm.
			void reset() {
				Stack.clear();
				Blocked.reset();
				B.assign(SUnits.size(), SmallPtrSet<SUnit *, 4>());
				NumPaths = 0;
			}
			void createAdjacencyStructure(SwingSchedulerDAG *DAG);
			bool circuit(int V, int S, NodeSetType &NodeSets, bool HasBackedge = false);
			void unblock(int U);

		};

		
	private:
		const RegisterClassInfo &RegClassInfo;
	public:
		SwingSchedulerDAG(DSPSWLoops &P, MachineLoop *L, const RegisterClassInfo &rci, LiveIntervals &LIV)
			:ScheduleDAGInstrs(*P.MF, *P.MLI, *P.MDT, false,false, &LIV), pass(P), MII(0), isScheduled(false),
			L(*L), Topo(SUnits, &ExitSU), RegClassInfo(rci),LIS(LIV){
			TII = P.MF->getTarget().getInstrInfo();
		}

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

		/// Return true if the dependence is a back-edge in the data dependence graph.
		/// Since the DAG doesn't contain cycles, we represent a cycle in the graph
		/// using an anti dependence from a Phi to an instruction.
		bool isBackedge(SUnit *Source, const SDep &Dep) {
			if (Dep.getKind() != SDep::Anti)
				return false;
			return Source->getInstr()->isPHI() || Dep.getSUnit()->getInstr()->isPHI();
		}

		/// The latency of the dependence.
		unsigned getLatency(SUnit *Source, const SDep &Dep) {
			// Anti dependences represent recurrences, so use the latency of the
			// instruction on the back-edge.
			if (Dep.getKind() == SDep::Anti) {
				if (Source->getInstr()->isPHI())
					return Dep.getSUnit()->Latency;
				if (Dep.getSUnit()->getInstr()->isPHI())
					return Source->Latency;
				return Dep.getLatency();
			}
			return Dep.getLatency();
		}

		/// The distance function, which indicates that operation V of iteration I
		/// depends on operations U of iteration I-distance.
		unsigned getDistance(SUnit *U, SUnit *V, const SDep &Dep) {
			// Instructions that feed a Phi have a distance of 1. Computing larger
			// values for arrays requires data dependence information.
			if (V->getInstr()->isPHI() && Dep.getKind() == SDep::Anti)
				return 1;
			return 0;
		}

		bool isLoopCarriedOrder(SUnit *Source, const SDep &Dep, bool isSucc);

		/// Return true if the dependence is an order dependence between non-Phis.
		static bool isOrder(SUnit *Source, const SDep &Dep) {
			if (Dep.getKind() != SDep::Order)
				return false;
			return (!Source->getInstr()->isPHI() &&
				!Dep.getSUnit()->getInstr()->isPHI());
		}

		MachineInstr *applyInstrChange(MachineInstr *MI, SMSchedule &Schedule,
			bool UpdateDAG = false);

		/// Return the new base register that was stored away for the changed
		/// instruction.
		unsigned getInstrBaseReg(SUnit *SU) {
			DenseMap<SUnit *, std::pair<unsigned, int64_t>>::iterator It =
				InstrChanges.find(SU);
			if (It != InstrChanges.end())
				return It->second.first;
			return 0;
		}
	private:
		void findCircuits(NodeSetType &NodeSets);

		MachineInstr *findDefInLoop(unsigned Reg);

		void updatePhiDependences();

		bool computeDelta(MachineInstr *MI, unsigned &Delta);

		//caculate the resource minimal initial interval
		unsigned calculateResMII(MachineLoop *L);

		//caculate the recursion minimal initial interval
		unsigned calculateRecMII(MachineLoop *L);

		void computeNodeFunctions(NodeSetType &NodeSets);





		void colocateNodeSets(NodeSetType &NodeSets);

		void computeNodeOrder(NodeSetType &NodeSets);

		void swapAntiDependences(std::vector<SUnit> &SUnits);

		bool schedulePipeline(SMSchedule &Schedule);

		void generatePipelinedLoop(SMSchedule &SMS);
	};

	class NodeSet{
	public:
		SetVector<SUnit*> Nodes;
		bool hasRecurrence;
		unsigned RecMII;
		int MaxMov;
		int MaxDepth;
		unsigned Colocate;
		SUnit *ExceedPressure;
	public:
		typedef SetVector<SUnit*>::iterator iterator;
		typedef SetVector<SUnit*>::const_iterator const_iterator;
		NodeSet() :Nodes(), hasRecurrence(false), RecMII(0), MaxMov(0), MaxDepth(0), Colocate(0), ExceedPressure(nullptr){}

		template <typename It>
		NodeSet(It S, It E)
			: Nodes(S, E), hasRecurrence(true), RecMII(0), MaxMov(0), MaxDepth(0),
			Colocate(0), ExceedPressure(nullptr) {}
		unsigned size() const { return Nodes.size(); }

		unsigned count(SUnit *SU) const { return Nodes.count(SU); }

		bool isExceedSU(SUnit *SU) { return ExceedPressure == SU; }

		SUnit *getNode(unsigned i) const { return Nodes[i]; };

		bool insert(SUnit *SU) { return Nodes.insert(SU); }

		void insert(iterator S, iterator E) { Nodes.insert(S, E); }
		void clear() {
			Nodes.clear();
			RecMII = 0;
			hasRecurrence = false;
			MaxMov = 0;
			MaxDepth = 0;
			Colocate = 0;
			ExceedPressure = nullptr;
		}
		iterator begin(){
			return Nodes.begin();
		}
		const_iterator begin() const {
			return Nodes.begin(); 
		}

		iterator end(){
			return Nodes.end();
		}
		const iterator end() const {
			return Nodes.end();
		}
		void print(raw_ostream &os) const {
			os << "Num nodes " << size() << " rec " << RecMII << " mov " << MaxMov
				<< " depth " << MaxDepth << " col " << Colocate << "\n";
			for (iterator I = begin(), E = end(); I != E; ++I)
				os << "   SU(" << (*I)->NodeNum << ") " << *((*I)->getInstr());
			os << "\n";
		}

		void dump() const { print(dbgs()); }
	};

	/// This class repesents the scheduled code.  The main data structure is a
	/// map from scheduled cycle to instructions.  During scheduling, the
	/// data structure explicitly represents all stages/iterations.   When
	/// the algorithm finshes, the schedule is collapsed into a single stage,
	/// which represents instructions from different loop iterations.
	///
	/// The SMS algorithm allows negative values for cycles, so the first cycle
	/// in the schedule is the smallest cycle value.
	class SMSchedule {
	private:
		/// Map from execution cycle to instructions.
		DenseMap<int, std::deque<SUnit *>> ScheduledInstrs;

		/// Map from instruction to execution cycle.
		std::map<SUnit *, int> InstrToCycle;

		/// Map for each register and the max difference between its uses and def.
		/// The first element in the pair is the max difference in stages. The
		/// second is true if the register defines a Phi value and loop value is
		/// scheduled before the Phi.
		std::map<unsigned, std::pair<unsigned, bool>> RegToStageDiff;

		/// Keep track of the first cycle value in the schedule.  It starts
		/// as zero, but the algorithm allows negative values.
		int FirstCycle;

		/// Keep track of the last cycle value in the schedule.
		int LastCycle;

		/// The initiation interval (II) for the schedule.
		int InitiationInterval;

		/// Target machine information.
		const DSPSubtarget &ST;

		/// Virtual register information.
		MachineRegisterInfo &MRI;

		DFAPacketizer *Resources;

	public:
		SMSchedule(MachineFunction *mf)
			: ST(mf->getTarget().getSubtarget<DSPSubtarget>()), MRI(mf->getRegInfo()),
			Resources(ST.getInstrInfo()->CreateTargetScheduleState(&mf->getTarget(),nullptr)) {
			FirstCycle = 0;
			LastCycle = 0;
			InitiationInterval = 0;
		}

		~SMSchedule() {
			ScheduledInstrs.clear();
			InstrToCycle.clear();
			RegToStageDiff.clear();
			delete Resources;
		}

		void reset() {
			ScheduledInstrs.clear();
			InstrToCycle.clear();
			RegToStageDiff.clear();
			FirstCycle = 0;
			LastCycle = 0;
			InitiationInterval = 0;
		}

		/// Set the initiation interval for this schedule.
		void setInitiationInterval(int ii) { InitiationInterval = ii; }

		/// Return the first cycle in the completed schedule.  This
		/// can be a negative value.
		int getFirstCycle() const { return FirstCycle; }

		/// Return the last cycle in the finalized schedule.
		int getFinalCycle() const { return FirstCycle + InitiationInterval - 1; }

		/// Return the cycle of the earliest scheduled instruction in the dependence
		/// chain.
		int earliestCycleInChain(const SDep &Dep);

		/// Return the cycle of the latest scheduled instruction in the dependence
		/// chain.
		int latestCycleInChain(const SDep &Dep);

		void computeStart(SUnit *SU, int *MaxEarlyStart, int *MinLateStart,
			int *MinEnd, int *MaxStart, int II, SwingSchedulerDAG *DAG);

		bool insert(SUnit *SU, int StartCycle, int EndCycle, int II);

		/// Iterators for the cycle to instruction map.
		typedef DenseMap<int, std::deque<SUnit *>>::iterator sched_iterator;
		typedef DenseMap<int, std::deque<SUnit *>>::const_iterator
			const_sched_iterator;

		/// Return true if the instruction is scheduled at the specified stage.
		bool isScheduledAtStage(SUnit *SU, unsigned StageNum) {
			return (stageScheduled(SU) == (int)StageNum);
		}

		/// Return the stage for a scheduled instruction.  Return -1 if
		/// the instruction has not been scheduled.
		int stageScheduled(SUnit *SU) const {
			std::map<SUnit *, int>::const_iterator it = InstrToCycle.find(SU);
			if (it == InstrToCycle.end())
				return -1;
			return (it->second - FirstCycle) / InitiationInterval;
		}

		/// Return the cycle for a scheduled instruction. This function normalizes
		/// the first cycle to be 0.
		unsigned cycleScheduled(SUnit *SU) const {
			std::map<SUnit *, int>::const_iterator it = InstrToCycle.find(SU);
			assert(it != InstrToCycle.end() && "Instruction hasn't been scheduled.");
			return (it->second - FirstCycle) % InitiationInterval;
		}

		/// Return the maximum stage count needed for this schedule.
		unsigned getMaxStageCount() {
			return (LastCycle - FirstCycle) / InitiationInterval;
		}

		/// Return the max. number of stages/iterations that can occur between a
		/// register definition and its uses.
		unsigned getStagesForReg(int Reg, unsigned CurStage) {
			std::pair<unsigned, bool> Stages = RegToStageDiff[Reg];
			if (CurStage > getMaxStageCount() && Stages.first == 0 && Stages.second)
				return 1;
			return Stages.first;
		}

		/// The number of stages for a Phi is a little different than other
		/// instructions. The minimum value computed in RegToStageDiff is 1
		/// because we assume the Phi is needed for at least 1 iteration.
		/// This is not the case if the loop value is scheduled prior to the
		/// Phi in the same stage.  This function returns the number of stages
		/// or iterations needed between the Phi definition and any uses.
		unsigned getStagesForPhi(int Reg) {
			std::pair<unsigned, bool> Stages = RegToStageDiff[Reg];
			if (Stages.second)
				return Stages.first;
			return Stages.first - 1;
		}

		/// Return the instructions that are scheduled at the specified cycle.
		std::deque<SUnit *> &getInstructions(int cycle) {
			return ScheduledInstrs[cycle];
		}

		bool isValidSchedule(SwingSchedulerDAG *SSD);
		void finalizeSchedule(SwingSchedulerDAG *SSD);
		bool orderDependence(SwingSchedulerDAG *SSD, SUnit *SU,
			std::deque<SUnit *> &Insts);
		bool isLoopCarried(SwingSchedulerDAG *SSD, MachineInstr *Phi);
		bool isLoopCarriedDefOfUse(SwingSchedulerDAG *SSD, MachineInstr *Inst,
			MachineOperand &MO);
		void print(raw_ostream &os) const;
		void dump() const;
	};


} // end anonymous namespace

char DSPSWLoops::ID = 0;
unsigned SwingSchedulerDAG::Circuits::MaxPaths = 5;
INITIALIZE_PASS_BEGIN(DSPSWLoops,"DSPSWLoops", "SoftWare Pipeline",false,false)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(DSPSWLoops, "DSPSWLoops", "SoftWare Pipeline",false,false)


//******************************static help function*********************************************
static bool isAluInst(MachineInstr *MI){
	return (MI->getDesc().TSFlags >> DSPII::isAluPos)&DSPII::isAluMask;
}


/// Return the Phi register value that comes the the loop block.
static unsigned getLoopPhiReg(MachineInstr *Phi, MachineBasicBlock *LoopBB) {
	for (unsigned i = 1, e = Phi->getNumOperands(); i != e; i += 2)
	if (Phi->getOperand(i + 1).getMBB() == LoopBB)
		return Phi->getOperand(i).getReg();
	return 0;
}
static bool isSlot0_Mov(MachineInstr *MI){
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

static bool isSlot01_Mov(MachineInstr *MI){
	switch (MI->getOpcode())
	{
	default:return false;
		break;
	}
}

/// Return the register values for  the operands of a Phi instruction.
/// This function assume the instruction is a Phi.
static void getPhiRegs(MachineInstr *Phi, MachineBasicBlock *Loop,
	unsigned &InitVal, unsigned &LoopVal) {
	assert(Phi->isPHI() && "Expecting a Phi.");

	InitVal = 0;
	LoopVal = 0;
	for (unsigned i = 1, e = Phi->getNumOperands(); i != e; i += 2)
	if (Phi->getOperand(i + 1).getMBB() != Loop)
		InitVal = Phi->getOperand(i).getReg();
	else if (Phi->getOperand(i + 1).getMBB() == Loop)
		LoopVal = Phi->getOperand(i).getReg();

	assert(InitVal != 0 && LoopVal != 0 && "Unexpected Phi structure.");
}

/// Return true for DAG nodes that we ignore when computing the cost functions.
/// We ignore the back-edge recurrence in order to avoid unbounded recurison
/// in the calculation of the ASAP, ALAP, etc functions.
static bool ignoreDependence(const SDep &D, bool isPred) {
	if (D.isArtificial())
		return true;
	return D.getKind() == SDep::Anti && isPred;
}

/// Compute the Pred_L(O) set, as defined in the paper. The set is defined
/// as the predecessors of the elements of NodeOrder that are not also in
/// NodeOrder.
static bool pred_L(SetVector<SUnit *> &NodeOrder,
	SmallSetVector<SUnit *, 8> &Preds,
	const NodeSet *S = nullptr) {
	Preds.clear();
	for (SetVector<SUnit *>::iterator I = NodeOrder.begin(), E = NodeOrder.end();
		I != E; ++I) {
		for (SUnit::pred_iterator PI = (*I)->Preds.begin(), PE = (*I)->Preds.end();
			PI != PE; ++PI) {
			if (S && S->count(PI->getSUnit()) == 0)
				continue;
			if (ignoreDependence(*PI, true))
				continue;
			if (NodeOrder.count(PI->getSUnit()) == 0)
				Preds.insert(PI->getSUnit());
		}
		// Back-edges are predecessors with an anti-dependence.
		for (SUnit::const_succ_iterator IS = (*I)->Succs.begin(),
			ES = (*I)->Succs.end();
			IS != ES; ++IS) {
			if (IS->getKind() != SDep::Anti)
				continue;
			if (S && S->count(IS->getSUnit()) == 0)
				continue;
			if (NodeOrder.count(IS->getSUnit()) == 0)
				Preds.insert(IS->getSUnit());
		}
	}
	return Preds.size() > 0;
}

/// Compute the Succ_L(O) set, as defined in the paper. The set is defined
/// as the successors of the elements of NodeOrder that are not also in
/// NodeOrder.
static bool succ_L(SetVector<SUnit *> &NodeOrder,
	SmallSetVector<SUnit *, 8> &Succs,
	const NodeSet *S = nullptr) {
	Succs.clear();
	for (SetVector<SUnit *>::iterator I = NodeOrder.begin(), E = NodeOrder.end();
		I != E; ++I) {
		for (SUnit::succ_iterator SI = (*I)->Succs.begin(), SE = (*I)->Succs.end();
			SI != SE; ++SI) {
			if (S && S->count(SI->getSUnit()) == 0)
				continue;
			if (ignoreDependence(*SI, false))
				continue;
			if (NodeOrder.count(SI->getSUnit()) == 0)
				Succs.insert(SI->getSUnit());
		}
		for (SUnit::const_pred_iterator PI = (*I)->Preds.begin(),
			PE = (*I)->Preds.end();
			PI != PE; ++PI) {
			if (PI->getKind() != SDep::Anti)
				continue;
			if (S && S->count(PI->getSUnit()) == 0)
				continue;
			if (NodeOrder.count(PI->getSUnit()) == 0)
				Succs.insert(PI->getSUnit());
		}
	}
	return Succs.size() > 0;
}

/// Return true if Set1 is a subset of Set2.
template <class S1Ty, class S2Ty> 
static bool isSubset(S1Ty &Set1, S2Ty &Set2) {
	for (typename S1Ty::iterator I = Set1.begin(), E = Set1.end(); I != E; ++I){
		if (Set2.count(*I) == 0)
			return false;
	}		
	return true;
}

/// Return true if Set1 contains elements in Set2. The elements in common
/// are returned in a different container.
static bool isIntersect(SmallSetVector<SUnit *, 8> &Set1, const NodeSet &Set2,
	SmallSetVector<SUnit *, 8> &Result) {
	Result.clear();
	for (unsigned i = 0, e = Set1.size(); i != e; ++i) {
		SUnit *SU = Set1[i];
		if (Set2.count(SU) != 0)
			Result.insert(SU);
	}
	return !Result.empty();
}

/// Return true if Inst1 defines a value that is used in Inst2.
static bool hasDataDependence(SUnit *Inst1, SUnit *Inst2) {
	for (auto &SI : Inst1->Succs){
		if (SI.getSUnit() == Inst2 && SI.getKind() == SDep::Data)
			return true;
	}
	return false;
}

/// If an instruction has a use that spans multiple iterations, then
/// return true. These instructions are characterized by having a back-ege
/// to a Phi, which contains a reference to another Phi.
static SUnit *multipleIterations(SUnit *SU, SwingSchedulerDAG *DAG) {
	for (auto &P : SU->Preds)
	if (DAG->isBackedge(SU, P) && P.getSUnit()->getInstr()->isPHI())
	for (auto &S : P.getSUnit()->Succs)
	if (S.getKind() == SDep::Order && S.getSUnit()->getInstr()->isPHI())
		return P.getSUnit();
	return nullptr;
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
unsigned SwingSchedulerDAG::calculateResMII(MachineLoop *L){
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




unsigned SwingSchedulerDAG::calculateRecMII(MachineLoop *L){
	MachineBasicBlock *LoopBody = L->getBlocks()[0];
	for (auto I = LoopBody->begin(),E = LoopBody->end(); I != E; I++)
	{
		if (!I->isPHI())
		{
			continue;
		}
		//getPhiCycles(Phi, Phi, tr, C);
	}
	return 2;
}

bool DSPSWLoops::swingModuloScheduler(MachineLoop *L){
	if (L->getBlocks().size() != 1) llvm_unreachable("SMS works on single block only");
	SwingSchedulerDAG SMS(*this, L, RegClassInfo, getAnalysis<LiveIntervals>());
	MachineBasicBlock *MBB = L->getBlocks()[0];
	DEBUG(dbgs() << "**********start build scheduler for " << MBB->getName() << "block" << "\n");
	SMS.startBlock(MBB);
	unsigned size = MBB->size();
	unsigned size2 = 0;
	for (MachineBasicBlock::iterator I = MBB->getFirstTerminator(),
		E = MBB->instr_end();
		I != E; ++I, --size);

	for (MachineBasicBlock::iterator i = MBB->getFirstNonPHI(),e = MBB->getFirstTerminator(); i !=e; i++)
	{
		size2++;
	}
	std::cout << "size	" << size<< std::endl;
	std::cout << "size2	" << size2 << std::endl;
	SMS.enterRegion(MBB, MBB->getFirstNonPHI(), MBB->getFirstTerminator(), size2);
	//SMS.enterRegion(MBB, MBB->begin(), MBB->getFirstTerminator(), size);
	SMS.schedule();
	SMS.exitRegion();
	SMS.finishBlock();
	return true;
}

void SwingSchedulerDAG::schedule(){
	AliasAnalysis *AA = pass.AA;
	buildSchedGraph(AA);
	Topo.InitDAGTopologicalSorting();
	NodeSetType NodeSets;
	findCircuits(NodeSets);

	unsigned ResMII = calculateResMII(&L);
	std::cout << "resMII" << ResMII << std::endl;

	MII = ResMII;
	NodeSet AllSet;

	assert(MII != 0 && "MII cannot be ZERO");
	for (int i = 0; i < SUnits.size(); i++){
		AllSet.insert(&SUnits[i]);
	}
	NodeSets.push_back(AllSet);
	computeNodeFunctions(NodeSets);

	DEBUG({
		for (auto &I : NodeSets) {
			dbgs() << "  NodeSet ";
			I.dump();
		}
	});

	computeNodeOrder(NodeSets);

	SMSchedule Schedule(pass.MF);
	bool Scheduled = schedulePipeline(Schedule);
}

//create the adjacency structure of the nodes in the graph
void SwingSchedulerDAG::Circuits::createAdjacencyStructure(SwingSchedulerDAG *DAG){
	BitVector Added(SUnits.size());
	for (int i = 0, e = SUnits.size(); i != e; i++){
		Added.reset();
		// Add any successor to the adjacency matrix and exclude duplicates.
		for each (auto &SI in SUnits[i].Succs)
		{
			if (SI.getKind() == SDep::Anti&&!SI.getSUnit()->getInstr()->isPHI())
				continue;
			if (SI.getSUnit()->getInstr()->isTerminator())
				continue;
			// N represent position in SUnits
			int N = SI.getSUnit()->NodeNum;
			if (!Added.test(N))
			{
				AdjK[i].push_back(N);
				Added.set(N);
			}
		}

		// A chain edge between a store and a load is treated as a back-edge in the
		// adjacency matrix.
		for (auto &PI : SUnits[i].Preds) {
			if (!SUnits[i].getInstr()->mayStore() ||
				!DAG->isLoopCarriedOrder(&SUnits[i], PI, false))
				continue;
			if (PI.getKind() == SDep::Order && PI.getSUnit()->getInstr()->mayLoad()&&PI.getSUnit()->getInstr()->isTerminator()) {
				int N = PI.getSUnit()->NodeNum;
				if (!Added.test(N)) {
					AdjK[i].push_back(N);
					Added.set(N);
				}
			}
		}
	}
}

/// Identify an elementary circuit in the dependence graph.
bool SwingSchedulerDAG::Circuits::circuit(int V, int S, NodeSetType &NodeSets,bool HasBackedge){
	SUnit *SV = &SUnits[V];
	bool F = false;
	Stack.insert(SV);
	Blocked.set(V);

	for (auto W : AdjK[V]) {
		if (NumPaths > MaxPaths)
			break;
		if (W < S)
			continue;
		if (W == S) {
			if (!HasBackedge)
				NodeSets.push_back(NodeSet(Stack.begin(), Stack.end()));
			F = true;
			++NumPaths;
			break;
		}
		else if (!Blocked.test(W)) {
			if (circuit(W, S, NodeSets, W < V ? true : HasBackedge))
				F = true;
		}
	}

	if (F)
		unblock(V);
	else {
		for (auto W : AdjK[V]) {
			if (W < S)
				continue;
			if (B[W].count(SV) == 0)
				B[W].insert(SV);
		}
	}
	Stack.pop_back();
	return F;
}

void SwingSchedulerDAG::Circuits::unblock(int U){
	Blocked.reset(U);
	SmallPtrSet<SUnit *, 4> &BU = B[U];
	while (!BU.empty()) {
		SmallPtrSet<SUnit *, 4>::iterator SI = BU.begin();
		assert(SI != BU.end() && "Invalid B set.");
		SUnit *W = *SI;
		BU.erase(W);
		if (Blocked.test(W->NodeNum))
			unblock(W->NodeNum);
	}
}

/// Apply changes to the instruction if needed. The changes are need
/// to improve the scheduling and depend up on the final schedule.
MachineInstr *SwingSchedulerDAG::applyInstrChange(MachineInstr *MI,
	SMSchedule &Schedule,
	bool UpdateDAG) {
	SUnit *SU = getSUnit(MI);
	DenseMap<SUnit *, std::pair<unsigned, int64_t>>::iterator It =
		InstrChanges.find(SU);
	if (It != InstrChanges.end()) {
		std::pair<unsigned, int64_t> RegAndOffset = It->second;
		unsigned BasePos, OffsetPos;
		if (!TII->getBaseAndOffsetPosition(MI, BasePos, OffsetPos))
			return nullptr;
		unsigned BaseReg = MI->getOperand(BasePos).getReg();
		MachineInstr *LoopDef = findDefInLoop(BaseReg);
		int DefStageNum = Schedule.stageScheduled(getSUnit(LoopDef));
		int DefCycleNum = Schedule.cycleScheduled(getSUnit(LoopDef));
		int BaseStageNum = Schedule.stageScheduled(SU);
		int BaseCycleNum = Schedule.cycleScheduled(SU);
		if (BaseStageNum < DefStageNum) {
			MachineInstr *NewMI = MF.CloneMachineInstr(MI);
			int OffsetDiff = DefStageNum - BaseStageNum;
			if (DefCycleNum < BaseCycleNum) {
				NewMI->getOperand(BasePos).setReg(RegAndOffset.first);
				if (OffsetDiff > 0)
					--OffsetDiff;
			}
			int64_t NewOffset =
				MI->getOperand(OffsetPos).getImm() + RegAndOffset.second * OffsetDiff;
			NewMI->getOperand(OffsetPos).setImm(NewOffset);
			if (UpdateDAG) {
				SU->setInstr(NewMI);
				MISUnitMap[NewMI] = SU;
			}
			NewMIs.insert(NewMI);
			return NewMI;
		}
	}
	return nullptr;
}
/// Return the instruction in the loop that defines the register.
/// If the definition is a Phi, then follow the Phi operand to
/// the instruction in the loop.
MachineInstr *SwingSchedulerDAG::findDefInLoop(unsigned Reg) {
	SmallPtrSet<MachineInstr *, 8> Visited;
	MachineInstr *Def = MRI.getVRegDef(Reg);
	while (Def->isPHI()) {
		if (!Visited.insert(Def))
			break;
		for (unsigned i = 1, e = Def->getNumOperands(); i < e; i += 2)
		if (Def->getOperand(i + 1).getMBB() == BB) {
			Def = MRI.getVRegDef(Def->getOperand(i).getReg());
			break;
		}
	}
	return Def;
}

/// Order the instructions within a cycle so that the definitions occur
/// before the uses. Returns true if the instruction is added to the start
/// of the list, or false if added to the end.
bool SMSchedule::orderDependence(SwingSchedulerDAG *SSD, SUnit *SU,
	std::deque<SUnit *> &Insts) {
	MachineInstr *MI = SU->getInstr();
	bool OrderBeforeUse = false;
	bool OrderAfterDef = false;
	bool OrderBeforeDef = false;
	unsigned MoveDef = 0;
	unsigned MoveUse = 0;
	int StageInst1 = stageScheduled(SU);

	unsigned Pos = 0;
	for (std::deque<SUnit *>::iterator I = Insts.begin(), E = Insts.end(); I != E;
		++I, ++Pos) {
		// Relative order of Phis does not matter.
		if (MI->isPHI() && (*I)->getInstr()->isPHI())
			continue;
		for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i) {
			MachineOperand &MO = MI->getOperand(i);
			if (!MO.isReg() || !TargetRegisterInfo::isVirtualRegister(MO.getReg()))
				continue;
			unsigned Reg = MO.getReg();
			unsigned BasePos, OffsetPos;
			if (ST.getInstrInfo()->getBaseAndOffsetPosition(MI, BasePos, OffsetPos))
			if (MI->getOperand(BasePos).getReg() == Reg)
			if (unsigned NewReg = SSD->getInstrBaseReg(SU))
				Reg = NewReg;
			bool Reads, Writes;
			std::tie(Reads, Writes) =
				(*I)->getInstr()->readsWritesVirtualRegister(Reg);
			if (MO.isDef() && Reads && stageScheduled(*I) <= StageInst1) {
				OrderBeforeUse = true;
				MoveUse = Pos;
			}
			else if (MO.isDef() && Reads && stageScheduled(*I) > StageInst1) {
				// Add the instruction after the scheduled instruction.
				OrderAfterDef = true;
				MoveDef = Pos;
			}
			else if (MO.isUse() && Writes && stageScheduled(*I) == StageInst1) {
				if (cycleScheduled(*I) == cycleScheduled(SU) && !(*I)->isSucc(SU)) {
					OrderBeforeUse = true;
					MoveUse = Pos;
				}
				else {
					OrderAfterDef = true;
					MoveDef = Pos;
				}
			}
			else if (MO.isUse() && Writes && stageScheduled(*I) > StageInst1) {
				OrderBeforeUse = true;
				MoveUse = Pos;
				if (MoveUse != 0) {
					OrderAfterDef = true;
					MoveDef = Pos - 1;
				}
			}
			else if (MO.isUse() && Writes && stageScheduled(*I) < StageInst1) {
				// Add the instruction before the scheduled instruction.
				OrderBeforeUse = true;
				MoveUse = Pos;
			}
			else if (MO.isUse() && stageScheduled(*I) == StageInst1 &&
				isLoopCarriedDefOfUse(SSD, (*I)->getInstr(), MO)) {
				OrderBeforeDef = true;
				MoveUse = Pos;
			}
		}
		// Check for order dependences between instructions. Make sure the source
		// is ordered before the destination.
		for (auto &S : SU->Succs)
		if (S.getKind() == SDep::Order) {
			if (S.getSUnit() == *I && stageScheduled(*I) == StageInst1) {
				OrderBeforeUse = true;
				MoveUse = Pos;
			}
		}
		else if (TargetRegisterInfo::isPhysicalRegister(S.getReg())) {
			if (cycleScheduled(SU) != cycleScheduled(S.getSUnit())) {
				if (S.isAssignedRegDep()) {
					OrderAfterDef = true;
					MoveDef = Pos;
				}
			}
			else {
				OrderBeforeUse = true;
				MoveUse = Pos;
			}
		}
		for (auto &P : SU->Preds)
		if (P.getKind() == SDep::Order) {
			if (P.getSUnit() == *I && stageScheduled(*I) == StageInst1) {
				OrderAfterDef = true;
				MoveDef = Pos;
			}
		}
		else if (TargetRegisterInfo::isPhysicalRegister(P.getReg())) {
			if (cycleScheduled(SU) != cycleScheduled(P.getSUnit())) {
				if (P.isAssignedRegDep()) {
					OrderBeforeUse = true;
					MoveUse = Pos;
				}
			}
			else {
				OrderAfterDef = true;
				MoveDef = Pos;
			}
		}
	}

	// A circular dependence.
	if (OrderAfterDef && OrderBeforeUse && MoveUse == MoveDef)
		OrderBeforeUse = false;

	// OrderAfterDef takes precedences over OrderBeforeDef. The latter is due
	// to a loop-carried dependence.
	if (OrderBeforeDef)
		OrderBeforeUse = !OrderAfterDef || (MoveUse > MoveDef);

	// The uncommon case when the instruction order needs to be updated because
	// there is both a use and def.
	if (OrderBeforeUse && OrderAfterDef) {
		SUnit *UseSU = Insts.at(MoveUse);
		SUnit *DefSU = Insts.at(MoveDef);
		if (MoveUse > MoveDef) {
			Insts.erase(Insts.begin() + MoveUse);
			Insts.erase(Insts.begin() + MoveDef);
		}
		else {
			Insts.erase(Insts.begin() + MoveDef);
			Insts.erase(Insts.begin() + MoveUse);
		}
		if (orderDependence(SSD, UseSU, Insts)) {
			Insts.push_front(SU);
			orderDependence(SSD, DefSU, Insts);
			return true;
		}
		Insts.pop_back();
		Insts.push_back(SU);
		Insts.push_back(UseSU);
		orderDependence(SSD, DefSU, Insts);
		return false;
	}
	// Put the new instruction first if there is a use in the list. Otherwise,
	// put it at the end of the list.
	if (OrderBeforeUse)
		Insts.push_front(SU);
	else
		Insts.push_back(SU);
	return OrderBeforeUse;
}
/// Return true for an order dependence that is loop carried potentially.
/// An order dependence is loop carried if the destination defines a value
/// that may be used by the source in a subsequent iteration.
bool SwingSchedulerDAG::isLoopCarriedOrder(SUnit *Source, const SDep &Dep,
	bool isSucc) {
	if (!isOrder(Source, Dep) || Dep.isArtificial())
		return false;

	//if (!SwpPruneLoopCarried)
		//return true;

	MachineInstr *SI = Source->getInstr();
	MachineInstr *DI = Dep.getSUnit()->getInstr();
	if (!isSucc)
		std::swap(SI, DI);
	assert(SI != nullptr && DI != nullptr && "Expecting SUnit with an MI.");

	// Assume ordered loads and stores may have a loop carried dependence.
	if (SI->hasUnmodeledSideEffects() || DI->hasUnmodeledSideEffects() ||
		SI->hasOrderedMemoryRef() || DI->hasOrderedMemoryRef())
		return true;

	// Only chain dependences between a load and store can be loop carried.
	if (!DI->mayStore() || !SI->mayLoad())
		return false;

	unsigned DeltaS, DeltaD;
	if (!computeDelta(SI, DeltaS) || !computeDelta(DI, DeltaD))
		return true;

	unsigned BaseRegS, OffsetS, BaseRegD, OffsetD;
	const TargetRegisterInfo *TRI = MF.getTarget().getRegisterInfo();
	if (!TII->getLdStBaseRegImmOfs(SI, BaseRegS, OffsetS, TRI) ||
		!TII->getLdStBaseRegImmOfs(DI, BaseRegD, OffsetD, TRI))
		return true;

	if (BaseRegS != BaseRegD)
		return true;

	uint64_t AccessSizeS = (*SI->memoperands_begin())->getSize();
	uint64_t AccessSizeD = (*DI->memoperands_begin())->getSize();

	// This is the main test, which checks the offset values and the loop
	// increment value to determine if the accesses may be loop carried.
	if (OffsetS >= OffsetD)
		return OffsetS + AccessSizeS > DeltaS;
	else if (OffsetS < OffsetD)
		return OffsetD + AccessSizeD > DeltaD;

	return true;
}

/// Return true if we can compute the amount the instruction changes
/// during each iteration. Set Delta to the amount of the change.
bool SwingSchedulerDAG::computeDelta(MachineInstr *MI, unsigned &Delta) {
	const TargetRegisterInfo *TRI = MF.getTarget().getRegisterInfo();
	unsigned BaseReg, Offset;
	if (!TII->getLdStBaseRegImmOfs(MI, BaseReg, Offset, TRI))
		return false;

	MachineRegisterInfo &MRI = MF.getRegInfo();
	// Check if there is a Phi. If so, get the definition in the loop.
	MachineInstr *BaseDef = MRI.getVRegDef(BaseReg);
	if (BaseDef && BaseDef->isPHI()) {
		BaseReg = getLoopPhiReg(BaseDef, MI->getParent());
		BaseDef = MRI.getVRegDef(BaseReg);
	}
	if (!BaseDef)
		return false;

	int D;
	if (!TII->getIncrementValue(BaseDef, D) || D < 0)
		return false;

	Delta = D;
	return true;
}

void SwingSchedulerDAG::swapAntiDependences(std::vector<SUnit> &SUnits){
	SmallVector<std::pair<SUnit *, SDep>, 8> DepsAdded;
	for (unsigned i = 0, e = SUnits.size(); i != e; ++i) {
		SUnit *SU = &SUnits[i];
		for (SUnit::pred_iterator IP = SU->Preds.begin(), EP = SU->Preds.end();
			IP != EP; ++IP) {
			if (IP->getKind() != SDep::Anti)
				continue;
			DepsAdded.push_back(std::make_pair(SU, *IP));
		}
	}
	for (SmallVector<std::pair<SUnit *, SDep>, 8>::iterator I = DepsAdded.begin(),
		E = DepsAdded.end();
		I != E; ++I) {
		// Remove this anti dependency and add one in the reverse direction.
		SUnit *SU = I->first;
		SDep &D = I->second;
		SUnit *TargetSU = D.getSUnit();
		unsigned Reg = D.getReg();
		unsigned Lat = D.getLatency();
		SU->removePred(D);
		SDep Dep(SU, SDep::Anti, Reg);
		Dep.setLatency(Lat);
		TargetSU->addPred(Dep);
	}
}
void  SwingSchedulerDAG::findCircuits(NodeSetType &NodeSets){

	// Swap all the anti dependences in the DAG. That means it is no longer a DAG,
	// but we do this to find the circuits, and then change them back.
	swapAntiDependences(SUnits);
	Circuits Cir(SUnits);

	Cir.createAdjacencyStructure(this);

	for (int i = 0, e = SUnits.size(); i != e; ++i) {
		Cir.reset();
		Cir.circuit(i, i, NodeSets,false);
	}
	swapAntiDependences(SUnits);      
}




/// Update the phi dependences to the DAG because ScheduleDAGInstrs no longer
/// processes dependences for PHIs. This function adds true dependences
/// from a PHI to a use, and a loop carried dependence from the use to the
/// PHI. The loop carried dependence is represented as an anti dependence
/// edge. This function also removes chain dependences between unrelated
/// PHIs.
void SwingSchedulerDAG::updatePhiDependences(){

}

void SwingSchedulerDAG::computeNodeFunctions(NodeSetType &NodeSets){
	ScheduleInfo.resize(SUnits.size());

	DEBUG({
		for (ScheduleDAGTopologicalSort::const_iterator I = Topo.begin(),
		E = Topo.end();
		I != E; ++I) {
			SUnit *SU = &SUnits[*I];
			SU->dump(this);
		}
	});

	int maxASAP = 0;
	// Compute ASAP.
	for (ScheduleDAGTopologicalSort::const_iterator I = Topo.begin(),
		E = Topo.end();
		I != E; ++I) {
		int asap = 0;
		SUnit *SU = &SUnits[*I];
		for (SUnit::const_pred_iterator IP = SU->Preds.begin(),
			EP = SU->Preds.end();
			IP != EP; ++IP) {
			if (ignoreDependence(*IP, true))
				continue;
			SUnit *pred = IP->getSUnit();
			// see paper 4.1
			asap = std::max(asap, (int)(getASAP(pred) + getLatency(SU, *IP) -
				getDistance(pred, SU, *IP) * MII));
		}
		maxASAP = std::max(maxASAP, asap);
		ScheduleInfo[*I].ASAP = asap;
	}
	std::cout << "ASAP " << std::endl;
	//compute ALAP
	for (ScheduleDAGTopologicalSort::const_reverse_iterator I = Topo.rbegin(),
		E = Topo.rend(); I != E; ++I)
	{
		int alap =maxASAP;
		SUnit *SU = &SUnits[*I];
		for (SUnit::const_succ_iterator IS = SU->Succs.begin(), ES = SU->Succs.end(); IS != ES; ++IS)
		{
			if (ignoreDependence(*IS, true))
				continue;
			SUnit *succ = IS->getSUnit();
			if (succ->getInstr()->isTerminator())
				continue;
			std::cout << getLatency(SU, *IS) << std::endl;
			alap = std::min(alap, (int)(getALAP(succ) - getLatency(SU, *IS) + getDistance(succ, SU, *IS)* MII));
		}
		ScheduleInfo[*I].ALAP = alap;
	}



	for (NodeSet &I : NodeSets){
		for (SUnit* su : I){
			I.MaxMov = std::max(I.MaxMov, this->getMOV(su));
			I.MaxDepth = std::max(I.MaxDepth, this->getDepth(su));
		}
	}

	DEBUG({
		for (unsigned i = 0; i < SUnits.size(); i++) {
			dbgs() << "\tNode " << i << ":\n";
			dbgs() << "\t   ASAP = " << getASAP(&SUnits[i]) << "\n";
			dbgs() << "\t   ALAP = " << getALAP(&SUnits[i]) << "\n";
			dbgs() << "\t   MOV  = " << getMOV(&SUnits[i]) << "\n";
			dbgs() << "\t   D    = " << getDepth(&SUnits[i]) << "\n";
			dbgs() << "\t   H    = " << getHeight(&SUnits[i]) << "\n";
		}
	});

}

/// Compute an ordered list of the dependence graph nodes, which
/// indicates the order that the nodes will be scheduled.  This is a
/// two-level algorithm. First, a partial order is created, which
/// consists of a list of sets ordered from highest to lowest priority.
void SwingSchedulerDAG::computeNodeOrder(NodeSetType &NodeSets) {
	SmallSetVector<SUnit *, 8> R;
	NodeOrder.clear();
	std::cout << "size"<< NodeSets.size() << std::endl;
	for (auto &Nodes : NodeSets) {
		DEBUG(dbgs() << "NodeSet size " << Nodes.size() << "\n");
		OrderKind Order;
		SmallSetVector<SUnit *, 8> N;
		if (pred_L(NodeOrder, N) && isSubset(N, Nodes)) {
			R.insert(N.begin(), N.end());
			Order = BottomUp;
			DEBUG(dbgs() << "  Bottom up (preds) ");
		}
		else if (succ_L(NodeOrder, N) && isSubset(N, Nodes)) {
			R.insert(N.begin(), N.end());
			Order = TopDown;
			DEBUG(dbgs() << "  Top down (succs) ");
		}
		else if (isIntersect(N, Nodes, R)) {
			// If some of the successors are in the existing node-set, then use the
			// top-down ordering.
			Order = TopDown;
			DEBUG(dbgs() << "  Top down (intersect) ");
		}
		else if (NodeSets.size() == 1) {
			for (auto &N : Nodes)
			if (N->Succs.size() == 0)
				R.insert(N);
			Order = BottomUp;
			DEBUG(dbgs() << "  Bottom up (all) ");
		}
		else {
			// Find the node with the highest ASAP.
			SUnit *maxASAP = nullptr;
			for (SUnit *SU : Nodes) {
				if (maxASAP == nullptr || getASAP(SU) >= getASAP(maxASAP))
					maxASAP = SU;
			}
			R.insert(maxASAP);
			Order = BottomUp;
			DEBUG(dbgs() << "  Bottom up (default) ");
		}

		while (!R.empty()) {
			if (Order == TopDown) {
				// Choose the node with the maximum height.  If more than one, choose
				// the node with the lowest MOV. If still more than one, check if there
				// is a dependence between the instructions.
				while (!R.empty()) {
					SUnit *maxHeight = nullptr;
					for (SUnit *I : R) {
						if (maxHeight == 0 || getHeight(I) > getHeight(maxHeight))
							maxHeight = I;
						else if (getHeight(I) == getHeight(maxHeight) &&
							getMOV(I) < getMOV(maxHeight) &&
							!hasDataDependence(maxHeight, I))
							maxHeight = I;
						else if (hasDataDependence(I, maxHeight))
							maxHeight = I;
					}
					NodeOrder.insert(maxHeight);
					DEBUG(dbgs() << maxHeight->NodeNum << " ");
					R.remove(maxHeight);
					for (const auto &I : maxHeight->Succs) {
						if (Nodes.count(I.getSUnit()) == 0)
							continue;
						if (NodeOrder.count(I.getSUnit()) != 0)
							continue;
						if (ignoreDependence(I, false))
							continue;
						R.insert(I.getSUnit());
					}
					// Back-edges are predecessors with an anti-dependence.
					for (const auto &I : maxHeight->Preds) {
						if (I.getKind() != SDep::Anti)
							continue;
						if (Nodes.count(I.getSUnit()) == 0)
							continue;
						if (NodeOrder.count(I.getSUnit()) != 0)
							continue;
						R.insert(I.getSUnit());
					}
				}
				Order = BottomUp;
				DEBUG(dbgs() << "\n   Switching order to bottom up ");
				SmallSetVector<SUnit *, 8> N;
				if (pred_L(NodeOrder, N, &Nodes))
					R.insert(N.begin(), N.end());
			}
			else {
				// Choose the node with the maximum depth.  If more than one, choose
				// the node with the lowest MOV. If there is still more than one, check
				// for a dependence between the instructions.
				while (!R.empty()) {
					SUnit *maxDepth = nullptr;
					for (SUnit *I : R) {
						if (maxDepth == 0 || getDepth(I) > getDepth(maxDepth))
							maxDepth = I;
						else if (getDepth(I) == getDepth(maxDepth) &&
							getMOV(I) < getMOV(maxDepth) &&
							!hasDataDependence(I, maxDepth))
							maxDepth = I;
						else if (hasDataDependence(maxDepth, I))
							maxDepth = I;
					}
					NodeOrder.insert(maxDepth);
					DEBUG(dbgs() << maxDepth->NodeNum << " ");
					R.remove(maxDepth);
					if (Nodes.isExceedSU(maxDepth)) {
						Order = TopDown;
						R.clear();
						R.insert(Nodes.getNode(0));
						break;
					}
					for (const auto &I : maxDepth->Preds) {
						if (Nodes.count(I.getSUnit()) == 0)
							continue;
						if (NodeOrder.count(I.getSUnit()) != 0)
							continue;
						if (I.getKind() == SDep::Anti)
							continue;
						R.insert(I.getSUnit());
					}
					// Back-edges are predecessors with an anti-dependence.
					for (const auto &I : maxDepth->Succs) {
						if (I.getKind() != SDep::Anti)
							continue;
						if (Nodes.count(I.getSUnit()) == 0)
							continue;
						if (NodeOrder.count(I.getSUnit()) != 0)
							continue;
						R.insert(I.getSUnit());
					}
				}
				Order = TopDown;
				DEBUG(dbgs() << "\n   Switching order to top down ");
				SmallSetVector<SUnit *, 8> N;
				if (succ_L(NodeOrder, N, &Nodes))
					R.insert(N.begin(), N.end());
			}
		}
		DEBUG(dbgs() << "\nDone with Nodeset\n");
	}

	DEBUG({
		dbgs() << "Node order: ";
		for (SUnit *I : NodeOrder)
			dbgs() << " " << I->NodeNum << " ";
		dbgs() << "\n";
	});
}

/// Process the nodes in the computed order and create the pipelined schedule
/// of the instructions, if possible. Return true if a schedule is found.
bool SwingSchedulerDAG::schedulePipeline(SMSchedule &sms){
	if (NodeOrder.size() == 0)
		return false;

	bool scheduleFound = false;
	for (unsigned II = MII; II < MII + 10 && !scheduleFound; ++II) {
		sms.reset();
		sms.setInitiationInterval(II);

		DEBUG(dbgs() << "Try to schedule with " << II << "\n");

		SetVector<SUnit *>::iterator NI = NodeOrder.begin();
		SetVector<SUnit *>::iterator NE = NodeOrder.end();
		do {
			SUnit *SU = *NI;

			// Compute the schedule time for the instruction, which is based
			// upon the scheduled time for any predecessors/successors.
			int EarlyStart = INT_MIN;
			int LateStart = INT_MAX;
			// These values are set when the size of the schedule window is limited
			// due to chain dependences.
			int SchedEnd = INT_MAX;
			int SchedStart = INT_MIN;
			sms.computeStart(SU, &EarlyStart, &LateStart, &SchedEnd, &SchedStart,
				II, this);
			DEBUG({
				dbgs() << "Inst (" << SU->NodeNum << ") ";
				SU->getInstr()->dump();
				dbgs() << "\n";
			});
			DEBUG({
				dbgs() << "\tes: " << EarlyStart << " ls: " << LateStart
				<< " me: " << SchedEnd << " ms: " << SchedStart << "\n";
			});

			if (EarlyStart > LateStart || SchedEnd < EarlyStart ||
				SchedStart > LateStart)
				scheduleFound = false;
			else if (EarlyStart != INT_MIN && LateStart == INT_MAX) {
				SchedEnd = std::min(SchedEnd, EarlyStart + (int)II - 1);
				scheduleFound = sms.insert(SU, EarlyStart, SchedEnd, II);
			}
			else if (EarlyStart == INT_MIN && LateStart != INT_MAX) {
				SchedStart = std::max(SchedStart, LateStart - (int)II + 1);
				scheduleFound = sms.insert(SU, LateStart, SchedStart, II);
			}
			else if (EarlyStart != INT_MIN && LateStart != INT_MAX) {
				SchedEnd =
					std::min(SchedEnd, std::min(LateStart, EarlyStart + (int)II - 1));
				// When scheduling a Phi it is better to start at the late cycle and go
				// backwards. The default order may insert the Phi too far away from
				// its first dependence.
				if (SU->getInstr()->isPHI())
					scheduleFound = sms.insert(SU, SchedEnd, EarlyStart, II);
				else
					scheduleFound = sms.insert(SU, EarlyStart, SchedEnd, II);
			}
			else {
				int FirstCycle = sms.getFirstCycle();
				scheduleFound = sms.insert(SU, FirstCycle + getASAP(SU),
					FirstCycle + getASAP(SU) + II - 1, II);
			}
			// Even if we find a schedule, make sure the schedule doesn't exceed the
			// allowable number of stages. We keep trying if this happens.
			DEBUG({
				if (!scheduleFound)
				dbgs() << "\tCan't schedule\n";
			});
		} while (++NI != NE && scheduleFound);

		// If a schedule is found, check if it is a valid schedule too.
		if (scheduleFound)
			scheduleFound = sms.isValidSchedule(this);

	DEBUG(dbgs() << "Schedule Found? " << scheduleFound << "\n");

	if (scheduleFound)
		sms.finalizeSchedule(this);
	else
		sms.reset();
	}
	return false;
}
//*********************************************function for find schedule*****************

/// Compute the scheduling start slot for the instruction.  The start slot
/// depends on any predecessor or successor nodes scheduled already.
void SMSchedule::computeStart(SUnit *SU, int *MaxEarlyStart, int *MinLateStart,
	int *MinEnd, int *MaxStart, int II,
	SwingSchedulerDAG *DAG){
	// Iterate over each instruction that has been scheduled already.  The start
	// slot computuation depends on whether the previously scheduled instruction
	// is a predecessor or successor of the specified instruction.
	for (int cycle = getFirstCycle(); cycle <= LastCycle; cycle++)
	{
		for (SUnit *I : getInstructions(cycle)){
			// Because we're processing a DAG for the dependences, we recognize
			// the back-edge in recurrences by anti dependences.
			for (unsigned i = 0, e = (unsigned)SU->Preds.size(); i != e; ++i) {
				const SDep &Dep = SU->Preds[i];
				if (Dep.getSUnit() == I) {
					if (!DAG->isBackedge(SU, Dep)) {
						int EarlyStart = cycle + DAG->getLatency(SU,Dep) -
							DAG->getDistance(Dep.getSUnit(), SU, Dep) * II;
						*MaxEarlyStart = std::max(*MaxEarlyStart, EarlyStart);
						if (DAG->isLoopCarriedOrder(SU, Dep, false)) {
							int End = earliestCycleInChain(Dep) + (II - 1);
							*MinEnd = std::min(*MinEnd, End);
						}
					}
					else {
						int LateStart = cycle - DAG->getLatency(SU, Dep) +
							DAG->getDistance(SU, Dep.getSUnit(), Dep) * II;
						*MinLateStart = std::min(*MinLateStart, LateStart);
					}
				}
				// For instruction that requires multiple iterations, make sure that
				// the dependent instruction is not scheduled past the definition.
				SUnit *BE = multipleIterations(I, DAG);
				if (BE && Dep.getSUnit() == BE && !SU->getInstr()->isPHI() &&
					!SU->isPred(I))
					*MinLateStart = std::min(*MinLateStart, cycle);
			}

			for (unsigned i = 0, e = (unsigned)SU->Succs.size(); i != e; ++i)
			if (SU->Succs[i].getSUnit() == I) {
				const SDep &Dep = SU->Succs[i];
				if (!DAG->isBackedge(SU, Dep)) {
					int LateStart = cycle - DAG->getLatency(SU, Dep) +
						DAG->getDistance(SU, Dep.getSUnit(), Dep) * II;
					*MinLateStart = std::min(*MinLateStart, LateStart);
					if (DAG->isLoopCarriedOrder(SU, Dep,false)) {
						int Start = latestCycleInChain(Dep) + 1 - II;
						*MaxStart = std::max(*MaxStart, Start);
					}
				}
				else {
					int EarlyStart = cycle + DAG->getLatency(SU, Dep) -
						DAG->getDistance(Dep.getSUnit(), SU, Dep) * II;
					*MaxEarlyStart = std::max(*MaxEarlyStart, EarlyStart);
				}
			}
		}
	}
}


// Return the cycle of the earliest scheduled instruction in the chain.
int SMSchedule::earliestCycleInChain(const SDep &Dep) {
	SmallPtrSet<SUnit *, 8> Visited;
	SmallVector<SDep, 8> Worklist;
	Worklist.push_back(Dep);
	int EarlyCycle = INT_MAX;
	while (!Worklist.empty()) {
		const SDep &Cur = Worklist.pop_back_val();
		SUnit *PrevSU = Cur.getSUnit();
		if (Visited.count(PrevSU))
			continue;
		std::map<SUnit *, int>::const_iterator it = InstrToCycle.find(PrevSU);
		if (it == InstrToCycle.end())
			continue;
		EarlyCycle = std::min(EarlyCycle, it->second);
		for (const auto &PI : PrevSU->Preds)
		if (SwingSchedulerDAG::isOrder(PrevSU, PI))
			Worklist.push_back(PI);
		Visited.insert(PrevSU);
	}
	return EarlyCycle;
}

int SMSchedule::latestCycleInChain(const SDep &Dep) {
	SmallPtrSet<SUnit *, 8> Visited;
	SmallVector<SDep, 8> Worklist;
	Worklist.push_back(Dep);
	int LateCycle = INT_MIN;
	while (!Worklist.empty()) {
		const SDep &Cur = Worklist.pop_back_val();
		SUnit *SuccSU = Cur.getSUnit();
		if (Visited.count(SuccSU))
			continue;
		std::map<SUnit *, int>::const_iterator it = InstrToCycle.find(SuccSU);
		if (it == InstrToCycle.end())
			continue;
		LateCycle = std::max(LateCycle, it->second);
		for (const auto &SI : SuccSU->Succs)
		if (SwingSchedulerDAG::isOrder(SuccSU, SI))
			Worklist.push_back(SI);
		Visited.insert(SuccSU);
	}
	return LateCycle;
}

/// Return true if the scheduled Phi has a loop carried operand.
bool SMSchedule::isLoopCarried(SwingSchedulerDAG *SSD, MachineInstr *Phi) {
	if (!Phi->isPHI())
		return false;
	assert(Phi->isPHI() && "Expecing a Phi.");
	SUnit *DefSU = SSD->getSUnit(Phi);
	unsigned DefCycle = cycleScheduled(DefSU);
	int DefStage = stageScheduled(DefSU);

	unsigned InitVal = 0;
	unsigned LoopVal = 0;
	getPhiRegs(Phi, Phi->getParent(), InitVal, LoopVal);
	SUnit *UseSU = SSD->getSUnit(MRI.getVRegDef(LoopVal));
	if (!UseSU)
		return true;
	if (UseSU->getInstr()->isPHI())
		return true;
	unsigned LoopCycle = cycleScheduled(UseSU);
	int LoopStage = stageScheduled(UseSU);
	return LoopCycle > DefCycle ||
		(LoopCycle <= DefCycle && LoopStage <= DefStage);
}

/// Return true if the instruction is a definition that is loop carried
/// and defines the use on the next iteration.
///        v1 = phi(v2, v3)
///  (Def) v3 = op v1
///  (MO)   = v1
/// If MO appears before Def, then then v1 and v3 may get assigned to the same
/// register.
bool SMSchedule::isLoopCarriedDefOfUse(SwingSchedulerDAG *SSD,
	MachineInstr *Def, MachineOperand &MO) {
	if (!MO.isReg())
		return false;
	if (Def->isPHI())
		return false;
	MachineInstr *Phi = MRI.getVRegDef(MO.getReg());
	if (!Phi || !Phi->isPHI() || Phi->getParent() != Def->getParent())
		return false;
	if (!isLoopCarried(SSD, Phi))
		return false;
	unsigned LoopReg = getLoopPhiReg(Phi, Phi->getParent());
	for (unsigned i = 0, e = Def->getNumOperands(); i != e; ++i) {
		MachineOperand &DMO = Def->getOperand(i);
		if (!DMO.isReg() || !DMO.isDef())
			continue;
		if (DMO.getReg() == LoopReg)
			return true;
	}
	return false;
}

// Check if the generated schedule is valid. This function checks if
// an instruction that uses a physical register is scheduled in a
// different stage than the definition. The pipeliner does not handle
// physical register values that may cross a basic block boundary.
bool SMSchedule::isValidSchedule(SwingSchedulerDAG *SSD) {
	const TargetRegisterInfo *TRI = ST.getRegisterInfo();
	for (int i = 0, e = SSD->SUnits.size(); i < e; ++i) {
		SUnit &SU = SSD->SUnits[i];
		if (!SU.hasPhysRegDefs)
			continue;
		int StageDef = stageScheduled(&SU);
		assert(StageDef != -1 && "Instruction should have been scheduled.");
		for (auto &SI : SU.Succs)
		if (SI.isAssignedRegDep())
		if (TRI->isPhysicalRegister(SI.getReg()))
		if (stageScheduled(SI.getSUnit()) != StageDef)
			return false;
	}
	return true;
}

/// After the schedule has been formed, call this function to combine
/// the instructions from the different stages/cycles.  That is, this
/// function creates a schedule that represents a single iteration.
void SMSchedule::finalizeSchedule(SwingSchedulerDAG *SSD) {
	// Move all instructions to the first stage from later stages.
	for (int cycle = getFirstCycle(); cycle <= getFinalCycle(); ++cycle) {
		for (int stage = 1, lastStage = getMaxStageCount(); stage <= lastStage;
			++stage) {
			std::deque<SUnit *> &cycleInstrs =
				ScheduledInstrs[cycle + (stage * InitiationInterval)];
			for (std::deque<SUnit *>::reverse_iterator I = cycleInstrs.rbegin(),
				E = cycleInstrs.rend();
				I != E; ++I)
				ScheduledInstrs[cycle].push_front(*I);
		}
	}
	// Iterate over the definitions in each instruction, and compute the
	// stage difference for each use.  Keep the maximum value.
	for (auto &I : InstrToCycle) {
		int DefStage = stageScheduled(I.first);
		MachineInstr *MI = I.first->getInstr();
		for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i) {
			MachineOperand &Op = MI->getOperand(i);
			if (!Op.isReg() || !Op.isDef())
				continue;

			unsigned Reg = Op.getReg();
			unsigned MaxDiff = 0;
			bool PhiIsSwapped = false;
			for (MachineRegisterInfo::use_iterator UI = MRI.use_begin(Reg),
				EI = MRI.use_end();
				UI != EI; ++UI) {
				MachineOperand &UseOp = *UI;
				MachineInstr *UseMI = UseOp.getParent();
				SUnit *SUnitUse = SSD->getSUnit(UseMI);
				int UseStage = stageScheduled(SUnitUse);
				unsigned Diff = 0;
				if (UseStage != -1 && UseStage >= DefStage)
					Diff = UseStage - DefStage;
				if (MI->isPHI()) {
					if (isLoopCarried(SSD, MI))
						++Diff;
					else
						PhiIsSwapped = true;
				}
				MaxDiff = std::max(Diff, MaxDiff);
			}
			RegToStageDiff[Reg] = std::make_pair(MaxDiff, PhiIsSwapped);
		}
	}

	// Erase all the elements in the later stages. Only one iteration should
	// remain in the scheduled list, and it contains all the instructions.
	for (int cycle = getFinalCycle() + 1; cycle <= LastCycle; ++cycle)
		ScheduledInstrs.erase(cycle);

	// Change the registers in instruction as specified in the InstrChanges
	// map. We need to use the new registers to create the correct order.
	for (int i = 0, e = SSD->SUnits.size(); i != e; ++i) {
		SUnit *SU = &SSD->SUnits[i];
		SSD->applyInstrChange(SU->getInstr(), *this, true);
	}

	// Reorder the instructions in each cycle to fix and improve the
	// generated code.
	for (int Cycle = getFirstCycle(), E = getFinalCycle(); Cycle <= E; ++Cycle) {
		std::deque<SUnit *> &cycleInstrs = ScheduledInstrs[Cycle];
		std::deque<SUnit *> newOrderZC;
		// Put the zero-cost, pseudo instructions at the start of the cycle.
		for (unsigned i = 0, e = cycleInstrs.size(); i < e; ++i) {
			SUnit *SU = cycleInstrs[i];
			if (ST.getInstrInfo()->isZeroCost(SU->getInstr()->getOpcode()))
				orderDependence(SSD, SU, newOrderZC);
		}
		std::deque<SUnit *> newOrderI;
		// Then, add the regular instructions back.
		for (unsigned i = 0, e = cycleInstrs.size(); i < e; ++i) {
			SUnit *SU = cycleInstrs[i];
			if (!ST.getInstrInfo()->isZeroCost(SU->getInstr()->getOpcode()))
				orderDependence(SSD, SU, newOrderI);
		}
		// Replace the old order with the new order.
		cycleInstrs.swap(newOrderZC);
		cycleInstrs.insert(cycleInstrs.end(), newOrderI.begin(), newOrderI.end());
	}

	DEBUG(dump(););
}
/// Try to schedule the node at the specified StartCycle and continue
/// until the node is schedule or the EndCycle is reached.  This function
/// returns true if the node is scheduled.  This routine may search either
/// forward or backward for a place to insert the instruction based upon
/// the relative values of StartCycle and EndCycle.
bool SMSchedule::insert(SUnit *SU, int StartCycle, int EndCycle, int II) {
	bool forward = true;
	if (StartCycle > EndCycle)
		forward = false;

	// The terminating condition depends on the direction.
	int termCycle = forward ? EndCycle + 1 : EndCycle - 1;
	for (int curCycle = StartCycle; curCycle != termCycle;
		forward ? ++curCycle : --curCycle) {

		// Add the already scheduled instructions at the specified cycle to the DFA.
		Resources->clearResources();
		for (int checkCycle = FirstCycle + ((curCycle - FirstCycle) % II);
			checkCycle <= LastCycle; checkCycle += II) {
			std::deque<SUnit *> &cycleInstrs = ScheduledInstrs[checkCycle];

			for (std::deque<SUnit *>::iterator I = cycleInstrs.begin(),
				E = cycleInstrs.end();
				I != E; ++I) {
				if (ST.getInstrInfo()->isZeroCost((*I)->getInstr()->getOpcode()))
					continue;
				assert(Resources->canReserveResources((*I)->getInstr()) &&"These instructions have already been scheduled.");
				Resources->reserveResources((*I)->getInstr());
			}
		}
		if (ST.getInstrInfo()->isZeroCost(SU->getInstr()->getOpcode()) ||
			Resources->canReserveResources(SU->getInstr())) {
			DEBUG({
				dbgs() << "\tinsert at cycle " << curCycle << " ";
				SU->getInstr()->dump();
			});

			ScheduledInstrs[curCycle].push_back(SU);
			InstrToCycle.insert(std::make_pair(SU, curCycle));
			if (curCycle > LastCycle)
				LastCycle = curCycle;
			if (curCycle < FirstCycle)
				FirstCycle = curCycle;
			return true;
		}
		DEBUG({
			dbgs() << "\tfailed to insert at cycle " << curCycle << " ";
			SU->getInstr()->dump();
		});
	}
	return false;
}

/// Print the schedule information to the given output.
void SMSchedule::print(raw_ostream &os) const {
	// Iterate over each cycle.
	for (int cycle = getFirstCycle(); cycle <= getFinalCycle(); ++cycle) {
		// Iterate over each instruction in the cycle.
		const_sched_iterator cycleInstrs = ScheduledInstrs.find(cycle);
		for (SUnit *CI : cycleInstrs->second) {
			os << "cycle " << cycle << " (" << stageScheduled(CI) << ") ";
			os << "(" << CI->NodeNum << ") ";
			CI->getInstr()->print(os);
			os << "\n";
		}
	}
}

/// Utility function used for debugging to print the schedule.
void SMSchedule::dump() const { print(dbgs()); }
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