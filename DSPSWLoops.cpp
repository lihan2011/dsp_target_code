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
#include "llvm/Pass.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/PassSupport.h"

using namespace llvm;


namespace {
	class DSPSWLoops : public MachineFunctionPass{
	public:
		static char ID;
		explicit DSPSWLoops() :MachineFunctionPass(ID){

		}
		bool runOnMachineFunction(MachineFunction &MF) override{
			MLI = &getAnalysis<MachineLoopInfo>();
			MDT = &getAnalysis<MachineDominatorTree>();
		}
		void getAnalysisUsage(AnalysisUsage &AU) const override{
			AU.addRequired<MachineLoopInfo>();
			AU.addRequired<DependenceAnalysis>();
		}
	private:
		const MachineLoopInfo *MLI;
		const MachineDominatorTree *MDT;
		DependenceAnalysis *DA;

		bool canPipelineLoop(MachineLoop *L);
		unsigned caculateResMII();
		unsigned caculateRecMII();
	};

}

bool DSPSWLoops::canPipelineLoop(MachineLoop *L){
	// Check if loop body has no control flow (single BasicBlock)
	unsigned NumBlocks = L->getNumBlocks();
	if (NumBlocks != 1){
		return false;
	}
}