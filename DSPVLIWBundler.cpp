
#include "DSPVLIWBundler.h"
#include "MCTargetDesc/DSPBaseInfo.h"
#include "DSPInstrInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "MCTargetDesc/DSPMCInst.h"
#include "llvm/PassSupport.h"

#include <map>
#include <vector>

using namespace llvm;
namespace llvm {
	//void initializeDSPVLIWBundlerDriverPass(PassRegistry&);
}

namespace {
	class DSPVLIWBundlerDriver : public MachineFunctionPass {
	public:
		//const DSPSubtarget *Subtarget;
	public:
		static char ID;
		TargetMachine &TM;
		DSPVLIWBundler *Bund;
		DSPVLIWBundlerDriver(TargetMachine &tm) :MachineFunctionPass(ID),TM(tm){
			Bund = DSPVLIWBundler::getBundler();
			std::cout << "driver" << std::endl;
			//initializeDSPVLIWBundlerDriverPass(*PassRegistry::getPassRegistry());
		}

		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesCFG();
			MachineFunctionPass::getAnalysisUsage(AU);
		}

		const char *getPassName() const override {
			return "DSP VLIWBundlerDrive";
		}

		bool runOnMachineFunction(MachineFunction &MF) override;
		bool runOnMachineBasicBlock(MachineBasicBlock &MBB);
		unsigned getUnit(const MachineInstr* MI);

	};
	char DSPVLIWBundlerDriver::ID = 2;
}


/*INITIALIZE_PASS_BEGIN(DSPVLIWBundlerDriver, "driver", "DSPVLIW",false,false)
INITIALIZE_AG_DEPENDENCY(AliasAnalysis)
INITIALIZE_PASS_END(DSPVLIWBundlerDriver, "driver", "DSPVLIW",false,false)*/

bool DSPVLIWBundlerDriver::runOnMachineFunction(MachineFunction &MF){
	bool change = true;
	
	for (MachineFunction::iterator MBB = MF.begin(), MBBe = MF.end(); MBB != MBBe; MBB++)
	{
		change = runOnMachineBasicBlock(*MBB);
	}
	return change;
}
static slot  getSlot(DSPVLIWBundler *VB, int FU){
	int avail = VB->usedSlot&FU;
	int index = 0;
	while (avail)
	{
		if (avail & 1)
			break;
		else {
			avail = avail >> 1;
			index++;
		}
	}
	return VB->Slots[index];
}

static bool isSinglePackage(DSPMCInst *MI){
	return MI->isPacketStart() && MI->isPacketEnd();
}

unsigned DSPVLIWBundlerDriver::getUnit(const MachineInstr* MI){
	InstrItineraryData DSPInstrItins = TM.getSubtargetImpl()->getInstrItineraryForCPU("dspse");
	unsigned InsnClass = MI->getDesc().getSchedClass();
	const llvm::InstrStage *IS = DSPInstrItins.beginStage(InsnClass);
	return IS->getUnits();
}

bool DSPVLIWBundlerDriver::runOnMachineBasicBlock(MachineBasicBlock &MBB){
	for (MachineBasicBlock::const_iterator MI = MBB.begin(), MIE = MBB.end(); MI != MIE; MI++)
	{
		if (MI->isBundle()){
			std::vector<const MachineInstr *> package;
			//std::cout << "drive bundled" << std::endl;
			MachineBasicBlock::const_instr_iterator MII = MI;
			++MII;
			unsigned int IgnoreCount = 0;
			while (MII != MIE && MII->isInsideBundle()){
				const MachineInstr *MInst = MII;
				if (MInst->getOpcode() == TargetOpcode::DBG_VALUE ||
					MInst->getOpcode() == TargetOpcode::IMPLICIT_DEF){
					IgnoreCount++;
					++MII;
					continue;
				}
				package.push_back(MInst);
				++MII;
			}
			unsigned size = package.size();
			//sort the package according to the FuncUnit using bubble sort because of the size is small
			for (unsigned int i = 0; i < size; i++)
			{

				for (unsigned j = i; j+1 < size; j++)
				{
					if (getUnit(package[i]) > getUnit(package[j + 1])){
						const MachineInstr * tmp = package[j+1];
						package[j + 1] = package[j];
						package[j] = tmp;
					}
				}
				
			}
			for (int i = 0; i < size; i++)
			{
				if (package[i]->isCFIInstruction())
					continue;
				int FU = getUnit(package[i]);
				slot CurrentSlot = getSlot(Bund, FU);
				int slot =CurrentSlot.SlotMask;
				//std::cout <<std::hex<< "slot" << slot << std::endl;
				int mask = CurrentSlot.mask;
				Bund->usedSlot = Bund->usedSlot&mask;
				Bund->InstrToSlot[package[i]]= slot;
			}
			Bund->reset();
		}
		else{
			int FU = getUnit(MI);
			slot CurrentSlot = getSlot(Bund, FU);
			int slot = CurrentSlot.SlotMask;
			//std::cout << "slot" << slot << std::endl;
			Bund->InstrToSlot[MI] = slot;
			Bund->reset();
		}
			

		

	}
	return true;
}

DSPVLIWBundler* DSPVLIWBundler::UniqueBundler;
DSPVLIWBundler::~DSPVLIWBundler(){
}
DSPVLIWBundler* DSPVLIWBundler::getBundler(){
	if (UniqueBundler==nullptr){
		UniqueBundler = new DSPVLIWBundler();
	}
	return UniqueBundler;
}
void DSPVLIWBundler::anchor(){
	std::cout << "lee" << std::endl;
}

void VLIWBundler::anchor(){
	
}
DSPVLIWBundler::DSPVLIWBundler(){
	usedSlot = 0xf;
	initializeResource();
}

void DSPVLIWBundler::initializeResource(){
	Slots.clear();
	slot slot0 = { 14, 0x9fffffff };
	slot slot1 = { 13, 0xbfffffff};
	slot slot2 = { 11, 0xdfffffff };
	slot slot3 = { 7, 0xffffffff };
	Slots = { slot0, slot1, slot2, slot3 };
}

void DSPVLIWBundler::reset(){
		usedSlot = 0xf;
}



void DSPVLIWBundler::PerformBundle(DSPMCInst *MI,uint32_t* Binary){
	bool isPacketStart = MI->isPacketStart();
	bool isPacketEnd = MI->isPacketEnd();
	const llvm::InstrStage *IS = MI->getIS();
	int resource;
	if (IS == nullptr){
		//std::cout << "is opcode" << MI->getOpcode() << std::endl;
		llvm_unreachable("IS is null");
	}
	else{
		unsigned FuncUnit = IS->getUnits();
		//std::cout << "FU " << FuncUnit << "opcode " << MI->getOpcode() << std::endl;
		uint32_t Flag = 0;
		if (isSinglePackage(MI))
			Flag = DSPVLIW::VLIWEND;
		else if (isPacketEnd)
			Flag = DSPVLIW::VLIWEND;
		else Flag = DSPVLIW::VLIWINSIDE;
		

		uint32_t slots = MI->getPos();
		//std::cout << "MI op" << MI->getOpcode() << std::endl;
		//std::cout << "slot!!" << std::hex << slots <<std::endl;
		//std::cout << "before binary" << std::hex << *Binary<< std::endl;
		(*Binary) = (*Binary)&Flag&slots;
		//::cout << "after binary" << std::hex << *Binary<< std::endl;
	}	
}




//===----------------------------------------------------------------------===//
//                         Public Constructor Functions
//===----------------------------------------------------------------------===//

FunctionPass *llvm::createDSPVLIWBundlerDrive(TargetMachine &TM){
	return new DSPVLIWBundlerDriver(TM);
}