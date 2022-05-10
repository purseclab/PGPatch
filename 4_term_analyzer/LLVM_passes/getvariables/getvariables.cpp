// Author: Hyungsub Kim
// Email: kim2956@purdue.edu
// Goal: Get a list of variables from *.bc file
// How to execute it?
// (e.g.,) llvm-10.0.0.obj$ opt -load lib/getvariables.so -getvariables < PX4_1_11.bc > /dev/null 
// Output: 'getvariables_outputs.txt' contains a list of all variables

#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/InstIterator.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Pass.h"

#include "llvm/Transforms/IPO/PassManagerBuilder.h"

#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"

#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/LoopInfo.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream> 
#include <set>

using namespace llvm;
using namespace std;

#define DEBUG_TYPE "getvariables"
std::ofstream OutputFile;
long getvariablesCounter = 0;
std::set<std::string> set_variable;

namespace {
// getvariables - The first implementation, without getAnalysisUsage.
struct getvariables: public FunctionPass {
	static char ID; // Pass identification, replacement for typeid
	getvariables() :
			FunctionPass(ID) {
		OutputFile.open("getvariables_outputs.txt");
	}
	~getvariables();

	bool runOnFunction(Function &F) override {

		for (BasicBlock &B : F) {
			for (Instruction &i : B) {

				StringRef variable_name;
				int got_name = 0;

				if (isa < StoreInst > (&i)) {
					auto *store = dyn_cast < StoreInst > (&i);

					for (auto op = i.op_begin(); op != i.op_end(); op++) {
						Value *v = op->get();
						variable_name = v->getName();
					}

					got_name = 1;

				} else if (isa < LoadInst > (&i)) {
					auto *load = dyn_cast < LoadInst > (&i);
					StringRef variable_name;

					for (auto op = i.op_begin(); op != i.op_end(); op++) {
						Value *v = op->get();
						variable_name = v->getName();
					}

					got_name = 1;

				} else if (isa < BitCastInst > (&i)) {
					StringRef variable_name;
					for (auto op = i.op_begin(); op != i.op_end(); op++) {
						Value *v = op->get();
						variable_name = v->getName();
					}
					got_name = 1;

				} else if (isa < GetElementPtrInst > (&i)) {
					auto *gep = dyn_cast < GetElementPtrInst > (&i);
					
					variable_name = gep->getName();
					got_name = 1;
				}

				if (got_name == 1) {
					// To handle SSA
					StringRef v_name;
					std::string str = std::string(variable_name);

					for (int index = 0; index < str.length(); index++) {
						if (isdigit(str[index])) {
							str.erase(index, 1);
							index--;
						}
					}

					set_variable.insert(str);
					
				}

			}
		}
		return false;
	}
};
}

getvariables::~getvariables() {

	std::set<std::string>::iterator itr;
	for (itr = set_variable.begin(); itr != set_variable.end(); itr++) {
		OutputFile << *itr << '\n';
		errs() << getvariablesCounter++ << "," << *itr << "\n";
	}

	OutputFile.close();
}
char getvariables::ID = 0;
static RegisterPass<getvariables> X("getvariables", "getvariables Pass", false,
		false);
