// Author: Hyungsub Kim
// Email: kim2956@purdue.edu
// Goal: Get a list of functions from *.bc file
// How to execute it?
// (e.g.,) llvm-10.0.0.obj$ opt -load lib/getfunctions.so -getfunctions < PX4_1_11.bc > /dev/null 
// Output: 'getfunctions_outputs.txt' contains a list of all functions

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

using namespace llvm;
using namespace std;

#define DEBUG_TYPE "getfunctions"
long getfunctionsCounter;
std::ofstream OutputFile;

namespace {
  // getfunctions - The first implementation, without getAnalysisUsage.
  struct getfunctions : public FunctionPass {
    static char ID; // Pass identification, replacement for typeid
    getfunctions() : FunctionPass(ID) {OutputFile.open("getfunctions_outputs.txt");}
    ~getfunctions();
      
    bool runOnFunction(Function &F) override {
      ++getfunctionsCounter;
      
      //errs() << getfunctionsCounter << ",";
      //errs().write_escaped(F.getName()) << '\n';
      
      //OutputFile << getfunctionsCounter << ",";
      OutputFile << std::string(F.getName()) << '\n';
                  
      return false;
    }
  };
}

getfunctions::~getfunctions() {
    OutputFile.close();
}
char getfunctions::ID = 0;
static RegisterPass<getfunctions> X("getfunctions", "getfunctions Pass", false, false);
