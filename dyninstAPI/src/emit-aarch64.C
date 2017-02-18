/*
 * See the dyninst/COPYRIGHT file for copyright information.
 *
 * We provide the Paradyn Tools (below described as "Paradyn")
 * on an AS IS basis, and do not warrant its validity or performance.
 * We reserve the right to update, modify, or discontinue this
 * software at any time.  We shall have no obligation to supply such
 * updates or modifications or any other form of support to you.
 *
 * By your use of Paradyn, you understand and agree that we (or any
 * other person or entity with proprietary rights in Paradyn) are
 * under no obligation to provide either maintenance services,
 * update services, notices of latent defects, or correction of
 * defects for Paradyn.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <assert.h>
#include <stdio.h>
#include "common/src/Types.h"
#include "dyninstAPI/src/codegen.h"
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/emit-aarch64.h"
#include "dyninstAPI/src/inst-aarch64.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/ast.h"
#include "dyninstAPI/h/BPatch.h"
#include "dyninstAPI/h/BPatch_memoryAccess_NP.h"
#include "dyninstAPI/src/registerSpace.h"

#include "dyninstAPI/src/dynProcess.h"

#include "dyninstAPI/src/binaryEdit.h"
#include "dyninstAPI/src/image.h"
// get_index...
#include "dyninstAPI/src/dynThread.h"
#include "ABI.h"
#include "liveness.h"
#include "RegisterConversion.h"

void emitPushReg64(Register src, codeGen &gen) {
    // See Processos blog Using the Stack in AArch64
    // ... is what the Gogole V8 JavaScript engine uses,
    // and it's also what VIXL's MacroAssembler uses
    // sub sp, x28, #8
    // str x0, [x28, #-8]!
    GET_PTR(insn, gen);
    insnCodeGen::generateSubs(gen, aarch64::x28, aarch64::sp, 8);
    SET_PTR(insn, gen);
    insnCodeGen::generateStoreImm(gen, src, aarch64::x0, -8, false); 
    SET_PTR(insn, gen);
    if (gen.rs()) gen.rs()->incStack(8);
}

void emitPopReg(Register src, codeGen &gen) {
    // ldr x0, [x28], #8 // pop {x0}
    GET_PTR(insn, gen);
    insnCodeGen::generateLoadReg(gen, aarch64::x28, aarch64::x1, aarch64::x0);
    SET_PTR(insn, gen);
}

static Register aarch64_arg_regs[] = { registerSpace::r0, registerSpace::r1, registerSpace::r2, registerSpace::r3, registerSpace::r4, registerSpace::r5, registerSpace::r6, registerSpace::r7  };
#define AARCH64_ARG_REGS (sizeof(aarch64_arg_regs) / sizeof(Register))
Register EmitterAARCH64::emitCall(opCode op, codeGen &gen, const pdvector<AstNodePtr> &operands,
        bool noCost, func_instance *callee)
{
    assert(op == callOp);
    pdvector<Register> srcs;

    bool inInstrumentation = true;
    //  Sanity check for NULL address arg
    if (!callee) {
        char msg[256];
        sprintf(msg, "%s[%d]:  internal error:  emitFuncCall called w/out"
                "callee argument", __FILE__, __LINE__);
        showErrorCallback(80, msg);
        assert(0);
    }

    // Before we generate argument code, save any register that's live across
    // the call.
    pdvector<pair<unsigned,int> > savedRegsToRestore;
    if (inInstrumentation) {
        bitArray regsClobberedByCall = ABI::getABI(8)->getCallWrittenRegisters();
        for (int i = 0; i < gen.rs()->numGPRs(); i++) {
            registerSlot *reg = gen.rs()->GPRs()[i];
            Register r = reg->encoding();
            static LivenessAnalyzer live(8);
            bool callerSave =
                regsClobberedByCall.test(live.getIndex(regToMachReg64.equal_range(r).first->second));
            if (!callerSave) {
                // We don't care!
                regalloc_printf("%s[%d]: pre-call, skipping callee-saved register %d\n", FILE__, __LINE__, reg->number);
            }

            regalloc_printf("%s[%d]: pre-call, register %d has refcount %d, keptValue %d, liveState %s\n",
                    FILE__, __LINE__, reg->number,
                    reg->refCount,
                    reg->keptValue,
                    (reg->liveState == registerSlot::live) ? "live" : ((reg->liveState == registerSlot::spilled) ? "spilled" : "dead"));

            if (reg->refCount > 0 ||  // Currently active
                    reg->keptValue || // Has a kept value
                    (reg->liveState == registerSlot::live)) { // needs to be saved pre-call
                regalloc_printf("%s[%d]: \tsaving reg\n", FILE__, __LINE__);
                pair<unsigned, unsigned> regToSave;
                regToSave.first = reg->number;

                regToSave.second = reg->refCount;
                // We can have both a keptValue and a refCount - so I invert
                // the refCount if there's a keptValue
                if (reg->keptValue)
                    regToSave.second *= -1;

                savedRegsToRestore.push_back(regToSave);

                // The register is live; save it.
                emitPushReg64(reg->encoding(), gen);
                // And now that it's saved, nuke it
                reg->refCount = 0;
                reg->keptValue = false;
            } else {
                if (regsClobberedByCall.test(live.getIndex(regToMachReg64.equal_range(r).first->second))) {
                    gen.markRegDefined(r);
                }
            }
        }
    }

    // generate code for arguments, still copy and pasted from the x86 counter part
    int frame_size = 0;
    for (int u = operands.size() - 1; u >= 0; u--) {
        Address unused = ADDR_NULL;
        unsigned reg = REG_NULL;
        if (u >= (int)AARCH64_ARG_REGS) {
            if (!operands[u]->generateCode_phase2(gen, noCost, unused, reg))
                assert(0);
            assert(reg != REG_NULL);
            emitPushReg64(reg, gen);
            gen.rs()->freeRegister(reg);
            frame_size++;
        } else {
            if (gen.rs()->allocateSpecificRegister(gen, (unsigned) aarch64_arg_regs[u], true))
                reg = aarch64_arg_regs[u];
            else {
                cerr << "Error: tried to allocate register " << aarch64_arg_regs[u] << " and failed!" << endl;
                assert(0);
            }
            gen.markRegDefined(reg);
            if (!operands[u]->generateCode_phase2(gen, noCost, unused, reg))
                assert(0);
            if (reg != aarch64_arg_regs[u]) {
                emitMoveRegToReg(aarch64_arg_regs[u], reg, gen);
            }
        }
    }

    //emitCallInstruction(gen, callee, REG_NULL);

    for (int i = 0; i < operands.size(); i++) {
        if (i == AARCH64_ARG_REGS) break; 

        if (operands[i]->decRefCount())
            gen.rs()->freeRegister(aarch64_arg_regs[i]);
    }
    if (frame_size) {
        
    }

    //if (alignment) {
//
  //  }

    if (!inInstrumentation) return REG_NULL;
    Register ret = gen.rs()->allocateRegister(gen, noCost);
    gen.markRegDefined(ret);
    emitMoveRegToReg(ret, aarch64::x0, gen);

    // Now restore any registers live over the call
    for (int i = savedRegsToRestore.size() - 1; i >= 0; i--) {
        registerSlot *reg = (*gen.rs())[savedRegsToRestore[i].first];
        emitPopReg(reg->encoding(), gen);
    }

    return ret;

}

bool EmitterAARCH64Dyn::emitCallInstruction(codeGen &gen, func_instance *callee, Register r) {
    if (gen.startAddr() != (Address)-1) {
        signed long disp = callee->addr() - (gen.currAddr() + 5);
        int disp_i = (int) disp;
        if (disp == (signed long) disp_i) {
            
            return true;
        }
    }

    pdvector<Register> excluded; // scratch regs are r16-18
    excluded.push_back(gen.rs()->GPR(16));
   

    Register ptr = gen.rs()->getScratchRegister(gen, excluded);

}

bool EmitterAARCH64::emitMoveRegToReg(Register src, Register dest, codeGen &gen) {
    if (dest == src) return true; 

    Register tmp_dest = dest;
    Register tmp_src = src;
    return true;
}

bool EmitterAARCH64::emitMoveRegToReg(registerSlot *source, registerSlot *dest, codeGen &gen) {
    return emitMoveRegToReg(source->encoding(), dest->encoding(), gen);
}

void EmitterAARCH64::emitOp(unsigned opcode, Register dest, Register src1, Register src2, codeGen &gen)
{
    assert(0);
}

void EmitterAARCH64::emitOpImm(unsigned opcode1, unsigned opcode2, Register dest, Register src1, RegValue src2imm,
                          codeGen &gen)
{
    assert(0);
}

void EmitterAARCH64::emitGetParam(Register dest, Register param_num, instPoint::Type pt_type, opCode op, bool addr_of, codeGen &gen)
{
    // Needs to fully convert to arm64
   if (!addr_of && param_num < 6) {
      emitLoadOrigRegister(aarch64_arg_regs[param_num], dest, gen);
      gen.markRegDefined(dest);
      return;
   }
   else if (addr_of && param_num < 6) {
      Register reg = aarch64_arg_regs[param_num];
      gen.markRegDefined(reg);
      //stackItemLocation loc = getHeightOf(stackItem::framebase, gen);
      registerSlot *regSlot = (*gen.rs())[reg];
      assert(regSlot);
      //loc.offset += (regSlot->saveOffset * 8);
      //emitLEA(loc.reg.reg(), REG_NULL, 0, loc.offset, dest, gen);
      return;
   }
   assert(param_num >= 6);
   //stackItemLocation loc = getHeightOf(stackItem::stacktop, gen);
   //if (!gen.bt() || gen.bt()->alignedStack) {
   //   emitMovRMToReg64(dest, loc.reg.reg(), loc.offset, 8, gen);
   //   loc.reg = RealRegister(dest);
   ///   loc.offset = 0;
   //}

   switch (op) {
      case getParamOp:
         if (pt_type == instPoint::FuncEntry) {
            //Return value before any parameters
            //loc.offset += 8;
         }
         break;
      case getParamAtCallOp:
         break;
      case getParamAtEntryOp:
         //loc.offset += 8;
         break;
      default:
         assert(0);
         break;
   }

   //loc.offset += (param_num-6)*8;
   //if (!addr_of)
   //   emitMovRMToReg64(dest, loc.reg.reg(), loc.offset, 8, gen);
   //else
   //   emitLEA(loc.reg.reg(), Null_Register, 0, loc.offset, dest, gen);
}

void EmitterAARCH64::emitStore(Address addr, Register src, int size, codeGen &gen) {
    Register scrach = gen.rs()->getScratchRegister(gen);
    gen.markRegDefined(scrach);

    //emitMovImToReg64(scrach, addr, true, gen);

    //emitMove
}

void EmitterAARCH64::emitLoad(Register dest, Address addr, int val, codeGen &gen) {
   GET_PTR(insn, gen);

}

void EmitterAARCH64::emitLoadOrigRegister(Address register_num, Register destination, codeGen &gen) {
    std::cout << __func__ << std::endl; // added to C+++ in C++11

    registerSlot *src = (*gen.rs())[register_num];
    assert(src);
    registerSlot *dest = (*gen.rs())[destination];
    assert(dest);

    if (register_num == registerSpace::sp) {
        std::cout << "sp is chosen " << std::endl;
        stackItemLocation loc = getHeightOf(stackItem::stacktop, gen);
        if (!gen.bt() || gen.bt()->alignedStack) {
            //emitMovRMToReg64(destination, loc.reg.reg(), loc.offset, 8, gen);
        } else {
            //emitLEA(loc.reg.reg(), Null_Register, 0, loc.offset, destination, gen);
        }
        return;
    }
    return;
}

bool EmitterAARCH64::emitPush(codeGen &gen, Register r) {
    //GET_PTR(insn, gen);
    //*insn++ = 
    //    insn.setBits(0, 7, r);
    //SET_PTR(insn, gen);
    return true;
}


bool shouldSaveReg(registerSlot *reg, baseTramp *inst, bool saveFlags)
{

    if (inst->point()) {
        regalloc_printf("\t shouldSaveReg for BT %p, from 0x%lx\n", inst, inst->point()->insnAddr() );
    } else {
        regalloc_printf("\t shouldSaveReg for iRPC\n");
    }
    if (reg->liveState != registerSlot::live) {
        regalloc_printf("\t Reg %d not live, concluding don't save\n", reg->number);
        return false;
    }
    if (saveFlags) {
    }
    if (inst && inst->validOptimizationInfo() && !inst->definedRegs[reg->encoding()]) {
        regalloc_printf("\t Base tramp instance doesn't have reg %d (num %d) defined; concluding don't save\n",
                reg->encoding(), reg->number);
        return false;
    }
    return true;
}

bool EmitterAARCH64::emitBTSaves(baseTramp* bt, codeGen &gen) {

    gen.setInInstrumentation(true);

    int instFrameSize = 0;

    bool useFPRs =  BPatch::bpatch->isForceSaveFPROn() ||
        ( BPatch::bpatch->isSaveFPROn()      &&
          gen.rs()->anyLiveFPRsAtEntry()     &&
          bt->saveFPRs() &&
          bt->makesCall() );
    useFPRs = false; // for the test_stack_1 test
    bool alignStack = useFPRs || !bt || bt->checkForFuncCalls();
    bool saveFlags = gen.rs()->checkVolatileRegisters(gen, registerSlot::live);
    bool createFrame = !bt || bt->needsFrame() || useFPRs;
    bool saveOrigAddr = createFrame && bt->instP();

    int num_saved = 0;
    int num_to_save = 0;
    // Calculate the number of registers we'll save
    for (int i = 0; i < gen.rs()->numGPRs(); i++) {
        registerSlot *reg = gen.rs()->GPRs()[i];
        if (!shouldSaveReg(reg, bt, saveFlags))
            continue;
        if (createFrame && reg->encoding() == aarch64::FPR)
            continue;
        num_to_save++;
    }

    if (createFrame) {
        num_to_save++;
    }
    if (saveOrigAddr) {
        num_to_save++;
    }
    if (saveFlags) {
        num_to_save++;
    }

    bool skipRedZone = (num_to_save > 0) || alignStack || saveOrigAddr || createFrame;

    if (alignStack) {
        //
    } else if (skipRedZone) {
        // instFrameSize += some_red_zone;
    }

    // Save the live ones
    for (int i = 0; i < gen.rs()->numGPRs(); i++) {
        registerSlot *reg = gen.rs()->GPRs()[i];

        if (!shouldSaveReg(reg, bt, saveFlags))
            continue;
        if (createFrame && reg->encoding() == aarch64::FPR)
            continue;
        //emitPushReg64(reg->encoding(),gen);
        // We move the FP down to just under here, so we're actually
        // measuring _up_ from the FP.
        num_saved++;
        gen.rs()->markSavedRegister(reg->encoding(), num_to_save-num_saved);
    }

    // Save flags if we need to

    // push a return address for stack walking
    if (saveOrigAddr) {
        num_saved++;
    }

    // Push FPR
    if (createFrame) {
        // set up a fresh stack frame
        gen.rs()->markSavedRegister(aarch64::FPR, 0);
        num_saved++;
        
    }

    assert(num_saved == num_to_save);

    // Prepare out stack bookkeeping data structures.
    instFrameSize += num_saved * 8;
    if (bt) {
        bt->stackHeight = instFrameSize;
    }
    gen.rs()->setInstFrameSize(instFrameSize);
    gen.rs()->setStackHeight(0);

    // When using FPRs

    return true;
}

