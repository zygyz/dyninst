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
                //emitPushReg64(reg->encoding(), gen);
                // And now that it's saved, nuke it
                reg->refCount = 0;
                reg->keptValue = false;
            }
            //if (regsClobberedByCall.test(live.getIndex(regToMachReg64.equal_range(r).first->second))) {
            //     gen.markRegDefined(r);
            //}
        }
    }

    // generate code for arguments
    int frame_size = 0;
    for (int u = operands.size() - 1; u >= 0; u--) {
        Address unused = ADDR_NULL;
        unsigned reg = REG_NULL;
        if (u >= (int)AARCH64_ARG_REGS) {
            if (!operands[u]->generateCode_phase2(gen, noCost, unused, reg))
                assert(0);
            assert(reg != REG_NULL);
            //emitPushReg64(reg, gen);
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
                //emitMovRegToReg64(amd64_arg_regs[u], reg, true, gen);
            }
        }
    }

    Register ret = gen.rs()->allocateRegister(gen, noCost);
    gen.markRegDefined(ret);
    //emitMovRegToReg64(ret, aarch64::x0, true, gen);

    return ret;

}

bool EmitterAARCH64::emitMoveRegToReg(Register src, Register dest, codeGen &gen) {
    assert(0);
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
