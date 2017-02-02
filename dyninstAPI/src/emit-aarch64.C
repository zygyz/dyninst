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

//const int EmitterAARCH64::mt_offset = -8;

bool shouldSaveReg(registerSlot *reg, baseTramp *inst, bool saveFlags) {

   if (inst->point()) {
      regalloc_printf("\t shouldSaveReg for BT %p, from 0x%lx\n", inst, inst->point()->insnAddr() );
   }
   else {
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

