/* This file is part of the dynarmic project.
 * Copyright (c) 2016 MerryMage
 * SPDX-License-Identifier: 0BSD
 */

#pragma once

#include <array>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include <mcl/stdint.hpp>
#include <xbyak/xbyak.h>
#include <boost/container/static_vector.hpp>

#include "dynarmic/backend/x64/block_of_code.h"
#include "dynarmic/backend/x64/hostloc.h"
#include "dynarmic/backend/x64/stack_layout.h"
#include "dynarmic/backend/x64/oparg.h"
#include "dynarmic/ir/cond.h"
#include "dynarmic/ir/microinstruction.h"
#include "dynarmic/ir/value.h"

namespace Dynarmic::IR {
enum class AccType;
}  // namespace Dynarmic::IR

namespace Dynarmic::Backend::X64 {

class RegAlloc;

struct HostLocInfo {
public:
    bool IsLocked() const;
    bool IsEmpty() const;
    bool IsLastUse() const;

    void SetLastUse();

    void ReadLock();
    void WriteLock();
    void AddArgReference();
    void ReleaseOne();
    void ReleaseAll();

    bool ContainsValue(const IR::Inst* inst) const;
    size_t GetMaxBitWidth() const;
    void AddValue(IR::Inst* inst);
    void EmitVerboseDebuggingOutput(BlockOfCode* code, size_t host_loc_index) const;
private:
//non trivial
    std::vector<IR::Inst*> values; //24
//sometimes zeroed
    size_t accumulated_uses = 0; //8
    // Block state
    size_t total_uses = 0; //8
    // Value state
    size_t max_bit_width = 0; //8
//always zeroed
    // Current instruction state
    size_t is_being_used_count = 0; //8
    size_t current_references = 0; //8
    bool is_scratch = false; //1
    bool is_set_last_use = false; //1
};
static_assert(sizeof(HostLocInfo) == 72);

struct Argument {
public:
    using copyable_reference = std::reference_wrapper<Argument>;

    IR::Type GetType() const;
    bool IsImmediate() const;
    bool IsVoid() const;

    bool FitsInImmediateU32() const;
    bool FitsInImmediateS32() const;

    bool GetImmediateU1() const;
    u8 GetImmediateU8() const;
    u16 GetImmediateU16() const;
    u32 GetImmediateU32() const;
    u64 GetImmediateS32() const;
    u64 GetImmediateU64() const;
    IR::Cond GetImmediateCond() const;
    IR::AccType GetImmediateAccType() const;

    bool IsInGpr() const;
    bool IsInXmm() const;
    bool IsInMemory() const;
private:
    friend class RegAlloc;
    explicit Argument(RegAlloc& reg_alloc) : reg_alloc(reg_alloc) {}

//data
    IR::Value value; //8
    RegAlloc& reg_alloc; //8
    bool allocated = false; //1
};

class RegAlloc final {
public:
    using ArgumentInfo = std::array<Argument, IR::max_arg_count>;
    RegAlloc() = default;
    RegAlloc(BlockOfCode* code, boost::container::static_vector<HostLoc, 28> gpr_order, boost::container::static_vector<HostLoc, 28> xmm_order);

    ArgumentInfo GetArgumentInfo(IR::Inst* inst);
    void RegisterPseudoOperation(IR::Inst* inst);
    bool IsValueLive(const IR::Inst* inst) const;

    Xbyak::Reg64 UseGpr(Argument& arg);
    Xbyak::Xmm UseXmm(Argument& arg);
    OpArg UseOpArg(Argument& arg);
    void Use(Argument& arg, HostLoc host_loc);

    Xbyak::Reg64 UseScratchGpr(Argument& arg);
    Xbyak::Xmm UseScratchXmm(Argument& arg);
    void UseScratch(Argument& arg, HostLoc host_loc);

    void DefineValue(IR::Inst* inst, const Xbyak::Reg& reg);
    void DefineValue(IR::Inst* inst, Argument& arg);

    void Release(const Xbyak::Reg& reg);

    Xbyak::Reg64 ScratchGpr();
    Xbyak::Reg64 ScratchGpr(HostLoc desired_location);
    Xbyak::Xmm ScratchXmm();
    Xbyak::Xmm ScratchXmm(HostLoc desired_location);

    void HostCall(IR::Inst* result_def = nullptr,
        const std::optional<Argument::copyable_reference> arg0 = {},
        const std::optional<Argument::copyable_reference> arg1 = {},
        const std::optional<Argument::copyable_reference> arg2 = {},
        const std::optional<Argument::copyable_reference> arg3 = {}
    );

    // TODO: Values in host flags
    void AllocStackSpace(const size_t stack_space);
    void ReleaseStackSpace(const size_t stack_space);
    void EndOfAllocScope();
    void AssertNoMoreUses();
    void EmitVerboseDebuggingOutput();
private:
    friend struct Argument;

    HostLoc SelectARegister(const boost::container::static_vector<HostLoc, 28>& desired_locations) const;
    std::optional<HostLoc> ValueLocation(const IR::Inst* value) const;

    HostLoc UseImpl(IR::Value use_value, const boost::container::static_vector<HostLoc, 28>& desired_locations);
    HostLoc UseScratchImpl(IR::Value use_value, const boost::container::static_vector<HostLoc, 28>& desired_locations);
    HostLoc ScratchImpl(const boost::container::static_vector<HostLoc, 28>& desired_locations);
    void DefineValueImpl(IR::Inst* def_inst, HostLoc host_loc);
    void DefineValueImpl(IR::Inst* def_inst, const IR::Value& use_inst);

    HostLoc LoadImmediate(IR::Value imm, HostLoc host_loc);
    void Move(HostLoc to, HostLoc from);
    void CopyToScratch(size_t bit_width, HostLoc to, HostLoc from);
    void Exchange(HostLoc a, HostLoc b);
    void MoveOutOfTheWay(HostLoc reg);

    void SpillRegister(HostLoc loc);
    HostLoc FindFreeSpill() const;
    HostLocInfo& LocInfo(HostLoc loc);
    const HostLocInfo& LocInfo(HostLoc loc) const;

    void EmitMove(size_t bit_width, HostLoc to, HostLoc from);
    void EmitExchange(HostLoc a, HostLoc b);
    Xbyak::Address SpillToOpArg(HostLoc loc);

//data
    alignas(64) boost::container::static_vector<HostLoc, 28> gpr_order;
    alignas(64) boost::container::static_vector<HostLoc, 28> xmm_order;
    alignas(64) boost::container::static_vector<HostLocInfo, NonSpillHostLocCount + SpillCount> hostloc_info;
    BlockOfCode* code = nullptr;
    size_t reserved_stack_space = 0;
};
// Ensure a cache line is used, this is primordial
static_assert(sizeof(boost::container::static_vector<HostLoc, 28>) == 64);

}  // namespace Dynarmic::Backend::X64
