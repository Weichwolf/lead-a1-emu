// SPDX-License-Identifier: GPL-3.0-only
// Single-translation executor approach adapted from public Weichwolf/blofeld-emu.
// No HCS12 semantics, state, interrupt or board behavior is reused.
#include "lead_a1/arm.hpp"

#include "address.hh"
#include "arm_exception.hpp"
#include "arm_status.hpp"
#include "arm_system.hpp"
#include "emulate.hh"
#include "error.hh"
#include "globalcontext.hh"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"
#include "loadimage.hh"
#include "memstate.hh"
#include "opbehavior.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "space.hh"
#include "translate.hh"
#include "types.h"
#include "xml.hh"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <ios>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lead_a1 {
namespace {

constexpr unsigned int instruction_bytes = 4;
constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();
constexpr std::array<const char *, 16> register_names{"r0",
                                                      "r1",
                                                      "r2",
                                                      "r3",
                                                      "r4",
                                                      "r5",
                                                      "r6",
                                                      "r7",
                                                      "r8",
                                                      "r9",
                                                      "r10",
                                                      "r11",
                                                      "r12",
                                                      "sp",
                                                      "lr",
                                                      "pc"};
constexpr std::array<const char *, 5> flag_names{"NG", "ZR", "CY", "OV", "Q"};
constexpr std::array<unsigned int, 5> flag_bits{31, 30, 29, 28, 27};
constexpr auto flags_mask = std::uint32_t{0xf8000000};
constexpr unsigned int program_counter_index = 15;
constexpr std::uint32_t thumb_jazelle_mask = 0x01000020;

std::string fault_context(std::uint32_t pc, const std::string &reason) {
  std::ostringstream text;
  text << "ARM reference pc=0x" << std::hex << pc << " width=4: " << reason;
  return text.str();
}

std::string xml_path(const std::filesystem::path &path) {
  std::string text;
  for (const char character : path.string()) {
    switch (character) {
      case '&': text += "&amp;"; break;
      case '<': text += "&lt;"; break;
      case '>': text += "&gt;"; break;
      default: text += character; break;
    }
  }
  return "<sleigh>" + text + "</sleigh>";
}

class CodeImage final : public ghidra::LoadImage {
public:
  struct Instruction {
    std::uint32_t address;
    std::uint32_t word;
  };

  explicit CodeImage(Memory &memory) : LoadImage("Lead A1 ARM reference RAM"), memory_(memory) {}

  void set_instruction(Instruction instruction) {
    pc_ = instruction.address;
    instruction_ = instruction.word;
  }

  void loadFill(ghidra::uint1 *bytes, ghidra::int4 size, const ghidra::Address &address) override {
    for (ghidra::int4 index = 0; index < size; ++index) {
      const auto offset = address.getOffset() + static_cast<ghidra::uintb>(index);
      if (offset >= pc_ && offset - pc_ < instruction_bytes) {
        constexpr unsigned int byte_bits = 8;
        const auto shift = static_cast<unsigned int>(offset - pc_) * byte_bits;
        bytes[index] = static_cast<ghidra::uint1>((instruction_ >> shift) & byte_mask);
        continue;
      }
      // Decoder lookahead is side-effect-free. Actual executed bytes are checked separately.
      const auto value = offset <= std::numeric_limits<std::uint32_t>::max()
                             ? memory_.peek(static_cast<std::uint32_t>(offset))
                             : std::nullopt;
      bytes[index] = std::to_integer<ghidra::uint1>(value.value_or(std::byte{}));
    }
  }

  [[nodiscard]] std::string getArchType() const override { return "ARM:LE:32:v5"; }

  void adjustVma(long /*adjustment*/) override {
    throw std::runtime_error("Reference RAM cannot be relocated");
  }

private:
  Memory &memory_;
  std::uint32_t pc_{};
  std::uint32_t instruction_{};
};

class SparseBank final : public ghidra::MemoryBank {
public:
  using Bytes = std::map<ghidra::uintb, std::uint8_t>;

  explicit SparseBank(ghidra::AddrSpace *space) : MemoryBank(space, 1, 256) {}

  Bytes bytes;

  [[nodiscard]] bool known(const ghidra::VarnodeData &node) const {
    for (ghidra::uint4 index = 0; index < node.size; ++index) {
      if (!bytes.contains(node.offset + index)) { return false; }
    }
    return true;
  }

protected:
  void insert(ghidra::uintb offset, ghidra::uintb value) override {
    if (value > byte_mask) { throw std::runtime_error("Invalid internal byte write"); }
    bytes[offset] = static_cast<std::uint8_t>(value);
  }

  [[nodiscard]] ghidra::uintb find(ghidra::uintb offset) const override {
    const auto found = bytes.find(offset);
    if (found == bytes.end()) {
      std::ostringstream text;
      text << "uninitialized " << getSpace()->getName() << " byte 0x" << std::hex << offset;
      throw std::runtime_error(text.str());
    }
    return found->second;
  }
};

struct Translation {
  std::vector<ghidra::PcodeOpRaw *> operations;
  std::vector<ghidra::VarnodeData *> varnodes;
  Translation() = default;
  Translation(const Translation &) = delete;
  Translation &operator=(const Translation &) = delete;

  ~Translation() {
    for (auto *operation : operations) { delete operation; }
    for (auto *varnode : varnodes) { delete varnode; }
  }
};

struct DecodedInstruction {
  std::string assembly;
  std::unique_ptr<Translation> semantics;
};

class Assembly final : public ghidra::AssemblyEmit {
public:
  std::string text;

  void dump(const ghidra::Address & /*address*/,
            const std::string &mnemonic,
            const std::string &operands) override {
    text = mnemonic + (operands.empty() ? "" : " " + operands);
  }
};

class Executor final : public ghidra::EmulateMemory {
public:
  Executor(detail::ArmSystem &bus, ghidra::Sleigh &translator, ghidra::MemoryState &state)
      : EmulateMemory(&state), bus_(bus), translator_(translator) {
    ghidra::OpBehavior::registerInstructions(behaviors_, &translator_);
    translator_.getUserOpNames(user_operations_);
  }

  Executor(const Executor &) = delete;
  Executor &operator=(const Executor &) = delete;

  ~Executor() override {
    for (auto *behavior : behaviors_) { delete behavior; }
  }

  void setExecuteAddress(const ghidra::Address &address) override {
    address_ = address;
    finished_ = true;
  }

  [[nodiscard]] ghidra::Address getExecuteAddress() const override { return address_; }

  [[nodiscard]] std::unique_ptr<Translation> decode(const ghidra::Address &address) {
    auto decoded = std::make_unique<Translation>();
    ghidra::PcodeEmitCache emitter(decoded->operations, decoded->varnodes, behaviors_, 0);
    const auto length = translator_.oneInstruction(emitter, address);
    if (length != instruction_bytes) {
      throw std::runtime_error("Unsupported ARM instruction width");
    }
    return decoded;
  }

  void execute(std::uint32_t pc, const Translation &decoded) {
    address_ = ghidra::Address(translator_.getDefaultCodeSpace(), pc);
    operations_ = decoded.operations;
    next_ = address_ + instruction_bytes;
    memstate->setValue("pc", next_.getOffset());
    index_ = 0;
    finished_ = false;
    constexpr unsigned int operation_limit = 100000;
    for (unsigned int steps = 0; !finished_; ++steps) {
      if (steps == operation_limit) {
        throw std::runtime_error("ARM p-code operation limit exceeded");
      }
      if (index_ == operations_.size()) {
        setExecuteAddress(next_);
        break;
      }
      currentOp = operations_[index_];
      currentBehave = currentOp->getBehavior();
      executeCurrentOp();
    }
    memstate->setValue("pc", address_.getOffset());
    clear();
  }

  void clear() {
    operations_ = {};
    currentOp = nullptr;
    currentBehave = nullptr;
  }

protected:
  void fallthruOp() override { ++index_; }

  void executeUnary() override {
    const auto *input = currentOp->getInput(0);
    const auto *output = currentOp->getOutput();
    write_value(
        output,
        currentBehave->evaluateUnary(operand_size(output), operand_size(input), read_value(input)));
  }

  void executeBinary() override {
    const auto *left = currentOp->getInput(0);
    const auto *right = currentOp->getInput(1);
    const auto *output = currentOp->getOutput();
    write_value(output,
                currentBehave->evaluateBinary(
                    operand_size(output), operand_size(left), read_value(left), read_value(right)));
  }

  void executeLoad() override { write_value(currentOp->getOutput(), bus_.read(access(false))); }

  void executeStore() override { bus_.write(access(true), read_value(currentOp->getInput(2))); }

  void executeCallother() override {
    const auto identifier = read_value(currentOp->getInput(0));
    const auto name = identifier < user_operations_.size()
                          ? user_operations_.at(static_cast<std::size_t>(identifier))
                          : "unknown intrinsic";
    if (name == "setISAMode") {
      if (currentOp->numInput() != 2 || currentOp->getOutput() != nullptr) {
        throw std::runtime_error("Unexpected ARM ISA-mode intrinsic shape");
      }
      const auto mode = read_value(currentOp->getInput(1));
      if (mode != 0) { throw std::runtime_error("Unsupported Thumb transition"); }
      // ARM5_le has one decode state. Explicitly retain ARM for an even BX target.
      memstate->setValue("TB", 0);
      fallthruOp();
      return;
    }
    throw std::runtime_error("Unsupported ARM intrinsic: " + name);
  }

  void executeBranch() override {
    const auto destination = currentOp->getInput(0)->getAddr();
    if (!destination.isConstant()) {
      setExecuteAddress(destination);
      return;
    }
    const auto displacement =
        std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(destination.getOffset()));
    const auto target = static_cast<std::int64_t>(index_) + displacement;
    if (target < 0 || static_cast<std::uint64_t>(target) > operations_.size()) {
      throw std::runtime_error("ARM p-code branch outside current instruction");
    }
    index_ = static_cast<std::size_t>(target);
  }

private:
  static ghidra::int4 operand_size(const ghidra::VarnodeData *node) {
    if (node->size == 0 || node->size > sizeof(ghidra::uintb)) {
      throw std::runtime_error("Unsupported ARM p-code operand width");
    }
    return static_cast<ghidra::int4>(node->size);
  }

  ghidra::uintb read_value(const ghidra::VarnodeData *node) const {
    const auto size = operand_size(node);
    if (node->space != translator_.getDefaultCodeSpace()) { return memstate->getValue(node); }
    return bus_.read(
        {.address = checked_address(node->offset), .size = static_cast<unsigned int>(size)});
  }

  void write_value(const ghidra::VarnodeData *node, ghidra::uintb value) {
    const auto size = operand_size(node);
    if (node->space != translator_.getDefaultCodeSpace()) {
      memstate->setValue(node, value);
      return;
    }
    bus_.write({.address = checked_address(node->offset), .size = static_cast<unsigned int>(size)},
               value);
  }

  static std::uint32_t checked_address(ghidra::uintb offset) {
    if (offset > std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error("ARM address exceeds 32 bits");
    }
    return static_cast<std::uint32_t>(offset);
  }

  [[nodiscard]] MemoryAccess access(bool store) const {
    const auto *space = currentOp->getInput(0)->getSpaceFromConst();
    if (space != translator_.getDefaultCodeSpace() || space->getWordSize() != 1) {
      throw std::runtime_error("Unsupported ARM operand address space");
    }
    const auto size =
        store ? operand_size(currentOp->getInput(2)) : operand_size(currentOp->getOutput());
    const auto address = checked_address(read_value(currentOp->getInput(1)));
    return {.address = address, .size = static_cast<unsigned int>(size)};
  }

  detail::ArmSystem &bus_;
  ghidra::Sleigh &translator_;
  std::vector<ghidra::OpBehavior *> behaviors_;
  std::vector<std::string> user_operations_;
  std::span<ghidra::PcodeOpRaw *const> operations_;
  ghidra::Address address_;
  ghidra::Address next_;
  std::size_t index_{};
  bool finished_{true};
};

}

class Arm::Impl {
public:
  Impl(Memory &bus,
       const std::filesystem::path &specification,
       const ArmState &initial,
       const std::optional<ArmSystemInitial> &system_initial,
       ArmExecutionOptions execution_options)
      : system(bus, system_initial),
        bus(bus),
        image(bus),
        context(std::make_unique<ghidra::ContextInternal>()),
        translator(&image, context.get()),
        memory(&translator),
        execution(execution_options) {
    std::istringstream configuration(xml_path(specification));
    ghidra::DocumentStorage documents;
    auto *root = documents.parseDocument(configuration)->getRoot();
    documents.registerTag(root);
    translator.initialize(documents);
    registers = std::make_unique<SparseBank>(translator.getSpaceByName("register"));
    temporaries = std::make_unique<SparseBank>(translator.getUniqueSpace());
    memory.setMemoryBank(registers.get());
    memory.setMemoryBank(temporaries.get());
    executor = std::make_unique<Executor>(system, translator, memory);
    if (!initial.registers[program_counter_index] || (initial.cpsr & thumb_jazelle_mask) != 0) {
      throw std::invalid_argument("ARM-state reference requires a known PC and ARM-state CPSR");
    }
    static_cast<void>(detail::arm_bank_index(initial.cpsr));
    apply(initial);
  }

  void apply(const ArmState &state) {
    registers->bytes.clear();
    for (std::size_t index = 0; index < state.registers.size(); ++index) {
      const auto value = state.registers[index];
      if (value) { memory.setValue(register_names[index], *value); }
    }
    memory.setValue("cpsr", state.cpsr);
    for (std::size_t index = 0; index < flag_names.size(); ++index) {
      memory.setValue(flag_names[index], (state.cpsr >> flag_bits[index]) & 1U);
    }
    memory.setValue("TB", 0);
    banks = state.banks;
  }

  ArmState state() const {
    ArmState result;
    for (std::size_t index = 0; index < register_names.size(); ++index) {
      const auto node = translator.getRegister(register_names[index]);
      if (registers->known(node)) {
        result.registers[index] = static_cast<std::uint32_t>(memory.getValue(&node));
      }
    }
    result.cpsr = static_cast<std::uint32_t>(memory.getValue("cpsr")) & ~flags_mask;
    for (std::size_t index = 0; index < flag_names.size(); ++index) {
      result.cpsr |= static_cast<std::uint32_t>(memory.getValue(flag_names[index]))
                     << flag_bits[index];
    }
    result.banks = banks;
    detail::capture_arm_banks(result);
    return result;
  }

  DecodedInstruction &decode(CodeImage::Instruction instruction) {
    constexpr unsigned int word_bits = 32;
    const auto key =
        (static_cast<std::uint64_t>(instruction.address) << word_bits) | instruction.word;
    if (execution.cache_translations) {
      const auto found = decoded.find(key);
      if (found != decoded.end()) {
        ++statistics.hits;
        return *found->second;
      }
    }
    image.set_instruction(instruction);
    // ARM5 has a fixed ISA here; per-address decompiler flow hints are not CPU state.
    auto next_context = std::make_unique<ghidra::ContextInternal>();
    translator.reset(&image, next_context.get());
    context = std::move(next_context);
    ghidra::DocumentStorage documents;
    translator.initialize(documents);
    const ghidra::Address address(translator.getDefaultCodeSpace(), instruction.address);
    Assembly assembly;
    translator.printAssembly(assembly, address);
    auto result = std::make_unique<DecodedInstruction>();
    result->assembly = std::move(assembly.text);
    result->semantics = executor->decode(address);
    ++statistics.translations;
    if (!execution.cache_translations) {
      uncached = std::move(result);
      return *uncached;
    }
    constexpr std::size_t maximum_entries = 128;
    if (decoded.size() == maximum_entries) { decoded.clear(); }
    auto &entry = *result;
    decoded.emplace(key, std::move(result));
    statistics.entries = decoded.size();
    return entry;
  }

  ArmStep step() {
    const auto before = state();
    const auto known_pc = before.registers[program_counter_index];
    if (!known_pc) { throw std::runtime_error("ARM program counter is unknown"); }
    const auto pc = *known_pc;
    bus.set_pc(pc);
    if (pc % instruction_bytes != 0) {
      throw std::runtime_error(fault_context(pc, "Unaligned ARM PC"));
    }
    auto saved_registers = registers->bytes;
    bus.begin_instruction();
    system.begin_instruction(before.cpsr);
    try {
      auto after = before;
      ArmStep result{.before = pc,
                     .after = pc,
                     .assembly = {},
                     .exception = detail::pending_arm_interrupt(before, interrupt_lines)};
      if (result.exception) {
        detail::enter_arm_exception(after, *result.exception, system.exception_vector_base());
        result.assembly = *result.exception == ArmException::fiq ? "FIQ entry" : "IRQ entry";
        apply(after);
      } else {
        const auto opcode = system.fetch(pc);
        const auto &instruction = decode({.address = pc, .word = opcode});
        result.assembly = instruction.assembly;
        temporaries->bytes.clear();
        constexpr std::uint32_t software_mask = 0x0f000000;
        if ((opcode & software_mask) == software_mask) {
          if (detail::arm_condition_passes(opcode, before)) {
            result.exception = ArmException::software_interrupt;
            detail::enter_arm_exception(after, *result.exception, system.exception_vector_base());
          } else {
            after.registers[program_counter_index] =
                detail::arm_wrapped_address(static_cast<std::uint64_t>(pc) + 4);
          }
          apply(after);
        } else if (detail::execute_arm_exception_return(opcode, after, system) ||
                   detail::execute_arm_status(opcode, after) || system.execute(opcode, after)) {
          apply(after);
        } else {
          executor->execute(pc, *instruction.semantics);
          after = state();
        }
      }
      if (memory.getValue("TB") != 0) { throw std::runtime_error("Unsupported Thumb transition"); }
      const auto next_pc = after.registers[program_counter_index];
      if (!next_pc) { throw std::runtime_error("ARM produced an unknown program counter"); }
      result.after = *next_pc;
      bus.commit_instruction();
      system.commit_instruction();
      banks = after.banks;
      return result;
    } catch (const ghidra::LowlevelError &error) {
      executor->clear();
      registers->bytes = std::move(saved_registers);
      banks = before.banks;
      bus.abort_instruction();
      system.abort_instruction();
      throw std::runtime_error(fault_context(pc, error.explain));
    } catch (const std::exception &error) {
      executor->clear();
      registers->bytes = std::move(saved_registers);
      banks = before.banks;
      bus.abort_instruction();
      system.abort_instruction();
      throw std::runtime_error(fault_context(pc, error.what()));
    }
  }

  ArmBanks banks;
  ArmInterruptLines interrupt_lines;
  detail::ArmSystem system;
  Memory &bus;
  CodeImage image;
  std::unique_ptr<ghidra::ContextInternal> context;
  ghidra::Sleigh translator;
  ghidra::MemoryState memory;
  std::unique_ptr<SparseBank> registers;
  std::unique_ptr<SparseBank> temporaries;
  std::unique_ptr<Executor> executor;
  ArmExecutionOptions execution;
  ArmDecodeStatistics statistics;
  std::map<std::uint64_t, std::unique_ptr<DecodedInstruction>> decoded;
  std::unique_ptr<DecodedInstruction> uncached;
};

Arm::Arm(Memory &memory,
         const std::filesystem::path &specification,
         const ArmState &initial,
         const std::optional<ArmSystemInitial> &system,
         ArmExecutionOptions execution) try
    : impl_(std::make_unique<Impl>(memory, specification, initial, system, execution)) {
} catch (const ghidra::LowlevelError &error) {

  throw std::runtime_error("Cannot initialize ARM semantics: " + error.explain);
}

Arm::~Arm() = default;

ArmState Arm::state() const {
  return impl_->state();
}

ArmSystemState Arm::system_state() const {
  return impl_->system.state();
}

ArmDecodeStatistics Arm::decode_statistics() const {
  return impl_->statistics;
}

void Arm::set_interrupts(ArmInterruptLines lines) {
  impl_->interrupt_lines = lines;
}

ArmInterruptLines Arm::interrupts() const {
  return impl_->interrupt_lines;
}

ArmStep Arm::step() {
  return impl_->step();
}

}
