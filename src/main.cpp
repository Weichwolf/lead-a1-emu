// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/firmware.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {

void inspect(const std::filesystem::path &path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) { throw std::runtime_error("Cannot open firmware bundle"); }
  const auto length = stream.tellg();
  if (length < 0 || length > static_cast<std::streamoff>(lead_a1::maximum_bundle_bytes)) {
    throw std::runtime_error("Unsupported firmware input size");
  }
  std::vector<char> data(static_cast<std::size_t>(length));
  stream.seekg(0);
  if (!stream.read(data.data(), static_cast<std::streamsize>(data.size()))) {
    throw std::runtime_error("Cannot read complete firmware bundle");
  }
  const auto parts = lead_a1::inspect_bundle(std::as_bytes(std::span(data)));
  std::cout << "Container structure only; verify OS/hash with tools/references.py.\n";
  for (std::size_t index = 0; index < parts.size(); ++index) {
    const auto &part = parts[index];
    std::cout << "part " << index << ": offset=" << part.offset << " bytes=" << part.length
              << " role=unresolved\n";
  }
  std::cout << "Firmware execution is not implemented.\n";
}

}

int main(int argc, char **argv) {
  const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
  if (arguments.size() == 2 && std::string_view(arguments[1]) == "--help") {
    std::cout << "Usage: lead-a1-emu --inspect FIRMWARE_BUNDLE\n"
                 "Native parameter viewer: lead-a1-ui\n"
                 "Development bootstrap; no original firmware execution yet.\n";
    return 0;
  }
  if (arguments.size() != 3 || std::string_view(arguments[1]) != "--inspect") {
    std::cerr << "Original firmware execution is not implemented. Use --help for inspection.\n";
    return 1;
  }
  try {
    inspect(arguments[2]);
  } catch (const std::exception &error) {
    std::cerr << "Firmware inspection failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
