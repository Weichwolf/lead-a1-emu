// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/firmware.hpp"
#include "lead_a1/image.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

std::vector<char> read_bundle(const std::filesystem::path &path) {
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
  return data;
}

void export_section(const std::filesystem::path &directory,
                    std::size_t index,
                    const lead_a1::ImageSection &section) {
  std::filesystem::create_directories(directory);
  const auto name =
      std::to_string(index) + "-" + std::string(section.tag.begin(), section.tag.end());
  std::ofstream stream(directory / (name + ".bin"), std::ios::binary);
  stream.write(reinterpret_cast<const char *>(section.bytes.data()),
               static_cast<std::streamsize>(section.bytes.size()));
  stream.close();
  if (!stream) { throw std::runtime_error("Cannot export complete decoded section"); }
}

void inspect(const std::filesystem::path &path,
             const std::optional<std::filesystem::path> &output) {
  const auto data = read_bundle(path);
  const auto bytes = std::as_bytes(std::span(data));
  const auto parts = lead_a1::inspect_bundle(bytes);
  // Validate both complete images before publishing any extracted bytes.
  std::array<lead_a1::FirmwareImage, lead_a1::bundle_part_count> images;
  for (std::size_t index = 0; index < parts.size(); ++index) {
    images[index] = lead_a1::decode_image(bytes.subspan(parts[index].offset, parts[index].length));
  }
  std::cout << "Structure/checksums only; verify OS/hash with tools/references.py.\n";
  for (std::size_t index = 0; index < parts.size(); ++index) {
    const auto &part = parts[index];
    std::cout << "part " << index << ": offset=" << part.offset << " bytes=" << part.length
              << " role=unresolved\n";
    std::cout << "  image version=" << images[index].version << " type=" << images[index].type
              << '\n';
    for (const auto &section : images[index].sections) {
      std::cout << "  " << std::string_view(section.tag.data(), section.tag.size())
                << " stored=" << section.stored_length << " decoded=" << section.bytes.size()
                << " load=0x" << std::hex << section.load_address << std::dec
                << " compressed=" << section.compressed << '\n';
      if (output) { export_section(*output, index, section); }
    }
  }
  std::cout << "Firmware execution is not implemented.\n";
}

}

int main(int argc, char **argv) {
  const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
  if (arguments.size() == 2 && std::string_view(arguments[1]) == "--help") {
    std::cout << "Usage: lead-a1-emu --inspect FIRMWARE_BUNDLE\n"
                 "       lead-a1-emu --extract FIRMWARE_BUNDLE LOCAL_DIRECTORY\n"
                 "Native parameter viewer: lead-a1-ui\n"
                 "Development bootstrap; no original firmware execution yet.\n";
    return 0;
  }
  const bool inspection = arguments.size() == 3 && std::string_view(arguments[1]) == "--inspect";
  const bool extraction = arguments.size() == 4 && std::string_view(arguments[1]) == "--extract";
  if (!inspection && !extraction) {
    std::cerr << "Original firmware execution is not implemented. Use --help for inspection.\n";
    return 1;
  }
  try {
    const auto output =
        extraction ? std::optional<std::filesystem::path>{arguments[3]} : std::nullopt;
    inspect(arguments[2], output);
  } catch (const std::exception &error) {
    std::cerr << "Firmware inspection failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
