#include "ExampleRegistry.hpp"

#include "../core/WorkbenchConstants.hpp"
#include "../scenes/BlockStackScene.hpp"
#include "../scenes/DoublePendulumScene.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace workbench {

namespace {

std::string titleCase(const std::string& raw)
{
  std::string result;
  result.reserve(raw.size());
  bool capitalize = true;
  for (char ch : raw) {
    if (ch == '_' || ch == '-' || ch == ' ') {
      result.push_back(' ');
      capitalize = true;
      continue;
    }
    if (capitalize)
      result.push_back(static_cast<char>(
          std::toupper(static_cast<unsigned char>(ch))));
    else
      result.push_back(static_cast<char>(
          std::tolower(static_cast<unsigned char>(ch))));
    capitalize = false;
  }
  return result;
}

std::string readSummaryLine(const std::filesystem::path& readmePath)
{
  if (readmePath.empty())
    return {};

  std::ifstream input(readmePath);
  if (!input)
    return {};

  std::string line;
  while (std::getline(input, line)) {
    if (line.empty())
      continue;

    std::string trimmed = line;
    trimmed.erase(trimmed.begin(), std::find_if(
        trimmed.begin(), trimmed.end(),
        [](unsigned char c) { return !std::isspace(c); }));
    trimmed.erase(std::find_if(
        trimmed.rbegin(), trimmed.rend(),
        [](unsigned char c) { return !std::isspace(c); }).base(),
        trimmed.end());

    if (trimmed.rfind("#", 0) == 0)
      trimmed.erase(0, trimmed.find_first_not_of("# "));

    if (!trimmed.empty())
      return trimmed;
  }

  return {};
}

ExampleList makeBuiltInExamples()
{
  ExampleList records;
  records.push_back(makeDoublePendulumScene());
  records.push_back(makeBlockStackScene());
  return records;
}

ExampleList discoverStandaloneExamples(const std::filesystem::path& root)
{
  ExampleList records;
  if (root.empty())
    return records;

  std::error_code ec;
  if (!std::filesystem::exists(root, ec) || !std::filesystem::is_directory(root, ec))
    return records;

  for (const auto& entry : std::filesystem::directory_iterator(root, ec)) {
    if (ec)
      break;
    if (!entry.is_directory())
      continue;

    const auto folderName = entry.path().filename().string();
    if (folderName.empty() || folderName.front() == '.')
      continue;
    if (folderName == "workbench")
      continue;

    const auto cmakeFile = entry.path() / "CMakeLists.txt";
    if (!std::filesystem::exists(cmakeFile, ec))
      continue;

    ExampleRecord record;
    record.id = folderName;
    record.name = titleCase(folderName);
    record.category = kStandaloneCategory;
    record.directory = entry.path();
    record.description = readSummaryLine(entry.path() / "README.md");
    records.push_back(std::move(record));
  }

  std::sort(
      records.begin(),
      records.end(),
      [](const ExampleRecord& a, const ExampleRecord& b) {
        return a.name < b.name;
      });

  return records;
}

} // namespace

ExampleList loadExampleRecords(const std::filesystem::path& examplesRoot)
{
  ExampleList records = makeBuiltInExamples();
  ExampleList standalone = discoverStandaloneExamples(examplesRoot);
  records.insert(records.end(), standalone.begin(), standalone.end());
  return records;
}

std::filesystem::path detectExamplesRoot()
{
  auto current = std::filesystem::current_path();
  for (int i = 0; i < 6; ++i) {
    std::filesystem::path candidate = current / "examples";
    std::error_code ec;
    if (std::filesystem::exists(candidate, ec)
        && std::filesystem::is_directory(candidate, ec))
      return candidate;
    if (!current.has_parent_path())
      break;
    current = current.parent_path();
  }
  return {};
}

} // namespace workbench
