#include "runtime/run_manifest.hpp"

#include <sstream>

namespace uw::runtime {

std::string RunManifest::ToJson() const {
  std::ostringstream out;
  out << "{\n"
      << "  \"run_id\": \"" << run_id << "\",\n"
      << "  \"git_commit\": \"" << git_commit << "\",\n"
      << "  \"config_hash\": \"" << config_hash << "\",\n"
      << "  \"calibration_hash\": \"" << calibration_hash << "\",\n"
      << "  \"derived_calibration_hash\": \"" << derived_calibration_hash << "\",\n"
      << "  \"model_hash\": \"" << model_hash << "\",\n"
      << "  \"dataset_or_scenario\": \"" << dataset_or_scenario << "\",\n"
      << "  \"simulator\": \"" << simulator << "\",\n"
      << "  \"os_info\": \"" << os_info << "\",\n"
      << "  \"cpu_info\": \"" << cpu_info << "\",\n"
      << "  \"gpu_info\": \"" << gpu_info << "\",\n"
      << "  \"seed\": " << seed << ",\n"
      << "  \"start_time_iso8601\": \"" << start_time_iso8601 << "\",\n"
      << "  \"end_time_iso8601\": \"" << end_time_iso8601 << "\"\n"
      << "}\n";
  return out.str();
}

}  // namespace uw::runtime
