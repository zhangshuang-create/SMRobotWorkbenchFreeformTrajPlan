#include <OptSafeTrajData/OptSafeTrajData.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <filesystem>
#ifdef _WIN32
#include <io.h>
#include <direct.h>
#include <windows.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

namespace PublicData
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    OptSafeTrajData::OptSafeTrajData()
        : transformMatrix_(Eigen::Matrix4d::Identity()),
        config_file_path_(""),
        robot1_xml_path_(""),
        robot1_stl_path_(""),
        robot2_xml_path_(""),
        robot2_stl_path_(""),
        trajectory_file_path_("")
    {
    }

    OptSafeTrajData::~OptSafeTrajData()
    {
    }

    void OptSafeTrajData::clear()
    {
        trajectoryPoints_.clear();
    }

    bool OptSafeTrajData::loadFromFile(const std::string& file_path)
    {
        std::ifstream fin(file_path);
        if (!fin.is_open()) {
            std::cerr << "[PublicTrajData] Error: Cannot open file: " << file_path << std::endl;
            return false;
        }

        // Clear existing data
        clear();

        std::string line;
        int point_index = -1;
        OptTrajectoryPoint current_point;
        int pose_row = 0;
        bool reading_pose = false;
        bool reading_orient_tol = false;
        bool reading_pos_tol = false;

        while (std::getline(fin, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // Check for point header
            if (line.find("=== Point") != std::string::npos) {
                // Save previous point if exists
                if (point_index >= 0) {
                    trajectoryPoints_[static_cast<double>(point_index)] = current_point;
                }

                // Extract point index
                std::istringstream iss(line);
                std::string dummy;
                iss >> dummy >> dummy >> point_index;

                // Reset for new point
                current_point = OptTrajectoryPoint();
                pose_row = 0;
                reading_pose = false;
                reading_orient_tol = false;
                reading_pos_tol = false;
                continue;
            }

            // Parse pose matrix
            if (line.find("Pose (4x4 matrix):") != std::string::npos) {
                reading_pose = true;
                pose_row = 0;
                continue;
            }

            if (reading_pose && pose_row < 4) {
                std::istringstream iss(line);
                for (int c = 0; c < 4; ++c) {
                    iss >> current_point.pose(pose_row, c);
                }
                pose_row++;
                if (pose_row >= 4) {
                    reading_pose = false;
                }
                continue;
            }

            // Parse orientation tolerance
            if (line.find("Orientation Tolerance") != std::string::npos) {
                reading_orient_tol = true;
                continue;
            }

            if (reading_orient_tol) {
                if (line.find("Rz:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        double min_deg = std::stod(line.substr(start + 1, comma - start - 1));
                        double max_deg = std::stod(line.substr(comma + 1, end - comma - 1));
                        current_point.orient_tolerance(2, 0) = min_deg * M_PI / 180.0;
                        current_point.orient_tolerance(2, 1) = max_deg * M_PI / 180.0;
                    }
                }
                else if (line.find("Rx:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        double min_deg = std::stod(line.substr(start + 1, comma - start - 1));
                        double max_deg = std::stod(line.substr(comma + 1, end - comma - 1));
                        current_point.orient_tolerance(0, 0) = min_deg * M_PI / 180.0;
                        current_point.orient_tolerance(0, 1) = max_deg * M_PI / 180.0;
                    }
                }
                else if (line.find("Ry:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        double min_deg = std::stod(line.substr(start + 1, comma - start - 1));
                        double max_deg = std::stod(line.substr(comma + 1, end - comma - 1));
                        current_point.orient_tolerance(1, 0) = min_deg * M_PI / 180.0;
                        current_point.orient_tolerance(1, 1) = max_deg * M_PI / 180.0;
                    }
                    reading_orient_tol = false;
                }
                continue;
            }

            // Parse position tolerance
            if (line.find("Position Tolerance") != std::string::npos) {
                reading_pos_tol = true;
                continue;
            }

            if (reading_pos_tol) {
                if (line.find("Z:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        current_point.pos_tolerance(2, 0) = std::stod(line.substr(start + 1, comma - start - 1));
                        current_point.pos_tolerance(2, 1) = std::stod(line.substr(comma + 1, end - comma - 1));
                    }
                }
                else if (line.find("X:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        current_point.pos_tolerance(0, 0) = std::stod(line.substr(start + 1, comma - start - 1));
                        current_point.pos_tolerance(0, 1) = std::stod(line.substr(comma + 1, end - comma - 1));
                    }
                }
                else if (line.find("Y:") != std::string::npos) {
                    size_t start = line.find('[');
                    size_t comma = line.find(',');
                    size_t end = line.find(']');
                    if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                        current_point.pos_tolerance(1, 0) = std::stod(line.substr(start + 1, comma - start - 1));
                        current_point.pos_tolerance(1, 1) = std::stod(line.substr(comma + 1, end - comma - 1));
                    }
                    reading_pos_tol = false;
                }
                continue;
            }

            // Parse adjustable flag
            if (line.find("Adjustable:") != std::string::npos) {
                current_point.is_adjustable = (line.find("true") != std::string::npos);
                continue;
            }

            // Parse tolerance frame
            if (line.find("Tolerance Frame:") != std::string::npos) {
                if (line.find("World") != std::string::npos) {
                    current_point.tolerance_frame = CoordinateFrame::World;
                }
                else if (line.find("Workpiece") != std::string::npos) {
                    current_point.tolerance_frame = CoordinateFrame::Workpiece;
                }
                else if (line.find("Local") != std::string::npos) {
                    current_point.tolerance_frame = CoordinateFrame::Local;
                }
                continue;
            }
        }

        // Save last point
        if (point_index >= 0) {
            trajectoryPoints_[static_cast<double>(point_index)] = current_point;
        }

        fin.close();

        std::cout << "[PublicTrajData] Loaded " << trajectoryPoints_.size()
            << " points from file: " << file_path << std::endl;

        return !trajectoryPoints_.empty();
    }

    bool OptSafeTrajData::loadFromData(const std::map<double, OptTrajectoryPoint>& traj_points)
    {
        // Clear existing data
        clear();

        // Copy data
        trajectoryPoints_ = traj_points;

        std::cout << "[PublicTrajData] Loaded " << trajectoryPoints_.size()
            << " points from data structure" << std::endl;

        return !trajectoryPoints_.empty();
    }

    // Helper function: Extract string value from JSON
    static std::string extractJsonStringValue(const std::string& json, const std::string& key)
    {
        std::string searchKey = "\"" + key + "\"";
        size_t keyPos = json.find(searchKey);
        if (keyPos == std::string::npos) {
            return "";
        }

        size_t colonPos = json.find(':', keyPos);
        if (colonPos == std::string::npos) {
            return "";
        }

        colonPos = json.find_first_not_of(" \t\n\r", colonPos + 1);
        if (colonPos == std::string::npos || json[colonPos] != '"') {
            return "";
        }

        size_t startPos = colonPos + 1;
        size_t endPos = json.find('"', startPos);
        if (endPos == std::string::npos) {
            return "";
        }

        return json.substr(startPos, endPos - startPos);
    }

    // Helper function: Get lowercase file extension
    static std::string getFileExtension(const std::string& file_path)
    {
        size_t lastDot = file_path.find_last_of(".");
        if (lastDot != std::string::npos && lastDot < file_path.length() - 1) {
            std::string ext = file_path.substr(lastDot);
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            return ext;
        }
        return "";
    }

    // Helper function: Find STL file in directory (platform-specific)
    static std::string findStlFileInDirectory(const std::string& dir_path)
    {
#ifdef __cpp_lib_filesystem
        try {
            namespace fs = std::filesystem;
            if (fs::exists(dir_path) && fs::is_directory(dir_path)) {
                for (const auto& entry : fs::directory_iterator(dir_path)) {
                    if (entry.is_regular_file()) {
                        std::string filePath = entry.path().string();
                        std::string ext = getFileExtension(filePath);
                        if (ext == ".stl") {
                            return filePath;
                        }
                    }
                }
            }
        }
        catch (...) {
        }
#endif

#ifdef _WIN32
        std::string searchPath = dir_path;
        if (!searchPath.empty() && searchPath.back() != '\\' && searchPath.back() != '/') {
            searchPath += "\\";
        }
        searchPath += "*.stl";

        WIN32_FIND_DATAA findData;
        HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findData);
        if (hFind != INVALID_HANDLE_VALUE) {
            std::string result = dir_path;
            if (!result.empty() && result.back() != '\\' && result.back() != '/') {
                result += "\\";
            }
            result += findData.cFileName;
            FindClose(hFind);
            return result;
        }

        searchPath = dir_path;
        if (!searchPath.empty() && searchPath.back() != '\\' && searchPath.back() != '/') {
            searchPath += "\\";
        }
        searchPath += "*.STL";
        hFind = FindFirstFileA(searchPath.c_str(), &findData);
        if (hFind != INVALID_HANDLE_VALUE) {
            std::string result = dir_path;
            if (!result.empty() && result.back() != '\\' && result.back() != '/') {
                result += "\\";
            }
            result += findData.cFileName;
            FindClose(hFind);
            return result;
        }
#else
        DIR* dir = opendir(dir_path.c_str());
        if (dir != nullptr) {
            struct dirent* entry;
            while ((entry = readdir(dir)) != nullptr) {
                std::string filename = entry->d_name;
                std::string ext = getFileExtension(filename);
                if (ext == ".stl" || ext == ".STL") {
                    closedir(dir);
                    std::string result = dir_path;
                    if (!result.empty() && result.back() != '/') {
                        result += "/";
                    }
                    result += filename;
                    return result;
                }
            }
            closedir(dir);
        }
#endif

        return "";
    }

    // Helper function: Get directory of a file
    static std::string getFileDirectory(const std::string& file_path)
    {
        size_t lastSlash = file_path.find_last_of("/\\");
        if (lastSlash != std::string::npos) {
            return file_path.substr(0, lastSlash + 1);
        }
        return "./";
    }

    bool OptSafeTrajData::loadProjectConfig(const std::string& json_file_path)
    {
        // 1. Read JSON file
        std::ifstream fin(json_file_path);
        if (!fin.is_open()) {
            std::cerr << "[PublicTrajData] Error: Cannot open JSON file: " << json_file_path << std::endl;
            return false;
        }

        std::ostringstream jsonContent;
        jsonContent << fin.rdbuf();
        fin.close();
        std::string json = jsonContent.str();

        // 2. Save config file path
        config_file_path_ = json_file_path;

        // 3. Get project root directory (directory of JSON file)
        std::string projectRoot = getFileDirectory(json_file_path);

        // 4. Parse robots array - find XML paths for robot1 and robot2
        std::string robot1XmlRelPath = "";
        std::string robot2XmlRelPath = "";

        size_t robot1Pos = json.find("\"robot_id\": \"robot1\"");
        size_t robot2Pos = json.find("\"robot_id\": \"robot2\"");

        if (robot1Pos != std::string::npos) {
            size_t modelFilePos = json.rfind("\"model_file\"", robot1Pos + 50);
            if (modelFilePos != std::string::npos && modelFilePos < robot1Pos + 200) {
                robot1XmlRelPath = extractJsonStringValue(json.substr(modelFilePos), "model_file");
            }
        }

        if (robot2Pos != std::string::npos) {
            size_t modelFilePos = json.rfind("\"model_file\"", robot2Pos + 50);
            if (modelFilePos != std::string::npos && modelFilePos < robot2Pos + 200) {
                robot2XmlRelPath = extractJsonStringValue(json.substr(modelFilePos), "model_file");
            }
        }

        // 5. Build full XML paths
        if (!robot1XmlRelPath.empty()) {
            robot1_xml_path_ = projectRoot + robot1XmlRelPath;
            std::string robot1Dir = getFileDirectory(robot1_xml_path_);
            robot1_stl_path_ = findStlFileInDirectory(robot1Dir);
            if (robot1_stl_path_.empty()) {
                std::cout << "[PublicTrajData] Warning: Could not find STL file in " << robot1Dir << std::endl;
            }
        }

        if (!robot2XmlRelPath.empty()) {
            robot2_xml_path_ = projectRoot + robot2XmlRelPath;
            std::string robot2Dir = getFileDirectory(robot2_xml_path_);
            robot2_stl_path_ = findStlFileInDirectory(robot2Dir);
            if (robot2_stl_path_.empty()) {
                std::cout << "[PublicTrajData] Warning: Could not find STL file in " << robot2Dir << std::endl;
            }
        }

        // 6. Parse trajectory object - find trajectory file path
        size_t trajectoryPos = json.find("\"trajectory\"");
        if (trajectoryPos != std::string::npos) {
            std::string trajectorySection = json.substr(trajectoryPos);

            std::string trajFileRelPath = extractJsonStringValue(trajectorySection, "trajectory_file_txt");
            if (trajFileRelPath.empty()) {
                trajFileRelPath = extractJsonStringValue(trajectorySection, "trajectory_file");
            }

            if (!trajFileRelPath.empty()) {
                trajectory_file_path_ = projectRoot + trajFileRelPath;
            }
        }

        // 7. Validate required paths
        if (robot1_xml_path_.empty() && robot2_xml_path_.empty()) {
            std::cerr << "[PublicTrajData] Warning: No robot XML paths found in JSON!" << std::endl;
        }

        if (trajectory_file_path_.empty()) {
            std::cerr << "[PublicTrajData] Warning: No trajectory file path found in JSON!" << std::endl;
        }

        // 8. Print load result
        std::cout << "[PublicTrajData] Project paths loaded from JSON:" << std::endl;
        std::cout << "  Config file: " << config_file_path_ << std::endl;
        if (!robot1_xml_path_.empty()) {
            std::cout << "  Robot1 XML: " << robot1_xml_path_ << std::endl;
            if (!robot1_stl_path_.empty()) {
                std::cout << "  Robot1 STL: " << robot1_stl_path_ << std::endl;
            }
        }
        if (!robot2_xml_path_.empty()) {
            std::cout << "  Robot2 XML: " << robot2_xml_path_ << std::endl;
            if (!robot2_stl_path_.empty()) {
                std::cout << "  Robot2 STL: " << robot2_stl_path_ << std::endl;
            }
        }
        if (!trajectory_file_path_.empty()) {
            std::cout << "  Trajectory: " << trajectory_file_path_ << std::endl;
        }

        return true;
    }
}