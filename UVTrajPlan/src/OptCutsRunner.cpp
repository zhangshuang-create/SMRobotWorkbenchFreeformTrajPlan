#include "OptCutsRunner.h"
#include "MeshCutter.h"
#include <OptCutsApi.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <map>
#include <tuple>
#include <nlohmann/json.hpp>

namespace SprayTrajPlan
{
    namespace
    {
        bool writeObj(const std::filesystem::path& path, const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces)
        {
            std::ofstream stream(path);
            if (!stream) return false;
            for (const auto& vertex : vertices) stream << "v " << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
            for (const auto& face : faces) stream << "f " << face[0] + 1 << " " << face[1] + 1 << " " << face[2] + 1 << "\n";
            return static_cast<bool>(stream);
        }

        bool writeSeam(const std::filesystem::path& path, const PrecutSeam& seam)
        {
            nlohmann::json json;
            json["index_base"] = 0;
            json["original_vertex_ids"] = seam.original_vertex_ids;
            json["cut_vertex_pairs"] = nlohmann::json::array();
            for (const auto& pair : seam.cut_vertex_pairs) json["cut_vertex_pairs"].push_back({pair.first, pair.second});
            json["xyz"] = nlohmann::json::array();
            for (const auto& point : seam.xyz) json["xyz"].push_back({point.x(), point.y(), point.z()});
            std::ofstream stream(path);
            if (!stream) return false;
            stream << json.dump(2);
            return static_cast<bool>(stream);
        }

        bool writeCacheV2(const std::filesystem::path& sourceMesh,
            const std::filesystem::path& precutObj, const std::filesystem::path& unfoldedObj,
            const PrecutSeam& seam)
        {
            const std::filesystem::path directory = sourceMesh.parent_path();
            const std::string stem = sourceMesh.stem().string();
            const std::filesystem::path cachedPrecut = directory / (stem + "_cutopt_precut.obj");
            const std::filesystem::path cachedUnfolded = directory / (stem + "_cutopt_result.obj");
            const std::filesystem::path cachePath = directory / (stem + "_cutopt.json");
            std::error_code error;
            std::filesystem::copy_file(precutObj, cachedPrecut,
                std::filesystem::copy_options::overwrite_existing, error);
            if (error) return false;
            std::filesystem::copy_file(unfoldedObj, cachedUnfolded,
                std::filesystem::copy_options::overwrite_existing, error);
            if (error) return false;

            nlohmann::json cache;
            cache["format"] = "optcuts-cache-v2";
            cache["model"] = {
                {"path", sourceMesh.string()},
                {"name", sourceMesh.filename().string()},
                {"size", std::filesystem::file_size(sourceMesh, error)},
            };
            cache["optcuts"] = {
                {"precut_obj", cachedPrecut.string()},
                {"unfolded_obj", cachedUnfolded.string()},
            };
            cache["precut_seam"] = {
                {"index_base", 0},
                {"original_vertex_ids", seam.original_vertex_ids},
                {"cut_vertex_pairs", nlohmann::json::array()},
                {"xyz", nlohmann::json::array()},
            };
            for (const auto& pair : seam.cut_vertex_pairs)
                cache["precut_seam"]["cut_vertex_pairs"].push_back({pair.first, pair.second});
            for (const auto& point : seam.xyz)
                cache["precut_seam"]["xyz"].push_back({point.x(), point.y(), point.z()});

            std::ofstream stream(cachePath);
            if (!stream) return false;
            stream << cache.dump(2);
            return static_cast<bool>(stream);
        }
    }

    OptCutsRunner::OptCutsRunner() = default;

    OptCutsRunner::~OptCutsRunner()
    {
    }

    void OptCutsRunner::setOptCutsRoot(const std::string& root_path)
    {
        optcuts_root_path_ = root_path;
    }

    bool OptCutsRunner::loadOrRunOptCuts(const std::string& mesh_path, OptCutsResult& result)
    {
        if (isCachePath(mesh_path)) return loadCache(mesh_path, result);
        const std::filesystem::path inputPath = std::filesystem::absolute(mesh_path);
        // A mesh selection always starts a fresh pre-cut and OptCuts run. A
        // cache is reused only when the user explicitly selects its .json
        // file, handled by the isCachePath branch above. Silently consuming a
        // sibling <mesh>_cutopt.json made Start UV Map appear to skip UV
        // unfolding after an earlier run.

        OptCutsResult candidate;
        candidate.mesh_path = inputPath.string();
        if (!loadMesh(candidate.mesh_path, candidate.mesh_vertices, candidate.mesh_faces)) {
            std::cerr << "Failed to load triangular mesh: " << mesh_path << std::endl;
            return false;
        }
        std::vector<Eigen::Vector3d> precutVertices;
        std::vector<Eigen::Vector3i> precutFaces;
        candidate.precut_seam = std::make_shared<PrecutSeam>();
        if (!MeshCutter::precutTwoBoundaries(candidate.mesh_vertices, candidate.mesh_faces,
                precutVertices, precutFaces, *candidate.precut_seam)) {
            std::cerr << "Mesh must have exactly two manifold boundary loops for UV mapping" << std::endl;
            return false;
        }

        const std::filesystem::path root = std::filesystem::absolute(optcuts_root_path_.empty()
            ? std::filesystem::temp_directory_path() / "SMRobotOptCuts"
            : std::filesystem::path(optcuts_root_path_));
        const std::filesystem::path officialInput = root / "input" / (inputPath.stem().string() + "_uvprecut.obj");
        std::filesystem::path seamJson = officialInput;
        seamJson.replace_extension(".seam.json");
        std::error_code error;
        std::filesystem::create_directories(officialInput.parent_path(), error);
        if (error || !writeObj(officialInput, precutVertices, precutFaces) ||
            !writeSeam(seamJson, *candidate.precut_seam)) {
            std::cerr << "Unable to prepare the OptCuts input files" << std::endl;
            return false;
        }

        std::string outputObj;
        std::cout << "Running OptCuts on pre-cut mesh " << officialInput << "..." << std::endl;
        if (!runOptCutsAlgorithm(officialInput.string(), outputObj)) return false;
        if (!parseOptCutsOutput(outputObj, seamJson.string(), candidate)) {
            std::cerr << "Failed to parse OptCuts output" << std::endl;
            return false;
        }
        candidate.optcuts_mesh_path = officialInput.string();
        candidate.result_obj_path = outputObj;
        if (!writeCacheV2(inputPath, officialInput, outputObj, *candidate.precut_seam)) {
            std::cerr << "OptCuts completed, but failed to save cache beside " << inputPath << std::endl;
        } else {
            std::cout << "Saved OptCuts cache: "
                << (inputPath.parent_path() / (inputPath.stem().string() + "_cutopt.json")) << std::endl;
        }
        result = std::move(candidate);
        return true;
    }

    bool OptCutsRunner::isCachePath(const std::string& path)
    {
        return std::filesystem::path(path).extension() == ".json";
    }

    bool OptCutsRunner::loadCache(const std::string& cache_path, OptCutsResult& result)
    {
        std::ifstream file(cache_path);
        if (!file.is_open()) return false;
        try {
            const nlohmann::json cache = nlohmann::json::parse(file);
            const std::string format = cache.value("format", "");
            if (format == "optcuts-cache-v1") {
                // v1 persisted only the final unfolded OBJ.  Rebuilding a
                // seam from the original STL produces different vertex IDs
                // and cannot be paired with that OBJ's UV copies.
                std::cerr << "OptCuts cache v1 has no pre-cut seam correspondence. Regenerate it with the Python baseline to create optcuts-cache-v2." << std::endl;
                return false;
            }
            if (format != "optcuts-cache-v2" || !cache.contains("model") || !cache.contains("optcuts") ||
                !cache.contains("precut_seam")) return false;
            const std::filesystem::path cacheDir = std::filesystem::absolute(cache_path).parent_path();
            std::filesystem::path meshPath = cache.at("model").at("path").get<std::string>();
            std::filesystem::path objPath = cache.at("optcuts").at("unfolded_obj").get<std::string>();
            std::filesystem::path precutPath = cache.at("optcuts").at("precut_obj").get<std::string>();
            if (!std::filesystem::exists(meshPath)) meshPath = cacheDir / meshPath.filename();
            if (!std::filesystem::exists(objPath)) objPath = cacheDir / objPath.filename();
            if (!std::filesystem::exists(precutPath)) precutPath = cacheDir / precutPath.filename();
            if (!std::filesystem::exists(meshPath) || !std::filesystem::exists(objPath) || !std::filesystem::exists(precutPath)) return false;

            OptCutsResult candidate;
            candidate.mesh_path = meshPath.string();
            candidate.optcuts_mesh_path = precutPath.string();
            candidate.result_obj_path = objPath.string();
            if (!loadMesh(candidate.mesh_path, candidate.mesh_vertices, candidate.mesh_faces)) return false;
            candidate.precut_seam = std::make_shared<PrecutSeam>();
            const auto& seamNode = cache.at("precut_seam");
            candidate.precut_seam->original_vertex_ids = seamNode.value("original_vertex_ids", std::vector<int>{});
            for (const auto& pair : seamNode.value("cut_vertex_pairs", nlohmann::json::array())) {
                if (pair.is_array() && pair.size() == 2) {
                    candidate.precut_seam->cut_vertex_pairs.emplace_back(pair[0].get<int>(), pair[1].get<int>());
                }
            }
            for (const auto& point : seamNode.value("xyz", nlohmann::json::array())) {
                if (point.is_array() && point.size() == 3) {
                    candidate.precut_seam->xyz.emplace_back(point[0].get<double>(), point[1].get<double>(), point[2].get<double>());
                }
            }
            if (candidate.precut_seam->cut_vertex_pairs.size() < 2 ||
                candidate.precut_seam->xyz.size() != candidate.precut_seam->cut_vertex_pairs.size()) return false;
            candidate.uv_data = std::make_shared<ObjUVData>();
            if (!loadObjWithUV(candidate.result_obj_path, *candidate.uv_data)) return false;
            for (const auto& pair : candidate.precut_seam->cut_vertex_pairs) {
                if (pair.first < 0 || pair.second < 0 || pair.first >= static_cast<int>(candidate.uv_data->xyz.size()) ||
                    pair.second >= static_cast<int>(candidate.uv_data->xyz.size())) return false;
            }
            result = std::move(candidate);
            return true;
        } catch (const nlohmann::json::exception&) {
            return false;
        }
    }

    bool OptCutsRunner::runOptCutsAlgorithm(const std::string& input_mesh, std::string& output_obj)
    {
        const std::filesystem::path inputPath = std::filesystem::absolute(input_mesh);
        const std::filesystem::path root = std::filesystem::absolute(optcuts_root_path_.empty()
            ? std::filesystem::temp_directory_path() / "SMRobotOptCuts"
            : std::filesystem::path(optcuts_root_path_));
        std::error_code error;
        std::filesystem::create_directories(root, error);
        if(error) {
            std::cerr << "Unable to create the OptCuts workspace: " << root << std::endl;
            return false;
        }
        const std::filesystem::path relativeInput = std::filesystem::relative(inputPath, root, error);
        const std::string relativeInputString = relativeInput.generic_string();
        if (error || relativeInput.empty() || relativeInputString.compare(0, 2, "..") == 0) {
            std::cerr << "OptCuts input must be located under OptCuts_ROOT: " << root << std::endl;
            return false;
        }

        // Retain the established OptCuts parameter order, but call its exported
        // DLL entry point directly. Mode 100 is its no-window batch mode.
        std::vector<std::string> arguments = {
            "OptCutsCore", "100", relativeInput.generic_string(), "0.999", "1",
            "0", "4.1", "1", "0", "workbench"
        };
        std::vector<char*> argv;
        argv.reserve(arguments.size());
        for(auto& argument : arguments) {
            argv.push_back(argument.data());
        }

        const std::filesystem::path previousDirectory = std::filesystem::current_path(error);
        if(error) {
            std::cerr << "Unable to read the current working directory before running OptCuts" << std::endl;
            return false;
        }
        std::filesystem::current_path(root, error);
        if(error) {
            std::cerr << "Unable to enter the OptCuts workspace: " << root << std::endl;
            return false;
        }
        const int result = runOptCuts(static_cast<int>(argv.size()), argv.data());
        std::filesystem::current_path(previousDirectory, error);
        if(result != 0 || error) {
            std::cerr << "OptCuts in-process execution failed with code " << result << std::endl;
            return false;
        }

        const std::filesystem::path outputRoot = root / "output";
        const std::string prefix = inputPath.stem().string() + "_";
        std::filesystem::file_time_type newestTime{};
        for (const auto& entry : std::filesystem::recursive_directory_iterator(outputRoot, error)) {
            if (error) break;
            const std::string parentName = entry.path().parent_path().filename().string();
            if (!entry.is_regular_file() || entry.path().filename() != "finalResult_mesh.obj" ||
                parentName.compare(0, prefix.size(), prefix) != 0) continue;
            const auto time = entry.last_write_time(error);
            if (!error && (output_obj.empty() || time > newestTime)) {
                newestTime = time;
                output_obj = entry.path().string();
            }
        }
        return !output_obj.empty();
    }

    bool OptCutsRunner::loadMesh(const std::string& path,
                                  std::vector<Eigen::Vector3d>& vertices,
                                  std::vector<Eigen::Vector3i>& faces)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }

        vertices.clear();
        faces.clear();

        const std::filesystem::path inputPath(path);
        std::string extension = inputPath.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (extension == ".stl") {
            std::map<std::tuple<float, float, float>, int> vertexIds;
            const auto addVertex = [&vertices, &vertexIds](float x, float y, float z) {
                const auto key = std::make_tuple(x, y, z);
                const auto existing = vertexIds.find(key);
                if (existing != vertexIds.end()) return existing->second;
                const int id = static_cast<int>(vertices.size());
                vertices.emplace_back(x, y, z);
                vertexIds.emplace(key, id);
                return id;
            };

            char header[80]{};
            std::uint32_t triangleCount = 0;
            file.read(header, sizeof(header));
            file.read(reinterpret_cast<char*>(&triangleCount), sizeof(triangleCount));
            const auto expectedSize = static_cast<std::uintmax_t>(84) + static_cast<std::uintmax_t>(triangleCount) * 50;
            const auto actualSize = std::filesystem::file_size(inputPath);
            if (file && triangleCount > 0 && actualSize >= expectedSize) {
                for (std::uint32_t i = 0; i < triangleCount; ++i) {
                    float record[12]{};
                    std::uint16_t attributes = 0;
                    file.read(reinterpret_cast<char*>(record), sizeof(record));
                    file.read(reinterpret_cast<char*>(&attributes), sizeof(attributes));
                    if (!file) return false;
                    faces.emplace_back(addVertex(record[3], record[4], record[5]),
                        addVertex(record[6], record[7], record[8]),
                        addVertex(record[9], record[10], record[11]));
                }
                return !vertices.empty() && !faces.empty();
            }

            // The STL header may start with "solid" even for binary input, so
            // select ASCII only after the binary-size validation above fails.
            file.clear();
            file.seekg(0);
            std::string token;
            std::vector<Eigen::Vector3d> triangle;
            while (file >> token) {
                if (token != "vertex") continue;
                double x = 0.0, y = 0.0, z = 0.0;
                if (!(file >> x >> y >> z)) return false;
                triangle.emplace_back(x, y, z);
                if (triangle.size() == 3) {
                    faces.emplace_back(addVertex(static_cast<float>(triangle[0].x()), static_cast<float>(triangle[0].y()), static_cast<float>(triangle[0].z())),
                        addVertex(static_cast<float>(triangle[1].x()), static_cast<float>(triangle[1].y()), static_cast<float>(triangle[1].z())),
                        addVertex(static_cast<float>(triangle[2].x()), static_cast<float>(triangle[2].y()), static_cast<float>(triangle[2].z())));
                    triangle.clear();
                }
            }
            return !vertices.empty() && !faces.empty();
        }

        file.close();
        file.open(path);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v") {
                double x, y, z;
                iss >> x >> y >> z;
                vertices.emplace_back(x, y, z);
            }
            else if (prefix == "f") {
                std::string v1_str, v2_str, v3_str;
                iss >> v1_str >> v2_str >> v3_str;

                // Parse face indices (handle v/vt/vn format)
                auto parse_index = [](const std::string& s) -> int {
                    size_t pos = s.find('/');
                    std::string idx_str = (pos != std::string::npos) ? s.substr(0, pos) : s;
                    return std::stoi(idx_str) - 1;  // OBJ is 1-indexed
                };

                int v1 = parse_index(v1_str);
                int v2 = parse_index(v2_str);
                int v3 = parse_index(v3_str);

                faces.emplace_back(v1, v2, v3);
            }
        }

        file.close();
        return !vertices.empty() && !faces.empty();
    }

    bool OptCutsRunner::loadObjWithUV(const std::string& path, ObjUVData& uv_data)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }

        uv_data.xyz.clear();
        uv_data.uv.clear();
        uv_data.xyz_faces.clear();
        uv_data.uv_faces.clear();

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v") {
                double x, y, z;
                iss >> x >> y >> z;
                uv_data.xyz.emplace_back(x, y, z);
            }
            else if (prefix == "vt") {
                double u, v;
                iss >> u >> v;
                uv_data.uv.emplace_back(u, v);
            }
            else if (prefix == "f") {
                std::string v1_str, v2_str, v3_str;
                iss >> v1_str >> v2_str >> v3_str;

                auto parse_indices = [](const std::string& s) -> std::pair<int, int> {
                    size_t pos1 = s.find('/');
                    if (pos1 == std::string::npos) return {-1, -1};
                    int v_idx = std::stoi(s.substr(0, pos1)) - 1;
                    int vt_idx = -1;
                    const size_t pos2 = s.find('/', pos1 + 1);
                    const std::string vt = s.substr(pos1 + 1,
                        pos2 == std::string::npos ? std::string::npos : pos2 - pos1 - 1);
                    if (!vt.empty()) {
                        vt_idx = std::stoi(vt) - 1;
                    }
                    return {v_idx, vt_idx};
                };

                auto [v1, vt1] = parse_indices(v1_str);
                auto [v2, vt2] = parse_indices(v2_str);
                auto [v3, vt3] = parse_indices(v3_str);

                const int xyzCount = static_cast<int>(uv_data.xyz.size());
                const int uvCount = static_cast<int>(uv_data.uv.size());
                if (v1 < 0 || v2 < 0 || v3 < 0 || v1 >= xyzCount || v2 >= xyzCount || v3 >= xyzCount ||
                    vt1 < 0 || vt2 < 0 || vt3 < 0 || vt1 >= uvCount || vt2 >= uvCount || vt3 >= uvCount) {
                    std::cerr << "Invalid or missing OBJ UV face indices in " << path << std::endl;
                    return false;
                }
                uv_data.xyz_faces.emplace_back(v1, v2, v3);
                uv_data.uv_faces.emplace_back(vt1, vt2, vt3);
            }
        }

        file.close();
        return !uv_data.xyz.empty() && !uv_data.uv.empty() && !uv_data.uv_faces.empty();
    }

    bool OptCutsRunner::parseSeamJson(const std::string& json_path, PrecutSeam& seam)
    {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            return false;
        }
        try {
            const nlohmann::json json = nlohmann::json::parse(file);
            const auto& seamNode = json.contains("precut_seam") ? json.at("precut_seam") : json;
            seam.original_vertex_ids = seamNode.value("original_vertex_ids", std::vector<int>{});
            seam.cut_vertex_pairs.clear();
            for (const auto& pair : seamNode.value("cut_vertex_pairs", nlohmann::json::array())) {
                if (pair.is_array() && pair.size() == 2) seam.cut_vertex_pairs.emplace_back(pair[0].get<int>(), pair[1].get<int>());
            }
            seam.xyz.clear();
            for (const auto& point : seamNode.value("xyz", nlohmann::json::array())) {
                if (point.is_array() && point.size() == 3) seam.xyz.emplace_back(point[0].get<double>(), point[1].get<double>(), point[2].get<double>());
            }
            return true;
        } catch (const nlohmann::json::exception&) {
            return false;
        }
    }

    bool OptCutsRunner::parseOptCutsOutput(const std::string& obj_path,
                                            const std::string& seam_json_path,
                                            OptCutsResult& result)
    {
        // Load UV data from OBJ
        result.uv_data = std::make_shared<ObjUVData>();
        if (!loadObjWithUV(obj_path, *result.uv_data)) {
            return false;
        }

        // Parse seam data
        result.precut_seam = std::make_shared<PrecutSeam>();
        if (!parseSeamJson(seam_json_path, *result.precut_seam)) {
            // Seam might not exist - this is okay
            result.precut_seam->xyz.clear();
        }

        return true;
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    OptCutsRunner::getCombinedCutEdges(const OptCutsResult& result)
    {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> combined_edges;

        // Add precut seam edges
        if (result.precut_seam) {
            auto seam_edges = result.precut_seam->getEdges();
            combined_edges.insert(combined_edges.end(), seam_edges.begin(), seam_edges.end());
        }

        // Add OptCuts detected edges
        combined_edges.insert(combined_edges.end(),
                            result.cut_edges.begin(),
                            result.cut_edges.end());

        return combined_edges;
    }

} // namespace SprayTrajPlan
