#pragma once

#include "CommonDataTypes.h"
#include <string>
#include <memory>

namespace SprayTrajPlan
{
    // Forward declarations
    class OptCutsRunner;
    struct OptCutsResult;

    // OptCuts integration result
    struct OptCutsResult {
        std::shared_ptr<ObjUVData> uv_data;
        std::shared_ptr<PrecutSeam> precut_seam;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> cut_edges;
        std::vector<Eigen::Vector3d> mesh_vertices;
        std::vector<Eigen::Vector3i> mesh_faces;
        std::string mesh_path;
        std::string optcuts_mesh_path;
        std::string result_obj_path;
    };

    // OptCuts runner class
    class OptCutsRunner
    {
    public:
        OptCutsRunner();
        ~OptCutsRunner();

        // Load or run OptCuts on a mesh file
        // Returns the OptCuts result with UV data and seam information
        bool loadOrRunOptCuts(const std::string& mesh_path, OptCutsResult& result);

        // Loads the Python-compatible optcuts-cache-v2 JSON without executing OptCuts.
        bool loadCache(const std::string& cache_path, OptCutsResult& result);

        // Get combined cut edges from OptCuts result
        static std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            getCombinedCutEdges(const OptCutsResult& result);

        // Set OptCuts executable path
        void setOptCutsExecutable(const std::string& exe_path);

        // Set OptCuts root directory
        void setOptCutsRoot(const std::string& root_path);

    private:
        std::string optcuts_exe_path_;
        std::string optcuts_root_path_;

        // Run OptCuts process
        bool runOptCutsProcess(const std::string& input_mesh, std::string& output_obj);

        // Parse OptCuts output
        bool parseOptCutsOutput(const std::string& obj_path,
                                const std::string& seam_json_path,
                                OptCutsResult& result);

        // Load mesh file
        bool loadMesh(const std::string& path,
                     std::vector<Eigen::Vector3d>& vertices,
                     std::vector<Eigen::Vector3i>& faces);

        // Load OBJ with UV coordinates
        bool loadObjWithUV(const std::string& path, ObjUVData& uv_data);

        // Parse seam JSON file
        bool parseSeamJson(const std::string& json_path, PrecutSeam& seam);
        static bool isCachePath(const std::string& path);
    };

} // namespace SprayTrajPlan
