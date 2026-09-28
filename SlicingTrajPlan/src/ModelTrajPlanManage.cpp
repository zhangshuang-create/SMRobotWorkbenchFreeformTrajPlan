#include <BurTrajPlan/ModelTrajPlanManage.h>

#include <STLProcessor/STLProcessor.h>

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace ModelTrajPlan
{
    namespace
    {
        std::string toAbsolutePath(const std::string& projectDir, const std::string& path)
        {
            const fs::path p(path);
            if (p.is_absolute())
            {
                return p.lexically_normal().string();
            }
            return (fs::path(projectDir) / p).lexically_normal().string();
        }

        nlohmann::json vec3dToJson(const Eigen::Vector3d& v)
        {
            return nlohmann::json::array({ v.x(), v.y(), v.z() });
        }

        bool jsonToVec3d(const nlohmann::json& j, Eigen::Vector3d& out)
        {
            if (!j.is_array() || j.size() != 3) return false;
            out = Eigen::Vector3d(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
            return true;
        }

        nlohmann::json boundaryLineToJson(const BoundaryLine& line)
        {
            nlohmann::json out;
            out["name"] = line.name;
            out["type"] = static_cast<int>(line.Type);
            out["z_level"] = line.z_level;
            out["center"] = vec3dToJson(line.center);
            out["normal"] = vec3dToJson(line.normal);
            out["points"] = nlohmann::json::array();
            for (const auto& p : line.points)
            {
                out["points"].push_back({
                    { "pos", vec3dToJson(p.position) },
                    { "angle", p.angle },
                    { "index", p.index }
                });
            }
            return out;
        }

        void jsonToBoundaryLine(const nlohmann::json& in, BoundaryLine& line)
        {
            if (in.contains("name")) line.name = in["name"].get<std::string>();
            if (in.contains("type")) line.Type = static_cast<BoundaryLineType>(in["type"].get<int>());
            if (in.contains("z_level")) line.z_level = in["z_level"].get<double>();
            if (in.contains("center")) jsonToVec3d(in["center"], line.center);
            if (in.contains("normal")) jsonToVec3d(in["normal"], line.normal);
            if (in.contains("points") && in["points"].is_array())
            {
                line.points.clear();
                for (const auto& p : in["points"])
                {
                    BasicPoint bp;
                    if (p.contains("pos")) jsonToVec3d(p["pos"], bp.position);
                    if (p.contains("angle")) bp.angle = p["angle"].get<double>();
                    if (p.contains("index")) bp.index = p["index"].get<size_t>();
                    line.points.push_back(std::move(bp));
                }
            }
        }

        bool hasGeneralModelData(const GeneralData& data)
        {
            return !data.modelDataManager_.mainModelData_.m_vertices.empty() ||
                   !data.modelDataManager_.allModelsData_.empty();
        }

        bool hasBoundaryData(const GeneralData& data)
        {
            return !data.boundaryData_.boundaryLines_.empty() ||
                   !data.boundaryData_.upperBoundary_.points.empty() ||
                   !data.boundaryData_.lowerBoundary_.points.empty();
        }

        bool hasTrajData(const GeneralData& data)
        {
            return !data.trajData_.trajectoryPoints_.empty() ||
                   !data.trajData_.connectionData_.connectedTrajectory_.empty();
        }

        bool hasSlicingData(const GeneralData& data)
        {
            return !data.slicingData_.cuttingPlanes_.empty() ||
                   !data.slicingData_.sliceLines_.empty() ||
                   !data.slicingData_.currentSliceResult_.internalBoundaries.empty();
        }
    }

    ModelTrajPlanManage::ModelTrajPlanManage(
        GeneralData& generalData,
        PublicData::PublicModelData& publicModelData,
        PublicData::TrajData& publicTrajData,
        PublicData::PublicDataRender& publicRenderData,
        IBurTrajPlan_spointer burTrajPlan,
        RenderData& renderData)
        : generalData_(generalData)
        , publicModelData_(publicModelData)
        , publicTrajData_(publicTrajData)
        , publicRenderData_(publicRenderData)
        , burTrajPlan_(burTrajPlan)
        , renderData_(renderData)
    {
    }

    bool ModelTrajPlanManage::save(const std::string& projectDir, nlohmann::json& root)
    {
        root["model_traj_plan"]["schema_version"] = 1;

        std::string modelPath = publicModelData_.getModel().m_file_path;
        if (modelPath.empty())
        {
            modelPath = generalData_.modelDataManager_.mainModelData_.m_file_path;
        }
        if (!modelPath.empty())
        {
            root["model_traj_plan"]["public_model"]["path"] = modelPath;
        }

        std::string trajPath = publicTrajData_.getTrajectoryFilePath();
        if (trajPath.empty() && !generalData_.filePaths_.empty())
        {
            trajPath = generalData_.filePaths_.back();
        }
        if (!trajPath.empty())
        {
            root["model_traj_plan"]["public_traj"]["path"] = trajPath;
        }

        const std::string relGeneralPath = "modeltrajplan/general_data.json";
        const std::string absGeneralPath = toAbsolutePath(projectDir, relGeneralPath);
        if (!saveGeneralData(absGeneralPath))
        {
            return false;
        }
        root["model_traj_plan"]["general_data"]["path"] = relGeneralPath;

        return true;
    }

    bool ModelTrajPlanManage::load(const std::string& projectDir, const nlohmann::json& root)
    {
        if (!root.contains("model_traj_plan"))
        {
            return false;
        }

        const auto& node = root["model_traj_plan"];
        std::vector<std::string> loadedPaths;

        if (node.contains("public_model") && node["public_model"].contains("path"))
        {
            const std::string rawPath = node["public_model"]["path"].get<std::string>();
            const std::string absPath = toAbsolutePath(projectDir, rawPath);
            if (!publicModelData_.loadModel(absPath))
            {
                return false;
            }
            publicModelData_.getModel().m_file_path = absPath;
            loadedPaths.push_back(absPath);
        }

        if (node.contains("public_traj") && node["public_traj"].contains("path"))
        {
            const std::string rawPath = node["public_traj"]["path"].get<std::string>();
            const std::string absPath = toAbsolutePath(projectDir, rawPath);
            if (!publicTrajData_.loadFromFile(absPath, publicTrajData_.getTrajectoryPoints()))
            {
                return false;
            }
            publicTrajData_.getTrajectoryFilePath() = absPath;
            loadedPaths.push_back(absPath);
        }

        if (node.contains("general_data") && node["general_data"].contains("path"))
        {
            const std::string absGeneralPath =
                toAbsolutePath(projectDir, node["general_data"]["path"].get<std::string>());
            if (!loadGeneralData(absGeneralPath))
            {
                return false;
            }
        }
        syncPublicToGeneralAfterLoad();
        if (!loadedPaths.empty())
        {
            generalData_.filePaths_ = loadedPaths;
        }
        rebuildPublicRenderableData();
        if (burTrajPlan_)
        {
            initializeRendererAndRestoreVisibility(*burTrajPlan_, generalData_, renderData_);
        }
        return true;
    }

    bool ModelTrajPlanManage::saveGeneralData(const std::string& absPath) const
    {
        std::error_code ec;
        fs::create_directories(fs::path(absPath).parent_path(), ec);
        if (ec)
        {
            return false;
        }

        nlohmann::json j;
        j["schema_version"] = 1;
        j["current_state"] = static_cast<int>(generalData_.currentState_);
        j["has_original_data"] = generalData_.hasOriginalData_;
        j["has_preprocessed_data"] = generalData_.hasPreprocessedData_;
        j["has_extraction_results"] = generalData_.hasExtractionResults_;
        j["file_paths"] = generalData_.filePaths_;
        j["processing_state"] = {
            { "is_stl_processed", generalData_.processingState_.isSTLProcessed },
            { "is_boundary_detected", generalData_.processingState_.isBoundaryDetected },
            { "is_slicing_completed", generalData_.processingState_.isSlicingCompleted },
            { "is_trajectory_generated", generalData_.processingState_.isTrajectoryGenerated },
            { "is_trajectory_connected", generalData_.processingState_.isTrajectoryConnected },
            { "keep_original_visible_after_preprocess", generalData_.processingState_.keepOriginalVisibleAfterPreprocess },
            { "original_data_visible", generalData_.processingState_.originalDataVisible }
        };

        if (hasGeneralModelData(generalData_))
        {
            nlohmann::json modelNode;
            modelNode["main_model_path"] = generalData_.modelDataManager_.mainModelData_.m_file_path;
            modelNode["all_model_paths"] = nlohmann::json::array();
            for (const auto& item : generalData_.modelDataManager_.allModelsData_)
            {
                if (!item.m_file_path.empty())
                {
                    modelNode["all_model_paths"].push_back(item.m_file_path);
                }
            }
            modelNode["loaded_file_paths"] = generalData_.modelDataManager_.loadedFilePaths_;
            j["modelDataManager"] = std::move(modelNode);
        }

        if (hasBoundaryData(generalData_))
        {
            nlohmann::json b;
            b["z_max_boundary"] = generalData_.boundaryData_.z_max_boundary_;
            b["z_min_boundary"] = generalData_.boundaryData_.z_min_boundary_;

            b["upper_boundary"] = boundaryLineToJson(generalData_.boundaryData_.upperBoundary_);
            b["lower_boundary"] = boundaryLineToJson(generalData_.boundaryData_.lowerBoundary_);
            b["upper_boundary_offset"] = boundaryLineToJson(generalData_.boundaryData_.upperBoundaryOffset_);
            b["lower_boundary_offset"] = boundaryLineToJson(generalData_.boundaryData_.lowerBoundaryOffset_);
            b["boundary_lines"] = nlohmann::json::array();
            for (const auto& line : generalData_.boundaryData_.boundaryLines_)
            {
                b["boundary_lines"].push_back(boundaryLineToJson(line));
            }

            j["boundaryData"] = std::move(b);
        }

        if (hasTrajData(generalData_))
        {
            nlohmann::json t;
            t["trajectory_points"] = nlohmann::json::array();
            for (const auto& p : generalData_.trajData_.trajectoryPoints_)
            {
                t["trajectory_points"].push_back({
                    { "pose", {
                        { p.pose(0,0), p.pose(0,1), p.pose(0,2), p.pose(0,3) },
                        { p.pose(1,0), p.pose(1,1), p.pose(1,2), p.pose(1,3) },
                        { p.pose(2,0), p.pose(2,1), p.pose(2,2), p.pose(2,3) },
                        { p.pose(3,0), p.pose(3,1), p.pose(3,2), p.pose(3,3) } } },
                    { "spray_distance", p.spray_distance },
                    { "trajectory_index", p.trajectory_index },
                    { "layer_index", p.layer_index },
                    { "timestamp", p.timestamp },
                    { "velocity", p.velocity },
                    { "acceleration", p.acceleration },
                    { "point_type", p.point_type }
                });
            }
            t["sampled_boundary_points"] = nlohmann::json::array();
            for (const auto& line : generalData_.trajData_.sampledBoundaryPoints_)
            {
                nlohmann::json arr = nlohmann::json::array();
                for (const auto& p : line)
                {
                    arr.push_back(vec3dToJson(p));
                }
                t["sampled_boundary_points"].push_back(std::move(arr));
            }
            j["trajData"] = std::move(t);
        }

        if (hasSlicingData(generalData_))
        {
            nlohmann::json s;
            s["last_selected_layer_index"] = generalData_.slicingData_.lastSelectedLayerIndex;
            s["min_z_level"] = generalData_.slicingData_.minZLevel_;
            s["max_z_level"] = generalData_.slicingData_.maxZLevel_;
            s["cutting_planes"] = nlohmann::json::array();
            for (const auto& cp : generalData_.slicingData_.cuttingPlanes_)
            {
                nlohmann::json cpNode;
                cpNode["normal"] = vec3dToJson(cp.normal);
                cpNode["center"] = vec3dToJson(cp.center);
                cpNode["layer_index"] = cp.layer_index;
                cpNode["translation_x"] = cp.translation_x;
                cpNode["translation_y"] = cp.translation_y;
                cpNode["translation_z"] = cp.translation_z;
                cpNode["rotation_x"] = cp.rotation_x;
                cpNode["rotation_y"] = cp.rotation_y;
                cpNode["rotation_z"] = cp.rotation_z;
                cpNode["is_selected"] = cp.isSelected;
                cpNode["vertices"] = nlohmann::json::array();
                for (const auto& v : cp.vertices)
                {
                    cpNode["vertices"].push_back(vec3dToJson(v));
                }
                s["cutting_planes"].push_back(std::move(cpNode));
            }
            s["slice_lines"] = nlohmann::json::array();
            for (const auto& line : generalData_.slicingData_.sliceLines_)
            {
                s["slice_lines"].push_back(boundaryLineToJson(line));
            }
            j["slicingData"] = std::move(s);
        }

        std::ofstream out(absPath);
        if (!out.is_open())
        {
            return false;
        }
        out << j.dump(4);
        return true;
    }

    bool ModelTrajPlanManage::loadGeneralData(const std::string& absPath)
    {
        std::ifstream in(absPath);
        if (!in.is_open())
        {
            return false;
        }

        nlohmann::json j;
        in >> j;

        if (j.contains("current_state")) generalData_.currentState_ = static_cast<DataState>(j["current_state"].get<int>());
        if (j.contains("has_original_data")) generalData_.hasOriginalData_ = j["has_original_data"].get<bool>();
        if (j.contains("has_preprocessed_data")) generalData_.hasPreprocessedData_ = j["has_preprocessed_data"].get<bool>();
        if (j.contains("has_extraction_results")) generalData_.hasExtractionResults_ = j["has_extraction_results"].get<bool>();
        if (j.contains("file_paths")) generalData_.filePaths_ = j["file_paths"].get<std::vector<std::string>>();
        if (j.contains("processing_state"))
        {
            const auto& p = j["processing_state"];
            if (p.contains("is_stl_processed")) generalData_.processingState_.isSTLProcessed = p["is_stl_processed"].get<bool>();
            if (p.contains("is_boundary_detected")) generalData_.processingState_.isBoundaryDetected = p["is_boundary_detected"].get<bool>();
            if (p.contains("is_slicing_completed")) generalData_.processingState_.isSlicingCompleted = p["is_slicing_completed"].get<bool>();
            if (p.contains("is_trajectory_generated")) generalData_.processingState_.isTrajectoryGenerated = p["is_trajectory_generated"].get<bool>();
            if (p.contains("is_trajectory_connected")) generalData_.processingState_.isTrajectoryConnected = p["is_trajectory_connected"].get<bool>();
            if (p.contains("keep_original_visible_after_preprocess")) generalData_.processingState_.keepOriginalVisibleAfterPreprocess = p["keep_original_visible_after_preprocess"].get<bool>();
            if (p.contains("original_data_visible")) generalData_.processingState_.originalDataVisible = p["original_data_visible"].get<bool>();
        }

        if (j.contains("modelDataManager"))
        {
            const auto& m = j["modelDataManager"];
            std::string mainPath;
            if (m.contains("main_model_path")) mainPath = m["main_model_path"].get<std::string>();
            if (m.contains("loaded_file_paths"))
            {
                generalData_.modelDataManager_.loadedFilePaths_ = m["loaded_file_paths"].get<std::vector<std::string>>();
            }
            if (mainPath.empty() && !generalData_.modelDataManager_.loadedFilePaths_.empty())
            {
                mainPath = generalData_.modelDataManager_.loadedFilePaths_.front();
            }
            if (!mainPath.empty())
            {
                const std::string absMainPath = toAbsolutePath(fs::path(absPath).parent_path().parent_path().string(), mainPath);
                generalData_.modelDataManager_.loadFromFile(absMainPath);
            }
        }

        if (j.contains("trajData"))
        {
            const auto& t = j["trajData"];
            if (t.contains("trajectory_points") && t["trajectory_points"].is_array())
            {
                generalData_.trajData_.trajectoryPoints_.clear();
                for (const auto& item : t["trajectory_points"])
                {
                    TrajectoryPoint p;
                    if (item.contains("pose") && item["pose"].is_array() && item["pose"].size() == 4)
                    {
                        for (int r = 0; r < 4; ++r)
                        {
                            if (item["pose"][r].is_array() && item["pose"][r].size() == 4)
                            {
                                for (int c = 0; c < 4; ++c) p.pose(r, c) = item["pose"][r][c].get<double>();
                            }
                        }
                    }
                    if (item.contains("spray_distance")) p.spray_distance = item["spray_distance"].get<double>();
                    if (item.contains("trajectory_index")) p.trajectory_index = item["trajectory_index"].get<int>();
                    if (item.contains("layer_index")) p.layer_index = item["layer_index"].get<int>();
                    if (item.contains("timestamp")) p.timestamp = item["timestamp"].get<double>();
                    if (item.contains("velocity")) p.velocity = item["velocity"].get<double>();
                    if (item.contains("acceleration")) p.acceleration = item["acceleration"].get<double>();
                    if (item.contains("point_type")) p.point_type = item["point_type"].get<int>();
                    generalData_.trajData_.trajectoryPoints_.push_back(std::move(p));
                }
            }
            if (t.contains("sampled_boundary_points") && t["sampled_boundary_points"].is_array())
            {
                generalData_.trajData_.sampledBoundaryPoints_.clear();
                for (const auto& line : t["sampled_boundary_points"])
                {
                    std::vector<Eigen::Vector3d> linePoints;
                    if (line.is_array())
                    {
                        for (const auto& p : line)
                        {
                            Eigen::Vector3d point;
                            if (jsonToVec3d(p, point)) linePoints.push_back(point);
                        }
                    }
                    generalData_.trajData_.sampledBoundaryPoints_.push_back(std::move(linePoints));
                }
            }
        }

        if (j.contains("slicingData"))
        {
            const auto& s = j["slicingData"];
            if (s.contains("last_selected_layer_index")) generalData_.slicingData_.lastSelectedLayerIndex = s["last_selected_layer_index"].get<int>();
            if (s.contains("min_z_level")) generalData_.slicingData_.minZLevel_ = s["min_z_level"].get<double>();
            if (s.contains("max_z_level")) generalData_.slicingData_.maxZLevel_ = s["max_z_level"].get<double>();
            if (s.contains("cutting_planes") && s["cutting_planes"].is_array())
            {
                generalData_.slicingData_.cuttingPlanes_.clear();
                for (const auto& item : s["cutting_planes"])
                {
                    CuttingPlane cp;
                    if (item.contains("normal")) jsonToVec3d(item["normal"], cp.normal);
                    if (item.contains("center")) jsonToVec3d(item["center"], cp.center);
                    if (item.contains("layer_index")) cp.layer_index = item["layer_index"].get<int>();
                    if (item.contains("translation_x")) cp.translation_x = item["translation_x"].get<double>();
                    if (item.contains("translation_y")) cp.translation_y = item["translation_y"].get<double>();
                    if (item.contains("translation_z")) cp.translation_z = item["translation_z"].get<double>();
                    if (item.contains("rotation_x")) cp.rotation_x = item["rotation_x"].get<double>();
                    if (item.contains("rotation_y")) cp.rotation_y = item["rotation_y"].get<double>();
                    if (item.contains("rotation_z")) cp.rotation_z = item["rotation_z"].get<double>();
                    if (item.contains("is_selected")) cp.isSelected = item["is_selected"].get<bool>();
                    if (item.contains("vertices") && item["vertices"].is_array())
                    {
                        cp.vertices.clear();
                        for (const auto& v : item["vertices"])
                        {
                            Eigen::Vector3d vv;
                            if (jsonToVec3d(v, vv)) cp.vertices.push_back(vv);
                        }
                    }
                    generalData_.slicingData_.cuttingPlanes_.push_back(std::move(cp));
                }
            }
        }

        if (j.contains("boundaryData"))
        {
            const auto& b = j["boundaryData"];
            if (b.contains("z_max_boundary")) generalData_.boundaryData_.z_max_boundary_ = b["z_max_boundary"].get<double>();
            if (b.contains("z_min_boundary")) generalData_.boundaryData_.z_min_boundary_ = b["z_min_boundary"].get<double>();
            if (b.contains("upper_boundary")) jsonToBoundaryLine(b["upper_boundary"], generalData_.boundaryData_.upperBoundary_);
            if (b.contains("lower_boundary")) jsonToBoundaryLine(b["lower_boundary"], generalData_.boundaryData_.lowerBoundary_);
            if (b.contains("upper_boundary_offset")) jsonToBoundaryLine(b["upper_boundary_offset"], generalData_.boundaryData_.upperBoundaryOffset_);
            if (b.contains("lower_boundary_offset")) jsonToBoundaryLine(b["lower_boundary_offset"], generalData_.boundaryData_.lowerBoundaryOffset_);
            if (b.contains("boundary_lines") && b["boundary_lines"].is_array())
            {
                generalData_.boundaryData_.boundaryLines_.clear();
                for (const auto& item : b["boundary_lines"])
                {
                    BoundaryLine line;
                    jsonToBoundaryLine(item, line);
                    generalData_.boundaryData_.boundaryLines_.push_back(std::move(line));
                }
            }
        }

        if (j.contains("slicingData"))
        {
            const auto& s = j["slicingData"];
            if (s.contains("slice_lines") && s["slice_lines"].is_array())
            {
                generalData_.slicingData_.sliceLines_.clear();
                for (const auto& item : s["slice_lines"])
                {
                    BoundaryLine line;
                    jsonToBoundaryLine(item, line);
                    generalData_.slicingData_.sliceLines_.push_back(std::move(line));
                }
            }
        }

        return true;
    }

    void ModelTrajPlanManage::syncPublicToGeneralAfterLoad()
    {
        if (publicModelData_.hasValidData())
        {
            STLProcessor::copyFromPublicModel(publicModelData_.getModel(), generalData_.modelDataManager_);
            generalData_.hasOriginalData_ = true;
            generalData_.processingState_.isSTLProcessed = true;
        }

        if (publicTrajData_.hasValidData())
        {
            generalData_.trajData_.trajectoryPoints_.clear();
            for (const auto& [timestamp, pubPoint] : publicTrajData_.getTrajectoryPoints())
            {
                TrajectoryPoint p;
                p.pose = pubPoint.pose;
                p.spray_distance = pubPoint.spray_distance;
                p.velocity = pubPoint.velocity;
                p.timestamp = timestamp;
                generalData_.trajData_.trajectoryPoints_.push_back(p);
            }
            if (!generalData_.trajData_.trajectoryPoints_.empty())
            {
                generalData_.processingState_.isTrajectoryGenerated = true;
            }
        }
        else if (!generalData_.modelDataManager_.mainModelData_.m_vertices.empty())
        {
            auto& pubModel = publicModelData_.getModel();
            pubModel.m_vertices = generalData_.modelDataManager_.mainModelData_.m_vertices;
            pubModel.m_normals = generalData_.modelDataManager_.mainModelData_.m_normals;
            pubModel.m_indices = generalData_.modelDataManager_.mainModelData_.m_indices;
            pubModel.m_file_path = generalData_.modelDataManager_.mainModelData_.m_file_path;
            pubModel.m_file_name = generalData_.modelDataManager_.mainModelData_.m_file_name;
        }

        if (!publicTrajData_.hasValidData() && !generalData_.trajData_.trajectoryPoints_.empty())
        {
            auto& publicPts = publicTrajData_.getTrajectoryPoints();
            publicPts.clear();
            for (const auto& p : generalData_.trajData_.trajectoryPoints_)
            {
                PublicData::TrajectoryPoint pubPoint;
                pubPoint.pose = p.pose;
                pubPoint.spray_distance = p.spray_distance;
                pubPoint.velocity = p.velocity;
                publicPts[p.timestamp] = pubPoint;
            }
        }
    }

    bool ModelTrajPlanManage::initializeRendererAndRestoreVisibility(
        IBurTrajPlan& burPlan,
        const GeneralData& generalData,
        RenderData& renderData)
    {
        if (!burPlan.initializeRenderer(renderData))
        {
            return false;
        }

        const bool hasModel = !generalData.modelDataManager_.mainModelData_.m_vertices.empty();
        const bool hasSlices = !generalData.slicingData_.cuttingPlanes_.empty();
        const bool hasTraj = !generalData.trajData_.trajectoryPoints_.empty();

        burPlan.setOriginalDataVisibility(renderData, hasModel);
        burPlan.setBoundaryVisibility(renderData, hasSlices);
        burPlan.setCuttingPlanesVisibility(renderData, hasSlices);
        burPlan.setTrajectoryVisibility(renderData, hasTraj);
        burPlan.setConnectionVisibility(
            renderData, !generalData.trajData_.connectionData_.connectedTrajectory_.empty());

        return true;
    }

    void ModelTrajPlanManage::rebuildPublicRenderableData()
    {
        publicRenderData_.m_PublicSTLRenderData.needsUpdate = true;
        publicRenderData_.m_PublicTrajectoryRenderData.needsUpdate = true;

        if (publicRenderData_.m_PublicSTLRenderData.meshModel)
        {
            PublicData::PublicDataRender::drawPublicModel(
                publicRenderData_.m_PublicSTLRenderData, publicModelData_);
        }
        if (publicRenderData_.m_PublicTrajectoryRenderData.trajectoryModel)
        {
            PublicData::PublicDataRender::drawPublicTrajectory(
                publicRenderData_.m_PublicTrajectoryRenderData, publicTrajData_);
        }
    }
}
