#include <SlicingTrajPlan/STLProcessor.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdint>

int main()
{
    const auto path = std::filesystem::temp_directory_path() / "bur_traj_model_loading.stl";
    std::ofstream stream(path, std::ios::binary);
    char header[80] = {};
    const std::uint32_t triangleCount = 1;
    const float facet[] = { 0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f };
    const std::uint16_t attributeByteCount = 0;
    stream.write(header, sizeof(header));
    stream.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));
    stream.write(reinterpret_cast<const char*>(facet), sizeof(facet));
    stream.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));
    stream.close();
    ModelTrajPlan::ModelData model;
    ModelTrajPlan::STLProcessor processor;
    const bool loaded = processor.preprocessSTL(path.string(), model);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    if(!loaded || !ModelTrajPlan::STLProcessor::hasValidData(model) ||
        ModelTrajPlan::STLProcessor::getTriangleCount(model.mainModelData_) != 1) {
        std::cerr << "Model loading did not populate ModelData.\n";
        return 1;
    }
    return 0;
}
