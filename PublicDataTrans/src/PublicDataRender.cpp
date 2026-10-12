#include <PublicDataRender/PublicDataRender.h>

#include <algorithm>
#include <numeric>
#include <map>
#include <array>
#include <cmath>
#include <utility>

namespace PublicData
{
    void PublicDataRender::clearMeshData(MeshData& meshData)
    {
        meshData.m_vertices.clear();
        meshData.m_normals.clear();
        meshData.m_colors.clear();
        meshData.m_indices.clear();
    }

    bool PublicDataRender::initializeRender(iscene_spointer scene, imodel_spointer& meshModel, MODELTYPE modelType)
    {
        if (!scene) {
            return false;
        }
        // Already initialized.
        if (meshModel) {
            return true;
        }

        meshModel = scene->get_new_model(modelType);
        if (!meshModel) {
            return false;
        }

        return true;
    }

    PublicDataRender::PublicDataRender()
    {
    }

    PublicDataRender::~PublicDataRender()
    {
    }

    void PublicDataRender::convertPublicModelToMeshData(const PublicModel& model, MeshData& meshData)
    {
        meshData.m_vertices.clear();
        meshData.m_normals.clear();
        meshData.m_colors.clear();
        meshData.m_indices.clear();

        // COLOR_MESH vertex format: position + color (x,y,z,r,g,b,a)
        const size_t vertexCount = model.m_vertices.size() / 3;
        meshData.m_vertices.reserve(vertexCount * 7);

        // Default public model color for COLOR_MESH.
        const float r = 0.0f, g = 0.5f, b = 0.0f, a = 1.0f;

        for (size_t i = 0; i < vertexCount; ++i) {
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 0]);
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 1]);
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 2]);

            meshData.m_vertices.push_back(r);
            meshData.m_vertices.push_back(g);
            meshData.m_vertices.push_back(b);
            meshData.m_vertices.push_back(a);
        }

        meshData.m_indices = model.m_indices;
    }

    void PublicDataRender::convertPublicModelToColoredMeshData(
        const PublicModel& model,
        const std::vector<float>& vertexColors,
        MeshData& meshData)
    {
        meshData.m_vertices.clear();
        meshData.m_normals.clear();
        meshData.m_colors.clear();
        meshData.m_indices.clear();

        const size_t vertexCount = model.m_vertices.size() / 3;
        const bool hasNormals = model.m_normals.size() >= vertexCount * 3;
        const bool hasVertexColors = vertexColors.size() >= vertexCount * 3;

        meshData.m_vertices.reserve(vertexCount * 7);
        meshData.m_normals.reserve(vertexCount * 3);

        // COLOR_MESH lighting needs one normal per vertex. Some imported STL
        // paths do not carry normals, so build smooth vertex normals from the
        // indexed triangles instead of leaving the mesh effectively unlit.
        std::vector<float> generatedNormals;
        if (!hasNormals) {
            generatedNormals.assign(vertexCount * 3, 0.0f);
            for (size_t i = 0; i + 2 < model.m_indices.size(); i += 3) {
                const unsigned int ia = model.m_indices[i];
                const unsigned int ib = model.m_indices[i + 1];
                const unsigned int ic = model.m_indices[i + 2];
                if (ia >= vertexCount || ib >= vertexCount || ic >= vertexCount) continue;
                const glm::vec3 a(model.m_vertices[ia * 3], model.m_vertices[ia * 3 + 1], model.m_vertices[ia * 3 + 2]);
                const glm::vec3 b(model.m_vertices[ib * 3], model.m_vertices[ib * 3 + 1], model.m_vertices[ib * 3 + 2]);
                const glm::vec3 c(model.m_vertices[ic * 3], model.m_vertices[ic * 3 + 1], model.m_vertices[ic * 3 + 2]);
                const glm::vec3 n = glm::cross(b - a, c - a);
                for (const unsigned int idx : {ia, ib, ic}) {
                    generatedNormals[idx * 3] += n.x;
                    generatedNormals[idx * 3 + 1] += n.y;
                    generatedNormals[idx * 3 + 2] += n.z;
                }
            }
            for (size_t i = 0; i < vertexCount; ++i) {
                glm::vec3 n(generatedNormals[i * 3], generatedNormals[i * 3 + 1], generatedNormals[i * 3 + 2]);
                if (glm::length(n) < 1e-6f) n = glm::vec3(0.0f, 0.0f, 1.0f);
                n = glm::normalize(n);
                generatedNormals[i * 3] = n.x;
                generatedNormals[i * 3 + 1] = n.y;
                generatedNormals[i * 3 + 2] = n.z;
            }
        }

        for (size_t i = 0; i < vertexCount; ++i) {
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 0]);
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 1]);
            meshData.m_vertices.push_back(model.m_vertices[i * 3 + 2]);

            const float r = hasVertexColors ? vertexColors[i * 3 + 0] : 0.7f;
            const float g = hasVertexColors ? vertexColors[i * 3 + 1] : 0.7f;
            const float b = hasVertexColors ? vertexColors[i * 3 + 2] : 0.7f;
            meshData.m_vertices.push_back(std::clamp(r, 0.0f, 1.0f));
            meshData.m_vertices.push_back(std::clamp(g, 0.0f, 1.0f));
            meshData.m_vertices.push_back(std::clamp(b, 0.0f, 1.0f));
            meshData.m_vertices.push_back(1.0f);

            const auto& normals = hasNormals ? model.m_normals : generatedNormals;
            meshData.m_normals.push_back(normals[i * 3 + 0]);
            meshData.m_normals.push_back(normals[i * 3 + 1]);
            meshData.m_normals.push_back(normals[i * 3 + 2]);
        }
        meshData.m_indices = model.m_indices;
    }


    void PublicDataRender::convertPublicTrajToFrameMeshData(
        const PublicData::TrajectoryPoints& pts,
        float frameScale,
        MeshData& trajectoryData)
    {
        trajectoryData.m_vertices.clear();
        trajectoryData.m_indices.clear();

        for (const auto& kv : pts) {
            const Eigen::Matrix4d& pose = kv.second.pose;
            Eigen::Vector3d position = pose.block<3, 1>(0, 3);
            Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);

            Eigen::Vector3d xAxis = position + rotation.col(0) * static_cast<double>(frameScale);
            Eigen::Vector3d yAxis = position + rotation.col(1) * static_cast<double>(frameScale);
            Eigen::Vector3d zAxis = position + rotation.col(2) * static_cast<double>(frameScale);

            unsigned int baseIndex = static_cast<unsigned int>(trajectoryData.m_vertices.size() / 7);

            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                1.0f, 0.0f, 0.0f, 1.0f,
                static_cast<float>(xAxis.x()), static_cast<float>(xAxis.y()), static_cast<float>(xAxis.z()),
                1.0f, 0.0f, 0.0f, 1.0f
                });

            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                0.0f, 1.0f, 0.0f, 1.0f,
                static_cast<float>(yAxis.x()), static_cast<float>(yAxis.y()), static_cast<float>(yAxis.z()),
                0.0f, 1.0f, 0.0f, 1.0f
                });

            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                0.0f, 0.0f, 1.0f, 1.0f,
                static_cast<float>(zAxis.x()), static_cast<float>(zAxis.y()), static_cast<float>(zAxis.z()),
                0.0f, 0.0f, 1.0f, 1.0f
                });

            trajectoryData.m_indices.insert(trajectoryData.m_indices.end(), {
                baseIndex + 0, baseIndex + 1,
                baseIndex + 2, baseIndex + 3,
                baseIndex + 4, baseIndex + 5
                });
        }
    }

    bool PublicDataRender::drawPublicModel(
        PublicSTLRenderData& renderData,
        const PublicModelData& publicModelData)
    {
        if (!renderData.meshModel) {
            return false;
        }

        if (renderData.needsUpdate) {
            const PublicModel& model = publicModelData.getModel();
            convertPublicModelToMeshData(model, renderData.meshData);
            renderData.meshModel->import(renderData.meshData);
            renderData.needsUpdate = false;
        }

        return true;
    }

    bool PublicDataRender::drawPublicModelWithVertexColors(
        PublicSTLRenderData& renderData,
        const PublicModelData& publicModelData,
        const std::vector<float>& vertexColors)
    {
        if (!renderData.meshModel) {
            return false;
        }

        const PublicModel& model = publicModelData.getModel();
        convertPublicModelToColoredMeshData(model, vertexColors, renderData.meshData);
        renderData.meshModel->import(renderData.meshData);
        renderData.isVisible = true;
        renderData.needsUpdate = false;

        return true;
    }

    bool PublicDataRender::drawPublicTrajectory(
        PublicTrajectoryRenderData& renderData,
        const TrajData& publicTrajData,
        float frameScale)
    {
        if (!renderData.trajectoryModel) {
            return false;
        }

        if (renderData.needsUpdate) {
            const auto& normalPts = publicTrajData.getTrajectoryPoints();
            const auto& spiralPts = publicTrajData.getSpiralTrajectoryPoints();
            const auto& pts = normalPts.empty() ? spiralPts : normalPts;
            convertPublicTrajToFrameMeshData(pts, frameScale, renderData.trajectoryData);
            renderData.trajectoryModel->import(renderData.trajectoryData);
            renderData.needsUpdate = false;
        }

        return true;
    }


    bool PublicDataRender::drawPublicThicknessHeatmap(
        PublicThicknessRenderData& renderData,
        const PublicModelData& publicModelData,
        const std::vector<float>& vertexColors)
    {
        const PublicModel& model = publicModelData.getModel();
        const size_t vertexCount = model.m_vertices.size() / 3;
        if (vertexCount == 0 || vertexColors.size() != vertexCount * 3) {
            return false;
        }
        convertPublicModelToColoredMeshData(model, vertexColors, renderData.heatmapMeshData);
        renderData.vertexThickness.clear();
        renderData.isVisible = true;
        renderData.needsUpdate = true;

        return true;
    }


    bool PublicDataRender::drawPublicThicknessHeatmap(
        PublicThicknessRenderData& renderData,
        const PublicModelData& publicModelData,
        const std::vector<float>& vertexColors,
        const std::vector<double>& thicknessValues,
        float displacementScale)
    {
        const PublicModel& src = publicModelData.getModel();
        const size_t srcCount = src.m_vertices.size() / 3;
        if (srcCount == 0 || vertexColors.size() != srcCount * 3 || thicknessValues.size() != srcCount) return false;

        // Rebuild a shared-vertex mesh. STL often stores three independent
        // vertices per triangle; those must map to one logical surface point.
        PublicModel mesh = src;
        mesh.m_vertices.clear();
        mesh.m_normals.clear();
        mesh.m_indices.resize(src.m_indices.size());
        std::map<std::array<int,3>, unsigned int> lookup;
        std::vector<double> sums;
        std::vector<float> colorSums;
        std::vector<unsigned int> counts;
        for (size_t i=0; i<srcCount; ++i) {
            std::array<int,3> key{(int)std::llround(src.m_vertices[i*3]*10000.0f),(int)std::llround(src.m_vertices[i*3+1]*10000.0f),(int)std::llround(src.m_vertices[i*3+2]*10000.0f)};
            auto it=lookup.find(key); unsigned int u;
            if(it==lookup.end()){u=(unsigned int)(mesh.m_vertices.size()/3); lookup.emplace(key,u); mesh.m_vertices.insert(mesh.m_vertices.end(),{src.m_vertices[i*3],src.m_vertices[i*3+1],src.m_vertices[i*3+2]}); sums.push_back(0); colorSums.insert(colorSums.end(),{0,0,0}); counts.push_back(0);} else u=it->second;
            sums[u]+=thicknessValues[i]; colorSums[u*3]+=vertexColors[i*3]; colorSums[u*3+1]+=vertexColors[i*3+1]; colorSums[u*3+2]+=vertexColors[i*3+2]; counts[u]++;
        }
        for(size_t i=0;i<src.m_indices.size();++i){unsigned int old=src.m_indices[i]; if(old>=srcCount)return false; std::array<int,3> key{(int)std::llround(src.m_vertices[old*3]*10000.0f),(int)std::llround(src.m_vertices[old*3+1]*10000.0f),(int)std::llround(src.m_vertices[old*3+2]*10000.0f)}; mesh.m_indices[i]=lookup[key];}
        const size_t n=mesh.m_vertices.size()/3; std::vector<double> thick(n); std::vector<float> colors(n*3); for(size_t i=0;i<n;++i){thick[i]=sums[i]/counts[i];colors[i*3]=colorSums[i*3]/counts[i];colors[i*3+1]=colorSums[i*3+1]/counts[i];colors[i*3+2]=colorSums[i*3+2]/counts[i];}
        // Blend the scalar field over the reconstructed mesh adjacency. The
        // analyzer samples a triangle soup, so per-vertex colors otherwise
        // preserve pass-shaped bands even when trajectories are very dense.
        std::vector<std::vector<unsigned int>> neighbors(n);
        for(size_t i=0;i+2<mesh.m_indices.size();i+=3){unsigned int a=mesh.m_indices[i],b=mesh.m_indices[i+1],c=mesh.m_indices[i+2]; neighbors[a].push_back(b);neighbors[a].push_back(c); neighbors[b].push_back(a);neighbors[b].push_back(c); neighbors[c].push_back(a);neighbors[c].push_back(b);}
        for(int pass=0;pass<2;++pass){std::vector<double> nextT=thick;std::vector<float> nextC=colors;for(size_t i=0;i<n;++i){if(neighbors[i].empty())continue;double t=thick[i];float r=colors[i*3],g=colors[i*3+1],b=colors[i*3+2];for(unsigned int j:neighbors[i]){t+=thick[j];r+=colors[j*3];g+=colors[j*3+1];b+=colors[j*3+2];}float inv=1.0f/static_cast<float>(neighbors[i].size()+1);nextT[i]=t*inv;nextC[i*3]=r*inv;nextC[i*3+1]=g*inv;nextC[i*3+2]=b*inv;}thick.swap(nextT);colors.swap(nextC);}
        mesh.m_normals.assign(n*3,0.0f);
        for(size_t i=0;i+2<mesh.m_indices.size();i+=3){unsigned int ia=mesh.m_indices[i],ib=mesh.m_indices[i+1],ic=mesh.m_indices[i+2];glm::vec3 a(mesh.m_vertices[ia*3],mesh.m_vertices[ia*3+1],mesh.m_vertices[ia*3+2]),b(mesh.m_vertices[ib*3],mesh.m_vertices[ib*3+1],mesh.m_vertices[ib*3+2]),c(mesh.m_vertices[ic*3],mesh.m_vertices[ic*3+1],mesh.m_vertices[ic*3+2]);glm::vec3 fn=glm::cross(b-a,c-a);for(unsigned int v:{ia,ib,ic}){mesh.m_normals[v*3]+=fn.x;mesh.m_normals[v*3+1]+=fn.y;mesh.m_normals[v*3+2]+=fn.z;}}
        for(size_t i=0;i<n;++i){glm::vec3 nn(mesh.m_normals[i*3],mesh.m_normals[i*3+1],mesh.m_normals[i*3+2]);if(glm::length(nn)<1e-6f)nn=glm::vec3(0,0,1);else nn=glm::normalize(nn);mesh.m_normals[i*3]=nn.x;mesh.m_normals[i*3+1]=nn.y;mesh.m_normals[i*3+2]=nn.z;float d=(float)std::max(0.0,thick[i])*displacementScale;mesh.m_vertices[i*3]+=nn.x*d;mesh.m_vertices[i*3+1]+=nn.y*d;mesh.m_vertices[i*3+2]+=nn.z*d;}
        convertPublicModelToColoredMeshData(mesh, colors, renderData.heatmapMeshData);
        renderData.vertexThickness = std::move(thick);
        renderData.isVisible = true;
        renderData.needsUpdate = true;
        return true;
    }}
