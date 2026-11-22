#include <iostream>
#include <fstream>
#include <iomanip>
#include <SlicingTrajPlan/STLProcessor.h>
#include "../BoundarylessUniformSlicingTest/CoplanarContourChecks.h"
using namespace ModelTrajPlan;

static void checkAverageNormals()
{
    auto model = coplanar_checks::cylinder();
    auto& vertices = model.mainModelData_.m_vertices;
    // The upper half tapers while the lower half stays vertical.
    for (size_t i = 0; i < vertices.size(); i += 3) {
        if (vertices[i + 2] == 1000.0f) {
            vertices[i] = 300.0f + 0.4f * (vertices[i] - 300.0f);
            vertices[i + 1] = 300.0f + 0.4f * (vertices[i + 1] - 300.0f);
        }
    }
    std::vector<CuttingPlane> planes(4);
    for (size_t i = 0; i < planes.size(); ++i) {
        planes[i].center = Eigen::Vector3d(300, 300, 125 + 250 * i);
        planes[i].normal = Eigen::Vector3d::UnitZ();
    }
    SliceResult rings;
    coplanar_checks::require(UniformSlicer::convertToCuttingPlanes(rings, &planes, &model), "Tapered ring setup failed");
    SliceResult slices = rings;
    slices.internalBoundaries = {rings.internalBoundaries[1], rings.internalBoundaries[2]};
    slices.usedParams.numTrajectories = 8;
    slices.usedParams.trajectorySpacing = 30;
    const auto generate = [&](int mode) {
        TrajectoryGenerationParams params;
        params.normalDirectionMode = mode;
        params.forwardDirectionMode = 2;
        params.z1Offset = params.z2Offset = params.offsetDistance = 0.0;
        BoundaryData boundaries;
        SlicingData slicing;
        TrajData data;
        TrajectoryGroupInfo groups;
        std::vector<TrajectoryPoint> points;
        TrajectoryGenerator generator;
        coplanar_checks::require(generator.generateMATLABTrajectory(rings.internalBoundaries.back(),
            rings.internalBoundaries.front(), slices, params, &model, points, groups, boundaries, slicing, data),
            "Tapered trajectory generation failed");
        return points;
    };
    const auto original = generate(0);
    const auto limited = generate(2);
    coplanar_checks::require(original.size() == limited.size(), "Normal mode changed the sample count");
    bool kept = false, clamped = false;
    const auto tilt = [](const TrajectoryPoint& p) {
        return std::atan2(std::abs(p.pose(2, 2)), p.pose.block<2, 1>(0, 2).norm());
    };
    for (size_t first = 0, end = 0; first < original.size(); first = end) {
        double mean = 0.0;
        for (end = first; end < original.size() && original[end].trajectory_index == original[first].trajectory_index; ++end) {
            mean += tilt(original[end]);
        }
        mean /= static_cast<double>(end - first);
        for (size_t i = first; i < end; ++i) {
            const auto frame = limited[i].pose.block<3, 3>(0, 0);
            coplanar_checks::require((frame.transpose() * frame).isIdentity(1e-8) &&
                std::abs(frame.determinant() - 1.0) < 1e-8, "Average-normal frame is not orthonormal");
            const double initial = tilt(original[i]);
            coplanar_checks::require(std::abs(tilt(limited[i]) - std::min(initial, mean)) < 1e-8,
                "Point did not honor its trajectory's mean tilt");
            if (initial < mean - 1e-6) {
                kept = true;
                coplanar_checks::require(frame.col(2).isApprox(original[i].pose.block<3, 1>(0, 2)),
                    "In-range original normal changed");
            }
            clamped = clamped || initial > mean + 1e-6;
        }
    }
    coplanar_checks::require(kept && clamped, "Fixture did not exercise both sides of the mean");
}

static void checkProjectedSprayTargets(const char* meshPath, const char* samplePath)
{
    auto model = coplanar_checks::cylinder();
    const Eigen::AngleAxisd tilt(0.8, Eigen::Vector3d::UnitY());
    auto& vertices = model.mainModelData_.m_vertices;
    for (size_t i = 0; i < vertices.size(); i += 3) {
        Eigen::Vector3d v(vertices[i], vertices[i+1], vertices[i+2]);
        if (v.z() > 900) v.head<2>() = (0.25 * (v.head<2>().array() - 300.0) + 300.0).matrix();
        v = tilt * v;
        for (int axis = 0; axis < 3; ++axis) vertices[i+axis] = static_cast<float>(v[axis]);
    }
    std::vector<CuttingPlane> planes(4);
    for (size_t i = 0; i < planes.size(); ++i) {
        planes[i].center = tilt * Eigen::Vector3d(300, 300, 125 + 250 * i);
        planes[i].normal = tilt * Eigen::Vector3d::UnitZ();
    }
    if (meshPath) {
        model = ModelData();
        STLProcessor processor;
        coplanar_checks::require(processor.preprocessSTL(meshPath, model), "Cannot load workpiece");
        const Eigen::Vector3d axis = Eigen::Vector3d(0.1, 0.0, 1.0).normalized();
        const auto& xyz = model.mainModelData_.m_vertices;
        double lo = std::numeric_limits<double>::max(), hi = -lo;
        for (size_t i = 0; i < xyz.size(); i += 3) {
            const double height = axis.dot(Eigen::Vector3d(xyz[i], xyz[i+1], xyz[i+2]));
            lo = std::min(lo, height);
            hi = std::max(hi, height);
        }
        for (size_t i = 0; i < planes.size(); ++i) {
            planes[i].center = axis * (lo + (hi-lo) * (i+1) / (planes.size()+1));
            planes[i].normal = axis;
        }
    }
    SliceResult rings;
    coplanar_checks::require(UniformSlicer::convertToCuttingPlanes(rings, &planes, &model), "Tilted ring setup failed");
    auto slices = rings;
    slices.internalBoundaries = {rings.internalBoundaries[1], rings.internalBoundaries[2]};
    slices.usedParams.numTrajectories = 32;
    slices.usedParams.trajectorySpacing = 30;
    TrajectoryGenerationParams params;
    params.normalAxis = 5; params.forwardAxis = 0; params.forwardDirectionMode = 2;
    params.z1Offset = params.z2Offset = params.offsetDistance = 100;
    params.upperTransitionSmoothPose = true;
    TrajData data; BoundaryData boundaries; SlicingData slicing; TrajectoryGroupInfo groups;
    TrajectoryGenerator generator;
    std::vector<TrajectoryPoint> surface, projected;
    const auto generate = [&](std::vector<TrajectoryPoint>& output) {
        return generator.generateMATLABTrajectory(rings.internalBoundaries.back(), rings.internalBoundaries.front(),
            slices, params, &model, output, groups, boundaries, slicing, data);
    };
    coplanar_checks::require(generate(surface), "Surface trajectory failed");
    params.normalDirectionMode = 1;
    coplanar_checks::require(generate(projected) && projected.size() == surface.size(), "Projected trajectory failed");
    std::ofstream samples;
    if (samplePath) {
        samples.open(samplePath);
        coplanar_checks::require(samples.good(), "Cannot create projected samples file");
        samples << std::setprecision(17);
    }
    double worstMiss = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const auto& p = projected[i];
        const Eigen::Matrix3d frame = p.pose.block<3,3>(0,0);
        const Eigen::Vector3d spray = -(Eigen::Quaterniond(frame).normalized() * Eigen::Vector3d::UnitZ());
        const auto& target = data.sampledBoundaryPoints_.at(p.trajectory_index).at(p.layer_index);
        const Eigen::Vector3d hit = p.pose.block<3,1>(0,3) + p.spray_distance * spray;
        worstMiss = std::max(worstMiss, (hit - target).norm());
        coplanar_checks::require((frame.transpose() * frame).isIdentity(1e-9) && std::abs(frame.determinant()-1) < 1e-9,
            "Projected tool frame is not a rotation");
        if (samplePath) {
            const Eigen::Quaterniond rotation(frame);
            samples << p.trajectory_index << ' ' << p.layer_index << ' ' << p.point_type << ' '
                    << p.pose.block<3,1>(0,3).transpose() << ' ' << rotation.coeffs().transpose() << ' '
                    << target.transpose() << ' ' << p.spray_distance << '\n';
        }
        Eigen::Vector3d expected = -surface[i].pose.block<3,1>(0,2);
        expected.z() = 0;
        coplanar_checks::require(expected.norm() > 1e-8 && (-frame.col(2)).isApprox(expected.normalized(), 1e-7),
            "Projection changed the side of the surface");
    }
    std::cout << "Projected points: " << projected.size() << ", maximum spray target error (mm): " << worstMiss << '\n';
    coplanar_checks::require(worstMiss < 1e-7, "Exported horizontal spray axis misses the sampled target");
}

int main(int argc, char** argv) {
    try {
        const auto model=coplanar_checks::cylinder();
        std::vector<CuttingPlane> planes(3);
        for(int i=0;i<3;++i) {planes[i].center=Eigen::Vector3d(300,300,500*i);planes[i].normal=Eigen::Vector3d::UnitZ();}
        SliceResult rings;
        coplanar_checks::require(UniformSlicer::convertToCuttingPlanes(rings,&planes,&model),"Ring setup failed");
        SliceResult result=rings;
        result.internalBoundaries={rings.internalBoundaries[1]};
        result.isBoundarylessFallback=false;
        result.usedParams.numTrajectories=8;
        result.usedParams.trajectorySpacing=30;
        TrajectoryGenerator generator;
        for(bool swapped : {false,true}) {
            std::vector<TrajectoryPoint> positive;
            for(int mode : {0,1,2}) for(int axis : {0,1}) {
                TrajectoryGenerationParams parameters;
                parameters.forwardAxis=axis;
                parameters.forwardDirectionMode=mode;
                parameters.z1Offset=parameters.z2Offset=parameters.offsetDistance=0;
                BoundaryData boundaries;
                LinePoint a{},b{};a.position=Eigen::Vector3d(600,300,0);b.position=Eigen::Vector3d(600,300,1000);
                boundaries.phaseReferenceLine_={a,b};boundaries.phaseReferencePlaneNormal_=Eigen::Vector3d::UnitY();
                SlicingData slicing;TrajData data;TrajectoryGroupInfo groups;std::vector<TrajectoryPoint> points;
                coplanar_checks::require(generator.generateMATLABTrajectory(rings.internalBoundaries[swapped?0:2],rings.internalBoundaries[swapped?2:0],result,parameters,&model,points,groups,boundaries,slicing,data),"Generation failed");
                coplanar_checks::require(points.size()>3,"Not enough points");
                Eigen::Vector3d travel=points[1].pose.block<3,1>(0,3)-points[0].pose.block<3,1>(0,3);
                travel.normalize();
                const double expected=axis==0?1.0:-1.0;
                for(const auto& p:points) {
                    if (mode != 2) {
                        coplanar_checks::require(expected*p.pose.block<3,1>(0,0).dot(travel)>.95,"Selected X sign opposes first-stroke reference or flips on return");
                    }
                    coplanar_checks::require(std::abs(p.pose.block<3,3>(0,0).determinant()-1)<1e-8,"Frame is not right-handed");
                }
                if(axis==0 && mode==0)positive=points;
                else for(size_t i=0;i<points.size();++i) {
                    coplanar_checks::require(points[i].pose.block<3,1>(0,3).isApprox(positive[i].pose.block<3,1>(0,3)),"Axis sign changed trajectory positions");
                    coplanar_checks::require(points[i].pose.block<3,1>(0,2).isApprox(positive[i].pose.block<3,1>(0,2)),"Axis sign changed spray normal");
                }
            }
        }
        checkAverageNormals();
        checkProjectedSprayTargets(argc > 1 ? argv[1] : nullptr, argc > 2 ? argv[2] : nullptr);
        std::cout<<"Signed forward-axis and average-normal regressions passed\n";return 0;
    }catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
