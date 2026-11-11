#include <iostream>
#include "../BoundarylessUniformSlicingTest/CoplanarContourChecks.h"
using namespace ModelTrajPlan;
int main() {
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
            for(int axis : {0,1}) {
                TrajectoryGenerationParams parameters;
                parameters.forwardAxis=axis;
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
                    coplanar_checks::require(expected*p.pose.block<3,1>(0,0).dot(travel)>.95,"Selected X sign opposes first-stroke reference or flips on return");
                    coplanar_checks::require(std::abs(p.pose.block<3,3>(0,0).determinant()-1)<1e-8,"Frame is not right-handed");
                }
                if(axis==0)positive=points;
                else for(size_t i=0;i<points.size();++i) {
                    coplanar_checks::require(points[i].pose.block<3,1>(0,3).isApprox(positive[i].pose.block<3,1>(0,3)),"Axis sign changed trajectory positions");
                    coplanar_checks::require(points[i].pose.block<3,1>(0,2).isApprox(positive[i].pose.block<3,1>(0,2)),"Axis sign changed spray normal");
                }
            }
        }
        std::cout<<"Signed forward-axis regression passed\n";return 0;
    }catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
