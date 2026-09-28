cmake_minimum_required( VERSION 3.20 )
include(CMake_FindOptCuts)
set( ${TARGET_NAME}_RequiredLibsPublic Eigen3::Eigen TrajectoryInterpolation::InterpolationData PublicData::ProjectManager )
set( ${TARGET_NAME}_RequiredLibsPrivate )
