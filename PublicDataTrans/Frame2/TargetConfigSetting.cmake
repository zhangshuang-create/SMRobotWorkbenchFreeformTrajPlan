cmake_minimum_required( VERSION 3.20 )

set( ${TARGET_NAME}_RequiredLibsPublic
        Eigen3::Eigen
        SMRobotGen1::ModelTangQing
        SMRobotGen1::SMRobotTangQing
)

set( ${TARGET_NAME}_RequiredLibsPrivate
        Boost::headers
        glm::glm
        FreeGLUT::freeglut
)

