cmake_minimum_required( VERSION 3.20 )

set( ${TARGET_NAME}_RequiredLibsPublic
    Eigen3::Eigen
    Qt5::Core
    Common::CustomLog
    SMRobotGen1::ModelTangQing
    SMRobotGen1::SMRobotTangQing
    SMRobotGen1::BasicSystemTangQing
    PublicData::ModelData
    PublicData::TrajData    
    PublicData::PublicThicknessData
    PublicData::PublicDataRender
)

set( ${TARGET_NAME}_RequiredLibsPrivate
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    GLEW::glew
)

