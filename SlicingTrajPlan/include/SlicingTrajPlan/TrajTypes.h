#pragma once

// TrajTypes.h - Common type definitions for trajectory-planning system
// This file serves as a wrapper for common types and includes all trajectory-planning-related data structures

// 包含基础数据类型
#include "CommonDataTypes.h"
#include "TrajData.h"
#include "ModelData.h"
#include "SlicingData.h"
#include "BoundaryData.h"
namespace ModelTrajPlan
{
	// Re-export all common types for backward compatibility
	using ::InputDataType;
	using ::TrajectoryPoint;
	using ::MeshTriangle;
	using ::AABB;
	using ::CuttingPlane;
	using ::ConnectionStrategy;
	using ::ConnectionSegment;
	using ::SlicingParams;
	using ::SliceResult;
	using ::ModelData;
	using ::BoundaryLine;

	// Forward declarations
	class BoundaryDetector;
	class UniformSlicer;
	class TrajectoryGenerator;
	class TrajectoryConnector;
	class STLProcessor;

}