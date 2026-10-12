#pragma once
// ���� BoundaryLine ����������
#include <SlicingTrajPlan/BoundaryData.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <SlicingTrajPlan/GeneralData_global.h>
namespace ModelTrajPlan
{
    enum class SlicingMode : int {
        AA = 0,
        Parallel = 1
    };

    // Cutting-plane structure - complete geometric definition
    struct CuttingPlane {
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();// Normal vector
        Eigen::Vector3d center = Eigen::Vector3d::Zero();// Center point
        int layer_index = 0; // Layer index
        // [Addition] Complete geometry for rectangular plane
        std::vector<Eigen::Vector3d> vertices;// Four vertex coordinates, defining plane size

        // Store original transform parameters to avoid back-solving from normal vectors
        double translation_x = 0.0;     // X-axis translation
        double translation_y = 0.0;     // Y-axis translation
        double translation_z = 0.0;     // Z-axis translation
        double rotation_x = 0.0;        // X-axis rotation (degree)
        double rotation_y = 0.0;        // Y-axis rotation (degree)
        double rotation_z = 0.0;        // Z-axis rotation (degree)
        // [Addition] Whether selected for highlight
        bool isSelected = false;

        // Constructor: initialize vertex array
        CuttingPlane() {
            vertices.resize(4);  // Pre-allocate 4 vertices
        }
    };
    struct SlicePlane//����CuttingPlane
    {
        Eigen::Vector3d point;   // ƽ����һ�㣨���е㣩
        Eigen::Vector3d normal;  // ƽ�淨��E��
    };
    using SlicingLocation = std::vector<SlicePlane>;
    // [Refactor] Slicing parameter structure - contains slicing-related parameters only, no offset parameters
    struct SlicingParams {
        int numTrajectories = 0;       // 0 derives the count from spacing; positive values override it
        double trajectorySpacing = 1.0; // Requested contour spacing d (mm)
        double minStep = 50.0;         // Minimum Z step used by parallel slicing (mm)
        int maxSliceCount = 0;         // Final slice-layer limit; 0 means unlimited
        SlicingMode mode = SlicingMode::AA;
        std::vector<double> customZPositions;  // Custom Z slicing positions

        // [Addition] Preset configuration helpers
        void applyCoarse() {
            numTrajectories = 100;
        }

        void applyFine() {
            numTrajectories = 30;
        }

        void applyDefault() {
            numTrajectories = 0;
            customZPositions.clear();
        }
    };

    // Slicing result - moved from UniformSlicer.h
    // [Refactor] UniformSlicer only handles geometric intersections, not normals or poses
    struct SliceResult {
        std::vector<BoundaryLine> internalBoundaries;              // Internal boundaries (position data only)
        SlicingParams usedParams;                                  // Actually used slicing parameters
        // True when planes are derived from the STL Z range because the two
        // open boundaries and their skeleton are unavailable.
        bool isBoundarylessFallback = false;
    };

    class GeneralData_SHAREDLIB SlicingData
    {
    public:
        SlicingData();
        ~SlicingData();
        int lastSelectedLayerIndex = -1;  // Record last selected layer index
        // Data members - public access, pure data container
        SliceResult currentSliceResult_;                  // Current slicing result
        std::vector<CuttingPlane> cuttingPlanes_;         // Cutting-plane collection
        SlicingLocation slicingLocations_;
        
        // Ordered slicing-boundary container - supports positional insertion

        std::vector<BoundaryLine> sliceLines_;           // Ordered slicing-boundary list (boundary line per slicing layer)
        // Whether updated
        bool slicingisupdata = false;
        // [Refactor] Unified slicing-parameter management
        SlicingParams managedParams_;                     // Managed slicing parameters (unified as SlicingParams)

        // Slicing-range data
        double minZLevel_;                                // Minimum Z level
        double maxZLevel_;                                // Maximum Z level

    private:

    };


}
