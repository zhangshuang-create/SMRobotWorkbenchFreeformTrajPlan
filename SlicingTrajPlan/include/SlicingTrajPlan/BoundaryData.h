#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <GeneralData/GeneralData_global.h>
namespace ModelTrajPlan
{
    // Boundary-line type enum - used to uniquely identify boundary-line type
    enum class BoundaryLineType : int {
        LowerBoundaryOffset = 0,  // Lower boundary offset line
        LowerBoundary = 1,        // Lower boundary line
        UpperBoundary = 2,        // Upper boundary line
        UpperBoundaryOffset = 3,  // Upper boundary offset line
        SlicePlane = 10           // Slice plane (reserved for future extension)
    };

    // Basic point data structure
    struct BasicPoint {
        Eigen::Vector3d position;
        double angle;  // Angle relative to center
        size_t index;  // Original index

        // Convenience accessors
        double x() const { return position.x(); }
        double y() const { return position.y(); }
        double z() const { return position.z(); }

    };


    struct PointInfo {
        Eigen::Vector3d pos;
        bool isBoundary;
    };
    struct CheckedSegment {
        PointInfo p1;
        PointInfo p2;
    };


    struct BasicPointComparator {
        bool operator()(const BasicPoint& a, const BasicPoint& b) const {
            // Same comparison logic as before: lexicographic order
            if (a.x() != b.x()) {
                return a.x() < b.x();
            }
            if (a.y() != b.y()) {
                return a.y() < b.y();
            }
            return a.z() < b.z();
        }
    };

    // Skeleton-line point data structure
    struct LinePoint {
        Eigen::Vector3d position;
        size_t index;  // Original index
        Eigen::Vector3d normal; // [Addition] Normal vector
        Eigen::Vector3d tangent;  // Tangent
        double curvature;
        // Convenience accessors
        double x() const { return position.x(); }
        double y() const { return position.y(); }
        double z() const { return position.z(); }

    };

    using Epiphyseal = std::vector<std::vector<LinePoint>>;

    // Basic boundary-line structure
    struct BoundaryLine {
        std::vector<BasicPoint> points;
        Eigen::Vector3d normal; // [Addition] Normal vector
        double z_level;  // Z height
        Eigen::Vector3d center;  // Center point
        // Color information
        Eigen::Vector3f color;  // RGB color values (0.0-1.0)
        std::string name;       // Boundary-line name
        BoundaryLineType Type;    // Boundary-line type
    };


    class GeneralData_SHAREDLIB BoundaryData
    {
    public:
        BoundaryData();
        ~BoundaryData();

        Epiphyseal intersectionPoints_;
        // The shorter of the two original curves produced by the one global
        // skeleton-cutting plane. It is the common phase-zero reference for
        // every closed contour, rather than a per-slice reference.
        std::vector<LinePoint> phaseReferenceLine_;
        Eigen::Vector3d phaseReferencePlaneNormal_ = Eigen::Vector3d::Zero();
        Epiphyseal LinePoints_;
        Epiphyseal Mainslicingnode;
        // Data members - public access, pure data container
        BoundaryLine upperBoundary_;           // Upper boundary line
        BoundaryLine lowerBoundary_;           // Lower boundary line
        BoundaryLine upperBoundaryOffset_;     // Upper boundary offset line
        BoundaryLine lowerBoundaryOffset_;     // Lower boundary offset line

        // Boundary range data
        double z_max_boundary_;                // Max Z boundary
        double z_min_boundary_;                // Min Z boundary
        double epsilon_;                       // Chord-error tolerance for adaptive node generation
        double minStep_;                       // Minimum spacing for adaptive node generation (mm)

        // Ordered boundary-line container - supports positional insertion
        std::vector<BoundaryLine> boundaryLines_;  // Ordered boundary-line list

        // Helper: find boundary line by ID
        // Returns pointer to boundary line, or nullptr if not found
        BoundaryLine* findBoundaryLineById(BoundaryLineType id);
        const BoundaryLine* findBoundaryLineById(BoundaryLineType id) const;

    private:

    };

}
