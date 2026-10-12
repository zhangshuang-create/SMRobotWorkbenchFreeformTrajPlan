#include <SlicingTrajPlan/BoundaryDetector.h>
#include <SlicingTrajPlan/STLProcessor.h>

#include <iostream>

int main(int argc, char** argv)
{
    ModelTrajPlan::BoundaryDetector detector;
    ModelTrajPlan::ModelData model;
    ModelTrajPlan::BoundaryData boundaries;
    ModelTrajPlan::Epiphyseal intersections;
    if(argc > 1) {
        ModelTrajPlan::STLProcessor loader;
        if(!loader.preprocessSTL(argv[1], model)) return 3;
        const bool detected = detector.detectBoundaries(model, boundaries, intersections);
        std::cout << "Detected=" << detected << " upper=" << boundaries.upperBoundary_.points.size()
            << " lower=" << boundaries.lowerBoundary_.points.size()
            << " intersections=" << intersections.size() << " references=" << boundaries.referenceLines_.size()
            << " normal=" << boundaries.phaseReferencePlaneNormal_.transpose() << std::endl;
        return detected && !boundaries.referenceLines_.empty() ? 0 : 4;
    }
    // An empty algorithm model has no boundary; this guards the document-only
    // contract without bringing rendering DTOs back into the algorithm test.
    if(detector.detectBoundaries(model, boundaries, intersections)) {
        std::cerr << "Boundary detection accepted an empty algorithm model.\n";
        return 1;
    }
    detector.clear(boundaries);
    return boundaries.upperBoundary_.points.empty() && boundaries.lowerBoundary_.points.empty() ? 0 : 2;
}
