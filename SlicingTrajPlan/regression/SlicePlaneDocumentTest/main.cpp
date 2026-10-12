#include <SlicingTrajPlan/BurTrajPlan.h>

#include <iostream>

int main()
{
    ModelTrajPlan::BurTrajPlan plan;
    ModelTrajPlan::GeneralData document;
    document.slicingData_.cuttingPlanes_.resize(1);
    auto& plane = document.slicingData_.cuttingPlanes_.front();
    plane.translation_x = 10.0; plane.translation_y = -20.0; plane.translation_z = 30.0;
    plane.rotation_x = 10.0; plane.rotation_y = 20.0; plane.rotation_z = 30.0;
    document.slicingData_.slicingisupdata = true;
    if(plane.translation_x != 10.0 || plane.rotation_z != 30.0 ||
        !document.slicingData_.slicingisupdata) {
        std::cerr << "Confirmed slicing transform was not retained by GeneralData.\n";
        return 1;
    }
    // A geometry refresh must reject a plane without an imported model.
    return plan.updateSingleCuttingPlaneSize(document, 0) ? 2 : 0;
}
