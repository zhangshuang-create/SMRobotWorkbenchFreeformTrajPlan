#include <SlicingTrajPlan/BurTrajPlan.h>

#include <iostream>

int main()
{
    ModelTrajPlan::BurTrajPlan plan;
    ModelTrajPlan::GeneralData document;
    if(plan.connectTrajectoryPoints(document, ModelTrajPlan::ConnectionStrategy::BowPath)) {
        std::cerr << "An empty trajectory must not produce a connection.\n";
        return 1;
    }
    return 0;
}
