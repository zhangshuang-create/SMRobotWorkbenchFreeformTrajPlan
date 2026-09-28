#pragma once

#include <BurTrajPlan/BurTrajPlan_global.h>
#include <ProjectManager/IProjectSerializable.h>

#include <GeneralData/GeneralData.h>
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>
#include <PublicDataRender/PublicDataRender.h>
#include <BurTrajPlan/IBurTrajPlan.h>

namespace ModelTrajPlan
{
    class BurTrajPlan_SHAREDLIB ModelTrajPlanManage : public PublicData::IProjectSerializable
    {
    public:
        ModelTrajPlanManage(
            GeneralData& generalData,
            PublicData::PublicModelData& publicModelData,
            PublicData::TrajData& publicTrajData,
            PublicData::PublicDataRender& publicRenderData,
            IBurTrajPlan_spointer burTrajPlan,
            RenderData& renderData);

        bool save(const std::string& projectDir, nlohmann::json& root) override;
        bool load(const std::string& projectDir, const nlohmann::json& root) override;

        static bool initializeRendererAndRestoreVisibility(
            IBurTrajPlan& burPlan,
            const GeneralData& generalData,
            RenderData& renderData);

    private:
        bool saveGeneralData(const std::string& absPath) const;
        bool loadGeneralData(const std::string& absPath);
        void syncPublicToGeneralAfterLoad();
        void rebuildPublicRenderableData();

    private:
        GeneralData& generalData_;
        PublicData::PublicModelData& publicModelData_;
        PublicData::TrajData& publicTrajData_;
        PublicData::PublicDataRender& publicRenderData_;
        IBurTrajPlan_spointer burTrajPlan_;
        RenderData& renderData_;
    };
}
