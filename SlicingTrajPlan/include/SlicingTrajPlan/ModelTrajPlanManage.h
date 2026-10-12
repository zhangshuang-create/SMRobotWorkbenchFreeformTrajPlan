#pragma once

#include <BurTrajPlan/BurTrajPlan_global.h>
#include <ProjectManager/IProjectSerializable.h>

#include <GeneralData/GeneralData.h>
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>

namespace ModelTrajPlan
{
    class BurTrajPlan_SHAREDLIB ModelTrajPlanManage : public PublicData::IProjectSerializable
    {
    public:
        ModelTrajPlanManage(
            GeneralData& generalData,
            PublicData::PublicModelData& publicModelData,
            PublicData::TrajData& publicTrajData);

        bool save(const std::string& projectDir, nlohmann::json& root) override;
        bool load(const std::string& projectDir, const nlohmann::json& root) override;

    private:
        bool saveGeneralData(const std::string& absPath) const;
        bool loadGeneralData(const std::string& absPath);
        void syncPublicToGeneralAfterLoad();

    private:
        GeneralData& generalData_;
        PublicData::PublicModelData& publicModelData_;
        PublicData::TrajData& publicTrajData_;
    };
}
