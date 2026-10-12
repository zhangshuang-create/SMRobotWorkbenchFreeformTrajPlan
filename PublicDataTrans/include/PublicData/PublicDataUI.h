#pragma once

#include <QtWidgets/QWidget>
#include <memory>
#include "PublicDataUI_global.h"
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>
#include <PublicDataRender/PublicDataRender.h>    // For PublicDataRender
#include <functional>
class BasicSystem;
using ibasicsystem_spointer = std::shared_ptr<BasicSystem>;

QT_BEGIN_NAMESPACE
namespace Ui { class PublicDataUI; };
QT_END_NAMESPACE
namespace PublicData {

    class PublicDataUI_SHAREDLIB PublicDataUI : public QWidget
    {
        Q_OBJECT

    public:
        PublicDataUI(QWidget* parent = nullptr);
        ~PublicDataUI();
        void set_system(const ibasicsystem_spointer& system);
        void set_public_data(PublicModelData* modelData, TrajData* trajData);
        void set_public_render(PublicDataRender* render);
        void setForceRefreshHook(const std::function<void()>& cb);
        void setModelClearedHook(const std::function<void()>& cb);
        void setTrajectoryClearedHook(const std::function<void()>& cb);
        void setThicknessClearedHook(const std::function<void()>& cb);

    private:
        Ui::PublicDataUI* ui;
        ibasicsystem_spointer m_spSystem;
        PublicModelData* m_publicModelData = nullptr;
        TrajData* m_publicTrajData = nullptr;
        PublicDataRender* m_publicDataRender = nullptr;
        std::function<void()> m_forceRefreshHook;
        std::function<void()> m_modelClearedHook;
        std::function<void()> m_trajectoryClearedHook;
        std::function<void()> m_thicknessClearedHook;
    private slots:
        void on_btn_clearModel_clicked();
        void on_btn_clearControlPoints_clicked();
        void on_btn_clearTrajectory_clicked();
        void on_btn_clearDeposit_clicked();
        void on_btn_clearThickness_clicked();

    };

}
using PublicDataUI = PublicData::PublicDataUI;
