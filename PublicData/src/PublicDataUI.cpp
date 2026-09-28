#include <PublicDataUI/PublicDataUI.h>
#include "ui_PublicDataUI.h"
#include <QMessageBox>
#include <QPalette>
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>
#include <PublicThicknessData/PublicThicknessData.h>

namespace PublicData {
    PublicDataUI::PublicDataUI(QWidget* parent)
        : QWidget(parent)
        , ui(new Ui::PublicDataUI())
    {
        ui->setupUi(this);
    }

    void PublicDataUI::set_system(const ibasicsystem_spointer& system)
    {
        m_spSystem = system;
    }

    void PublicDataUI::set_public_data(PublicModelData* modelData, TrajData* trajData)
    {
        m_publicModelData = modelData;
        m_publicTrajData = trajData;
    }

    void PublicDataUI::set_public_render(PublicDataRender* render)
    {
        m_publicDataRender = render;
    }

    void PublicDataUI::setForceRefreshHook(const std::function<void()>& cb)
    {
        m_forceRefreshHook = cb;
    }

    void PublicDataUI::setModelClearedHook(const std::function<void()>& cb)
    {
        m_modelClearedHook = cb;
    }

    void PublicDataUI::setTrajectoryClearedHook(const std::function<void()>& cb)
    {
        m_trajectoryClearedHook = cb;
    }

    void PublicDataUI::setThicknessClearedHook(const std::function<void()>& cb)
    {
        m_thicknessClearedHook = cb;
    }

    PublicDataUI::~PublicDataUI()
    {
        delete ui;
    }
    void PublicDataUI::on_btn_clearModel_clicked()
    {
        if (!m_spSystem || !m_publicModelData) {
            QMessageBox::warning(this, "Warning", "System is not initialized.");
            return;
        }

        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Confirm");
        msgBox.setText("Are you sure you want to clear model data?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

        // set words black
        msgBox.setStyleSheet("QLabel{color: black;} QPushButton{color: black;}");

        if (msgBox.exec() == QMessageBox::Yes) {
            if (m_publicModelData) {
                m_publicModelData->clear();
            }
            if (m_publicDataRender) {
                PublicDataRender::clearMeshData(m_publicDataRender->m_PublicSTLRenderData.meshData);
                if (m_publicDataRender->m_PublicSTLRenderData.meshModel) {
                   m_publicDataRender->m_PublicSTLRenderData.meshModel->import(
                       m_publicDataRender->m_PublicSTLRenderData.meshData);
                }
                m_publicDataRender->m_PublicSTLRenderData.isVisible = false;
                m_publicDataRender->m_PublicSTLRenderData.needsUpdate = true;
            }
            if (m_modelClearedHook) {
                m_modelClearedHook();
            }
            if (m_forceRefreshHook) {
                m_forceRefreshHook();
            }
        }
    }

    // Clear Control Points
    void PublicDataUI::on_btn_clearControlPoints_clicked()
    {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Confirm");
        msgBox.setText("Are you sure you want to clear control points?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setStyleSheet("QLabel{color: black;} QPushButton{color: black;}");

        if (msgBox.exec() == QMessageBox::Yes) {
            if (m_publicTrajData) {
                m_publicTrajData->clear();
            }
            if (m_publicDataRender) {
                PublicDataRender::clearMeshData(m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryData);
                if (m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryModel) {
                    m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryModel->import(
                        m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryData);
                }
                m_publicDataRender->m_PublicTrajectoryRenderData.isVisible = false;
                m_publicDataRender->m_PublicTrajectoryRenderData.needsUpdate = true;
            }
            if (m_trajectoryClearedHook) {
                m_trajectoryClearedHook();
            }
            if (m_forceRefreshHook) {
                m_forceRefreshHook();
            }
        }
    }

    // Clear Trajectory Points
    void PublicDataUI::on_btn_clearTrajectory_clicked()
    {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Confirm");
        msgBox.setText("Are you sure you want to clear trajectory points?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setStyleSheet("QLabel{color: black;} QPushButton{color: black;}");

        if (msgBox.exec() == QMessageBox::Yes) {
            if (m_publicTrajData) {
                m_publicTrajData->getSpiralTrajectoryPoints().clear();
                m_publicTrajData->getTrajectoryPoints().clear();
            }
            if (m_publicDataRender) {
                PublicDataRender::clearMeshData(m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryData);
                if (m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryModel) {
                    m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryModel->import(
                        m_publicDataRender->m_PublicTrajectoryRenderData.trajectoryData);
                }
                m_publicDataRender->m_PublicTrajectoryRenderData.isVisible = false;
                m_publicDataRender->m_PublicTrajectoryRenderData.needsUpdate = true;
            }
            if (m_trajectoryClearedHook) {
                m_trajectoryClearedHook();
            }
            if (m_forceRefreshHook) {
                m_forceRefreshHook();
            }
        }
    }

    // Clear Deposition Data
    void PublicDataUI::on_btn_clearDeposit_clicked()
    {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Confirm");
        msgBox.setText("Are you sure you want to clear deposition data?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setStyleSheet("QLabel{color: black;} QPushButton{color: black;}");

        if (msgBox.exec() == QMessageBox::Yes) {
            if (m_forceRefreshHook) {
                m_forceRefreshHook();
            }
        }
    }

    void PublicDataUI::on_btn_clearThickness_clicked()
    {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Confirm");
        msgBox.setText("Are you sure you want to clear thickness prediction data?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setStyleSheet("QLabel{color: black;} QPushButton{color: black;}");

        if (msgBox.exec() == QMessageBox::Yes) {
            PublicData::globalThicknessPredictData().clear();
            if (m_thicknessClearedHook) {
                m_thicknessClearedHook();
            }
            if (m_forceRefreshHook) {
                m_forceRefreshHook();
            }
        }
    }

}
