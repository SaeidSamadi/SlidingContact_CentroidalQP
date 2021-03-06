#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/LookAtSurfaceTask.h>

#include "CoMQP.h"
#include "ModifiedSupportPolygon.h"
#include "SupportPolygon.h"
#include "FrictionEstimator.h"

struct MC_CONTROL_DLLAPI WipingController : public mc_control::fsm::Controller
{
  WipingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  bool computeCoMQP();
  void setTargetFromCoMQP();
	void setFeetTargetFromCoMQP();

  CoMQP & comQP()
  {
    return comQP_;
  }
  ModifiedSupportPolygon & shiftedSupportPolygon()
  {
    return shiftedSupportPolygon_;
  }

  void addFootForceControl();
  void removeFootForceControl();

	void addHandForceControl();
	void removeHandForceControl();
private:
  void updateFootForceDifferenceControl();

public:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::force::CoPTask> admittanceTask;
  std::shared_ptr<mc_tasks::LookAtSurfaceTask> lookAtTask;
  FrictionEstimator frictionEstimator;

  std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask;
  double dfzAdmittance_ = 1e-4; /**< Admittance for vertical foot force control */
  double vdcDamping_ = 0.; /**< Vertical Drift Compensation damping */
  double vdcFrequency_ = 1.; /**< Vertical Drift Compensation frequency */
  sva::PTransformd lfTarget_, rfTarget_;

  bool comQPComputed = false;
private:
  mc_rtc::Configuration config_;
  SupportPolygon supportPolygon_;
  ModifiedSupportPolygon shiftedSupportPolygon_;
  CoMQP comQP_;

  bool useFeetForceControl_ = false;
	double comHeight_ = 0.0;
};
