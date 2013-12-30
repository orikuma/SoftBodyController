// -*- C++ -*-
/*!
 * @file  SoftBodyController.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "SoftBodyController.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )

// Module specification
// <rtc-template block="module_spec">
static const char* softbodycontroller_spec[] =
{
  "implementation_id", "SoftBodyController",
  "type_name",         "SoftBodyController",
  "description",       "null component",
  "version",           "1.0",
  "vendor",            "AIST",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  // Configuration variables
  "conf.default.debugLevel", "1",
  ""
};

// without specialization, stringTo only convert 0/1 in bool
// namespace coil{
//   template <>
//   bool stringTo(bool& val, const char* str)
//   {
//     if (str == 0) { return false; }
//     std::stringstream s;
//     if ((s << str).fail()) { return false; }
//     if ((s >> std::boolalpha >> val).fail()) { return false; }
//     return true;
//   }
// }

// </rtc-template>

SoftBodyController::SoftBodyController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauIn("tau", m_tau),
    m_TorqueControllerServicePort("TorqueControllerService"),
    // </rtc-template>
    m_debugLevel(1)
{
}

SoftBodyController::~SoftBodyController()
{
}

RTC::ReturnCode_t SoftBodyController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "1");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("tau", m_tauIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_TorqueControllerServicePort.registerConsumer("service0", "TorqueControllerService", m_TorqueControllerService0);
  
  // Set CORBA Service Ports
  addPort(m_TorqueControllerServicePort);
  
  // </rtc-template>
  
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  // make robot model
  m_robot = hrp::BodyPtr(new hrp::Body());
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
        )){
    std::cerr << "failed to load model[" << prop["model"] << "] in "
              << m_profile.instance_name << std::endl;
    return RTC::RTC_ERROR;
  }

  coil::vstring frictionParamsFromConf = coil::split(prop["friction_params"], ",");
  m_frictionCoeffs.resize(m_robot->numJoints());
  if (frictionParamsFromConf.size() == m_robot->numJoints()) {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_frictionCoeffs[i], frictionParamsFromConf[i].c_str());
    }
  } else { // default
    std::cerr << "[WARNING] friction params are not correct number, " << frictionParamsFromConf.size() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_frictionCoeffs[i] = 0.01;
    }
  }

  coil::vstring torqueMarginFromConf = coil::split(prop["torque_margin"], ",");
  m_torqueMargin.resize(m_robot->numJoints());
  if (torqueMarginFromConf.size() == m_robot->numJoints()) {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_torqueMargin[i], torqueMarginFromConf[i].c_str());
    }
  } else { // default
    std::cerr << "[WARNING] torque margin are not correct number, " << frictionParamsFromConf.size() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_torqueMargin[i] = 5.0;
    }
  }
  
  m_robot->initializeConfiguration();
  
  return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t SoftBodyController::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SoftBodyController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftBodyController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftBodyController::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;

  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;

  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauIn.isNew()) {
    m_tauIn.read();
  }
  
  if ( m_qCurrent.data.length() == m_robot->numJoints()
       && m_tau.data.length() == m_robot->numJoints()) {

    // update joint angles
    hrp::dvector tmp_dq(m_robot->numJoints());
    hrp::dvector tmp_ddq(m_robot->numJoints());
    for(int i = 0; i < m_robot->numJoints(); i++) {
      tmp_dq[i] = (m_qCurrent.data[i] - m_robot->joint(i)->q) / m_dt;
      tmp_ddq[i] = (tmp_dq[i] - m_robot->joint(i)->dq) / m_dt;
      m_robot->joint(i)->q = m_qCurrent.data[i];
      m_robot->joint(i)->dq = tmp_dq[i];
      m_robot->joint(i)->ddq = tmp_ddq[i];
    }

    if ( DEBUGP ) {
      std::string prefix = "[SoftBodyController]";
      std::cerr << prefix << "q:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << m_robot->joint(i)->q;
      }
      std::cerr << std::endl;
      std::cerr << prefix << "dq:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << m_robot->joint(i)->dq;
      }
      std::cerr << std::endl;
      std::cerr << prefix << "ddq:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << m_robot->joint(i)->ddq;
      }
      std::cerr << std::endl;
    }

    // update reference robot model
    m_robot->calcForwardKinematics(true, true);
    m_robot->calcCM();
    m_robot->rootLink()->calcSubMassCM();

    // calc inertia
    hrp::dvector inertia_torque(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      // tau = J * ddq
      // inertia_torque[i] = m_robot->joint(i)->Ir * m_robot->joint(i)->ddq + inertiaTorque;

      // inertia = rotorInertia + InertiaAroundJointAxis
      hrp::Vector3 cog_world = m_robot->joint(i)->submwc / m_robot->joint(i)->subm - m_robot->joint(i)->p;
      double inertia = m_robot->joint(i)->Ir + (m_robot->joint(i)->subm * cog_world.squaredNorm());
      inertia_torque[i] = inertia * m_robot->joint(i)->ddq;
    }

    // calc friction
    hrp::dvector friction_torque(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      // B * dq
      friction_torque[i] = m_frictionCoeffs[i] * m_robot->joint(i)->dq;
    }
    
    // calc gravity compensation of each joints
    hrp::Vector3 g(0, 0, 9.8);
    hrp::dvector gravity_compensation(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      // (submwc/subm - p) x subm*g . R*a
      // subm: mass, g: gravity, submwc/subm: cog in worldcoords, p: pos in worldcoords, R: posture, a: axis in worldcoords
      // gravity_compensation[i] = (m_robot->joint(i)->submwc / m_robot->joint(i)->subm - m_robot->joint(i)->p).cross(m_robot->joint(i)->subm*g).dot(m_robot->joint(i)->R * m_robot->joint(i)->a);
      hrp::Vector3 cog_world = (m_robot->joint(i)->submwc / m_robot->joint(i)->subm) - m_robot->joint(i)->p;
      hrp::Vector3 mg = m_robot->joint(i)->subm*g;
      hrp::Vector3 axis_world = m_robot->joint(i)->R * m_robot->joint(i)->a;
      gravity_compensation[i] = (cog_world.cross(mg)).dot(axis_world);
    }

    // decide dist_tau
    hrp::dvector dist_tau(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      dist_tau[i] = inertia_torque[i] + friction_torque[i] + gravity_compensation[i];
    }

    // consider torque margin
    hrp::dvector actual_dist_tau(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      if (std::abs(dist_tau[i] - m_tau.data[i]) < m_torqueMargin[i]){
        actual_dist_tau[i] = m_tau.data[i];
      } else {
        actual_dist_tau[i] = dist_tau[i];
      }
      m_TorqueControllerService0->setReferenceTorque(m_robot->joint(i)->name.c_str(), actual_dist_tau[i]);
    }
    
    if ( DEBUGP ) {
      std::string prefix = "[SoftBodyController]";
      std::cerr << prefix << "inertia:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << inertia_torque[i];
      }
      std::cerr << std::endl;
      std::cerr << prefix << "friction:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << friction_torque[i];
      }
      std::cerr << std::endl;
      std::cerr << prefix << "gravity:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << gravity_compensation[i];
      }
      std::cerr << std::endl;
      std::cerr << prefix << "dist_tau:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << dist_tau[i];
      }
      std::cerr << std::endl;
      std::cerr << prefix << "actual_dist_tau:";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << actual_dist_tau[i];
      }
      std::cerr << std::endl;
    }
    
  }
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SoftBodyController::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SoftBodyController::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

  void SoftBodyControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(softbodycontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<SoftBodyController>,
                             RTC::Delete<SoftBodyController>);
  }

};


