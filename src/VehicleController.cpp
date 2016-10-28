// -*- C++ -*-
/*!
 * @file  VehicleController.cpp
 * @brief Vehicle Controller
 * @date $Date$
 *
 * @author 宮本　信彦　n-miyamoto@aist.go.jp
 * 産業技術総合研究所　ロボットイノベーション研究センター
 * ロボットソフトウエアプラットフォーム研究チーム
 *
 * $Id$
 */

#include "VehicleController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* vehiclecontroller_spec[] =
  {
    "implementation_id", "VehicleController",
    "type_name",         "VehicleController",
    "description",       "Vehicle Controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Sensor",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.rotation_by_angle", "-0.6",
    "conf.default.velocity_by_angle", "-0.1",

    // Widget
    "conf.__widget__.rotation_by_angle", "text",
    "conf.__widget__.velocity_by_angle", "text",
    // Constraints

    "conf.__type__.rotation_by_angle", "double",
    "conf.__type__.velocity_by_angle", "double",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
VehicleController::VehicleController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_outOut("out", m_out)

    // </rtc-template>
{
	large_motor0 = NULL;
	large_motor1 = NULL;
}

/*!
 * @brief destructor
 */
VehicleController::~VehicleController()
{
}



RTC::ReturnCode_t VehicleController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("rotation_by_angle", m_rotation_by_angle, "-0.6");
  bindParameter("velocity_by_angle", m_velocity_by_angle, "-0.1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onFinalize()
{
	if(large_motor0)
	{
		if(large_motor0->connected())
		{
			large_motor0->reset();
		}
		delete large_motor0;
	}

	if(large_motor1)
	{
		if(large_motor1->connected())
		{
			large_motor1->reset();
		}
		delete large_motor1;
	}
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


/**
* @brief モーターの位置取得
* @param m モーター
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float VehicleController::get_position_motor(ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)m->position()*M_PI/180.0;
	}
	ret = false;
	return 0;
}


/**
* @brief モーターのカウントを角度に変換
* @param count カウント
* @param m モーター
* @param ret 変換できた場合はtrue、失敗した場合はfalse
* @return 角度
*/
float VehicleController::count_to_rot(int count, ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)count/(180/M_PI)/((float)m->count_per_rot()/360.0);
	}
	ret = false;
	return 0;
}


RTC::ReturnCode_t VehicleController::onActivated(RTC::UniqueId ec_id)
{


	if(large_motor0 == NULL)
	{
		large_motor0 = new ev3dev::large_motor(LARGE_MOTOR0_ADDRESS);
		if(large_motor0->connected())
		{
			
		}
		else
		{
			delete large_motor0;
			large_motor0 = NULL;
			return RTC::RTC_ERROR;
		}

	}

	if(large_motor1 == NULL)
	{
		large_motor1 = new ev3dev::large_motor(LARGE_MOTOR1_ADDRESS);
		if(large_motor1->connected())
		{
			
		}
		else
		{
			delete large_motor1;
			large_motor1 = NULL;
			return RTC::RTC_ERROR;
		}

	}

	large_motor0->reset();
	large_motor1->reset();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onExecute(RTC::UniqueId ec_id)
{
	bool ret = false;
	float pos0 = get_position_motor(large_motor0, ret);
	if(ret == false)return RTC::RTC_ERROR;

	float pos1 = get_position_motor(large_motor1, ret);
	if(ret == false)return RTC::RTC_ERROR;

	//std::cout << pos0 << "\t" << pos1 << std::endl;



	m_out.data.vx = m_velocity_by_angle*pos1;
	m_out.data.vy = 0;
	m_out.data.va = m_rotation_by_angle*pos0;

	setTimestamp(m_out);

	m_outOut.write();
	
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t VehicleController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}



extern "C"
{
 
  void VehicleControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(vehiclecontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<VehicleController>,
                             RTC::Delete<VehicleController>);
  }
  
};


