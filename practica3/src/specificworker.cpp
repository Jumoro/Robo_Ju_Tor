/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	//float rot;
	Eigen:: Matrix2f rot;
	rot<<cos(bState.alpha),-sin(bState.alpha),sin(bState.alpha),cos(bState.alpha);

	if( auto target_o = objetivo.get(); target_o.has_value())
	{
		auto target = target_o.value();
		Eigen:: Vector2f target2 (target.x,target.z);
		Eigen:: Vector2f rw (bState.x,bState.z);

		auto tr = rot*(target2-rw);
		auto beta = atan2(tr[0],tr[1]);
		auto dist =tr.norm();
		cout<<"alfa "<<bState.alpha<<" beta "<<beta<<" distancia "<<dist<<endl;
		if(beta > 0.1 || beta< -0.1){
			differentialrobot_proxy->setSpeedBase(0,beta);
			objetivo.full = true;
		}
		// if (beta < 0.5 && beta > -0.1){
		// 	objetivo.full = false;
		// }
		else{
			objetivo.full = false;
		}
		if(objetivo.activate == true && objetivo.full == false){
			RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
			auto rght = std::min(ldata.begin() +50, ldata.begin() +55);
			cout<<"Javi es subnormal a "<<rght->dist<< " mm"<<endl;
		    differentialrobot_proxy->setSpeedBase(1000,0);
		    if(dist < 1000){
				differentialrobot_proxy->setSpeedBase(dist,0);
			}
			// differentialrobot_proxy->setSpeedBase(1000,0);
			// if(dist < 1000){
			// }
			if(dist < 30){
				objetivo.set_task_finished();
				differentialrobot_proxy->setSpeedBase(0,0);
			}		
		}
		// usleep(1000000);
		//int iteracciones=dist/1000;
		// for (int i = 0; i < iteracciones; i++){
		// 	differentialrobot_proxy->setSpeedBase(1000,0);
		// 	usleep(1000000);
		// }
		// differentialrobot_proxy->setSpeedBase(dist-iteracciones*1000,0);
		// usleep(1000000);
		// differentialrobot_proxy->setSpeedBase(0,0);
		//me muevo
		/*rot = atan2((target.x-bState.x),(target.z-bState.z));
		printf("%f\n",rot);
		if((CRX=(target.x-bState.x))==0 && (CRZ=(target.z-bState.z))==0){
			objetivo.set_task_finished();
		}
		else{
			if(click){
				//cambio de direccion
				differentialrobot_proxy->setSpeedBase(0,rot);
				
				int algo=0;
			}
			else{
				//sigo
				int algo=0;
			}
		}*/
	}


	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


//SUBSCRIPTION to setPick method from RCISMousePicker interface
void SpecificWorker::RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick)
{
//subscribesToCODE
	printf("X:%f ,Y:%f , Z:%f\n",myPick.x,myPick.y,myPick.z);
	coordenada coor;
	coor.x=myPick.x; coor.z=myPick.z;
	objetivo.put(coor);
	click=true;
}



/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompRCISMousePicker you can use this types:
// RoboCompRCISMousePicker::Pick


