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
void SpecificWorker::seguirPuntero(RoboCompGenericBase::TBaseState bState,coordenada target)
{
	Eigen:: Matrix2f rot;
	rot<<cos(bState.alpha),-sin(bState.alpha),sin(bState.alpha),cos(bState.alpha);
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
	std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }); 
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
	else{
		objetivo.full = false;
	}
	if(objetivo.activate == true && objetivo.full == false){
		if(ldata.front().dist < 200 && dist > ldata.front().dist)
		{
			obstaculo = true;

		}
		differentialrobot_proxy->setSpeedBase(1000,0);
		if(dist < 1000){
			differentialrobot_proxy->setSpeedBase(dist,0);
		}
		if(dist < 30){
			objetivo.set_task_finished();
			differentialrobot_proxy->setSpeedBase(0,0);
		}	
	}
}
void SpecificWorker::obstaculoEncontrado(RoboCompGenericBase::TBaseState bState)
{
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
	auto r =std::min_element( ldata.begin(), ldata.begin()+5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
	cout<< "distancia :" <<r->dist<<endl;
	cout<< "distancia a la recta :" <<algBug.DistanciaPuntoARecta(bState.x,bState.z)<<endl;
	if(r->dist > 400){
		differentialrobot_proxy->setSpeedBase(0,0.7);
	}
	else if(r->dist < 200){
		differentialrobot_proxy->setSpeedBase(0,-0.7);
	}
	else{
		differentialrobot_proxy->setSpeedBase(300,0);
		if(algBug.DistanciaPuntoARecta(bState.x,bState.z)>40){
			fueraRecta=true;
		}
		if(algBug.DistanciaPuntoARecta(bState.x,bState.z)<25 && fueraRecta==true){
			obstaculo=false;
			fueraRecta=false;
		}
	}
}
void SpecificWorker::compute()
{
	RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);

	if( auto target_o = objetivo.get(); target_o.has_value())
	{
		if (!obstaculo){
			auto target = target_o.value();
			seguirPuntero(bState,target);

		}
		if(obstaculo){
			obstaculoEncontrado(bState);
			
		}  		
	}
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
	algBug.xt=myPick.x;
	algBug.zt=myPick.z;
	RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	algBug.xr=bState.x;
	algBug.zr=bState.z;
	objetivo.put(coor);
	click=true;
	obstaculo=false;
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

