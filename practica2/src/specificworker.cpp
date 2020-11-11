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
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
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
/*
    const float threshold = 200; // millimeters
    float rot = 0.6;  // rads per second
*/
    const float threshold = 150; // millimeters
    float rot = M_PI/2;  // rads per second
	int x=0,z=0;
	float angle=0;
	float seg=1000000;
    try
    {
    	// read laser data 
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
	//sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });  
	differentialrobot_proxy->getBasePose(x,z,angle);
		std::cout  << "la z es " << z << std::endl;
	if(z==0 && x==0){
		differentialrobot_proxy->setSpeedBase(5, rot);
		usleep(seg);
		differentialrobot_proxy->setSpeedBase(1000, 0);
	}
	if (z < -2200 && choques > 10){
    differentialrobot_proxy->setSpeedBase(0, 0);
	}
	else {
	if(z > 2100){
		alternativa = true; 
		
	}
	if( ldata.front().dist < threshold && !alternativa)
	{	choques++;
		if(choques==1){
			differentialrobot_proxy->setSpeedBase(100, rot);
			usleep(seg);
			differentialrobot_proxy->setSpeedBase(1000, 0);
			usleep(seg); 
		}
		else if(choques==2){
			differentialrobot_proxy->setSpeedBase(100, rot);
			usleep(seg);
			differentialrobot_proxy->setSpeedBase(1000, 0);
			usleep(seg); 
		}
		else{
		
			if(!giro){
				differentialrobot_proxy->setSpeedBase(100, rot);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, 0);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, rot);
				usleep(seg);
				giro=1;

			}
			else{
				differentialrobot_proxy->setSpeedBase(100, -rot);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, 0);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, -rot);
				usleep(seg);
				giro=0;
				
			}
		}

	
	}
	if (alternativa){
		if (choques%2 == 0){

		        differentialrobot_proxy->setSpeedBase(100, -rot);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(1000, 0);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, -rot);
				usleep(seg);
				
				
				alternativa = false;
				
		}
		if (choques%2 != 0){

                differentialrobot_proxy->setSpeedBase(100, rot);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(1000, 0);
				usleep(seg);
				differentialrobot_proxy->setSpeedBase(100, rot);
				usleep(seg);
					
				alternativa = false; 
		}
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(1000, 0); 
  	}
	}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
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

