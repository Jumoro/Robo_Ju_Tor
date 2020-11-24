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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <Eigen/Dense>
#include <math.h>
template <typename T>
struct Target
{
  T content;
  std::mutex mymutex;
  bool activate = false;
  bool full = false;

  void put(const T &data)
  {
    std::lock_guard<std::mutex> guard(mymutex);
    content = data;   // generic type must be copy-constructable
    activate = true;
    full = true;
  }
  std::optional<T> get()
  {
    std::lock_guard<std::mutex> guard(mymutex);
    if(activate){
       return content;
    }else
       return {};
  }  
  void set_task_finished()
 {
    std::lock_guard<std::mutex> guard(mymutex);
    activate = false;
 }
};

struct coordenada
{
   float x;
   float z;
};
struct recta
{
   float xt;
   float zt;
   float xr;
   float zr;
   float A(){
      return zr-zt;
   }
   float B(){
      return xt-xr;
   }
   float C(){
     return -B()*zr-A()*xr;
   }
   float M(){
      return (zt-zr)/(xt-xr);
   }
   float DistanciaPuntoARecta(float x, float z)
   {
      return fabs(A()*x + B()*z + C())/(sqrt(pow(A(),2)+pow(B(),2)));
   }
};
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


	void RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
   void obstaculoEncontrado(RoboCompGenericBase::TBaseState bState);
   void seguirPuntero(RoboCompGenericBase::TBaseState bState,coordenada target);
private:
	std::shared_ptr < InnerModel > innerModel;
   recta algBug;
	bool startup_check_flag;
   bool obstaculo = false;
   bool click=false;
   bool fueraRecta=false;
   bool avance=true;

   Target <coordenada> objetivo;
};

#endif
