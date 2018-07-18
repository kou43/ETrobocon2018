/******************************************************************************
 *  Scenario.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/02/07
 *  Implementation of the Class Scenario
 *  Author: Kenya Yabe
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "ParameterChanger.h"

double param[2][5]={{0.38, 0.06, 0.027, 20, 30},
                    {0.38, 0.06, 0.027, 20, 60}};
int i=0;


ParameterChanger::ParameterChanger(){
  }


 void ParameterChanger::setparam(){
   i = 1;
 }

double ParameterChanger::getp(){
  return param[i][0];
};

double ParameterChanger::geti(){
  return param[i][1];
};

double ParameterChanger::getd(){
  return param[i][2];
};

double ParameterChanger::gettarget(){
  return param[i][3];
};

double ParameterChanger::getspeed(){
  return param[i][4];
};
