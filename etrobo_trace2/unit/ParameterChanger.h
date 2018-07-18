/******************************************************************************
 *  ScenarioTracer.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/02/07
 *  Implementation of the Class ScenarioTracer
 *  Author: Kenya Yabe
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_PARAMETERCHANGER_H_
#define EV3_APP_PARAMETERCHANGER_H_

#include "Parameter.h"

class ParameterChanger{
public:
    ParameterChanger();

    void setparam();

    double getp();
    double geti();
    double getd();
    double gettarget();
    double getspeed();

private:

};

#endif  // EV3EV3_APP_PARAMETERCHANGER_H_
