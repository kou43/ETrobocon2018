/******************************************************************************
 *  Starter.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Starter
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_STARTER_H_
#define EV3_UNIT_STARTER_H_

#include "TouchSensor.h"

class Starter {
public:
    explicit Starter(const ev3api::TouchSensor& touchSensor);

    bool isPushed() const;

private:
    const ev3api::TouchSensor& mTouchSensor;
};

#endif  // EV3_UNIT_STARTER_H_
