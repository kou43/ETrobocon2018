/******************************************************************************
 *  LineMonitor.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineMonitor
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_PidControler_H_
#define EV3_UNIT_PidControler_H_

#include "ColorSensor.h"

// 定義
class PidControler {
public:
    explicit PidControler(const ev3api::ColorSensor& colorSensor);

    #define DELTA_T 0.004

    double math_limit(double sum, double a, double b) const;
    double calcDirection(double KP, double KI, double KD, short target_val) const;

private:

    const ev3api::ColorSensor& mColorSensor;
};

#endif  // EV3_UNIT_LINEMONITOR_H_
