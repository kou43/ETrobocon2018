/******************************************************************************
 *  LineMonitor.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineMonitor
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "PidControler.h"

static long diff[2];
static double integral;


/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
PidControler::PidControler(const ev3api::ColorSensor& colorSensor)
    : mColorSensor(colorSensor) {
}


double PidControler::math_limit(double sum, double a, double b) const{
  if(sum<a){
    return a;
  }else if(sum>b){
    return b;
  }else{
    return sum;
  }
}

double PidControler::calcDirection(double KP, double KI, double KD, short target_val) const{
  double p, i, d;
  int8_t sensor_val = mColorSensor.getBrightness();

  diff[0] = diff[1];
  diff[1] = sensor_val - target_val;
  integral += (diff[1] + diff[0])/2.0*DELTA_T;

  p = KP * diff[1];
  i = KI * integral;
  d = KD * (diff[1] - diff[0])/DELTA_T;

  return math_limit(p+i+d, -100.0, 100.0);
}
