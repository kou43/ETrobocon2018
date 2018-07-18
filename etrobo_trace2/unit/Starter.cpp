/******************************************************************************
 *  Starter.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Starter
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Starter.h"

/**
 * コンストラクタ
 * @param touchSensor タッチセンサ
 */
Starter::Starter(const ev3api::TouchSensor& touchSensor)
    : mTouchSensor(touchSensor) {
}

/**
 * 押下中か否か
 * @retval true  押下している
 * @retval false 押下していない
 */
bool Starter::isPushed() const {
    return mTouchSensor.isPressed();
}
