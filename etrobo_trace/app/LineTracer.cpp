/******************************************************************************
 *  LineTracer.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracer.h"
#include "PidControler.h"

/**
 * コンストラクタ
 * @param PidControler     ライン判定
 * @param balancingWalker 倒立走行
 */
LineTracer::LineTracer(const PidControler* PidControler,
                       BalancingWalker* balancingWalker)
    : mPidControler(PidControler),
      mBalancingWalker(balancingWalker),
      mIsInitialized(false) {
      }

/**
 * ライントレースする
 */
void LineTracer::run() {

    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    //pidで方向を計算
    int direction = (int)mPidControler->calcDirection(0.38, 0.06, 0.027, 20);

    mBalancingWalker->setCommand(BalancingWalker::LOW, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}
