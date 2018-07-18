/******************************************************************************
 *  LineTracer.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracer.h"
#include "PidControler.h"

double p, ii, d, target, fw;

/**
 * コンストラクタ
 * @param PidControler     ライン判定
 * @param balancingWalker 倒立走行
 */
LineTracer::LineTracer(const PidControler* PidControler,
                       BalancingWalker* balancingWalker,
                     ParameterChanger* parameterChanger)
    : mPidControler(PidControler),
      mBalancingWalker(balancingWalker),
      mIsInitialized(false),
      mParameterChanger(parameterChanger) {
      }

/**
 * ライントレースする
 */
void LineTracer::run() {

    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    //パラメータの取得
    p = mParameterChanger->getp();
    ii = mParameterChanger->geti();
    d = mParameterChanger->getd();
    target = mParameterChanger->gettarget();
    fw = mParameterChanger->getspeed();

    //pidで方向を計算
    int direction = (int)mPidControler->calcDirection(p, ii, d, target);

    mBalancingWalker->setCommand(fw, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}
