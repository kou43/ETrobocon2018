/******************************************************************************
 *  LineTracer.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_LINETRACER_H_
#define EV3_APP_LINETRACER_H_

#include "PidControler.h"
#include "BalancingWalker.h"


class LineTracer {
public:
    LineTracer(const PidControler* PidControler,
               BalancingWalker* balancingWalker);

    void run();

private:
    const PidControler* mPidControler;
    BalancingWalker* mBalancingWalker;
    bool mIsInitialized;

};

#endif  // EV3_APP_LINETRACER_H_
