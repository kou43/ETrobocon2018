/******************************************************************************
 *  LineTracerWithStarter.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracerWithStarter
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_LINETRACERWITHSTARTER_H_
#define EV3_APP_LINETRACERWITHSTARTER_H_

#include "Starter.h"
#include "LineTracer.h"

class LineTracerWithStarter {
public:
    LineTracerWithStarter(LineTracer* lineTracer,
               const Starter* starter);

    void run();

private:
    enum State {
        UNDEFINED,
        WAITING_FOR_START,
        WALKING
    };

    LineTracer* mLineTracer;
    const Starter* mStarter;
    State mState;

    void execUndefined();
    void execWaitingForStart();
    void execWalking();
};

#endif  // EV3_APP_LINETRACERWITHSTARTER_H_
