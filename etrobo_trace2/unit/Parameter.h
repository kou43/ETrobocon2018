#ifndef EV3_APP_PARAMETER_H_
#define EV3_APP_PARAMETER_H_

struct Parameter {
public:
  double p;
  double i;
  double d;
  double target;
  double fw;
};

#endif  // EV3_APP_PARAMETER_H_
