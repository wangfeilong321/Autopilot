#pragma once

const float M_PI = 3.14159265358979323846f;

const float radtodeg = 57.295779513082320876798154814105f;
const float degtorad = 0.017453292519943295769236907684886f;
const float feettometers = 0.3048f;
const float meterstofeet = 3.2808f;

const int ENGINE_PIN_1 = 5;
const int ENGINE_PIN_2 = 6;
const int ENGINE_PIN_3 = 13;
const int ENGINE_PIN_4 = 22;

const int NUMBER_OF_ANGLES = 3;

const int MAX_RPM = 15000;

const int START_TIMER = 15;

const int TRIM_TIMER = 30;

const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 2000;

const float dt = 0.008333333f;

enum { eX = 1, eY, eZ };
enum { eP = 1, eQ, eR };
enum { ePhi = 1, eTht, ePsi };

enum eIntegrateType { eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3, eAdamsBashforth4, eBuss1, eBuss2, eLocalLinearization, eAdamsBashforth5 };

const float Gftsec2 = 32.1850393f;