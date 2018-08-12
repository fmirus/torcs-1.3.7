#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <car.h>
#include <raceman.h>
#include <robot.h>
#include <robottools.h>
#include <tgf.h>
#include <track.h>

#include "pidAcc.h"
#include "pidSteer.h"
#include "opponent.h"
class Opponents;
class Opponent;

// Needed to extract sgn
template <typename T> int sgn(T val) {
    if(val == 0) return 1;
    return (T(0) < val) - (val < T(0));
}

class Driver {
  public:
    Driver(int index);
    ~Driver();
    float filterBColl(float brake);
    Opponents *opponents;
    Opponent *opponent;

    /* callback functions called from TORCS */
    void initTrack(tTrack *t, void *carHandle, void **carParmHandle, tSituation *s);
    void newRace(tCarElt *car, tSituation *s);
    void drive(tCarElt *car, tSituation *s);
    int pitCommand(tCarElt *car, tSituation *s);
    void endRace(tCarElt *car, tSituation *s);

    tCarElt *getCarPtr() { return car; }
    tTrack *getTrackPtr() { return track; }
    float getSpeed() { return speed; }

    // Each track segment allows a different maximum speed, e.g. curves
    float getAllowedSpeed(tTrackSeg *segment);
    // Get Distance to the end of the current segment. Needed to evaluate driver
    float getDistToSegEnd(tCarElt *car);
    // Get the maximal allowed acceleration for the current situation
    float getMaxAccel(tCarElt *car);
    // Wrapper function for the acceleration PID controller
    float getAccel(tCarElt *car);
    // Set best acceleration for the current situation
    void handleSpeed();
    // Set steering for the current situation
    void handleSteering();
    // Calculates an updated x distance to the reference car including a safety margin
    float getGoalPosX();

    float getBrake(tCarElt *car);
    int getGear(tCarElt *car);

    float filterABS(float brake);

    void initCa();
    void initCw();

    // Helper functions for distance and speed measurements
    float getOpponentDistanceX(Opponent o);
    float getOpponentDistanceY(Opponent o);
    float getOpponentSpeedDiffX(Opponent o);
    float getOpponentSpeedDiffY(Opponent o);

  private:
    // The PID controller for acceleration
    PidAcc _pidAcc;
    // The PID controller for steering
    PidSteer _pidSteer;

    /* utility functions */
    bool isStuck(tCarElt *car);
    void update(tCarElt *car, tSituation *s);

    /* per robot global data */
    int stuck;
    float trackangle;
    float angle;

    /* data that should stay constant after first initialization */
    int MAX_UNSTUCK_COUNT;
    int INDEX;

    /* class constants */
    static const float MAX_UNSTUCK_ANGLE;
    static const float UNSTUCK_TIME_LIMIT;
    static const float MAX_UNSTUCK_SPEED;
    static const float MIN_UNSTUCK_DIST;
    static const float G;
    static const float FULL_ACCEL_MARGIN;
    static const float SHIFT;
    static const float SHIFT_MARGIN;
    static const float ABS_SLIP;
    static const float ABS_MINSPEED;

    /* track variables */
    tTrack *track;

    tCarElt *car; // Pointer to tCarElt struct.

    float mass;        /* mass of car + fuel */
    float CARMASS;     /* mass of the car only */
    float CA;          /* aerodynamic downforce coefficient */
    float CW;      /* aerodynamic drag coefficient */

    float speed; /* speed in track direction */
};

#endif // _DRIVER_H_
