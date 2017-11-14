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

#include "opponent.h"
class Opponents;
class Opponent;

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

    float getAllowedSpeed(tTrackSeg *segment);
    float getDistToSegEnd(tCarElt *car);
    float getAccel(tCarElt *car);

    float getBrake(tCarElt *car);
    int getGear(tCarElt *car);

    float filterABS(float brake);

    void initCa();
    void initCw();

  private:
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
