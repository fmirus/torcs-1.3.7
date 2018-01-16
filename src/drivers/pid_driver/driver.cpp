#include "driver.h"

#include <cmath>
#include <iostream>
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

const float Driver::MAX_UNSTUCK_ANGLE = 30.0 / 180.0 * PI; /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;              /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;               /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;                /* [m] */
const float Driver::G = 9.81;                              /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;               /* [m/s] */
const float Driver::SHIFT = 0.9;                           /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;                    /* [m/s] */
const float Driver::ABS_SLIP = 0.9;                        /* [-] range [0.95..0.3] */
const float Driver::ABS_MINSPEED = 3.0;                    /* [m/s] */
const float Y_DIST_TO_MIDDLE = 5.0;
const float GOAL_POS_Y = -20;
const float GOAL_POS_X = 0;
const float MIN_DIST = 4;

Driver::Driver(int index) {
    float dt = 0.02;
    float Kp = -0.3;
    float Kd = -0.2;
    float Ki = -0.001;
    _pidAcc = PidAcc(dt, Kp, Kd, Ki);
    Kp = -0.1;
    Kd = -1.0;
    Ki = -0.005;
    float G1 = 0.2;
    float G2 = 0.8;
    _pidSteer = PidSteer(dt, Kp, Kd, Ki, G1, G2, 12);
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack *t, void *carHandle, void **carParmHandle, tSituation *s) {
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void Driver::newRace(tCarElt *car, tSituation *s) {
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
    initCa();
    initCw();

    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / RCM_MAX_DT_ROBOTS);
    stuck = 0;

    /* initialize the list of opponents */
    opponents = new Opponents(s, this);
    opponent = opponents->getOpponentPtr();
}

/* Drive during race. */
void Driver::drive(tCarElt *car, tSituation *s) {
    update(car, s);
    // std::cout << "Other cars ----" << std::endl;
    // for (int i = 0; i < opponents->getNOpponents(); i++) {
        // std::cout << "SpeedY " << i << ": " << getSpeed() << std::endl;
        // std::cout << "SpeedDiffX " << i << ": " << getOpponentSpeedDiffX(opponent[i]) << std::endl;
        // std::cout << "SpeedDiffY " << i << ": " << getOpponentSpeedDiffY(opponent[i]) << std::endl;
        // std::cout << "DistanceY " << i << ": " << getOpponentDistanceY(opponent[i]) << std::endl;
        // std::cout << "DistanceX " << i << ": " << getOpponentDistanceX(opponent[i]) << std::endl;
        // std::cout << getOpponentDistanceX(opponent[i]) << ", " << getOpponentDistanceY(opponent[i]) << std::endl;
    // }

    // std::cout << "AI car ----" << std::endl;
    // std::cout << "To middle: " << car->_trkPos.toMiddle << std::endl;
    // std::cout << "To left: " << car->_trkPos.toLeft << std::endl;
    // std::cout << "To right: " << car->_trkPos.toRight << std::endl;
    // std::cout << "Angle:" << angle << std::endl;

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    car->ctrl.gear = getGear(car);
    handleSpeed();
    handleSteering();
}

void Driver::handleSteering() {
    float currentPosX = getOpponentDistanceX(opponent[0]);
    float goalX = getGoalPosX();
    // std::cout << "goalX: " << goalX << std::endl;
    car->ctrl.steer = _pidSteer.step(goalX, 0.0, currentPosX, angle);
    if (getSpeed() < 0)
        car->ctrl.steer *= -1;
    // std::cout << "Steering: " << car->ctrl.steer << std::endl;
}

float Driver::getGoalPosX() {
    float dist = GOAL_POS_X;
    // Only add a safety margin during overtake, if the goal position is closer than the safety distance
    if (std::abs(GOAL_POS_X) < MIN_DIST) {
        dist += sgn(GOAL_POS_X) * MIN_DIST *
                std::pow((MIN_DIST + Y_DIST_TO_MIDDLE) / getOpponentDistanceY(opponent[0]), 2);

        // Check if the distance becomes too large
        if (std::abs(dist) > std::abs(MIN_DIST)) {
            dist = sgn(GOAL_POS_X) * MIN_DIST;
        }
    }
    // Always overtake on the side of the front car, that directs towards the center of the street
    if (getOpponentDistanceX(opponent[0]) + car->_trkPos.toMiddle > 0) {
        dist *= sgn(dist);
        // std::cout << "overtake right" << std::endl;
    } else {
        dist *= -1 * sgn(dist);
        // std::cout << "overtake left" << std::endl;
    }
    return dist;
}

// This decides over the current speed
void Driver::handleSpeed() {
    // This is for abs
    car->ctrl.brakeCmd = filterABS(getBrake(car));
    if (car->ctrl.brakeCmd != 0.0) {
        car->ctrl.accelCmd = 0.0;
        return;
    }

    // Try to keep a certain distance
    float currentPosY = getOpponentDistanceY(opponent[0]);
    float maxAcc = getMaxAccel(car);
    car->ctrl.accelCmd = _pidAcc.step(GOAL_POS_Y, currentPosY, maxAcc);
    // std::cout << "Acceleration: " << car->ctrl.accelCmd << std::endl;

    // Check if car is upside down
    if (std::abs(angle) > M_PI * 0.5) {
        car->ctrl.accelCmd *= -1;
    }

    // Check if you need the reversed gear
    if (car->ctrl.accelCmd < 0) {
        car->ctrl.accelCmd *= -1;
        car->ctrl.gear = -1;
    }

    // Check if you need to break to control the speed
    if (car->ctrl.accelCmd < 0 && getOpponentDistanceY(opponent[0]) < GOAL_POS_Y) {
        car->ctrl.gear = -1;
        car->ctrl.brakeCmd = 1.0;
    }

    // If you are too close to the front car and centered behind it, brake
    if (getOpponentDistanceY(opponent[0]) > 0 && getOpponentDistanceY(opponent[0]) < Y_DIST_TO_MIDDLE * 1.25 &&
        std::abs(getOpponentDistanceX(opponent[0])) < 1) {
        // std::cout << "BREAK" << std::endl;
        car->ctrl.accelCmd = 0.0;
        car->ctrl.brakeCmd = 1.0;
    }
}

/* Update my private data every timestep */
void Driver::update(tCarElt *car, tSituation *s) {
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);

    mass = CARMASS + car->_fuel;
    speed = Opponent::getSpeed(car);
    opponents->update(s, this);
}

float Driver::getOpponentDistanceX(Opponent o) { return o.getDistanceToMiddle() - car->_trkPos.toMiddle; }

float Driver::getOpponentDistanceY(Opponent o) {
    float distance = sqrt(pow(o.getDistance(), 2) - pow(getOpponentDistanceX(o), 2));
    if (std::isnan(distance))
        distance = 0;
    return sgn(o.getDistance()) * distance + Y_DIST_TO_MIDDLE;
}

float Driver::getOpponentSpeedDiffX(Opponent o) { return car->_speed_y - o.getSpeedY(); }

float Driver::getOpponentSpeedDiffY(Opponent o) { return car->_speed_x - o.getSpeedX(); }

/* Antilocking filter for brakes */
float Driver::filterABS(float brake) {
    if (car->_speed_x < ABS_MINSPEED)
        return brake;
    int i;
    float slip = 0.0;
    for (i = 0; i < 4; i++) {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
    }
    slip = slip / 4.0;
    if (slip < ABS_SLIP)
        brake = brake * slip;
    return brake;
}

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd(tCarElt *car) {
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart) * car->_trkPos.seg->radius;
    }
}

/* Compute gear */
int Driver::getGear(tCarElt *car) {
    if (car->_gear <= 0)
        return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine / gr_up;
    float wr = car->_wheelRadius(2);

    if (omega * wr * SHIFT < car->_speed_x) {
        return car->_gear + 1;

    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine / gr_down;
        if (car->_gear > 1 && omega * wr * SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

/* Compute fitting acceleration */
float Driver::getMaxAccel(tCarElt *car) {
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed / car->_wheelRadius(REAR_RGT) * gr / rm;
    }
}

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment) {
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt((mu * G * segment->radius) / (1.0 - MIN(1.0, segment->radius * CA * mu / mass)));
    }
}

/* Set pitstop commands. */
int Driver::pitCommand(tCarElt *car, tSituation *s) { return ROB_PIT_IM; /* return immediately */ }

/* End of the current race */
void Driver::endRace(tCarElt *car, tSituation *s) {}

float Driver::getBrake(tCarElt *car) {
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x * car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr / (2.0 * mu * G);
    float lookaheaddist = getDistToSegEnd(car);
    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x)
        return 1.0;
    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
        allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {
            float allowedspeedsqr = allowedspeed * allowedspeed;
            float brakedist = mass * (currentspeedsqr - allowedspeedsqr) /
                              (2.0 * (mu * G * mass + allowedspeedsqr * (CA * mu + CW)));
            if (brakedist > lookaheaddist) {
                return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
}

/* Check if I'm stuck */
bool Driver::isStuck(tCarElt *car) {
    if (fabs(angle) > MAX_UNSTUCK_ANGLE && car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle * angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

/* Compute aerodynamic drag coefficient CW */
void Driver::initCw() {
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char *)NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char *)NULL, 0.0);
    CW = 0.645 * cx * frontarea;
}

/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa() {
    char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char *)NULL, 0.0);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char *)NULL, 0.0);
    float wingca = 1.23 * rearwingarea * sin(rearwingangle);
    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char *)NULL, 0.0) +
               GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char *)NULL, 0.0);
    float h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char *)NULL, 0.20);
    h *= 1.5;
    h = h * h;
    h = h * h;
    h = 2.0 * exp(-3.0 * h);
    CA = h * cl + 4.0 * wingca;
}

Driver::~Driver() { delete opponents; }
