/***************************************************************************

    file                 : test_bot.cpp
    created              : Do 2. Nov 08:49:38 CET 2017
    copyright            : (C) 2017 Michael Heinrich, Jonas Natzer

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif


#include "hemic.h"


static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);

static tBot bots[NBBOTS];


void openPositionLog(tBot *bot);
void writePositionLog(tBot *bot, float x, float y);
void closePositionLog(tBot *bot);

/* 
 * Module entry point  
 */
extern "C" int
hemic(tModInfo *modInfo)
{
    memset(modInfo, 0, NBBOTS * sizeof (tModInfo));

    for (int i = 0; i < NBBOTS; i++)
    {
        modInfo->name = strdup("Hemic"); /* name of the module (short) */
        modInfo->desc = strdup("2017 TU München research project by "
                               "Michael Heinrich and Jonas Natzer"); /* description of the module (can be long) */
        modInfo->fctInit = InitFuncPt; /* init function */
        modInfo->gfId = ROB_IDENT; /* supported framework version */
        modInfo->index = i + 1;
    }

    return 0;
}

/* Module interface initialization. */
static int
InitFuncPt(int index, void *pt)
{
    if (index < 1 || index > NBBOTS)
    {
        std::cout << "test_bot.cpp: array index out of bounds\n";
        return 0;
    }

    tRobotItf *itf = (tRobotItf *) pt;

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    /* for every track change or new race */
    itf->rbNewRace = newrace; /* Start a new race */
    itf->rbDrive = drive; /* Drive during race */
    itf->rbPitCmd = NULL;
    itf->rbEndRace = endrace; /* End of the current race */
    itf->rbShutdown = shutdown; /* Called before the module is unloaded */
    itf->index = index; /* Index used if multiple interfaces */
    return 0;
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    if (index < 1 || index > NBBOTS)
    {
        std::cout << "test_bot.cpp: array index out of bounds\n";
        return;
    }

    tBot *bot = bots + index - 1;
    // track, base, blab, suctionParam
    bot->trackModel.initialize(
                               track,
                               MAX_BREAK_G, // max Break G
                               MAX_LATERAL_G, // max lateral G
                               SUCTION_G_PER_M_SS // additional G per m/s (through suction)
                               );
    *carParmHandle = NULL;
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{
    std::cout << "hemic.cpp: Version 0.08\n";


    if (index < 1 || index > NBBOTS)
    {
        std::cout << "test_bot.cpp: array index out of bounds\n";
        return;
    }

    tBot *bot = bots + index - 1;

    bot->lastManeuver = 0;
    bot->lastMoveStep = 0;
    bot->currentStep = 0;
    bot->remainingBackwardSteps = 0;
    bot->hasLaunched = false;
    bot->slipDist = 0;
    bot->throttlingI = THROTTLING_I_INIT;

    openPositionLog(bot);
}

/* Drive during race. */

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{
    memset(&car->ctrl, 0, sizeof (tCarCtrl));

    if (index < 1 || index > NBBOTS)
    {
        std::cout << "test_bot.cpp: array index out of bounds\n";
        return;
    }

    tBot *bot = bots + index - 1;

    float angle;
    const float SC = 1.0;

    tCarElt *closestOponent = 0x00;
    float closestOpDist = 1e9f;
    int ncars = s->raceInfo.ncars;

    tdble cX, cY, oX, oY;
    v2t<float> opponentDir;

    RtTrackLocal2Global(&(car->_trkPos), &cX, &cY, 0);

    for (int i = 0; i < ncars; i++)
    {
        tCarElt *op = s->cars[i];

        if (op == car)
        {
            continue;
        }

        if (op != 0x00)
        {
            RtTrackLocal2Global(&(op->_trkPos), &oX, &oY, 1);
            v2t<float> dif(oX - cX, oY - cY);
            float len = dif.len();



            if (len < closestOpDist)
            {
                closestOpDist = len;
                closestOponent = op;
                opponentDir = dif;
            }
        }
    }

    float maneuver = (1 - MANEUVER_INNOVATION) * bot->lastManeuver;
    float emergency = 0;
    float threatSpeed = 0;

    if (closestOponent != 0x00 && closestOpDist < COLLISION_WARNING_DIST)
    {
        float ownPos = car->_trkPos.toLeft;
        float otherPos = closestOponent->_trkPos.toLeft;

        float diff = ownPos - otherPos;
        float dir = diff;
        dir = dir >= 0 ? 1 : dir;
        dir = dir < 0 ? -1 : dir;

        bool isBehind = car->race.pos < closestOponent->race.pos;
        bool isCollisionImmenent = (closestOpDist < COLLISION_IMMINENT_DIST)
                && !isBehind;

        float intrusion = dir * (COLLISION_WARNING_DIST - closestOpDist)
                / (COLLISION_WARNING_DIST - COLLISION_IMMINENT_DIST);

        // avoid car that intrudes into our comfort radius
        maneuver += MANEUVER_INNOVATION * COLLISION_AVOID_GAIN * intrusion;

        float dirX = opponentDir.x;
        float dirY = opponentDir.y;

        v2t<float> dirV = opponentDir;
        dirV.normalize();

        v2t<float> dv;

        tPosd vCar = car->pub.DynGCg.vel;
        tPosd vOpp = car->pub.DynGCg.vel;

        dv.x = vOpp.x - vCar.x;
        dv.y = vOpp.y - vCar.y;

        threatSpeed = -(dv.x * dirV.x + dv.y * dirV.y);

        if (threatSpeed > 0.5 * EMERGENCY_BRAKE_DV)
        {
            emergency = (2 * (threatSpeed / EMERGENCY_BRAKE_DV)) - 1;

            emergency = emergency > 1 ? 1 : emergency;
            emergency = emergency < 0 ? 0 : emergency;
        }

        emergency = isCollisionImmenent ? 1.0 : 0.0;

        /*
        std::cout << "COLLISION_WARNING: intrusion=" << intrusion <<
                ", maneuver=" << maneuver <<
                ", obstructManeuver=" << isBehind
                << "\n";
         */
    }

    angle = bot->trackModel.getTangentAngle(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

    float offset = bot->trackModel.getOffsetFromCenter(&(car->_trkPos)) + maneuver;

    // half of width minus security margin
    float wh = 0.20 * (car->_trkPos.seg->startWidth + car->_trkPos.seg->endWidth);

    offset = offset > wh ? wh : offset;
    offset = offset < -wh ? -wh : offset;

    float correctiveAngle = -(SC * (car->_trkPos.toMiddle + offset)) / car->_trkPos.seg->width;

    float corrLim = 0.2;

    bool offRoad = offset > wh || offset < -wh;

    if (offRoad)
    {
        corrLim = 1;
    }


    if (correctiveAngle > corrLim)
    {
        correctiveAngle = corrLim;
    }

    if (correctiveAngle < -corrLim)
    {
        correctiveAngle = -corrLim;
    }

    angle += correctiveAngle;


    float speed = car->pub.speed;
    float speedLim = bot->trackModel.getMaximumSpeed(&(car->_trkPos));

    float relPosition = (car->race.pos - 1) / (float) (ncars - 1);

    float difPos = -(NOMINAL_REL_POSITION - relPosition);

    bot->throttlingI += THROTTLING_I * difPos;
    float throttlingP = THROTTLING_P * difPos;

    throttlingP = throttlingP > THROTTLING_P_LIM ? THROTTLING_P_LIM : throttlingP;
    throttlingP = throttlingP < -THROTTLING_P_LIM ? -THROTTLING_P_LIM : throttlingP;

    float throttling = throttlingP + bot->throttlingI;

    throttling = throttling > MAX_THROTTLING ? MAX_THROTTLING : throttling;
    throttling = throttling < MIN_THROTTLING ? MIN_THROTTLING : throttling;

    speedLim *= throttling;


    // set up the values to return
    car->ctrl.steer = angle / car->_steerLock;

    float dv = speed - speedLim;

    if (dv < -2)
    {
        car->ctrl.accelCmd = 1.0; // 100% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if (dv < -1)
    {
        // dv between -2 and -1
        float cmd = -(dv + 1);

        car->ctrl.accelCmd = cmd; // linear accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if (dv < 0)
    {
        // dv between -1 and 0
        car->ctrl.accelCmd = 0.0; // no accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if (dv < 1)
    {
        // dv between 0 and 1
        float cmd = dv;

        car->ctrl.accelCmd = 0.0; // no accelerator pedal
        car->ctrl.brakeCmd = cmd; // linear brakes
    }
    else
    {
        car->ctrl.accelCmd = 0.0; // no accelerator pedal
        car->ctrl.brakeCmd = 1; // 100% brakes
    }

    float maxRpm = car->_enginerpmRedLine * .9f;
    float minRpm = maxRpm * .6;

    int gear = car->_gear;

    if (car->_enginerpm > maxRpm)
    {
        gear++;
    }
    else if (car->_enginerpm < minRpm)
    {
        gear--;
    }

    if (gear > 6)
    {
        gear = 6;
    }

    if (gear < 1)
    {
        gear = 1;
    }

    float slip =
            car->priv.wheel[0].slipAccel +
            car->priv.wheel[1].slipAccel +
            car->priv.wheel[2].slipAccel +
            car->priv.wheel[3].slipAccel;

    slip *= 0.25;

    float skid = car->priv.skid[0] + car->priv.skid[1] +
            car->priv.skid[2] + car->priv.skid[3];

    skid *= 0.25;

    /*
    std::cout <<
            "skid=" << skid <<
            ", gear=" << gear <<
            ", slip=" << slip <<
            "\n";
     */


    if (slip < -10)
    {
        car->ctrl.accelCmd = 0;
        bot->slipDist -= slip;
    }
    else if (slip < -1)
    {
        float aSlip = -slip;
        float rel = (aSlip - 1.0) / (10.0 - 1.0);
        // smooth reduction of throttle on starting slip
        car->ctrl.accelCmd = rel * 0.0f + (1.0f - rel) * 1.0;
        bot->slipDist -= slip;
    }
    else
    {
        // AIMD algorithm estimates severity of slip event
        bot->slipDist *= 0.9;
    }

    if (bot->slipDist > 10)
    {
        // Disable engine if slip event gets out of control
        car->ctrl.accelCmd = 0;
    }

    car->ctrl.gear = gear;


    bool moves = speed > 1;

    if (moves)
    {
        bot->lastMoveStep = bot->currentStep;
        bot->hasLaunched = true;
    }
    else if (bot->currentStep - bot->lastMoveStep > 20 && bot->hasLaunched)
    {
        bot->remainingBackwardSteps = UNSTUCKING_STEPS;
    }

    if (bot->remainingBackwardSteps > 0)
    {
        car->ctrl.gear = -1;
        car->ctrl.accelCmd = .3;
        car->ctrl.brakeCmd = 0;
        car->ctrl.steer = -100 * angle / car->_steerLock;

        bot->remainingBackwardSteps--;
    }

    if (!bot->hasLaunched)
    {
        car->ctrl.gear = 1;
        car->ctrl.accelCmd = .6;
        car->ctrl.brakeCmd = 0;
    }

    car->ctrl.accelCmd *= throttling;

    float accCmd = emergency * 0.0 + (1 - emergency) * car->ctrl.accelCmd;
    float brakeCmd = (1 - emergency) * car->ctrl.brakeCmd + emergency * 1.0;


    car->ctrl.accelCmd = accCmd;
    car->ctrl.brakeCmd = brakeCmd;

    std::time_t t = std::time(NULL);
    std::time_t dt = t - bot->lastDebugOutTime;

    if (true || dt >= 1)
    {
        bot->lastDebugOutTime = t;

        //std::ofstream logfile;
        //logfile.open ("~/test_bot.log", std::ofstream::out | std::ofstream::app);

/*

        std::cout << std::showpos << std::setprecision(3) << std::fixed
                << "off=" << std::setw(3) << offset
                << std::noshowpos
                << ", acc=" << std::setw(3) << car->ctrl.accelCmd
                << ", brk=" << std::setw(3) << car->ctrl.brakeCmd
                //<< ", spd=" << std::setw(3) << speed
                << ", spdLim=" << std::setw(3) << speedLim
                << ", thr=" << std::setw(3) << throttling
                << ", thrP=" << std::setw(3) << throttlingP
                << ", thrI=" << std::setw(3) << bot->throttlingI
                << ", relPos=" << std::setw(3) << relPosition
                << ", threatSpd=" << std::setw(3) << threatSpeed
                << ", emergency=" << std::setw(3) << emergency

                //<< ", friction=" << car->_trkPos.seg->surface->kFriction
                << "\n";

*/

        bot->lastManeuver = maneuver;


        // statistic capture

        float ownCt = car->race.curTime;
        float othCt = 0;
        float othBL = 0;

        bot->otherTime = 0;

        for (int i = 0; i < s->raceInfo.ncars; i++)
        {
            tCarElt *c = s->cars[i];

            if (c == car)
            {
                continue;
            }

            othCt = c->race.curTime;
            othBL = c->race.timeBehindLeader;
        }

        if (ownCt == 0)
        {
            bot->otherTime = othCt;
            bot->ownTime = othCt + car->race.timeBehindLeader;
        }
        else
        {
            bot->ownTime = ownCt;
            bot->otherTime = ownCt + othBL;
        }

        if (closestOponent != NULL)
        {
            float diffX = 0, diffY = 0;

            if (false)
            {
                tTrackSeg *currentSeg = car->pub.trkPos.seg;
                tTrkLocPos opPos = (closestOponent->pub.trkPos);
                float opX, opY;

                RtTrackLocal2Global(&opPos, &opX, &opY, 1);
                RtTrackGlobal2Local(currentSeg, opX, opY, &opPos, 1);

                diffX = car->pub.trkPos.toLeft - opPos.toLeft;
                diffY = car->pub.trkPos.toStart - opPos.toStart;

            }
            else
            {
                tTrkLocPos *opPos = &(closestOponent->pub.trkPos);

                diffX = car->pub.trkPos.toLeft - opPos->toLeft;
                diffY = car->pub.trkPos.toStart - opPos->toStart;

                writePositionLog(bot, diffX, diffY);
            }



        }



        //logfile.close();
    }


    bot->currentStep++;
    /*
    car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    car->ctrl.brakeCmd = 0.0; // no brakes
     */
}

inline bool exists_file(const std::string& name)
{
    if (FILE * file = fopen(name.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    std::cout << "endrace called\n";

}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    std::cout << "shutdown called\n";

    std::ostringstream oss;

    oss << "stats_lim_" << NOMINAL_REL_POSITION << ".csv";

    std::string path = oss.str();

    bool existed = exists_file(path);

    FILE *f = fopen(path.c_str(), "a");

    if (f == 0x00)
    {
        std::cout << "Couldn't open stats file " << path << "\n";
        return;
    }

    if (!existed)
    {
        fputs("ownTime, otherTime\n", f);
    }



    fprintf(
            f,
            "%f, %f\n",
            bots[index].ownTime, bots[index].otherTime
            );

    fclose(f);


    closePositionLog(bots + index - 1);
}

void openPositionLog(tBot *bot)
{
    if (bot->distanceLog == NULL)
    {
        bot->distanceLog = fopen("relative_distance.csv", "a");
    }
}

void writePositionLog(tBot *bot, float x, float y)
{
    if (bot->distanceLog != NULL)
    {
        fprintf(
                bot->distanceLog,
                "%f, %f\n",
                x, y
                );
    }
}

void closePositionLog(tBot *bot)
{
    if (bot->distanceLog != NULL)
    {
        fclose(bot->distanceLog);
        bot->distanceLog = NULL;
    }

}
