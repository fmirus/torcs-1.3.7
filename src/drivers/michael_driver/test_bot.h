/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   test_bot.h
 * Author: michaelheinrich
 *
 * Created on 26. November 2017, 23:30
 */

#ifndef TEST_BOT_H
#define TEST_BOT_H

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>
#include <vector>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <tmath/linalg_t.h>
#include <iomanip>
#include <string>
#include <sstream>

#include "optimal_line.h"


#define MAX_BREAK_G 1.60
#define MAX_LATERAL_G 1.60
#define SUCTION_G_PER_M_SS (0.5 / 1000.0)


#define COLLISION_WARNING_DIST 10
#define COLLISION_AVOID_GAIN 5

/**
 * If the closest car is closer than COLLISION_IMMINENT_DIST and
 * has a lower race position, issue a brake maneuver
 * 
 */
#define COLLISION_IMMINENT_DIST 5
#define EMERGENCY_BRAKE_DV 1

#define UNSTUCKING_STEPS 90
#define MANEUVER_INNOVATION 0.1

#define THROTTLING_I_INIT 0.8
#define THROTTLING_P 1.0
#define THROTTLING_I 0.0003

#define THROTTLING_P_LIM 0.1

#define MAX_THROTTLING 1.0
#define MIN_THROTTLING 0.3
//#define NOMINAL_REL_POSITION 0.5
#define NOMINAL_REL_POSITION 0.5

#define NBBOTS 1

typedef struct Bot
{
    TrackModel trackModel;
    float lastManeuver;
    std::time_t lastDebugOutTime;
    int lastMoveStep;
    int currentStep;
    int remainingBackwardSteps;
    bool hasLaunched;
    
    float throttlingI;
    float slipDist;
    
    float ownTime;
    float otherTime;
    
} tBot;

#endif /* TEST_BOT_H */

