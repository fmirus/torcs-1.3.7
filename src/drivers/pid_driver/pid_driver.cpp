#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iostream>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "driver.h"

static Driver* driver;

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);
static int  pitcmd(int index, tCarElt* car, tSituation *s);


/*
 * Module entry point
 */
extern "C" int
pid_driver(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("pid_driver");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0;
}

/* Module interface initialization. */
static int
InitFuncPt(int index, void *pt)
{
    tRobotItf *itf  = (tRobotItf *)pt;
    // Create new driver
    driver = new Driver(index);

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
				 /* for every track change or new race */
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = pitcmd;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0;
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    driver->initTrack(track, carHandle, carParmHandle, s);
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{
    driver->newRace(car, s);
}

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{
    driver->drive(car, s);
}

/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver->pitCommand(car, s);
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    driver->endRace(car, s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    delete driver;
}

