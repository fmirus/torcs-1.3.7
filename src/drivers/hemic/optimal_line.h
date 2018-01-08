#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>
#include <vector>
#include <map>

/**
 * Encapsulation of the data we keep for a track segment.
 * We think of it as a straight line to get the approximation of
 * the optimal line.
 */
class LineSegment
{
public:
    v2t<float> left; // Leftmost point of the track
    v2t<float> right; // Rightmost point of the track

    float optimalSpot; // (1 - optimalSpot) * left + optimalSpot * right
    // is the position that is currently believed to
    // be optimal

    float radiusOfCurve;
    float maximumSpeedAtPoint;
    float rationalSpeedLimit;

    float friction;
    
    /**
     * TR_RGT	    1
     * TR_LFT	    2
     * TR_STR	    3
     */
    int type;
    
    int state;

    v2t<float> pos(float offset);
    v2t<float> pos();
};

/**
 * Minimal model of track on which we will do the optimizations
 */
class TrackModel
{
public:
    std::vector<LineSegment> segments;

    std::map<tTrackSeg *, int> segmentIndices;

    void initialize(tTrack* track, float maxBreakG, float maxSideG, float suctionParam);

    void getTorcsTrajectory(tTrack *track);

    float getRadius(int index);

    float calculateLengthCost();

    float calculateCost();

    void costOptimizationStep(float amount);

    void costOptimization();

    float filterSegment(int index);
    float filterSegmentSpatial(int index);
    
    void filterLine();
    void filterFinishLine();
    
    void filterSegments(int startInd, int stopInd);
    
    void filterRadii();
    
    void fixFinishRadii();
    
    void printRadii();

    float directedDev(int index);

    void fillRadii();
    
    void fillPointLimits(float maxG);
    void propagateRationalLimit(float maxG);

    void clipLineToLimits();

    LineSegment *getSegment(int index);

    float getOffsetFromCenter(tTrkLocPos *p);

    float getTangentAngle(tTrkLocPos *p);

    float getMaximumSpeed(tTrkLocPos *p);
    
    void dumpDataToFile(const char *path);
    void dumpTrackToFile(tTrack* track, const char *path);

private:
    float suctionParam;
};





























