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
#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>

#include "optimal_line.h"
#include "Trajectory.h"

/**
 * Get the displaced position of the spot on the line
 */
v2t<float>
LineSegment::pos(float offset)
{
    v2t<float> p = (1 - optimalSpot - offset) * left + (optimalSpot + offset) * right;
    return p;
}

/**
 * Get the position of the spot on the line
 */
v2t<float>
LineSegment::pos()
{
    v2t<float> p = (1 - optimalSpot) * left + (optimalSpot) * right;
    return p;
}

/**
 * Access segments with wrapping indices. Indices smaller zero and greater
 * than the segment count are wrapped around to access neighbors in circular
 * track.
 */
LineSegment *
TrackModel::getSegment(int index)
{
    int ind = index;
    int size = segments.size();
    if (size == 0)
    {
        return NULL;
    }

    while (ind < 0)
    {
        ind += size;
    }


    while (ind >= size)
    {
        ind -= size;
    }

    return &segments[ind];
}

/**
 * 
 */
void
TrackModel::initialize(tTrack* track, float maxBreakG, float maxSideG, float suctionParam)
{
    // creal old data first
    segments.clear();
    segmentIndices.clear();

    this->suctionParam = suctionParam;

    // check if we have valid new data
    if (track == NULL)
    {
        return;
    }

    std::cout << "optimal_line.cpp: Version 0.07\n";

    tTrackSeg *first = track->seg;
    tTrackSeg *current = first;

    for (int i = 0; i < track->nseg; i++) // limit maximum segments
    {
        if (current == NULL)
        {
            break; // error
        }

        // initialize LineSegment from track data

        LineSegment seg;

        seg.left.x = current->vertex[0].x;
        seg.left.y = current->vertex[0].y;

        seg.right.x = current->vertex[1].x;
        seg.right.y = current->vertex[1].y;

        seg.optimalSpot = 0.5;

        seg.type = current->type;
        seg.friction = current->surface->kFriction;

        segments.push_back(seg);

        segmentIndices[current] = i;

        /*
        std::cout << "optimal_line.h: "
                << "i=" << i 
                << ", lx=" << seg.left.x 
                << ", ly=" << seg.left.y 
                << ", rx=" << seg.right.x 
                << ", ry=" << seg.right.y
                << "\n";
         */


        // iterate linked list of segments
        tTrackSeg *next = current->next;

        current = next;
    }

    /*
            costOptimization();
            filterLine();
            filterLine();
            filterLine();
            clipLineToLimits();
            //optimizeLine();
     */

    getTorcsTrajectory(track);

    clipLineToLimits();
    filterLine();

    // fix problems at finish line
    filterFinishLine();


    fillRadii();


    filterRadii();
    fixFinishRadii();
    fillPointLimits(maxSideG);
    propagateRationalLimit(maxBreakG);

    dumpDataToFile("OptimalLineData.csv");
    dumpTrackToFile(track, "TrackGeom.csv");
    
    //printRadii();
    return;
}

const float weightsUnnorm[7] = {1, 1.5, 2, 2.1, 2, 1.5, 1};
const float weightNorm = 1.0 / (weightsUnnorm[0]
        + weightsUnnorm[1]
        + weightsUnnorm[2]
        + weightsUnnorm[3]
        + weightsUnnorm[4]
        + weightsUnnorm[5]
        + weightsUnnorm[6]);

const float weights[7] = {
    weightsUnnorm[0] * weightNorm,
    weightsUnnorm[1] * weightNorm,
    weightsUnnorm[2] * weightNorm,
    weightsUnnorm[3] * weightNorm,
    weightsUnnorm[4] * weightNorm,
    weightsUnnorm[5] * weightNorm,
    weightsUnnorm[6] * weightNorm
};

float
TrackModel::filterSegment(int index)
{
    float vn3 = getSegment(index - 3)->optimalSpot;
    float vn2 = getSegment(index - 2)->optimalSpot;
    float vn1 = getSegment(index - 1)->optimalSpot;
    float v = getSegment(index)->optimalSpot;
    float vp1 = getSegment(index + 1)->optimalSpot;
    float vp2 = getSegment(index + 2)->optimalSpot;
    float vp3 = getSegment(index + 3)->optimalSpot;

    float s = weights[0] * vn3
            + weights[1] * vn2
            + weights[2] * vn1
            + weights[3] * v
            + weights[4] * vp1
            + weights[5] * vp2
            + weights[6] * vp3;

    return s;
}

float
TrackModel::filterSegmentSpatial(int index)
{
    LineSegment *seg = getSegment(index);

    v2t<float> vn3 = getSegment(index - 3)->pos();
    v2t<float> vn2 = getSegment(index - 2)->pos();
    v2t<float> vn1 = getSegment(index - 1)->pos();
    v2t<float> v = seg->pos();
    v2t<float> vp1 = getSegment(index + 1)->pos();
    v2t<float> vp2 = getSegment(index + 2)->pos();
    v2t<float> vp3 = getSegment(index + 3)->pos();

    v2t<float> p = weights[0] * vn3
            + weights[1] * vn2
            + weights[2] * vn1
            + weights[3] * v
            + weights[4] * vp1
            + weights[5] * vp2
            + weights[6] * vp3;

    float toL = (seg->left - p).len();
    float toR = (seg->right - p).len();

    float w = (seg->left - seg->right).len();

    float r = -(toL * toL - toR * toR - w * w) / (2 * w);
    float l = w - r;

    float s = l / w;

    return s;
}

void TrackModel::filterSegments(int startInd, int stopInd)
{
    int n = stopInd - startInd;

    float tmpOpt[n];

    for (int i = 0; i < n; i++)
    {
        tmpOpt[i] = filterSegmentSpatial(i + startInd);
        //tmpOpt[i] = filterSegment(i + startInd);
    }

    for (int i = 0; i < n; i++)
    {
        getSegment(i + startInd)->optimalSpot = tmpOpt[i];
    }
}

/**
 * There is a problem with the optimization convergence on the start and finish
 * line. We compensate that by overriding all the values on the start and finish
 * straight with 0.5 and then filter the coefficients.
 */
void TrackModel::filterFinishLine()
{
    int start = -1;
    int end = 1;

    // scan backwards
    for (size_t i = 0; i < segments.size(); i++)
    {
        int index = -i;
        LineSegment *seg = getSegment(index);

        if (seg->type != TR_STR)
        {
            start = i + 1;
            break;
        }
    }


    // scan forwards
    for (size_t i = 0; i < segments.size(); i++)
    {
        int index = i;
        LineSegment *seg = getSegment(index);

        if (seg->type != TR_STR)
        {
            start = i - 1;
            break;
        }
    }

    for (int i = start + 1; i <= end - 1; i++)
    {
        getSegment(i)->optimalSpot = .5;
    }

    filterSegments(start, end);
    filterSegments(start, end);
    filterSegments(start, end);
    filterSegments(start, end);
}

void
TrackModel::filterLine()
{
    float tmpOpt[segments.size()];


    for (size_t i = 0; i < segments.size(); i++)
    {
        float vn3 = getSegment(i - 3)->optimalSpot;
        float vn2 = getSegment(i - 2)->optimalSpot;
        float vn1 = getSegment(i - 1)->optimalSpot;
        float v = getSegment(i)->optimalSpot;
        float vp1 = getSegment(i + 1)->optimalSpot;
        float vp2 = getSegment(i + 2)->optimalSpot;
        float vp3 = getSegment(i + 3)->optimalSpot;

        float s = weights[0] * vn3
                + weights[1] * vn2
                + weights[2] * vn1
                + weights[3] * v
                + weights[4] * vp1
                + weights[5] * vp2
                + weights[6] * vp3;

        tmpOpt[i] = s;

    }

    for (size_t i = 0; i < segments.size(); i++)
    {
        getSegment(i)->optimalSpot = tmpOpt[i];
    }
}

void
TrackModel::filterRadii()
{
    float tmpOpt[segments.size()];

    float weights[7] = {1, 1.5, 2, 2.1, 2, 1.5, 1};

    float sum = weights[0]
            + weights[1]
            + weights[2]
            + weights[3]
            + weights[4]
            + weights[5]
            + weights[6];

    sum = 1.0 / sum;

    weights[0] = weights[0] * sum;
    weights[1] = weights[1] * sum;
    weights[2] = weights[2] * sum;
    weights[3] = weights[3] * sum;
    weights[4] = weights[4] * sum;
    weights[5] = weights[5] * sum;
    weights[6] = weights[6] * sum;

    for (size_t i = 0; i < segments.size(); i++)
    {
        float vn3 = getSegment(i - 3)->radiusOfCurve;
        float vn2 = getSegment(i - 2)->radiusOfCurve;
        float vn1 = getSegment(i - 1)->radiusOfCurve;
        float v = getSegment(i)->radiusOfCurve;
        float vp1 = getSegment(i + 1)->radiusOfCurve;
        float vp2 = getSegment(i + 2)->radiusOfCurve;
        float vp3 = getSegment(i + 3)->radiusOfCurve;

        float s = weights[0] * vn3
                + weights[1] * vn2
                + weights[2] * vn1
                + weights[3] * v
                + weights[4] * vp1
                + weights[5] * vp2
                + weights[6] * vp3;

        tmpOpt[i] = s;

    }

    for (size_t i = 0; i < segments.size(); i++)
    {
        getSegment(i)->radiusOfCurve = tmpOpt[i];
    }
}

void
TrackModel::fixFinishRadii()
{
    for (int i = -10; i < 10; i++)
    {
        float radius = getSegment(i)->radiusOfCurve;
        if (radius < 1000)
        {
            getSegment(i)->radiusOfCurve = 1000;
        }
    }
}

void
TrackModel::getTorcsTrajectory(tTrack *track)
{

    const int N = track->nseg;

    const int iterations = 10 * N;

    float avgSegLen = track->length / N;

    std::cout << "nSegs=" << N << ", its=" << iterations << ", len=" <<
            track->length << ", avgSegLen=" << avgSegLen << "\n";

    Trajectory trajectory;
    SegmentList segment_list;

    tTrackSeg* seg = track->seg;
    seg = track->seg;

    float lengthLimit = 10;
    float length = lengthLimit;

    int indices[N];
    int id = 0;


    for (int i = 0; i < N; i++, seg = seg->next)
    {
        length += seg->length;
        indices[i] = id;

        if (length >= lengthLimit)
        {
            Point left(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y);
            Point right(seg->vertex[TR_SR].x, seg->vertex[TR_SR].y);
            segment_list.Add(Segment(left, right));
            id++;
            length = 0;
        }

    }

    trajectory.Optimise(segment_list, iterations, 0.02f, "/tmp/result");

    /**
     *  position in the index array, where the segment started
     */
    int segmentStartIndS = 0;

    /**
     *  segment index of the current position
     */
    int lastSegmentIndexT = 0;

    for (int aS = 0; aS < N; aS++)
    {
        int curIndT = indices[aS];
        int nextSegmentStartS = aS;

        if (curIndT > lastSegmentIndexT)
        {
            lastSegmentIndexT = curIndT;
            segmentStartIndS = aS;
        }
        for (; nextSegmentStartS <= N; nextSegmentStartS++)
        {
            int indT = indices[nextSegmentStartS % N];
            if (indT > curIndT)
            {
                break;
            }
        }

        float distance = nextSegmentStartS - segmentStartIndS;

        // modulo for negative number
        while (distance < 0)
        {
            distance += N;
        }

        float relative = ((float) (aS - segmentStartIndS)) / distance;

        relative = relative > 1 ? 1 : relative;
        relative = relative < 0 ? 0 : relative;

        int nextIndT = (curIndT + 1) % N;

        float cW = 1 - trajectory.w[curIndT];
        float nW = 1 - trajectory.w[nextIndT];

        float weighted = (1.0f - relative) * cW + relative * nW;

        /*
        std::cout << "seg#" << std::setw(4) << aS << std::fixed <<
                ": segmentStartInd=" << segmentStartIndS <<
                ", nextSegmentIndex=" << nextSegmentStartS <<
                ", cW=" << cW <<
                ", nW=" << nW <<
                ", relative=" << relative <<
                ", weighted=" << weighted << "\n";
         
         */

        getSegment(aS)->optimalSpot = weighted;
    }

    /*
    for (int i = 0; i < N; i++, seg = seg->next)
    {
        getSegment(i)->optimalSpot = 1 - trajectory.w[i];
    }
     */

    //clipLineToLimits();
    //filterLine();
    //filterLine();
}

float
TrackModel::getRadius(int i)
{
    // get segment centers at current state.
    // out of bounds indexing is handled by getSegment.

    v2t<float> a = getSegment(i - 1)->pos();
    v2t<float> p = getSegment(i)->pos();
    v2t<float> b = getSegment(i + 1)->pos();
    // find perpendicular bisectors to calculate radius
    v2t<float> apC = 0.5f * a + 0.5f * p;
    v2t<float> pbC = 0.5f * b + 0.5f * p;
    v2t<float> apD = p - a;
    v2t<float> pbD = b - p;

    /*
    -90 degree rotation matrix
    0	-1
    1	 0
     */

    v2t<float> acD;
    acD.x = -apD.y;
    acD.y = apD.x;

    v2t<float> bcD;
    bcD.x = -pbD.y;
    bcD.y = pbD.x;

    straight2t<float> la(apC, acD);
    straight2t<float> lb(pbC, bcD);

    v2t<float> center = la.intersect(lb);

    float dx = p.x - center.x;
    float dy = p.y - center.y;

    float r = sqrt(dx * dx + dy * dy);

    // check for extremely huge values or bad values

    if (r > 1000000 || isnan(r) || isinf(r))
    {
        r = 1000000;
    }

    return r;
}

/**
 * Update the radii for all segments
 */
void
TrackModel::fillRadii()
{
    // 0 - look for first low radius
    // 1 - look for second low radius
    // 2 - look for high radius
    int state = 0;
    float lastRadius = getRadius(-1);
    float suppressRadius = lastRadius;

    for (size_t i = 0; i < segments.size(); i++)
    {
        float r = getRadius(i);
        bool suppress = false;

        switch (state)
        {
        case 0:
            if (r < 0.5 * lastRadius)
            {
                state = 1;
            }
            else
            {
                suppressRadius = r;
            }
            break;

        case 1:
            if (r < 2 * lastRadius)
            {
                state = 2;
            }
            else if (r > 2 * lastRadius)
            {
                getSegment(i - 1)->radiusOfCurve = suppressRadius;
                state = 0;
                
                /*
                std::cout << "segment " << (i - 1) << " suppressed from "
                        << r << " to " << suppressRadius << "\n";
                
                */
            }
            else
            {
                state = 0;
                suppressRadius = r;
            }
            break;

        case 2:
            if (r > 2 * lastRadius)
            {
                suppress = true;
            }
            state = 0;
            break;
        }

        LineSegment *seg = getSegment(i);

        seg->radiusOfCurve = r;
        seg->state = state;

        if (suppress)
        {

            std::cout << "segment " << (i - 1) << " suppressed from "
                    << getSegment(i - 1)->radiusOfCurve << " to "
                    << suppressRadius << "\n";

            std::cout << "segment " << (i - 2) << " suppressed from "
                    << getSegment(i - 2)->radiusOfCurve << " to "
                    << suppressRadius << "\n";


            getSegment(i - 1)->radiusOfCurve = suppressRadius;
            getSegment(i - 2)->radiusOfCurve = suppressRadius;


            suppressRadius = r;
        }

        lastRadius = r;
    }
}

void
TrackModel::printRadii()
{
    for (size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *seg = getSegment(i);
        float r = seg->radiusOfCurve;
        float max = seg->maximumSpeedAtPoint;
        float rat = seg->rationalSpeedLimit;
        int state = seg->state;

        std::cout << "segment " << std::setw(4) << i << std::fixed <<
                ": optimalSpot=" << getSegment(i)->optimalSpot <<
                ", r=" << r << ", maxSpd=" << max << ", ratSpd=" << rat
                << ", state=" << state
                << "\n";
    }
}

float
TrackModel::calculateLengthCost()
{
    float lengthCost = 0;

    for (size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *start = getSegment(i);
        LineSegment *end = getSegment(i + 1);

        float len = (end->pos() - start->pos()).len();

        lengthCost += len;
    }

    return lengthCost;
}

float
TrackModel::calculateCost()
{
    /*
    float cost = 0;

    for(size_t i = 0; i < segments.size(); i++)
    {
            float r = getRadius(i);

            cost += 1.0 / (r * r);
    }
     */

    return /*cost + */calculateLengthCost();
}

float
TrackModel::directedDev(int index)
{
    float oldCost = calculateCost();

    LineSegment *seg = getSegment(index);

    float old = seg->optimalSpot;

    seg->optimalSpot += 0.001f;

    seg->optimalSpot = seg->optimalSpot > 0.95f ? 0.95f : seg->optimalSpot;
    seg->optimalSpot = seg->optimalSpot < 0.05f ? 0.05f : seg->optimalSpot;

    float newCost = calculateCost();

    seg->optimalSpot = old;

    float dev = (newCost - oldCost) / 0.001f;

    return dev;
}

void
TrackModel::costOptimizationStep(float amount)
{

    float tmpOpt[segments.size()];

    for (size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *seg = getSegment(i);
        float grad_i = directedDev(i);
        tmpOpt[i] = seg->optimalSpot - amount * grad_i;
    }

    for (size_t i = 0; i < segments.size(); i++)
    {
        segments[i].optimalSpot = tmpOpt[i];
    }

    clipLineToLimits();
}

void
TrackModel::costOptimization()
{
    float cost = calculateCost();
    for (int pass = 0; pass < 10; pass++)
    {

        costOptimizationStep(0.1);

        float costBefore = cost;
        cost = calculateCost();
        float diff = -(cost - costBefore);

        filterLine();

        std::cout << "optimization pass " << pass <<
                " cost=" << cost <<
                ", improvement=" << diff << "\n";
    }
}

void
TrackModel::clipLineToLimits()
{
    for (size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *seg = getSegment(i);

        float sp = seg->optimalSpot;
        sp = sp > .9 ? .9 : sp;
        sp = sp < .1 ? .1 : sp;

        seg->optimalSpot = sp;
    }
}

void
TrackModel::fillPointLimits(float maxG)
{
    float maxA = 9.81 * maxG;
    for (size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *seg = getSegment(i);
        float radius = seg->radiusOfCurve;
        float friction = seg->friction;

        float maxV = sqrt(friction * maxA * radius);


        float suction = suctionParam * maxV * maxV;

        maxV = sqrt(friction * (maxA + suction) * radius);

        if (maxV < 5)
        {
            maxV = 5;
        }

        seg->maximumSpeedAtPoint = maxV;
        seg->rationalSpeedLimit = maxV;
    }
}

void
TrackModel::propagateRationalLimit(float maxG)
{
    float maxA = 9.81 * maxG;
    for (size_t i = 0; i < 2 * segments.size(); i++)
    {
        LineSegment *seg = getSegment(-i);
        float plim = seg->rationalSpeedLimit;

        LineSegment *nseg = getSegment(-i + 1);
        float neighbourLim = nseg->rationalSpeedLimit;


        // calculate limit assuming maximimum deceleration
        // over straight segment

        float len = (seg->pos() - nseg->pos()).len();

        float v0 = neighbourLim;

        float friction = seg->friction;
        float r = seg->radiusOfCurve;
        float suction = suctionParam * v0 * v0;
        float aLim = friction * (maxA + suction);

        // maximum acceleration is the sum of lateral
        // acceleration and brake acceleration
        float aBrake = sqrt(aLim * aLim - ((v0 * v0) / r));

        float x = -len;
        float a0 = -aBrake;
        //float a0 = -maxA * seg->friction;

        float v_x = sqrt(v0 * v0 + 2 * a0 * x);

        if (v_x < plim)
        {
            plim = v_x;
        }

        seg->rationalSpeedLimit = plim;
    }
}

float
TrackModel::getOffsetFromCenter(tTrkLocPos *p)
{
    float relPos = 0;

    switch (p->seg->type)
    {
    case TR_STR:
        relPos = p->toStart / p->seg->length;
        break;
    case TR_LFT:
    case TR_RGT:
        relPos = p->toStart / p->seg->arc;
        break;
    }

    int ind = segmentIndices[p->seg];
    LineSegment *startSeg = getSegment(ind);
    LineSegment *endSeg = getSegment(ind + 1);

    if (startSeg == NULL || endSeg == NULL)
    {
        return 0;
    }



    float startWidth = p->seg->startWidth;
    float endWidth = p->seg->endWidth;

    float startOpt = startSeg->optimalSpot;
    float endOpt = endSeg->optimalSpot;

    float sC = 1 - relPos;
    float eC = relPos;

    float width = sC * startWidth + eC * endWidth;
    float opt = sC * startOpt + eC * endOpt;

    float off = width * (opt - 0.5);

    return off;

    //return p->seg->width;
    //return (startSeg->left - startSeg->right).len();
    //return p->seg->startWidth;
}

float
TrackModel::getTangentAngle(tTrkLocPos *p)
{
    float baseAngle = 0;

    switch (p->seg->type)
    {
    case TR_STR:
        baseAngle = p->seg->angle[TR_ZS];
        break;
    case TR_RGT:
        baseAngle = p->seg->angle[TR_ZS] - p->toStart;
        break;
    case TR_LFT:
        baseAngle = p->seg->angle[TR_ZS] + p->toStart;
        break;
    }

    int ind = segmentIndices[p->seg];
    LineSegment *startSeg = getSegment(ind);
    LineSegment *endSeg = getSegment(ind + 1);

    float len = (endSeg->pos() - startSeg->pos()).len();

    float lrStart = startSeg->optimalSpot * (startSeg->right - startSeg->left).len();
    float lrEnd = endSeg->optimalSpot * (endSeg->right - endSeg->left).len();

    float tanAngle = -(lrEnd - lrStart) / len;
    float displAngle = atan(tanAngle);

    return baseAngle + displAngle;
}

float
TrackModel::getMaximumSpeed(tTrkLocPos *p)
{
    float relPos = 0;

    switch (p->seg->type)
    {
    case TR_STR:
        relPos = p->toStart / p->seg->length;
        break;
    case TR_LFT:
    case TR_RGT:
        relPos = p->toStart / p->seg->arc;
        break;
    }

    int ind = segmentIndices[p->seg];
    LineSegment *startSeg = getSegment(ind);
    LineSegment *endSeg = getSegment(ind + 1);

    if (startSeg == NULL || endSeg == NULL)
    {
        return 50;
    }

    float startLimit = startSeg->rationalSpeedLimit;
    float endLimit = endSeg->rationalSpeedLimit;

    float sC = 1 - relPos;
    float eC = relPos;

    float maxV = sC * startLimit + eC * endLimit;

    if (maxV < 5)
    {
        maxV = 5;
    }

    return maxV;
}

void
TrackModel::dumpTrackToFile(tTrack* track, const char* path)
{
    FILE *f = fopen(path, "w");

    if (f == 0x00)
    {
        return;
    }

    tTrackSeg *seg = track->seg;

    // memory leak
    std::cout << "dump track data to " << canonicalize_file_name(path) << "\n";
    fputs("startLX, startLY, startRX, startRY, endLX, endLY, endRX, endRY\n", f);

    for (int i = 0; i < track->nseg && seg != NULL; i++)
    {
        t3Dd sl = seg->vertex[TR_SL];
        t3Dd sr = seg->vertex[TR_SR];
        t3Dd el = seg->vertex[TR_EL];
        t3Dd er = seg->vertex[TR_ER];

        fprintf(
                f,
                "%f, %f, %f, %f, %f, %f, %f, %f\n",
                sl.x, sl.y, sr.x, sr.y, el.x, el.y, er.x, er.y
                );

        seg = seg->next;
    }

    fclose(f);
}

void
TrackModel::dumpDataToFile(const char* path)
{
    FILE *f = fopen(path, "w");

    if (f == 0x00)
    {
        return;
    }
    
    // memory leak
    std::cout << "dump line data to " << canonicalize_file_name(path) << "\n";
    fputs("x, y, trackPos, pointLim, rationalLim\n", f);
    
    float trackPos = 0;
    for(size_t i = 0; i < segments.size(); i++)
    {
        LineSegment *seg = getSegment(i);
        LineSegment *nseg = getSegment(i + 1);
        
        v2t<float> pos = seg->pos();
        float segLng = (nseg->pos() - pos).len();
        float x = pos.x;
        float y = pos.y;
        float pL = seg->maximumSpeedAtPoint;
        float rL = seg->rationalSpeedLimit;
        
        fprintf(
                f,
                "%f, %f, %f, %f, %f\n",
                x, y, trackPos, pL, rL
                );
        
        trackPos += segLng;
    }

    fclose(f);
}




















