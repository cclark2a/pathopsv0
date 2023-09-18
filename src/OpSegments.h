// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegments_DEFINED
#define OpSegments_DEFINED

#include "OpMath.h"

struct OpContours;
struct OpSegment;
enum class FoundIntersections;
enum class IntersectResult;

struct OpSegments {
    OpSegments(OpContours& contours);
    static void AddLineCurveIntersection(OpSegment* opp, OpSegment* seg);
    void findCoincidences();
//    void findLineCoincidences();
    FoundIntersections findIntersections();
    IntersectResult lineCoincidence(OpSegment* seg, OpSegment* opp);

#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    std::vector<OpSegment*> inX;
};

#endif
