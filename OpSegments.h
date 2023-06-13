#ifndef OpSegments_DEFINED
#define OpSegments_DEFINED

#include "OpEdges.h"

struct OpSegment;

struct OpSegments {
    OpSegments(OpContours& contours);
    static void AddLineCurveIntersection(OpSegment* opp, OpSegment* seg);
    void findCoincidences();
    FoundIntersections findIntersections();

#if OP_DEBUG_DUMP
    DUMP_COMMON_DECLARATIONS();
    DUMP_IMPL_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    std::vector<OpSegment*> inX;
};

#endif
