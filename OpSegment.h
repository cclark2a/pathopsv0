#ifndef OpSegment_DEFINED
#define OpSegment_DEFINED

#include "OpEdge.h"
#include "OpEdges.h"
#include "OpIntersection.h"
#include "OpTightBounds.h"
#include <vector>

struct OpContour;
struct OpContours;

enum class AllowReversal {
    no,
    yes
};

enum class MatchEnds {
    none,
    start,
    end,
    both
};

struct FoundEdge {
    FoundEdge(OpEdge* e, EdgeMatch w, AllowReversal reversal = AllowReversal::no) 
        : edge(e)
        , whichEnd(w)
        , reverse(reversal) {
    }

    OpEdge* edge;
    EdgeMatch whichEnd;
    AllowReversal reverse;
};

struct OpSegment {
    OpSegment(const OpPoint pts[], OpType type, OpContour* parent);
    OpSegment(const OpPoint pts[3], float w, OpContour* parent);
    OpSegment(const OpPoint& pt1, const OpPoint& pt2, OpContour* parent);
    void activeAtT(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& , AllowReversal ) const;
    void apply();
    void calcBounds(); // recompute tight bounds from adjusted intersections
    int coinID(bool flipped);
    void complete();
    bool containsSect(float t, const OpSegment* opp) const;
    OpEdge* findActive(OpPtT , EdgeMatch ) const;
    void findCoincidences();
    void fixEdges(const OpPtT& alias, OpPoint master);
    void findExtrema();
    void fixIntersections(OpPoint alias, OpPoint master);
    float findPtT(OpPoint opp) const;
    static void flip(WindZero* windZero);
    void intersectEdge();
    // count and sort extrema; create an edge for each extrema + 1
    void makeEdges();
    MatchEnds matchEnds(const OpSegment* opp, bool* reversed) const;
    void matchIntersections();
    void resolveCoincidence();
    void resolvePoints();
    void setCoincident(XyChoice , OpPoint newA, OpPoint newB, bool flipped, OpSegment* oppSegment
            OP_DEBUG_PARAMS(const OpEdge& opp, const OpEdge& edge));
    void sortIntersections();
    bool validEdge(float startT, float endT) const;  // check if edge crosses extrema or inflection
#if OP_DEBUG
    void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
    void dumpCount() const;
    void dumpDetail() const;
    void dumpEnd() const;
    void dumpFull() const;
    void dumpStart() const;
    DEBUG_COMMON_DECLARATIONS();
    DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    OpContour* contour;
    OpCurve c;
    OpPointBounds pointBounds;
    OpTightBounds tightBounds;
    std::vector<OpEdge> edges;
    // all intersections are stored here before edges are rewritten
    std::vector<OpIntersection> intersections;
    OpWinding winding;
    bool recomputeBounds;
    int id;     // !!! could be debug only; is currently used to disambiguate sort, but that may be unneeded
};

struct OpSegments {
    OpSegments(OpContours& contours);
    static IntersectResult AddIntersection(OpSegment* opp, OpSegment* seg);
    void findCoincidences();
    FoundIntersections findIntersections();

#if OP_DEBUG_DUMP
    void dump() const;
    void dumpDetail() const;
    void dumpEdges() const;
    void dumpFull() const;
    void dumpHex() const;
    void dumpIntersections() const;
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    std::vector<OpSegment*> inX;
};

#endif
