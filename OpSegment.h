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

enum class FoundPtT {
    single,
    multiple
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
        assert(e);
        assert(e->winding.visible());
        assert(e->active_impl);
    }

    OpEdge* edge;
    EdgeMatch whichEnd;
    AllowReversal reverse;
};

struct MissingIntersection {
    MissingIntersection(const OpPtT& start_, const OpPtT& end_,
            OpSegment* segment_, const OpIntersection& intersection_) 
        : start(start_)
        , end(end_)
        , segment(segment_)
        , intersection(intersection_) {
    }
    const OpPtT& start;
    const OpPtT& end;
    OpSegment* segment; // segment new intersection will be added to
    const OpIntersection& intersection;   // intersection point and t are copied from
};

struct OpSegment {
    OpSegment(const OpPoint pts[], OpType type, OpContour* parent);
    OpSegment(const OpPoint pts[3], float w, OpContour* parent);
    OpSegment(OpPoint pt1, OpPoint pt2, OpContour* parent);
    void activeAtT(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& , AllowReversal ) const;
    OpIntersection* addIntersection(const OpPtT&  OP_DEBUG_PARAMS(IntersectMaker maker));
    OpIntersection* addIntersection(const OpPtT& , int coinID  OP_DEBUG_PARAMS(IntersectMaker maker));
    OpIntersection* addIntersection(const OpPtT& , SelfIntersect  OP_DEBUG_PARAMS(IntersectMaker maker));
    void apply();
    void calcBounds(); // recompute tight bounds from adjusted intersections
    int coinID(bool flipped);
    void complete();
    bool containsSect(float t, const OpSegment* opp) const;
    OpEdge* findActive(OpPtT , EdgeMatch ) const;
    void findExtrema();
    void fixEdges(const OpPtT& alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID));
    void fixIntersections(OpPoint alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID));
    float findPtT(float start, float end, OpPoint opp) const;
    FoundPtT findPtT(float start, float end, OpPoint opp, float* result) const;
    static void flip(WindZero* windZero);
    void intersectEdge();
    // count and sort extrema; create an edge for each extrema + 1
    void makeEdges();
    MatchEnds matchEnds(const OpSegment* opp, bool* reversed) const;
    void missingCoincidence();  // find missing intersections / edges
    void missingCoincidence(const OpPtT& s, const OpPtT& e, OpSegment* , std::vector<int>&, 
            std::vector<MissingIntersection>& ) const;
    bool resolveCoincidence();
    void resolvePoints();
    void setCoincident(XyChoice , OpPoint newA, OpPoint newB, bool flipped, OpSegment* oppSegment
            OP_DEBUG_PARAMS(const OpEdge& opp, const OpEdge& edge));
    void sortIntersections();
    bool splitAtWinding(const std::vector<const OpEdge*>& windingChanges, const OpEdge* first,
            int direction  OP_DEBUG_PARAMS(const OpIntersection* last, EdgeMatch oppositeMatch));
    bool validEdge(float startT, float endT) const;  // check if edge crosses extrema or inflection
    OpEdge* visibleAdjacent(OpEdge* , const OpPtT& );
#if OP_DEBUG
    void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
    void dumpCount() const;
    DEBUG_COMMON_DECLARATIONS();
    DUMP_COMMON_DECLARATIONS();
    DUMP_IMPL_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
    bool debugContains(const OpEdge* ) const; // distinguishes owned edges from temporary edges
    void draw() const;
#endif

    OpContour* contour;
    OpCurve c;
    OpPointBounds ptBounds;
    OpTightBounds tightBounds;
    std::vector<OpEdge> edges;
    // all intersections are stored here before edges are rewritten
    std::vector<OpIntersection*> intersections;
    OpWinding winding;
    bool recomputeBounds;
    bool resortIntersections;
    int id;     // !!! could be debug only; currently used to disambiguate sort, may be unneeded
};

struct OpSegments {
    OpSegments(OpContours& contours);
    static IntersectResult AddIntersection(OpSegment* opp, OpSegment* seg);
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
