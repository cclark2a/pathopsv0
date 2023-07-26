#ifndef OpSegment_DEFINED
#define OpSegment_DEFINED

#include "OpEdge.h"
#include "OpIntersection.h"
#include "OpTightBounds.h"
#include <vector>

struct OpContour;

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

enum class MatchSect {
    allow,  // any ends of segment can match
    existing    // consecutive segments cannot match
};

struct FoundEdge {
    FoundEdge(OpEdge* e, EdgeMatch w) 
        : edge(e)
        , whichEnd(w) {
    }

    OpEdge* edge;
    EdgeMatch whichEnd;
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
    OpSegment* segment;  // segment new intersection will be added to
    const OpIntersection& intersection;   // intersection point and t are copied from
};

struct OpSegment {
    OpSegment(const OpCurve& pts, OpType type, OpContour*  
            OP_DEBUG_PARAMS(SectReason , SectReason ));
    OpSegment(const LinePts& pts, OpContour*  OP_DEBUG_PARAMS(SectReason , SectReason ));
    bool activeAtT(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& ) const;
    bool activeNeighbor(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& ) const;
    OpIntersection* addEdgeSect(const OpPtT&  
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason ,
            const OpEdge* e, const OpEdge* o));
    OpIntersection* addSegBase(const OpPtT&  
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason , const OpSegment* o));
    OpIntersection* addSegSect(const OpPtT&  
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason , const OpSegment* o));
    OpIntersection* addCoin(const OpPtT& , int coinID  
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason , const OpSegment* o));
//    OpIntersection* addIntersection(const OpPtT& , SectFlavor  
//            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason , const OpIntersection* ,
//            const OpEdge* e, const OpEdge* o));
    OpIntersection* addUnsectable(const OpPtT& , int unsectableID, const OpSegment* o 
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string));
    OpIntersection* alreadyContains(const OpPtT& edgePtT, const OpSegment* opp) const;
    void apply();
//    void calcBounds(); // recompute tight bounds from adjusted intersections
    int coinID(bool flipped) const;
    void complete(OpContour* );
    bool containsIntersection(OpPtT , const OpSegment* ) const;
    OpEdge* findEnabled(OpPtT , EdgeMatch ) const;
//    void fixEdges(OpPoint alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID));
//    void fixIntersections(OpPoint alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID));
    float findPtT(float start, float end, OpPoint opp) const;
    FoundPtT findPtT(Axis , float start, float end, float oppXY, float* result) const;
    FoundPtT findPtT(float start, float end, OpPoint opp, float* result) const;
    FoundPtT findPtT(const OpPtT& start, const OpPtT& end, OpPoint opp, float* result) const;
//    void intersectEdge();
    std::vector<OpIntersection*> intersectRange(const OpSegment* );
    // count and sort extrema; create an edge for each extrema + 1
    void makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file));
    void makeEdges();
    MatchEnds matchEnds(const OpSegment* opp, bool* reversed, MatchSect ) const;
    MatchEnds matchExisting(const OpSegment* opp) const;
//    void missingCoincidence();  // find missing intersections / edges
//    void missingCoincidence(const OpPtT& s, const OpPtT& e, OpSegment* , std::vector<int>&, 
//            std::vector<MissingIntersection>& ) const;
    bool resolveCoincidence();
//    void resolvePoints();
	void setDisabled(OP_DEBUG_CODE(ZeroReason reason));
    void sortIntersections();
//    bool splitAtWinding(const std::vector<const OpEdge*>& windingChanges, const OpEdge* first,
//            int direction  OP_DEBUG_PARAMS(const OpIntersection* last, EdgeMatch oppositeMatch,
//            const OpEdge* firstEdge, SectReason ));
    void windCoincidences();
    int unsectableID() const;
//    OpEdge* visibleAdjacent(OpEdge* , const OpPtT& );

    bool debugFail() const;
    bool debugSuccess() const;
#if OP_DEBUG
    bool debugContains(OpPtT , const OpSegment* opp) const;  // check for duplicates
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
    std::vector<OpEdge> edges;
    // all intersections are stored here before edges are rewritten
    std::vector<OpIntersection*> intersections;
    OpWinding winding;
    bool disabled; // winding has canceled this edge out
    bool recomputeBounds;
    bool resortIntersections;
    int id;     // !!! could be debug only; currently used to disambiguate sort, may be unneeded
#if OP_DEBUG
    SectReason debugStart;
    SectReason debugEnd;
    ZeroReason debugZero;
#endif
};

#endif
