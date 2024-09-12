// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegment_DEFINED
#define OpSegment_DEFINED

#include "OpEdge.h"
#include "OpIntersection.h"
#include "OpTightBounds.h"
#include <vector>

struct OpContour;

enum class MatchSect {
    allow,  // any ends of segment can match
    existing    // consecutive segments cannot match
};

enum class ChopUnsortable {
    none,
    prior,
    next
};

struct FoundEdge {
    FoundEdge() {
        reset();
    }

    FoundEdge(OpEdge* e, EdgeMatch w, int i = -1) 
        : edge(e)
        , perimeter(OpInfinity)
        , closeSq(OpInfinity)
        , distSq(0)
        , index(i)
        , whichEnd(w)
        , chop(ChopUnsortable::none)
        , connects(false)
        , loops(false) {
    }

    void check(std::vector<FoundEdge>* edges, OpEdge* test, EdgeMatch , OpPoint match);
    void reset();
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

    OpEdge* edge;
    float perimeter;
    float closeSq;  // distance to make a loop if this edge is chosen
    float distSq;  // used to track closest edge if no exact match was found
    int index;  // used to track entry in linkups to remove after use
    EdgeMatch whichEnd;
    ChopUnsortable chop;  // true if edge has one or more linked unsortables to be removed
    bool connects; // true if edge connects in correct direction with existing link
    bool loops;  // true if edge when connected to existing link forms a loop
};

struct OpSegment {
    OpSegment(PathOpsV0Lib::AddCurve , PathOpsV0Lib::AddWinding );
    bool activeAtT(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& , bool* hadLinkTo) const;
    bool activeNeighbor(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& ) const;
    OpIntersection* addEdgeSect(const OpPtT&    
            OP_LINE_FILE_DEF(const OpEdge* e, const OpEdge* o));
    OpIntersection* addSegBase(const OpPtT&  
            OP_LINE_FILE_DEF(const OpSegment* o));
    OpIntersection* addSegSect(const OpPtT& , const OpSegment* o  
            OP_LINE_FILE_DEF());
    OpIntersection* addCoin(const OpPtT& , int coinID, MatchEnds , const OpSegment* o  
            OP_LINE_FILE_DEF());
    OpIntersection* addUnsectable(const OpPtT& , int usectID, MatchEnds , const OpSegment* o 
            OP_LINE_FILE_DEF());
    void apply();
    void betweenIntersections();
    int coinID(bool flipped) const;
//    void complete();
    OpEdge* findEnabled(const OpPtT& , EdgeMatch ) const;
    float findAxisT(Axis , float start, float end, float oppXY);
//    float findNearbyT(const OpPtT& start, const OpPtT& end, OpPoint opp) const;
    float findValidT(float start, float end, OpPoint opp);
    // count and sort extrema; create an edge for each extrema + 1
    void makeEdge(OP_LINE_FILE_NP_DEF());
    void makeEdges();
    void makePals();
    MatchReverse matchEnds(const LinePts& opp) const;
    MatchReverse matchEnds(const OpSegment* opp) const;
//    MatchEnds matchExisting(const OpSegment* opp) const;
    void moveTo(float t , OpPoint );  // move segment/sect point to match another endpont
    void moveWinding(OpSegment* opp, bool backwards);
    bool nearby(float t, const OpSegment* opp) const;
    int nextID() const;
    void newWindCoincidences();  // !!! will eventually replace wind coincidences
    void setBounds();
	void setDisabled(OP_LINE_FILE_NP_DEF());
    void windCoincidences();

    bool debugFail() const;
    bool debugSuccess() const;
#if OP_DEBUG_VALIDATE
    void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
    OpSegment();
    void dumpCount() const;
    #define OP_X(Thing) \
    std::string debugDump##Thing() const; \
    void dump##Thing() const;
    DEBUG_DUMP
    SEGMENT_DETAIL
    EDGE_OR_SEGMENT_DETAIL
    #undef OP_X
    #include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
    bool debugContains(const OpEdge* ) const; // distinguishes owned edges from temporary edges
#endif

    OpContour* contour;
    OpCurve c;
    OpPointBounds ptBounds;
    OpRect closeBounds;
    OpIntersections sects;
    std::vector<OpEdge> edges;
    OpWinding winding;
    int id;     // !!! could be debug only; currently used to disambiguate sort, may be unneeded
    bool disabled; // winding has canceled this edge out
    bool hasCoin;
    bool hasUnsectable;
#if OP_DEBUG_IMAGE
	uint32_t debugColor;
#endif
#if OP_DEBUG_MAKER
	OpDebugMaker debugSetDisabled;
#endif
};

#endif
