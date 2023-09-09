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

enum class MatchSect {
    allow,  // any ends of segment can match
    existing    // consecutive segments cannot match
};

struct FoundEdge {
    FoundEdge(OpEdge* e, EdgeMatch w) 
        : edge(e)
        , index(-1)
        , whichEnd(w)
        , connects(false)
        , loops(false) {
    }

	FoundEdge(OpEdge* e, int i, EdgeMatch w)
		: edge(e)
		, index(i)
		, whichEnd(w)        
        , connects(false)
        , loops(false) {
	}

#if OP_DEBUG_DUMP
	void dump() const;
	void dumpDetail() const;
#endif

    OpEdge* edge;
    int index;  // used to track entry in linkups to remove after use
    EdgeMatch whichEnd;
    bool connects; // true if edge connects in correct direction with existing link
    bool loops;  // true if edge when connected to existing link forms a loop
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
    OpIntersection* addCoin(const OpPtT& , int coinID, MatchEnds coinEnd  
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string , SectReason , const OpSegment* o));
    OpIntersection* addUnsectable(const OpPtT& , int usectID, MatchEnds end, const OpSegment* o 
            OP_DEBUG_PARAMS(IntersectMaker , int , std::string));
    void apply();
    int coinID(bool flipped) const;
    void complete(OpContour* );
    OpEdge* findEnabled(const OpPtT& , EdgeMatch ) const;
    float findPtT(float start, float end, OpPoint opp) const;
    FoundPtT findPtT(Axis , float start, float end, float oppXY, float* result) const;
    FoundPtT findPtT(float start, float end, OpPoint opp, float* result) const;
    FoundPtT findPtT(const OpPtT& start, const OpPtT& end, OpPoint opp, float* result) const;
    // count and sort extrema; create an edge for each extrema + 1
    void makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file));
    void makeEdges();
    MatchEnds matchEnds(const OpSegment* opp, bool* reversed, MatchSect ) const;
    MatchEnds matchExisting(const OpSegment* opp) const;
    int nextID() const { 
        return nextID(contour); }
    int nextID(OpContour* ) const;
	void setDisabled(OP_DEBUG_CODE(ZeroReason reason));
    void windCoincidences();

    bool debugFail() const;
    bool debugSuccess() const;
#if OP_DEBUG
    void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
    void dumpCount() const;
    #define OP_X(Thing) \
    void dump##Thing() const;
    SEGMENT_DETAIL
    EDGE_OR_SEGMENT_DETAIL
    #undef OP_X
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
    bool debugContains(const OpEdge* ) const; // distinguishes owned edges from temporary edges
    void draw() const;
#endif

    OpContour* contour;
    OpCurve c;
    OpPointBounds ptBounds;
    OpIntersections sects;
    std::vector<OpEdge> edges;
    OpWinding winding;
    bool disabled; // winding has canceled this edge out
    bool recomputeBounds;
    int id;     // !!! could be debug only; currently used to disambiguate sort, may be unneeded
#if OP_DEBUG
    SectReason debugStart;
    SectReason debugEnd;
    ZeroReason debugZero;
#endif
};

#endif
