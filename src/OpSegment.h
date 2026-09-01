// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegment_DEFINED
#define OpSegment_DEFINED

#include "OpEdge.h"
#include "OpIntersection.h"

struct OpContour;
enum class AliasType;

#define ChopUnsortable_Enums \
    OP_ENUM_MEMBER(none), \
    OP_ENUM_MEMBER(prior), \
    OP_ENUM_MEMBER(next)

enum class ChopUnsortable {
	ChopUnsortable_Enums
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
		, neighborEnd(EdgeMatch::none)
		, chop(ChopUnsortable::none)
		, connects(false)
		, loops(false) {
	}

//	void check(std::vector<FoundEdge>* edges, OpEdge* test, EdgeMatch , OpPoint match);
	void reset();
	DUMP_DECLARATIONS

	OpEdge* edge;
	float perimeter;
	float closeSq;  // distance to make a loop if this edge is chosen
	float distSq;  // used to track closest edge if no exact match was found
	int index;  // used to track entry in linkups to remove after use
	EdgeMatch whichEnd;
	EdgeMatch neighborEnd;  // !!! not sure why active neighbor doesn't set which end
	ChopUnsortable chop;  // true if edge has one or more linked unsortables to be removed
	bool connects; // true if edge connects in correct direction with existing link
	bool loops;  // true if edge when connected to existing link forms a loop
};

#define PtType_Enums \
	OP_ENUM_MEMBER(noMatch),   /* pt not close to alias or original */ \
	OP_ENUM_MEMBER(original),  /* pt maps to alias (if not equal to existing original, pt->alias added) */ \
	OP_ENUM_MEMBER(isAlias),   /* pt is existing alias */ \
	OP_ENUM_MEMBER(mapSegment) /* move end and/or sect of segment  */ \

enum class PtType {
    PtType_Enums
};

struct SegPt {
	DUMP_DECLARATIONS

	OpPoint pt;
	PtType ptType;
};

enum class PrefFound {
    disabled,
    retry,
    ok
};

enum class MatchZero {
	no,
	yes
};

enum class AllowLinked {
	no,
	yes
};

#undef OP_X

struct OpSegment {
	OpSegment(PathOpsV0Lib::Contour* , PathOpsV0Lib::AddCurve);
	bool activeAtT(OpEdge* , EdgeMatch , MatchZero , std::vector<FoundEdge>& ) const; // true if pal
	bool activeNeighbor(const OpEdge* , EdgeMatch , AllowLinked , std::vector<FoundEdge>& ) const; // true if pal
	OpIntersection* addCoin(const OpPtT& , int coinID, MatchEnds , CoinOpp , const OpSegment* o  
			OP_LINE_FILE_ARGS());
	void addDisjointIntersections();
	OpIntersection* addSegBase(const OpPtT&  OP_LINE_FILE_DEF(const OpSegment* o));
	OpIntersection* addSegEnd(const OpPtT& , const OpSegment* o  OP_LINE_FILE_ARGS());
	OpIntersection* addSegSect(const OpPtT& , const OpSegment* o  OP_LINE_FILE_ARGS());
	OpIntersection* addUnsectable(const OpPtT& , int usectID, MatchEnds , const OpSegment* o 
			OP_LINE_FILE_ARGS());
	OpPtT alignToEnd(OpPoint oppPt) const;
	WindingCondition apply();
	void betweenCoincidence();
	int coinID(bool flipped);
	void disableSmall();
	OpPtT distance(const OpPtT& segPtT, OpSegment* opp);
	bool endMoved() const { 
		return c.end != c.c.data->end; }
	OpEdge* findEnabled(const OpPtT& , EdgeMatch ) const;
	float findLineT(OpPoint opp);
	bool fixCCSects();
    void init();
	bool isFinite() const {
		return c.start.isFinite() && c.end.isFinite(); } 
	void makeCoins();
	void makeEdge(OP_LINE_FILE_NP_ARGS());
	void makeEdges();
	void makePals();
	OpPtT matchEnd(OpPoint opp);
	MatchReverse matchEnds(const LinePts& opp) const;
	MatchReverse matchEnds(const OpSegment* opp) const;
	bool mergeEndPoints();
	bool mergeIntersections();
	void mergeMultiple(OpPoint masterPt, int masterID, OpPoint mergePt, int mergeID);
//	bool mergeOpposites();
	PrefFound moveSects(OpPtT match, OpPoint dest);	// move matching sects and cleanup segment state
	bool moveWinding(OpSegment* opp, bool backwards);
	void manyCoincidences();
	bool nearby(float t, const OpSegment* opp) const;
	int nextID() const;
	void normalize();
	OpPtT ptAtT(const OpPtT& ) const;
	void remap(OpPoint oldAlias, OpPoint newAlias);  // local remap
	void setDisabled(OP_LINE_FILE_NP_ARGS());
	void setEndsUnmerged();
	void setUnmerged();
	bool simpleEnd(const OpEdge* ) const;  // true if edge end connects to only one segment
	bool simpleStart(const OpEdge* ) const;  // true if edge start connects to only one segment
	bool startMoved() const { 
		return c.start != c.c.data->start; }
	OpVector threshold() const;
	float thresholdLength() const;
	void transferCoins();
//	void tripleSect();  // check intersections for three or more identical points
	bool zeroSmall(bool zeroStart);

#if OP_DEBUG
	bool debugFail() const;
	bool debugSuccess() const;
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP || OP_DEBUGGER
	OpSegment();
	float debugFindAxisT(Axis , float start, float end, float oppXY);
    std::string debugEdges(DebugLevel ) const;
    std::string debugFull(DebugLevel ) const;
	#define OP_X(Thing) \
	std::string debugDump##Thing() const; \
	void dump##Thing() const;
	SEGMENT_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
#endif
#if OP_DEBUG_SERIALIZE
	std::string debugDumpID() const;
    bool dumpInitialized() const;
	#include "OpDebugDeclarations.h"
#endif

	OpContour* contour;
	OpCurve c;
//	OpPointBounds ptBounds;
	OpIntersections sects;
//	OpIntersections smallSects;	!!! start here; put small in sects array; add flag to op intersection
	std::vector<OpEdge> edgeList;
	OpWinding winding;
	int id;     // used to normalize each end point once
	bool disabled; // winding has canceled this edge out
	bool endsMerged = false;
//	bool willDisable;  // moveTo aligned ends; will be disabled by disable small segments
	bool hasCoin;
	bool hasPals = false;
	bool hasUnsectable;
	bool merged = false;
	bool oppMerged = false;
//	bool startMoved;
//	bool endMoved;
#if OP_DEBUGGER
	uint32_t debugColor = debugBlack;
#endif
    OP_LINE_FILE_DECLARE(debugSetDisabled)
};

#endif
