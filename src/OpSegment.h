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

//	void check(std::vector<FoundEdge>* edges, OpEdge* test, EdgeMatch , OpPoint match);
	void reset();
	DUMP_DECLARATIONS

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

enum class PtType {
	noMatch,   // pt not close to alias or original
	original,  // pt maps to alias (if not equal to existing original, pt->alias added)
	isAlias,   // pt is existing alias
	mapSegment // move end and/or sect of segment 
};

struct SegPt {
	DUMP_DECLARATIONS

	OpPoint pt;
	PtType ptType;
};

struct OpSegment {
	OpSegment(PathOpsV0Lib::AddCurve , PathOpsV0Lib::AddWinding );
	bool activeAtT(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& ) const; // true if pal
	bool activeNeighbor(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& ) const; // true if pal
	void addAlias(OpPoint original, OpPoint alias);
//	OpIntersection* addEdgeSect(const OpPtT&    
//			OP_LINE_FILE_DEF(const OpEdge* e, const OpEdge* o));
	OpIntersection* addSegBase(const OpPtT&  
			OP_LINE_FILE_DEF(const OpSegment* o));
	OpIntersection* addSegSect(const OpPtT& , const OpSegment* o  
			OP_LINE_FILE_ARGS());
	OpIntersection* addCoin(const OpPtT& , int coinID, MatchEnds , const OpSegment* o  
			OP_LINE_FILE_ARGS());
	OpIntersection* addUnsectable(const OpPtT& , int usectID, MatchEnds , const OpSegment* o 
			OP_LINE_FILE_ARGS());
//    OpPoint aliasOriginal(MatchEnds ) const;
	OpPtT alignToEnd(OpPoint oppPt) const;
	void apply();
	void betweenIntersections();
	SegPt checkAliases(OpPtT );
	int coinID(bool flipped);
//    void complete();
	void disableSmall();
	OpEdge* findEnabled(const OpPtT& , EdgeMatch ) const;
//	float findAxisT(Axis , float start, float end, float oppXY);
	void findMissingEnds();
//    float findNearbyT(const OpPtT& start, const OpPtT& end, OpPoint opp) const;
	float findValidT(float start, float end, OpPoint opp);
	// count and sort extrema; create an edge for each extrema + 1
	bool isFinite() const {
		return closeBounds.isFinite(); } 
//    bool isSimple() const {
//        return 1 == edges.size() && 2 == sects.i.size(); }
	bool isSmall();
	void makeCoins();
	void makeEdge(OP_LINE_FILE_NP_ARGS());
	void makeEdges();
	void makePals();
	OpPtT matchEnd(OpPoint opp) const;
	MatchReverse matchEnds(const LinePts& opp) const;
	MatchReverse matchEnds(const OpSegment* opp) const;
//    MatchEnds matchExisting(const OpSegment* opp) const;
	OpPoint mergePoints(OpPtT segPtT, OpSegment* opp, OpPtT oppPtT);
	OpPoint movePt(OpPtT match, OpPoint dest);  // move segment/sect point to match another endpont
	bool moveWinding(OpSegment* opp, bool backwards);
	bool nearby(float t, const OpSegment* opp) const;
	int nextID() const;
//    void newWindCoincidences();  // !!! will eventually replace wind coincidences
	void normalize();
	OpPtT ptAtT(const OpPtT& ) const;
	void remap(OpPoint oldAlias, OpPoint newAlias);  // local remap
//	OpPoint remapPts(OpPoint oldAlias, OpPoint newAlias);  // call through
	void resetBounds();
	void setDisabled(OP_LINE_FILE_NP_ARGS());
	bool simpleEnd(const OpEdge* ) const;  // true if edge end connects to only one segment
	bool simpleStart(const OpEdge* ) const;  // true if edge start connects to only one segment
	OpVector threshold() const;
	void transferCoins();
//    void windCoincidences();

#if OP_DEBUG
	bool debugFail() const;
	bool debugSuccess() const;
#endif
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
	bool willDisable;  // moveTo aligned ends; will be disabled by disable small segments
	bool hasCoin;
	bool hasUnsectable;
	bool startMoved;
	bool endMoved;
#if OP_DEBUG_IMAGE
	uint32_t debugColor;
#endif
#if OP_DEBUG_MAKER
	OpDebugMaker debugSetDisabled;
#endif
};

#endif
