// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpEdge_DEFINED
#define OpEdge_DEFINED

#include "OpCurve.h"
#include "OpTightBounds.h"
#include "OpOperators.h"
#include "OpWinding.h"
#include <vector>

struct FoundEdge;
struct OpContext;
struct OpCurveCurve;
struct OpIntersection;
struct OpOutPath;
struct OpSegment;

enum class CurveRef;
enum class FoundPtT;

enum class NewEdge {
	isLeft,
	isRight,
};

inline NewEdge operator!(const NewEdge& a) {
	return (NewEdge) !static_cast<int>(a);
}

enum RotateVertical {
	rotateVertical,
};

enum class IntersectResult {
	fail,
	no,
	yes,
	coincident,
	maybe,
};

enum EdgeRay {
	xRay,
	yRay,
};

enum class EdgeMatch : int8_t {
	none = -1,
	start,
	end,
};

inline EdgeMatch operator!(EdgeMatch m) {
	OP_ASSERT(EdgeMatch::start == m || EdgeMatch::end == m);
	return static_cast<EdgeMatch>(!static_cast<int>(m));
}

enum class EdgeFail : uint8_t {
	none,
//	center,
	horizontal,
	vertical,
};

struct EdgePal {
	EdgePal(OpEdge* e, float c, float tIn, bool r);  // called when winder can't resolve order
	EdgePal(OpEdge* e, bool r  OP_DEBUG_PARAMS(int uID)); // called when edge maker is between unsectables
	OpPoint matchPt(EdgeMatch ) const;

#if OP_DEBUG_DUMP
	EdgePal() {}
	DUMP_DECLARATIONS
#endif

	OpEdge* edge;
	float cept;  // where normal intersects edge (e.g. for home, axis horz: center.x)
	float edgeInsideT;  // !!! t value from 0 to 1 within edge range (seems bizarre)
	bool reversed;
	OP_DEBUG_CODE(int debugUID);  // unsect id from sect in edge's segment
};

enum class FindCept {
	ok,
	addPal,
	retry,
	unsectable,  // pal already added
	unsortable
};

enum class DistEnd {
	front = -1,
	back = 1
};

inline DistEnd operator!(DistEnd de) {
	return static_cast<DistEnd>(-static_cast<int>(de));
}

// captures ray info from edge that intersects other edges, horizontally or vertically
struct SectRay {
	SectRay()
		: normal(OpNaN)
		, homeCept(OpNaN)
		, homeT(OpNaN)
		, axis(Axis::neither)
		, firstTry(true)
	{
	}
	void addPals(OpEdge* );
	bool checkOrder(const OpEdge* ) const;
	const EdgePal* end(DistEnd e) const {
		return DistEnd::front == e ? &distances.front() : &distances.back(); }
	FindCept findIntercept(OpEdge* home, OpEdge* test);
	const EdgePal* find(const OpEdge* ) const;  // returns edge in distances
	const EdgePal* next(const EdgePal* dist, DistEnd e) const {
		return dist + (int) e; }
	bool sectsAllPals(const OpEdge* ) const;  // returns if edge + all of its pals are in distances
	void sort();
	DUMP_DECLARATIONS

	std::vector<EdgePal> distances;
	OpVector homeTangent;  // used to determine if unsectable edge is reversed
	float normal;  // ray used to find windings on home edge (e.g., axis: h, center.y)
	float homeCept;  // intersection of normal on home edge (e.g., axis: h, center.x)
	float homeT;  // value from 0 to 1 within edge range (akin to edgeInsideT)
	Axis axis;
	bool firstTry;  // used to cache unsectable test
};

enum class SectType {
	none,
	endHull,  // intersection is close to or equal to curve end point
	controlHull,  // intersection is on hull between end and control, or pair of control points
	midHull,  // hull intersects, but not near end point
	snipLo,   // snip at t lower than intersection
	snipHi    // snip at t higher than intersection
};

// intersections of opposite curve's hull and this edge's segment
struct HullSect {
	HullSect(const OpPtT& ptT, float dist, SectType st, const OpEdge* o)
		: opp(o)
		, sect(ptT)
		, oppDist(dist)
		, type(st) {
	}
#if OP_DEBUG_DUMP
	HullSect() {}
	DUMP_DECLARATIONS
#endif
	const OpEdge* opp;
	OpPtT sect;			// point and t of intersection with hull on this edge
	float oppDist;		// if ptT came from edge end: the distance from there to the opposite curve
	SectType type;		// separate from match: intersection origin (or near origin)
};

struct OpHulls {
	bool add(const OpPtT& ptT, OpVector threshold, float dist, SectType st, 
			const OpEdge* o = nullptr);
	void clear() { h.clear(); }
	bool nudgeDeleted(const OpEdge& edge, const OpCurveCurve& cc, CurveRef which);
	void sort(bool useSmall);
	DUMP_DECLARATIONS
	OP_DEBUG_CODE(bool debugSectCandidates(int index, const OpEdge& edge) const);
	OP_DEBUG_VALIDATE_CODE(void debugValidate() const);

	std::vector<HullSect> h;
};

enum class CalcFail {
	none,
	fail,
};

// !!! worrisome: normal direction below is downLeft (correct); why is this downRight?
enum class EdgeDirection {
	downRight = -1,
	upLeft = 1,
};

enum class ResolveWinding {
	resolved,
	loop,
	retry,
	fail,
};

enum class Unsectable {
	none,
	single,
	multiple,
};

enum class Unsortable {
	none,
	addCalcFail,
	addCalcFail2,
	betweenCoins,
	filler,
	homeUnsectable,
	noMidT,
	noNormal,
	rayTooShallow,
	tooManyTries,
	underflow
};

enum class InOutput {
	no,
	yes
};

#if OP_DEBUG
enum class LeadingLoop {
	in,
	will,
};
#endif

struct CoinPal {
	friend bool operator==(CoinPal a, CoinPal b) {
		return a.coinID == b.coinID;
	}

	OpSegment* opp;
	int coinID;
};

struct OpEdge {
		friend struct OpEdgeStorage;
#if !OP_DEBUG_DUMP
private:
#endif
	OpEdge()	// note : not all release values are zero (which end)
		: priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
		, upright_impl( { SetToNaN::dummy, SetToNaN::dummy } )
		, winding(WindingUninitialized::dummy)
		, sum(WindingUninitialized::dummy)
		, many(WindingUninitialized::dummy)
		, whichEnd_impl(EdgeMatch::none)
		, rayFail(EdgeFail::none)
		, windZero(WindZero::unset)
		, isUnsortable(Unsortable::none)
		, active_impl(false)
		, inLinkups(false)
		, linkHead(false)
		, inOutput(false)
		, disabled(false)
		, isUnsplitable(false)
		, ccEnd(false)
		, ccLarge(false)
		, ccOverlaps(false)
		, ccSmall(false)
		, ccStart(false)
		, centerless(false)
//		, windPal(false)
		, startSeen(false)
		, endSeen(false)
	{
#if OP_DEBUG // a few debug values are also nonzero
		id = 0;
		startDist = OpNaN;
		endDist = OpNaN;
		segment = nullptr;
		startT = OpNaN;
		endT = OpNaN;
		debugMatch = nullptr;
		debugZeroErr = nullptr;
		debugOutPath = 0;
		debugParentID = 0;
		debugDepth = 0;
		debugRayMatch = 0;
#endif
#if OP_DEBUG_DUMP
		dumpContours = nullptr;
#endif
#if OP_DEBUG_IMAGE
		debugColor = debugBlack;
		debugCustom = 0;
		debugDraw = false;
		debugJoin = false;
		debugLimb = false;
#endif
#if OP_DEBUG_VALIDATE
		debugScheduledForErasure = false;
#endif
	}
public:
	OpEdge(OpSegment*  OP_LINE_FILE_ARGS());  // segment make edge; used by curve curve
	OpEdge(OpSegment* , const OpPtT& start, const OpPtT& end  OP_LINE_FILE_ARGS());  // cc init clip
	OpEdge(OpIntersection* , OpIntersection*  OP_LINE_FILE_ARGS());  // sect make edges
	OpEdge(OpContext* , const OpPtT& start, const OpPtT& end  OP_LINE_FILE_ARGS());  // make filler 
	OpEdge(const OpEdge* e, const OpPtT& newPtT, NewEdge isLeftRight  OP_LINE_FILE_ARGS());
	OpEdge(const OpEdge* e, const OpPtT& start, const OpPtT& end  OP_LINE_FILE_ARGS());
	OpEdge(OpSegment* , float t1, float t2  OP_LINE_FILE_ARGS());

	CalcFail addIfUR(Axis xis, float t, OpWinding* ) const;
	void addPal(const EdgePal& );
	CalcFail addSub(OpContour* winderOwner, Axis axis, float t, OpWinding* ) const;
	OpEdge* advanceToEnd(EdgeMatch );
	void apply();
	const OpRect& bounds() { return ptBounds; }
	void calcCenterT();
	void ccInit(bool overlaps);
	void clearActiveAndPals(OP_LINE_FILE_NP_ARGS());
	void clearLastEdge(InOutput );
	void clearNextEdge();
	void clearPriorEdge();
	void complete(OpPoint start, OpPoint end);
	bool containsLink(const OpEdge* edge) const;
	OpContext* context() const;
	OpPtT end() const { return OpPtT(endPt(), endT); }
	OpPoint endPt() const { return curve.lastPt(); }
	OpPtT flipPtT(EdgeMatch match) const { 
		return match == which() ? end() : start(); }
	bool hasLinkTo(EdgeMatch match) const { 
		return EdgeMatch::start == match ? priorEdge : nextEdge; }
	bool isActive() const { 
		return active_impl; }
	bool isLine() {
		return curve.isLine(); }
	bool isPal(const OpEdge* opp) const {
		return pals.end() != std::find_if(pals.begin(), pals.end(), 
				[opp](const auto& test) { return opp == test.edge; }); }
	bool isSimple() const {
		return !disabled && !isUnsectable() && Unsortable::none == isUnsortable; }
	bool isUnsectable() const { 
		return pals.size(); }
	void linkToEdge(FoundEdge& , EdgeMatch );
	MatchReverse matchEnds(const LinePts& linePts) const {
		return curve.matchEnds(linePts); }
	void markPals();
	OpEdge* nextOut();
	NormalDirection normalDirection(Axis axis, float edgeInsideT) const {  // t value is not segment t
		return curve.normalDirection(axis, edgeInsideT); }
	void output(bool closed);  // provided by the graphics implementation
	void outputLinkedList(const OpEdge* firstEdge, bool first);
	OpPtT ptT(EdgeMatch match) const { 
		return EdgeMatch::start == match ? start() : end(); }
	void setActive(bool state);  // setter exists so debug breakpoints can be set
	void setDisabled(OP_LINE_FILE_NP_ARGS());
	void setDisabledZero(OP_LINE_FILE_NP_ARGS());
	OpEdge* setLastEdge();
	void setLastEdge(OpEdge* first, OpEdge* last, InOutput );
	bool setLastLink(EdgeMatch );  // returns true if link order was changed
	OpPointBounds setLinkBounds();
	bool setLinkDirection(EdgeMatch , std::vector<OpEdge*>* linkupsErasures, InOutput );
	void setNextEdge(OpEdge*);  // setter exists so debug breakpoints can be set
	void setPointBounds();
	void setPriorEdge(OpEdge* );  // setter exists so debug breakpoints can be set
	void setSum(const OpWinding&  OP_LINE_FILE_ARGS());  // called by macro SET_SUM
	void setUnsortable(Unsortable );  // setter exists so debug breakpoints can be set
	const OpCurve& setVertical(const LinePts& , MatchEnds);
	void setWhich(EdgeMatch );  // setter exists so debug breakpoints can be set
	OpPtT start() const { return OpPtT(startPt(), startT); }
	OpPoint startPt() const { return curve.firstPt(); }
	void subDivide(OpPoint start, OpPoint end);
	CalcFail subIfDL(OpContour* winderOwner, Axis axis, float t, OpWinding* ) const;
	void unlink();  // restore edge to unlinked state (for reusing unsortable or unsectable edges)
	EdgeMatch which() const {
		return whichEnd_impl; }
	OpPtT whichPtT(EdgeMatch match = EdgeMatch::start) const { 
		return match == (EdgeMatch::none == whichEnd_impl ? EdgeMatch::start : whichEnd_impl)
				? start() : end(); }
	bool debugFail() const;
	bool debugSuccess() const;
#if OP_DEBUG_DUMP
	void debugCompare(std::string ) const;
	std::string debugDumpCenter(DebugLevel , DebugBase ) const;
	std::string debugDumpLink(EdgeMatch , DebugLevel , DebugBase ) const;
	OpPtT debugFindT(Axis , float oppXY) const;
	#define OP_X(Thing) \
			std::string debugDump##Thing() const; \
			void dump##Thing() const;
	DEBUG_DUMP
	EDGE_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
	#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	struct DebugOpCurve debugSetCurve() const;
#endif
#if OP_DEBUG
	const OpEdge* debugAdvanceToEnd(EdgeMatch match) const;
	bool debugIsLoop() const {
		return debugIsLoop(EdgeMatch::start) || debugIsLoop(EdgeMatch::end); }
	const OpEdge* debugIsLoop(EdgeMatch , LeadingLoop = LeadingLoop::will) const;
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;  // make sure pointer to edge is valid
#endif
#if OP_DEBUG_IMAGE
	void addLink();
	void color(uint32_t );
	void drawLink();
#endif

	OpSegment* segment;
	SectRay ray;
	OpEdge* priorEdge;	// edges that link to form completed contour
	OpEdge* nextEdge;
	OpEdge* lastEdge;
	OpPtT center;  // curve location used to find winding contribution
	OpCurve curve;
	OpCurve vertical_impl;	// only access through set vertical function
	LinePts upright_impl;   //  "
	OpPointBounds ptBounds;
	OpPointBounds linkBounds;
	OpWinding winding;	// contribution: always starts as 1, 0 (or 0, 1)
	OpWinding sum;  // total incl. normal side of edge for operands (fill count in normal direction)
	OpWinding many;  // temporary used by unsectables to contain all pal windings combined
	std::vector<CoinPal> coinPals;  // track coincidences bracketing edge by ID
	std::vector<OpIntersection*> unSects;  // unsectable intersections bracketing edge
	std::vector<EdgePal> pals;	 // edge + pals share sect overlap; or ray can't order edge and pals
	OpHulls hulls;  // curve-curve intersections
	float startT;  // used to be ptT; needs sect to find unsectable
	float endT;
	float startDist;  // distance from start to opposite in curve-curve intersection
	float endDist;  // distance from end to opposite in curve-curve intersection
	OpPtT startOpp;	// opp pt at distance (uninitialized; may be unused)
	OpPtT endOpp;
	int id;
	EdgeMatch whichEnd_impl;  // if 'start', prior end equals start; if 'end' prior end matches end
	EdgeFail rayFail;   // how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero;  // zero: edge normal points to zero side (the exterior of the loop)
	Unsortable isUnsortable;  // unsectable is unsortable; others (e.g., very small) are also unsortable
	bool active_impl;  // used by ray casting to mark edges that may be to the left of casting edge
	bool inLinkups; // set for edges in linkups l vector
	bool linkHead;  // used to remove edge from contour linkups when edge is output 
	bool inOutput;	// set when edge is added to output path
	bool disabled;	// winding is zero, or apply disqualified edge from appearing in output
	bool isUnsplitable;  // too small to split in two during curve-curve intersection
	bool ccEnd;  // set if edge end is closest to already found curve-curve intersection
	bool ccLarge;  // set if curve/curve has large t match and this edge is last
	bool ccOverlaps;  // set if curve/curve edges have bounds that overlap
	bool ccSmall;  // set if curve/curve has small t match and this edge is first
	bool ccStart;  // set if edge start is closest to already found curve-curve intersection
	bool centerless;  // center could not be computed (likely edge is too small)
	bool startSeen;  // tracks start of edge in joiner linked list to add to tree only once
	bool endSeen;  // tracks end of edge in joiner linked list to add to tree only once
#if OP_DEBUG
	OpEdge* debugMatch;  // left side of nonzero ray from this edge
	OpEdge* debugZeroErr;  // debug match ray found edge that does not match -- diagnostic for now
	int debugOutPath;	// id to color output contours
	int debugParentID;
	int debugDepth;  // depth of curve-curve when edge was created
	mutable int debugRayMatch;	// id: edges in common output contour determined from ray
#endif
#if OP_DEBUG_DUMP
	OpContext* dumpContours;  // temporary edges don't have segment ptrs when unflattened
#endif
#if OP_DEBUG_IMAGE
	uint32_t debugColor;
	uint32_t debugCustom;  // non-zero if color set by immediate mode debugging
	bool debugDraw;
	bool debugJoin;	 // true if included by joiner
	bool debugLimb;  // true if a part of tree
#endif
#if OP_DEBUG_MAKER
	OpDebugMaker debugSetDisabled;
	OpDebugMaker debugSetMaker;
	OpDebugMaker debugSetSum;
#endif
#if OP_DEBUG_VALIDATE
	bool debugScheduledForErasure;
#endif
};

// allocating storage separately allows filler edges to be immobile and have reliable pointers
// !!! fix this to resemble sect storage, which has array of structs instead of array of bytes
#if OP_DEBUG_DUMP
enum class DumpStorage {
	cc,
	filler,
};
#endif

struct OpEdgeStorage {
	OpEdgeStorage()
		: next(nullptr)
		, used(0) {
	}
	bool contains(OpIntersection* start, OpIntersection* end) const;
	bool contains(OpPoint start, OpPoint end) const;
	void reuse();
#if OP_DEBUG_DUMP
	int debugCount() const;
	std::string debugDump(std::string label, DebugLevel l, DebugBase b);
	OpEdge* debugFind(int id);
	OpEdge* debugIndex(int index);
	static void DumpSet(const char*& str, OpContext* , DumpStorage );
	DUMP_DECLARATIONS
#endif

	OpEdgeStorage* next;
	OpEdge storage[256];
	int used;
};

#endif
