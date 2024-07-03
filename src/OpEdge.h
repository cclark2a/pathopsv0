// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpEdge_DEFINED
#define OpEdge_DEFINED

#include "OpTightBounds.h"
#include "OpOperators.h"
#include "OpWinding.h"
#include <vector>

struct FoundEdge;
struct OpContours;
struct OpCurveCurve;
struct OpIntersection;
struct OpOutPath;
struct OpSegment;

enum class CurveRef;

#if !OP_TEST_NEW_INTERFACE
enum class OpOperand {
	left,
	right
};
#endif

enum class NewEdge {
	isLeft,
	isRight
};

inline NewEdge operator!(const NewEdge& a) {
	return (NewEdge) !static_cast<int>(a);
}

enum RotateVertical {
	rotateVertical
};

enum class IntersectResult {
	fail,
	no,
	yes,
	coincident,
	maybe
};

enum EdgeRay {
	xRay,
	yRay,
};

enum class EdgeMatch : int8_t {
	none = -1,
	start,
	end,
//	both	// used by flip
};

enum class FoundPtT;

inline EdgeMatch operator!(EdgeMatch m) {
	OP_ASSERT(EdgeMatch::start == m || EdgeMatch::end == m);
	return static_cast<EdgeMatch>(!static_cast<int>(m));
}

#if 0  // not needed for now
inline EdgeMatch operator| (const EdgeMatch& a, const EdgeMatch& b) {
	return (EdgeMatch) (static_cast<int>(a) | static_cast<int>(b));
}

inline EdgeMatch& operator|= (EdgeMatch& a, const EdgeMatch& b) {
	return a = a | b;
}
#endif

enum class EdgeFail : uint8_t {
	none,
	center,
	horizontal,
	vertical
};

enum class ZeroReason : uint8_t {
	uninitialized,
	addedPalToOutput,
	addIntersection,
	applyOp,
	centerNaN,
	collapsed,
	filler,
	findCoincidences,
	hvCoincidence1,
	hvCoincidence2,
	hvCoincidence3,
	hvCoincidence4,
	isPoint,
	noFlip,
	none,
	palWinding,
};

struct EdgeDistance {
	EdgeDistance(OpEdge* e, float c, float tIn, bool r);

#if OP_DEBUG_DUMP
	EdgeDistance() {}
	DUMP_DECLARATIONS
#endif

	OpEdge* edge;
	float cept;  // where normal intersects edge (e.g. for home, axis horz: center.x)
	float edgeInsideT;  // !!! t value from 0 to 1 within edge range (seems bizarre)
	bool reversed;
};

enum class FindCept {
	ok,
	okNew,	// intercept was found, and new distance was pushed
	retry,
	unsectable,
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
	{
	}
	void addPals(OpEdge* );
	bool checkOrder(const OpEdge* ) const;
	const EdgeDistance* end(DistEnd e) {
		return DistEnd::front == e ? &distances.front() : &distances.back(); }
	FindCept findIntercept(OpEdge* );
	EdgeDistance* find(OpEdge* );
	EdgeDistance* next(EdgeDistance* dist, DistEnd e) {
		return dist + (int) e; }
	void sort();
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<EdgeDistance> distances;
	OpVector homeTangent;  // used to determine if unsectable edge is reversed
	float normal;  // ray used to find windings on home edge (e.g., axis: h, center.y)
	float homeCept;  // intersection of normal on home edge (e.g., axis: h, center.x)
	float homeT;  // value from 0 to 1 within edge range (akin to edgeInsideT)
	Axis axis;
};

enum class SectType {
	none,
	endHull,  // intersection is close to or equal to curve end point
	center,   // break curve at geometric center
	controlHull,  // intersection is on hull between end and control, or pair of control points
	midHull,  // hull intersects, but not near end point
	snipLo,   // snip at t lower than intersection
	snipHi    // snip at t higher than intersection
};

// intersections of opposite curve's hull and this edge's segment
struct HullSect {
	HullSect(const OpPtT& ptT, SectType st, const OpEdge* o)
		: opp(o)
		, sect(ptT)
		, type(st) {
	}
#if OP_DEBUG_DUMP
	HullSect() {}
	DUMP_DECLARATIONS
#endif
	const OpEdge* opp;
	OpPtT sect;			// point and t of intersection with hull on this edge
	SectType type;
};

struct OpHulls {
	bool add(const OpPtT& ptT, SectType st, const OpEdge* o = nullptr);
	void clear() { h.clear(); }
	bool closeEnough(int index, const OpEdge& edge, const OpEdge& oEdge, OpPtT* oPtT, OpPtT* close);
	void nudgeDeleted(const OpEdge& edge, const OpCurveCurve& cc, CurveRef which);
	bool sectCandidates(int index, const OpEdge& edge);
	void sort(bool useSmall);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<HullSect> h;
};

enum class AllowPals {
	no,
	yes
};

enum class AllowClose {
	no,
	yes
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

#if 0
enum class EdgeLoop {
	link,
	sum,
};
#endif

enum class EdgeSplit : uint8_t {
	no,
	keep,  // used only by curve experiment
	defer,  // used only by curve experiment : set split like partner
	yes = 3, // allows |= keep to set no->keep, keep->keep, yes->yes
};

inline EdgeSplit& operator|=(EdgeSplit& a, EdgeSplit b) {
    return a = (EdgeSplit)((int) a | (int) b);
}

enum class LeadingLoop {
	in,
	will,
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

#if OP_DEBUG
enum class EdgeMaker {
	empty,
	filler,
	hull,
	makeEdges,
	oppSect,
	segSect,
	snip,
	split,
	splitLeft,
	splitRight,
	// tests only
	addTest,
	opTest,
};

#endif

constexpr float OP_CURVACIOUS_LIMIT = 1.f / 16;  // !!! tune to guess line/line split ratio

struct OpEdge {
#if !OP_DEBUG_DUMP
private:
#endif
	OpEdge()	// note : not all release values are zero (which end, wind zero, opp dist)
		: priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
		, upright_impl( { SetToNaN::dummy, SetToNaN::dummy } )
		, winding(WindingUninitialized::dummy)
		, sum(WindingUninitialized::dummy)
		, many(WindingUninitialized::dummy)
		, unsectableID(0)
		, whichEnd_impl(EdgeMatch::none)
		, rayFail(EdgeFail::none)
		, windZero(WindZero::unset)
		, doSplit(EdgeSplit::no)
		, closeSet(false)
//		, curvySet(false)
		, lineSet(false)
//		, verticalSet(false)
		, isClose_impl(false)
		, isLine_impl(false)
		, exactLine(false)
		, active_impl(false)
		, inLinkups(false)
		, inOutput(false)
		, disabled(false)
		, unsortable(false)
		, between(false)
		, ccEnd(false)
		, ccLarge(false)
		, ccOverlaps(false)
		, ccSmall(false)
		, ccStart(false)
		, centerless(false)
		, windPal(false)
		, startSeen(false)
		, endSeen(false)
	{
#if OP_DEBUG // a few debug values are also nonzero
        id = 0;
        segment = nullptr;
		debugStart = nullptr;
		debugEnd = nullptr;
		debugMatch = nullptr;
		debugZeroErr = nullptr;
		debugMaker = EdgeMaker::empty;
		debugZero = ZeroReason::uninitialized;
		debugOutPath = 0;
		debugParentID = 0;
		debugRayMatch = 0;
		debugFiller = false;
#endif
#if OP_DEBUG_DUMP
		dumpContours = nullptr;
#endif
#if OP_DEBUG_IMAGE
		debugColor = debugBlack;
		debugDraw = true;
		debugJoin = false;
#endif
	}
public:
	OpEdge(OpSegment* s, const OpPtT& t1, const OpPtT& t2
			OP_LINE_FILE_DEF(EdgeMaker maker, OpIntersection* i1, OpIntersection* i2))
		: OpEdge() {
		segment = s;
		start = t1;
		end = t2;
#if OP_DEBUG
		debugMaker = maker;
		debugSetMaker = { fileName, lineNo };
		debugStart = i1;
		debugEnd = i2;
#endif
		complete();
	}

	OpEdge(const OpEdge* e, const OpPtT& newPtT, NewEdge isLeftRight  OP_LINE_FILE_DEF(EdgeMaker ));
	OpEdge(const OpEdge* e, const OpPtT& start, const OpPtT& end  OP_LINE_FILE_DEF(EdgeMaker ));
	OpEdge(const OpEdge* e, float t1, float t2  OP_LINE_FILE_DEF(EdgeMaker ));

#if OP_DEBUG_IMAGE
	OpEdge(const OpEdge&) = default;
	OpEdge(OpEdge&&) = default;
	OpEdge& operator=(const OpEdge&) = default;
	OpEdge& operator=(OpEdge&&) = default;
	struct DebugOpCurve debugSetCurve() const;
#endif
	CalcFail addIfUR(Axis xis, float t, OpWinding* );
	void addPal(EdgeDistance& );
	CalcFail addSub(Axis axis, float t, OpWinding* );
	OpEdge* advanceToEnd(EdgeMatch );
	void apply();
	const OpRect& bounds() { return ptBounds; }
	void calcCenterT();
	void clearActiveAndPals(ZeroReason );
	void clearLastEdge();
	void clearLinkBounds() { OP_ASSERT(!linkBounds.isSet()); } // !!! see if this is needed
	void clearNextEdge();
	void clearPriorEdge();
	const OpRect& closeBounds();  // returns bounds with slop
	void complete();
	bool containsLink(const OpEdge* edge) const;
	OpContours* contours() const;
	size_t countUnsortable() const;
	bool ctrlPtNearlyEnd();
//	float curviness();
	OpIntersection* findSect(EdgeMatch );
	OpPtT findT(Axis , float oppXY) const;
	OpPtT flipPtT(EdgeMatch match) const { 
		return match == which() ? end : start; }
	bool hasLinkTo(EdgeMatch match) const { 
		return EdgeMatch::start == match ? priorEdge : nextEdge; }  // !!! was reversed!
	bool isActive() const { 
		return active_impl; }
	bool isClose();
	bool isLinear();
	bool isPal(const OpEdge* opp) const {
		return pals.end() != std::find_if(pals.begin(), pals.end(), 
				[opp](const auto& test) { return opp == test.edge; }); }
	void linkToEdge(FoundEdge& , EdgeMatch );
//	void linkNextPrior(OpEdge* first, OpEdge* last);
	bool linksTo(OpEdge* match) const;
	void markPals();
	void matchUnsectable(EdgeMatch , const std::vector<OpEdge*>& unsectInX, 
			std::vector<FoundEdge>& , AllowPals , AllowClose );
	OpEdge* nextOut();
	NormalDirection normalDirection(Axis axis, float t);
//	float oppDist() const;
	void output(OpOutPath& path, bool closed);  // provided by the graphics implmentation
	OpPtT ptT(EdgeMatch match) const { 
		return EdgeMatch::start == match ? start : end; }
	OpPtT ptTCloseTo(OpPtT oPtPair, const OpPtT& ptT) const;
	void reenable() {  // only used for coincidence
		disabled = false; OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized); }
	void setActive(bool state);  // setter exists so debug breakpoints can be set
	void setBetween();  // setter exists so debug breakpoints can be set
//	const OpCurve& setCurve();  // copies start, end to points 0, last
//	void setCurveCenter();  // adds center point after curve points
	void setDisabled(OP_DEBUG_CODE(ZeroReason reason));
	void setDisabledZero(OP_DEBUG_CODE(ZeroReason reason)) {
		winding.zero();
		setDisabled(OP_DEBUG_CODE(reason)); 
	}
//	void setFromPoints(const OpPoint pts[]);
	OpEdge* setLastEdge();
	bool setLastLink(EdgeMatch );  // returns true if link order was changed
	OpPointBounds setLinkBounds();
	bool setLinkDirection(EdgeMatch );  // reverse links if handed link end instead of link start
	void setNextEdge(OpEdge*);  // setter exists so debug breakpoints can be set
	void setPointBounds();
	void setPriorEdge(OpEdge* );  // setter exists so debug breakpoints can be set
#if OP_TEST_NEW_INTERFACE
	void setSum(const PathOpsV0Lib::Winding&  OP_LINE_FILE_DEF(int dummy));
#else
	void setSumImpl(OpWinding w) {	// use debug macro instead to record line/file
		sum.setSum(w, contours());
	}
#endif
	void setUnsortable();
	const OpCurve& setVertical(const LinePts& );
	void setWhich(EdgeMatch );  // setter exists so debug breakpoints can be set
	void skipPals(EdgeMatch match, std::vector<FoundEdge>& edges);
//	OpPtT splitPt(float oMidDist, OpPtT* result) const;
	void subDivide();
	CalcFail subIfDL(Axis axis, float t, OpWinding* );
	OpType type();
	void unlink();  // restore edge to unlinked state (for reusing unsortable or unsectable edges)
	EdgeMatch which() const {
		return whichEnd_impl; }
	OpPtT whichPtT(EdgeMatch match = EdgeMatch::start) const { 
		return match == which() ? start : end; }

	bool debugFail() const;
    bool debugSuccess() const;
#if OP_DEBUG_DUMP
	void debugCompare(std::string ) const;
	std::string debugDumpCenter(DebugLevel , DebugBase ) const;
	std::string debugDumpLink(EdgeMatch , DebugLevel , DebugBase ) const;
    #define OP_X(Thing) \
    std::string debugDump##Thing() const; \
    void dump##Thing() const;
    DEBUG_DUMP
    EDGE_DETAIL
    EDGE_OR_SEGMENT_DETAIL
    #undef OP_X
	#include "OpDebugDeclarations.h"
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
	// !!! Can start, end be shared with intersection?
	// what about id/ptr struct (union?) with intersect id and ptr
	// ptr is set up once intersects is sorted
	// should there be a different edge structure after intersections are known?
	OpPtT start;
	OpPtT center;  // curve location used to find winding contribution
	OpPtT end;
	OpCurve curve;
	OpCurve vertical_impl;	// only access through set vertical function
	LinePts upright_impl;   //  "
	OpPointBounds ptBounds;
	OpPointBounds linkBounds;
	OpWinding winding;	// contribution: always starts as 1, 0 (or 0, 1)
	OpWinding sum;  // total incl. normal side of edge for operands (fill count in normal direction)
	OpWinding many;  // temporary used by unsectables to contain all pal windings combined
	std::vector<EdgeDistance> pals;	 // list of unsectable adjacent edges !!! should be pointers?
	std::vector<OpEdge*> lessRay;  // edges found placed with smaller edge distance cept values
	std::vector<OpEdge*> moreRay;  // edges found placed with larger edge distance cept values
	OpHulls hulls;  // curve-curve intersections
//	float curvy;  // rough ratio of midpoint line point line to length of end point line
//	OpPtT oppEnd;  // pt and t for closest point on opposite curve from end point
	int id;
	int unsectableID;	// used to pair unsectable intersections and find edges between
	EdgeMatch whichEnd_impl;  // if 'start', prior end equals start; if 'end' prior end matches end
	EdgeFail rayFail;   // how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero;  // zero: edge normal points to zero side (the exterior of the loop)
	EdgeSplit doSplit;  // used by curve/curve intersection to track subdivision
	bool closeSet;
//	bool curvySet;
	bool lineSet;
//	bool verticalSet;
	bool isClose_impl;  // set if start is close to end
	bool isLine_impl;	// ptBounds 0=h/0=w catches horz/vert lines; if true, line is diagonal(?)
	bool exactLine;   // set if quad/conic/cubic is degenerate; clear if control points are linear
	bool active_impl;  // used by ray casting to mark edges that may be to the left of casting edge
	bool inLinkups; // like inOutput, to marks unsectable edges; all edges in linkups l vector
	bool inOutput;	// likely only used to find inactive unsectables that are not on output path
	bool disabled;	// winding is zero, or apply disqualified edge from appearing in output
	bool unsortable;
	bool between;  // between unsectables (also unsortable); !!! begs for an enum class, instead...
	bool ccEnd;  // set if edge end is closest to already found curve-curve intersection
	bool ccLarge;  // set if curve/curve has large t match and this edge is last
	bool ccOverlaps;  // set if curve/curve edges have bounds that overlap
	bool ccSmall;  // set if curve/curve has small t match and this edge is first
	bool ccStart;  // set if edge start is closest to already found curve-curve intersection
	bool centerless;  // center could not be computed (likely edge is too small)
	bool windPal;  // winding could not computed because of pal
	bool startSeen;  // experimental tree to track adding edges to output
	bool endSeen;  // experimental tree to track adding edges to output
#if OP_DEBUG
	OpIntersection* debugStart;
	OpIntersection* debugEnd;
	OpEdge* debugMatch;  // left side of nonzero ray from this edge
	OpEdge* debugZeroErr;  // debug match ray found edge that does not match -- diagnostic for now
	EdgeMaker debugMaker;
	ZeroReason debugZero;	// why edge was disabled
	OpDebugMaker debugSetMaker;
	OpDebugMaker debugSetSum;
	int debugOutPath;	// id to color output contours
	int debugParentID;
	mutable int debugRayMatch;	// id: edges in common output contour determined from ray
	bool debugFiller;  // edge created to span short gaps
#endif
#if OP_DEBUG_DUMP
	OpContours* dumpContours;  // temporary edges don't have segment ptrs when unflattened
#endif
#if OP_DEBUG_IMAGE
	uint32_t debugColor;
	bool debugDraw;
	bool debugJoin;	 // true if included by joiner
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
        OP_DEBUG_CODE(storage[0] = 0);
	}
	bool contains(OpIntersection* start, OpIntersection* end) const;
#if OP_DEBUG_DUMP
	size_t debugCount() const;
	std::string debugDump(std::string label, DebugLevel l, DebugBase b) const;
	OpEdge* debugFind(int id) const;
	OpEdge* debugIndex(int index) const;
	static void DumpSet(const char*& str, OpContours* , DumpStorage );
	DUMP_DECLARATIONS
#endif

	OpEdgeStorage* next;
	uint8_t storage[sizeof(OpEdge) * 256];	// !!! change this to array of edges
	int used;
};

// new interface
#if OP_TEST_NEW_INTERFACE
#define OP_EDGE_SET_SUM(edge, winding) edge->setSum(winding  OP_LINE_FILE_PARAMS(0))
#else
#if OP_DEBUG == 0
#define OP_EDGE_SET_SUM(edge, winding) edge->setSumImpl(winding)
#else
#define OP_EDGE_SET_SUM(edge, winding) \
	do {	\
		OP_ASSERT(!edge->sum.isSet());  \
		edge->setSumImpl(winding); \
		edge->debugSetSum = { __FILE__, __LINE__ }; \
	} while (false)
#endif
#endif

#endif
