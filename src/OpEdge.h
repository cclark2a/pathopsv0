// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpEdge_DEFINED
#define OpEdge_DEFINED

#include "OpCurve.h"
#include "OpWinding.h"

struct FoundEdge;
struct OpContext;
struct OpCurveCurve;
struct OpIntersection;
struct OpOutPath;
struct OpSegment;
struct OpWinder;
struct RayTargets;

enum class CurveRef;
enum class FoundPtT;

enum class NewEdge {
    none,
	isLeft,
	isRight,
};

inline NewEdge operator!(const NewEdge& a) {
	return (NewEdge) !static_cast<int>(a);
}

enum class IntersectResult {
	fail,
	no,
	yes,
	coincident,
	maybe,
};

#define EdgeMatch_Base \
    OP_ENUM_BASE(uninitialized, -1)

#define EdgeMatch_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(start), \
	OP_ENUM_MEMBER(end),


enum class EdgeMatch : int8_t {
	EdgeMatch_Base,
	EdgeMatch_Enums
};

inline EdgeMatch operator!(EdgeMatch m) {
	OP_ASSERT(EdgeMatch::start == m || EdgeMatch::end == m);
	return static_cast<EdgeMatch>(static_cast<int>(m) ^ 0x3);
}

#define EdgeFail_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(horizontal), \
	OP_ENUM_MEMBER(vertical),

enum class EdgeFail : uint8_t {
    EdgeFail_Enums
};

#define RayOrder_Enums \
	OP_ENUM_MEMBER(uninitialized), \
	OP_ENUM_MEMBER(ok), \
	OP_ENUM_MEMBER(unordered),  /* informative - unsectable could have been noted here */ \
	OP_ENUM_MEMBER(tooClose),   /* actionable - find another ray that is unambiguous  */

enum class RayOrder : uint8_t {
    RayOrder_Enums
};

struct EdgePal {
	EdgePal(OpEdge* e, int uID, bool reversed);
	OpPoint matchPt(EdgeMatch ) const;

#if OP_DEBUG_DUMP
	EdgePal();
#endif
	DUMP_DECLARATIONS

	OpEdge* edge;
	int unsectID;  // unsect id from sect in edge's segment, if any; or unique ID if missing
	int8_t reversed;
};

struct Distance {
	Distance(OpEdge* e, float c, float tIn);

#if OP_DEBUG_DUMP
	Distance();
#endif
	DUMP_DECLARATIONS

	OpEdge* edge;
	RayOrder rayOrder = RayOrder::uninitialized;  // note if computed sum can't be used because distance entries are unordered
	float cept;  // where normal intersects edge (e.g. for home, axis horz: center.x)
	float edgeInsideT;  // !!! t value from 0 to 1 within edge range (seems bizarre)
	int8_t dependent = false;  // set if edge contains dependencies (i.e., get sum winding from edge)
	int8_t reversed = false;
	int8_t over = false;  // set if edge is home, or edge cept is close to or greater than home cept
#if OP_DEBUG || OP_DEBUGGER
	int debugID;
#endif
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

// target bounds is intersection of contour bounds, parent bounds, and home/ray bounds
struct RayTarget {
	DUMP_DECLARATIONS

	OpContour* contour = nullptr;
	OpPointBounds bounds;
};

struct RayTargets {
	bool addContainer(Axis , OpContour* container, OpRect& bounds);
	void build(OpEdge* );
	bool match(OpContour* ) const;
	OpEdge* next(Axis , float homeCept);
	void reset(Axis );
	void set(Axis );
	DUMP_DECLARATIONS

    OpContext* context;  // to access threshold
	// built from edge and its contour and overlapping contours
	std::vector<RayTarget> t;
	OpRect chainBounds;
	// current set of edges from nth ray target
	std::vector<OpEdge*>* inXY = nullptr;  // either inX or inY from t[t index].contour
	size_t edgeIndex = SIZE_MAX;  // index into inX or inY
	size_t tIndex = SIZE_MAX;  // index into ray target
#if OP_DEBUG_SERIALIZE  // edges is set from contour + axis, so track them
    OpContour* debugEdgesContour = nullptr;
    Axis debugEdgesAxis = Axis::neither;
#endif
};

enum class AllowTooManyRetries {
	no,
	yes
};

// captures ray info from edge that intersects other edges, horizontally or vertically
struct SectRay {
//	bool add(OpWinder* , OpEdge* , float xy, float root, bool reversed);  // add to distances
//	bool addCoinContours(OpWinder* );
	bool addContainers(OpEdge* add);
	bool addDependentContours(OpWinder* );
	void addDistance(OpEdge* , float xy, float root, bool reversed  
			OP_DEBUG_PARAMS(OpEdge* debugParent));
	void addPals();
	bool checkAdd(OpEdge* toAdd);
	RayOrder checkClose() const;
	void checkOrder();
	bool contains(OpPoint );  // checks if point is in inside bounds
	bool cull();  // remove distances further from home than first dependent
	const Distance* end(DistEnd e) const {
		return DistEnd::front == e ? &distances.front() : &distances.back(); }
	const Distance* find(const OpEdge* ) const;  // returns edge in distances
	FindCept findCept(OpEdge* test, AllowTooManyRetries );
	FindCept findIntercept(OpWinder* , OpEdge* test);
//	bool incomplete() const;
	bool isOrdered(size_t index) const;  // false if dist edge and neighbors are reversed elsewhere
	void markDependents();
//	bool missingContour(OpWinder* , OpEdge* ) const;
//	bool missingContour(OpWinder* , OpSegment* ) const;
	const Distance* next(const Distance* dist, DistEnd e) const {
		return dist + (int) e; }
//	bool sectsAllPals(const OpEdge* ) const;  // returns if edge + all of its pals are in distances
//	void setInsideBounds(const OpEdge* );
	void sort();
	bool tryADifferentCenter();
	DUMP_DECLARATIONS
#if OP_DEBUG_SERIALIZE
	std::string debugDumpHeader(DebugLevel l, DebugBase b) const;
#endif

	RayTargets targets;
	std::vector<Distance> distances;
#if OP_DEBUG  // !!! no code to use this, yet
	std::vector<Distance> debugErased;  // distances saved in case axis conflict requires restoring
#endif
	OpRect insideBounds;  // intersection of curve bounds and alias bounds
	OpVector homeTangent;  // used to determine if unsectable edge is reversed
	OpEdge* home = nullptr;
	float normal = OpNaN;  // ray used to find windings on home edge (e.g., axis: h, center.y)
	float homeCept = OpNaN;  // intersection of normal on home edge (e.g., axis: h, center.x)
	float homeT = OpNaN;  // value from 0 to 1 within edge range (akin to edgeInsideT)
//	float interceptLimit = OpNaN;
	float mid = .5;
	float midEnd = .5;
	Axis axis = Axis::neither;
	bool sorted = false;
};

#define SectType_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(endHull),  /* intersection is close to or equal to curve end point */ \
	OP_ENUM_MEMBER(controlHull),  /* sect is on hull between end & control, or pair of ctrl pts */ \
	OP_ENUM_MEMBER(midHull),  /* hull intersects, but not near end point */ \
	OP_ENUM_MEMBER(snipLo),   /* snip at t lower than intersection */ \
	OP_ENUM_MEMBER(snipHi)    /* snip at t higher than intersection */

enum class SectType {
    SectType_Enums
};

// distance from one edge to another, used by curve-curve intersection
struct EdgeDist {
	EdgeDist()
		OP_DEBUG_CODE(: dist(OpDebugNaN))  // uninitialized, unset
	{
	}

    EdgeDist(OpPtT o, float d)
        : opp(o)
        , dist(d) {
    }

    EdgeDist(SetToNaN)
        : opp(SetToNaN::dummy)
        , dist(OpNaN) {
    }

    bool isSet() const {
        return !OpMath::IsNaN(opp.t); }

	DUMP_DECLARATIONS
#if OP_DEBUG || OP_DEBUGGER
    bool debugIsSet() const {
        return !OpMath::IsDebugNaN(opp.t) && isSet(); }
#endif

    OpPtT opp;
    float dist;
};

// intersections of opposite curve's hull and this edge's segment
struct HullSect {
	HullSect(const OpEdge* o, const OpPtT& ptT, SectType st)
		: opp(o)
		, sect(ptT)
        , oppDist(SetToNaN::dummy)
		, type(st) {
	}

#if OP_DEBUG_DUMP
	HullSect() {}
#endif
	DUMP_DECLARATIONS
	const OpEdge* opp;
	OpPtT sect;			// point and t of intersection with hull on this edge
	EdgeDist oppDist;	// if sect came from edge end: the point closest on the opposite curve
	SectType type;		// separate from match: intersection origin (or near origin)
};

struct OpHulls {
	bool add(const OpPtT& ptT, OpVector threshold, const EdgeDist& dist, SectType st, 
			const OpEdge* o = nullptr);
	void clear() { h.clear(); }
	bool nudgeDeleted(const OpEdge& edge, OpCurveCurve& cc, CurveRef which);
	void sort(bool useSmall);
	DUMP_DECLARATIONS
	OP_DEBUG_CODE(bool debugSectCandidates(int index, const OpEdge& edge) const);
	OP_DEBUG_VALIDATE_CODE(void debugValidate() const;)

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
//	recursed,
	retry,
	fail,
};

enum class Unsectable {
	none,
	single,
	multiple,
};

// !!! removed 	OP_ENUM_MEMBER(alternateEnd),
#define Unsortable_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(addCalcFail), \
	OP_ENUM_MEMBER(addCalcFail2), \
	OP_ENUM_MEMBER(betweenCoins), \
	OP_ENUM_MEMBER(filler), \
	OP_ENUM_MEMBER(homeUnsectable), \
	OP_ENUM_MEMBER(noMidT), \
	OP_ENUM_MEMBER(noNormal), \
	OP_ENUM_MEMBER(palsEnd), \
	OP_ENUM_MEMBER(rayTooShallow), \
	OP_ENUM_MEMBER(tooManyTries), \
	OP_ENUM_MEMBER(underflow), \

enum class Unsortable : uint8_t {
	Unsortable_Enums
};

enum class InOutput {
	no,
	yes
};

#if OP_DEBUG || OP_DEBUGGER
enum class LeadingLoop {
	in,
	will,
};
#endif

typedef int WindingCondition;

struct CoinPal {
	friend bool operator==(CoinPal a, CoinPal b) {
		return a.coinID == b.coinID;
	}

	OpSegment* opp;
	int coinID;
};

struct EdgeOutput {
    EdgeOutput(OpContext* context, OpEdge* edge, bool isLoop);
};

struct OpEdge {
		friend struct OpEdgeStorage;
#if !OP_DEBUG_DUMP
private:
#endif
	OpEdge()	// note : not all release values are zero (which end)
		: segment(nullptr)
		, priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
		, upright_impl( { SetToNaN::dummy, SetToNaN::dummy } )
		, winding(WindingUninitialized::dummy)
		, sum(WindingUninitialized::dummy)
        , startDist(SetToNaN::dummy)
        , endDist(SetToNaN::dummy)
		, startT(OpNaN)
		, endT(OpNaN)
		, ccUnsectID(0)
		, whichEnd_impl(EdgeMatch::none)
		, rayFail(EdgeFail::none)
		, windZero(WindZero::unset)
		, unsortable(Unsortable::none)
		, active_impl(false)
//		, alternateEnd(false)
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
		, unsectableStart(false)
		, unsectableEnd(false)
		, unsummable(false)
	{
#if OP_DEBUG || OP_DEBUGGER // a few debug values are also nonzero
		id = -2;
		debugMatch = nullptr;
		debugZeroErr = nullptr;
		debugParentID = 0;
		debugDepth = 0;
        debugCC = 0;
		debugRayMatch = 0;
		debugUnordered = false;
		debugSumSet = false;
#endif
#if OP_DEBUG_DUMP
		dumpContext = nullptr;
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
	OpEdge(OpContext* , OpPoint start, OpPoint end  OP_LINE_FILE_ARGS());  // make filler 
	OpEdge(const OpEdge* e, const OpPtT& newPtT, NewEdge isLeftRight  OP_LINE_FILE_ARGS());
	OpEdge(const OpEdge* e, const OpPtT& start, const OpPtT& end  OP_LINE_FILE_ARGS());

	CalcFail addIfUR(Axis xis, float t, OpWinding* ) const;
	void addPal(OpEdge* , int uid, bool reversed);
	void addPal(const Distance* d) {
		addPal(d->edge, 0, d->reversed); }
	CalcFail addSub(Axis axis, float t, OpWinding* ) const;
	OpEdge* advanceToEnd(EdgeMatch );
	WindingCondition apply();
	OpRect bounds() const {
		return curve.aliasBounds(); }
	void calcCenterT();
	void clearActiveAndPals(OP_LINE_FILE_NP_ARGS());
	void clearLast(/* InOutput */);
	void clearLastEdge();
	void clearNextEdge();
	void clearPriorEdge();
	std::vector<OpPoint> collectMatch(EdgeMatch m, float* t = nullptr) const;
	bool compareMatch(EdgeMatch m, OpEdge* opp, EdgeMatch oppM) const;
	void complete(OpPtT startPtT, OpPtT endPtT);
	void complete(OpPoint startPt, OpPoint endPt);
	bool containsLink(const OpEdge* edge) const;
	OpContext* context() const;
	OpPtT endPtT() const { return OpPtT(endPt(), endT); }
	OpPoint endPt() const { return curve.lastPt(); }
//	OpPtT flipPtT(EdgeMatch match) const { 
//		return match == which() ? endPtT() : startPtT(); }
	bool hasLinkTo(EdgeMatch match) const { 
		return EdgeMatch::start == match ? priorEdge : nextEdge; }
	bool hasPals() const { 
		return !pals.empty(); }
	bool isActive() const { 
		return active_impl; }
	bool isKept() const;  // cumulative affect of unsortable edges' winding keeps this edge
	bool isLine() {
		return curve.isLine(); }
	bool isPal(const OpEdge* opp) const {
		return pals.end() != std::find_if(pals.begin(), pals.end(), 
				[opp](const auto& test) { return opp == test.edge; }); }
	bool isSimple() const {
		return !disabled && isSortable(); }
	bool isSortable() const {
		return Unsortable::none == unsortable; }
	bool isSummable() const {
		return !unsummable; }
	void linkToEdge(FoundEdge& , EdgeMatch );
	MatchReverse matchEnds(const LinePts& linePts) const {
		return curve.matchEnds(linePts); }
	float margin() const;
	void markPals();
	void markUnsortable(Unsortable );
	NormalDirection normalDirection(Axis axis, float edgeInsideT) const {  // t value is not segment t
		return curve.normalDirection(axis, edgeInsideT); }
	bool output(bool closed);  // provided by the graphics implementation
	void outputLink(OpEdge* , bool closeLoop);
	bool outputLinkedList();
	OpPtT ptT(EdgeMatch match) const { 
		return EdgeMatch::start == match ? startPtT() : endPtT(); }
	void setActive(bool state);  // setter exists so debug breakpoints can be set
	void setDisabled(OP_LINE_FILE_NP_ARGS());
	OpEdge* updateLastEdge();
	void setLast(OpEdge* first, OpEdge* last, InOutput );
	void setLastEdge(OpEdge* );  // actual setter
	bool setLastLink(EdgeMatch );  // returns true if link order was changed
	OpPointBounds setLinkBounds();
	bool setLinkDirection(EdgeMatch , std::vector<OpEdge*>* linkupsErasures, InOutput );
	void setNextEdge(OpEdge* );  // setter exists so debug breakpoints can be set
//	void setPointBounds();
	void setPriorEdge(OpEdge* );  // setter exists so debug breakpoints can be set
//    void setSplitLast(OpEdge* first, OpEdge* last, OpEdge* clearedLink);
	void setSum(const OpWinding&  OP_LINE_FILE_ARGS());  // called by macro SET_SUM
	void setUnsortable(Unsortable );  // setter exists so debug breakpoints can be set
	const OpCurve& setVertical(const LinePts& , MatchEnds);
	void setUnsetWhich() {
		if (EdgeMatch::none == whichEnd_impl) setWhich(EdgeMatch::start); }
	void setWhich(EdgeMatch );  // setter exists so debug breakpoints can be set
	OpPtT startPtT() const { return OpPtT(startPt(), startT); }
	OpPoint startPt() const { return curve.firstPt(); }
	void subDivide(OpPoint start, OpPoint end);
	CalcFail subIfDL(Axis axis, float t, OpWinding* ) const;
	void unlink();  // restore edge to unlinked state (for reusing unsortable or unsectable edges)
	EdgeMatch which() const {
		return whichEnd_impl; }
	EdgeMatch which(EdgeMatch match) const {
		return EdgeMatch::start == match ? whichEnd_impl : !whichEnd_impl; }
	OpPoint whichCallerPt(EdgeMatch match = EdgeMatch::start) const { 
		return curve.callerPt(which(match)); }
	OpPoint whichCurvePt(EdgeMatch match = EdgeMatch::start) const { 
		return curve.whichPt(which(match)); }
    OpPtT whichSect(EdgeMatch match = EdgeMatch::start) const;
	bool debugFail() const;
	bool debugSuccess() const;
#if OP_DEBUG_SERIALIZE
	std::string debugDumpCenter(DebugLevel , DebugBase ) const;
	#define OP_X(Thing) \
			std::string debugDump##Thing() const;
	DEBUG_DUMP
	EDGE_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
#endif
#if OP_DEBUG_DUMP
	void debugCompare(std::string ) const;
	OpPtT debugFindT(Axis , float oppXY) const;
	#define OP_X(Thing) \
			void dump##Thing() const;
	DEBUG_DUMP
	EDGE_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
#endif
#include "OpDebugDeclarations.h"
#if OP_DEBUG || OP_DEBUGGER
	const OpEdge* debugAdvanceToEnd(EdgeMatch match) const;
	bool debugIsLoop() const {
		return debugIsLoop(EdgeMatch::start) || debugIsLoop(EdgeMatch::end); }
	const OpEdge* debugIsLoop(EdgeMatch , LeadingLoop = LeadingLoop::will) const;
#endif
#if OP_DEBUG_VALIDATE
	OpEdge(const OpEdge& ) = default;
	~OpEdge();
	void debugValidate() const;  // make sure pointer to edge is valid
#endif

	OpSegment* segment;
	SectRay ray;  // captures ray info that intersects other edges, horizontally or vertically
	OpEdge* priorEdge;	// edges that link to form completed contour
	OpEdge* nextEdge;
	OpEdge* lastEdge;
	OpPtT center;  // curve location used to find winding contribution
	OpCurve curve;
	OpCurve vertical_impl;	// only access through set vertical function
	LinePts upright_impl;   //  "
	OpPointBounds linkBounds;
	OpWinding winding;	// contribution: always starts as 1, 0 (or 0, 1)
	OpWinding sum;  // total incl. normal side of edge for operands (fill count in normal direction)
	std::vector<CoinPal> coinPals;  // track coincidences bracketing edge by ID
	std::vector<OpIntersection*> unSects;  // unsectable sects bracketing edge (to mark as pals)
	std::vector<EdgePal> pals;	 // edge + pals share sect overlap; or ray can't order edge and pals
	OpHulls hulls;  // curve-curve intersections
    EdgeDist startDist;  // distance from start to opposite in curve-curve intersection
    EdgeDist endDist;  // distance from end to opposite in curve-curve intersection
	float startT;  // used to be ptT; needs sect to find unsectable
	float endT;
	int id;
	int ccUnsectID;  // filler made to connect pair of unsectables
	EdgeMatch whichEnd_impl;  // if 'start', prior end equals start; if 'end' prior end matches end
	EdgeFail rayFail;   // how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero;  // zero: edge normal points to zero side (the exterior of the loop)
	Unsortable unsortable;  // unsectable is unsortable; others (e.g., very small) are also unsortable
	bool active_impl;  // used by ray casting to mark edges that may be to the left of casting edge
//	bool alternateEnd;  // set if line length is proportionately similar to end to sect end dist
	bool inLinkups; // set for edges in linkups l vector
	bool linkHead;  // used to remove edge from contour linkups when edge is output 
	bool inOutput;	// set when edge is added to output path
	bool disabled;	// winding is zero, or apply disqualified edge from appearing in output
	bool smallTRange;  // t range is below some caller defined threshold
	bool isUnsplitable;  // too small to split in two during curve-curve intersection
	bool ccEnd;  // set if edge end is closest to already found curve-curve intersection
	bool ccLarge;  // set if curve/curve has large t match and this edge is last
	bool ccOverlaps;  // set if curve/curve edges have bounds that overlap
	bool ccSmall;  // set if curve/curve has small t match and this edge is first
	bool ccStart;  // set if edge start is closest to already found curve-curve intersection
	bool centerless;  // center could not be computed (likely edge is too small)
	bool startSeen;  // tracks start of edge in joiner linked list to add to tree only once
	bool endSeen;  // tracks end of edge in joiner linked list to add to tree only once
	bool unsectableStart;  // set if start intersection has an unsectable id
	bool unsectableEnd;  // set if end intersection has an unsectable id
	bool unsummable;  // set if ray through adjacent edge is too close
#if CHECK_SNIP
    bool snipped;  // set if intersections that made it were snipped
#endif
#if OP_DEBUG || OP_DEBUGGER
	OpEdge* debugMatch;  // left side of nonzero ray from this edge
	OpEdge* debugZeroErr;  // debug match ray found edge that does not match -- diagnostic for now
	int debugParentID;
	int debugDepth;  // depth of curve-curve when edge was created
	int debugCC;   // depth when edge is in edgeCurves or oppCurves
	mutable int debugRayMatch;	// id: edges in common output contour determined from ray
	bool debugUnordered;  // set if check order detected some rays are out of order
	bool debugSumSet;  // for 'set winding by distance' to detect infinite recursion
#endif
#if OP_DEBUG_DUMP
	OpContext* dumpContext;  // temporary edges don't have segment ptrs when unflattened
	bool debugLimb;  // true if a part of tree
#endif
OP_LINE_FILE_DECLARE(debugSetDisabled)
OP_LINE_FILE_DECLARE(debugSetMaker)
OP_LINE_FILE_DECLARE(debugSetSum)
#if OP_DEBUG_VALIDATE
	int debugPriorID = 0;
	bool debugScheduledForErasure;
#endif
};

// allocating storage separately allows filler edges to be immobile and have reliable pointers
// !!! fix this to resemble sect storage, which has array of structs instead of array of bytes
#if OP_DEBUG_DUMP
enum class DumpStorage {
	cc,
	filler,
#if OP_TEST_RASTER
    rasterFiller,
#endif
};
#endif

struct OpEdgeStorage {
	OpEdgeStorage()
		: next(nullptr)
		, used(0) {
	}
	int edgeCount() const;
	OpEdge* edgeIndex(int index) const;
	void reuse();
#if OP_DEBUG_DUMP
	static void DumpSet(const char*& str, OpContext* , DumpStorage );
#endif
#if OP_DEBUG_SERIALIZE
//	int debugCount() const;
	OpEdge* debugFind(int id);
//	OpEdge* debugIndex(int index) const;
#endif
	DUMP_DECLARATIONS

	OpEdgeStorage* next;
	OpEdge storage[256];
	int used;
#if OP_DEBUG_DUMP
    std::string debugName;
#endif
};

#endif
