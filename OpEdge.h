#ifndef OpEdge_DEFINED
#define OpEdge_DEFINED

#include "OpTightBounds.h"
#include "OpOperators.h"
#include <vector>

struct FoundEdge;
struct OpContours;
struct OpIntersection;
struct OpSegment;

enum class OpOperand {
	left,
	right
};

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
	maybe
};

enum EdgeRay {
	xRay,
	yRay,
};

enum class EdgeMatch : uint8_t {
	none,
	start,
	end,
	both	// used by flip
};

enum class FoundPtT;

inline EdgeMatch Opposite(EdgeMatch match) {
	OP_ASSERT(EdgeMatch::start == match || EdgeMatch::end == match);
	return EdgeMatch::start == match ? EdgeMatch::end : EdgeMatch::start;
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
	nextDistance,
	priorDistance,
	recalcCenter,
	vertical
};

enum class EdgeLink : uint8_t {
	unlinked,	// when assembled, does not connect to another edge
	single,		// end of assembled run of edges; connects to one other edge
	multiple	// connects to multiple edges
};

enum class WindingEdge {
	dummy
};

enum class WindingUninitialized {
	dummy
};

enum class WindingSum {
	dummy
};

enum class WindingTemp {	// used to accumulate winding sum before it is applied
	dummy
};

enum class ZeroReason : int8_t {
	uninitialized = -1,
	none,
	addIntersection,
	addTemp,
	applyOp,
	centerNaN,
	coincidence,
	collapsed,
//	failCenter,
//	failNext,
//	failPrior,
	findCoincidences,
	isCoinPoint,
	isPoint,
//	linkUp,
	looped,
//	loopyPair,
//	matchClosest,
//	matchLink,
	move,
	noFlip,
	noNormal,
//	rayNormal,
	recalcCenter,
	resolveCoin,
	setWind,
	subTemp,
	swapped,
	tangentXRay
};


enum class WindingType  {
	uninitialized = -1,
	temp,
	winding,
	sum
};

struct OpWinding {
private:
	// used by operator-
	OpWinding(int l, int r  OP_DEBUG_PARAMS(int s, ZeroReason z))
		: left_impl(l)
		, right_impl(r)
#if OP_DEBUG
		, debugLeft(OpMax)
		, debugRight(OpMax)
		, debugSetter(s)
		, debugType(WindingType::winding)
		, debugReason(z)
#endif
	{
	}

public:
	OpWinding(WindingTemp)  // used for winding accumulators before sum is set
		: left_impl(0)
		, right_impl(0)
#if OP_DEBUG
		, debugLeft(OpMax)
		, debugRight(OpMax)
		, debugSetter(0)
		, debugType(WindingType::temp)
		, debugReason(ZeroReason::none)
#endif
	{
	}


	OpWinding(WindingUninitialized)	 // used by edge and segment winding before they are set
#if OP_DEBUG
		: left_impl(OpMax)
		, right_impl(OpMax)
		, debugLeft(OpMax)
		, debugRight(OpMax)
		, debugSetter(0)
		, debugType(WindingType::uninitialized)
		, debugReason(ZeroReason::uninitialized)
#endif	
	{
	}

	OpWinding(OpOperand operand)	// used to set initial segment winding
		: left_impl(OpOperand::left == operand ? 1 : 0)
		, right_impl(OpOperand::right == operand ? 1 : 0)
#if OP_DEBUG
		, debugLeft(OpMax)
		, debugRight(OpMax)
		, debugSetter(0)
		, debugType(WindingType::winding)
		, debugReason(ZeroReason::none)
#endif	
	{
	}

	bool operator==(OpWinding w) {
		return left_impl == w.left_impl && right_impl == w.right_impl;
	}

	OpWinding operator-() const {
		OP_ASSERT(WindingType::winding == debugType);
		return { -left_impl, -right_impl  OP_DEBUG_PARAMS(0, ZeroReason::none) };
	}

	OpWinding& operator+=(const OpWinding& w) {
		OP_ASSERT(WindingType::temp == debugType
				|| WindingType::winding == debugType);
		left_impl += w.left_impl;
		right_impl += w.right_impl;
	#if OP_DEBUG
		if (WindingType::winding == debugType)
			debugReason = 0 == left_impl && 0 == right_impl ? ZeroReason::addTemp :
					ZeroReason::uninitialized;
	#endif
		return *this;
	}

	OpWinding& operator-=(const OpWinding& w) {
		OP_ASSERT(WindingType::temp == debugType);
		left_impl -= w.left_impl;
		right_impl -= w.right_impl;
	#if OP_DEBUG
		if (WindingType::winding == debugType)
			debugReason = 0 == left_impl && 0 == right_impl ? ZeroReason::subTemp :
					ZeroReason::uninitialized;
	#endif
		return *this;
	}

	bool isSet() const {
		return OpMax != left_impl;
	}

	int left() const {
		return left_impl;
	}

	void move(OpWinding opp, const OpContours* , bool backwards);

	int oppSide(OpOperand operand) const {
		return OpOperand::left == operand ? right_impl : left_impl;
	}

	int right() const {
		return right_impl;
	}

	void setSum(OpWinding winding, const OpSegment* segment);

	void setWind(int left, int right) {	// shouldn't be 0, 0 (call zero() for that)
		OP_ASSERT(WindingType::uninitialized == debugType);
		OP_ASSERT(left || right);
		left_impl = left;
		right_impl = right;
	#if OP_DEBUG
		debugType = WindingType::winding;
		debugReason = 0 == left_impl && 0 == right_impl ? ZeroReason::setWind :
				ZeroReason::uninitialized;
	#endif
	}

	int sum() const {
		return left_impl + right_impl;
	}

	bool visible() const {
		return left_impl || right_impl;
	}

	// debug parameter tracks the caller that zeroed the edge
	void zero(ZeroReason r) {
#if OP_DEBUG
		// if we already zeroed, don't zero again?
		if (ZeroReason::none == debugReason || ZeroReason::uninitialized == debugReason) {  
			debugLeft = left_impl;
			debugRight = right_impl;
			debugReason = r;
		}
#endif
		left_impl = 0;
		right_impl = 0;
	}

#if OP_DEBUG_DUMP
	std::string debugDump() const;
	void dump() const;
#endif

	int left_impl;	// indirection to make set debugging breakpoints easier 
	int right_impl;

#if OP_DEBUG
	int debugLeft;	// value prior to zero
	int debugRight;
	int debugSetter;
	WindingType debugType;
	ZeroReason debugReason;
#endif
};

// An edge that can contribute to the answer has a zero winding on one side
// For a pair of edges to connect, they have to have zero windings on the same side
// If they have zero windings that do not match, there should be a third (and fourth)
// edge at the same point that is a better match.
// A pair of edges that are nearly coincident may be mis-sorted so that the zero
// winding is wrong.
enum class WindZero : uint8_t {
	noFlip,
	normal,
	opp,
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	OP_ASSERT(WindZero::noFlip != left || WindZero::noFlip != right);
	OP_ASSERT((int) left + (int) right != 3);	// not normal and opp at same time
}

inline void WindZeroFlip(WindZero* windZero) {
    if (WindZero::noFlip == *windZero)
        return;
    *windZero = WindZero::normal == *windZero ? WindZero::opp : WindZero::normal;
}

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
	yes,
};

enum class EdgeSum : uint8_t {
	unset,
//	loop,			// prior edge chain loops back to start (no longer a thing?)
	point,			// edge totally collapsed (no longer a thing?)
	set,			// edge is ray ordered
	unsectable,		// too close to another edge to sort by ray
	unsortable,     // sum was not resolvable by ray in either axis (likely edge is very small) 
};

enum class LeadingLoop {
	in,
	will,
};

enum class ResolveWinding {
	resolved,
	loop,
	fail,
};

enum class Unsectable {
	none,
	single,
	multiple,
};

enum class WhichLoop {
	prior,
	next,
	undetermined,
};

#if OP_DEBUG
enum class EdgeMaker {
	empty,
	intersectEdge1,
	intersectEdge2,
	makeEdges,
	oppSect,
	resolveCoin1,
	resolveCoin2,
	segSect,
	split1,
	split2,
	// tests only
	addTest,
	opTest,
};

#define EDGE_MAKER(maker) EdgeMaker::maker, __LINE__, std::string(__FILE__)

#endif

struct OpEdge {
private:
	OpEdge()	// note : not all values are zero
		: priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
//		, priorSum_impl(nullptr)
//		, loopStart(nullptr)
		, winding(WindingUninitialized::dummy)
		, sum(WindingUninitialized::dummy)
		, many(WindingUninitialized::dummy)
//		, sumNormal(0)
//		, sumT(0)
//		, sumAxis(Axis::neither)	// not zero (-1)
		, nextLink(EdgeLink::unlinked)
		, priorLink(EdgeLink::unlinked)
		, whichEnd(EdgeMatch::none)
		, fail(EdgeFail::none)
		, windZero(WindZero::noFlip)
		, doSplit(EdgeSplit::no)
		, sumType(EdgeSum::unset)
		, curveSet(false)
//		, endAliased(false)
		, lineSet(false)
		, verticalSet(false)
		, isLine_impl(false)
		, active_impl(false)
//		, startAliased(false)
	{
#if OP_DEBUG
		debugStart = nullptr;
		debugEnd = nullptr;
		debugMaker = EdgeMaker::empty;
		debugMakerLine = 0;
		debugSetSumLine = 0;
		debugParentID = 0;
//		debugAliasStartID = 0;
//		debugAliasEndID = 0;
		debugUnOpp = false;
#endif
	}
public:
	OpEdge(const OpSegment* s, OpPtT t1, OpPtT t2, EdgeSum type
			OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file, const OpIntersection* i1, 
			const OpIntersection* i2))
		: OpEdge() {
		segment = s;
		start = t1;
		end = t2;
		sumType = type;
#if OP_DEBUG
		debugStart = i1;
		debugEnd = i2;
		debugMaker = maker;
		debugMakerLine = line;
		debugMakerFile = file;
		debugParentID = 0;
#endif
		complete();
	}

	OpEdge(const OpEdge* e, OpPtT newPtT, NewEdge isLeftRight  
			OP_DEBUG_PARAMS(EdgeMaker , int line, std::string file, const OpIntersection* , 
			const OpIntersection*));
	OpEdge(const OpEdge* e, const OpPtT& start, const OpPtT& end  
			OP_DEBUG_PARAMS(EdgeMaker , int line, std::string file, const OpIntersection* , 
			const OpIntersection*));

#if OP_DEBUG_IMAGE
	OpEdge(const OpEdge&) = default;
	OpEdge(OpEdge&&) = default;
	OpEdge& operator=(const OpEdge&) = default;
	OpEdge& operator=(OpEdge&&) = default;
	~OpEdge();	// reason: removes temporary edges from image list
#endif
	CalcFail addIfUR(Axis xis, float t, OpWinding* );
//	void addMatchingEnds(const OpEdge& ) const;
	CalcFail addSub(Axis axis, float t, OpWinding* );
	void apply();
	void calcCenterT();
//	CalcFail calcPrior(Axis axis, float sumT, OpWinding* prevRight);
	CalcFail calcWinding(Axis axis, float centerT);
	void clearActive();  // setter exists so debug breakpoints can be set
	void clearNextEdge();
	void clearPriorEdge();
	void complete();
//	bool containsChain(const OpEdge* edge, EdgeLoop) const;
	bool containsLink(const OpEdge* edge) const; // { return containsChain(edge, EdgeLoop::link); }
//	bool containsSum(const OpEdge* edge) const { return containsChain(edge, EdgeLoop::sum); }
	float findT(Axis , float oppXY) const;
	OpPtT flipPtT(EdgeMatch match) const { return match == whichEnd ? end : start; }
	void flipWhich() { whichEnd = (EdgeMatch)((int)whichEnd ^ (int)EdgeMatch::both); }
	ResolveWinding findWinding(Axis axis, float t  OP_DEBUG_PARAMS(int* debugWindingLimiter));
	bool hasLinkTo(EdgeMatch match) const { 
		return EdgeLink::single == (EdgeMatch::start == match ? nextLink : priorLink); }
	OpEdge* hasLoop(WhichLoop w, LeadingLoop l) {
		return const_cast<OpEdge*>((const_cast<const OpEdge*>(this))->isLoop(w, l)); }
	bool inLinkLoop(const OpEdge* );
//	bool inSumLoop(const OpEdge* );
	bool isActive() const { 
		return active_impl; }
	bool isClosed(OpEdge* test);
	const OpEdge* isLoop(WhichLoop , LeadingLoop ) const; 
	OpEdge* linkUp(EdgeMatch, OpEdge* firstEdge);
	void linkNextPrior(OpEdge* first, OpEdge* last);
	bool matchLink(std::vector<OpEdge*>& linkups, std::vector<OpEdge*>& unsectInX);
	const OpEdge* nextChain() const {
		return nextEdge; }
	NormalDirection normalDirection(Axis axis, float t);
	void output(OpOutPath path);	// provided by the graphics implmentation
	OpEdge* prepareForLinkup();
	const OpEdge* priorChain() const {
		return priorEdge; }
//	OpEdge* priorSum() { return priorSum_impl; }	// hide to avoid changing prior sum outside set
	OpPtT ptT(EdgeMatch match) const { 
		return EdgeMatch::start == match ? start : end; }
	void reverse();	// only call on temporary edges (e.g., used to make coincident intersections)
	void setActive();  // setter exists so debug breakpoints can be set
	void setFromPoints(const std::array<OpPoint, 4>& pts);
	OpPointBounds setLinkBounds();
	const OpCurve& setCurve();
	void setLinkDirection(EdgeMatch ); // reverse links if handed link end instead of link start
	bool setLinear();
	void setNextEdge(OpEdge*);  // setter exists so debug breakpoints can be set
	void setPointBounds();
	void setPoints(std::array<OpPoint, 4>& pts) const;
	void setPriorEdge(OpEdge* );  // setter exists so debug breakpoints can be set
//	void setPriorSum(OpEdge*);   // setter exists so debug breakpoints can be set
	void setSumImpl(OpWinding w) {	// use debug macro instead to record line/file
		sum.setSum(w, segment); }
	const OpCurve& setVertical();
	void skipPals(EdgeMatch match, std::vector<FoundEdge>& edges) const; 
	void subDivide();
	CalcFail subIfDL(Axis axis, float t, OpWinding* );
	bool validLoop() const;
//	OpEdge* visibleAdjacent(EdgeMatch );
//	OpPtT whichPtT() const { return EdgeMatch::start == whichEnd ? start : end;  }
	OpPtT whichPtT(EdgeMatch match = EdgeMatch::start) const { 
		return match == whichEnd ? start : end; }

	bool debugFail() const;
    bool debugSuccess() const;
#if OP_DEBUG_DUMP
	OpEdge(std::string );
	OpEdge(OpPtT data[2]);
	OpEdge(OpHexPtT data[2]);
	void debugCompare(std::string ) const;
	std::string debugDumpBrief() const;
	std::string debugDumpChain(WhichLoop , bool detail) const;
	void debugValidate() const;    // make sure pointer to edge is valid
	void dumpChain(bool detail = false) const;
	DEBUG_COMMON_DECLARATIONS();
	DUMP_COMMON_DECLARATIONS();
	DUMP_IMPL_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void addLink() const;
	void addSum() const;
	void drawChain() const;
	void drawLink() const;
//	void drawSum() const;
	void draw() const;
#endif

	const OpSegment* segment;
	OpEdge* priorEdge;	// edges that link to form completed contour
	OpEdge* nextEdge;
	OpEdge* lastEdge;
//	OpEdge* priorSum_impl;	// edge that set sum winding (access through debugging get/set)
//	OpEdge* loopStart;	// if sum loops, furthest edge away from end
	OpPoint ctrlPts[2];	// quad, conic, cubic
	float weight;
	// !!! Can start, end be shared with intersection?
	// what about id/ptr struct (union?) with intersect id and ptr
	// ptr is set up once intersects is sorted
	// should there be a different edge structure after intersections are known?
	OpPtT start;
	OpPtT center;  // curve location used to find winding contribution
	OpPtT end;
	OpCurve curve_impl;	// only access through set curve function
	OpCurve vertical_impl;	// only access through set vertical function
	OpPointBounds ptBounds;
	OpPointBounds linkBounds;
	OpWinding winding;	// contribution: always starts as 1, 0 (or 0, 1)
	OpWinding sum; // total incl. normal side of edge for operands (fill count in normal direction)
	OpWinding many;	 // if two or more edges are unsortable by ray, store their cumulative winding
	// !!! many pals is hopefully temporary while I figure out something better
	std::vector<OpEdge* > pals;	// pointers to unsectable adjacent edges 
//	float sumNormal; // normal of edge projecting ray (e.g., y value for horizontal ray)
//	float sumT; // t value of priorSum when hit by ray
	int id;
//	Axis sumAxis; // the ray axis when sum chain was computed (there may not be a prior)
	EdgeLink nextLink;
	EdgeLink priorLink;
	EdgeMatch whichEnd;	// if 'start', prior link end equals start; if 'end' prior end matches end
	EdgeFail fail;	// how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero; // normal means edge normal points to zero side; opposite, normal is non-zero
	EdgeSplit doSplit;
	EdgeSum sumType;
	bool curveSet;
	bool lineSet;
	bool verticalSet;
	bool isLine_impl;	// ptBounds 0=h/0=w catches horz/vert lines; if true, line is diagonal(?)
//	bool isPoint;
//	bool isSumLoop;
	bool active_impl;  // used by ray casting to mark edges that may be to the left of casting edge
//	bool endAliased;
//	bool startAliased;
//	bool unsectable;
//	bool unsortable;	
#if OP_DEBUG
	const OpIntersection* debugStart;
	const OpIntersection* debugEnd;
	EdgeMaker debugMaker;
	int debugMakerLine;
	std::string debugMakerFile;
//	OpPoint debugOriginalStart;
//	OpPoint debugOriginalEnd;
	int debugParentID;
	int debugSetSumLine;
	std::string debugSetSumFile;
//	int debugAliasStartID;
//	int debugAliasEndID;
	bool debugUnOpp;	// used to distinguish left unsectables from right unsectables
#endif
};

#if OP_DEBUG == 0
#define OP_EDGE_SET_SUM(edge, winding) edge->setSum(winding)
#else
#define OP_EDGE_SET_SUM(edge, winding) \
	do {	\
		OP_ASSERT(!edge->sum.isSet());  \
		edge->setSumImpl(winding); \
		edge->debugSetSumLine = __LINE__; \
		edge->debugSetSumFile = __FILE__; \
	} while (false)
#endif

#endif
