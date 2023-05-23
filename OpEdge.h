#ifndef OpEdge_DEFINED
#define OpEdge_DEFINED

#include "OpTightBounds.h"
#include "OpOperators.h"
#include <vector>

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
	assert(EdgeMatch::start == match || EdgeMatch::end == match);
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
	nextDistance,
	priorDistance,
	recalcCenter,
	winding
};

enum class EdgeLink : uint8_t {
	unlinked,	// when assembled, does not connect to another edge
	single,		// end of assembled run of edges; connects to one other edge
	multiple	// connects to multiple edges
};

enum WindingEdge {
	windingEdge
};

enum WindingSum {
	windingSum
};

enum class ZeroReason : uint8_t {
	none,
	addIntersection,
	applyOp,
	centerNaN,
	coincidence,
	collapsed,
	failCenter,
	failNext,
	failPrior,
	findCoincidences,
	isCoinPoint,
	isPoint,
	linkUp,
	looped,
	loopyPair,
	matchClosest,
	matchLink,
	noFlip,
	noNormal,
	rayNormal,
	recalcCenter,
	resolveCoin,
	tangentXRay
};


enum class WindingType  {
	none,
	winding,
	sum
};

struct OpWinding {
private:
	OpWinding(int l, int r  OP_DEBUG_PARAMS(int s, ZeroReason z))
		: left(l)
		, right(r)
#if OP_DEBUG
		, debugSetter(s)
		, debugType(WindingType::winding)
		, debugReason(z)
#endif
	{
	}

public:
	OpWinding(WindingEdge) 
		: left(0)
		, right(0)
#if OP_DEBUG
		, debugSetter(0)
		, debugType(WindingType::winding)
		, debugReason(ZeroReason::none)
#endif
	{
	}

	OpWinding(WindingSum)
		: left(OpMax)
		, right(OpMax)
#if OP_DEBUG
		, debugSetter(0)
		, debugType(WindingType::sum)
		, debugReason(ZeroReason::none)
#endif	
	{
	}

	OpWinding(OpOperand operand)
		: left(OpOperand::left == operand ? 1 : 0)
		, right(OpOperand::right == operand ? 1 : 0)
#if OP_DEBUG
		, debugSetter(0)
		, debugType(WindingType::winding)
		, debugReason(ZeroReason::none)
#endif	
	{
	}

	bool operator==(OpWinding w) {
		return left == w.left && right == w.right;
	}

	OpWinding operator-() const {
		return { -left, -right  OP_DEBUG_PARAMS(0, ZeroReason::none) };
	}

	bool isSet() const {
		assert(WindingType::sum == debugType);
		return OpMax != left || OpMax != right;
	}

	void move(const OpWinding& opp, const OpContours* , bool backwards);

	int oppSide(OpOperand operand) const {
		return OpOperand::left == operand ? right : left;
	}

	int sum() const {
		return left + right;
	}

	bool visible() const {
		return left || right;
	}

	// debug parameter tracks the caller that zeroed the edge
	void zero(ZeroReason r) { 
		left = 0;
		right = 0;
#if OP_DEBUG
		if (ZeroReason::none == debugReason)	// if we already failed, don't fail again?
			debugReason = r;
#endif
	}

#if OP_DEBUG_DUMP
	std::string debugDump() const;
	void dump() const;
#endif

	int left;
	int right;

#if OP_DEBUG
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
	opp
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	assert(WindZero::noFlip != left || WindZero::noFlip != right);
	assert((int) left + (int) right != 3);	// not normal and opp at same time
}

enum class EdgeLoop {
	link,
	sum
};

enum class EdgeSplit {
	no,
	yes,
	unsplittable
};

enum class LeadingLoop {
	in,
	will
};

enum class ResolveWinding {
	resolved,
	loop
};

enum class WhichLoop {
	prior,
	next,
	undetermined
};

#if OP_DEBUG
enum class EdgeMaker {
	addTest,
	intersectEdge1,
	intersectEdge2,
	makeEdges,
	opTest,
	resolveCoin1,
	resolveCoin2,
	resolveCoin3,
	split1,
	split2,
	split3,
	split4
};
#endif

struct OpEdge {
private:
	OpEdge()	// note : all values are zero
		: priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
		, priorSum(nullptr)
		, loopStart(nullptr)
		, winding(windingEdge)
		, sum(windingSum)
		, priorNormal(0)
		, priorT(0)
		, priorAxis(Axis::vertical)
		, nextLink(EdgeLink::unlinked)
		, priorLink(EdgeLink::unlinked)
		, whichEnd(EdgeMatch::none)
		, fail(EdgeFail::none)
		, windZero(WindZero::noFlip)
		, doSplit(EdgeSplit::no)
		, curveSet(false)
		, endAliased(false)
		, lineSet(false)
		, verticalSet(false)
		, isLine_impl(false)
		, isPoint(false)
		, isSumLoop(false)
		, active_impl(false)
		, startAliased(false)
		, unsortable(false) {
		OP_DEBUG_CODE(debugAliasStartID = 0);
		OP_DEBUG_CODE(debugAliasEndID = 0);
	}
public:
	OpEdge(const OpSegment* s, OpPtT t1, OpPtT t2  OP_DEBUG_PARAMS(EdgeMaker maker))
		: OpEdge() {
		segment = s;
		start = t1;
		end = t2;
#if OP_DEBUG
		debugMaker = maker;
		debugParentID = 0;
#endif
		complete();
	}

	OpEdge(const OpEdge* e, OpPtT newPtT, NewEdge isLeftRight  OP_DEBUG_PARAMS(EdgeMaker maker));
	OpEdge(const OpEdge* e, const OpPtT& start, const OpPtT& end  OP_DEBUG_PARAMS(EdgeMaker maker));

#if OP_DEBUG_IMAGE
	OpEdge(const OpEdge&) = default;
	OpEdge(OpEdge&&) = default;
	OpEdge& operator=(const OpEdge&) = default;
	OpEdge& operator=(OpEdge&&) = default;
	~OpEdge();	// reason: removes temporary edges from image list
#endif
	void addMatchingEnds(const OpEdge& ) const;
	void apply();
	void calcCenterT();
	void calcWinding(Axis axis);
	void clearActive();  // setter exists so debug breakpoints can be set
	void clearNextEdge();
	void clearPriorEdge();
	void complete();
	bool containsLink(const OpEdge* edge) const;
	bool containsSum(const OpEdge*) const;
	float findPtT(OpPoint pt) const;
	FoundPtT findPtT(OpPoint pt, float* result) const;
	OpPtT flipPtT(EdgeMatch match) const { return match == whichEnd ? end : start; }
	void flipWhich() { whichEnd = (EdgeMatch)((int)whichEnd ^ (int)EdgeMatch::both); }
	ResolveWinding findWinding(Axis axis  OP_DEBUG_PARAMS(int* debugWindingLimiter));
	bool hasLinkTo(EdgeMatch match) const { 
		return EdgeLink::single == (EdgeMatch::start == match ? nextLink : priorLink); }
	OpEdge* hasLoop(WhichLoop w, EdgeLoop e, LeadingLoop l) {
		return const_cast<OpEdge*>((const_cast<const OpEdge*>(this))->isLoop(w, e, l)); }
	bool inLinkLoop(const OpEdge* );
	bool inSumLoop(const OpEdge* );
	bool isActive() const { 
		return active_impl; }
	bool isClosed(OpEdge* test);
	const OpEdge* isLoop(WhichLoop , EdgeLoop , LeadingLoop ) const; 
	OpEdge* linkUp(EdgeMatch, OpEdge* firstEdge);
	void linkNextPrior(OpEdge* first, OpEdge* last);
	bool matchLink(std::vector<OpEdge*>& linkups );
	const OpEdge* nextChain(EdgeLoop edgeLoop) const {
		assert(EdgeLoop::link == edgeLoop); return nextEdge; }
	OpEdge* prepareForLinkup();
	const OpEdge* priorChain(EdgeLoop edgeLoop) const {
		return EdgeLoop::link == edgeLoop ? priorEdge : priorSum; }
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
	void setPriorSum(OpEdge*);   // setter exists so debug breakpoints can be set
	const OpCurve& setVertical();
	void setWinding(OpVector ray);
	void subDivide();
	bool validLoop() const;
	OpEdge* visibleAdjacent(EdgeMatch );
//	OpPtT whichPtT() const { return EdgeMatch::start == whichEnd ? start : end;  }
	OpPtT whichPtT(EdgeMatch match = EdgeMatch::start) const { 
		return match == whichEnd ? start : end; }

#if OP_DEBUG_DUMP
	OpEdge(std::string );
	OpEdge(OpPtT data[2]);
	OpEdge(OpHexPtT data[2]);
	void debugCompare(std::string ) const;
	std::string debugDumpBrief() const;
	std::string debugDumpChain(WhichLoop , EdgeLoop , bool detail) const;
	void debugValidate() const;    // make sure pointer to edge is valid
	void dumpChain(EdgeLoop , bool detail = false) const;
	DEBUG_COMMON_DECLARATIONS();
	DUMP_COMMON_DECLARATIONS();
	DUMP_IMPL_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void addLink() const;
	void addSum() const;
	void drawChain(EdgeLoop ) const;
	void drawLink() const;
	void drawSum() const;
	void draw() const;
#endif

	const OpSegment* segment;
	OpEdge* priorEdge;	// edges that link to form completed contour
	OpEdge* nextEdge;
	OpEdge* lastEdge;
	OpEdge* priorSum;	// edge that set sum winding
	OpEdge* loopStart;	// if sum loops, furthest edge away from end
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
	float priorNormal;  // e.g., for horizontal axis, y value of intersecting ray
	float priorT; // temporary used to carry result from prior sum to find winding
	int id;
	Axis priorAxis;	// the axis state when prior sum was found
	EdgeLink nextLink;
	EdgeLink priorLink;
	EdgeMatch whichEnd;	// if 'start', prior link end equals start; if 'end' prior end matches end
	EdgeFail fail;	// how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero; // normal means edge normal points to zero side; opposite, normal is non-zero
	EdgeSplit doSplit;
	bool curveSet;
	bool endAliased;
	bool lineSet;
	bool verticalSet;
	bool isLine_impl;	// ptBounds 0=h/0=w catches horz/vert lines; if true, line is diagonal(?)
	bool isPoint;
	bool isSumLoop;
	bool active_impl;  // used by ray casting to mark edges that may be to the left of casting edge
	bool startAliased;
	bool unsortable;	// sum was not resolvable by ray in either axis (likely edge is very small) 
#if OP_DEBUG
	EdgeMaker debugMaker;
	OpPoint debugOriginalStart;
	OpPoint debugOriginalEnd;
	int debugParentID;
	int debugAliasStartID;
	int debugAliasEndID;
#endif
};

#endif
