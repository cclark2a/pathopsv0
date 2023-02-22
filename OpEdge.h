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
	applyOp,
	centerNaN,
	coincidence,
	collapsed,
	failCenter,
	failNext,
	failPrior,
	failWinding,
	isPoint,
	linkUp,
	looped,
	matchClosest,
	matchLink,
	noFlip,
	rayNormal
};

struct OpWinding {
	OpWinding(WindingEdge) 
		: left(0)
		, right(0)
#if OP_DEBUG
		, setter(0)
		, reason(ZeroReason::none)
#endif
	{
	}

	OpWinding(WindingSum)
		: left(OpMax)
		, right(OpMax)
#if OP_DEBUG
		, setter(0)
		, reason(ZeroReason::none)
#endif	
	{
	}

	OpWinding(OpOperand operand)
		: left(OpOperand::left == operand ? 1 : 0)
		, right(OpOperand::right == operand ? 1 : 0)
#if OP_DEBUG
		, setter(0)
		, reason(ZeroReason::none)
#endif	
	{
	}

	void move(OpWinding& opp, const OpContours* , bool backwards);

	int oppSide(OpOperand operand) const {
		return OpOperand::left == operand ? right : left;
	}

	int sum() const {
		return left + right;
	}

	// only for sum
	bool unset() const {
		return OpMax == left && OpMax == right;
	}

	bool visible() const {
		return left || right;
	}

	// debug parameter tracks the caller that zeroed the edge
	void zero(ZeroReason r) { 
		left = 0;
		right = 0;
#if OP_DEBUG
		if (ZeroReason::none == reason)	// if we already failed, don't fail again?
			reason = r;
#endif
	}

#if OP_DEBUG_DUMP
	std::string debugDump() const;
	void dump() const;
#endif

	int left;
	int right;

#if OP_DEBUG
	int setter;
	ZeroReason reason;
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

enum class EdgeLoop {
	link,
	sum
};

enum class EdgeSplit {
	no,
	yes,
	unsplittable
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	assert(WindZero::noFlip != left || WindZero::noFlip != right);
	assert((int) left + (int) right != 3);	// not normal and opp at same time
}

#if OP_DEBUG
enum class EdgeMaker {
	addTest,
	intersectEdge1,
	intersectEdge2,
	makeEdges,
	opTest,
	split
};
#endif

struct OpEdge {
private:
	OpEdge()	// note : all values are zero
		: priorEdge(nullptr)
		, nextEdge(nullptr)
		, lastEdge(nullptr)
		, priorSum(nullptr)
		, nextSum(nullptr)
		, winding(windingEdge)
		, sum(windingSum)
		, priorT(0)
		, nextAxis(Axis::vertical)
		, nextLink(EdgeLink::unlinked)
		, priorAxis(Axis::vertical)
		, priorLink(EdgeLink::unlinked)
		, whichEnd(EdgeMatch::none)
		, fail(EdgeFail::none)
		, windZero(WindZero::noFlip)
		, doSplit(EdgeSplit::no)
		, curveSet(false)
		, lineSet(false)
		, verticalSet(false)
		, isLine_impl(false)
		, isPoint(false)
		, active(false)
		, seenNext(false)
		, seenPrior(false)
		, unsortable(false) {
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

#if OP_DEBUG_IMAGE
	OpEdge(const OpEdge&) = default;
	OpEdge(OpEdge&&) = default;
	OpEdge& operator=(const OpEdge&) = default;
	OpEdge& operator=(OpEdge&&) = default;

	~OpEdge() {
		if (OpDebugPathOpsEnable::inPathOps)
			OpDebugImage::clear(id);
	}
#endif
	void apply();
	void calcCenterT();
	void calcWinding(Axis axis);
	void clearActive();  // setter exists so debug breakpoints can be set
	void clearNextEdge();
	void clearPriorEdge();
	void complete();
	bool containsLink(const OpEdge* edge) const;
	float findPtT(OpPoint pt) const;
	OpPtT flipPtT(EdgeMatch match) const { return match == whichEnd ? end : start; }
	void flipWhich() { whichEnd = (EdgeMatch)((int)whichEnd ^ (int)EdgeMatch::both); }
	void findWinding(Axis axis  OP_DEBUG_PARAMS(int* debugWindingLimiter));
	bool hasLinkTo(EdgeMatch match) const { 
			return EdgeLink::single == (EdgeMatch::start == match ? nextLink : priorLink); }
	bool isClosed(OpEdge* test);
	const OpEdge* isLoop(EdgeLoop ) const;
	OpEdge* linkUp(EdgeMatch , OpEdge* firstEdge);
	void addMatchingEnds(const OpEdge& ) const;
	void markFailNext(std::vector <OpEdge*>& , Axis );
	void markFailPrior(std::vector <OpEdge*>& , Axis );
	bool matchLink(std::vector<OpEdge*>& linkups );
	bool nextSumLoops();
	OpEdge* prepareForLinkup();
	bool priorSumLoops();
	OpPtT ptT(EdgeMatch match) const { return EdgeMatch::start == match ? start : end; }
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
	const OpCurve& setVertical();
	void setWinding(OpVector ray);
	void subDivide();
	bool validLoop(EdgeLoop ) const;
//	OpPtT whichPtT() const { return EdgeMatch::start == whichEnd ? start : end;  }
	OpPtT whichPtT(EdgeMatch match = EdgeMatch::start) const { return match == whichEnd ? start : end; }

#if OP_DEBUG_DUMP
	OpEdge(std::string );
	OpEdge(OpPtT data[2]);
	OpEdge(OpHexPtT data[2]);
	void debugCompare(std::string ) const;
	std::string debugDumpBrief() const;
	std::string debugDumpChain(EdgeLoop , bool detail) const;
	std::string debugDumpWinding() const;
	void debugValidate() const;    // make sure pointer to edge is valid
	void dumpChain(EdgeLoop ) const;
	void dumpChainDetail(EdgeLoop ) const;
	void dumpDetail() const;
	void dumpEnd() const;
	void dumpFull() const;
	void dumpLink() const;
	void dumpLinkDetail() const;
	void dumpStart() const;
	void dumpSum() const;
	void dumpSumDetail() const;
	void dumpWinding() const;
	DEBUG_COMMON_DECLARATIONS();
	DUMP_COMMON_DECLARATIONS();
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
	OpEdge* nextSum;	// closest edge to the right/bottom
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
	OpPointBounds pointBounds;
	OpPointBounds linkBounds;
	OpWinding winding;	// contribution: always starts as 1, 0 (or 0, 1)
	OpWinding sum; // total incl. normal side of edge for operands (fill count in normal direction)
	float priorT; // temporary used to carry result from prior sum to find winding
	int id;
	Axis nextAxis;	// the axis state when prior sum was found
	EdgeLink nextLink;
	Axis priorAxis;	// the axis state when prior sum was found
	EdgeLink priorLink;
	EdgeMatch whichEnd;	// if 'start', prior link end equals start; if 'end' prior end matches end
	EdgeFail fail;	// how computation (e.g., center) failed (on fail, windings are set to zero)
	WindZero windZero; // normal means edge normal points to zero side; opposite, normal is non-zero
	EdgeSplit doSplit;
	bool curveSet;
	bool lineSet;
	bool verticalSet;
	bool isLine_impl;	// pointBounds 0=h/0=w catches horz/vert lines; if true, line is diagonal(?)
	bool isPoint;
	bool active;  // used by ray casting to mark edges that may be to the left of casting edge
	bool seenNext;
	bool seenPrior;
	bool unsortable;
#if OP_DEBUG
	EdgeMaker debugMaker;
	int debugParentID;
#endif

};

#endif
