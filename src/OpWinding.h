// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpWinding_DEFINED
#define OpWinding_DEFINED

#include "PathOpsTypes.h"

struct OpContour;

enum class WindingEdge {
	dummy
};

enum class WindingSegment {  // used only by new interface
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

enum class WindingType  {
	uninitialized = -1,
	temp,
	winding,
	sum,
	copy  // used only by new interface
};

struct OpWinding {
#if OP_TEST_NEW_INTERFACE
	OpWinding(OpEdge* edge, WindingSum );
	OpWinding(OpContour* c, WindingSegment );

	OpWinding(WindingUninitialized )
		: contour(nullptr)
		OP_DEBUG_PARAMS(debugType(WindingType::uninitialized)) {
		w.data = nullptr;
		w.size = 0;
	}

	OpWinding(OpContour* c, PathOpsV0Lib::Winding );

#else
private:
	// used by operator-
	OpWinding(int l, int r)
		: left_impl(l)
		, right_impl(r)
		OP_DEBUG_PARAMS(debugType(WindingType::winding)) {
	}

public:
	OpWinding(WindingTemp)  // used for winding accumulators before sum is set
		: left_impl(0)
		, right_impl(0)
		OP_DEBUG_PARAMS(debugType(WindingType::temp)) {
	}

	OpWinding(WindingUninitialized)	 // used by edge and segment winding before they are set
		: left_impl(OpMax)
		OP_DEBUG_PARAMS(right_impl(OpMax))
		OP_DEBUG_PARAMS(debugType(WindingType::uninitialized)) {
	}

	OpWinding(OpOperand operand)	// used to set initial segment winding
		: left_impl(OpOperand::left == operand ? 1 : 0)
		, right_impl(OpOperand::right == operand ? 1 : 0)
		OP_DEBUG_PARAMS(debugType(WindingType::winding)) {
	}

	bool operator==(OpWinding w) const {
		return left_impl == w.left_impl && right_impl == w.right_impl;
	}

	OpWinding operator-() const {
		OP_ASSERT(WindingType::winding == debugType);
		return { -left_impl, -right_impl };
	}
#endif

#if OP_TEST_NEW_INTERFACE
	void add(const OpWinding& );
#else
	OpWinding& operator+=(const OpWinding& w) {
		OP_ASSERT(WindingType::temp == debugType || WindingType::winding == debugType);
		left_impl += w.left_impl;
		right_impl += w.right_impl;
		return *this;
	}
#endif

#if OP_TEST_NEW_INTERFACE
	bool compare(const PathOpsV0Lib::Winding ) const;  // returns true if not equal
	PathOpsV0Lib::Winding copyData() const;

	bool isSet() const {
		return !!contour;
	}

	void subtract(const OpWinding& );
	void move(const OpWinding& opp, bool backwards);
#else
	OpWinding& operator-=(const OpWinding& w) {
		OP_ASSERT(WindingType::temp == debugType || WindingType::winding == debugType);
		left_impl -= w.left_impl;
		right_impl -= w.right_impl;
		return *this;
	}

	bool isSet() const {
		return OpMax != left_impl;
	}

	int left() const {
		return left_impl;
	}

	void move(OpWinding opp, const OpContours* , bool backwards);
#endif


#if 0 // !!! unused?
	int oppSide(OpOperand operand) const {
		return OpOperand::left == operand ? right_impl : left_impl;
	}
#endif

#if !OP_TEST_NEW_INTERFACE
	int right() const {
		return right_impl;
	}
#endif


#if OP_TEST_NEW_INTERFACE
	void setWind(const OpWinding& fromSegment) {
		contour = fromSegment.contour;
		w = fromSegment.copyData();
		OP_DEBUG_CODE(debugType = WindingType::winding);
	}
#else
	void setWind(int left, int right) {	// shouldn't be 0, 0 (call zero() for that)
		OP_ASSERT(WindingType::uninitialized == debugType);
		OP_ASSERT(left || right);
		left_impl = left;
		right_impl = right;
		OP_DEBUG_CODE(debugType = WindingType::winding);
	}

	void setSum(OpWinding winding, const OpContours* segment);
#endif

#if OP_TEST_NEW_INTERFACE
//	void setSum(const OpWinding*   OP_DEBUG_PARAMS(char* file, int line));
	int sum() const;
	bool visible() const;
	void zero();
#else
	int sum() const { return left_impl + right_impl; }
	bool visible() const { return left_impl || right_impl; }
	void zero() { left_impl = right_impl = 0;	// only used by coincident lines }
#endif
#if OP_DEBUG_DUMP
	void dumpSet(const char*& str, OpContours*);
	DUMP_DECLARATIONS
#endif

#if OP_TEST_NEW_INTERFACE
	OpContour* contour;
	PathOpsV0Lib::Winding w;
#else
	int left_impl;	// indirection to make set debugging breakpoints easier 
	int right_impl;
#endif
#if OP_DEBUG
	WindingType debugType;
#endif

};

// An edge that can contribute to the answer has a zero winding on one side
// For a pair of edges to connect, they have to have zero windings on the same side
// If they have zero windings that do not match, there should be a third (and fourth)
// edge at the same point that is a better match.
// A pair of edges that are nearly coincident may be mis-sorted so that the zero
// winding is wrong.
// The normal zero winding is computed before the edge orientation (e.g., whichEnd) 
// is known, so it may be reversed if the edge is to be connected backwards.
enum class WindZero : int8_t {
	unset = -1,
	zero,
	nonZero,
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	OP_ASSERT(WindZero::unset != left || WindZero::unset != right);
	OP_ASSERT(left == right);	// not normal and opp at same time
}

inline WindZero operator!(const WindZero& a) {
    if (WindZero::unset == a)
        return a;
    return (WindZero) !static_cast<int>(a);
}

#endif
