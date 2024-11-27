// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpWinding_DEFINED
#define OpWinding_DEFINED

struct OpContour;

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

enum class WindingType  {
	uninitialized = -1,
	temp,
	winding,
	sum,
	copy  // used only by new interface
};

struct OpWinding {
	OpWinding(OpEdge* edge, WindingSum );

	OpWinding(WindingUninitialized )
		: contour(nullptr)
		OP_DEBUG_PARAMS(debugType(WindingType::uninitialized)) {
		w.data = nullptr;
		w.size = 0;
	}

	OpWinding(OpContour* c, PathOpsV0Lib::Winding );
	OpWinding& operator=(const OpWinding&);
	OpWinding(const OpWinding&);

	void add(const OpWinding& );
	bool equal(const PathOpsV0Lib::Winding ) const;
	PathOpsV0Lib::Winding copyData() const;

	bool isSet() const {
		return !!contour;
	}

	void subtract(const OpWinding& );
	void move(const OpWinding& opp, bool backwards);

	void setWind(const OpWinding& fromSegment) {
		contour = fromSegment.contour;
		w = fromSegment.copyData();
		OP_DEBUG_CODE(debugType = WindingType::winding);
	}

	int sum() const;
	bool visible() const;
	void zero();

#if OP_DEBUG_DUMP
	void dumpSet(const char*& str, OpContours*);
	DUMP_DECLARATIONS
#endif

	OpContour* contour;
	PathOpsV0Lib::Winding w;
	OP_DEBUG_CODE(WindingType debugType);
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
	unset,
	zero,
	nonZero,
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	OP_ASSERT(WindZero::unset != left || WindZero::unset != right);
	OP_ASSERT(left == right);	// not normal and opp at same time
}

inline WindZero operator!(const WindZero& a) {
	switch (a) {
		case WindZero::unset: return WindZero::unset;
		case WindZero::zero: return WindZero::nonZero;
		case WindZero::nonZero: return WindZero::zero;
	}
	OP_ASSERT(0);
	return WindZero::unset;
}

#endif
