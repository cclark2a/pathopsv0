// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

#include "OpEdge.h"
#include "OpIntersection.h"

enum class SectFound {
	no,
	fail,
	intersects,
	overflow,
	split
};

enum class DoSplit {
	marked,
	all
};

enum class CurveRef {
	edge,
	opp
};

struct OpCurveCurve {
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 
	static constexpr int maxDepth = 24;  // !!! no idea what this should be

	OpCurveCurve(const OpEdge* edge, const OpEdge* opp)
		: originalEdge(edge)
		, originalOpp(opp)
		, sectResult(false) {
		edgeCurves.emplace_back(*edge);
		oppCurves.emplace_back(*opp);
#if OP_DEBUG_DUMP
		debugActive = this;
//		debugSaveID();
#endif
	}

	SectFound addUnsectable(); // if curve doesn't devolve into line segments
	SectFound curvesIntersect(CurveRef );
	SectFound divideAndConquer();
	SectFound divideExperiment();
	void findEdgesTRanges(CurveRef );
	void linearIntersect(std::vector<OpEdge>& lines, std::vector<OpEdge>& linesOrCurves);
	bool split(CurveRef , DoSplit );
	bool tooFew(CurveRef );

#if OP_DEBUG_DUMP
	static const OpCurveCurve* debugActive;

	~OpCurveCurve() {
		debugActive = nullptr;
//		debugRestoreID();	// see implementation for why this is disabled
	}

	void debugSaveID();
	void debugRestoreID();
	void dump(bool detail) const;
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif

	const OpEdge* originalEdge;
	const OpEdge* originalOpp;
	std::vector<OpEdge> edgeCurves;
	std::vector<OpEdge> edgeLines;
	std::vector<OpEdge> oppCurves;
	std::vector<OpEdge> oppLines;
	std::vector<OpEdge> edgeRuns;
	std::vector<OpEdge> oppRuns;
	bool sectResult;
};

#endif
