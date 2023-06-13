#ifndef OpSegmentBuilder_DEFINED
#define OpSegmentBuilder_DEFINED

#include "OpMath.h"
#include "PathOps.h"

struct OpContour;
struct ExtremaT;

struct OpSegmentBuilder {
	static void AddConic(OpContour*, const OpPoint pts[3], float weight);
	static void AddCubic(OpContour*, const OpPoint pts[4]);
	static void AddQuad(OpContour*, const OpPoint pts[3]);
    static bool Build(OpInPath , OpContours&, OpOperand operand);
	static std::vector<ExtremaT> FindExtrema(const OpTightBounds& bounds);
};

#endif
