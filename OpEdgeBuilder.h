#ifndef OpEdgeBuilder_DEFINED
#define OpEdgeBuilder_DEFINED

#include "OpContour.h"
#include "PathOps.h"

struct OpEdgeBuilder {
    static bool Assemble(OpContours& , OpOutPath );
    static void Output(OpEdge* edge, OpOutPath );
};

struct OpSegmentBuilder {
    static bool Build(OpInPath , OpContours&, OpOperand operand);
};

#if OP_DEBUG_DUMP
void DumpLinkups(const std::vector<OpEdge*>& linkups);
#endif

#endif
