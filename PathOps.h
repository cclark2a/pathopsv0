#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "OpOperators.h"

#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
class SkPath;
#endif

struct OpInPath {
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	OpInPath(const SkPath* sk)
		: skPath(sk) {
		OP_ASSERT(skPath);
	}
#elif PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_PENTREK
	OpInPath(const void* ext) 
		: externalReference(ext) {
	}
#endif

	bool isInverted() const;
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	const SkPath* skPath;
#elif PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_PENTREK
	const void* externalReference;
#endif
};

struct OpOutPath {
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	OpOutPath(SkPath* sk)
		: skPath(sk) {
		OP_ASSERT(skPath);
	}
#elif PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_PENTREK
	OpOutPath(const void* ext) 
		: externalReference(ext) {
	}
#endif

	void setEmpty();
	void setInverted(bool wasInverted);
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
#if OP_DEBUG_DUMP
	bool debugIsEmpty() const;
	int debugNextID(struct OpEdge* );
	void dump() const;
	void dumpDetail() const;
#endif
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	SkPath* skPath;
#elif PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_PENTREK
	const void* externalReference;
#endif
#if OP_DEBUG_DUMP
	int debugID;
#endif
};

bool PathOps(OpInPath left, OpInPath right, OpOperator , OpOutPath result);
bool PathSimplify(OpInPath path, OpOutPath result);

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath left, OpInPath right, OpOperator , OpOutPath result,
        OpDebugExpect expected);
#endif

#endif
