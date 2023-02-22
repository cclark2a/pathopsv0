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
		assert(skPath);
	}
#endif

	bool isInverted() const;
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	const SkPath* skPath;
#endif
};

struct OpOutPath {
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	OpOutPath(SkPath* sk)
		: skPath(sk) {
		assert(skPath);
	}
#endif

	bool setEmpty();
	bool setInverted(bool wasInverted);
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
#if OP_DEBUG_DUMP
	void dump() const;
#endif
#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
	SkPath* skPath;
#endif
};

bool PathOps(OpInPath left, OpInPath right, OpOperator op, OpOutPath result);

#endif
