#ifndef PathOps_DEFINED
#define PathOps_DEFINED

// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"
#include "OpOperators.h"

struct OpInPath {
	OpInPath(const void* ext) 
		: externalReference(ext) {
	}
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
	const SkPath* skPath() const { return (const SkPath*) externalReference; }
#endif

	bool isInverted() const;
	const void* externalReference;
};

struct OpOutPath {
	OpOutPath(void* ext) 
		: externalReference(ext) {
	}

	void setEmpty();
	void setInverted(bool wasInverted);
#if OP_DEBUG
	bool debugIsEmpty() const;
	int debugNextID(struct OpEdge* );
#endif
#if OP_DEBUG_DUMP
	void dump() const;
	void dumpDetail() const;
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
	SkPath* skPath() { return (SkPath*) externalReference; }
	const SkPath* skPath() const { return (const SkPath*) externalReference; }
#endif

	void* externalReference;
#if OP_DEBUG
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
