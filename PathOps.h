#ifndef PathOps_DEFINED
#define PathOps_DEFINED

// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"
#include "OpOperators.h"

struct OpInPath {
	OpInPath(const void* ext) 
		: externalReference(ext) {
	}
	const SkPath* skPath() const { return (const SkPath*) externalReference; }

	bool isInverted() const;
	const void* externalReference;
};

struct OpOutPath {
	OpOutPath(void* ext) 
		: externalReference(ext) { 
		OP_DEBUG_CODE(debugID = 0);
	}

	void setEmpty();
	void setInverted(bool wasInverted);
	SkPath* skPath() { return (SkPath*) externalReference; }
	const SkPath* skPath() const { return (const SkPath*) externalReference; }
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
