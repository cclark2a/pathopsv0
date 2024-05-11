// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "OpDebugDump.h"
#include "OpDebugImage.h"
#include "OpOperators.h"

#if OP_DEBUG
#include <vector>
#endif

struct OpInPath {
	OpInPath(const void* ext) 
		: externalReference(ext) {
	}
	bool isInverted() const;
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	const void* externalReference;
};

struct OpOutPath {
	OpOutPath(void* ext) 
		: externalReference(ext) { 
		OP_DEBUG_CODE(debugID = 0);
	}

	void setEmpty();
	void setInverted(bool wasInverted);
#if OP_DEBUG
	bool debugIsEmpty() const;
	void debugNextID(struct OpEdge* );
#endif
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
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
        OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
bool DebugPathSimplify(OpInPath path, OpOutPath result, 
		OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
#endif

#endif
