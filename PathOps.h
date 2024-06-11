// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "OpDebugDump.h"
#include "OpOperators.h"

#if OP_DEBUG
#include <vector>
#endif

struct OpInPath {
	OpInPath(const void* ext) 
		: externalReference(ext)  {
	}
	bool isInverted() const;
#if OP_DEBUG_DUMP
	~OpInPath();
	static const void* MakeExternalReference();
	static void ReleaseExternalReference(const void* );
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
	~OpOutPath();
	static void* MakeExternalReference();
	static void ReleaseExternalReference(void* );
	DUMP_DECLARATIONS
#endif

	void* externalReference;
#if OP_DEBUG
	int debugID;
#endif
};

bool PathOps(OpInPath& left, OpInPath& right, OpOperator , OpOutPath& result);
bool PathSimplify(OpInPath& path, OpOutPath& result);

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath& left, OpInPath& right, OpOperator , OpOutPath& result,
        OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
bool DebugPathSimplify(OpInPath& path, OpOutPath& result, 
		OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
#endif

// the interfaces above will be replaced with the context model below (maybe?)
#include "PathOpsTypes.h"

struct OpPair;

namespace PathOpsV0Lib {

/* usage:


*/

// functions
/* Adds one curve. Uses SetCurveLength() to retrieve the length to copy.
 */
void Add(Context* , Curve );

/* Makes a PathOps context. 
 */
Context* Create();

/* Deletes a PathOps context, and frees any memory associated with that context.
 */
void Delete(Context* );

/* returns error code of previous call
 */
int Error(Context* );

/* subsequent calls to Add() associates data and type arrays with this contour ID
 */
void Contour(Context* , int contourID);

/* Operate on curves provided by Add() with user defined operation.
   Calls user supplied functions to return computed path one curve at a time.
 */
void Resolve(Context* , int operation);

OpType SetCurveCallBacks(Context* , AxisRawHit, CurveHull, CurveIsLine, CurveIsLinear,
		CurveControls, SetControls, CurveLength, CurveNormal, 
		CurveReverse, CurveTangent, CurvesEqual, PtAtT, DoublePtAtT,
		PtCount, SubDivide, XYAtT
#if OP_DEBUG_IMAGE
		, DebugAddToPath
#endif
	);

int SetOperationCallBacks(Context* ); // !!! incomplete

// utilities
size_t AddQuads(Context* , Curve );
OpPoint QuadControlPt(OpPoint start, OpPoint control, OpPoint end, OpPtT ptT1, OpPtT ptT2);
OpPoint QuadPointAtT(OpPoint start, OpPoint control, OpPoint end, float t);
OpVector QuadTangent(OpPoint start, OpPoint control, OpPoint end, float t);
OpPair QuadXYAtT(OpPoint start, OpPoint control, OpPoint end, OpPair t, XyChoice xy);
OpRoots QuadAxisRawHit(OpPoint start, OpPoint control, OpPoint end, Axis, float axisIntercept);

} // namespace PathOpsV0Lib

#endif
