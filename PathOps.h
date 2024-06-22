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
		: externalReference(ext)
	#if OP_DEBUG_DUMP
	, debugDumpReference(false)
	#endif
	{
	}
	bool isInverted() const;
#if OP_DEBUG_DUMP
	~OpInPath();
	static const void* MakeExternalReference();
	static void ReleaseExternalReference(const void* );
	DUMP_DECLARATIONS
#endif

	const void* externalReference;
#if OP_DEBUG_DUMP
	bool debugDumpReference;
#endif
};

struct OpOutPath {
	OpOutPath(void* ext) 
		: externalReference(ext) 
		#if OP_DEBUG_DUMP
		, debugDumpReference(false)
		#endif
	{
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
#if OP_DEBUG_DUMP
	bool debugDumpReference;
#endif
#if OP_DEBUG
	int debugID;
#endif
};

bool PathOps(OpInPath& left, OpInPath& right, OpOperator , OpOutPath& result  
		OP_DEBUG_PARAMS(OpDebugData& ));
bool PathSimplify(OpInPath& path, OpOutPath& result  OP_DEBUG_PARAMS(OpDebugData& ));

// the interfaces above will be replaced with the context model below (maybe?)
#include "PathOpsTypes.h"

struct OpPair;

namespace PathOpsV0Lib {

/* usage:


*/

// functions
/* Adds one curve. Uses SetCurveLength() to retrieve the length to copy.
 */
void Add(AddCurve , AddWinding );

/* Makes a PathOps context. 
 */
Context* CreateContext();

/* Deletes a PathOps context, and frees any memory associated with that context.
 */
void DeleteContext(Context* );

/* Makes a PathOps contour.
 */
Contour* CreateContour(Context* );

/* returns error code of previous call
 */
int Error(Context* );

/* Operate on curves provided by Add() with user defined operation.
   Calls user supplied functions to return computed path one curve at a time.
 */
void Resolve(Context* , OpType operation);

OpType SetCurveCallBacks(Context* , AxisRawHit, CurveHull, CurveIsLine, CurveIsLinear,
		CurveControls, SetControls, /*CurveLength,*/ CurveNormal, 
		CurveReverse, CurveTangent, CurvesEqual, PtAtT, DoublePtAtT,
		PtCount, SubDivide, XYAtT
#if OP_DEBUG_IMAGE
		, DebugAddToPath
#endif
	);

OpType SetOperationCallBacks(Context* ); // !!! incomplete

// utilities
size_t AddQuads(AddCurve , AddWinding );
OpPoint QuadControlPt(OpPoint start, OpPoint control, OpPoint end, OpPtT ptT1, OpPtT ptT2);
OpPoint QuadPointAtT(OpPoint start, OpPoint control, OpPoint end, float t);
OpVector QuadTangent(OpPoint start, OpPoint control, OpPoint end, float t);
OpPair QuadXYAtT(OpPoint start, OpPoint control, OpPoint end, OpPair t, XyChoice xy);
OpRoots QuadAxisRawHit(OpPoint start, OpPoint control, OpPoint end, Axis, float axisIntercept);

#if OP_DEBUG
void Debug(Context* , OpDebugData& );
#endif

} // namespace PathOpsV0Lib

#endif
