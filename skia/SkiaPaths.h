// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"
#include "include/pathops/SkPathOps.h"

class SkPath;

enum class SkiaOperand {
	left,
	right
};

struct SimplifyUserData {
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawSkPath);
};

struct WindingUserData {
    SkPathOp op;
    SkiaOperand operand;
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawSkPath);
};

SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);

void SetSkiaCurveCallBacks(PathOpsV0Lib::Context* );
PathOpsV0Lib::Contour* SetSkiaWindingSimplifyCallBacks(PathOpsV0Lib::Context*  
		OP_DEBUG_PARAMS(const SkPath& ));
PathOpsV0Lib::Contour* SetSkiaWindingCallBacks(PathOpsV0Lib::Context* , SkPathOp op, SkiaOperand 
		OP_DEBUG_PARAMS(const SkPath& ));
void AddSkiaPath(PathOpsV0Lib::AddWinding , const SkPath& path);

#if OP_DEBUG_IMAGE
void debugLineAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugQuadAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugConicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugCubicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
#endif
