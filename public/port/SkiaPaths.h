// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef SkiaPaths_DEFINED
#define SkiaPaths_DEFINED

#include "PathOps.h"
// #include "SkiaEnumSkPathOp.h"
#include "SkiaPathsDebug.h"

class SkPath;
enum class TinyOps;

namespace PathOpsV0Lib {

enum class BinaryOperand : int;
enum class BinaryWindType : int;

enum class SkiaUserData {
	useDoubleConics = (int) UserDataType::optional
};

void emptySkPathFunc(Context* );

}

TinyOps MapInvertedSkPathOp(TinyOps op, bool leftOperandIsInverted, bool rightOperandIsInverted);
bool SkPathOpInvertOutput(TinyOps op, bool leftOperandIsInverted, bool rightOperandIsInverted);

void AlignSkiaOutput(PathOpsV0Lib::Output o);
void SetSkiaContextCallbacks(PathOpsV0Lib::Context* );
void SetSkiaCurveCallbacks(PathOpsV0Lib::Context* );
PathOpsV0Lib::Contour* SetSkiaSimplifyCallbacks(PathOpsV0Lib::Context* , PathOpsV0Lib::WindingData , 
		size_t size, bool isWindingFill  OP_DEBUG_PARAMS(const SkPath* debugSkPath = nullptr));
void SetSkiaOpContextCallbacks(PathOpsV0Lib::Context* , TinyOps , PathOpsV0Lib::BinaryWindType );
PathOpsV0Lib::Contour* SetSkiaOpContourCallbacks(PathOpsV0Lib::Context* , PathOpsV0Lib::WindingData , 
		size_t size, PathOpsV0Lib::BinaryOperand  OP_DEBUG_PARAMS(const SkPath* debugSkPath = nullptr));
void AddSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , const SkPath& 
        /* OP_DEBUG_PARAMS(AddDebugContour* debugAddContour = nullptr) */);
bool VeryLargeSkiaPath(const SkPath& );

#endif
