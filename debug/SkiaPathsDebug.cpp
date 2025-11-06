// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG

#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

#include "port/SkiaPaths.h"
#include "OpSkiaTests.h"
#if TEST_RASTER
#include "OpDebugRaster.h"
#endif

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"

using namespace PathOpsV0Lib;

struct UnaryContext {
};

struct BinaryContext : public UnaryContext {
	BinaryOperation operation;
};

#if 0  // disabled until we need it
#if !OP_TINY_SKIA
#include "include/core/SkStream.h"

std::string dumpSkPath(const SkPath* path, bool inHex) {
    SkDynamicMemoryWStream memoryStream;
    path->dump(&memoryStream, inHex);
    std::string str;
    str.resize(memoryStream.bytesWritten());
    memoryStream.copyTo(str.data());
    str.pop_back();
    return str;
}


std::string dumpSkContour(const SkPath* path, int contour, bool inHex) {
	std::string s = dumpSkPath(path, inHex);
	size_t pos = s.find("path.moveTo(", 0);
	if (std::string::npos == pos || contour < 0)
		return "";
	std::string closeLine = "path.close();\n";
	size_t endPos = std::string::npos;
	while (contour >= 0) {
		size_t closePos = s.find(closeLine, pos);
		endPos = std::string::npos == closePos ? s.size() : closePos + closeLine.size();
		size_t movePos = s.find("path.moveTo(", pos + 1);
		if (std::string::npos != movePos && movePos < endPos)
			endPos = movePos;
		if (contour--) {
			if (std::string::npos == movePos)
				return "*** error: " + s;
			pos = endPos;
		}
	}
	std::string result = s.substr(pos, endPos - pos);
	if ('\n' == result.back())
		result.pop_back();
	return result;
}
#endif

#if OP_DEBUG_DUMP
std::string PathOpsV0Lib::dumpUnaryContourFunc(DebugContourData caller, DebugLevel , 
        DebugBase debugBase) {
#if OP_TINY_SKIA
    return "";
#else
    if (sizeof(UnaryContour) > caller.size)
        return "size error:" + STR(caller.size);
    UnaryContour callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    return dumpSkContour(callerData.pathPtr, callerData.contourIndex,
			DebugBase::hex == debugBase) + "\n";
#endif
}

std::string PathOpsV0Lib::dumpBinaryContourFunc(DebugContourData caller, DebugLevel l, DebugBase b) {
	std::string s = dumpUnaryContourFunc(caller, l, b);
#if !OP_TINY_SKIA
    BinaryContour callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    OP_ASSERT(BinaryOperand::left == callerData.operand ||
            BinaryOperand::right == callerData.operand);
    s += "BinaryOperand:" 
            + std::string(BinaryOperand::left == callerData.operand ? "left" : "right");
#endif
    return s;
}
#endif

std::string PathOpsV0Lib::dumpBinaryContextFunc(DebugContextData caller, DebugLevel l, DebugBase b) {
#if OP_TINY_SKIA
    return "";
#else
    std::vector<std::string> skPathOpNames { "Difference", "Intersect", "Union",  "XOR",
        "ReverseDifference"  };
    BinaryContext callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    if (BinaryOperation::Difference > callerData.operation 
            || callerData.operation > BinaryOperation::ReverseDifference)
        return "caller.operation error:" + STR((int) callerData.operation);
    std::string s = "SkPathOp:" + skPathOpNames[(int) callerData.operation] + " ";
    return s;
#endif
}
#endif

#if OP_DEBUG_IMAGE
void* PathOpsV0Lib::debugSimplifyPathFunc(DebugContourData data) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return (void*) simplifyContourData.pathPtr;
}

bool PathOpsV0Lib::debugSimplifyGetDrawFunc(DebugContourData data) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return simplifyContourData.drawNativePath;
}

void PathOpsV0Lib::debugSimplifySetDrawFunc(DebugContourData data, bool draw) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    simplifyContourData.drawNativePath = draw;
    std::memcpy(data.data, &simplifyContourData, data.size);
}

void* PathOpsV0Lib::debugOpPathFunc(DebugContourData data) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return (void*) opContourData.pathPtr;
}

bool PathOpsV0Lib::debugOpGetDrawFunc(DebugContourData data) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return opContourData.drawNativePath;
}

void PathOpsV0Lib::debugOpSetDrawFunc(DebugContourData data, bool draw) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    opContourData.drawNativePath = draw;
    std::memcpy(data.data, &opContourData, data.size);
}

bool PathOpsV0Lib::debugOpSetIsOppFunc(DebugContourData data, int opp) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return BinaryOperand::left != opContourData.operand;
}
#endif

#if 0  // !!! unused?
// 0x00 (black) is 'on' ; 0xFF (white) is 'off' -- this reverses the intuitive operators
uint8_t skiaDebugBitOper(DebugContourData data, uint8_t src, uint8_t opp) {
    BinaryContext opContextData;
    OP_ASSERT(sizeof(opContextData) == data.size);
    std::memcpy(&opContextData, data.data, data.size);
//	uint8_t constexpr blackBit = 0x00;	// aide memoire
	uint8_t constexpr whiteBit = 0xFF;
	switch (opContextData.operation) {
		case BinaryOperation::Difference:
			return src ? whiteBit : ~opp;
		case BinaryOperation::Intersect:
			return src | opp;
		case BinaryOperation::Union:
			return src & opp;
		case BinaryOperation::ExclusiveOr:
			return ~(src ^ opp);
		case BinaryOperation::ReverseDifference:
			return opp ? whiteBit : ~src;
	}
	OP_ASSERT(0);
	return 0;
}
#endif

void SetSkiaSimplifyCallbacksDebug(Context* context, Contour* contour, const SkPath& path) {
    UnaryContour simplifyUserData { &path };
	SetDebugContourData(contour, { &simplifyUserData, sizeof(simplifyUserData) }, 
            DebugContourType::windingUserData );
	SetDebugContourCallbacks(contour, { 
    //        OP_DEBUG_DUMP_CODE(dumpUnaryContourFunc)
            OP_DEBUG_IMAGE_CODE(debugSimplifyPathFunc,
	        debugSimplifyGetDrawFunc, debugSimplifySetDrawFunc) }
    );
	SetDebugContextCallbacks(context, {
            unaryDebugIsFill
			OP_DEBUG_DUMP_PARAMS(unaryDumpOutFunc, unaryDumpSetFunc)
            OP_DEBUG_IMAGE_PARAMS(unaryImageOutXFunc, unaryImageOutFunc, unaryColorFuncPtr) }
    );
}

void SetSkiaOpContextCallbacksDebug(Context* context, SkPathOp op) {
    BinaryContext windingUserData { {}, (BinaryOperation) op };
	SetDebugContextData(context, { &windingUserData, sizeof(windingUserData) }, 
            DebugContextType::windingUserData );
}

void SetSkiaOpContourCallbacksDebug(Context* context, Contour* contour,
        BinaryOperand operand, const SkPath& path) {
    BinaryContour windingUserData { { &path }, operand };
	SetDebugContourData(contour, { &windingUserData, sizeof(windingUserData) },
            DebugContourType::windingUserData);
	SetDebugContourCallbacks(contour, {
	//		OP_DEBUG_DUMP_CODE(dumpBinaryContourFunc)
            OP_DEBUG_IMAGE_CODE(debugOpPathFunc,
	        debugOpGetDrawFunc, debugOpSetDrawFunc, debugOpSetIsOppFunc) }
    );
	SetDebugContextCallbacks(context, {
            binaryDebugIsFill
			OP_DEBUG_DUMP_PARAMS(binaryDumpOutFunc, binaryDumpSetFunc)
            OP_DEBUG_IMAGE_PARAMS(binaryImageOutXFunc, binaryImageOutFunc, binaryColorFuncPtr) }
    );
}

void AddDebugContour::add(PathOpsV0Lib::Contour* contour) {
	SetDebugContourData(contour, { &debugData, debugSize }, debugContourType );
	debugData.contourIndex++;
}

#if !OP_DEBUG_FAST_TEST && OP_DEBUG
#if !TEST_ANALYZE

bool PathOpsV0Lib::DebugAnalyze(Context* ) {
    return false;
}

#else

#include "OpContext.h"

inline int minMaxLimbs(Context* ) {
	return 120;
}

bool PathOpsV0Lib::DebugAnalyze(Context* context) {
	OpContext* context = (OpContext*) context;
	OpDebugData& debugData = context->debugData;
	if (debugData.limitContours <= 0)
		return false;
    SetContextCallbacks(context, { setSkiaLineType, emptySkPathFunc, nullptr, nullptr
			nullptr, nullptr, minMaxLimbs });
	return true;
}

void AddDebugSkiaPath(Context* context, Contour* contour, const SkPath& path) {
	OpContext* context = (OpContext*) context;
	OpDebugData& debugData = context->debugData;
	OpPointBounds snag { 20, 0, 40, 10 };  // only snag contours that start in this bounds
	bool snagOn = false;
	int contourCount = 0;
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(context, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1]) {
                if (snagOn) AddLine(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
				if (++contourCount >= debugData.limitContours)
					return;
			}			
            closeLine[0] = closeLine[1] = { pts[0].fX, pts[0].fY };
			snagOn = snag.contains(closeLine[1]);
            pts[1] = pts[0];
			contour = Clone(contour);
            OP_DEBUG_CODE(addDebugContour.add(contour));
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                if (snagOn) AddLine(contour, { (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkiaCurveType::skiaLineType } );
            closeLine[0] = { pts[1].fX, pts[1].fY };
            break;
        case SkPath::kQuad_Verb:
            if (snagOn) AddQuads(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkiaCurveType::skiaQuadType } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kConic_Verb:
            pts[3].fX = iter.conicWeight(); // !!! hacky
            if (snagOn) AddConics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkiaCurveType::skiaConicType } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kCubic_Verb:
            if (snagOn) AddCubics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkiaCurveType::skiaCubicType } );
            closeLine[0] = { pts[3].fX, pts[3].fY };
            break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                if (snagOn) AddLine(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
			if (++contourCount >= debugData.limitContours)
				return;
            if (SkPath::kDone_Verb == verb) {
				debugData.limitReached = true;
                return;
			}
            closeLine[0] = closeLine[1];
            continue;
        default:
            OP_ASSERT(0);
        }
    }
}

#endif
#endif

#endif
