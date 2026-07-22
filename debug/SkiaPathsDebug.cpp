// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG

#include "TinySkia.h"
#include "port/SkiaPaths.h"
#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"
#include "OpContext.h"

// char* so it can be called from immediate window
namespace PathOpsV0Lib {

std::string dumpSkiaOutPath(Context* context) {
    ContextUserData userData = UserData(context, UserDataType::outPath);
    OP_ASSERT(userData.size == sizeof(const SkPath*));
    const SkPath* outPath = (const SkPath*) userData.data;
    return dumpSkPath(outPath, true, "    path.");
}

}

using namespace PathOpsV0Lib;

void SetSkiaSimplifyCallbacksDebug(Context* context, Contour*, SkPath const&) {
    SetDebugContextCallbacks(context, {
            unaryDebugIsFill, nullptr, nullptr
            OP_DEBUG_SERIALIZE_PARAMS(unaryDumpOutFunc, unaryDumpSetFunc, dumpSkiaOutPath,
                    unaryImageOutFunc, nullptr, nullptr, nullptr)
#if OP_DEBUGGER
                    , unaryColorFunc
#else
                    , "unaryColorFunc"
#endif
            }
    );
}

void SetSkiaOpContourCallbacksDebug(Context* context, Contour*, 
        BinaryOperand, SkPath const&) {
    SetDebugContextCallbacks(context, {
            binaryDebugIsFill, nullptr, nullptr
            OP_DEBUG_SERIALIZE_PARAMS(binaryDumpOutFunc, binaryDumpSetFunc, dumpSkiaOutPath,
                    binaryImageOutFunc, binaryImageNamesFunc, binaryVisibleFunc, nullptr)
#if OP_DEBUGGER
                    , binaryColorFunc
#else
                    , "binaryColorFunc"
#endif
    });
}

void AddDebugSkiaPath(Context* c, Contour* contour, const SkPath& path) {
	OpContext* context = (OpContext*) c;
	OpDebugData& debugData = context->debugData;
	OpRect snag = { debugData.limitBoundsL, debugData.limitBoundsT, 
            debugData.limitBoundsR, debugData.limitBoundsB };  // only snag contours that start in this bounds
	bool snagOn = false;
	int contourCount = 0;
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(c, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
    for (;;) {
        SkPoint pts[4];
        OpPoint* opPts = (OpPoint*) pts;
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1]) {
                if (snagOn) AddLine(contour, { c, closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
				if (++contourCount >= debugData.limitContours)
					return;
			}			
            closeLine[0] = closeLine[1] = opPts[0];
			snagOn = snag.contains(closeLine[1]);
            opPts[1] = opPts[0];
			contour = Clone(contour);
            break;
        case SkPath::kLine_Verb:
            if (snagOn) AddLine(contour, { c, opPts, sizeof(OpPoint) * 2, SkPath::kLine_Verb } );
            closeLine[0] = opPts[1];
            break;
        case SkPath::kQuad_Verb:
            if (snagOn) AddQuads(contour, { c, opPts, sizeof(OpPoint) * 3, SkPath::kQuad_Verb } );
            closeLine[0] = opPts[2];
            break;
        case SkPath::kConic_Verb:
            opPts[3].x = iter.conicWeight(); // !!! hacky
            if (snagOn) AddConics(contour, { c, opPts, sizeof(OpPoint) * 3 + sizeof(float), 
                    SkPath::kConic_Verb } );
            closeLine[0] = opPts[2];
            break;
        case SkPath::kCubic_Verb:
            if (snagOn) AddCubics(contour, { c, opPts, sizeof(SkPoint) * 4, SkPath::kCubic_Verb } );
            closeLine[0] = opPts[3];
            break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1] && snagOn)
                AddLine(contour, { c, closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
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
