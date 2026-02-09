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

int minMaxLimbs(Context* ) {
	return 120;
}

bool DebugAnalyze(Context* c) {
	OpContext* context = (OpContext*) c;
	if (context->debugData.limitContours > 0) {
        context->contextCallbacks.maxLimbsFuncPtr = minMaxLimbs;
        return true;
    }
    return false;
}

void SetSkiaSimplifyCallbacksDebug(Context* context, Contour*, SkPath const&) {
    SetDebugContextCallbacks(context, {
            unaryDebugIsFill
            OP_DEBUG_DUMP_PARAMS(unaryDumpOutFunc, unaryDumpSetFunc, dumpSkiaOutPath,
                    unaryImageOutFunc, nullptr, unaryColorFunc) }
    );
}

void SetSkiaOpContourCallbacksDebug(Context* context, Contour*, 
        BinaryOperand, SkPath const&) {
    SetDebugContextCallbacks(context, {
            binaryDebugIsFill
            OP_DEBUG_DUMP_PARAMS(binaryDumpOutFunc, binaryDumpSetFunc, dumpSkiaOutPath,
                    binaryImageOutFunc, binaryImageNamesFunc, binaryColorFunc,
                    binaryVisibleFunc)
    });
}

void AddDebugSkiaPath(Context* c, Contour* contour, const SkPath& path) {
	OpContext* context = (OpContext*) c;
	OpDebugData& debugData = context->debugData;
	OpPointBounds snag { 20, 0, 40, 10 };  // only snag contours that start in this bounds
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
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1]) {
                if (snagOn) AddLine(contour, { c, closeLine, sizeof(closeLine), 
						(CurveType) SkPath::kLine_Verb } );
				if (++contourCount >= debugData.limitContours)
					return;
			}			
            closeLine[0] = closeLine[1] = { pts[0].fX, pts[0].fY };
			snagOn = snag.contains(closeLine[1]);
            pts[1] = pts[0];
			contour = Clone(contour);
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                if (snagOn) AddLine(contour, { c, (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkPath::kLine_Verb } );
            closeLine[0] = { pts[1].fX, pts[1].fY };
            break;
        case SkPath::kQuad_Verb:
            if (snagOn) AddQuads(contour, { c, (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkPath::kQuad_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kConic_Verb:
            pts[3].fX = iter.conicWeight(); // !!! hacky
            if (snagOn) AddConics(contour, { c, (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkPath::kConic_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kCubic_Verb:
            if (snagOn) AddCubics(contour, { c, (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkPath::kCubic_Verb } );
            closeLine[0] = { pts[3].fX, pts[3].fY };
            break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                if (snagOn) AddLine(contour, { c, closeLine, sizeof(closeLine), 
						(CurveType) SkPath::kLine_Verb } );
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
