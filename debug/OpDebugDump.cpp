// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_DUMP || OP_DEBUGGER
#include <ctype.h>
#ifdef _WIN32
#include <cstdio>  // for std::remove()
#endif

#include "OpCurveCurve.h"
#include "OpDebugColor.h"
#include "OpDebugDump.h"
#include "OpDebugRaster.h"
#include "OpEdge.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

// !!! things to do:
// decrement debug level (and indent) when dumping (for example) edges within an edge
// allow more flexible abbreviations for labels (none, first letter, string)
// consider what to macro-ize
// (and, in the end..) replace old calls with new ones
std::array<EdgeFilters, 3> edgeFilters;
int defaultLineWidth = 200;

#if defined _MSC_VER
#pragma optimize( "", off )
#endif

#define OP_X(Thing) \
	void dmp(const std::vector<Thing>& things) { \
		for (const auto& thing : things) { \
			dmp(thing); \
			OpDebugOut("\n"); \
		} \
	} \
	void dmpHex(const std::vector<Thing>& things) { \
		for (const auto& thing : things) { \
			dmpHex(thing); \
			OpDebugOut("\n"); \
		} \
	} \
	void dmp(const std::vector<Thing>* things) { \
        dmp(*things); \
    } \
	void dmpHex(const std::vector<Thing>* things) { \
        dmpHex(*things); \
    } \
	void dmpIDs(const std::vector<Thing>* things) { \
        dmpIDs(*things); \
    }
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void dmpIDs(const std::vector<Thing>& things) { \
		std::string s = "["; \
		for (const auto& thing : things) \
			s += thing.debugDumpID() + " "; \
        debugPopMatching(s, ' '); \
		OpDebugFormat(s + "]"); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dmpIDs(const std::vector<Thing>& things) { \
		std::string s = "["; \
		for (const auto& thing : things) \
			s += STR(thing->id) + " "; \
        debugPopMatching(s, ' '); \
		OpDebugFormat(s + "]"); \
	}
	VECTOR_PTRS

#undef OP_X
#define OP_X(Thing) \
    void dmp(const Thing& thing) { \
        OpDebugFormat(thing.debugDump(defaultLevel, defaultBase)); \
    } \
    void dmp(const Thing* thing) { \
        dmp(*thing); \
    } \
	void dmpBrief(const Thing& thing) { \
		OpDebugFormat(thing.debugDump(DebugLevel::brief, defaultBase)); \
    } \
    void dmpBrief(const Thing* thing) { \
        dmpBrief(*thing); \
    } \
	void dmpDetailed(const Thing& thing) { \
		OpDebugFormat(thing.debugDump(DebugLevel::detailed, defaultBase)); \
    } \
    void dmpDetailed(const Thing* thing) { \
        dmpDetailed(*thing); \
    } \
    void dmpHex(const Thing& thing) { \
        OpDebugFormat(thing.debugDump(defaultLevel, DebugBase::hex)); \
    } \
    void dmpHex(const Thing* thing) { \
        dmpHex(*thing); \
    } \
    void Thing::dump(DebugLevel dl, DebugBase db) const { \
        OpDebugFormat(this->debugDump(dl, db)); \
    } \
    void Thing::dump() const { \
        dmp(*this); \
    } \
    void Thing::dumpBrief() const { \
        dmpBrief(*this); \
    } \
    void Thing::dumpDetailed() const { \
        dmpDetailed(*this); \
    } \
    void Thing::dumpHex() const { \
        dmpHex(*this); \
    }
    VECTOR_STRUCTS
    OP_STRUCTS
#undef OP_X
#define OP_X(Thing, Struct) \
    void dmp##Thing(const struct Op##Struct* opStruct) { \
        dmp##Thing(*opStruct); \
    }
DETAIL_POINTS
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(const struct OpEdge* edge) { \
        dmp##Thing(*edge); \
    } \
    void OpEdge::dump##Thing() const { \
        dmp##Thing(*this); \
    }
EDGE_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(const struct OpSegment* segment) { \
        dmp##Thing(*segment); \
    } \
    void OpSegment::dump##Thing() const { \
        dmp##Thing(*this); \
    }
SEGMENT_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(int id) { \
        if (auto seg = findSegment(id)) \
            return dmp##Thing(seg); \
        if (auto edge = findEdge(id)) \
            return dmp##Thing(*edge); \
        if (auto intersection = findIntersection(id)) \
            return dmp##Thing(*intersection); \
    }
EDGE_OR_SEGMENT_DETAIL
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(int id) { \
        if (auto seg = findSegment(id)) \
            return dmp##Thing(seg); \
        if (auto edge = findEdge(id)) \
            return dmp##Thing(edge->segment); \
        if (auto intersection = findIntersection(id)) \
            return dmp##Thing(intersection->segment); \
    }
SEGMENT_DETAIL
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(int id) { \
        if (auto edge = findEdge(id)) \
            return dmp##Thing(edge); \
    }
EDGE_DETAIL
#undef OP_X

void OpDebugFormat(std::string s) {
    std::string result = stringFormat(s, defaultLineWidth);
	if (!result.empty() && '\n' != result.back())
		s += "\n";
    OpDebugOut(result);
}

void dmpToHex(float f) {
    OpDebugOut(OpDebugDumpHex(f));
}

void dmpToHex(uint32_t u) {
    OpDebugOut(OpDebugIntToHex(u));
}

void dmpWidth(int width) {
    defaultLineWidth = width;
}

#if OP_DEBUG_GLOBALS

void dmpActive() {
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isActive())
                    edge.dump();
            }
        }
    }
}

void dmpCoincidences() {
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto sect : seg.sects.i) {
                if (sect->coincidenceID)
                    sect->dump();
            }
        }
    }
}

void dmpCoins() {
    dmpCoincidences();
}

void dmpEdges() {
    std::string s;
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
			if (!seg.edges.size())
				continue;
            s += seg.debugDump(defaultLevel, defaultBase) + "\n";
            for (const auto& edge : seg.edges) {
                s += edge.debugDump(defaultLevel, defaultBase) + "\n";
            }
        }
    }
	if (debugGlobalContext->fillerStorage) {
		s += "fillerStorage:\n";
		int count = debugGlobalContext->fillerStorage->debugCount();
		for (int index = 0; index < count; ++index) {
			s += debugGlobalContext->fillerStorage->debugIndex(index)
					->debugDump(defaultLevel, defaultBase) + "\n";
		}
	}
	if (debugGlobalContext->ccStorage) {
		s += "ccStorage:\n";
		int count = debugGlobalContext->ccStorage->debugCount();
		for (int index = 0; index < count; ++index) {
			s += debugGlobalContext->ccStorage->debugIndex(index)
					->debugDump(defaultLevel, defaultBase) + "\n";
		}
	}
    OpDebugOut(s);
}

std::string debugDumpContext() {
    return debugGlobalContext->debugDump(DebugLevel::detailed, DebugBase::hex);
}

std::string debugDumpEdges() {
    size_t edgeCount = 0;
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            edgeCount += seg.edges.size();
        }
    }
    std::string s = "edges:" + STR(edgeCount) + "\n";
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                s += edge.debugDump(defaultLevel, defaultBase) + "\n";
            }
        }
    }
    return debugPopMatching(s, '\n');
}

std::string debugDumpIntersections() {
    size_t sectCount = 0;
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            sectCount += seg.sects.i.size();
        }
    }
    std::string s = "sects:" + STR(sectCount) + "\n";
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto sect : seg.sects.i) {
                s += sect->debugDump(DebugLevel::detailed, DebugBase::hex) + "\n";
            }
        }
    }
    return debugPopMatching(s, '\n');
}

#endif

void dmpFile() {
    debugGlobalContext->dumpBaseFile();
}

#if OP_DEBUG_GLOBALS

void dmpRays() {
    size_t edgeCount = 0;
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (!edge.ray.distances.empty())
					++edgeCount;
            }
        }
    }
    std::string s = "edges with rays:" + STR(edgeCount) + "\n";
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
				if (edge.ray.distances.empty())
					continue;
				s += "[" + STR(edge.id) + "] ";
				if (DebugLevel::detailed == defaultLevel) {
					s += "seg[" + STR(edge.segment->id) + "] ";
					s += "contour[" + STR(edge.segment->contour->id)+ "] ";
				}
                s += edge.ray.debugDump(defaultLevel, defaultBase) + "\n";
            }
        }
    }
    debugPopMatching(s, '\n');
    OpDebugFormat(s);
}

#endif

void OpContext::dumpResolve(OpContour*& contourRef) {
    int contourID = (int) (size_t) contourRef;
    if (!contourStorage)  // !!! looks like a macro candidate to me
        OpDebugExit(__func__ + std::string(": !contourStorage"));
    contourRef = contourStorage->debugFind(contourID);
    if (!contourRef)
        return; // OpDebugExit(__func__ + std::string(": !contourRef"));
    if (contourRef->id != contourID)
        OpDebugExit(__func__ + std::string(": contourRef->id [") + STR(contourRef->id)
                + "] != contourID [" + STR(contourID) + "]");
}

void OpContext::dumpResolve(OpEdge*& edgeRef) {
    int edgeID = (int) (size_t) edgeRef;
    if (0 == edgeID)
        return;
    for (auto c : contours) {
        for (auto& seg : c->segments) {
            for (auto& edge : seg.edges) {
                if (edge.id == edgeID) {
                    OP_ASSERT((int) (size_t) edgeRef == edgeID);
                    edgeRef = &edge;
                }
            }
        }
    }
    if (fillerStorage) {
        if (OpEdge* fillEdge = fillerStorage->debugFind(edgeID)) {
            OP_ASSERT((int) (size_t) edgeRef == edgeID);
            edgeRef = fillEdge;
        }
    }
    // if edge intersect is active, search there too
    if (ccStorage) {
        if (OpEdge* ccEdge = ccStorage->debugFind(edgeID)) {
            OP_ASSERT((int) (size_t) edgeRef == edgeID);
            edgeRef = ccEdge;
        }
    }
    if ((int) (size_t) edgeRef == edgeID)
        edgeRef = nullptr;  // !!! can happen in dump resolve?
}

void OpContext::dumpResolve(const OpEdge*& edgeRef) {
    dumpResolve(const_cast<OpEdge*&>(edgeRef));
}

void OpContext::dumpResolve(OpLimb*& limbRef) {
    int limbID = (int) (size_t) limbRef;
    if (0 == limbID)
        return;
    OpLimb* limb = limbStorage->debugFind(limbID);
    OP_ASSERT(limb);
    limbRef = limb;
}

void OpContext::dumpResolve(const OpLimb*& limbRef) {
    dumpResolve(const_cast<OpLimb*&>(limbRef));
}

void OpContext::dumpResolve(OpIntersection*& sectRef) {
    int sectID = (int) (size_t) sectRef;
    if (0 == sectID)
        return;
    OpIntersection* sect = sectStorage->debugFind(sectID);
    OP_ASSERT(sect);
    sectRef = sect;
}

void OpContext::dumpResolve(OpSegment*& segRef) {
    int segID = (int) (size_t) segRef;
    if (0 == segID)
        return;
    for (auto c : contours) {
        for (auto& seg : c->segments) {
            if (segID == seg.id) {
                OP_ASSERT((int) (size_t) segRef == segID);
                segRef = &seg;
            }
        }
    }
    if ((int) (size_t) segRef == segID)
        segRef = nullptr;  // !!! can happen in dump resolve?
}

#if OP_DEBUG_GLOBALS

void dmp(int id) {
    std::string s = DebugDump(id, defaultLevel, defaultBase);
    OpDebugFormat(s);
}

void dmpHex(int id) {
    std::string s = DebugDump(id, defaultLevel, DebugBase::hex);
    OpDebugFormat(s);
}

void dmpDisabled() {
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.disabled)
                    edge.dump();
            }
        }
    }
}

void dmpInOutput() {
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.inOutput)
                    edge.dump(DebugLevel::detailed, defaultBase);
            }
        }
    }
}

void dmpIntersections() {
    std::string s;
    for (const auto& c : contourIterator) {
        for (const auto& seg : c->segments) {
            s += seg.debugDumpIntersections() + "\n";
        }
    }
    OpDebugFormat(s);
}

#endif

std::string debugDmpJoin(OpContext* context, DebugLevel l, DebugBase b) {
	std::string s;
	if (!context->debugJoiner)
        return "(no debug joiner in context)\n";
	s += context->debugJoiner->debugDump(l, b) + "\n";
    for (const auto c : context->contours) {
		s += c->debugDumpJoin(l, b);
	}
    return s;
}

#if OP_DEBUG_GLOBALS

void dmpJoin() {
    std::string s = debugDmpJoin(debugGlobalContext, defaultLevel, defaultBase);
    OpDebugFormat(s);
}

void dmpTree() {
	std::string s;
	if (debugGlobalContext->debugTree) {
		dmp(debugGlobalContext->debugTree);
		s = "\n";
	}
    OpDebugFormat(s);
}

void dmpSects() {
    dmpIntersections();
}

void dmpSegments() {
	std::string s;
    for (const auto& c : contourIterator) {
        for (const auto& seg : c->segments) {
            s += seg.debugDump(defaultLevel, defaultBase) + "\n";
        }
    }
    OpDebugFormat(s);
}

void dmpSorted() {
	std::string s = "sorted[";
    for (const auto& c : debugGlobalContext->sortedContours) {
		s += STR(c->id) + " ";
	}
    debugPopMatching(s, ' ');
	OpDebugFormat(s + "]\n");
}

void dmpUnsectable() {
    for (const auto& c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isUnsectable())
                    edge.dump(DebugLevel::detailed, defaultBase);
            }
        }
    }
}

void dmpUnsortable() {
    for (const auto& c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (Unsortable::none != edge.isUnsortable)
                    edge.dump(DebugLevel::detailed, defaultBase);
            }
        }
    }
}

void dmpWindings() {
    std::string s;
    for (const auto& c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                s += "edge[" + STR(edge.id) + "] ";
                s += edge.debugDumpWinding() + "\n";
            }
        }
    }
    OpDebugOut(s);
}

#endif

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { OpDebugExpect::w, #w }
#define OpDebugExpect_Base
ENUM_NAME_STRUCT(OpDebugExpect)

namespace PathOpsV0Lib {

ContextError contextErrorStr(const char*& str, const char* label, ContextError enumDefault) {
    if (!OpDebugOptional(str, label))
        return enumDefault;
    size_t strLen = 0;
    while (isalnum(str[strLen]))
        ++strLen;
    for (size_t index = 0; index < contextErrorNames.size(); ++index) {
        size_t nameLen = contextErrorNames[index].name.size();
        if (strLen == nameLen && !strncmp(str, contextErrorNames[index].name.c_str(), nameLen)) {
            str += contextErrorNames[index].name.size();
            if (' ' == str[0]) ++str;
            return contextErrorNames[index].element;
        }
    }
    OpDebugExitOnFail("missing enum", false);
    return (ContextError) -1;
}

}

static void debugCallbacksDumpSet(std::vector<PathOpsV0Lib::DebugCurveCallbacks>& debugCallbacks, 
        const char*& str) {
    if (OpDebugOptional(str, "debugCallbacks")) {
        size_t size = OpDebugReadSizeT(str);
        debugCallbacks.resize(size);
        for (auto& debugCallback : debugCallbacks) {
            static_assert(0 == offsetof(PathOpsV0Lib::DebugCurveCallbacks, scaleFuncPtr));
            debugCallback.scaleFuncPtr = (PathOpsV0Lib::DebugScale) debugFindFunction(str);
	        DEBUG_FIND_FUNCTION(debugCallback, scaleFuncPtr,      curveNameFuncPtr);
	        DEBUG_FIND_FUNCTION(debugCallback, curveNameFuncPtr, curveExtraFuncPtr);
            DEBUG_FIND_FUNCTION(debugCallback, curveExtraFuncPtr, debugSubDivideFuncPtr);
#if 0 && OP_TEST_RASTER
            DEBUG_FIND_FUNCTION(debugCallback, debugSubDivideFuncPtr, addRasterFuncPtr);
            static_assert(offsetof(PathOpsV0Lib::DebugCurveCallbacks, addRasterFuncPtr) 
                    + sizeof(debugCallback.addRasterFuncPtr) == sizeof(debugCallback));
#else
            static_assert(offsetof(PathOpsV0Lib::DebugCurveCallbacks, debugSubDivideFuncPtr) 
                    + sizeof(debugCallback.debugSubDivideFuncPtr) == sizeof(debugCallback));
#endif
        }
    }
}

static void debugContextCallbacksDumpSet(PathOpsV0Lib::DebugContextCallbacks& debugContextCallbacks, 
        const char*& str) {
    OpDebugRequired(str, "debugContextCallbacks");
    static_assert(0 == offsetof(PathOpsV0Lib::DebugContextCallbacks, debugIsFillFuncPtr));
    debugContextCallbacks.debugIsFillFuncPtr = (PathOpsV0Lib::DebugIsFill) debugFindFunction(str);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugIsFillFuncPtr, debugMergeEndsFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugMergeEndsFuncPtr, debugMergeFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugMergeFuncPtr, debugDumpWindingOutFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpWindingOutFuncPtr, debugDumpWindingSetFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpWindingSetFuncPtr, debugDumpOutFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpOutFuncPtr, debugImageWindingOutFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugImageWindingOutFuncPtr, debugImageWindingNamesFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugImageWindingNamesFuncPtr, debugWindingVisibleFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugWindingVisibleFuncPtr, debugSafetyLinksFuncPtr);
#if OP_DEBUGGER
    OpDebugOptional(str, "debugEdgeColorFuncName");
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugSafetyLinksFuncPtr, debugEdgeColorFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugEdgeColorFuncPtr) 
            + sizeof(debugContextCallbacks.debugEdgeColorFuncPtr) == sizeof(debugContextCallbacks));
#else
    ASSERT_SERIAL(debugContextCallbacks, debugSafetyLinksFuncPtr, debugEdgeColorFuncName);
    if (OpDebugOptional(str, "debugEdgeColorFuncName"))
        debugContextCallbacks.debugEdgeColorFuncName = OpDebugLabel(str);
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugEdgeColorFuncName) 
            + sizeof(debugContextCallbacks.debugEdgeColorFuncName) == sizeof(debugContextCallbacks));
#endif
}

void OpContext::dumpSet(const char*& str) {
    static_assert(0 == offsetof(OpContext, callbacks));
    OpDebugRequired(str, "callbacks");
    size_t size = OpDebugReadSizeT(str);
    callbacks.resize(size);
    for (auto& callback : callbacks) {
        static_assert(0 == offsetof(PathOpsV0Lib::CurveCallbacks, axisTFuncPtr));
	    callback.axisTFuncPtr = (PathOpsV0Lib::AxisT) debugFindFunction(str);
	    DEBUG_FIND_FUNCTION(callback, axisTFuncPtr,          rotateTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, rotateTFuncPtr,        curveHullFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveHullFuncPtr,      curveIsFiniteFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveIsFiniteFuncPtr,  curveIsLineFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveIsLineFuncPtr,    setBoundsFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, setBoundsFuncPtr,      curvePinFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curvePinFuncPtr,       curveTangentFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveTangentFuncPtr,   curvesEqualFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curvesEqualFuncPtr,    ptAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptAtTFuncPtr,          ptDAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptDAtTFuncPtr,         ptCountFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptCountFuncPtr,        rotateFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, rotateFuncPtr,         subDivideFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, subDivideFuncPtr,      xyAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, xyAtTFuncPtr,          curveReverseFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveReverseFuncPtr,   cutFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, cutFuncPtr,            interceptFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, interceptFuncPtr,      normalLimitFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, normalLimitFuncPtr,    maxAlternateEndFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, maxAlternateEndFuncPtr, smallTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, smallTFuncPtr,         maxCutFuncPtr);
        static_assert(offsetof(PathOpsV0Lib::CurveCallbacks, maxCutFuncPtr) 
                + sizeof(callback.maxCutFuncPtr) == sizeof(callback));
    }
    ASSERT_ORDERED(callbacks, userData);
#if 0  // don't serialize user data
    OpDebugRequired(str, "userData");
    size = OpDebugReadSizeT(str);
    userData.resize(size);
    for (PathOpsV0Lib::ContextUserData& data : userData) {
        OpDebugRequired(str, "offset");
        data.data = (void*) OpDebugReadSizeT(str);
        OpDebugRequired(str, "size");
        data.size = OpDebugReadSizeT(str);
        OpDebugRequired(str, "type");
        data.type = (PathOpsV0Lib::UserDataType) OpDebugReadSizeT(str);
    }
#endif
    ASSERT_ORDERED(userData, nativeCurveTypes);
    OpDebugRequired(str, "nativeCurveTypes");
    size = OpDebugReadSizeT(str);
    nativeCurveTypes.resize(size);
    for (int& nativeCurveType : nativeCurveTypes) {
       nativeCurveType = OpDebugReadSizeT(str);
    }
    ASSERT_ORDERED(nativeCurveTypes, contextCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::ContextCallbacks, curveOutputFuncPtr));
#if 0  // omit curveOutputFuncPtr
	contextCallbacks.curveOutputFuncPtr = (PathOpsV0Lib::CurveOutput) debugFindFunction(str);
#endif
    DEBUG_FIND_FUNCTION(contextCallbacks, curveOutputFuncPtr, emptyCallerPathFuncPtr);
#if 0  // skip find function for best loop func
	DEBUG_FIND_FUNCTION(contextCallbacks, emptyCallerPathFuncPtr, bestLoopFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, bestLoopFuncPtr, setLineTypeFuncPtr);
#else
	ASSERT_SERIAL(contextCallbacks, emptyCallerPathFuncPtr, bestLoopFuncPtr);
	ASSERT_SERIAL(contextCallbacks, bestLoopFuncPtr, setLineTypeFuncPtr);
#endif
	DEBUG_FIND_FUNCTION(contextCallbacks, setLineTypeFuncPtr, maxAngleMatchFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxAngleMatchFuncPtr, maxAngleSweepFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxAngleSweepFuncPtr, maxSplitFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxSplitFuncPtr, maxBoundedEdgeFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxBoundedEdgeFuncPtr, maxSignSwapFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxSignSwapFuncPtr, maxTSlopFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxTSlopFuncPtr, maxSplitBiasFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxSplitBiasFuncPtr, maxOverlapFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxOverlapFuncPtr, maxUnsectableFuncPtr);
    DEBUG_FIND_FUNCTION(contextCallbacks, maxUnsectableFuncPtr, maxDistFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxDistFuncPtr, maxDeepFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxDeepFuncPtr, maxShallowFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxShallowFuncPtr, maxSplitsFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxSplitsFuncPtr, maxMarginFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxMarginFuncPtr, rootAdjustFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, rootAdjustFuncPtr, hullNudgeFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, hullNudgeFuncPtr, maxUnsectableTFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxUnsectableTFuncPtr, maxUnsectDistFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxUnsectDistFuncPtr, maxCheckSplitFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxCheckSplitFuncPtr, maxLimbsFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxLimbsFuncPtr, maxLoopsFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxLoopsFuncPtr, windingBytesFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, windingBytesFuncPtr, maxGapFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxGapFuncPtr, linkupScaleFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, linkupScaleFuncPtr, enabledRatioFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::ContextCallbacks, enabledRatioFuncPtr) 
            + sizeof(contextCallbacks.enabledRatioFuncPtr) == sizeof(contextCallbacks));
    static_assert(0 == offsetof(PathOpsV0Lib::WindingCallbacks, windingAddFuncPtr));
	windingCallbacks.windingAddFuncPtr = (PathOpsV0Lib::WindingAdd) debugFindFunction(str);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingAddFuncPtr, windingKeepFuncPtr);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingKeepFuncPtr, windingSubtractFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingSubtractFuncPtr, windingWoundFuncPtr);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingWoundFuncPtr, windingVisibleFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingVisibleFuncPtr, windingZeroFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingZeroFuncPtr, windingIntersectFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingIntersectFuncPtr, windingShortFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingShortFuncPtr, windingShortAllFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingShortAllFuncPtr, windingLoopFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::WindingCallbacks, windingLoopFuncPtr) 
            + sizeof(windingCallbacks.windingLoopFuncPtr) == sizeof(windingCallbacks));
    // out of order ... but callbacks must be set before curves and windings are read
    debugCallbacksDumpSet(debugCallbacks, str);
    debugContextCallbacksDumpSet(debugContextCallbacks, str);
    ASSERT_ORDERED(windingCallbacks, errorHandler);  // omit errorHandler
    ASSERT_ORDERED(errorHandler, sortedContours);
    if (OpDebugOptional(str, "sortedContours")) {
        size = OpDebugReadSizeT(str);
        sortedContours.resize(size);
        for (auto& sortedContour : sortedContours) {
            sortedContour = (OpContour*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_ORDERED(sortedContours, curveDataStorage);
    if (OpDebugOptional(str, "curveDataStorage"))
        CurveDataStorage::DumpSet(str, &curveDataStorage);
    ASSERT_ORDERED(curveDataStorage, ccStorage);
    if (OpDebugOptional(str, "ccStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::cc);
    ASSERT_ORDERED(ccStorage, contourStorage);
    if (OpDebugOptional(str, "contourStorage"))
        OpContourStorage::DumpSet(str, this);
    ASSERT_ORDERED(contourStorage, contours);
    if (OpDebugOptional(str, "contours")) {
        size = OpDebugReadSizeT(str);
        contours.resize(size);
        for (auto& contour : contours) {
            contour = (OpContour*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_ORDERED(contours, fillerStorage);
    if (OpDebugOptional(str, "fillerStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::filler);
    ASSERT_ORDERED(fillerStorage, sectStorage);
    if (OpDebugOptional(str, "sectStorage"))
        OpSectStorage::DumpSet(str, this);
    ASSERT_ORDERED(sectStorage, limbStorage);
    if (OpDebugOptional(str, "limbStorage"))
        OpLimbStorage::DumpSet(str, this);
    ASSERT_ORDERED(limbStorage, limbCurrent);  // omit limbCurrent
    ASSERT_ORDERED(limbCurrent, callerStorage);
    if (OpDebugOptional(str, "callerStorage"))
        CallerDataStorage::DumpSet(str, &callerStorage);
    ASSERT_ORDERED(callerStorage, maxBounds);
    if (OpDebugOptional(str, "maxBounds"))
        maxBounds.dumpSet(str);
    ASSERT_ORDERED(maxBounds, threshold);
    if (OpDebugOptional(str, "threshold"))
        threshold.dumpSet(str);
    ASSERT_ORDERED(threshold, thresholdLength);
    thresholdLength = OpDebugReadNamedFloat(str, "thresholdLength");
    ASSERT_ORDERED(thresholdLength, error);
    error = PathOpsV0Lib::contextErrorStr(str, "error", PathOpsV0Lib::ContextError::none);
    ASSERT_ORDERED(error, uniqueID);
    OpDebugRequired(str, "uniqueID");
    uniqueID = (int) OpDebugReadSizeT(str);
    DEBUG_SET_BOOL(uniqueID, initialized);
    DEBUG_SET_BOOL(initialized, allDiscarded);
    DEBUG_SET_BOOL(allDiscarded, allKept);
    DEBUG_SET_BOOL(allKept, fatalError);
    DEBUG_SET_BOOL(fatalError, outputOne);
    DEBUG_SET_BOOL(outputOne, linkErased);
    DEBUG_SET_BOOL(linkErased, windingSet);
#if OP_DEBUG_VALIDATE
    ASSERT_SERIAL_OFFSET(*this, windingSet, 1, debugValidateEdgeIndex);
    OpDebugRequired(str, "debugValidateEdgeIndex");
    debugValidateEdgeIndex = (int) OpDebugReadSizeT(str);
    ASSERT_ORDERED(debugValidateEdgeIndex, debugValidateJoinerIndex);
    OpDebugRequired(str, "debugValidateJoinerIndex");
    debugValidateJoinerIndex = (int) OpDebugReadSizeT(str);
    ASSERT_SERIAL_OFFSET(*this, debugValidateJoinerIndex, 4, debugCallbacks);
#else
    ASSERT_ORDERED(dumpDummy, debugCallbacks);
#endif
// debug call backs must be set before segments' curves can be set
    ASSERT_ORDERED(debugCallbacks, debugContextCallbacks);
    ASSERT_ORDERED(debugContextCallbacks, debugData);  // omit most of debugData (!!! omit for now, may have uses...)
    if (OpDebugOptional(str, "debugTestname"))
        debugData.testname = OpDebugLabel(str);
    ASSERT_ORDERED(debugData, debugCurveCurve);
    if (OpDebugOptional(str, "debugCurveCurve")) {
        if (!debugCurveCurve)
            debugCurveCurve = new OpCurveCurve(this);
        debugCurveCurve->dumpSet(str);
    }
    ASSERT_ORDERED(debugCurveCurve, debugJoiner);
    if (OpDebugOptional(str, "debugJoiner")) {
        if (!debugJoiner)
            debugJoiner = new OpJoiner(DumpSerialization::dummy, this);
        debugJoiner->dumpSet(str);
    }
    ASSERT_ORDERED(debugJoiner, debugTree);
    if (OpDebugOptional(str, "debugTree")) {
        if (!debugTree)
            debugTree = new OpTree(DumpSerialization::dummy, this);
        debugTree->dumpSet(str);
    }
    ASSERT_ORDERED(debugTree, debugErasures);
    if (OpDebugOptional(str, "debugErasures")) {
        size = OpDebugReadSizeT(str);
        debugErasures = &debugDumpErasures;
        debugDumpErasures.resize(size);
        for (auto& debugDumpErasure : debugDumpErasures) {
            debugDumpErasure = (OpEdge*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_ORDERED(debugErasures, debugErrorID);
    debugErrorID = OpDebugReadNamedInt(str, "debugErrorID");
    ASSERT_ORDERED(debugErrorID, debugOppErrorID);
    debugOppErrorID = OpDebugReadNamedInt(str, "debugOppErrorID");
    ASSERT_ORDERED(debugOppErrorID, debugExpect);
    debugExpect = OpDebugExpectStr(str, "debugExpect", OpDebugExpect::fail);
    ASSERT_ORDERED(debugExpect, debugInPathOps);
    debugInPathOps = OpDebugOptional(str, "debugInPathOps");
    ASSERT_ORDERED(debugInPathOps, debugInClearEdges);
    debugInClearEdges = OpDebugOptional(str, "debugInClearEdges");
    ASSERT_ORDERED(debugInClearEdges, debugCheckLastEdge);
    debugCheckLastEdge = OpDebugOptional(str, "debugCheckLastEdge");
    ASSERT_ORDERED(debugCheckLastEdge, debugFailOnEqualCepts);
    debugFailOnEqualCepts = OpDebugOptional(str, "debugFailOnEqualCepts");
    ASSERT_ORDERED(debugFailOnEqualCepts, debugFilename);
    ASSERT_ORDERED(debugFilename, debugDescription);
    if (OpDebugOptional(str, "debugDescription")) {
        const char* descStart = str;
        while (*str++ != '\n')
            ;
        debugDescription = std::string(descStart, str - descStart - 1);
    }
    ASSERT_ORDERED(debugDescription, debugOutPath);  // omit for now
    if (OpDebugOptional(str, "debugOutPath")) {
        while (str[0] < ' ')
            str++;
        const char* outPathStart = str;
        const char* outPathEnd;
        do {
            while (*str++ != '\n')
                ;
            outPathEnd = str;
        } while (!OpDebugOptional(str, ":debugOutPath"));
        debugOutPath = std::string(outPathStart, outPathEnd - outPathStart - 1);
    }
    ASSERT_ORDERED(debugOutPath, dumpIndex);  // omit for now
    ASSERT_ORDERED_OFFSET(dumpIndex, debugDumpErasures, 4);  // omit for now
    ASSERT_ORDERED(debugDumpErasures, debugDumpInit);  // omit for now
#if OP_TEST_RASTER
    // don't dump raster storage for now
    ASSERT_ORDERED(rasterStorage, debugRaster);
    if (OpDebugOptional(str, "debugRaster")) {
        if (!debugRaster)
            debugRaster = new DebugRaster(this);
        debugRaster->dumpSet(str);
    }

#endif
	debugDumpInit = true;
}

void OpContext::dumpResolveAll(OpContext* self) {
    OP_ASSERT(this == self);
    for (PathOpsV0Lib::ContextUserData& data : userData) {
       callerStorage->dumpResolve(data); 
    }
    if (dumpInitialized()) {
        for (auto& sortedContour : sortedContours) {
            self->dumpResolve(sortedContour);
        }
    }
    for (auto& contour : contours) {    // out of order: resolve contour array
        self->dumpResolve(contour);     //  before contents of contour storage
    }
    if (ccStorage)
        ccStorage->dumpResolveAll(self);
    if (contourStorage)
        contourStorage->dumpResolveAll(self);
    if (dumpInitialized() && fillerStorage)
        fillerStorage->dumpResolveAll(self);
    if (dumpInitialized() && sectStorage)
        sectStorage->dumpResolveAll(self);
    if (dumpInitialized() && limbStorage)
        limbStorage->dumpResolveAll(self);
#if OP_DEBUG
    if (debugCurveCurve)
        debugCurveCurve->dumpResolveAll(self);
    if (debugJoiner)
        debugJoiner->dumpResolveAll(self);
    if (debugTree)
        debugTree->dumpResolveAll(self);
    for (OpEdge*& edge : debugDumpErasures) {
        self->dumpResolve(edge);
    }
    if (debugRaster)
        debugRaster->dumpResolveAll(self);
#endif
}

#if OP_DEBUG_GLOBALS

void dmpContext() {
    dmp(*debugGlobalContext);
}

void dmpContours() {
	std::string s;
	for (OpContour* contour : contourIterator) {
		s += contour->debugDump(defaultLevel, defaultBase) + "\n";
	}
	debugPopMatching(s, '\n');
	OpDebugFormat(s);
}


// !!! when the need arises, rewrite this in terms of context so that visual debugger
//     can generate this without requiring globals (only use of most iterators)
void dmpMatch(const OpPoint& pt, bool detail) {
    DebugLevel level = detail ? DebugLevel::detailed : defaultLevel;
	for (auto segment : segmentIterator) {
        OpSegment& seg = *segment;
        if (pt == seg.c.firstPt() || pt == seg.c.lastPt())
            OpDebugFormat("seg: " + seg.debugDump(level, defaultBase) + "\n");
    }
    for (auto sect : intersectionIterator) {
        if (sect->ptT.pt == pt)
            OpDebugFormat("sect: " +  sect->debugDump(level, defaultBase) + "\n");
    }
    for (auto edgePtr : edgeIterator) {
    	OpEdge& edge = *edgePtr;
        if (edge.curve.firstPt() == pt)
            OpDebugFormat("edge start: " + edge.debugDump(level, defaultBase) + "\n");
        if (edge.curve.lastPt() == pt)
            OpDebugFormat("edge end: " + edge.debugDump(level, defaultBase) + "\n");
        if (edge.curve.firstPt() != edge.startPt() && edge.curve.firstPt() == pt)
            OpDebugFormat("edge curve.firstPt(): " + edge.debugDump(level, defaultBase) + "\n");
        if (edge.curve.lastPt() != edge.endPt() && edge.curve.lastPt() == pt)
            OpDebugFormat("edge curve.lastPt(): " + edge.debugDump(level, defaultBase) + "\n");
    }
}

void dmpMatch(const OpPoint& pt) {
    dmpMatch(pt, false);
}

void dmpMatch(const OpPtT& ptT) {
    dmpMatch(ptT.pt, false);
}

void dmpMatchStart(int id) {
	if (OpEdge* edge = findEdge(id))
		return dmpMatch(edge->curve.firstPt());
	if (const OpSegment* seg = findSegment(id))
		return dmpMatch(seg->c.firstPt());
}

void dmpMatchEnd(int id) {
	if (OpEdge* edge = findEdge(id))
		return dmpMatch(edge->curve.lastPt());
	if (const OpSegment* seg = findSegment(id))
		return dmpMatch(seg->c.lastPt());
}

#endif

static std::string getline(const char*& str) {
    if ('}' == str[0]) {
        ++str;
        OP_ASSERT(';' == *str++);
        if ('\r' == str[0])
            ++str;
        OP_ASSERT('\n' == *str++);
        if ('\r' == str[0])
            ++str;
        OP_ASSERT('\n' == *str++);
    }
    const char* structCheck = "OpDebug";
    if (!strncmp(structCheck, str, sizeof(structCheck) - 1)) {
        str = strchr(str, '\n');
        OP_ASSERT(str);
        str += 1;
    }
    OP_ASSERT(!strncmp("// ", str, 3));
    const char* start = strchr(str, '\n');
    OP_ASSERT(start);
    start += 1;
    OP_ASSERT('{' == start[0]);
    const char* end = strchr(start, '\n');
    OP_ASSERT(end);
    str = end + 1;
    if ('\r' == end[-1])
        --end;
    OP_ASSERT(',' == end[-1]);
    OP_ASSERT('/' == str[0] || '}' == str[0]);
    std::string line = std::string(start, end - start - 1);
    return line;
}

void OpContext::debugCompare(std::string s) {
    const char* str = s.c_str();
    for (const auto c : contours) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                std::string line = getline(str);
                intersection->debugCompare(line);
            }
            for (const auto& edge : seg.edges) {
                std::string line = getline(str);
                edge.debugCompare(line);
            }
        }
    }
}

enum class ShowContour {
	no,
	yes
};

void Curve_DumpSet(PathOpsV0Lib::Curve& c, const char*& str) {
    OpContext& context = *(OpContext*) c.context;
    size_t strLen = 0;
    while (isalnum(str[strLen]))
        ++strLen;
    for (size_t index = 0; index < context.callbacks.size(); ++index) {
		auto curveName = context.debugCallbacks[index].curveNameFuncPtr;
		if (!curveName)
			continue;
        std::string name = (*curveName)();
        if (name.size() != strLen || strncmp(str, name.c_str(), strLen))
			continue;
        str += strLen;
        if (' ' == str[0])
            ++str;
        c.type = (PathOpsV0Lib::CurveType) index;
    }
    OpDebugRequired(str, "size");
    c.size = OpDebugReadSizeT(str);
    OpDebugRequired(str, "data");
    c.data = context.curveDataStorage->dumpSet(str);  // do not allocate, just point to
}

static void dumpEdges(const char*& str, const char* arrayName, std::vector<OpEdge*>& edgeArray) {
    if (!OpDebugOptional(str, arrayName))
        return;
    int count = (int) OpDebugReadSizeT(str);
    edgeArray.resize(count);
    for (auto& edge : edgeArray)
        edge = (OpEdge*) OpDebugReadSizeT(str);
}

#define DUMP_EDGES(instance, lastField, edgePtrArray) \
    ASSERT_SERIAL(instance, lastField, edgePtrArray); \
    ::dumpEdges(str, #edgePtrArray, edgePtrArray)

#define DUMP_NAMED_EDGES(instance, lastField, arrayName, edgePtrArray) \
    ASSERT_SERIAL(instance, lastField, edgePtrArray); \
    ::dumpEdges(str, arrayName, edgePtrArray)

void DebugCurveData_DumpSet(PathOpsV0Lib::DebugCurveData& dcd, const char*& str) {
    OpDebugRequired(str, "data");
    dcd.data = (PathOpsV0Lib::DebugCurve*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "size");
    dcd.size = OpDebugReadSizeT(str);
}

void OpContour::dumpSet(const char*& str) {
    OpDebugRequired(str, "contour");
    OP_DEBUG_CODE(id = (int) OpDebugReadSizeT(str));
    static_assert(0 == offsetof(OpContour, segments));
    OpDebugRequired(str, "segments");
    int segmentCount = (int) OpDebugReadSizeT(str);
    segments.resize(segmentCount);
    for (int index = 0; index < segmentCount; ++index)
        segments[index].contour = this;
    for (int index = 0; index < segmentCount; ++index)
        segments[index].dumpSet(str);
    ASSERT_ORDERED(segments, sorted);
    if (OpDebugOptional(str, "sorted")) {
        int count = (int) OpDebugReadSizeT(str);
        sorted.resize(count);
        for (OpSegment*& seg : sorted)
            seg = (OpSegment*) OpDebugReadSizeT(str);
    }
    ASSERT_ORDERED(sorted, overlaps);
    if (OpDebugOptional(str, "overlaps")) {
        int count = (int) OpDebugReadSizeT(str);
        overlaps.resize(count);
        for (int index = 0; index < count; ++index)
            overlaps[index] = (OpContour*) OpDebugReadSizeT(str);
    }
    ASSERT_ORDERED(overlaps, merges);
    if (OpDebugOptional(str, "merges")) {
        int count = (int) OpDebugReadSizeT(str);
        merges.resize(count);
        for (int index = 0; index < count; ++index)
            merges[index] = (OpContour*) OpDebugReadSizeT(str);
    }
	DUMP_EDGES(*this, merges, inX);
	DUMP_EDGES(*this, inX, inY);
	DUMP_EDGES(*this, inY, byArea);
	DUMP_EDGES(*this, byArea, unsectByArea);
	DUMP_EDGES(*this, unsectByArea, disabledBackwards);
	DUMP_EDGES(*this, disabledBackwards, disabledCenterless);
	DUMP_EDGES(*this, disabledCenterless, disabledPals);
	DUMP_EDGES(*this, disabledPals, smallEdges);
	DUMP_EDGES(*this, smallEdges, unsortables);
    ASSERT_ORDERED(unsortables, windingStorage);
    OpDebugRequired(str, "windingStorage");
    size_t windingSize = OpDebugReadSizeT(str);
    windingStorage.resize(windingSize);
    OpDebugByteArray(str, windingSize, &windingStorage.front());
    DUMP_NAMED_EDGES(*this, windingStorage, "linkups", linkups.l);
    DUMP_NAMED_EDGES(*this, linkups, "endLinks", endLinks.l);
    ASSERT_ORDERED(endLinks, overlapBounds);
    if (OpDebugOptional(str, "overlapBounds"))
        overlapBounds.dumpSet(str);
    ASSERT_ORDERED(overlapBounds, bounds);
    if (OpDebugOptional(str, "bounds"))
        bounds.dumpSet(str);
    ASSERT_ORDERED(bounds, context);  // omit context
    ASSERT_ORDERED(context, overlapOwner);
    if (OpDebugOptional(str, "overlapOwner"))
        overlapOwner = (OpContour*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "contextIndex"); 
    contextIndex = OpDebugReadSizeT(str);
    ASSERT_ORDERED(contextIndex, id);  // id written up front
    ASSERT_ORDERED(id, treeID);
    if (OpDebugOptional(str, "treeID"))
        treeID = OpDebugReadSizeT(str);
    DEBUG_SET_BOOL(treeID, backwardsBuilt);
    DEBUG_SET_BOOL(backwardsBuilt, centerlessBuilt);
    DEBUG_SET_BOOL(centerlessBuilt, hasPals);
    DEBUG_SET_BOOL(hasPals, palsBuilt);
    DEBUG_SET_BOOL(palsBuilt, disabled);
    DEBUG_SET_BOOL(disabled, overlapsMerged);
    DEBUG_SET_BOOL(overlapsMerged, segEndsMerged);
    DEBUG_SET_BOOL(segEndsMerged, segMerged);
    DEBUG_SET_BOOL(segMerged, debugEmpty);
#if OP_DEBUGGER || OP_TEST
    ASSERT_SERIAL_OFFSET(*this, debugEmpty, 3, debugCurveData);
    if (OpDebugOptional(str, "debugCurveData")) {
        debugCurveData.resize(OpDebugReadSizeT(str));
        for (PathOpsV0Lib::DebugCurveData& curveData : debugCurveData) {
            DebugCurveData_DumpSet(curveData, str);
        }
    }
    ASSERT_ORDERED(debugCurveData, debugWinding);
#endif
}

#undef DUMP_EDGES
#undef DUMP_NAMED_EDGES

void DebugCurveData_DumpResolve(OpContext* context, PathOpsV0Lib::DebugCurveData& dcd) {
    CurveDataStorage* storage = context->curveDataStorage;
    OP_ASSERT(storage);
    size_t offset = (size_t) dcd.data;
    while (offset >= sizeof(storage->storage)) {
        OP_ASSERT(storage->next);
        offset -= sizeof(storage->storage);
        storage = storage->next;
    }
    OP_ASSERT(dcd.size);
    OP_ASSERT(offset + dcd.size <= storage->used);
    dcd.data = (PathOpsV0Lib::DebugCurve*) &storage->storage[offset];
}

#define DUMP_RESOLVE_ARRAY(obj) \
    for (auto& o : obj) \
        c->dumpResolve(o)

void OpContour::dumpResolveAll(OpContext* c) {
    for (OpSegment& segment : segments)
        segment.dumpResolveAll(c);
    if (!c->dumpInitialized())
        return;
    DUMP_RESOLVE_ARRAY(sorted);
    DUMP_RESOLVE_ARRAY(overlaps);
    DUMP_RESOLVE_ARRAY(merges);
	DUMP_RESOLVE_ARRAY(inX);
	DUMP_RESOLVE_ARRAY(inY);
	DUMP_RESOLVE_ARRAY(byArea);
	DUMP_RESOLVE_ARRAY(unsectByArea);
	DUMP_RESOLVE_ARRAY(disabledBackwards);
	DUMP_RESOLVE_ARRAY(disabledCenterless);
	DUMP_RESOLVE_ARRAY(disabledPals);
	DUMP_RESOLVE_ARRAY(smallEdges);
	DUMP_RESOLVE_ARRAY(unsortables);
	DUMP_RESOLVE_ARRAY(linkups.l);
	DUMP_RESOLVE_ARRAY(endLinks.l);
    if (overlapOwner)
        c->dumpResolve(overlapOwner);
#if OP_DEBUG_IMAGE
    for (PathOpsV0Lib::DebugCurveData& dcd : debugCurveData) {
        DebugCurveData_DumpResolve(context, dcd);
    }
#endif
}

#undef DUMP_RESOLVE_ARRAY

void dmp(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dump();
}

void OpContourStorage::debugCheck(const OpContour* contour) {
	for (int index = 0; index < used; index++) {
		const OpContour& test = storage[index];
        if (&test == contour)
            return;
	}
    if (next)
        return next->debugCheck(contour);
    OpDebugExit(__func__ + std::string("missing contour"));
}

OpContour* OpContourStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index++) {
		const OpContour& test = storage[index];
        if (test.id == ID)
            return const_cast<OpContour*>(&test);
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

void OpContourStorage::DumpSet(const char*& str, OpContext* dumpContext) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpContour* sect = dumpContext->allocateContour();
        sect->context = dumpContext;
        sect->dumpSet(str);
    }
}

void OpContourStorage::dumpResolveAll(OpContext* c) {
    int count = debugCount();
    for (int index = 0; index < count; ++index) {
        debugIndex(index)->dumpResolveAll(c);
    }
}

// finds pointer to caller data stored in contours; string points to byte offset
PathOpsV0Lib::CurveData* CurveDataStorage::dumpSet(const char*& str) {
    size_t offset = OpDebugReadSizeT(str);
    CurveDataStorage* test = this;
    while (offset >= test->used) {
        offset -= test->used;
        test = test->next;
        OP_ASSERT(test);
    }
    PathOpsV0Lib::CurveData* result = (PathOpsV0Lib::CurveData*) (test->storage + offset);
    return result;
}

// sets caller data in contours from string encoded bytes
void CurveDataStorage::DumpSet(const char*& str, CurveDataStorage** previousPtr) {
    CurveDataStorage* storage = new CurveDataStorage;
    *previousPtr = storage;
    storage->next = (CurveDataStorage*) OpDebugOptional(str, "next");  // non-zero means there is more
    OpDebugRequired(str, "used");
    storage->used = OpDebugReadSizeT(str);
    OpDebugByteArray(str, storage->used, storage->storage);
    if (storage->next)
        DumpSet(str, &storage->next);
}

void CutRangeT::dumpSet(const char*& str) {
    DEBUG_SET_FIRST_STRUCT(lo);
    DEBUG_SET_LAST_STRUCT(lo, hi);
}

void dmp(const PathOpsV0Lib::AddCurve& c) {
	OpCurve curve(c, Rotated::debug);
	OpDebugFormat(curve.debugDump(defaultLevel, defaultBase));
}

void dmp(const PathOpsV0Lib::AddCurve* c) {
	dmp(*c);
}

void dmp(const PathOpsV0Lib::Curve& c) {
	OpCurve curve(c, Rotated::debug);
	OpDebugFormat(curve.debugDump(defaultLevel, defaultBase));
}

void dmp(const PathOpsV0Lib::Curve* c) {
	dmp(*c);
}

bool debugDmpIsLine(const PathOpsV0Lib::AddCurve& c) {
	OpCurve test(c, Rotated::debug);
	return test.debugIsLine();
}

bool debugDmpIsLine(const PathOpsV0Lib::Curve& c) {
	OpCurve test(c, Rotated::debug);
	return test.debugIsLine();
}

#undef OP_ENUM_BASE
#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { Rotated::w, #w }
#define Rotated_Base
ENUM_NAME_STRUCT(Rotated)

void OpCurve::dumpSet(const char*& str) {
    Curve_DumpSet(c, str);
    if (OpDebugOptional(str, "start"))
        start.dumpSet(str);
    else
        start = c.data->start;
    if (OpDebugOptional(str, "end"))
        end.dumpSet(str);
    else
        end = c.data->end;
    rotated = RotatedStr(str, "rotated", Rotated::no);
    isLineSet = OpDebugOptional(str, "isLineSet");
    isLineResult = OpDebugOptional(str, "isLineResult");
    isSmall = OpDebugOptional(str, "isSmall");
    reversed = OpDebugOptional(str, "reversed");
}

#undef OP_ENUM_BASE
#define OP_ENUM_BASE(w, val) { EdgeMatch::w, #w },
#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { EdgeMatch::w, #w }
ENUM_NAME_STRUCT(EdgeMatch)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { EdgeFail::w, #w }
#define EdgeFail_Base
ENUM_NAME_STRUCT(EdgeFail)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { WindZero::w, #w }
#define WindZero_Base
ENUM_NAME_STRUCT(WindZero)

#undef OP_ENUM_BASE
#define OP_ENUM_BASE(w, val) { Axis::w, #w },
#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { Axis::w, #w }
ENUM_NAME_STRUCT(Axis)

EdgeFilterName filterNames[] = {
#define OP_X(s) \
    { EdgeFilter::s, #s },
EDGE_FILTER
#undef OP_X
#define OP_X(s) \
    { EdgeFilter::s, #s },
EDGE_VIRTUAL
#undef OP_X
#if OP_DEBUG
#define OP_X(s) \
    { EdgeFilter::debug##s, "debug" #s },
EDGE_DEBUG
#undef OP_X
#endif
#if OP_DEBUG_DUMP
#define OP_X(s) \
    { EdgeFilter::debug##s, "debug" #s },
EDGE_DUMP
#undef OP_X
#endif
};

OpSaveEF::OpSaveEF(std::vector<EdgeFilter>& temp) {
    save = edgeFilters[(int) defaultLevel].filter;
    for (EF ef = (EF) 0; ef < EF::last; ef = (EF)((int) ef + 1)) {
        if (temp.end() == std::find(temp.begin(), temp.end(), ef))
            edgeFilters[(int) defaultLevel].filter.push_back(ef);
    }
}

OpSaveEF::~OpSaveEF() {
    edgeFilters[(int) defaultLevel].filter = save;
}

void addEdgeFilter(std::vector<EdgeFilter>& efSet, EdgeFilter ef) {
    if (efSet.end() != std::find(efSet.begin(), efSet.end(), ef))
        return;
    efSet.push_back(ef);
}

void addAlways(EdgeFilter ef) {
    addEdgeFilter(edgeFilters[(int) defaultLevel].always, ef);
}

void addFilter(EdgeFilter ef) {
    addEdgeFilter(edgeFilters[(int) defaultLevel].filter, ef);
}

void clearEdgeFilter(std::vector<EdgeFilter>& efSet, EdgeFilter ef) {
    auto efImpl = std::find(efSet.begin(), efSet.end(), ef);
    if (efSet.end() == efImpl)
        return;
    efSet.erase(efImpl);
}

void clearAlways(EdgeFilter ef) {
    clearEdgeFilter(edgeFilters[(int) defaultLevel].always, ef);
}

void clearFilter(EdgeFilter ef) {
    clearEdgeFilter(edgeFilters[(int) defaultLevel].filter, ef);
}

void dmpFilters() {
    auto dmpOne = [](std::vector<EdgeFilter>& efSet, std::string name) {
        std::string s = name + ": ";
        for (auto ef : efSet) {
            if ((size_t) ef < ARRAY_COUNT(filterNames))
                s += filterNames[(size_t) ef].name + std::string(", ");
            else 
                s += "(out of range) [" + STR_E(ef) + "] ";
        }
        debugPopMatching(s, ' ');
        if (debugIfMatching(s, ':'))
            return;
        OpDebugFormat(s);
    };
    for (int level = 0; level < 3; ++level) {
        OpDebugOut(!level ? "brief\n" : 1 == level ? "normal\n" : "detailed\n"); 
        dmpOne(edgeFilters[level].filter, "filter");
        dmpOne(edgeFilters[level].always, "always");
    }
}

void dmpEdgePts() {
    std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, EF::winding, 
            EF::sum, EF::whichEnd_impl };
    OpSaveEF saveEF(showFields);
    dmpEdges();
}

void dmpPts(int ID) {
    if (findEdge(ID)) {
        std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, EF::winding, 
                EF::sum, EF::whichEnd_impl };
        OpSaveEF saveEF(showFields);
        ::dmp(ID);
        return;
    }
    const OpSegment* seg = findSegment(ID);
    if (seg) {
        std::string s = seg->debugDump(DebugLevel::brief, defaultBase);
        OpDebugFormat(s + "\n");
        return;
    }
}

void dmpPts(const OpEdge* e) {
    dmpPts(e->id);
}

void dmpPts(const OpEdge& e) {
    dmpPts(&e);
}

void dmpPts(const OpSegment* e) {
    dmpPts(e->id);
}

void dmpPts(const OpSegment& e) {
    dmpPts(&e);
}

#if 0
void dmpPlayback(FILE* file) {
	if (!file)
		return;
	char str[4096];
    for (int level = 0; level < 3; ++level) {
        if (!level)
            strcpy(str, "brief\n"); // work around lack of understanding of how FILE works
        else
            fgets(str, sizeof(str), file);
        const char* matchLevel = !level ? "brief\n" : 1 == level ? "normal\n" : "detailed\n";
	    if (strcmp(matchLevel, str)) {
		    OpDebugOut("reading " + std::string(matchLevel) + " failed\n");
		    fclose(file);
		    return;
	    }
        for (int filter = 0; filter < 2; ++filter) {
            const char* matchFilter = !filter ? "filter" : "always";
            std::vector<EdgeFilter>& efs = !filter ? edgeFilters[level].filter
                    : edgeFilters[level].always;
            fgets(str, sizeof(str), file);
            const char* s = str;
            const char* e = str + strlen(str);
            if (' ' != str[0]) {
	            if (strncmp(matchFilter, str, strlen(matchFilter))) {
		            OpDebugOut("reading " + std::string(matchFilter) + " failed\n");
		            fclose(file);
		            return;
                }
                s += strlen(matchFilter);
            } else
                s++;
            if (':' != *s++ || ' ' < *s++) {
		        OpDebugOut("missing : after " + std::string(matchFilter) + "\n");
		        fclose(file);
		        return;
            }
            int peek = -1;
            do {
                if (' ' == peek) {
                    fgets(str, sizeof(str), file);
                    s = str;
                    e = str + strlen(str);
                    peek = -1;
                }
                 while (s < e) {
                    const char* word = s;
                    while (s < e && (isalnum(*s) || '_' == *s))
                        s++;
                    size_t len = s - word;
                    if (!len) {
		                OpDebugOut("word too small " + std::string(str) + "\n");
		                fclose(file);
		                return;
                    }
                    bool wordFound = false;
                    for (size_t index = 0; index < ARRAY_COUNT(filterNames); ++index) {
                        const char* test = filterNames[index].name;
                        if (strlen(test) != len)
                            continue;
                        if (strncmp(test, word, len))
                            continue;
                        wordFound = true;
                        if (efs.end() == std::find_if(efs.begin(), efs.end(), 
                                [&test](const EdgeFilter& ef) {
                            return !strcmp(filterNames[(int) ef].name, test);
                        })) {
                            efs.push_back((EdgeFilter) index);
                        }
                        break;
                    }
                    if (!wordFound) {
                        OpDebugOut("word " + std::string(word, len) + " not found in "
                                + std::string(matchFilter) + "\n");
		                fclose(file);
		                return;
                    }
                    while (s < e && (',' == *s || ' ' >= *s)) {
                        s++;
                    }
                }
            } while (' ' == (peek = fgetc(file)));
            ungetc(peek, file);
        }
    }
    if (fscanf(file, "lineWidth: %d\n", &defaultLineWidth) != 1) {
		OpDebugOut("reading lineWidth failed\n");
		fclose(file);
		return;
	}
    if (fscanf(file, "defaultBase: %d\n", (int*) &defaultBase) != 1) {
		OpDebugOut("reading defaultBase failed\n");
		fclose(file);
		return;
	}
    if (fscanf(file, "defaultLevel: %d\n", (int*) &defaultLevel) != 1) {
		OpDebugOut("reading defaultLevel failed\n");
		fclose(file);
		return;
	}
}

void dmpRecord(FILE* file) {
    auto dmpOne = [file](std::vector<EdgeFilter>& efSet, std::string name) {
        std::string s = name + ": ";
        for (auto ef : efSet) {
            if ((size_t) ef < ARRAY_COUNT(filterNames))
                s += filterNames[(size_t) ef].name + std::string(", ");
        }
        debugPopMatching(s, ' ');
        debugPopMatching(s, ',');
        s = stringFormat(s, 100);
        fprintf(file, "%s\n", s.c_str());
    };
    for (int level = 0; level < 3; ++level) {
        fprintf(file, "%s\n", !level ? "brief" : 1 == level ? "normal" : "detailed"); 
        dmpOne(edgeFilters[level].filter, "filter");
        dmpOne(edgeFilters[level].always, "always");
    }
    fprintf(file, "lineWidth: %d\n", defaultLineWidth);
    fprintf(file, "defaultBase: %d\n", (int) defaultBase);
    fprintf(file, "defaultLevel: %d\n", (int) defaultLevel);
}
#endif

void dmpT(int ID, float t) {
    const OpEdge* e = findEdge(ID);
    if (e)
        return dmpT(e, t);
    const OpSegment* s = findSegment(ID);
    if (s)
        return dmpT(s, t);
}

void dmpT(const OpEdge* e, float t) {
    OpPoint pt = e->curve.ptAtT((t - e->startT) / (e->endT - e->startT));
    OpDebugOut(e->debugDump(defaultLevel, defaultBase) + " t:" + STR(t) + " pt:" 
            + pt.debugDump(defaultLevel, defaultBase) + "\n");
}

void dmpT(const OpSegment* s, float t) {
    OpPoint pt = s->c.ptAtT(t);
    OpDebugOut(s->debugDump(defaultLevel, defaultBase) + " t:" + STR(t) + " pt:" 
            + pt.debugDump(defaultLevel, defaultBase) + "\n");
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { SectType::w, #w }
#define SectType_Base
ENUM_NAME_STRUCT(SectType)

ENUM_NAME_STRUCT_ABBR(SectType);
#define SECTTYPE_NAME(w, abbr) { SectType::w, #w, #abbr }

static _SectTypeAbbr SectTypeAbbrs[] = {
    SECTTYPE_NAME(none, none),
    SECTTYPE_NAME(endHull, end),
    SECTTYPE_NAME(controlHull, ctrl),
	SECTTYPE_NAME(midHull, mid),
	SECTTYPE_NAME(snipLo, snpL),
	SECTTYPE_NAME(snipHi, snpH),
};

ENUM_NAME_ABBR(SectType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { Unsortable::w, #w }
#define Unsortable_Base
ENUM_NAME_STRUCT(Unsortable)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { RayOrder::w, #w }
#define RayOrder_Base
ENUM_NAME_STRUCT(RayOrder)

static int8_t OpDebugBool(const char*& str, const char* label) {
    if (!OpDebugOptional(str, label))
        return -1;
    size_t b = OpDebugReadSizeT(str);
    if (0 != b && 1 != b) {
        OpDebugOut("!!! " + std::string(label) + ": expected bool 0 or 1; got " + STR(b) + "\n");
        exit(1);
    }
    return b;
}

void EdgePal::dumpSet(const char*& str) {
    OpDebugRequired(str, "edge");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "unsectID"))
        unsectID = (int) OpDebugReadSizeT(str);
    reversed = OpDebugBool(str, "reversed");
}

void Distance::dumpSet(const char*& str) {
    OpDebugOptional(str, "{");
    OpDebugOptional(str, ",");
    OpDebugRequired(str, "edge");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "cept"))
        cept = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "edgeInsideT"))
        edgeInsideT = OpDebugHexToFloat(str);
    rayOrder = RayOrderStr(str, "rayOrder", RayOrder::uninitialized);
    reversed = OpDebugBool(str, "reversed");
    dependent = OpDebugBool(str, "dependent");
    over = OpDebugBool(str, "over");
    OpDebugOptional(str, "}");
}

void Distance::dumpResolveAll(OpContext* context) {
    context->dumpResolve(edge);
}

void EdgeDist::dumpSet(const char*& str) {
    OpDebugOptional(str, "{");
    if (OpDebugOptional(str, "opp"))
        opp.dumpSet(str);
    if (OpDebugOptional(str, "dist"))
        dist = OpDebugHexToFloat(str);
    OpDebugOptional(str, "}");
}

OpSaveDump::OpSaveDump(DebugLevel l, DebugBase b) {
    saveL = defaultLevel;
    saveB = defaultBase;
    defaultLevel = l;
    defaultBase = b;
}
OpSaveDump::~OpSaveDump() {
    defaultLevel = saveL;
    defaultBase = saveB;
}

void dmpBase(int v) {
    defaultBase = (DebugBase) v;
}

void dmpLevel(int v) {
    defaultLevel = (DebugLevel) v;
}

void dp(const OpEdge* e) { 
    dp(*e); 
}

void dp(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(defaultLevel, defaultBase));
}

void dp(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    dp(e);
}

void OpEdge::dumpSet(const char*& str) {
    // note that edge pointers are returned as IDs since edge may not have been inflated yet
    auto strID = [&str](const char* label) {
        return (int64_t) OpDebugReadNamedInt(str, label);
    };
    id = (int) strID("edge");
    static_assert(0 == offsetof(OpEdge, segment));
    segment = (OpSegment*) strID("segment");
    (void) strID("contour");  // can't do anything with this here
    ASSERT_ORDERED(segment, ray);
    ray.dumpSet(str);
    ASSERT_ORDERED(ray, priorEdge);
    priorEdge = (OpEdge*) strID("prior");  // non-zero must be replaced with pointer later
    ASSERT_ORDERED(priorEdge, nextEdge);
    nextEdge = (OpEdge*) strID("next");
    ASSERT_ORDERED(nextEdge, lastEdge);
    lastEdge = (OpEdge*) strID("last");
    ASSERT_ORDERED(lastEdge, center);
    if (OpDebugOptional(str, "center"))
        center.dumpSet(str);
    ASSERT_SERIAL_OFFSET(*this, center, 4, curve);
    OpDebugRequired(str, "curve");
    curve.c.context = (ContextPtr) dumpContext;
    curve.dumpSet(str);
    ASSERT_ORDERED(curve, vertical_impl);
    ASSERT_ORDERED(vertical_impl, upright_impl);
    if (OpDebugOptional(str, "upright_impl")) {
        upright_impl.dumpSet(str);
        OpDebugRequired(str, "vertical_impl");
        vertical_impl.c.context = (ContextPtr) dumpContext;
        vertical_impl.dumpSet(str);
    }
    ASSERT_ORDERED(upright_impl, linkBounds);
    if (OpDebugOptional(str, "linkBounds"))
        linkBounds.dumpSet(str);
    ASSERT_ORDERED(linkBounds, winding);
    if (OpDebugOptional(str, "winding"))
        winding.dumpSet(dumpContext, str);
    ASSERT_ORDERED(winding, sum);
    if (OpDebugOptional(str, "sum"))
        sum.dumpSet(dumpContext, str);
    ASSERT_ORDERED(sum, many);
    if (OpDebugOptional(str, "many"))
        many.dumpSet(dumpContext, str);
    ASSERT_ORDERED(many, coinPals);
    if (OpDebugOptional(str, "coinPals")) {
        coinPals.resize(OpDebugReadSizeT(str));
        for (auto& pal : coinPals) {
            pal.opp = (OpSegment*) strID("opp");
            pal.coinID = strID("coinID");
        }
    }
    ASSERT_ORDERED(coinPals, unSects);
    if (OpDebugOptional(str, "unSects")) {
        unSects.resize(OpDebugReadSizeT(str));
        for (auto& unSect : unSects) {
            unSect = (OpIntersection*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_ORDERED(unSects, pals);
    if (OpDebugOptional(str, "pals")) {
        pals.resize(OpDebugReadSizeT(str));
        for (auto& pal : pals) {
            pal.dumpSet(str);
        }
    }
    ASSERT_ORDERED(pals, hulls);
    if (OpDebugOptional(str, "hulls")) {
        hulls.h.resize(OpDebugReadSizeT(str));
        for (auto& hull : hulls.h)
            hull.dumpSet(str);
    }
    ASSERT_ORDERED(hulls, startDist);
    if (OpDebugOptional(str, "startDist"))
        startDist.dumpSet(str);
    ASSERT_ORDERED(startDist, endDist);
    if (OpDebugOptional(str, "endDist"))
        endDist.dumpSet(str);
    ASSERT_ORDERED(endDist, startT);
    startT = OpDebugReadNamedFloat(str, "startT");
    ASSERT_ORDERED(startT, endT);
    endT = OpDebugReadNamedFloat(str, "endT");
    // id up front
    ASSERT_ORDERED(endT, id);
    ASSERT_ORDERED(id, ccUnsectID);
    ccUnsectID = strID("ccUnsectID");
    ASSERT_ORDERED(ccUnsectID, whichEnd_impl);
    whichEnd_impl = EdgeMatchStr(str, "whichEnd", EdgeMatch::none);
    ASSERT_ORDERED(whichEnd_impl, rayFail);
    rayFail = EdgeFailStr(str, "rayFail", EdgeFail::none);
    ASSERT_ORDERED(rayFail, windZero);
    windZero = WindZeroStr(str, "windZero", WindZero::unset);
    ASSERT_ORDERED(windZero, isUnsortable);
    isUnsortable = UnsortableStr(str, "isUnsortable", Unsortable::none);
	DEBUG_SET_BOOL(isUnsortable, active_impl);
    DEBUG_SET_BOOL(active_impl, inLinkups);
    DEBUG_SET_BOOL(inLinkups, linkHead);
    DEBUG_SET_BOOL(linkHead, inOutput);
    DEBUG_SET_BOOL(inOutput, disabled);
    DEBUG_SET_BOOL(disabled, smallTRange);
    DEBUG_SET_BOOL(smallTRange, isUnsplitable);
    DEBUG_SET_BOOL(isUnsplitable, ccEnd);
    DEBUG_SET_BOOL(ccEnd, ccLarge);
    DEBUG_SET_BOOL(ccLarge, ccOverlaps);
    DEBUG_SET_BOOL(ccOverlaps, ccSmall);
    DEBUG_SET_BOOL(ccSmall, ccStart);
    DEBUG_SET_BOOL(ccStart, centerless);
    DEBUG_SET_BOOL(centerless, startSeen);
    DEBUG_SET_BOOL(startSeen, endSeen);
    DEBUG_SET_BOOL(endSeen, unsectableStart);
    DEBUG_SET_BOOL(unsectableStart, unsectableEnd);
#if OP_DEBUG
    ASSERT_SERIAL_OFFSET(*this, unsectableEnd, 3, debugMatch);
    debugMatch = (OpEdge*) strID("debugMatch");
    ASSERT_ORDERED(debugMatch, debugZeroErr);
    debugZeroErr = (OpEdge*) strID("debugZeroErr");
    ASSERT_ORDERED(debugZeroErr, debugParentID);
    debugParentID = (int) strID("debugParentID");
    ASSERT_ORDERED(debugParentID, debugDepth);
    debugDepth = (int) strID("debugDepth");
    ASSERT_ORDERED(debugDepth, debugCC);
    debugCC = (int) strID("debugCC");
    ASSERT_ORDERED(debugCC, debugRayMatch);
    debugRayMatch = (int) strID("debugRayMatch");
	DEBUG_SET_BOOL(debugRayMatch, debugUnordered);
	DEBUG_SET_BOOL(debugUnordered, debugSumSet);
#endif
    ASSERT_SERIAL_OFFSET(*this, debugSumSet, 6, dumpContext);
    // omit dumpContext
#if OP_DEBUG_DUMP
    DEBUG_SET_BOOL(dumpContext, debugJoin);
    DEBUG_SET_BOOL(debugJoin, debugLimb);
    DEBUG_SET_BOOL(debugLimb, debugReleased);
    ASSERT_ORDERED_OFFSET(debugReleased, debugSetDisabled, 5);
#else
    ASSERT_ORDERED(dumpContext, debugSetDisabled);
#endif
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
    ASSERT_ORDERED(debugSetDisabled, debugSetMaker);
    if (OpDebugOptional(str, "debugSetMaker"))
        debugSetMaker.dumpSet(str);
    ASSERT_ORDERED(debugSetMaker, debugSetSum);
    if (OpDebugOptional(str, "debugSetSum"))
        debugSetSum.dumpSet(str);
    ASSERT_ORDERED(debugSetSum, debugPriorID);
#elif OP_DEBUG_IMAGE
    ASSERT_ORDERED(debugOne, debugPriorID);
#else
    ASSERT_ORDERED(dumpContext, debugPriorID);
#endif
#if OP_DEBUG_VALIDATE
    debugPriorID = (int) strID("debugPriorID");
    DEBUG_SET_BOOL(debugPriorID, debugScheduledForErasure);
    static_assert(offsetof(OpEdge, debugScheduledForErasure) + sizeof(debugScheduledForErasure)
            + 3 == sizeof(OpEdge));
#elif OP_DEBUG_IMAGE
    static_assert(offsetof(OpEdge, debugOne) + sizeof(debugOne)
            + 0 == sizeof(OpEdge));
#else
    static_assert(offsetof(OpEdge, dumpContext) + sizeof(dumpContext)
            + 0 == sizeof(OpEdge));
#endif
}

void OpEdge::dumpResolveAll(OpContext* c) {
    c->dumpResolve(segment);
    ray.dumpResolveAll(c);
    c->dumpResolve(priorEdge);
    c->dumpResolve(nextEdge);
    c->dumpResolve(lastEdge);
    winding.dumpResolveAll(c);
    sum.dumpResolveAll(c);
    many.dumpResolveAll(c);
    for (auto& coinPal : coinPals)
        c->dumpResolve(coinPal.opp);
    for (auto& unSect : unSects)
        c->dumpResolve(unSect);
    for (auto& pal : pals)
        c->dumpResolve(pal.edge);
    for (auto& hull : hulls.h)
        hull.dumpResolveAll(c);
#if OP_DEBUG
    c->dumpResolve(debugMatch);
    c->dumpResolve(debugZeroErr);
#endif
}

void OpEdge::debugCompare(std::string s) const {
#if 0
    OpEdge test(s);
    OP_ASSERT(segment->id == test.segment->id);
    OP_ASSERT(start == test.start);
    OP_ASSERT(end == test.end);

#endif
}

void dmpWinding(const OpEdge& edge) {
    std::string s = edge.debugDumpWinding();
    OpDebugOut(s + "\n");
}

void dmpEnd(const OpEdge& edge)  {
    dmpMatch(edge.curve.lastPt());
    if (edge.endPt() != edge.curve.lastPt())
        dmpMatch(edge.endPt());
}

void dmpFull(const OpEdge& edge) { 
    edge.segment->dumpFull(); 
}

void dmpCenter(const OpEdge& edge) {
    std::string s = edge.debugDumpCenter(defaultLevel, defaultBase);
    OpDebugOut(s + "\n");
}

void dmpEdges(const OpEdge& edge) {
    OpDebugFormat(edge.segment->debugDumpEdges());
}

void dmpIntersections(const OpEdge& edge) {
    OpDebugFormat(edge.segment->debugDumpIntersections());
}

// don't just dump it, find the best theoretical one through binary search
std::string OpEdge::debugDumpCenter(DebugLevel l, DebugBase b) const {
    std::string s = "[" + STR(id) + "] center:" + center.debugDump(l, b);
    OpRect r = bounds();
    OpPoint c = { (r.left + r.right) / 2, (r.top + r.bottom) / 2 };
    s += " bounds center:" + c.debugDump(l, b) + "\n";
    float lo = startT;
    float hi = endT;
    OpPtT bestX, bestY;
    for (XyChoice xy : { XyChoice::inX, XyChoice::inY } ) {
        for (;;) {
            float mid = (lo + hi) / 2;
            OpPoint loPt = segment->c.ptAtT(lo);
            OpPoint midPt = segment->c.ptAtT(mid);
            OpPoint hiPt = segment->c.ptAtT(hi);
            bool inLo = OpMath::Between(loPt.choice(xy), c.choice(xy), midPt.choice(xy));
            bool inHi = OpMath::Between(midPt.choice(xy), c.choice(xy), hiPt.choice(xy));
            OP_ASSERT(inLo || inHi);
            if ((inLo && inHi) || lo >= mid || mid >= hi) {
                (XyChoice::inX == xy ? bestX.pt : bestY.pt) = midPt;
                (XyChoice::inX == xy ? bestX.t : bestY.t) = mid;
                break;
            }
            (inLo ? hi : lo) = mid;
        }
    }
    s += "bestX:" + bestX.debugDump(l, b);
    s += " bestY:" + bestY.debugDump(l, b);
    return s;
}

OpPtT OpEdge::debugFindT(Axis axis, float oppXY) const {
	OpPtT found;
	float startXY = curve.firstPt().choice(axis);
	float endXY = curve.lastPt().choice(axis);
	if (oppXY == startXY)
		found = OpPtT(curve.firstPt(), startT);
	else if (oppXY == endXY)
		found = OpPtT(curve.lastPt(), endT);
	else {
		found.pt = OpPoint(SetToNaN::dummy);
		found.t = segment->debugFindAxisT(axis, startT, endT, oppXY);
		if (OpMath::IsNaN(found.t))
			found = (oppXY < startXY) == (startXY < endXY) 
                    ? OpPtT(curve.firstPt(), startT) : OpPtT(curve.lastPt(), endT);
	}
	return found;
}

#if OP_DEBUG_GLOBALS

void dmpCompare(OpPoint a, OpPoint b) {
    OpVector diff = a - b;
    OpVector threshold = debugGlobalContext->threshold;
    OpDebugOut("difference: " + STR(diff.dx / threshold.dx) + "x, " 
            + STR(diff.dy / threshold.dy) + "x\n");
}

void dmpCompare(const OpPtT& a, const OpPtT& b) {
    dmpCompare(a.pt, b.pt);
}

#endif

OpPtT dc_ex, dc_ey, dc_ox, dc_oy;
extern void draw(const OpPtT& );

void OpCurveCurve::drawClosest(const OpPoint& originalPt) const {
#if 0   // !!! out of date (rework when needed)
    dumpClosest(originalPt);
    ::draw(dc_ex);
    ::draw(dc_ey);
    ::draw(dc_ox);
    ::draw(dc_oy);
#endif
}

// find and report closest t value of both curves though binary search
void OpCurveCurve::dumpClosest(const OpPoint& originalPt) const {
    auto tMatch = [](const OpEdge* e, XyChoice inXy, OpPoint pt, float& dist, std::string name) {
        const OpCurve& c = e->segment->c;
        // !!! will fail with new interface
        OpPair endCheck = c.xyAtT( { e->startT, e->endT }, inXy);
        float goal = pt.choice(inXy);
        if (!OpMath::Between(endCheck.s, goal, endCheck.l)) {
            dist = OpNaN;
            return OpPtT();
        }
        float mid = (e->startT + e->endT) / 2;
        float step = (mid - e->startT) / 2;
        OpPair test, x;
        while (true) {
            test = { mid - step, mid + step };
            if (test.s == mid || mid == test.l)
                break;
        // !!! will fail with new interface
            x = c.xyAtT(test, inXy);
            if (x.s == x.l)
                break;
            bool ordered = x.s < x.l;
            if (ordered ? goal < x.s : goal > x.s)
                mid = test.s;
            else if (ordered ? goal > x.l : goal < x.l)
                mid = test.l;
            step /= 2;
        }
        OpPtT result = OpPtT(c.ptAtT(mid), mid);
        dist = OpMath::IsFinite(result.t) ? (result.pt - pt).length() : OpNaN;
        std::string s;
        s += "edge:" + e->debugDump(defaultLevel, defaultBase);
        s += "\ngoal:" + pt.debugDump(defaultLevel, defaultBase) 
                + " (" + (XyChoice::inX == inXy ? "inX" : "inY") + ")";
        s += " mid[" + debugFloat(defaultBase, test.s) + ", " + debugFloat(defaultBase, mid)
                + ", " + debugFloat(defaultBase, test.l) + "]";
        s += " step:" + debugFloat(defaultBase, step);
        s += " xy[" + debugFloat(defaultBase, x.s) + ", " 
                + debugFloat(defaultBase, result.pt.choice(inXy)) + ", "
                + debugFloat(defaultBase, x.l) + "]";
        auto xyDist = [&c, pt](float xy) {
            OpPoint xyPt = c.ptAtT(xy);
            float d = OpMath::IsFinite(xy) ? (xyPt - pt).length() : OpNaN;
            return d;
        };
        s += " dist[" + debugFloat(defaultBase, xyDist(test.s)) + ", "
                + debugFloat(defaultBase, dist) + ", "
                + debugFloat(defaultBase, xyDist(test.l)) + "]";
        s += " " + name + " result:" + result.debugDump(defaultLevel, defaultBase);
        OpDebugFormat(s + "\n");
        return result;
    };
    auto ptMinMax = [](const OpEdge* e, const OpPtT& ePtT, OpPtT& eSm, OpPtT& eLg) {
        const OpCurve& eC = e->segment->c;
        eSm.t = ePtT.t;
        do {
            eSm.t = std::max(e->startT, eSm.t - OpEpsilon);
            eSm.pt = eC.ptAtT(eSm.t);
        } while (eSm.pt == ePtT.pt && eSm.t != e->startT);
        eLg.t = ePtT.t;
        do {
            eLg.t = std::min(e->endT, eLg.t + OpEpsilon);
            eLg.pt = eC.ptAtT(eLg.t);
        } while (eLg.pt == ePtT.pt && eLg.t != e->endT);
    };
    OpPtT bestEPtT, bestOPtT = OpPtT(originalPt, OpNaN);
    float exd, eyd, oxd, oyd;
    int iterations = 0;
    OpPointBounds eLast;
    OpPointBounds oLast;
    const OpEdge* originalEdge = &seg->edges[0];
    const OpEdge* originalOpp = &opp->edges[0];
    do {
        ++iterations;
        dc_ex = tMatch(originalEdge, XyChoice::inX, bestOPtT.pt, exd, "ex");
        dc_ey = tMatch(originalEdge, XyChoice::inY, bestOPtT.pt, eyd, "ey");
        bestEPtT = !OpMath::IsFinite(eyd) || exd < eyd ? dc_ex : dc_ey;
        dc_ox = tMatch(originalOpp, XyChoice::inX, bestEPtT.pt, oxd, "ox");
        dc_oy = tMatch(originalOpp, XyChoice::inY, bestEPtT.pt, oyd, "oy");
        bestOPtT = !OpMath::IsFinite(oyd) || oxd < oyd ? dc_ox : dc_oy;
        OpPtT eSm, eLg, oSm, oLg;
        ptMinMax(originalEdge, bestEPtT, eSm, eLg);
        ptMinMax(originalOpp, bestOPtT, oSm, oLg);
        OpPointBounds eBounds(eSm.pt, eLg.pt);
        OpPointBounds oBounds(oSm.pt, oLg.pt);
        std::string s = "eBounds:" + eBounds.debugDump(defaultLevel, defaultBase);
        s += " oBounds:" + oBounds.debugDump(defaultLevel, defaultBase);
        s += " intersects:" + std::string(eBounds.intersects(oBounds) ? "true" : "false");
        OpDebugFormat(s);
        if (eLast == eBounds && oLast == oBounds)
            break;
        eLast = eBounds;
        oLast = oBounds;
        if (eBounds.intersects(oBounds))
            break;
    } while (true);
    auto axisPtT = [originalPt](const OpEdge* e, Axis axis) {
        OpPtT result = e->debugFindT(axis, originalPt.choice(axis));
        if (!result.pt.isFinite())
            result.pt = e->segment->c.ptAtT(result.t);
        return result;
    };
    OpPtT elx = axisPtT(originalEdge, Axis::vertical);
    float elxd = (originalEdge->segment->c.ptAtT(elx.t) - originalPt).length();
    OpPtT ely = axisPtT(originalEdge, Axis::horizontal);
    float elyd = (originalEdge->segment->c.ptAtT(ely.t) - originalPt).length();
    OpPtT olx = axisPtT(originalOpp, Axis::vertical);
    float olxd = (originalOpp->segment->c.ptAtT(olx.t) - originalPt).length();
    OpPtT oly = axisPtT(originalOpp, Axis::horizontal);
    float olyd = (originalOpp->segment->c.ptAtT(oly.t) - originalPt).length();
    std::string eClosestStr;
    float closestDistance = OpInfinity;
    OpPtT closestPtT;
    std::string closestLabel;
    auto checkClosest = [&closestDistance, &closestLabel, &closestPtT]
            (float dist, const OpPtT& distPtT, std::string distLabel) {
        if (closestDistance > dist) {
            closestDistance = dist;
            closestLabel = distLabel;
            closestPtT = distPtT;
        }
    };
    checkClosest(exd, dc_ex, "ex");
    checkClosest(eyd, dc_ey, "ey");
    checkClosest(elxd, elx, "elx");
    checkClosest(elyd, ely, "ely");
    checkClosest(oxd, dc_ox, "ox");
    checkClosest(oyd, dc_oy, "oy");
    checkClosest(olxd, olx, "olx");
    checkClosest(olyd, oly, "oly");
    auto floatString = [&closestDistance, &closestLabel]
            (std::string s, const OpPtT& ptT, float dist) {
        if (closestDistance == dist && s != closestLabel)
            return s + "=" + closestLabel;
        return s + ":" + ptT.debugDump(defaultLevel, defaultBase) 
                + " dist:" + debugFloat(defaultBase, dist);
    };
    std::string s = "iterations:" + STR(iterations);
    s += " original:" + originalPt.debugDump(defaultLevel, defaultBase) + " closest:" + closestLabel;
    s += "\noriginalEdge:" + floatString("ex", dc_ex, exd) + " " + floatString("ey", dc_ey, eyd);
    s += "\n " + floatString("elx", elx, elxd) + " " + floatString("ely", ely, elyd);
    s += "\noriginalOpp:" + floatString("ox", dc_ox, oxd) + ", " + floatString("oy", dc_oy, oyd);
    s += "\n " + floatString("olx", olx, olxd) + " " + floatString("oly", oly, olyd);
    OpDebugFormat(s);
}

void dmpClosest(const OpCurveCurve& cc, const OpPoint& p) {
    cc.dumpClosest(p);
}

void dmpHulls(const OpEdge& edge) {
	std::string s;
    for (auto& hs : edge.hulls.h)
        s += hs.debugDump(defaultLevel, defaultBase) + "\n";
    OpDebugFormat(s);
}

std::string debugDmpLink(const OpEdge& edge, DebugLevel l, DebugBase b) {
    std::string s;
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::contour, 
			EF::priorEdge, EF::nextEdge, EF::lastEdge,
			EF::startT, EF::endT, 
			EF::active_impl, EF::inLinkups, EF::inOutput, EF::disabled, EF::isUnsplitable,
			EF::centerless };
    OpSaveEF saveEF(showFields);
    std::vector<const OpEdge*> links;
    auto dumpEm = [edge, &links, &s, l, b](std::string post = "") {
        for (const OpEdge* link : links) {
            if (link == &edge)
                s += ">>> ";
            s += link->debugDump(l, b) + "\n";
            if (link == &edge)
                s += " <<<";
        }
        return s;
    };
    links.push_back(&edge);
    const OpEdge* link = &edge;
    while ((link = link->priorEdge)) {
        if (links.end() != std::find(links.begin(), links.end(), link))
            return dumpEm(" (loop prior)\n");
        links.insert(links.begin(), link);
    }
    link = &edge;
    while ((link = link->nextEdge)) {
        if (links.end() != std::find(links.begin(), links.end(), link))
            return dumpEm(" (loop next)\n");
        links.push_back(link);
    }
    return dumpEm();
}

void dmpLink(const OpEdge& edge) {
    std::string s = debugDmpLink(edge, defaultLevel, defaultBase);
    OpDebugFormat(s + "\n");
}

static void addToSeen(OpContext* context, std::vector<const OpEdge*>& seen, const OpEdge& edge) {
    const OpEdge* loopStart = edge.debugIsLoop(EdgeMatch::start, LeadingLoop::in);
    int safetyCount = 0;
    int safetyLimit = 700;
    PathOpsV0Lib::DebugValue fun = context->debugContextCallbacks.debugSafetyLinksFuncPtr;
    if (fun)
        safetyLimit = (*fun)();
    const OpEdge* link = &edge;
    if (seen.end() == std::find(seen.begin(), seen.end(), &edge))
        seen.push_back(&edge);
    while ((link = link->priorEdge)) {
        if (seen.end() == std::find(seen.begin(), seen.end(), link))
            seen.push_back(link);
        if (link == loopStart)
            break;
        if (++safetyCount > safetyLimit) {
            OpDebugOut("!!! likely loops forever: prior\n");
            break;
        }
    }
    const OpEdge* loopEnd = edge.debugIsLoop(EdgeMatch::end, LeadingLoop::in);
    safetyCount = 0;
    link = &edge;
    while ((link = link->nextEdge)) {
        if (seen.end() == std::find(seen.begin(), seen.end(), link))
            seen.push_back(link);
        if (link == loopEnd)
            break;
        if (++safetyCount > safetyLimit) {
            OpDebugOut("!!! likely loops forever: next\n");
            break;
        }
    }
}

std::string debugDmpLinks(OpContext* context, DebugLevel l, DebugBase b) {
    std::string s;
    std::vector<const OpEdge*> seen;
    for (const OpContour* c : context->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.priorEdge)
                    continue;
                addToSeen(context, seen, edge);
                if (edge.priorEdge || edge.nextEdge || edge.lastEdge)
                    s += debugDmpLink(edge, l, b) + "\n";
            }
        }
    }
    for (const OpContour* c : context->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (seen.end() == std::find(seen.begin(), seen.end(), &edge)) {
                    if (edge.priorEdge || edge.nextEdge || edge.lastEdge)
                        s += debugDmpLink(edge, l, b) + "\n";
                    addToSeen(context, seen, edge);
                }
            }
        }
    }
    return debugPopMatching(s, '\n');
}

#if OP_DEBUG_GLOBALS

void dmpLinks() {
    std::string s = debugDmpLinks(debugGlobalContext, defaultLevel, defaultBase);
    OpDebugFormat(s + "\n");
}

#endif

void dmpPoints(const OpEdge& edge) {
    std::string s = edge.debugDumpPoints();
    OpDebugFormat(s);
}

void dmpRay(const OpEdge& edge) {
	std::string s = edge.ray.debugDumpHeader(defaultLevel, defaultBase) + "\n";
	s += edge.ray.targets.debugDump(defaultLevel, defaultBase) + "\n";
	for (const auto& distance : edge.ray.distances) {
		s += distance.debugDump(defaultLevel, defaultBase) + "\n";
	}
	for (const auto& erase : edge.ray.erased) {
		s += "erased " + erase.debugDump(defaultLevel, defaultBase) + "\n";
	}
    OpDebugFormat(s);
}

void dmpStart(const OpEdge& edge) {
    dmpMatch(edge.curve.firstPt());
    if (edge.startPt() != edge.curve.firstPt())
        dmpMatch(edge.startPt());
}

void CallerDataStorage::DumpSet(const char*& str, CallerDataStorage** previousPtr) {
    CallerDataStorage* storage = new CallerDataStorage;
    *previousPtr = storage;
    storage->next = (CallerDataStorage*) OpDebugOptional(str, "next");  // non-zero means there is more
    OpDebugRequired(str, "used");
    storage->used = OpDebugReadSizeT(str);
    OpDebugByteArray(str, storage->used, storage->storage);
    if (storage->next)
        DumpSet(str, &storage->next);
}

size_t CallerDataStorage::dumpOffset(PathOpsV0Lib::ContextUserData data) const {
    const CallerDataStorage* stowage = this;
    size_t offset = 0;
	while (stowage) {
        if (stowage->storage <= data.data 
                && ((uint8_t*) data.data + data.size <= stowage->storage + stowage->used))
            return offset + ((uint8_t*) data.data - stowage->storage);
		stowage = stowage->next;
	}
    OP_ASSERT(0);
    return 0;
}

void CallerDataStorage::dumpResolve(PathOpsV0Lib::ContextUserData& data) {
    const CallerDataStorage* stowage = this;
    size_t offset = (size_t) data.data;
    while (offset + data.size > stowage->used) {
        OP_ASSERT(offset > stowage->used);
        OP_ASSERT(stowage->next);
        stowage = stowage->next;
        offset -= stowage->used;
    }
    data.data = (void*) (stowage->storage + offset);
}

void OpEdgeStorage::DumpSet(const char*& str, OpContext* dumpContext, DumpStorage type) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpEdge* edge = nullptr;
        // !!! hackery ahead: note that 'contours->allocateEdge(this)' won't compile
        if (DumpStorage::cc == type)
            edge = dumpContext->allocateEdge(dumpContext->ccStorage  OP_DEBUG_PARAMS("ccStorage"));
        else if (DumpStorage::filler == type)
            edge = dumpContext->allocateEdge(dumpContext->fillerStorage  OP_DEBUG_PARAMS("fillerStorage"));
        else {
            OpDebugExit("edge storage missing");
        }
        (void) new(edge) OpEdge();
        edge->dumpContext = dumpContext;
        edge->dumpSet(str);
    }
}

void OpEdgeStorage::dumpResolveAll(OpContext* c) {
    int count = debugCount();
    for (int index = 0; index < count; ++index)
        ((OpEdge*) debugIndex(index))->dumpResolveAll(c);
}

void OpLimbStorage::DumpSet(const char*& str, OpContext* dumpContext) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpLimb* limb = dumpContext->allocateLimb();
        limb->dumpSet(str);
    }
}

void OpLimbStorage::dumpResolveAll(OpContext* c) {
    int count = (int) debugCount();
    for (int index = 0; index < count; ++index)
        debugIndex(index)->dumpResolveAll(c);
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { LinkPass::w, #w }
#define LinkPass_Base
ENUM_NAME_STRUCT(LinkPass)

std::string OpContour::debugDumpJoin(DebugLevel l, DebugBase b) const {
    std::string s;
	s += "contour:" + STR(id) + " " ;
	s += "merges: " + STR(merges.size()) + " [";
	for (OpContour* member : merges) {
        s += STR(member->id) + " ";
	}
    debugPopMatching(s, ' ');
    s += "]";
	s += DebugLevel::detailed == l ? "\n" : " ";
    auto dumpEdgeIDs = [&s, l](const std::vector<OpEdge*>& edges, std::string name) {
        if (!edges.size())
            return;
		if (DebugLevel::detailed != l && debugIfMatching(s, '\n'))
			s.back() = ' ';
        s += name + ":" + STR(edges.size()) + " [";
        for (auto e : edges)
            s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
        s += "]\n";
    };
    dumpEdgeIDs(byArea, "byArea");
    dumpEdgeIDs(unsectByArea, "unsectByArea");
	if (!backwardsBuilt && DebugLevel::detailed == l)
		s += "disabledBackwards (not built)\n";
    else
		dumpEdgeIDs(disabledBackwards, "disabledBackwards");
	if (!centerlessBuilt && DebugLevel::detailed == l)
		s += "disabledCenterless (not built)\n";
    else
		dumpEdgeIDs(disabledCenterless, "disabledCenterless");
	if (!palsBuilt && DebugLevel::detailed == l)
		s += "disabledPals (not built)\n";
	else
		dumpEdgeIDs(disabledPals, "disabledPals");
    dumpEdgeIDs(unsortables, "unsortables");
    if (linkups.l.size()) {
		if (DebugLevel::normal == l)
			s += linkups.debugDump(DebugLevel::brief, b);
		else
			s += "linkups: " + linkups.debugDump(l, b);
	}
    if (endLinks.l.size()) {
        if (DebugLevel::file == l || DebugLevel::normal == l) {
            s += "endLinks:" + STR(endLinks.l.size()) + " [";
            for (OpEdge* linkup : endLinks.l) {
				s += STR(linkup->id);
				if (linkup->lastEdge)
					s += ".." + STR(linkup->lastEdge->id);
				s += " ";
			}
            debugPopMatching(s, ' ');
            s += "]\n";
        } else {
            s += "";
            s += "-- endLinks:" + STR(endLinks.l.size()) + "\n";
            s += endLinks.debugDump(l, b) + "\n";
        }
    }
	return s;
}

void OpJoiner::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "bestGap"))
        bestGap.dumpSet(str);
    linkMatch = EdgeMatchStr(str, "linkMatch", EdgeMatch::none);
    linkPass = LinkPassStr(str, "linkPass", LinkPass::none);
    if (OpDebugOptional(str, "edge"))
        edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "lastLink"))
        lastLink = (OpEdge*) OpDebugReadSizeT(str);
//    if (OpDebugOptional(str, "matchPt"))
//       matchPt.dumpSet(str);
}

void OpJoiner::dumpResolveAll(OpContext* c) {
    bestGap.dumpResolveAll(c);
    c->dumpResolve(edge);
    c->dumpResolve(lastLink);
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { LimbPass::w, #w }
#undef LimbPass_Base
#define LimbPass_Base
ENUM_NAME_STRUCT(LimbPass)

void OpLimb::dumpResolveAll(OpContext* c) {
    c->dumpResolve(edge);
    c->dumpResolve(lastLimbEdge);
    c->dumpResolve(linkedContour);
    c->dumpResolve(parent);
    for (OpLimb*& limb : debugBranches)
        c->dumpResolve(limb);
}

void OpLimb::dumpSet(const char*& str) {
    static_assert(0 == offsetof(OpLimb, limbBounds));
    if (OpDebugOptional(str, "limbBounds"))
        limbBounds.dumpSet(str);
    ASSERT_ORDERED(limbBounds, firstPts);
    if (OpDebugOptional(str, "firstPts")) {
        size_t count = OpDebugReadSizeT(str);
        firstPts.resize(count);
        for (OpPoint& pt : firstPts)
            pt.dumpSet(str);
    }

    ASSERT_ORDERED(firstPts, lastPts);
    if (OpDebugOptional(str, "lastPts")) {
        size_t count = OpDebugReadSizeT(str);
        lastPts.resize(count);
        for (OpPoint& pt : lastPts)
            pt.dumpSet(str);
    }
    ASSERT_ORDERED(lastPts, edge);
    edge = (OpEdge*) (OpDebugOptional(str, "edge") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(edge, lastLimbEdge);
    lastLimbEdge = (OpEdge*) (OpDebugOptional(str, "lastLimbEdge") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(lastLimbEdge, parent);
    parent = (const OpLimb*) (OpDebugOptional(str, "parent") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(parent, linkedContour);
    linkedContour = (OpContour*) (OpDebugOptional(str, "linkedContour") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(linkedContour, lastT);
    lastT = OpDebugReadNamedFloat(str, "lastT");
    ASSERT_ORDERED(lastT, closeDistance);
    closeDistance = OpDebugReadNamedFloat(str, "closeDistance");
    ASSERT_ORDERED(closeDistance, linkedIndex);
    linkedIndex = (uint32_t) (OpDebugOptional(str, "linkedIndex") ? OpDebugReadSizeT(str) : OpMax);
    ASSERT_ORDERED(linkedIndex, match);
    match = EdgeMatchStr(str, "match", EdgeMatch::none);
    ASSERT_ORDERED(match, lastMatch);
    lastMatch = EdgeMatchStr(str, "lastMatch", EdgeMatch::none);
    ASSERT_ORDERED(lastMatch, treePass);
    treePass = LimbPassStr(str, "treePass", LimbPass::uninitialized);
    ASSERT_ORDERED(treePass, deadEnd);
    deadEnd = OpDebugBool(str, "deadEnd");
    ASSERT_ORDERED(deadEnd, looped);
    looped = OpDebugBool(str, "looped");
    ASSERT_ORDERED(looped, resetPass);
    resetPass = OpDebugBool(str, "resetPass");
    ASSERT_SERIAL_OFFSET(*this, resetPass, 6, debugBranches);
    if (OpDebugOptional(str, "debugBranches")) {
        size_t count = OpDebugReadSizeT(str);
        for (size_t index = 0; index < count; ++index)
            debugBranches.push_back((OpLimb*) OpDebugReadSizeT(str));
    }
    ASSERT_ORDERED(debugBranches, id);
    OpDebugRequired(str, "id");
    id = (int) OpDebugReadSizeT(str);
    static_assert(sizeof(*this) == offsetof(OpLimb, id) + sizeof(id) + 4);
}

void OpTree::dumpSet(const char*& str) {
    static_assert(0 == offsetof(OpTree, context));  // skip context
    ASSERT_ORDERED(context, trunk);
    trunk = (OpLimb*) (OpDebugOptional(str, "trunk") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(trunk, bestGapLimb);
    bestGapLimb = (OpLimb*) (OpDebugOptional(str, "bestGapLimb") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(bestGapLimb, bestLimb);
    bestLimb = (OpLimb*) (OpDebugOptional(str, "bestLimb") ? OpDebugReadSizeT(str) : 0);
    ASSERT_ORDERED(bestLimb, bestDistance);
    bestDistance = OpDebugReadNamedFloat(str, "bestDistance");
    ASSERT_ORDERED(bestDistance, bestPerimeter);
    bestPerimeter = OpDebugReadNamedFloat(str, "bestPerimeter");
    ASSERT_ORDERED(bestPerimeter, maxLimbs);
    maxLimbs = OpDebugReadNamedInt(str, "maxLimbs");
    ASSERT_ORDERED(maxLimbs, totalUsed);
    totalUsed = OpDebugReadNamedInt(str, "totalUsed");
    ASSERT_ORDERED(totalUsed, id);
    id = OpDebugReadNamedInt(str, "id");
    ASSERT_ORDERED(id, limbPass);
    limbPass = LimbPassStr(str, "limbPass", LimbPass::none);
    ASSERT_ORDERED(limbPass, disabled);
    disabled = OpDebugOptional(str, "disabled");
    ASSERT_ORDERED(disabled, smallGap);
    smallGap = OpDebugOptional(str, "smallGap");
    ASSERT_SERIAL_OFFSET(*this, smallGap, 1, debugAddEach);
    debugAddEach = OpDebugReadNamedInt(str, "debugAddEach");
    static_assert(sizeof(OpTree) == offsetof(OpTree, debugAddEach) + sizeof(debugAddEach) + 4);
}

void OpTree::dumpResolveAll(OpContext* c) {
    c->dumpResolve(trunk);
    c->dumpResolve(bestGapLimb);
    c->dumpResolve(bestLimb);
}

void dmp(std::array<CoinEnd, 4>& coinEndArray) {
    for (auto& cea : coinEndArray)
        OpDebugOut(cea.debugDump(defaultLevel, defaultBase) + "\n");
}

void EdgeRun::dumpSet(const char*& str) {
    OpDebugRequired(str, "edgePtT:");
    edgePtT.dumpSet(str);
    OpDebugRequired(str, "oppPtT:");
    oppPtT.dumpSet(str);
    oppDist = OpDebugReadNamedFloat(str, "oppDist");
    fromFoundT = OpDebugOptional(str, "fromFoundT") ? LimitFrom::yes : LimitFrom::no;
    byZero = OpDebugOptional(str, "byZero");
#if OP_DEBUG
//    OpDebugRequired(str, "debugBetween");
//    debugBetween = (int) OpDebugReadSizeT(str);
#endif
#if OP_DEBUG_MAKER
    debugSetMaker.dumpSet(str);
#endif
}

void FoundLimit::dumpSet(const char*& str) {
    parentEdge = OpDebugOptional(str, "parentEdge") ? (const OpEdge*) OpDebugReadSizeT(str) : nullptr; 
    parentOpp = OpDebugOptional(str, "parentOpp") ? (const OpEdge*) OpDebugReadSizeT(str) : nullptr; 
    OpDebugRequired(str, "segPtT");
    segPtT.dumpSet(str);
    OpDebugRequired(str, "oppPtT");
    oppPtT.dumpSet(str);
    fromFoundT = OpDebugOptional(str, "fromFoundT") ? LimitFrom::yes : LimitFrom::no;
    oppOutOfOrder = OpDebugOptional(str, "oppOutOfOrder") ? Unordered::yes : Unordered::no;
    used = OpDebugOptional(str, "used") ? LimitUsed::yes : LimitUsed::no;
    match = OpDebugOptional(str, "match") ? LimitMatch::yes : LimitMatch::no;
    swapped = OpDebugOptional(str, "swapped") ? LimitSwapped::yes : LimitSwapped::no;
    bettered = OpDebugOptional(str, "bettered") ? LimitBettered::yes : LimitBettered::no;
    edgeLine = OpDebugOptional(str, "edgeLine") ? LimitLine::yes : LimitLine::no;
    oppLine = OpDebugOptional(str, "oppLine") ? LimitLine::yes : LimitLine::no;
#if OP_DEBUG_MAKER
    OpDebugRequired(str, "debugMaker");
    debugMaker.dumpSet(str);
#endif
}

void FoundLimit::dumpResolveAll(OpContext* c) {
    c->dumpResolve(parentEdge);
    c->dumpResolve(parentOpp);
}

void FoundLimits::dumpSet(const char*& str) {
	DEBUG_SET_FIRST_VECTOR(i);
	DEBUG_SET_VECTOR(i, snips);
    ASSERT_ORDERED(snips, cc);
    DEBUG_SET_OPTIONAL_VALUE(cc, unique);
	DEBUG_SET_BOOL(unique, smSegT);
    DEBUG_SET_BOOL(smSegT, lgSegT);
    DEBUG_SET_BOOL(lgSegT, smOppT);
	DEBUG_SET_BOOL(smOppT, lgOppT);
}

void FoundLimits::dumpResolveAll(OpContext* c) {
    for (FoundLimit& limit : i) {
        limit.dumpResolveAll(c);
    }
}

void SnipPtTs::dumpSet(const char*& str) {
    OpDebugRequired(str, "segPtT");
    segPtT.dumpSet(str);
    OpDebugRequired(str, "oppPtT");
    oppPtT.dumpSet(str);
    OpDebugRequired(str, "segCut");
    segCut.dumpSet(str);
    OpDebugRequired(str, "oppCut");
    oppCut.dumpSet(str);
}

void RayTarget::dumpSet(const char*& str) {
    static_assert(0 == offsetof(RayTarget, contour));
    OpDebugRequired(str, "contour");
    contour = (OpContour*) OpDebugReadSizeT(str);
    ASSERT_ORDERED(contour, bounds);
    OpDebugRequired(str, "bounds");
    bounds.dumpSet(str);
    static_assert(sizeof(RayTarget) == offsetof(RayTarget, bounds) + sizeof(bounds));
}

void RayTarget::dumpResolveAll(OpContext* context) {
    context->dumpResolve(contour);
}

void RayTargets::dumpSet(const char*& str) {
    ASSERT_ORDERED(context, t);
    OpDebugRequired(str, "t");
    size_t size = OpDebugReadSizeT(str);
    t.resize(size);
    for (RayTarget& target : t) {
        target.dumpSet(str);
    }
    OpDebugOptional(str, "]");
    ASSERT_ORDERED(t, chainBounds);
    OpDebugRequired(str, "chainBounds");
    chainBounds.dumpSet(str);
    ASSERT_ORDERED(chainBounds, inXY);  // either target.contour->inX or inY (not worth serializing)
    ASSERT_ORDERED(inXY, edgeIndex);
    if (OpDebugOptional(str, "edgeIndex"))
        edgeIndex = OpDebugReadSizeT(str);
    ASSERT_ORDERED(edgeIndex, tIndex);
    if (OpDebugOptional(str, "tIndex"))
        tIndex = OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "debugEdgesContour")) {
        ASSERT_ORDERED(debugEdgesContour, debugEdgesAxis);
        debugEdgesContour = (OpContour*) OpDebugReadSizeT(str);
        ASSERT_ORDERED(debugEdgesContour, debugEdgesAxis);
        debugEdgesAxis = AxisStr(str, "debugEdgesAxis", Axis::neither);
        static_assert(sizeof(RayTargets) == offsetof(RayTargets, debugEdgesAxis) 
                + sizeof(debugEdgesAxis) + 7);
    }
}

void RayTargets::dumpResolveAll(OpContext* ctx) {
    context = ctx;
    for (RayTarget& target : t) {
        target.dumpResolveAll(context);
    }
    if (debugEdgesContour) {
        context->dumpResolve(debugEdgesContour);
	    inXY = Axis::horizontal == debugEdgesAxis 
                ? &debugEdgesContour->inX : &debugEdgesContour->inY;
    }
}

void SectRay::dumpSet(const char*& str) {
    static_assert(0 == offsetof(SectRay, targets));
    if (!OpDebugOptional(str, "targets"))
        return;
    targets.dumpSet(str);
    ASSERT_ORDERED(targets, distances);
    OpDebugRequired(str, "distances");
    size_t size = OpDebugReadSizeT(str);
    distances.resize(size);
    for (Distance& dist : distances)
        dist.dumpSet(str);
    ASSERT_ORDERED(distances, erased);
    OpDebugRequired(str, "erased");
    size = OpDebugReadSizeT(str);
    erased.resize(size);
    for (Distance& erase : erased)
        erase.dumpSet(str);
    ASSERT_ORDERED(erased, homeTangent);
    if (OpDebugOptional(str, "homeTangent"))
        homeTangent.dumpSet(str);
    ASSERT_ORDERED(homeTangent, normal);
    if (OpDebugOptional(str, "normal"))
        normal = OpDebugHexToFloat(str);
    ASSERT_ORDERED(normal, homeCept);
    if (OpDebugOptional(str, "homeCept"))
        homeCept = OpDebugHexToFloat(str);
    ASSERT_ORDERED(homeCept, homeT);
    if (OpDebugOptional(str, "homeT"))
        homeT = OpDebugHexToFloat(str);
    ASSERT_ORDERED(homeT, interceptLimit);
    if (OpDebugOptional(str, "interceptLimit"))
        interceptLimit = OpDebugHexToFloat(str);
    ASSERT_ORDERED(interceptLimit, mid);
    if (OpDebugOptional(str, "mid"))
        mid = OpDebugHexToFloat(str);
    ASSERT_ORDERED(mid, midEnd);
    if (OpDebugOptional(str, "midEnd"))
       midEnd = OpDebugHexToFloat(str);
    ASSERT_ORDERED(midEnd, axis);
    axis = AxisStr(str, "axis:", Axis::neither);
    ASSERT_ORDERED(axis, sorted);
    sorted = OpDebugOptional(str, "sorted");
    static_assert(sizeof(SectRay) == offsetof(SectRay, sorted) 
            + sizeof(sorted) + 6);
}

void SectRay::dumpResolveAll(OpContext* context) {
    targets.dumpResolveAll(context);
    for (Distance& dist : distances)
        dist.dumpResolveAll(context);
    for (Distance& erase : erased)
        erase.dumpResolveAll(context);
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { PtType::w, #w }
#define PtType_Base
ENUM_NAME_STRUCT(PtType)

#if OP_DEBUG_VERBOSE
std::string DebugDepth::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    ASSERT_FIRST(all);
    DEBUG_DUMP_START_REQUIRED_VALUE(all);
    DEBUG_DUMP_REQUIRED_VALUE(all, depth);
    ASSERT_LAST_OFFSET(depth, 4);
    return s;
}

void DebugDepth::dumpSet(const char*& str) {
    ASSERT_FIRST(all);
    DEBUG_SET_START_REQUIRED_VALUE(all);
    DEBUG_SET_REQUIRED_VALUE(all, depth);
    ASSERT_LAST_OFFSET(depth, 4);
}

std::string DebugRunSize::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    ASSERT_FIRST(edgeRuns);
    DEBUG_DUMP_START_REQUIRED_VALUE(edgeRuns);
    DEBUG_DUMP_REQUIRED_VALUE(edgeRuns, oppRuns);
    ASSERT_LAST(oppRuns);
    return s;
}

void DebugRunSize::dumpSet(const char*& str) {
    ASSERT_FIRST(edgeRuns);
    DEBUG_SET_START_REQUIRED_VALUE(edgeRuns);
    DEBUG_SET_REQUIRED_VALUE(edgeRuns, oppRuns);
    ASSERT_LAST(oppRuns);
}
#endif

void CcCurves::dumpSet(const char*& str) {
    ASSERT_FIRST(cc);   // skip  (must be set by caller)
    DEBUG_SET_VECTOR_IDS(cc, c);
    DEBUG_SET_VECTOR(c, runs);
    DEBUG_SET_VECTOR(runs, deleted);
    DEBUG_SET_ID(deleted, seg);
    DEBUG_SET_ID(seg, opp);
    ASSERT_SERIAL(*this, opp, oppCurves);  // skip  (must be set by caller)
    DEBUG_SET_FLOAT(oppCurves, scaledMax);
    DEBUG_SET_VECTOR_OFFSET(scaledMax, debugRuns, 4);
    ASSERT_LAST(debugRuns);
}

void CcCurves::dumpResolveAll(OpContext* context) {
    context->dumpResolve(seg);
    context->dumpResolve(opp);
    for (auto& edge : c)
        context->dumpResolve(edge);
}

std::string DumpCurveCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l) {
        ASSERT_FIRST(cc);
        DEBUG_DUMP_COMMON_VECTOR(cc);
        DEBUG_DUMP_VECTOR(cc, runIndex);
        DEBUG_DUMP_VECTOR(runIndex, runs);
        DEBUG_DUMP_REQUIRED_VALUE(runs, nthCall);
        ASSERT_LAST_OFFSET(nthCall, 4);
        return s;
    }
    if (DebugLevel::file == l)
        s += "nthCall:" + STR(nthCall) + " ";
    if (DebugLevel::file == l) {
        if (runIndex.size()) {
            s += "runIndex[" + STR(runIndex.size()) + "\n";
            for (const DebugRunSize& size : runIndex)
                s += "{edgeRuns:" + STR(size.edgeRuns) + " oppRuns:" + STR(size.oppRuns) + "} ";
            debugPopMatching(s, ' ');
            s += "] ";
        }
        if (runs.size()) {
            s += "runs:" + STR(runs.size()) + "\n";
            for (const auto& run : runs)
                s += run.debugDump(l, b) + "\n";
        }
    }
    return s;
}

void OpCurveCurve::dumpSet(const char*& str) {
    ASSERT_FIRST(context);
    DEBUG_SET_ID(context, seg);
    DEBUG_SET_ID(seg, opp);
    DEBUG_SET_ID(opp, parentEdge);
    DEBUG_SET_ID(parentEdge, parentOpp);
    DEBUG_SET_STRUCT(parentOpp, edgeCurves);
    DEBUG_SET_STRUCT(edgeCurves, oppCurves);
    // note that edgeCurves, oppCurves have pointers to each other and to parent
    edgeCurves.baseInit(this, &oppCurves);
    oppCurves.baseInit(this, &edgeCurves);
    DEBUG_SET_STRUCT(oppCurves, limits);
    DEBUG_SET_STRUCT(limits, maxSplit);
    DEBUG_SET_STRUCT(maxSplit, maxBoundedEdge);
    DEBUG_SET_STRUCT(maxBoundedEdge, maxUnsectable);
    DEBUG_SET_REQUIRED_VALUE(maxUnsectable, endMatches);
    DEBUG_SET_FLOAT(endMatches, maxAngleMatch);
    DEBUG_SET_FLOAT(maxAngleMatch, maxAngleSweep);
    DEBUG_SET_FLOAT(maxAngleSweep, maxSignSwap);
    DEBUG_SET_FLOAT(maxSignSwap, maxSplitBias);
    DEBUG_SET_FLOAT(maxSplitBias, maxDist);
    DEBUG_SET_FLOAT(maxDist, maxEdgeTSlop);
    DEBUG_SET_OPTIONAL_VALUE(maxEdgeTSlop, depth);
    DEBUG_SET_REQUIRED_VALUE(depth, unsplitables);
    DEBUG_SET_REQUIRED_VALUE(unsplitables, maxCheckSplit);
    DEBUG_SET_REQUIRED_VALUE(maxCheckSplit, maxDeep);
    DEBUG_SET_REQUIRED_VALUE(maxDeep, maxShallow);
    DEBUG_SET_REQUIRED_VALUE(maxShallow, maxSplits);
    DEBUG_SET_BOOL(maxSplits, reversed);
    DEBUG_SET_BOOL(reversed, boundedEdgeFailed);
    DEBUG_SET_BOOL(boundedEdgeFailed, overlap);
    DEBUG_SET_BOOL(overlap, rotateFailed);
    DEBUG_SET_BOOL(rotateFailed, sectResult);
    DEBUG_SET_BOOL(sectResult, lastDepthReduced);
    DEBUG_SET_BOOL(lastDepthReduced, foundGap);
    DEBUG_SET_BOOL(foundGap, splitMid);
    DEBUG_SET_BOOL(splitMid, splitHullFail);
//    ASSERT_LAST_OFFSET(splitHullFail, 3);
}

void DumpCurveCurve::dumpSet(const char*& str) {
    ASSERT_FIRST(cc);
    DEBUG_SET_COMMON_VECTOR(cc);
    DEBUG_SET_VECTOR(cc, runIndex);
    DEBUG_SET_VECTOR(runIndex, runs);
    DEBUG_SET_REQUIRED_VALUE(runs, nthCall);
    ASSERT_LAST_OFFSET(nthCall, 4);
}

void OpCurveCurve::dumpResolveAll(OpContext* c) {
    c->dumpResolve(seg);
    c->dumpResolve(opp);
    c->dumpResolve(parentEdge);
    c->dumpResolve(parentOpp);
    edgeCurves.dumpResolveAll(c);
    oppCurves.dumpResolveAll(c);
    limits.dumpResolveAll(c);
}

#if OP_DEBUG_VERBOSE
std::string OpCurveCurve::debugDumpDepth(int level) {
    std::string s;
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::startT, EF::endT, 
			EF::isUnsplitable,
            EF::ccEnd, EF::ccLarge, EF::ccOverlaps, EF::ccSmall, EF::ccStart,
            EF::hulls, EF::debugParentID, EF::debugSetMaker };
    OpSaveEF saveEF(showFields);
    s += "depth:" + STR(level) + "\n";
    if (depth <= level) {
        for (const auto e : edgeCurves.c)
            s += e->debugDump(defaultLevel, defaultBase) + "\n";
        for (const auto e : oppCurves.c)
            s += e->debugDump(defaultLevel, defaultBase) + "\n";
        s += "edgeCurves.runs: " + STR(edgeCurves.runs.size()) + "\n";
        for (const auto& run : edgeCurves.runs)
            s += run.debugDump(defaultLevel, defaultBase) + "\n";
        s += "oppCurves.runs: " + STR(oppCurves.runs.size()) + "\n";
        for (const auto& run : oppCurves.runs)
            s += run.debugDump(defaultLevel, defaultBase) + "\n";
        return s;
    }
    if (context->ccStorage) {
		int count = context->ccStorage->debugCount();
		for (int index = 0; index < count; ++index) {
            const OpEdge* edge = context->ccStorage->debugIndex(index);
            if (edge->debugDepth == level)
			    s += edge->debugDump(defaultLevel, defaultBase) + "\n";
		}

    }
    if ((size_t) level < dumpCurveCurve.runIndex.size()) {
        size_t lo = dumpCurveCurve.runIndex[level].edgeRuns;
        size_t hi = dumpCurveCurve.runIndex[level].oppRuns;
        s += "edgeCurves.runs: " + STR(hi - lo) + "\n";
        for (size_t index = lo; index < hi; ++index) {
            const EdgeRun& run = dumpCurveCurve.runs[index];
            s += run.debugDump(defaultLevel, defaultBase) + "\n";
        }
        lo = hi;
        hi = (int) dumpCurveCurve.runIndex.size() <= level + 1 ? dumpCurveCurve.runs.size() 
                : dumpCurveCurve.runIndex[level + 1].edgeRuns;
        s += "oppCurves.runs: " + STR(hi - lo) + "\n";
        for (size_t index = lo; index < hi; ++index) {
            const EdgeRun& run = dumpCurveCurve.runs[index];
            s += run.debugDump(defaultLevel, defaultBase) + "\n";
        }
    }
    return s;
}

void OpCurveCurve::dumpDepth(int level) {
    std::string s = debugDumpDepth(level);
    OpDebugFormat(s);
}

void dmpDepth(int level) {
    OpCurveCurve* cc = debugGlobalContext->debugCurveCurve;
    if (!cc)
        return OpDebugOut("!debugGlobalContext->debugCurveCurve\n");
    cc->dumpDepth(level);
}

void OpCurveCurve::dumpDepth() {
    for (int level = 0; level <= depth; ++level) {
        dumpDepth(level);
    }
}

void dmpDepth() {
    OpCurveCurve* cc = debugGlobalContext->debugCurveCurve;
    if (!cc)
        return OpDebugOut("!debugGlobalContext->debugCurveCurve\n");
    cc->dumpDepth();
}
#endif

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { MatchEnds::w, #w }
#define MatchEnds_Base
ENUM_NAME_STRUCT(MatchEnds)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { CoinOpp::w, #w }
#define CoinOpp_Base
ENUM_NAME_STRUCT(CoinOpp)

void OpIntersection::dumpSet(const char*& str) {
    static_assert(0 == offsetof(OpIntersection, segment));
    if (OpDebugOptional(str, "segment"))
        segment = (OpSegment*) OpDebugReadSizeT(str);
    ASSERT_ORDERED(segment, opp);
    if (OpDebugOptional(str, "opp"))
        opp = (OpIntersection*) OpDebugReadSizeT(str);
    ASSERT_ORDERED(opp, ptT);
    OpDebugRequired(str, "ptT");
    ptT.dumpSet(str);
    ASSERT_ORDERED(ptT, callerPt);
    if (OpDebugOptional(str, "callerPt"))
        callerPt.dumpSet(str);
    ASSERT_ORDERED(callerPt, coincidenceID);
    coincidenceID = OpDebugReadNamedInt(str, "coincidenceID");
    ASSERT_ORDERED(coincidenceID, unsectID);
    unsectID = OpDebugReadNamedInt(str, "unsectID");
    ASSERT_ORDERED(unsectID, mergeID);
    mergeID = OpDebugReadNamedInt(str, "mergeID");
    ASSERT_ORDERED(mergeID, coinEnd);
    coinEnd = MatchEndsStr(str, "coinEnd", MatchEnds::none);
    ASSERT_ORDERED(coinEnd, unsectEnd);
    unsectEnd = MatchEndsStr(str, "unsectEnd", MatchEnds::none);
    ASSERT_ORDERED(unsectEnd, coinOpp);
    coinOpp = CoinOppStr(str, "coinOpp", CoinOpp::no);
	DEBUG_SET_BOOL(coinOpp, betweenCoins);
	DEBUG_SET_BOOL(betweenCoins, ccLine);
	DEBUG_SET_BOOL(ccLine, ccSect);
	DEBUG_SET_BOOL(ccSect, ccUnsectable);
	DEBUG_SET_BOOL(ccUnsectable, collapsed);
#if OP_DEBUG
    id = OpDebugReadNamedInt(str, "id");
    debugSrcID = OpDebugReadNamedInt(str, "debugSrcID");
    debugOppID = OpDebugReadNamedInt(str, "debugOppID");
    debugCoincidenceID = OpDebugReadNamedInt(str, "debugCoincidenceID");
	DEBUG_SET_BOOL(debugCoincidenceID, debugErased);
#endif
#if OP_DEBUG_MAKER
    debugSetMaker.dumpSet(str);
#endif
}

void OpIntersection::dumpResolveAll(OpContext* c) {
    c->dumpResolve(segment);
    c->dumpResolve(opp);
}

void OpIntersection::debugCompare(std::string s) const {
    OpIntersection test;
    const char* str = s.c_str();
    test.dumpSet(str);
    OP_ASSERT(segment->id == test.segment->id);
    OP_ASSERT(ptT == test.ptT);
}

void dmpFull(const OpIntersection* sect) {
    dmpFull(*sect);
}

void dmpFull(const OpIntersection& sect) {
    dmpFull(sect.segment);
}

void dmpEnd(const OpIntersection& sect) {
    dmp(sect);
}

void dmpStart(const OpIntersection& sect) {
    dmp(sect);
}

void dmpMatch(const OpIntersection& sect) {
    dmpMatch(sect.ptT.pt);
}

void dmpEdges(const OpIntersection& sect) {
    OpDebugFormat(sect.segment->debugDumpEdges());
}

void dmpIntersections(const OpIntersection& sect) {
    OpDebugFormat(sect.segment->debugDumpIntersections());
}

void OpIntersections::dumpSet(const char*& str) {
    unsorted = OpDebugOptional(str, "unsorted");
    if (!OpDebugOptional(str, "intersections"))
        return;
    int sectCount = (int) OpDebugReadSizeT(str);
    i.resize(sectCount);
    OpDebugRequired(str, "[");
    for (int index = 0; index < sectCount; index++) {
        i[index] = (OpIntersection*) OpDebugReadSizeT(str);
    }
    OpDebugRequired(str, "]");
}

OpIntersection* OpSectStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index++) {
		const OpIntersection& test = storage[index];
        if (test.id == ID)
            return const_cast<OpIntersection*>(&test);
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

void OpSectStorage::DumpSet(const char*& str, OpContext* dumpContext) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpIntersection* sect = dumpContext->allocateIntersection();
        sect->dumpSet(str);
    }
}

void OpSectStorage::dumpResolveAll(OpContext* c) {
    int count = debugCount();
    for (int index = 0; index < count; ++index) {
        debugIndex(index)->dumpResolveAll(c);
    }
}

OpSegment::OpSegment() 
    : winding(WindingUninitialized::dummy) {
}

void OpSegment::dumpSet(const char*& str) {
    id = (int) OpDebugReadSizeT(str);
    static_assert(0 == offsetof(OpSegment, contour));
    int contourID = OpDebugOptional(str, "contour[") ? (int) OpDebugReadSizeT(str) : 0;
    OpDebugExitOnFail("mismatched contour id", contourID == contour->id);
    ASSERT_ORDERED(contour, c);
    c.c.context = (ContextPtr) contour->context;
    c.dumpSet(str);
    ASSERT_ORDERED(c, sects);
    if (OpDebugOptional(str, "sects:")) {
        int sectCount = (int) OpDebugReadSizeT(str);
        sects.i.resize(sectCount);
        for (int index = 0; index < sectCount; ++index) {
            sects.i[index] = (OpIntersection*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_ORDERED(sects, edges);
    if (OpDebugOptional(str, "edges:")) {
        int edgeCount = (int) OpDebugReadSizeT(str);
        edges.resize(edgeCount);
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpContext = contour->context;
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpSet(str);
    }
    ASSERT_ORDERED(edges, winding);
    OpDebugRequired(str, "winding");
    winding.dumpSet(contour->context, str);
    ASSERT_ORDERED(winding, id);  // write at front
    DEBUG_SET_BOOL(id, disabled);
    DEBUG_SET_BOOL(disabled, endsMerged);
    DEBUG_SET_BOOL(endsMerged, hasCoin);
    DEBUG_SET_BOOL(hasCoin, hasPals);
    DEBUG_SET_BOOL(hasPals, hasUnsectable);
    DEBUG_SET_BOOL(hasUnsectable, merged);
    // !!! skip debug color for now
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
    static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugSetDisabled) 
            + sizeof(debugSetDisabled));
#endif
}

void OpSegment::dumpResolveAll(OpContext* context) {
//    context->dumpResolve(contour);
    context->contourStorage->debugCheck(contour);  // asserts and exists if missing
    for (auto& sect : sects.i)
        context->dumpResolve(sect);
    for (auto& edge : edges)
        edge.dumpResolveAll(context);
    winding.dumpResolveAll(context);
}

std::string OpSegment::debugDumpEdges() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDump(defaultLevel, defaultBase) + "\n";
    return debugPopMatching(s, '\n');
}

// used to find unsectable range; assumes range all has about the same slope
// !!! this may be a bad idea if two near coincident edges turn near 90 degrees
float OpSegment::debugFindAxisT(Axis axis, float start, float end, float opp) {
	if (!c.isLine()) {
		OpRoots roots = c.axisRayHit(axis, opp, start, end);
		if (1 == roots.count())
			return roots.roots[0];
	} else {
		float pt0xy = c.firstPt().choice(axis);
		float result = (opp - pt0xy) / (c.lastPt().choice(axis) - pt0xy);
		if (start <= result && result <= end)
			return result;
	}
	return OpNaN;
}

void dmpEdges(const OpSegment& seg) {
    OpDebugFormat(seg.debugDumpEdges());
}

std::string OpSegment::debugDumpFull() const {
    std::string s = debugDump(defaultLevel, defaultBase);
    if (sects.unsorted)
        s += " ";
    else
        s += "\n";
    s += debugDumpIntersections();
    s += "edges:";
	if (!edges.empty())
        s += "\n";
    s += debugDumpEdges();
    return s;
}

std::string OpSegment::debugDumpIntersections() const {
    std::string s;
    if (sects.i.empty())
        return s;
    if (sects.unsorted)
        s += "unsorted\n";
    for (auto i : sects.i) {
        std::string is = i->debugDump(defaultLevel, defaultBase);
        std::string match = "segment:";
        size_t matchStart = is.find(match);
        if (std::string::npos != matchStart) {
            size_t digit = matchStart + match.size();
            while ('0' <= is[digit] && is[digit] <= '9')
                ++digit;
            while (' ' == is[digit])
                ++digit;
            is.erase(matchStart, digit - matchStart);
        }
        s += is + "\n";
    }
    return s;
}

void dmpIntersections(const OpSegment& seg) {
    OpDebugFormat(seg.debugDumpIntersections());
}

void dmpCount(const OpSegment& seg) {
    OpDebugFormat("seg:" + seg.debugDumpID() + " edges:" + STR(seg.edges.size())
            + " intersections:" + STR(seg.sects.i.size()) + "\n");
}

void dmpEnd(const OpSegment& seg) {
    dmpMatch(seg.c.lastPt());
}

void dmpFull(const OpSegment& seg) {
    OpDebugFormat(seg.debugDumpFull() + "\n"); 
}

void dmpSegmentEdges(const OpSegment& seg) {
    OpDebugFormat(seg.debugDumpEdges() + "\n");
}

void dmpSegmentIntersections(const OpSegment& seg) {
    OpDebugFormat(seg.debugDumpIntersections() + "\n");
}

void dmpSegmentSects(const OpSegment& seg) {
    dmpSegmentIntersections(seg);
}

void dmpStart(const OpSegment& seg) {
    dmpMatch(seg.c.firstPt());
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { WindingType::w, #w }
#define WindingType_Base
ENUM_NAME_STRUCT(WindingType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { DebugWindingType::w, #w }
#define DebugWindingType_Base
ENUM_NAME_STRUCT(DebugWindingType)

void DumpSet(OpContext* context, PathOpsV0Lib::Winding& w, char const*& str, bool raster) {
    OpDebugRequired(str, "w.contour");
    w.contour = (PathOpsV0Lib::Contour*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "w.size");
    w.size = OpDebugReadSizeT(str);
    w.data = context->allocateWinding(w.size  OP_DEBUG_RASTER_PARAMS(raster));
    for (size_t index = 0; index < w.size; ++index) {
        ((uint8_t*) w.data)[index] = OpDebugByteToInt(str);
	}
}

void DumpResolveAll(PathOpsV0Lib::Winding& w, OpContext* context) {
    OpContour* tempContour = (OpContour*) w.contour;
    if (!tempContour)
        return;
    context->dumpResolve(tempContour);
    w.contour = (ContourPtr) tempContour;
}

void OpWinding::dumpSet(OpContext* context, const char*& str) {
    DumpSet(context, w, str, usedByRaster);
    type = WindingTypeStr(str, "type", WindingType::uninitialized);
    debugType = DebugWindingTypeStr(str, "debugType", DebugWindingType::uninitialized);
}

void OpWinding::dumpResolveAll(OpContext* context) {
    DumpResolveAll(w, context);
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { ChopUnsortable::w, #w }
#define ChopUnsortable_Base
ENUM_NAME_STRUCT(ChopUnsortable)

void FoundEdge::dumpSet(const char*& str) {
    edge = (OpEdge*) OpDebugReadSizeT(str);
    distSq = OpDebugOptional(str, "distSq") ? OpDebugHexToFloat(str) : 0;
    index = OpDebugOptional(str, "index") ? (int) OpDebugReadSizeT(str) : 0;
    whichEnd = EdgeMatchStr(str, "whichEnd", EdgeMatch::none);
    connects = OpDebugOptional(str, "connects");
    loops = OpDebugOptional(str, "loops");
    chop = ChopUnsortableStr(str, "chopUnsortable", ChopUnsortable::none);
}

void FoundEdge::dumpResolveAll(OpContext* c) {
    c->dumpResolve(edge);
}

void HullSect::dumpSet(const char*& str) {
    OpDebugRequired(str, "opp");
    opp = (OpEdge*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "sect");
    sect.dumpSet(str);
    if (OpDebugOptional(str, "oppDist"))
        oppDist.dumpSet(str);
    else
        oppDist.opp.t = OpNaN;   // !!! hacky
    type = SectTypeStr(str, "type", SectType::none);
}

void HullSect::dumpResolveAll(OpContext* c) {
    if (opp)
        c->dumpResolve(opp);
}

void OpVector::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    dx = OpDebugHexToFloat(str);
    dy = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
    OpDebugOptional(str, ",");
}

void OpPoint::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    x = OpDebugHexToFloat(str);
    y = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
    OpDebugOptional(str, ",");
}

void OpPtT::dumpSet(const char*& str) {
    pt.dumpSet(str);
    t = OpDebugReadNamedFloat(str, "t");
}

#if 0
std::string OpRootPts::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += " raw[" + raw.debugDump(l, b) + "]";
    s += " valid[" + valid.debugDump(l, b) + "]";
    s += " count:" + STR(count);
    for (size_t index = 0; index < count; ++index)
        s += ptTs[index].debugDump(l, b) + ", ";
    debugPopMatching(s, ' ');
    return debugPopMatching(s, ',');
}
#endif

void OpRect::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    left = OpDebugHexToFloat(str);
    top = OpDebugHexToFloat(str);
    right = OpDebugHexToFloat(str);
    bottom = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
}

void MatchReverse::dumpSet(const char*& str) {
    match = MatchEndsStr(str, "match", MatchEnds::none);
    reversed = OpDebugOptional(str, "reversed");
}

#if OP_DEBUG_MAKER
std::string OpDebugMaker::debugDump() const {
    if (!valid() || 0 == file.size())
        return "! file name error";
    size_t opPos = file.find("Op");
	return (std::string::npos == opPos ? file : file.substr(opPos)) + ":" + STR(line);
}

void OpDebugMaker::dumpSet(const char*& str) {
    const char* colon = str;
    while (':' != *colon) {
        OP_ASSERT('\0' != *colon);
        OP_ASSERT(colon - str < 100);
        ++colon;
    }
    file = std::string(str, colon - str);
    str = ++colon;
    line = (int) OpDebugReadSizeT(str);
}
#endif

void LinePts::dumpSet(const char*& str) {
//    OpDebugRequired(str, "{");
    pts[0].dumpSet(str);
    pts[1].dumpSet(str);
//    OpDebugRequired(str, "}");
}

#if 0
std::string OpSegments::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDump(l, b) + "\n";
    }
    return s;
}

void dmpEdges(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpEdges() + "\n";
    }
    OpDebugOut(s);
}

void dmpIntersections(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpIntersections() + "\n";
    }
    OpDebugOut(s);
}

void dmpFull(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpFull() + "\n";
    }
    OpDebugOut(s);
}
#endif

#if OP_DEBUGGER
void dmpColor(uint32_t c) {
    OpDebugOut(debugDumpColor(DebugLevel::normal, c) + "\n");
}

void dmpColor(const OpEdge* edge) {
    dmpColor(*edge);
}

void dmpColor(const OpEdge& e) {
    OpContext* context = e.context();
    OpEdge* ccEdge = context->ccStorage->debugFind(e.id);
    bool isCurveCurve = ccEdge && e.id == ccEdge->id;
    PathOpsV0Lib::DebugEdgeType edgeType {
        e.disabled, e.inOutput, Unsortable::none != e.isUnsortable, isCurveCurve, e.ccOverlaps };
    PathOpsV0Lib::DebugEdgeColor debugEdgeColor = 
            context->debugContextCallbacks.debugEdgeColorFuncPtr;
    uint32_t color = debugEdgeColor ? (*debugEdgeColor)(e.winding.w, edgeType) : debugBlack;
    dmpColor(color);
}
#endif

// for typing in immediate window as parameters to dmpBase
extern int dec, hex, hexdec;

int dec = 0;
int hex = 1;
int hexdec = 2;

// for typing in immediate window as parameters to dmpLevel
extern int brief, normal, detailed;
int brief = 0;
int normal = 1;
int detailed = 2;

#endif
