// (c) 2025, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_SERIALIZE

#include <filesystem>
#include "DebugOpsTypes.h"
#include "OpCurveCurve.h"
#include "OpDebugRaster.h"
#include "OpDebugSerialize.h"
#include "OpWinder.h"

DebugBase defaultBase = DebugBase::dec;
DebugLevel defaultLevel = DebugLevel::normal;

#undef STRUCT_ID
#define STRUCT_ID \
OP_X(CoinPair) \
OP_X(OpContour) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpSegment)

#define STRUCT_NO_ID \
OP_X(EdgeRun) \
OP_X(FoundLimits) \
OP_X(HullSect) \
OP_X(OpPtT) \
OP_X(SnipPtTs)

#undef OP_X
#define OP_X(Thing) \
	std::string Thing::debugDumpID() const { \
		return STR(id); \
	}
	STRUCT_ID

#undef OP_X
#define OP_X(Thing) \
	std::string Thing::debugDumpID() const { \
		return ""; \
	}
	STRUCT_NO_ID

#define EDGE_BOOL(lastField, thisBool) \
    ASSERT_ORDERED(lastField, thisBool); \
    if (thisBool) \
        s += #thisBool " "

namespace PathOpsV0Lib {
extern std::string contextErrorName(ContextError element);
}

std::string debugFloat(DebugBase b, float value) {
    std::string s;
    if (DebugBase::hex == b || DebugBase::hexdec == b)
        s = OpDebugDumpHex(value);
    if (DebugBase::dec == b || DebugBase::hexdec == b) {
        if (s.size())
            s += " ";
        s += STR(value);
    }
    return s;
}

std::string debugFloat(DebugLevel l, float value) {
    return debugFloat(DebugLevel::file == l ? DebugBase::hex : defaultBase, value);
}

bool debugIfMatching(std::string& s, char match) {
    return !s.empty() && match == s.back();
}

std::string debugPopMatching(std::string& s, char match) {
    if (debugIfMatching(s, match))
        s.pop_back();
    return s;
}

std::vector<const OpIntersection*> findCoincidence(int ID) {
    std::vector<const OpIntersection*> result;
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == abs(intersection->coincidenceID) 
                        OP_DEBUG_CODE(|| ID == abs(intersection->debugCoincidenceID)))
                    result.push_back(intersection);
            }
        }
    }
#endif
    return result;
}

const OpContour* findContour(int ID) {
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator)
        if (ID == c->id)
            return c;
#endif
    return nullptr;
}

OpEdge* findEdge(int ID) {
#if OP_DEBUG_GLOBALS
    auto match = [ID](const OpEdge& edge) {
        return edge.id == ID || edge.debugRayMatch == ID;
    };
    for (auto c : contourIterator) {
        for (auto& seg : c->segments) {
            for (auto& edge : seg.edges) {
                if (match(edge))
                    return &edge;
            }
        }
    }
    if (OpEdge* filler = debugGlobalContext->fillerStorage
            ? debugGlobalContext->fillerStorage->debugFind(ID) : nullptr)
        return filler;
    // if edge intersect is active, search there too
    if (OpEdge* ccEdge = debugGlobalContext->ccStorage
            ? debugGlobalContext->ccStorage->debugFind(ID) : nullptr)
        return ccEdge;
#endif
    return nullptr;
}

std::vector<const OpEdge*> findEdgeUnsectable(int ID) {
    std::vector<const OpEdge*> result;
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                for (const EdgePal& pal : edge.pals) {
                    if (ID == pal.unsectID)
                        result.push_back(&edge);
                }
            }
        }
    }
#endif
    return result;
}

std::vector<const OpEdge*> findEdgeRayMatch(int ID) {
    std::vector<const OpEdge*> result;
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.debugRayMatch)
                    result.push_back(&edge);
            }
        }
    }
#endif
    return result;
}

const OpIntersection* findIntersection(int ID) {
#if OP_DEBUG_GLOBALS
	OpSectStorage* sectStorage = debugGlobalContext->sectStorage;
	while (sectStorage) {
        for (int index = 0; index < sectStorage->used; ++index) {
			OpIntersection* intersection = &sectStorage->storage[index];
            if (ID == intersection->id)
                return intersection;
        }
		sectStorage = sectStorage->next;
	}
#endif
    return nullptr;
}

const OpLimb* findLimb(int ID) {
#if OP_DEBUG_GLOBALS
    if (const OpLimb* limb = debugGlobalContext->limbStorage
            ? debugGlobalContext->limbStorage->debugFind(ID) : nullptr)
        return limb;
#endif
    return nullptr;
}

std::vector<const OpIntersection*> findMerge(int ID) {
    std::vector<const OpIntersection*> result;
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == intersection->mergeID)
                    result.push_back(intersection);
            }
        }
    }
#endif
    return result;
}

std::vector<const OpIntersection*> findSectUnsectable(int ID) {
    std::vector<const OpIntersection*> result;
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == abs(intersection->unsectID))
                    result.push_back(intersection);
            }
        }
    }
#endif
    return result;
}

const OpSegment* findSegment(int ID) {
#if OP_DEBUG_GLOBALS
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            if (ID == seg.id)
                return &seg;
        }
    }
#endif
    return nullptr;
}

std::string stringFormat(std::string s, int lineWidth, int maxLines) {
    if (!s.size())
		return "";
    std::string result;
	const char* start = &s.front();
    const char* end = &s.back();
    auto overflowed = [&maxLines, start, end, &result]() {
        if (0 == --maxLines) {
            result.pop_back();
            if (start < end)
                result += " (..." + STR((int) (end - start)) + " more)";
            return true;
        }
        return false;
    };
    while (lineWidth && start + lineWidth <= end) {
        const char* c = start;
        for (int i = 0; i < lineWidth; ++i) {
            if ('\n' == c[i]) {
                std::string line = s.substr(start - &s.front(), i + 1);
                result += line;
                start += i + 1;
                if (overflowed())
                    return result;
                break;
            }
        }
        if (start != c)
            continue;
        c = start + lineWidth - 1;
        while (' ' != *c && c > start)
            --c;
        std::string line = s.substr(start - &s.front(), c == start ? lineWidth : c - start);
        result += line + "\n";
        start += line.size();
        if (overflowed())
            return result;
    }
    if (start <= end)
        result += s.substr(start - &s.front());
    debugPopMatching(result, '\n');
    return result;
}

// !!! temporary : I need to figure out where to put the dmp.txt file
std::string dmpFileToPath(std::string name) {
#ifdef __APPLE__
    std::string filename = "/Users/cary/pathopsv0/build/" + name;
#elif _WIN32
    std::string filename = "c:/users/cclar/source/repos/v0/v0/" + name;
#else
    std::string filename = name;
#endif
//    OpDebugOut(filename + "\n");  // !!! calling this too often makes vscode debugging unstable...
    return filename;
}

#undef ENUM_NAME_STRUCT
#define ENUM_NAME_STRUCT(enum) \
struct _##enum##Name { \
    enum element; \
    const char* name; \
}; \
\
static _##enum##Name enum##Names[] = { \
    enum##_Base \
    enum##_Enums \
}; \
\
std::string enum##Name(enum element) { \
    int first = (int) enum##Names[0].element; \
    return enum##Names[(int) element - first].name; \
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

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { LimbPass::w, #w }
#undef LimbPass_Base
#define LimbPass_Base
ENUM_NAME_STRUCT(LimbPass)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { OpDebugExpect::w, #w }
#define OpDebugExpect_Base
ENUM_NAME_STRUCT(OpDebugExpect)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { Unsortable::w, #w }
#define Unsortable_Base
ENUM_NAME_STRUCT(Unsortable)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { RayOrder::w, #w }
#define RayOrder_Base
ENUM_NAME_STRUCT(RayOrder)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { MatchEnds::w, #w }
#define MatchEnds_Base
ENUM_NAME_STRUCT(MatchEnds)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { CoinOpp::w, #w }
#define CoinOpp_Base
ENUM_NAME_STRUCT(CoinOpp)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { PtType::w, #w }
#define PtType_Base
ENUM_NAME_STRUCT(PtType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { ChopUnsortable::w, #w }
#define ChopUnsortable_Base
ENUM_NAME_STRUCT(ChopUnsortable)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { SectType::w, #w }
#define SectType_Base
ENUM_NAME_STRUCT(SectType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { LinkPass::w, #w }
#define LinkPass_Base
ENUM_NAME_STRUCT(LinkPass)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { WindingType::w, #w }
#define WindingType_Base
ENUM_NAME_STRUCT(WindingType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { DebugWindingType::w, #w }
#define DebugWindingType_Base
ENUM_NAME_STRUCT(DebugWindingType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { Rotated::w, #w }
#define Rotated_Base
ENUM_NAME_STRUCT(Rotated)

namespace PathOpsV0Lib {

// don't want funny macros in public interface, so this is explicitly for the only public enum... 
#define CONTEXT_ERROR_NAME(r) { ContextError::r, #r }

std::vector<ContextErrorName> contextErrorNames {
	CONTEXT_ERROR_NAME(none),
	CONTEXT_ERROR_NAME(end),
    CONTEXT_ERROR_NAME(finite),
    CONTEXT_ERROR_NAME(gap),
	CONTEXT_ERROR_NAME(intersection),
	CONTEXT_ERROR_NAME(loop),
	CONTEXT_ERROR_NAME(missing),
	CONTEXT_ERROR_NAME(root),
	CONTEXT_ERROR_NAME(toVertical),
    CONTEXT_ERROR_NAME(tree),
};

#undef CONTEXT_ERROR_NAME

static bool contextErrorOutOfDate = false;

std::string contextErrorName(ContextError element) {
    static bool contextErrorChecked = false;
    int first = (int) contextErrorNames[0].element;
    if (!contextErrorChecked) {
        for (size_t index = 0; index < contextErrorNames.size(); ++index)
           if (!contextErrorOutOfDate && (int) contextErrorNames[index].element != index + first) {
               OpDebugOut("!!! contextErrorNames out of date\n");
               contextErrorOutOfDate = true;
               break;
           }
        contextErrorChecked = true;
    }
    if (contextErrorOutOfDate)
        return STR_E(element);
    return contextErrorNames[(int) element - first].name;
}

}

extern EdgeFilterName filterNames[];
extern std::array<EdgeFilters, 3> edgeFilters;
extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

struct LabelAbbr {
    const char* detailed;
    const char* normal;
    const char* brief;
};

std::vector<LabelAbbr> labelAbbrs = {
    {"last", "l", "l"},
    {"next", "n", "n"},
    {"prior", "p", "p"},
    {"segment", "seg", "s"},
    {"winding", "wind", "w"},
	{"homeCept", "hCept", "hc"},
	{"homeT", "hT", "hT"},
	{"homeTangent", "hTan", "hTan"},
    {"isUnsplitable", "ccIsUS", "cUS"},
    {"ccEnd", "ccE", "cE"},
    {"ccLarge", "ccLg", "lg"},
    {"ccOverlaps", "ovrlaps", "laps"},
    {"ccSmall", "ccSm", "sm"},
    {"ccStart", "ccSt", "cS"},
    {"debugMaker", "dbgMkr", "mkr"},
    {"debugParentID", "dbgParent", "par"},
    {"oppDist", "oDist", "d"},
};

static std::string debugLabel(DebugLevel l, std::string label) {
    if (DebugLevel::file != l && DebugLevel::detailed != l) {
        auto abbrIter = std::find_if(labelAbbrs.begin(), labelAbbrs.end(),
                [label](const LabelAbbr& abbr) {
            return label == abbr.detailed; });
        if (labelAbbrs.end() != abbrIter) {
            return DebugLevel::brief == l ? (*abbrIter).brief : (*abbrIter).normal;
        }
    }
    return DebugLevel::brief == l ? label.substr(0, 1) : label;
}

std::string debugValue(DebugLevel l, DebugBase b, std::string label, float value) {
    std::string s;
    if (DebugLevel::error != l && DebugLevel::file != l && !OpMath::IsFinite(value))
        return s;
    s = debugLabel(l, label) + ":";
    return s + debugFloat(b, value);
}

static std::string debugErrorValue(DebugLevel l, DebugBase b, std::string label, float value) {
    return debugValue(DebugLevel::file == l ? l : DebugLevel::error, b, label, value);
}

static std::string BoolToStr(DebugLevel l, int8_t b, const char* label, const char* brief) {
    if (-1 == b)
        return "";
    if (DebugLevel::file != l) {
        if (b)
            return std::string(DebugLevel::brief == l ? brief : label) + " ";
        return "";
    }
    return label + std::string(":") + STR((int) b) + " ";
}

std::string CallerDataStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (next)
        s += "next ";  // only zero/nonzero is read
    s += "used:" + STR(used) + " ";
    if (DebugLevel::detailed == l || DebugLevel::file == l) {
        s += "\n";
        s += OpDebugDumpByteArray(storage, used);   // 'b' is ignored for now; always return hex
        if (next)
            s += "\n";
    }
    if (next)
        s += " " + next->debugDump(l, b);
    return debugPopMatching(s, ' ');
}

std::string CcCurves::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
//    if (DebugLevel::file == l || DebugLevel::detailed == l) {
        ASSERT_FIRST(cc);   // skip
        DEBUG_DUMP_VECTOR_IDS(cc, c);
        DEBUG_DUMP_VECTOR(c, runs);
        DEBUG_DUMP_VECTOR(runs, deleted);
        DEBUG_DUMP_ID(deleted, seg);
        DEBUG_DUMP_ID(seg, opp);
        ASSERT_SERIAL(*this, opp, oppCurves);  // skip
        DEBUG_DUMP_FLOAT(oppCurves, scaledMax);
        DEBUG_DUMP_VECTOR_OFFSET(scaledMax, debugRuns, 4);
        ASSERT_LAST(debugRuns);
        return s;
//    }
#if 0
    // set up edge::debugDump to only show curvecurve relevant fields
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::startT, EF::endT,
			EF::startDist, EF::endDist, EF::isUnsplitable,
            EF::ccEnd, EF::ccLarge, EF::ccOverlaps, EF::ccSmall, EF::ccStart,
            EF::hulls, 
            EF::debugSetMaker, 
            EF::debugParentID };
    OpSaveEF saveEF(showFields);
    DebugLevel down1 = (DebugLevel) ((int) l - 1);
    for (auto& edge : c)
        s += edge->debugDump(down1, b) + "\n";
    for (auto& run : runs)
        s += run.debugDump(down1, b) + "\n";
    return debugPopMatching(s, '\n');
#endif
}

std::string CoinEnd::debugDump(DebugLevel l, DebugBase b) const { 
    std::string s;
    s += "seg:" + STR(seg->id) + " opp:" + STR(opp->id) + " ptT:" + ptT.debugDump(l, b);
    s += " oppT:" + oppT.debugDump(DebugLevel::error, b);
    return s;
}

std::string CoinPair::debugDump(DebugLevel l, DebugBase b) const {
    DebugLevel oneUp = std::max(DebugLevel::brief, (DebugLevel) ((int) l - 1));
    std::string s = "start:" + start->debugDump(l, b) + "\n";
    if (end) s += "end:" + end->debugDump(l, b) + "\n";
    if (oStart) s += "oStart:" + oStart->debugDump(l, b) + "\n";
    if (oEnd) s += "oEnd:" + oEnd->debugDump(l, b) + "\n";
    if (edge) s += "edge:" + edge->debugDump(oneUp, b) + "\n";
    if (oppEdge) s += "oppEdge:" + oppEdge->debugDump(oneUp, b) + "\n";
    s += "id:" + STR(id) + " ";
    if (lastEdge) s += "lastEdge:" + lastEdge->debugDump(oneUp, b);
    s += "\n";
    return s;
}

std::string Curve_DebugDump(PathOpsV0Lib::Curve c, DebugLevel l, DebugBase b) {
    std::string s;
    OpContext& context = *(OpContext*) c.context;
	if ((size_t) c.type > context.debugCallbacks.size())
		s += "(missing curve name) ";
    else if (!c.type)
        s += "degenerateLine ";
    else {
		auto curveName = context.debugCallback(c).curveNameFuncPtr;
		if (curveName)
			s += (*curveName)() + " ";
	}
    if (DebugLevel::file == l) {
        s += "size:" + STR(c.size) + " ";
        s += "data:" + context.curveDataStorage->debugDump(c.data) + " ";
    } else {
        debugPopMatching(s, ' ');
        s += "{";
    	PathOpsV0Lib::HullPtCount funcPtr = context.callback(c.type).ptCountFuncPtr;
	    int pointCount = 2 + (funcPtr ? (*funcPtr)() : 0);
        for (int i = 0; i < pointCount; ++i) {
            OpPoint pt;
	        if (0 == i)
		        pt = c.data->start;
	        else if (pointCount - 1 == i)
		        pt = c.data->end;
	        else
                pt = context.callback(c.type).curveHullFuncPtr(c, i);
            s += pt.debugDump(DebugLevel::error, b) + ", ";
        }
        debugPopMatching(s, ' ');
        debugPopMatching(s, ',');
        s += "}";
		if ((size_t) c.type <= context.debugCallbacks.size()) {
			auto curveExtra = context.debugCallback(c).curveExtraFuncPtr;
			if (curveExtra)
				s += (*curveExtra)(c, l, b);
		}
    }
    return s;
}

// returns caller curve data stored in contours as bytes encoded in string
std::string CurveDataStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (next)        
        s += "next ";  // only zero/nonzero is read
    s += "used:" + STR(used) + " ";
    if (DebugLevel::detailed == l || DebugLevel::file == l) {
        s += "\n";
        s += OpDebugDumpByteArray(storage, used);   // 'b' is ignored for now; always return hex
        if (next)
            s += "\n";
    }
    if (next)
        s += " " + next->debugDump(l, b);
    return debugPopMatching(s, ' ');
}

// returns byte offset of caller curve data stored in contours
std::string CurveDataStorage::debugDump(PathOpsV0Lib::CurveData* curveData) const {
    const CurveDataStorage* test = this;
    uint8_t* data = (uint8_t*) curveData;
    size_t result = 0;
    while (data < test->storage || data >= &test->storage[sizeof(test->storage)]) {
        result += test->used;
        test = test->next;
        OP_ASSERT(test);
    }
    OP_ASSERT(data < test->storage + test->used);
    ptrdiff_t diff = data - test->storage;
    result += diff;
    return STR(result);
}

std::string CutRangeT::debugDump(DebugLevel l, DebugBase b) const {
    DEBUG_DUMP_FIRST_STRUCT(lo);
    DEBUG_DUMP_LAST_STRUCT(lo, hi);
}

// must be written before segments' curves for curve names
static std::string debugCallbacksDump(const std::vector<PathOpsV0Lib::DebugCurveCallbacks>& 
        debugCallbacks, DebugLevel l, DebugBase b) {
    std::string s = "debugCallbacks:" + STR(debugCallbacks.size()) + "\n";
    for (auto& debugCallback : debugCallbacks) {
        static_assert(0 == offsetof(PathOpsV0Lib::DebugCurveCallbacks, scaleFuncPtr));
        s += debugFindTag(reinterpret_cast<DebugFunction>(debugCallback.scaleFuncPtr));
	    DEBUG_FIND_TAG(debugCallback, scaleFuncPtr,  ptAtDTFuncPtr);    
        DEBUG_FIND_TAG(debugCallback, ptAtDTFuncPtr,  curveNameFuncPtr);
	    DEBUG_FIND_TAG(debugCallback, curveNameFuncPtr,  curveExtraFuncPtr);
        DEBUG_FIND_TAG(debugCallback, curveExtraFuncPtr, debugSubDivideFuncPtr);
#if 0 && OP_TEST_RASTER
        DEBUG_FIND_TAG(debugCallback, debugSubDivideFuncPtr, addRasterFuncPtr);
        static_assert(sizeof(PathOpsV0Lib::DebugCurveCallbacks)  
                == offsetof(PathOpsV0Lib::DebugCurveCallbacks, addRasterFuncPtr)
                + sizeof(debugCallback.addRasterFuncPtr));
#else
        static_assert(sizeof(PathOpsV0Lib::DebugCurveCallbacks)  
                == offsetof(PathOpsV0Lib::DebugCurveCallbacks, debugSubDivideFuncPtr)
                + sizeof(debugCallback.debugSubDivideFuncPtr));
#endif
    }
    return s;
}

// must be written before contours' windings
static std::string debugContextCallbacksDump(const PathOpsV0Lib::DebugContextCallbacks& 
        debugContextCallbacks, DebugLevel l, DebugBase b) {
    std::string s = "debugContextCallbacks:";
    static_assert(0 == offsetof(PathOpsV0Lib::DebugContextCallbacks, debugIsFillFuncPtr));
    s += debugFindTag(reinterpret_cast<DebugFunction>(debugContextCallbacks.debugIsFillFuncPtr));
    DEBUG_FIND_TAG(debugContextCallbacks, debugIsFillFuncPtr, debugMergeEndsFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugMergeEndsFuncPtr, debugMergeFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugMergeFuncPtr, debugDumpWindingOutFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpWindingOutFuncPtr, debugDumpWindingSetFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpWindingSetFuncPtr, debugDumpOutFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpOutFuncPtr, debugImageWindingOutFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugImageWindingOutFuncPtr, debugImageWindingNamesFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugImageWindingNamesFuncPtr, debugWindingVisibleFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugWindingVisibleFuncPtr, debugSafetyLinksFuncPtr);
#if OP_DEBUGGER
    DEBUG_FIND_TAG(debugContextCallbacks, debugSafetyLinksFuncPtr, debugEdgeColorFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugEdgeColorFuncPtr) 
            + sizeof(debugContextCallbacks.debugEdgeColorFuncPtr) == sizeof(debugContextCallbacks));
#else
    ASSERT_SERIAL(debugContextCallbacks, debugSafetyLinksFuncPtr, debugEdgeColorFuncName);
    if (!debugContextCallbacks.debugEdgeColorFuncName.empty())
        s += "debugEdgeColorFuncName:" + debugContextCallbacks.debugEdgeColorFuncName + " ";
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugEdgeColorFuncName) 
            + sizeof(debugContextCallbacks.debugEdgeColorFuncName) == sizeof(debugContextCallbacks));
#endif
    return s;
}

std::string DebugCurveData_DebugDump(OpContext* context, const PathOpsV0Lib::DebugCurveData& dcd) {
    std::string s;
    CurveDataStorage* storage = context->curveDataStorage;
    OP_ASSERT(storage);
    uint8_t* dcdPtr = (uint8_t*) dcd.data;
    size_t offset = 0;
    while (dcdPtr < storage->storage || &storage->storage[sizeof(storage->storage)] <= dcdPtr) {
        OP_ASSERT(storage->next);
        offset += sizeof(storage->storage);
        storage = storage->next;
    }
    offset += dcdPtr - storage->storage;
    s += "data:" + STR(offset) + " ";
    s += "size:" + STR(dcd.size) + " ";
    return s;
}

std::string DebugDump(const PathOpsV0Lib::Winding& w, DebugLevel l, DebugBase b) {
    std::string s;
    OpContour* contour = (OpContour*) w.contour;
    OpContext* context = contour ? contour->context : nullptr;
    if (DebugLevel::file == l) {
        s += "w.contour:" + STR(contour ? contour->id : 0) + " ";
		s += "w.size:" + STR(w.size) + " ";
        s += OpDebugDumpByteArray((const uint8_t*) w.data, w.size);
    } else {
		if (w.size && context) {
			auto windingOut = context->debugContextCallbacks.debugDumpWindingOutFuncPtr;
			if (windingOut)
				s += (*windingOut)(w) + " ";
		}
    }
    return s;
}

#if OP_DEBUG_GLOBALS
std::string DebugDump(int id, DebugLevel l, DebugBase b) {
    std::string s;
    if (std::vector<const OpIntersection*> coins = findCoincidence(id); coins.size()) {
        for (auto coin : coins) {
            s += coin->debugDump(l, b) + "\n";
        }
    }
    if (const OpContour* contour = findContour(id))
        s += contour->debugDump(l, b) + "\n";
    if (const OpEdge* edge = findEdge(id))
        s += edge->debugDump(l, b) + "\n";
    if (std::vector<const OpEdge*> matches = findEdgeRayMatch(id); matches.size()) {
        for (auto match : matches) {
            s += match->debugDump(l, b) + "\n";
        }
    }
    if (const OpIntersection* intersection = findIntersection(id))
        s += intersection->debugDump(l, b) + "\n";
    if (const OpLimb* limb = findLimb(id))
        s += limb->debugDump(l, b) + "\n";
    if (std::vector<const OpIntersection*> merges = findMerge(id); merges.size()) {
        for (auto merge : merges) {
            s += merge->debugDump(l, b) + "\n";
        }
    }
    if (std::vector<const OpIntersection*> uSects = findSectUnsectable(id); uSects.size()) {
        for (auto uSect : uSects) {
            s += uSect->debugDump(l, b) + "\n";
        }
    }
    if (std::vector<const OpEdge*> uEdges = findEdgeUnsectable(id); uEdges.size()) {
        for (auto uEdge : uEdges) {
            s += uEdge->debugDump(l, b) + "\n";
        }
    }
    if (const OpSegment* segment = findSegment(id))
        s += segment->debugDump(l, b) + "\n";
    debugPopMatching(s, '\n');
    return s;
}
#endif

std::string Distance::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "{edge[" + debugDumpID() + "] ";
    if (!OpMath::IsDebugNaN(cept))
        s += debugValue(l, b, "cept", cept) + " ";
    if (!OpMath::IsDebugNaN(edgeInsideT))
        s += debugValue(l, b, "edgeInsideT", edgeInsideT) + " ";
    if (RayOrder::uninitialized != rayOrder)
		s += "rayOrder:" + RayOrderName(rayOrder) + " ";
    s += BoolToStr(l, reversed, "reversed", "r");
    s += BoolToStr(l, dependent, "dependent", "d");
    s += BoolToStr(l, over, "over", "o");
    debugPopMatching(s, ' ');
    s += "}";
    return s;
}

std::string Distance::debugDumpID() const {
    std::string s;
    s += STR(edge->id);
    return s;
}

std::string EdgeDist::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (isSet()) 
		s += "opp:" + opp.debugDump(l, b) + " ";
    s += debugValue(DebugLevel::error, b, "dist", dist) + " ";
    return s;
}

std::string EdgePal::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "edge[" + debugDumpID() + "] ";
    if (DebugLevel::file != l) {
	    if (DebugLevel::detailed == l && edge->segment)
		    s += "seg[" + STR(edge->segment->id) + "] ";
	    if (edge->segment)
		    s += "contour[" + STR(edge->segment->contour->id) + "] ";
    }
    if (unsectID)
		s += "unsectID:" + STR(unsectID) + " ";
    s += BoolToStr(l, reversed, "reversed", "r");
    if (DebugLevel::detailed == l)
		s += edge->debugDumpWinding() + " ";
    return debugPopMatching(s, ' ');
}

std::string EdgePal::debugDumpID() const {
    std::string s;
    s += STR(edge->id);
    return s;
}

std::string EdgeRun::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "edgePtT:" + edgePtT.debugDump(l, b) + " ";
    s += "oppPtT:" + oppPtT.debugDump(DebugLevel::error, b) + " ";
    s += debugErrorValue(l, b, "oppDist", oppDist) + " ";
    if (LimitFrom::yes == fromFoundT)
        s += "fromFoundT ";
    if (byZero)
        s += "byZero ";
 //   OP_DEBUG_CODE(s += "debugBetween:" + STR(debugBetween) + " ");
#if OP_DEBUG_MAKER
    s += debugSetMaker.debugDump();
#endif
    return s;
}

std::string FoundEdge::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l)
         s += debugDumpID() + " ";
    else
         s += edge->debugDump(l, b) + "\n";
    bool dumpDetails = DebugLevel::detailed == l || DebugLevel::file == l;
    if (dumpDetails && !distSq)
        s += "distSq:" + STR(distSq) + " ";
    if (dumpDetails && index >= 0)
        s += "index:" + STR(index) + " ";
    if (whichEnd != EdgeMatch::none)
        s += "whichEnd:" + EdgeMatchName(whichEnd) + " ";
    if (dumpDetails && connects)
        s += "connects ";
    if (dumpDetails && loops)
        s += "loops ";
    if (dumpDetails && ChopUnsortable::none != chop)
        s += "chopUnsortable:" + ChopUnsortableName(chop) + " ";
    return s;
}

std::string FoundEdge::debugDumpID() const {
    std::string s;
    s += STR(edge->id);
    return s;
}

std::string FoundLimit::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (parentEdge)
        s += "parentEdge:" + STR(parentEdge->id) + " ";
    if (parentOpp)
        s += "parentOpp:" + STR(parentOpp->id) + " ";
    s += "segPtT:" + segPtT.debugDump(l, b) + " ";
    s += "oppPtT:" + oppPtT.debugDump(l, b) + " ";
    if (LimitFrom::yes == fromFoundT)
        s += "fromFoundT ";
    if (Unordered::yes == oppOutOfOrder)
        s += "oppOutOfOrder ";
    if (LimitUsed::yes == used)
        s += "used ";
    if (LimitMatch::yes == match)
        s += "match ";
    if (LimitSwapped::yes == swapped)
        s += "swapped ";
    if (LimitBettered::yes == bettered)
        s += "bettered ";
    if (LimitLine::yes == edgeLine)
        s += "edgeLine ";
    if (LimitLine::yes == oppLine)
        s += "oppLine ";
#if OP_DEBUG_MAKER
    s += "debugMaker:" + debugMaker.debugDump() + " ";
#endif
    s.pop_back();
    return s;
}

std::string FoundLimits::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file != l) {
        if (i.size())
            s += "limits:" + STR(i.size()) + "\n";
        for (const auto& limit : i) {
            s += limit.debugDump(l, b) + "\n";
        }
        if (snips.size())
            s += "snip:" + STR(snips.size()) + "\n";
        for (const auto& snip : snips) {
            s += snip.debugDump(l, b) + "\n";
        }
    } else {
        DEBUG_DUMP_FIRST_VECTOR(i);
        DEBUG_DUMP_VECTOR(i, lastSnips);
        DEBUG_DUMP_VECTOR(lastSnips, snips);
        ASSERT_ORDERED(snips, cc);
        DEBUG_DUMP_OPTIONAL_POS_VALUE(cc, unique);
        DEBUG_DUMP_BOOL(unique, smSegT);
        DEBUG_DUMP_BOOL(smSegT, lgSegT);
        DEBUG_DUMP_BOOL(lgSegT, smOppT);
        DEBUG_DUMP_BOOL(smOppT, lgOppT);
    }
    return s;
}

std::string HullSect::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l)
        s += "opp:" + STR(opp ? opp->id : 0) + " ";
    else if (opp)
        s += "[" + STR(opp->id) + "] ";
    s += "sect:" + sect.debugDump(l, b) + " ";
	if (oppDist.isSet())
		s += "oppDist:" + oppDist.debugDump(l, b) + " ";
    s += "type:" + SectTypeName(type) + " ";
    return debugPopMatching(s, ' ');
}

std::string LinePts::debugDump(DebugLevel l, DebugBase b) const {
    return "0:" + pts[0].debugDump(l, b) + ", 1:" + pts[1].debugDump(l, b);
}

std::string LinkUps::debugDump(DebugLevel li, DebugBase b) const {
    std::string s;
    int index = 0;
    for (const auto& linkup : l) {
        int linkupIndex = index++;
        if (!linkup)
           return "!!! expected non-null linkup\n";
        int count = 0;
        auto next = linkup;
        auto looped = linkup->debugIsLoop(EdgeMatch::start, LeadingLoop::in);
        if (!looped)
            looped = linkup->debugIsLoop(EdgeMatch::end, LeadingLoop::in);
        bool firstLoop = false;
        while (next) {
            if (looped == next) {
                if (firstLoop)
                    break;
                firstLoop = true;
            }
            next = next->nextEdge;
            ++count;
        }
        auto prior = linkup;
        int priorCount = 0;
        while (prior && prior->priorEdge) {
            prior = prior->priorEdge;
            ++priorCount;
            if (looped == prior) {
                if (firstLoop)
                    break;
                firstLoop = true;
            }
        }
        if (looped && count != priorCount)
            OpDebugOut("!!! linkup " + STR(linkupIndex) + "[" + STR(linkup->id) + "]"
                    " with unexpected tail: count " + STR(count) 
                    + "!= priorCount " + STR(priorCount) + "\n");
        if (looped)
            priorCount = 0;
        if (debugIfMatching(s, '\n'))
            s += "\n";
        s += "[" + STR(linkupIndex) + "] linkup count:" + STR(count + priorCount);
        if (priorCount && !looped)
            s += " (prior count:" + STR(priorCount) + ")";
        if (looped)
            s += " loop";
        s += " bounds:" + linkup->linkBounds.debugDump(li, b);
		s += DebugLevel::brief == li ? " " : "\n";
        if (!looped) {
            if (1 == count + priorCount && !linkup->lastEdge)
                s += "p/n/l:-/-/- ";
            else if (priorCount) {
				if (DebugLevel::brief == li)
					s += " prior:" + STR(prior->id) + " ";
				else
					s += " prior:\n" + prior->debugDump(li, b);
                prior = linkup;
                while ((prior = prior->priorEdge) && count--)
                    s += STR(prior->id) + " ";
                s += "\n next:";
				if (DebugLevel::brief != li)
					s += "\n";
            }
        }
        next = linkup->nextEdge;
		if (DebugLevel::brief == li)
			s += "[" + STR(linkup->id) + " ";
		else
			s += " " + linkup->debugDump(li, b);
        bool hasNext = false;
        while (next) {
            if (looped == next)
                break;
            if (next == linkup->lastEdge) {
                OP_ASSERT(!next->nextEdge || looped);
				if (DebugLevel::brief == li)
					s += STR(next->id) + " ";
				else
					s += "\n " + next->debugDump(li, b);
                hasNext = false;
            } else {
                if (!hasNext && DebugLevel::brief != li)
                    s += "\n      ";
                s += STR(next->id) + " ";
                hasNext = true;
            }
            next = next->nextEdge;
        }
		if (DebugLevel::brief == li) {
            debugPopMatching(s, ' ');
			s += "] ";
		}
        if (hasNext)
            s += "(loop)";
        s += "\n";
    }
    return s;
}

std::string MatchReverse::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "match:" + MatchEndsName(match) + " ";
    if (reversed)
        s += "reversed ";
    return debugPopMatching(s, ' ');
}

void OpContext::dumpString(const std::string& s) const {
    // special descriptions are also filenames, to allow verifying that dump works correctly
    std::string tmpFilePath = dmpFileToPath("tmp_" + debugFilename);
    FILE* file = fopen(tmpFilePath.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + tmpFilePath + " to write\n");
        return;
    }
    std::string fS = stringFormat(s, 133);  // accomodate op debug bitmap (66 bytes x 2)
    fwrite(&fS[0], 1, fS.size(), file);
    fclose(file);
    std::string filePath = dmpFileToPath(debugFilename);
    if (rename(tmpFilePath.c_str(), filePath.c_str()))  // so debugger doesn't read partial file
        OpDebugOut("could not rename " + tmpFilePath + " to " + filePath + "\n");
}

void OpContext::dumpBaseFile(DumpRaster dumpRaster) const {
    std::string s = debugDump(DebugLevel::file, DebugBase::hex, dumpRaster);
    dumpString(s);
#if OP_DEBUG_VALIDATE && OP_DEBUG_DUMP
	OpContext* fileContext = fromFile(debugFilename);
    std::string copy = fileContext->debugDump(DebugLevel::file, DebugBase::hex, dumpRaster);
    copy = stringFormat(copy, 133);
    std::string orig = dmpFileToStr(debugFilename);
    if (orig != copy) {
        std::string copyPath = dmpFileToPath("DumpCopy.txt");
        FILE* file = fopen(copyPath.c_str(), "w");
        fwrite(&copy[0], 1, copy.size(), file);
        fclose(file);

        OpDebugOut("!!! " + debugFilename + " != DumpCopy.txt\n");
#ifndef _WIN32
        std::string filePath = dmpFileToPath(debugFilename);
        std::string bashStr = "bash -c 'diff " + filePath + " " + copyPath + "'";
        system(bashStr.c_str());
#else
        const char* o = &orig.front();
        const char* c = &copy.front();
        auto lineCount = [](const std::string& str, char lineChar, const char* label) {
            const char* s = &str.front();
            int count = 0;
            int line = 0;
            while (s <= &str.back()) {
                count += lineChar == *s++;
            }
            if (count)
                OpDebugOut(std::string(label) + ":" + STR(count) + " ");
            return count;
        };
        int oCrCount = lineCount(orig, '\r', "orig CR");
        int cCrCount = lineCount(copy, '\r', "copy CR");
        int oLfCount = lineCount(orig, '\n', "orig LF");
        int clfCount = lineCount(copy, '\n', "copy LF");
        auto showCR = [](const std::string& str) {
            const char* s = &str.front();
            int line = 0;
            while (s <= &str.back()) {
                if ('\r' == *s)
                    OpDebugOut("CR on line " + STR(line) + "\n");
                line += '\n' == *s++;
            }
        };
        if (oCrCount)
            showCR(orig); 
#endif
        OpNop();
    }
    delete fileContext;
#endif
}

void OpContext::dumpFile(std::string description, DumpRaster dumpRaster) {
    if (!debugData.runOneFile || debugData.defeatDumps)
        return;
    // !!! not sure that this is always the right thing to do, but ...
    //     remove all older files so that the debugger does not see a mix of old and new data
    //     Note that DumpCopy.txt is not deleted because the debugger doesn't read it, and in
    //     case it's useful to see the last sucessful dump.
    if (0 == debugData.dumpIndex) {
        for (;;) {
            debugFilename = DumpFile + STR(++debugData.dumpIndex) + ".txt";
            std::string filePath = dmpFileToPath(debugFilename);
            if (!std::filesystem::exists(filePath))
                break;
            std::filesystem::remove(filePath);
        }
        debugData.dumpIndex = 0;
    }
    debugFilename = DumpFile + STR(++debugData.dumpIndex) + ".txt";
    OP_ASSERT(description.size() <= 80);  // !!! forced by line wrapping, could be relaxed
    debugDescription = description;
    if (debugContextCallbacks.debugDumpOutFuncPtr)
        debugOutPath = (*debugContextCallbacks.debugDumpOutFuncPtr)((ContextPtr) this);
    dumpBaseFile(dumpRaster);
}

std::string OpContext::debugDump(DebugLevel l, DebugBase b) const {
    return debugDump(l, b, DumpRaster::no);
}

std::string OpContext::debugDump(DebugLevel l, DebugBase b, DumpRaster dumpRaster) const {
    std::string s;
    static_assert(0 == offsetof(OpContext, callbacks));
    s += "callbacks:" + STR(callbacks.size()) + "\n";
    for (auto& callback : callbacks) {
        static_assert(0 == offsetof(PathOpsV0Lib::CurveCallbacks, axisTFuncPtr));
        s += debugFindTag(reinterpret_cast<DebugFunction>(callback.axisTFuncPtr));
	    DEBUG_FIND_TAG(callback, axisTFuncPtr,          rotateTFuncPtr);
	    DEBUG_FIND_TAG(callback, rotateTFuncPtr,        curveHullFuncPtr); 
	    DEBUG_FIND_TAG(callback, curveHullFuncPtr,      curveIsFiniteFuncPtr);
	    DEBUG_FIND_TAG(callback, curveIsFiniteFuncPtr,  curveIsLineFuncPtr);
	    DEBUG_FIND_TAG(callback, curveIsLineFuncPtr,    setBoundsFuncPtr);
	    DEBUG_FIND_TAG(callback, setBoundsFuncPtr,      curvePinFuncPtr);
	    DEBUG_FIND_TAG(callback, curvePinFuncPtr,       curveTangentFuncPtr);
	    DEBUG_FIND_TAG(callback, curveTangentFuncPtr,   curvesEqualFuncPtr);
	    DEBUG_FIND_TAG(callback, curvesEqualFuncPtr,    ptAtTFuncPtr);
	    DEBUG_FIND_TAG(callback, ptAtTFuncPtr,          ptCountFuncPtr);
	    DEBUG_FIND_TAG(callback, ptCountFuncPtr,        rotateFuncPtr);
	    DEBUG_FIND_TAG(callback, rotateFuncPtr,         subDivideFuncPtr);
	    DEBUG_FIND_TAG(callback, subDivideFuncPtr,      xyAtTFuncPtr);
	    DEBUG_FIND_TAG(callback, xyAtTFuncPtr,          curveReverseFuncPtr);
	    DEBUG_FIND_TAG(callback, curveReverseFuncPtr,   cutFuncPtr);
	    DEBUG_FIND_TAG(callback, cutFuncPtr,            interceptFuncPtr);
	    DEBUG_FIND_TAG(callback, interceptFuncPtr,      normalLimitFuncPtr);
	    DEBUG_FIND_TAG(callback, normalLimitFuncPtr,    maxAlternateEndFuncPtr);
	    DEBUG_FIND_TAG(callback, maxAlternateEndFuncPtr, smallTFuncPtr);
	    DEBUG_FIND_TAG(callback, smallTFuncPtr,         maxCutFuncPtr);
	    DEBUG_FIND_TAG(callback, maxCutFuncPtr,         closeEndFuncPtr);
	    DEBUG_FIND_TAG(callback, closeEndFuncPtr,         rayEndFuncPtr);
        static_assert(offsetof(PathOpsV0Lib::CurveCallbacks, rayEndFuncPtr) 
                + sizeof(callback.rayEndFuncPtr) == sizeof(callback));
    }
    ASSERT_ORDERED(callbacks, userData);
#if 0  // don't serialize user data
    s += "userData:" + STR(userData.size()) + "\n";
    for (auto& data : userData) {
        s += "offset:" + STR(callerStorage->dumpOffset(data)) + " ";
        s += "size:" + STR(data.size) + " ";
        s += "type:" + STR((int) data.type) + " ";
    }
    debugPopMatching(s, ' ');
    s += "\n";
#endif
    ASSERT_ORDERED(userData, nativeCurveTypes);
    s += "nativeCurveTypes:" + STR(nativeCurveTypes.size()) + " ";
    for (int nativeCurveType : nativeCurveTypes) {
        s += STR(nativeCurveType) + " ";
    }
    s += "\n";
    ASSERT_ORDERED(nativeCurveTypes, contextCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::ContextCallbacks, curveOutputFuncPtr));
#if 0  // omit curve output function
    s += debugFindTag(reinterpret_cast<DebugFunction>(contextCallbacks.curveOutputFuncPtr));
#endif
#if 0  // skip find tag for best loop function
	DEBUG_FIND_TAG(contextCallbacks, curveOutputFuncPtr, emptyCallerPathFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, emptyCallerPathFuncPtr, bestLoopFuncPtr);
#else
	ASSERT_SERIAL(contextCallbacks, curveOutputFuncPtr, emptyCallerPathFuncPtr);
	ASSERT_SERIAL(contextCallbacks, emptyCallerPathFuncPtr, bestLoopFuncPtr);
#endif
	DEBUG_FIND_TAG(contextCallbacks, bestLoopFuncPtr, setLineTypeFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, setLineTypeFuncPtr, maxAngleMatchFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxAngleMatchFuncPtr, maxAngleSweepFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxAngleSweepFuncPtr, maxSplitFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxSplitFuncPtr, maxBoundedEdgeFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxBoundedEdgeFuncPtr, maxSignSwapFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxSignSwapFuncPtr, maxTSlopFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxTSlopFuncPtr, maxSplitBiasFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxSplitBiasFuncPtr, maxOverlapFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxOverlapFuncPtr, maxUnsectableFuncPtr);
    DEBUG_FIND_TAG(contextCallbacks, maxUnsectableFuncPtr, maxDistFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxDistFuncPtr, maxDeepFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxDeepFuncPtr, maxShallowFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxShallowFuncPtr, maxSplitsFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxSplitsFuncPtr, maxMarginFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxMarginFuncPtr, rootAdjustFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, rootAdjustFuncPtr, hullNudgeFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, hullNudgeFuncPtr, maxUnsectableTFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxUnsectableTFuncPtr, maxUnsectDistFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxUnsectDistFuncPtr, maxCheckSplitFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxCheckSplitFuncPtr, maxLimbsFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxLimbsFuncPtr, maxLoopsFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxLoopsFuncPtr, windingBytesFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, windingBytesFuncPtr, maxGapFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxGapFuncPtr, linkupScaleFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, linkupScaleFuncPtr, enabledRatioFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::ContextCallbacks, enabledRatioFuncPtr) 
            + sizeof(contextCallbacks.enabledRatioFuncPtr) == sizeof(contextCallbacks));
    ASSERT_ORDERED(contextCallbacks, windingCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::WindingCallbacks, windingAddFuncPtr));
	s += debugFindTag(reinterpret_cast<DebugFunction>(windingCallbacks.windingAddFuncPtr));
	DEBUG_FIND_TAG(windingCallbacks, windingAddFuncPtr, windingKeepFuncPtr);
	DEBUG_FIND_TAG(windingCallbacks, windingKeepFuncPtr, windingSubtractFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingSubtractFuncPtr, windingWoundFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingWoundFuncPtr, windingVisibleFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingVisibleFuncPtr, windingZeroFuncPtr);
	DEBUG_FIND_TAG(windingCallbacks, windingZeroFuncPtr, windingIntersectFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingIntersectFuncPtr, windingShortFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingShortFuncPtr, windingShortAllFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingShortAllFuncPtr, windingLoopFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::WindingCallbacks, windingLoopFuncPtr) 
            + sizeof(windingCallbacks.windingLoopFuncPtr) == sizeof(windingCallbacks));
    // out of order ... but callbacks must be set before curves and windings are read
    if (!debugCallbacks.empty())
        s += debugCallbacksDump(debugCallbacks, l, b);
    s += debugContextCallbacksDump(debugContextCallbacks, l, b);
    ASSERT_ORDERED(windingCallbacks, errorHandler);  // omit errorHandler
    ASSERT_ORDERED(errorHandler, sortedContours);
    if (!sortedContours.empty()) {
        s += "sortedContours:" + STR(sortedContours.size()) + " ";
        for (auto sortedContour : sortedContours) {
            s += STR(sortedContour->id) + " ";
        }
        s += "\n";
    }
    ASSERT_ORDERED(sortedContours, curveDataStorage);
    if (curveDataStorage) {
        s += "curveDataStorage:";
        s += curveDataStorage->debugDump(l, b) + "\n";
    }
    ASSERT_ORDERED(curveDataStorage, ccStorage);
    if (ccStorage)
        s += ccStorage->debugDump(l, b) + "\n";
    ASSERT_ORDERED(ccStorage, contourStorage);
    if (contourStorage)
        s += contourStorage->debugDump(l, b) + "\n";
    ASSERT_ORDERED(contourStorage, contours);
    if (!contours.empty()) {
        s += "contours:" + STR(contours.size()) + " ";
        for (auto contour : contours) {
            s += STR(contour->id) + " ";
        }
        s += "\n";
    }
    ASSERT_ORDERED(contours, fillerStorage);
    if (fillerStorage)
        s += fillerStorage->debugDump(l, b) + "\n";
    ASSERT_ORDERED(fillerStorage, sectStorage);
    if (sectStorage)
        s += sectStorage->debugDump(l, b) + "\n";
    ASSERT_ORDERED(sectStorage, limbStorage);
    if (limbStorage && limbStorage->debugCount())
        s += limbStorage->debugDump(l, b) + "\n";
    ASSERT_ORDERED(limbStorage, limbCurrent);  // omit limbCurrent
    ASSERT_ORDERED(limbCurrent, callerStorage);
    if (callerStorage) {
        s += "callerStorage:";
        s += callerStorage->debugDump(l, b) + "\n";
    }
    ASSERT_ORDERED(callerStorage, maxBounds);
    if (maxBounds.isFinite()) {
        s += "maxBounds:";
        s += maxBounds.debugDump(l, b) + "\n"; 
    }
    ASSERT_ORDERED(maxBounds, threshold);
    s += "threshold:" + threshold.debugDump(DebugLevel::error, b) + " ";
    ASSERT_ORDERED(threshold, thresholdLength);
    if (!OpMath::IsDebugNaN(thresholdLength))
        s += debugValue(DebugLevel::error, b, "thresholdLength", thresholdLength) + " ";
    ASSERT_ORDERED(thresholdLength, error);
    if (PathOpsV0Lib::ContextError::none != error)
        s += "error:" + PathOpsV0Lib::contextErrorName(error) + "\n";
    ASSERT_ORDERED(error, uniqueID);
    s += "uniqueID:" + STR(uniqueID) + " ";
    DEBUG_DUMP_BOOL(uniqueID, initialized);
    DEBUG_DUMP_BOOL(initialized, allDiscarded);
    DEBUG_DUMP_BOOL(allDiscarded, allKept);
    DEBUG_DUMP_BOOL(allKept, fatalError);
    DEBUG_DUMP_BOOL(fatalError, outputOne);
    DEBUG_DUMP_BOOL(outputOne, linkErased);
    DEBUG_DUMP_BOOL(linkErased, windingSet);
#if OP_DEBUG_VALIDATE
    ASSERT_SERIAL_OFFSET(*this, windingSet, 1, debugValidateEdgeIndex);
    s += "debugValidateEdgeIndex:" + STR(debugValidateEdgeIndex) + " ";
    ASSERT_ORDERED(debugValidateEdgeIndex, debugValidateJoinerIndex);
    s += "debugValidateJoinerIndex:" + STR(debugValidateJoinerIndex) + " ";
    ASSERT_SERIAL_OFFSET(*this, debugValidateJoinerIndex, 4, debugCallbacks);
#else
    ASSERT_ORDERED_OFFSET(windingSet, debugCallbacks, 5);
#endif
    debugPopMatching(s, ' ');
    s += "\n";
    // debugCallbacks is out of order; must be set before curves are read
    // debugContextCallbacks is out of order; must be set before windings in contours are read
    ASSERT_ORDERED(debugCallbacks, debugContextCallbacks);
    ASSERT_ORDERED(debugContextCallbacks, debugData);  // omit debugData (!!! omit most for now, may have uses...)
    if (debugData.testname.size())
        s += "debugTestname:" + debugData.testname + " ";
    ASSERT_ORDERED(debugData, debugCurveCurve);
    if (debugCurveCurve)
        s += "debugCurveCurve:" + debugCurveCurve->debugDump(l, b) + "\n";
    ASSERT_ORDERED(debugCurveCurve, debugJoiner);
    if (debugJoiner)
        s += "debugJoiner:" + debugJoiner->debugDump(l, b) + "\n";
    ASSERT_ORDERED(debugJoiner, debugTree);
    if (debugTree)
        s += "debugTree:" + debugTree->debugDump(l, b) + "\n";
    ASSERT_ORDERED(debugTree, debugErasures);
    if (debugErasures) {
        s += "debugErasures:" + STR(debugErasures->size()) + "[";
        for (OpEdge* edge : *debugErasures) {
            s += STR(edge->id) + " ";
        }
        debugPopMatching(s, ' ');
        s += "]\n";
    }
    ASSERT_ORDERED(debugErasures, debugErrorID);
    if (debugErrorID)
        s += "debugErrorID:" + STR(debugErrorID) + " ";
    ASSERT_ORDERED(debugErrorID, debugOppErrorID);
    if (debugOppErrorID)
        s += "debugOppErrorID:" + STR(debugOppErrorID) + " ";
    ASSERT_ORDERED(debugOppErrorID, debugExpect);
    s += "debugExpect:" + OpDebugExpectName(debugExpect) + " ";
	DEBUG_DUMP_BOOL(debugExpect, debugInPathOps);
	DEBUG_DUMP_BOOL(debugInPathOps, debugInClearEdges);
	DEBUG_DUMP_BOOL(debugInClearEdges, debugCheckLastEdge);
	DEBUG_DUMP_BOOL(debugCheckLastEdge, debugFailOnEqualCepts);
    ASSERT_ORDERED(debugFailOnEqualCepts, debugFilename);  // omit for now
    ASSERT_ORDERED(debugFilename, debugDescription);
    if (!debugDescription.empty())
        s += "\ndebugDescription:" + debugDescription + "\n";
    ASSERT_ORDERED(debugDescription, debugOutPath);
    if (!debugOutPath.empty()) {
        s += "debugOutPath:\n";
        s += debugOutPath;
        s += "\n:debugOutPath\n";
    }
#if OP_DEBUG_DUMP
    ASSERT_ORDERED(debugOutPath, debugDumpErasures);  // omit for now
    ASSERT_ORDERED(debugDumpErasures, debugDumpInit);  // omit for now
#endif
#if OP_TEST_RASTER
//    ASSERT_ORDERED_OFFSET(debugDumpInit, rasterStorage, 7);  // !!! dump may not be defined
// only dump raster winding storage at first and last
    ASSERT_ORDERED(rasterStorage, debugRaster);
    if (debugRaster && DumpRaster::yes == dumpRaster)
        s += "debugRaster:" + debugRaster->debugDump(l, b) + "\n";
#endif
    return s;
}

const OpEdge* OpContext::debugFindEdge(int id) const {
    for (OpContour* contour : contours) {
        for (const auto& seg : contour->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.id == id)
                    return &edge;
            }
        }
    }
	if (fillerStorage) {
        int index = 0;
        while (OpEdge* edge = fillerStorage->edgeIndex(index++)) {
            if (edge->id == id)
                return edge;
        }
	}
	if (ccStorage) {
        int index = 0;
        while (OpEdge* edge = ccStorage->edgeIndex(index++)) {
            if (edge->id == id)
                return edge;
        }
	}
    return nullptr;
}

const OpSegment* OpContext::debugFindSegment(int id) const {
    for (OpContour* contour : contours) {
        for (const auto& seg : contour->segments) {
            if (seg.id == id)
                return &seg;
        }
    }
    return nullptr;
}

const OpLimb& OpContext::debugNthLimb(int index) const {
    OpLimbStorage* saveCurrent = limbCurrent;
    OpContext* writeable = const_cast<OpContext*>(this);
    const OpLimb& result = writeable->nthLimb(index);
    writeable->limbCurrent = saveCurrent;
    return result;
}

std::string OpContour::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "contour[" + STR(id) + "] ";
#if 0  // disable until we need it
    if (DebugLevel::detailed == l) {
		auto contourExtra = debugCallbacks.debugDumpContourExtraFuncPtr;
		if (contourExtra)
			s += (*contourExtra)(debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], l, b) + " ";
	}
#endif
    static_assert(0 == offsetof(OpContour, segments));
    s += "segments:" + STR(segments.size()) + (DebugLevel::detailed == l ? "\n" : " ");
    if (DebugLevel::detailed != l && DebugLevel::file != l) {
        s += "[";
        for (auto& segment : segments)
            s += STR(segment.id) + " ";
        debugPopMatching(s, ' ');
        s += "] ";
    } else {
		// limit segment to id / curve / sects / edges / winding / disabled
		// !!! add segment filter ala edges?  refactor this to call common code?
        for (auto& segment : segments) {
			s += segment.debugDump(l, b);
			s += "\n";
		}
    }
	std::string closeBracket = DebugLevel::detailed == l ? "]\n" : "] ";
    ASSERT_ORDERED(segments, sorted);
	if (!sorted.empty()) {
		s += "sorted:" + STR(sorted.size()) + "[";
		for (OpSegment* seg : sorted)
			s += STR(seg->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
	if (DebugLevel::detailed != l)
		s += "\n  ";
    ASSERT_ORDERED(sorted, overlaps);
	if (!overlaps.empty()) {
		s += "overlaps:" + STR(overlaps.size()) + "[";
		for (OpContour* member : overlaps)
			s += STR(member->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(overlaps, merges);
	if (!merges.empty()) {
		s += "merges:" + STR(merges.size()) + "[";
		for (OpContour* member : merges)
			s += STR(member->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(merges, inX);
	if (!inX.empty()) {
		s += "inX:" + STR(inX.size()) + "[";
		for (OpEdge* e : inX)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(inX, inY);
	if (!inY.empty()) {
		s += "inY:" + STR(inY.size()) + "[";
		for (OpEdge* e : inY)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(inY, byArea);
	if (!byArea.empty()) {
		s += "byArea:" + STR(byArea.size()) + "[";
		for (OpEdge* e : byArea)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(byArea, unsectByArea);
	if (!unsectByArea.empty()) {
		s += "unsectByArea:" + STR(unsectByArea.size()) + "[";
		for (OpEdge* e : unsectByArea)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(unsectByArea, coinPals);
	if (!coinPals.empty()) {
		s += "coinPals:" + STR(coinPals.size()) + "[";
		for (OpEdge* e : coinPals)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(coinPals, disabledBackwards);
	if (!disabledBackwards.empty()) {
		s += "disabledBackwards:" + STR(disabledBackwards.size()) + "[";
		for (OpEdge* e : disabledBackwards)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(disabledBackwards, disabledCenterless);
	if (!disabledCenterless.empty()) {
		s += "disabledCenterless:" + STR(disabledCenterless.size()) + "[";
		for (OpEdge* e : disabledCenterless)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(disabledCenterless, disabledEdges);
	if (disabledEdges.size()) {
		s += "disabledEdges:" + STR(disabledEdges.size()) + "[";
		for (OpEdge* e : disabledEdges)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(disabledEdges, disabledPals);
	if (disabledPals.size()) {
		s += "disabledPals:" + STR(disabledPals.size()) + "[";
		for (OpEdge* e : disabledPals)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(disabledPals, smallEdges);
	if (smallEdges.size()) {
		s += "smallEdges:" + STR(smallEdges.size()) + "[";
		for (OpEdge* e : smallEdges)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(smallEdges, unsortables);
	if (unsortables.size()) {
		s += "unsortables:" + STR(unsortables.size()) + "[";
		for (OpEdge* e : unsortables)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(unsortables, windingStorage);
    if (DebugLevel::file == l) {
        s += "windingStorage:" + STR(windingStorage.size()) + " ";
        s += OpDebugDumpByteArray(&windingStorage.front(), windingStorage.size()) + " ";
    } else {
        s += "winding(";
	    auto debugImageOut = context->debugContextCallbacks.debugImageWindingOutFuncPtr;
        if (debugImageOut) {
            PathOpsV0Lib::Winding winding { (ContourPtr) this, (void*) &windingStorage.front(), 
                    windingStorage.size() };
            s += (*debugImageOut)(winding) + ") ";
        } else
            s += "?) ";
    }
    ASSERT_ORDERED(windingStorage, linkups);
	if (linkups.l.size()) {
		s += "linkups:" + STR(linkups.l.size()) + "[";
		for (OpEdge* e : linkups.l)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    ASSERT_ORDERED(linkups, endLinks);
	if (endLinks.l.size()) {
		s += "endLinks:" + STR(endLinks.l.size()) + "[";
		for (OpEdge* e : endLinks.l)
			s += STR(e->id) + " ";
        debugPopMatching(s, ' ');
		s += closeBracket;
	}
    DEBUG_DUMP_OPTIONAL_STRUCT(endLinks, overlapBounds, overlapBounds.isFinite());
    DEBUG_DUMP_OPTIONAL_STRUCT(overlapBounds, bounds, bounds.isFinite());
    ASSERT_ORDERED(bounds, context);  // omit context
    DEBUG_DUMP_OPTIONAL_ID(context, overlapOwner);
    DEBUG_DUMP_REQUIRED_VALUE(overlapOwner, contextIndex);
    ASSERT_ORDERED(contextIndex, id);  // id written up front
    DEBUG_DUMP_OPTIONAL_VALUE(id, treeID);
    DEBUG_DUMP_BOOL(treeID, backwardsBuilt);
    DEBUG_DUMP_BOOL(backwardsBuilt, centerlessBuilt);
    DEBUG_DUMP_BOOL(centerlessBuilt, coinPalsBuilt);
    DEBUG_DUMP_BOOL(coinPalsBuilt, disabledBuilt);
    DEBUG_DUMP_BOOL(disabledBuilt, hasPals);
    DEBUG_DUMP_BOOL(hasPals, palsBuilt);
    DEBUG_DUMP_BOOL(palsBuilt, disabled);
    DEBUG_DUMP_BOOL(disabled, overlapsMerged);
    DEBUG_DUMP_BOOL(overlapsMerged, segEndsMerged);
    DEBUG_DUMP_BOOL(segEndsMerged, segMerged);
    DEBUG_DUMP_BOOL(segMerged, debugEmpty);
#if OP_DEBUGGER || OP_TEST
    ASSERT_SERIAL_OFFSET(*this, debugEmpty, 1, debugCurveData);
    if (!debugCurveData.empty()) {
		s += "debugCurveData:" + STR(debugCurveData.size()) + "[";
		for (int index = 0; index < (int) debugCurveData.size(); ++index) {
            const PathOpsV0Lib::DebugCurveData& dcd = debugCurveData[index];
            if (DebugLevel::file == l)
                s += DebugCurveData_DebugDump(context, dcd);
            else {
                std::vector<float> extrema;
                PathOpsV0Lib::Curve c = debugCurve(index, &extrema);
                s += Curve_DebugDump(c, l, b) + " ";
                if (!extrema.empty()) {
                    s += "extrema:" + STR(extrema.size()) + "[";
                    for (float extreme : extrema) {
                        s += STR(extreme) + " ";
                    }
                    debugPopMatching(s, ' ');
		            s += closeBracket;
                }
            }
        }
        debugPopMatching(s, ' ');
		s += closeBracket;
    }
    ASSERT_ORDERED(debugCurveData, debugWinding);
#endif
    return debugPopMatching(s, ' ');
}

int OpContourStorage::debugCount() const {
    int result = used;
    OpContourStorage* block = next;
    while (block) {
        result += block->used;
        block = block->next;
    }
    return result;
}

std::string OpContourStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    int count = debugCount();
    if (!count)
        return s;
    s += "contourStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        debugPopMatching(s, ' ');
        s += "]";
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        debugPopMatching(s, '\n');
    }
    return s;
}

// this walks 'backwards', from oldest to newest
OpContour* OpContourStorage::debugIndex(int contourIndex) const {
    const OpContourStorage* block = this;
    // build an array from that can be walked from back to front
    std::vector<const OpContourStorage*> blocks;
    do {
	    blocks.push_back(block);
        block = block->next;
    } while (block);
    // walk the array of blocks in the order they were allocated (back to front)
    for (size_t index = blocks.size(); index-- != 0; ) {
        block = blocks[index];
        if (contourIndex < block->used)
            return const_cast<OpContour*>(&block->storage[contourIndex]);
        contourIndex -= block->used;
    }
    return nullptr;
}

std::string OpCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::normal == l)
        s += "{";
    s += Curve_DebugDump(c, l, b) + " ";
    if (DebugLevel::brief != l) {
        if (start != c.data->start)
            s += "start:" + start.debugDump(l, b) + " ";
        if (end != c.data->end)
            s += "end:" + end.debugDump(l, b) + " ";
        if (Rotated::no != rotated)
            s += "rotated:" + RotatedName(rotated) + " ";
        if (isLineSet)
            s += "isLineSet ";
        if (isLineResult)
            s += "isLineResult ";
        if (isSmall)
            s += "isSmall ";
        if (reversed)
            s += "reversed ";
    }
    debugPopMatching(s, ' ');
    if (DebugLevel::normal == l)
        s += "}";
    return s;
}

std::string OpCurveCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    ASSERT_FIRST(context);
    if (DebugLevel::file != l) {
        DebugLevel down1 = DebugLevel::file == l ? DebugLevel::file : (DebugLevel) ((int) l - 1);
        if (!seg->edges.size())
            s += "seg:" + STR(seg->id) + " ";
        else  {
            const OpEdge* originalEdge = &seg->edges[0];
            s += "originalEdge:" + originalEdge->debugDump(down1, b) + "\n";
        }
        if (!opp->edges.size())
            s += "opp:" + STR(opp->id) + "\n";
        else {
            const OpEdge* originalOpp = &opp->edges[0];
            s += "originalOpp:" + originalOpp->debugDump(down1, b) + "\n";
        }
        std::string names[] = { "edge curves", "opp curves" };
        int count = 0;
	    for (auto edgesPtrs : { &edgeCurves, &oppCurves } ) {
            const auto& edges = *edgesPtrs;
            if (edges.c.size()) {
                s += "-- " + names[count] + ":" + STR(edges.c.size()) + " --\n";
                s += edges.debugDump(l, b) + "\n";
            }
            ++count;
        }
        if (limits.size())
            s += limits.debugDump(l, b);
   } else {
        DEBUG_DUMP_ID(context, seg);
        DEBUG_DUMP_ID(seg, opp);
        DEBUG_DUMP_ID(opp, parentEdge);
        DEBUG_DUMP_ID(parentEdge, parentOpp);
        DEBUG_DUMP_STRUCT(parentOpp, edgeCurves);
        DEBUG_DUMP_STRUCT(edgeCurves, oppCurves);
        DEBUG_DUMP_STRUCT(oppCurves, limits);
        DEBUG_DUMP_STRUCT(limits, maxSplit);
        DEBUG_DUMP_STRUCT(maxSplit, maxBoundedEdge);
        DEBUG_DUMP_STRUCT(maxBoundedEdge, maxUnsectable);
        DEBUG_DUMP_REQUIRED_VALUE(maxUnsectable, endMatches);
        DEBUG_DUMP_FLOAT(endMatches, maxAngleMatch);
        DEBUG_DUMP_FLOAT(maxAngleMatch, maxAngleSweep);
        DEBUG_DUMP_FLOAT(maxAngleSweep, maxSignSwap);
        DEBUG_DUMP_FLOAT(maxSignSwap, maxSplitBias);
        DEBUG_DUMP_FLOAT(maxSplitBias, maxDist);
        DEBUG_DUMP_FLOAT(maxDist, maxEdgeTSlop);
        DEBUG_DUMP_OPTIONAL_VALUE(maxEdgeTSlop, depth);
        DEBUG_DUMP_REQUIRED_VALUE(depth, unsplitables);
        DEBUG_DUMP_REQUIRED_VALUE(unsplitables, maxCheckSplit);
        DEBUG_DUMP_REQUIRED_VALUE(maxCheckSplit, maxDeep);
        DEBUG_DUMP_REQUIRED_VALUE(maxDeep, maxShallow);
        DEBUG_DUMP_REQUIRED_VALUE(maxShallow, maxSplits);
        DEBUG_DUMP_BOOL(maxSplits, reversed);
        DEBUG_DUMP_BOOL(reversed, boundedEdgeFailed);
        DEBUG_DUMP_BOOL(boundedEdgeFailed, overlap);
        DEBUG_DUMP_BOOL(overlap, rotateFailed);
        DEBUG_DUMP_BOOL(rotateFailed, sectResult);
        DEBUG_DUMP_BOOL(sectResult, lastDepthReduced);
        DEBUG_DUMP_BOOL(lastDepthReduced, foundGap);
        DEBUG_DUMP_BOOL(foundGap, splitMid);
        DEBUG_DUMP_BOOL(splitMid, splitHullFail);
//        ASSERT_LAST_OFFSET(splitHullFail, 7);
        return s;
    }
    s += "depth:" + STR(depth) + " ";
    if (reversed)
        s += "reversed ";
    if (boundedEdgeFailed) 
        s += "boundedEdgeFailed ";
    if (overlap) 
        s += "overlap ";
    if (rotateFailed) 
        s += "rotateFailed ";
    if (sectResult) 
        s += "sectResult ";
    if (lastDepthReduced) 
        s += "lastDepthReduced ";
    if (foundGap) 
        s += "foundGap ";
    if (splitMid) 
        s += "splitMid ";
    if (splitHullFail) 
        s += "splitHullFail ";
    return debugPopMatching(s, ' ');
}

struct Field {
    const char* name;
    const char* data;
    const char* end;
};

std::string OpEdge::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::brief == l) {
        std::string master = debugDump(DebugLevel::normal, b);
        std::vector<std::string> showFields = { "edge", "startT", "endT", "curve",
            "wind", "sum", "whichEnd_impl" };
        std::vector<Field> foundFields;
        std::string s;
        for (const char* ch = &master.front(); ch < &master.back(); ) {
            if (!isalpha(*ch)) {
                OP_ASSERT(*ch <= ' ');
                ++ch;
                continue;
            }
            const char* start = ch;
            while (ch < &master.back() && isalpha(*ch))
                ++ch;
            const char* data = ch;
            int brackets = 0;
            while (ch < &master.back()) {
                brackets += '[' == *ch || '{' == *ch;
                brackets -= ']' == *ch || '}' == *ch;
                ++ch;
                if (!brackets && ' ' == *ch)
                    break;
            }
            foundFields.push_back({ start, data, ch });
        }
        for (std::string& show : showFields) {
            for (const Field& found : foundFields) {
                if (show.size() == found.data - found.name  
                        && 0 == strncmp(&show.front(), found.name, show.size()))
                    s += std::string(found.name, found.end - found.name) + " ";
            }
        }
        debugPopMatching(s, ' ');
        return s;
    }
    DebugLevel debugLevelRay = l;
    if (DebugLevel::ray == debugLevelRay)
        l = DebugLevel::normal;
    DebugLevel error = DebugLevel::file != l ? DebugLevel::error : DebugLevel::file;
    auto strLabel = [l](std::string label) {
        return debugLabel(l, label);
    };
    auto strCurve = [b, l, strLabel](std::string label, const OpCurve& c) {
        return strLabel(label) + ":" + c.debugDump(l, b) + " ";
    };
    auto strPts = [b, l, strLabel](std::string label, const LinePts& p) {
        return strLabel(label) 
                + "[" + p.pts[0].debugDump(l, b) + ", " + p.pts[1].debugDump(l, b) + "] ";
    };
    auto strEdge = [l, strLabel](EdgeFilter match, std::string label, const OpEdge* edge) {
        if (DebugLevel::file == l) {
            if (!edge)
                return std::string();
            return label + ":" + STR(edge->id) + " ";
        }
        return strLabel(label) + ":" + (edge ? STR(edge->id) : std::string("-")) + "/";
    };
    auto strFloat = [b, error](EdgeFilter match, std::string label, float t) {
        if (!OpMath::IsFinite(t))
            return std::string("");
        return debugValue(error, b, label, t) + " ";
    };
 #if 0
    auto strPoint = [b, error, strLabel](EdgeFilter match, std::string label, OpPoint pt) {
        if (!pt.isFinite())
            return std::string("");
        return strLabel(label) + pt.debugDump(error, b) + " ";
    };
#endif
    auto strPtT = [b, error, strLabel](EdgeFilter match, std::string label, OpPtT ptT) {
        if (!ptT.pt.isFinite() || !OpMath::IsFinite(ptT.t))
            return std::string("");
        return strLabel(label) + "{" + ptT.debugDump(error, b) + "} ";
    };
    auto strID = [strLabel](EdgeFilter match, std::string label, int ID) {
        if (!ID)
            return std::string("");
        return strLabel(label) + "[" + STR(ID) + "] ";
    };
    auto strBounds = [l, b, strLabel](EdgeFilter match, 
            std::string label, const OpPointBounds& ptBounds) {
        if (!ptBounds.isSet())
            return std::string("");
        return strLabel(label) + ptBounds.debugDump(l, b)+ " ";
    };
    auto strWinding = [l, b, strLabel](EdgeFilter match, std::string label,
             const OpWinding& wind) {
        if (!wind.isSet())
            return std::string("");
        return strLabel(label) + ":" + wind.debugDump(l, b) + " ";
    };
    auto strEnum = [strLabel](EdgeFilter match, std::string label,
            bool enumHasDefault, std::string enumName) {
        if (enumHasDefault)
            return std::string("");
        return strLabel(label) + ":" + enumName + " ";
    };
    std::string s = strID(EdgeFilter::id, "edge", id);
    static_assert(0 == offsetof(OpEdge, segment));
    if (segment) 
        s += strID(EdgeFilter::segment, "segment", segment->id);
    if (segment && segment->contour)
        s += strID(EF::contour, "contour", segment->contour->id);
    ASSERT_ORDERED(segment, ray);
    if (ray.distances.size()) 
        s += ray.debugDump(debugLevelRay, b) + " ";
    ASSERT_ORDERED(ray, priorEdge);
    ASSERT_ORDERED(priorEdge, nextEdge);
    ASSERT_ORDERED(nextEdge, lastEdge);
    if (priorEdge || nextEdge || lastEdge) { 
        s += strEdge(EdgeFilter::priorEdge, "prior", priorEdge);
        s += strEdge(EdgeFilter::nextEdge, "next", nextEdge);
        s += strEdge(EdgeFilter::lastEdge, "last", lastEdge);
        if (debugIfMatching(s, '/'))
            s.back() = ' ';
    }
    ASSERT_ORDERED(lastEdge, center);
	if (!center.debugIsUninitialized())
		s += strPtT(EdgeFilter::center, "center", center);
    ASSERT_SERIAL_OFFSET(*this, center, 4, curve);
    s += strCurve("curve", curve);
    ASSERT_ORDERED(curve, vertical_impl);
    ASSERT_ORDERED(vertical_impl, upright_impl);
    if (upright_impl.pts[0].isFinite() || upright_impl.pts[1].isFinite()) {
        s += strPts("upright_impl", upright_impl);
        s += strCurve("vertical_impl", vertical_impl);
    }
    ASSERT_ORDERED(upright_impl, linkBounds);
    s += strBounds(EF::linkBounds, "linkBounds", linkBounds);
    ASSERT_ORDERED(linkBounds, winding);
    s += strWinding(EdgeFilter::winding, "winding", winding);
    ASSERT_ORDERED(winding, sum);
    s += strWinding(EdgeFilter::sum, "sum", sum);
#if OP_EDGE_PAL_MANY
    ASSERT_ORDERED(sum, palMany);
    s += strWinding(EdgeFilter::palMany, "palMany", palMany);
    ASSERT_ORDERED(palMany, coinPals);
#else
    ASSERT_ORDERED(sum, coinPals);
#endif
    if (coinPals.size()) {
        s += strLabel("coinPals:") + STR(coinPals.size()) + "{";
        for (auto& cPal : coinPals) {
            s += "{opp[" + STR(cPal.opp->id) + "] coinID[" + STR(cPal.coinID) + "]} ";
        }
        debugPopMatching(s, ' ');
        s += "} ";
    }
    ASSERT_ORDERED(coinPals, unSects);
    if (unSects.size()) {
        s += strLabel("unSects:") + STR(unSects.size()) + "[";
        for (auto& uSect : unSects)
            s += STR(uSect->id) + " ";
        debugPopMatching(s, ' ');
        s += "] ";
    }
    ASSERT_ORDERED(unSects, pals);
    if (pals.size()) {
        s += strLabel("pals:") + STR(pals.size()) + "{";
        for (auto& pal : pals) {
            s += pal.debugDump(l, b) + " ";
        }
        debugPopMatching(s, ' ');
        s += "} ";
    }
    ASSERT_ORDERED(pals, hulls);
    if (hulls.h.size()) {
        s += "hulls";
        if (DebugLevel::file == l)
            s += ":" + STR(hulls.h.size()) + " ";
        else
            s += "{";  // don't abbreviate in brief
        for (auto& hs : hulls.h)
            s += hs.debugDump(l, b) + " ";
        if (DebugLevel::file != l) {
            debugPopMatching(s, ' ');
            s += "} ";
        }
    }
    ASSERT_ORDERED(hulls, startDist);
    if (startDist.debugIsSet())
        s += "startDist{" + startDist.debugDump(l, b) + "} ";
    ASSERT_ORDERED(startDist, endDist);
    if (endDist.debugIsSet())
        s += "endDist{" + endDist.debugDump(l, b) + "} ";
    ASSERT_ORDERED(endDist, startT);
    s += strFloat(EdgeFilter::startT, "startT", startT);
    ASSERT_ORDERED(startT, endT);
    s += strFloat(EdgeFilter::endT, "endT", endT);
    ASSERT_ORDERED(endT, id);
    ASSERT_ORDERED(id, ccUnsectID);
    s += strID(EF::ccUnsectID, "ccUnsectID", ccUnsectID);
    ASSERT_ORDERED(ccUnsectID, whichEnd_impl);
    s += strEnum(EF::whichEnd_impl, "whichEnd", EdgeMatch::none == which(), EdgeMatchName(which()));
    ASSERT_ORDERED(whichEnd_impl, rayFail);
    s += strEnum(EF::rayFail, "rayFail", EdgeFail::none == rayFail, EdgeFailName(rayFail));
    ASSERT_ORDERED(rayFail, windZero);
    s += strEnum(EF::windZero, "windZero", WindZero::unset == windZero, WindZeroName(windZero));
    ASSERT_ORDERED(windZero, unsortable);
    s += strEnum(EF::unsortable, "unsortable", Unsortable::none == unsortable, 
			UnsortableName(unsortable));
	EDGE_BOOL(unsortable, active_impl);
    EDGE_BOOL(active_impl, inLinkups);
    EDGE_BOOL(inLinkups, linkHead);
    EDGE_BOOL(linkHead, inOutput);
    EDGE_BOOL(inOutput, disabled);
    EDGE_BOOL(disabled, smallTRange);
    EDGE_BOOL(smallTRange, isUnsplitable);
    EDGE_BOOL(isUnsplitable, ccEnd);
    EDGE_BOOL(ccEnd, ccLarge);
    EDGE_BOOL(ccLarge, ccOverlaps);
    EDGE_BOOL(ccOverlaps, ccSmall);
    EDGE_BOOL(ccSmall, ccStart);
    EDGE_BOOL(ccStart, centerless);
    EDGE_BOOL(centerless, startSeen);
    EDGE_BOOL(startSeen, endSeen);
    EDGE_BOOL(endSeen, unsectableStart);
    EDGE_BOOL(unsectableStart, unsectableEnd);
    EDGE_BOOL(unsectableEnd, unsummable);
#if OP_DEBUG
    ASSERT_ORDERED_OFFSET(unsummable, debugMatch, 2);
    if (debugMatch)
        s += "debugMatch:" + (debugMatch ? STR(debugMatch->id) : std::string("-")) + " ";
    ASSERT_ORDERED(debugMatch, debugZeroErr);
    if (debugZeroErr)
        s += "debugZeroErr:" + (debugZeroErr ? STR(debugZeroErr->id) : std::string("-")) + " ";
    ASSERT_ORDERED(debugZeroErr, debugParentID);
    s += strID(EF::debugParentID, "debugParentID", debugParentID);
    ASSERT_ORDERED(debugParentID, debugDepth);
    s += strID(EF::debugDepth, "debugDepth", debugDepth);
    ASSERT_ORDERED(debugDepth, debugCC);
    s += strID(EF::debugCC, "debugCC", debugCC);
    ASSERT_ORDERED(debugCC, debugRayMatch);
    s += strID(EF::debugRayMatch, "debugRayMatch", debugRayMatch);
	EDGE_BOOL(debugRayMatch, debugUnordered);
	EDGE_BOOL(debugUnordered, debugSumSet);
#endif
#if OP_DEBUG_DUMP
    ASSERT_ORDERED_OFFSET(debugSumSet, dumpContext, 6);
    // omit dumpContext
    EDGE_BOOL(dumpContext, debugLimb);
    EDGE_BOOL(debugLimb, debugReleased);
#endif
#if OP_DEBUG_MAKER
    if (debugSetDisabled.valid())
        s += "debugSetDisabled:" + debugSetDisabled.debugDump() + " ";
    ASSERT_ORDERED(debugSetDisabled, debugSetMaker);
    if (DebugLevel::file == l)
        s += "debugSetMaker:";
    s += debugSetMaker.debugDump() + " ";
    ASSERT_ORDERED(debugSetMaker, debugSetSum);
    if (debugSetSum.valid())
        s += "debugSetSum:" + debugSetSum.debugDump() + " ";
#endif
#if OP_DEBUG_VALIDATE
    s += strID(EF::debugPriorID, "debugPriorID", debugPriorID);
    EDGE_BOOL(debugPriorID, debugScheduledForErasure);
#endif
    return s;
}

std::string OpEdge::debugDumpPoints() const {
    std::string s = "[" + STR(id) + "]";
    s += " " + debugValue(DebugLevel::error, defaultBase, "startT", startT);
    s += " " + debugValue(DebugLevel::error, defaultBase, "endT", endT);
    s += " curve:" + curve.debugDump(defaultLevel, defaultBase);
    s += " which:" + EdgeMatchName(which());
    const OpEdge* startE = debugAdvanceToEnd(EdgeMatch::start);
    if (startE != this)
        s += " start[" + STR(startE->id) + "] " + startE->whichSect()
                .debugDump(defaultLevel, defaultBase);
    const OpEdge* endE = debugAdvanceToEnd(EdgeMatch::end);
    if (endE != this)
        s += " end[" + STR(endE->id) + "] " + endE->whichSect(!endE->which())
                .debugDump(defaultLevel, defaultBase);
    return s;
}

std::string OpEdge::debugDumpWinding() const {
    std::string s;
	DebugLevel l = DebugLevel::detailed == defaultLevel ? DebugLevel::normal : defaultLevel;
    if (winding.isSet())
        s += "winding" + winding.debugDump(l, defaultBase) + " ";
    if (sum.isSet())
        s += "sum" + sum.debugDump(l, defaultBase) + " ";
#if OP_EDGE_PAL_MANY
    if (palMany.isSet())
        s += "palMany" + palMany.debugDump(l, defaultBase);
#endif
    return s;
}

#if 0
// don't count curve that hasn't been built
int OpEdgeStorage::debugCount() const {
	const OpEdge* last = debugIndex(used - 1);
    int result = used - (PathOpsV0Lib::degenerateLine == last->curve.c.type);
    OpEdgeStorage* block = next;
    while (block) {
        result += block->used;
        block = block->next;
    }
    return result;
}
#endif

OpEdge* OpEdgeStorage::debugFind(int ID) {
	for (int index = 0; index < used; index++) {
		OpEdge& test = storage[index];
        if (test.id == ID || test.debugRayMatch == ID)
            return &test;
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

#if 0
// this walks 'backwards', from oldest to newest
OpEdge* OpEdgeStorage::debugIndex(int edgeIndex) const {
    const OpEdgeStorage* block = this;
    // build an array from that can be walked from back to front
    std::vector<const OpEdgeStorage*> blocks;
    do {
	    blocks.push_back(block);
        block = block->next;
    } while (block);
    // walk the array of blocks in the order they were allocated (back to front)
    for (size_t index = blocks.size(); index-- != 0; ) {
        block = blocks[index];
        if (edgeIndex < block->used)
            return const_cast<OpEdge*>(&block->storage[edgeIndex]);
        edgeIndex -= block->used;
    }
    return nullptr;
}
#endif

std::string OpEdgeStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    int cnt = edgeCount();
    if (!cnt)
        return s;
#if OP_DEBUG_DUMP
    s = debugName + ":" + STR(cnt) + "\n";
#endif
    if (DebugLevel::brief == l) {
        s += "[";
        for (int idx = 0; idx < cnt; ++idx)
            s += STR(edgeIndex(idx)->id) + " ";
        debugPopMatching(s, ' ');
        s += "]";
    } else {
	    for (int idx = 0; idx < cnt; idx++) {
		    const OpEdge* test = edgeIndex(idx);
            s += test->debugDump(l, b) + "\n";
	    }
        debugPopMatching(s, '\n');
    }
    return s;
}

std::string OpHulls::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    for (auto& hull : h)
        s += hull.debugDump(l, b) + "\n";
    return s;
}

std::string OpIntersection::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file != l) {
        s = "[" + debugDumpID() + "] ";
        if (DebugLevel::brief == l) {
            s += "{" + ptT.debugDump(l, b) + ", ";
            s += "seg:" + segment->debugDumpID();
            return s;
        }
        s += ptT.debugDump(id ? l : DebugLevel::error, b) + " ";   // !!! may be uninitialized?
        if (!callerPt.debugIsUninitialized() && callerPt != ptT.pt)
            s += "callerPt:" + callerPt.debugDump(l, b) + " ";
        std::string segmentID = segment ? segment->debugDumpID() : "-";
        const OpSegment* oppParent = opp ? opp->segment : nullptr;
        std::string oppID = opp ? opp->debugDumpID() : "-";
        std::string oppParentID = oppParent ? oppParent->debugDumpID() : "-";
        s += "segment:" + segmentID + " ";
        s += "opp/sect:" + oppParentID + "/" + oppID + " ";
        if (coincidenceID  OP_DEBUG_CODE(|| debugCoincidenceID)) {
            s += "coinID:" + STR(coincidenceID)  OP_DEBUG_CODE(+ "/" + STR(debugCoincidenceID)) 
                    + " ";
            s += MatchEndsName(coinEnd) + " ";
            s += CoinOppName(coinOpp) + " ";
        }
        if (unsectID) {
            s += "unsectID:" + STR(unsectID)+ " ";
            s += MatchEndsName(unsectEnd) + " ";
        }
        if (mergeID)
            s += "mergeID:" + STR(mergeID)+ " ";
        if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && !unsectID 
                && MatchEnds::none != coinEnd)
            s += "!!! (unexpected) " + MatchEndsName(coinEnd) + " ";
        if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && CoinOpp::yes == coinOpp)
            s += "!!! (unexpected) " + CoinOppName(coinOpp) + " ";
    } else {
        static_assert(0 == offsetof(OpIntersection, segment));
        if (segment)
            s += "segment:" + segment->debugDumpID() + " ";
        ASSERT_ORDERED(segment, opp);
        if (opp)
            s += "opp:" + opp->debugDumpID() + " ";
        ASSERT_ORDERED(opp, ptT);
        s += "ptT:" + ptT.debugDump(id ? l : DebugLevel::error, b) + " ";
        ASSERT_ORDERED(ptT, callerPt);
        if (!callerPt.debugIsUninitialized() && callerPt != ptT.pt)
            s += "callerPt:" + callerPt.debugDump(l, b) + " ";
        ASSERT_ORDERED(callerPt, coincidenceID);
        if (coincidenceID)
            s += "coincidenceID:" + STR(coincidenceID) + " ";
        ASSERT_ORDERED(coincidenceID, unsectID);
        if (unsectID)
            s += "unsectID:" + STR(unsectID) + " ";
        ASSERT_ORDERED(unsectID, mergeID);
        if (mergeID)
            s += "mergeID:" + STR(mergeID) + " ";
        ASSERT_ORDERED(mergeID, coinEnd);
        if (MatchEnds::none != coinEnd)
            s += "coinEnd:" + MatchEndsName(coinEnd) + " ";
        ASSERT_ORDERED(coinEnd, unsectEnd);
        if (MatchEnds::none != unsectEnd)
            s += "unsectEnd:" + MatchEndsName(unsectEnd) + " ";
        ASSERT_ORDERED(unsectEnd, coinOpp);
        if (CoinOpp::yes == coinOpp)
            s += "coinOpp:" + CoinOppName(coinOpp) + " ";
    }
	DEBUG_DUMP_BOOL(coinOpp, betweenCoins);
	DEBUG_DUMP_BOOL(betweenCoins, ccLine);
	DEBUG_DUMP_BOOL(ccLine, ccSect);
	DEBUG_DUMP_BOOL(ccSect, ccUnsectable);
	DEBUG_DUMP_BOOL(ccUnsectable, collapsed);
#if OP_DEBUG
    if (DebugLevel::file == l && id)
        s += "id:" + STR(id) + " ";
    if (debugSrcID)
        s += "debugSrcID:" + STR(debugSrcID) + " ";
    if (debugOppID)
        s += "debugOppID:" + STR(debugOppID) + " ";
    if (debugCoincidenceID)
        s += "debugCoincidenceID:" + STR(debugCoincidenceID) + " ";
    DEBUG_DUMP_BOOL(debugCoincidenceID, debugErased);
#endif
#if OP_DEBUG_MAKER
    s += debugSetMaker.debugDump() + " ";
#endif
    return debugPopMatching(s, ' ');
}

std::string OpIntersections::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (unsorted)
        s += "unsorted ";
    if (!i.size())
        return s;
    s += "intersections:" + STR(i.size()) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (OpIntersection* sect : i)
            s += STR(sect->id) + " ";
        debugPopMatching(s, ' ');
        s += "] ";
    } else {
        for (OpIntersection* sect : i)
            s += sect->debugDump(l, b) + "\n";
        debugPopMatching(s, '\n');
    }
    return s;
}

std::string OpJoiner::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (bestGap.edge)
        s += "bestGap:" + bestGap.debugDump(l, b) + "\n";
    s += "linkMatch:" + EdgeMatchName(linkMatch) + " ";
    s += "linkPass:" + LinkPassName(linkPass) + " ";
    if (edge)
		s += "edge:" + STR(edge->id) + " ";
    if (lastLink)
        s += "lastLink:" + STR(lastLink->id) + " ";
//    if (!OpMath::IsDebugNaN(matchPt.x) && !OpMath::IsDebugNaN(matchPt.y))
//        s += "matchPt:" + matchPt.debugDump(l, b) + " ";
    return debugPopMatching(s, ' ');
}

std::string OpLimb::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file != l)
        s = debugDumpIDs(l, false) + " ";  // note: dumps edge
    static_assert(0 == offsetof(OpLimb, limbBounds));
    if (limbBounds.isFinite())
        s += "limbBounds:" + limbBounds.debugDump(l, b) + " ";
    ASSERT_ORDERED(limbBounds, firstPts);
    if (firstPts.size()) {
        s += " firstPts:" + STR(firstPts.size()) + "{";
        for (OpPoint pt : firstPts) {
            s += pt.debugDump(l, b) + ", ";
        }
        debugPopMatching(s, ' ');
        debugPopMatching(s, ',');
        s += "} ";
    }
    ASSERT_ORDERED(firstPts, lastPts);
    if (lastPts.size()) {
        s += " lastPts:" + STR(lastPts.size()) + "{";
        for (OpPoint pt : lastPts) {
            s += pt.debugDump(l, b) + ", ";
        }
        debugPopMatching(s, ' ');
        debugPopMatching(s, ',');
        s += "} ";
    }
    ASSERT_ORDERED(lastPts, tree);
    ASSERT_ORDERED(tree, edge);
    if (DebugLevel::file == l && OP_DEBUG_INITED_PTR(edge))
        s += "edge:" + STR(edge->id) + " ";
    ASSERT_ORDERED(edge, lastLimbEdge);
    if (OP_DEBUG_INITED_PTR(lastLimbEdge))
        s += "lastLimbEdge:" + STR(lastLimbEdge->id) + " ";
    ASSERT_ORDERED(lastLimbEdge, parent);
    if (OP_DEBUG_INITED_PTR(parent))
        s += "parent:" +  parent->debugDumpIDs(l, true) + " ";
    ASSERT_ORDERED(parent, linkedContour);
    if (OP_DEBUG_INITED_PTR(linkedContour))
        s += "linkedContour:" + STR(linkedContour->id) + " ";
    ASSERT_ORDERED(linkedContour, lastT);
    if (!OpMath::IsDebugNaN(lastT))
        s += "lastT:" + debugFloat(l, lastT) + " ";
    ASSERT_ORDERED(lastT, closeDistance);
    if (!OpMath::IsDebugNaN(closeDistance))
        s += "closeDistance:" + debugFloat(l, closeDistance) + " ";
    ASSERT_ORDERED(closeDistance, linkedIndex);
    if (UINT_MAX != linkedIndex)
        s += "linkedIndex:" + STR((int) linkedIndex) + " ";
    ASSERT_ORDERED(linkedIndex, match);
    if (EdgeMatch::uninitialized != match)
        s += "match:" + EdgeMatchName(match) + " ";
    ASSERT_ORDERED(match, lastMatch);
    if (EdgeMatch::uninitialized != lastMatch)
        s += "lastMatch:" + EdgeMatchName(lastMatch) + " ";
    ASSERT_ORDERED(lastMatch, treePass);
    if (LimbPass::uninitialized != treePass)
        s += "treePass:" + LimbPassName(treePass) + " ";
    ASSERT_ORDERED(treePass, deadEnd);
    s += BoolToStr(l, deadEnd, "deadEnd", "deadEnd") + " "; 
    ASSERT_ORDERED(deadEnd, looped);
    s += BoolToStr(l, looped, "looped", "looped") + " ";
//    ASSERT_ORDERED(looped, resetPass);
//    s += BoolToStr(l, resetPass, "resetPass", "resetPass") + " ";
#if OP_DEBUG_DUMP
    ASSERT_SERIAL_OFFSET(*this, looped, 7, debugBranches);
    if (debugBranches.size()) {
        s += "debugBranches:" + STR(debugBranches.size()) + "[";
        for (auto limb : debugBranches)
            s += STR(limb->id) + " ";
        debugPopMatching(s, ' ');
        s += "] ";
    }
    ASSERT_ORDERED(debugBranches, id);
    if (DebugLevel::file == l)
        s += "id:" + STR(id) + " ";
    static_assert(sizeof(*this) == offsetof(OpLimb, id) + sizeof(id) + 4);
#endif
    return s;
}

std::string OpLimb::debugDumpIDs(DebugLevel l, bool bracket) const {
    if (DebugLevel::file == l)
        return STR(id);
    std::string s = (bracket ? "[" : "id:") + STR(id);
    if (edge && edge != OP_DEBUG_INIT_VALUE(OpEdge)) {
        s += (bracket ? " e:" : " edge:") + STR(edge->id);
        if (EdgeMatch::none != match)
            s += EdgeMatch::start == match ? "s" : "e";
        if (edge->lastEdge && edge != edge->lastEdge) {
            s += ".." + STR(edge->lastEdge->id);
            if (EdgeMatch::none != lastMatch)
                s += EdgeMatch::start == lastMatch ? "s" : "e";
        } else if (edge->priorEdge) {
            const OpEdge* firstEdge = edge;
            if (!edge->debugIsLoop()) {
                firstEdge = edge->debugAdvanceToEnd(EdgeMatch::start);
                s += ".." + STR(firstEdge->id);
            } else {
                s += " (loop)";
            }
        }
        if (!OpMath::IsDebugNaN(closeDistance))
            s += " closeD:" + debugFloat(l, closeDistance);
        if (bracket)
            s += "]";
    }
    return s;
}

int OpLimbStorage::debugCount() const {
    int result = used;
    OpLimbStorage* block = nextBlock;
    while (block) {
        result += block->used;
        block = block->nextBlock;
    }
    return result;
}

OpLimb* OpLimbStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index++) {
        if (storage[index].id == ID)
            return (OpLimb*) &storage[index];
    }
    if (nextBlock)
        return nextBlock->debugFind(ID);
    return nullptr;
}

OpLimb* OpLimbStorage::debugIndex(int index) const  {
    const OpLimbStorage* block = this;
    while (block->nextBlock)
        block = block->nextBlock;
    while (index >= block->used) {
        index -= block->used;
        block = block->prevBlock;
        if (!block)
            return nullptr;
    }
    if (block->used <= index)
        return nullptr;
    return (OpLimb*) &block->storage[index];
}

std::string OpLimbStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    int count = (int) debugCount();
    if (!count)
        return s;
    s += "limbStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
#if OP_DEBUG
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugFind(index)->id) + " ";
        debugPopMatching(s, ' ');
        s += "]";
#endif
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        debugPopMatching(s, '\n');
    }
    return s;
}

std::string OpPoint::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !isFinite())
        return "";
    return "{" + debugFloat(b, x) + ", " + debugFloat(b, y) + "}";
}

std::string OpPtT::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !pt.debugIsUninitialized() && !pt.isFinite() 
            && !OpMath::IsDebugNaN(t) && !OpMath::IsFinite(t))
        return "";
    std::string s;
    if (!pt.debugIsUninitialized())
        s += pt.debugDump(DebugLevel::error, b) + " ";
    if (!OpMath::IsDebugNaN(t))
        s += debugValue(DebugLevel::error, b, "t", t) + " ";
    debugPopMatching(s, ' ');
    return s;
}

std::string OpRect::debugDump(DebugLevel l, DebugBase b) const {
    return "{" + debugFloat(b, left) + ", " + debugFloat(b, top) + ", "
        + debugFloat(b, right) + ", " + debugFloat(b, bottom) + "}";
}

std::string OpRoots::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "count:" + STR(count()) + " ";
    for (int index = 0; index < count(); ++index)
        s += debugFloat(b, roots[index]) + ", ";
    debugPopMatching(s, ' ');
    return debugPopMatching(s, ',');
}

int OpSectStorage::debugCount() const {
    int result = used;
    OpSectStorage* block = next;
    while (block) {
        result += block->used;
        block = block->next;
    }
    return result;
}

OpIntersection* OpSectStorage::debugIndex(int index) const {
    const OpSectStorage* block = this;
    while (index < block->debugStart) {
        block = block->next;
        if (!block)
            return nullptr;
    }
    index -= block->debugStart;
    if (block->used <= index)
        return nullptr;
    return const_cast<OpIntersection*>(&block->storage[index]);
}

std::string OpSectStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    int count = debugCount();
    if (!count)
        return s;
    s += "sectStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        debugPopMatching(s, ' ');
        s += "]";
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        debugPopMatching(s, '\n');
    }
    return s;
}

std::string OpSegment::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::brief != l) {
        std::string s = "[" + STR(id) + "] ";
        static_assert(0 == offsetof(OpSegment, contour));
        if (contour)
            s += "contour[" + STR(contour->id) + "] ";
        ASSERT_ORDERED(contour, c);
        s += c.debugDump(l, b) + "\n";
        ASSERT_ORDERED(c, sects);
        if (!sects.i.empty()) {
            s += "sects:" + STR(sects.i.size()) + "[";
            for (auto sect : sects.i)
                s += STR(sect->id) + " ";
            debugPopMatching(s, ' ');
            s += "]\n";
        }
        ASSERT_ORDERED(sects, edges);
        if (!edges.empty() && dumpInitialized()) {
            s += "edges:" + STR(edges.size());
            if (DebugLevel::normal == l) {
                s += "[";
                for (auto& edge : edges)
                    s += STR(edge.id) + " ";
                debugPopMatching(s, ' ');
                s += "]\n";
            } else {
                s += "\n";
                for (auto& edge : edges)
                    s += edge.debugDump(l, b) + "\n";
            }
        }
        ASSERT_ORDERED(edges, winding);
        s += "winding:" + winding.debugDump(l, b) + " ";
        ASSERT_ORDERED(winding, id);  // write at front
        DEBUG_DUMP_BOOL(id, disabled);
        DEBUG_DUMP_BOOL(disabled, endsMerged);
        DEBUG_DUMP_BOOL(endsMerged, hasCoin);
        DEBUG_DUMP_BOOL(hasCoin, hasPals);
        DEBUG_DUMP_BOOL(hasPals, hasUnsectable);
        DEBUG_DUMP_BOOL(hasUnsectable, merged);
        // !!! skip debug color for now
#if OP_DEBUG_MAKER
        if (debugSetDisabled.valid())
            s += "debugSetDisabled:" + debugSetDisabled.debugDump() + " ";
        static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugSetDisabled) 
                + sizeof(debugSetDisabled));
#endif
        return debugPopMatching(s, ' ');
    }
    return "seg:" + STR(id) + " " + c.debugDump(l, b);
}

bool OpSegment::dumpInitialized() const {
    return contour && contour->context->dumpInitialized(); 
}

std::string OpTree::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    static_assert(0 == offsetof(OpTree, passIndex));  // skip context
    ASSERT_ORDERED(passIndex, context);
    ASSERT_ORDERED(context, trunk);
    if (trunk)
        s += " trunk:" + STR(trunk->id);
    ASSERT_ORDERED(trunk, bestGapLimb);
    if (bestGapLimb)
        s += " bestGapLimb:" + bestGapLimb->debugDumpIDs(l, true);
    ASSERT_ORDERED(bestGapLimb, bestLimb);
    if (bestLimb)
        s += " bestLimb:" + bestLimb->debugDumpIDs(l, true);
    ASSERT_ORDERED(bestLimb, bestDistance);
    if (OpMath::IsFinite(bestDistance))
        s += " bestDistance:" + debugFloat(b, bestDistance);
    ASSERT_ORDERED(bestDistance, bestPerimeter);
    if (OpMath::IsFinite(bestPerimeter))
        s += " bestPerimeter:" + debugFloat(b, bestPerimeter);
    ASSERT_ORDERED(bestPerimeter, maxLimbs);
    if (maxLimbs)
        s += " maxLimbs:" + STR(maxLimbs);
    ASSERT_ORDERED(maxLimbs, totalUsed);
    if (totalUsed)
        s += " totalUsed:" + STR(totalUsed);
    ASSERT_ORDERED(totalUsed, id);
    if (DebugLevel::file == l)
        s += " id:" + STR(id);
    ASSERT_ORDERED(id, limbPass);
    if (LimbPass::uninitialized != limbPass)
        s += " limbPass:" + LimbPassName(limbPass);
    ASSERT_ORDERED(limbPass, smallGap);
 //   if (disabled)
 //       s += " disabled";
 //   ASSERT_ORDERED(disabled, smallGap);
    if (smallGap)
        s += " smallGap";
    ASSERT_SERIAL_OFFSET(*this, smallGap, 2, debugAddEach);
    if (debugAddEach)
        s += " debugAddEach" + STR(debugAddEach);
    static_assert(sizeof(OpTree) == offsetof(OpTree, debugAddEach) + sizeof(debugAddEach) + 4);
    s.erase(s.begin());
    if (DebugLevel::file == l)
		return s;
	if (totalUsed)
		s += "\n";
    OpLimbStorage* saveCurrent = context->limbCurrent;
    for (int index = 0; index < totalUsed; ++index) {
        const OpLimb& limb = context->debugNthLimb(index);
//		OpDebugFormat(s);
        s += limb.debugDumpIDs(l, true);
        s += " parent:" + (limb.parent && limb.parent != OP_DEBUG_INIT_VALUE(OpLimb)
                ? limb.parent->debugDumpIDs(l, true) : "-");
        if (limb.debugBranches.size()) {
            s += " children:";
            for (OpLimb* child : limb.debugBranches) {
                s += child->debugDumpIDs(l, true) + " ";
            }
            debugPopMatching(s, ' ');
        }
        if (LimbPass::uninitialized != limb.treePass)
            s += " treePass:" + LimbPassName(limb.treePass) + "\n";
    }
    debugPopMatching(s, '\n');
    context->limbCurrent = saveCurrent;
    return s;
}

std::string OpVector::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !isFinite())
        return "";
    return "{" + debugFloat(b, dx) + ", " + debugFloat(b, dy) + "}";
}

std::string OpWinding::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += DebugDump(w, l, b);
	if (DebugLevel::detailed == l || DebugLevel::file == l) {
		if (WindingType::uninitialized != type)
			s += "type:" + WindingTypeName(type) + " ";
		if (DebugWindingType::uninitialized != debugType)
			s += "debugType:" + DebugWindingTypeName(debugType) + " ";
	}
	debugPopMatching(s, ' ');
    return s;
}

std::string RayTarget::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
    static_assert(0 == offsetof(RayTarget, contour));
    ASSERT_ORDERED(contour, bounds);
	if (DebugLevel::detailed == l || DebugLevel::file == l)
		s += "contour[" + STR(contour->id) + "] bounds:" + bounds.debugDump(l, b);
	else
		s += STR(contour->id);
    static_assert(sizeof(RayTarget) == offsetof(RayTarget, bounds) + sizeof(bounds));
	return s;
}

std::string RayTargets::debugDump(DebugLevel l, DebugBase b) const {
    ASSERT_ORDERED(context, t);
    std::string s = "t:" + STR(t.size()) + "[";
    for (const RayTarget& target : t) {
		if (DebugLevel::detailed == l)
			s += " ";  // indent
        s += target.debugDump(DebugLevel::detailed == l || DebugLevel::file == l ? l 
                : DebugLevel::brief, b) + " ";
	}
	debugPopMatching(s, ' ');
	s += "] ";
    ASSERT_ORDERED(t, chainBounds);
	s += "chainBounds" + chainBounds.debugDump(l, b) + " ";
    ASSERT_ORDERED(chainBounds, inXY);
    // not worth serializing: can be determined from debug edges contour and axis, below
    ASSERT_ORDERED(inXY, edgeIndex);
	if (SIZE_MAX != edgeIndex)
		s += "edgeIndex:" + STR(edgeIndex) +  " ";
    ASSERT_ORDERED(edgeIndex, tIndex);
	if (SIZE_MAX != tIndex)
		s += "tIndex:" + STR(tIndex) + " ";
    if (DebugLevel::file == l && debugEdgesContour) {
        ASSERT_ORDERED(tIndex, debugEdgesContour);
        s += "debugEdgesContour:" + STR(debugEdgesContour->id) + " ";
        ASSERT_ORDERED(debugEdgesContour, debugEdgesAxis);
        s += "debugEdgesAxis:" + AxisName(debugEdgesAxis) + " ";
        static_assert(sizeof(RayTargets) == offsetof(RayTargets, debugEdgesAxis) 
                + sizeof(debugEdgesAxis) + 7);
    }
	return debugPopMatching(s, ' ');
}

std::string SectRay::debugDumpHeader(DebugLevel l, DebugBase b) const {
    std::string s;
    ASSERT_ORDERED(debugErased, insideBounds);
    if (insideBounds.isFinite())
        s += debugLabel(l, "insideBounds") + insideBounds.debugDump(l, b) + " ";
    ASSERT_ORDERED(insideBounds, homeTangent);
	if (homeTangent.isFinite())
		s += debugLabel(l, "homeTangent") + homeTangent.debugDump(l, b) + " ";
    ASSERT_ORDERED(homeTangent, home);
    if (home)
        s += "home:" + STR(home->id) + " "; 
    ASSERT_ORDERED(home, normal);
	if (OpMath::IsFinite(normal))
		s += debugValue(l, b, "normal", normal) + " ";
    ASSERT_ORDERED(normal, homeCept);
	if (OpMath::IsFinite(homeCept))
	    s += debugValue(l, b, "homeCept", homeCept) + " ";
    ASSERT_ORDERED(homeCept, homeT);
	if (OpMath::IsFinite(homeT))
	    s += debugValue(l, b, "homeT", homeT) + " ";
    ASSERT_ORDERED(homeT, mid);
	if (.5 != mid)
	    s += debugValue(l, b, "mid", mid) + " ";
    ASSERT_ORDERED(mid, midEnd);
	if (.5 != midEnd)
	    s += debugValue(l, b, "midEnd", midEnd) + " ";
    ASSERT_ORDERED(midEnd, axis);
	if (Axis::neither != axis)
		s += "axis:" + AxisName(axis) + " ";
    ASSERT_ORDERED(axis, sorted);
	if (sorted) s += "sorted ";
    static_assert(sizeof(SectRay) == offsetof(SectRay, sorted) 
            + sizeof(sorted) + 2);
	return debugPopMatching(s, ' ');
}

std::string SectRay::debugDump(DebugLevel l, DebugBase b) const {
    bool addLF = DebugLevel::ray == l;
    if (addLF)
        l = DebugLevel::normal;
    static_assert(0 == offsetof(SectRay, targets));
    std::string s = "targets:" + targets.debugDump(l, b) + " ";
    ASSERT_ORDERED(targets, distances);
    s += "distances:" + STR(distances.size()) + "{";
    for (const Distance& dist : distances) {
        if (addLF)
            s += "\n";
        s += dist.debugDump(l, b) + ", ";
	}
    debugPopMatching(s, ' ');
    debugPopMatching(s, ',');
    if (addLF && !distances.empty())
        s += "\n ";
    s += "} ";
    ASSERT_ORDERED(distances, debugErased);
    s += "debugErased:" + STR(debugErased.size()) + "{";
    for (const Distance& erase : debugErased) {
        s += erase.debugDump(l, b) + ", ";
	}
    debugPopMatching(s, ' ');
    debugPopMatching(s, ',');
    s += "} ";
    ASSERT_ORDERED(debugErased, insideBounds);
    s += debugDumpHeader(l, b); 
    return s;
}

std::string SegPt::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "pt:" + pt.debugDump(l, b) + " ";
    s += "ptType:" + PtTypeName(ptType);
    return s;
}

std::string SnipPtTs::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "segPtT:" + segPtT.debugDump(l, b) + " ";
    s += "oppPtT:" + oppPtT.debugDump(l, b) + "\n";
    s += "segCut:" + segCut.debugDump(l, b) + "\n";
    s += "oppCut:" + oppCut.debugDump(l, b);
    return s;
}

std::string dmpFileToStr(std::string name) {
    std::string filename = dmpFileToPath(name);
    std::string buffer;
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        OpDebugOut("could not open " + filename + " to read (1st time)\n");
        return "";
    }
    int seek = fseek(file, 0, SEEK_END);
    OP_ASSERT(!seek);
    long size = ftell(file);
    fclose(file);
    file = fopen(filename.c_str(), "r");
    if (!file) {
        OpDebugOut("could not open " + filename + " to read (2nd time)\n");
        return "";
    }
    buffer.resize(size);
    fread(&buffer[0], 1, size, file);
    fclose(file);
#ifdef _WIN32
    // Windows 11 and earlier has a bug where files written with Unix lines (LF) 
    // are measured as if they had Windows lines (LF CR). Adjust the string 
    // length by counting the number of Unix-style line endings in the string.
    // !!! To prepare for a future bug fix: add code to write file w/ LF w/o CR 
    //     then measure size to see if it added 1 or not
    if (buffer.size()) {
        // !!! probably should move bug tester into some run-once test setup
        filename = dmpFileToPath("WindowsBugTest.txt");
        file = fopen(filename.c_str(), "w");
        if (!file)
            OpDebugOut("could not open " + filename + " to check for Windows bug fix\n");
        fwrite("x\n", 1, 2, file);
        fclose(file);
        fopen(filename.c_str(), "r");
        seek = fseek(file, 0, SEEK_END);
        OP_ASSERT(!seek);
        size = ftell(file);
        fclose(file);
        std::remove(filename.c_str());
        if (3 == size) {  // bug exists
            const char* s = &buffer.front();
            int line = 0;
            while (s <= &buffer.back()) {
                if ('\n' == s[0] && '\r' != s[1])
                    buffer.pop_back();
                s++;
            }
        }
    }
#endif
    return buffer;
}

#if OP_DEBUGGER
std::string debugDumpColor(DebugLevel l, uint32_t c) {
    char asHex[11];
    int written = snprintf(asHex, sizeof(asHex), "0x%08x", c);
    if (written != 10)
        return "snprintf of " + STR_E(c) + " to hex failed (written:" + STR(written) + ")";
    if (DebugLevel::file != l) {
        auto result = std::find_if(debugColorArray.begin(), debugColorArray.end(), [c](auto color) {
            return color.first == c; });
        if (debugColorArray.end() == result)
            return "color " + std::to_string(c) + " (" + std::string(asHex) + ") not found";
        return std::string(asHex) + " " + (*result).second;
    }
    return std::string(asHex);
}
#endif

#if OP_DEBUG_DUMP
OpContext* fromFile(std::string filename) {
    std::string buffer = dmpFileToStr(filename);
    if (buffer.empty())
        return nullptr;
    buffer += '\0';  // add terminator so debug set calls can check for the end
    const char* str = buffer.c_str();
    OpContext* save = debugGlobalContext;
    OpContext* fileContext = new OpContext();
    debugGlobalContext = fileContext;
    fileContext->debugFilename = filename;
    fileContext->dumpSet(str);  // also reads segments, which read segments' edges, etc.
    fileContext->dumpResolveAll(fileContext);
    debugGlobalContext = save;
    return fileContext;
}
#endif

#endif
