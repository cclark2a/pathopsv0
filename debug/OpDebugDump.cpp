// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_DUMP
#include <ctype.h>

#include "OpCurveCurve.h"
#include "OpContour.h"
#include "OpDebugColor.h"
#include "OpDebugDump.h"
#include "OpEdge.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

// some compilers warn about 'this' being checked for null
// this debugging code needs to do that anyway
#ifdef _WIN32
#define OP_I_KNOW_WHAT_IM_DOING 1
#else
#define OP_I_KNOW_WHAT_IM_DOING 0
#endif


int OpCurveCurve::debugCall;  // which call to curve-curve was made

// !!! things to do:
// decrement debug level (and indent) when dumping (for example) edges within an edge
// allow more flexible abbreviations for labels (none, first letter, string)
// consider what to macro-ize
// (and, in the end..) replace old calls with new ones
struct EdgeFilters {
    std::vector<EdgeFilter> filter;
    std::vector<EdgeFilter> always;
};

std::array<EdgeFilters, 3> edgeFilters;
extern std::vector<std::pair<uint32_t, std::string>> debugColorArray;
int lineWidth = 200;
DebugBase defaultBase = DebugBase::dec;
DebugLevel defaultLevel = DebugLevel::normal;

#ifdef _WIN32
#pragma optimize( "", off )
#endif

#define OP_X(Thing) \
	void dmp(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmp(thing); \
	} \
	void dmpHex(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmpHex(thing); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dmp(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmp(thing); \
	} \
	void dmpHex(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmpHex(thing); \
	}
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void dmp(const std::vector<Thing>* things) { \
        dmp(*things); \
    } \
	void dmpHex(const std::vector<Thing>* things) { \
        dmpHex(*things); \
    }
	VECTOR_STRUCTS
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

#define OP_X(Global, Method) \
    void Global(int ID) { \
        bool found = false; \
        if (std::vector<const OpIntersection*> coins = findCoincidence(ID); coins.size()) { \
            for (auto coin : coins) { \
                coin->Method(); \
                found = true; \
            } \
        } \
        if (const OpContour* contour = findContour(ID)) { \
            contour->Method(); \
            found = true; \
        } \
        if (const OpEdge* edge = findEdge(ID)) { \
            edge->Method(); \
            found = true; \
        } \
        if (std::vector<const OpEdge*> outputs = findEdgeOutput(ID); outputs.size()) { \
            for (auto output : outputs) { \
                output->Method(); \
                found = true; \
            } \
        } \
        if (std::vector<const OpEdge*> matches = findEdgeRayMatch(ID); matches.size()) { \
            for (auto match : matches) { \
                match->Method(); \
                found = true; \
            } \
        } \
        if (const OpIntersection* intersection = findIntersection(ID)) { \
            intersection->Method(); \
            found = true; \
        } \
        if (const OpLimb* limb = findLimb(ID)) { \
            limb->Method(); \
            found = true; \
        } \
        if (std::vector<const OpIntersection*> uSects = findSectUnsectable(ID); uSects.size()) { \
            for (auto uSect : uSects) { \
                uSect->Method(); \
                found = true; \
            } \
        } \
        if (const OpSegment* segment = findSegment(ID)) { \
            segment->Method(); \
            found = true; \
        } \
        if (!found) \
            OpDebugOut("ID: " + STR(ID) + " not found\n"); \
    }
    DUMP_BY_DUMPID
#undef OP_X

#define ENUM_NAME_STRUCT(enum) \
struct enum##Name { \
    enum element; \
    const char* name; \
}

#define ENUM_NAME_STRUCT_ABBR(enum) \
struct enum##Name { \
    enum element; \
    const char* name; \
    const char* abbr; \
}


#define ENUM_NAME(Enum, enum) \
static bool enum##OutOfDate = false; \
\
std::string enum##Name(Enum element) { \
    static bool enum##Checked = false; \
    int first = (int) enum##Names[0].element; \
    if (!enum##Checked) { \
        for (int index = 0; index < (int) ARRAY_COUNT(enum##Names); ++index) \
           if (!enum##OutOfDate && (int) enum##Names[index].element != index + first) { \
               OpDebugOut("!!! " #Enum " out of date\n"); \
               enum##OutOfDate = true; \
               break; \
           } \
        enum##Checked = true; \
    } \
    if (enum##OutOfDate) \
        return STR_E(element); \
    return enum##Names[(int) element - first].name; \
} \
\
Enum enum##Str(const char*& str, const char* label, Enum enumDefault) { \
    if (!OpDebugOptional(str, label)) \
        return enumDefault; \
    size_t strLen = 0; \
    while (isalnum(str[strLen])) \
        ++strLen; \
    for (int index = 0; index < (int) ARRAY_COUNT(enum##Names); ++index) { \
        size_t nameLen = strlen(enum##Names[index].name); \
        if (strLen == nameLen && !strncmp(str, enum##Names[index].name, nameLen)) { \
            str += strlen(enum##Names[index].name); \
            if (' ' == str[0]) ++str; \
            return enum##Names[index].element; \
        } \
    } \
    OpDebugExitOnFail("missing enum", false); \
    return (Enum) -1; \
}

#define ENUM_NAME_ABBR(Enum, enum) \
static bool enum##OutOfDate = false; \
\
std::string enum##Name(Enum element, DebugLevel l) { \
    static bool enum##Checked = false; \
    int first = (int) enum##Names[0].element; \
    if (!enum##Checked) { \
        for (int index = 0; index < (int) ARRAY_COUNT(enum##Names); ++index) \
           if (!enum##OutOfDate && (int) enum##Names[index].element != index + first) { \
               OpDebugOut("!!! " #Enum " out of date\n"); \
               enum##OutOfDate = true; \
               break; \
           } \
        enum##Checked = true; \
    } \
    if (enum##OutOfDate) \
        return STR_E(element); \
    return DebugLevel::brief == l ? enum##Names[(int) element - first].abbr \
        : enum##Names[(int) element - first].name; \
} \
\
Enum enum##Str(const char*& str, const char* label, Enum enumDefault) { \
    if (!OpDebugOptional(str, label)) \
        return enumDefault; \
    size_t strLen = 0; \
    while (isalnum(str[strLen])) \
        ++strLen; \
    for (int index = 0; index < (int) ARRAY_COUNT(enum##Names); ++index) { \
        size_t nameLen = strlen(enum##Names[index].name); \
        if (strLen == nameLen && !strncmp(str, enum##Names[index].name, nameLen)) { \
            str += nameLen; \
            if (' ' == str[0]) ++str; \
            return enum##Names[index].element; \
        } \
    } \
    OpDebugExitOnFail("missing enum", false); \
    return (Enum) -1; \
}


std::string stringFormat(std::string s) {
    if (!s.size())
        return "";
    if (!lineWidth || lineWidth >= (int) s.size())
        return s;
    std::string result;
    const char* start = &s.front();
    const char* end = &s.back();
    while (start + lineWidth <= end) {
        const char* c = start;
        for (int i = 0; i < lineWidth; ++i) {
            if ('\n' == c[i]) {
                std::string line = s.substr(start - &s.front(), i + 1);
                result += line;
                start += i + 1;
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
    }
    if (start <= end)
        result += s.substr(start - &s.front());
    return result;
}

void OpDebugFormat(std::string s) {
    std::string result = stringFormat(s);
    OpDebugOut(result + "\n");
}

void dmpHex(float f) {
    OpDebugOut(OpDebugDumpHex(f));
}

void dmpHex(uint32_t u) {
    OpDebugOut(OpDebugIntToHex(u));
}

void dmpWidth(int width) {
    lineWidth = width;
}

void dmpActive() {
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isActive())
                    edge.dump();
            }
        }
    }
}

void dmpAliases() {
    debugGlobalContours->aliases.dump();
}

void dmpCoincidences() {
    for (const auto c : debugGlobalContours->contours) {
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
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            s += seg.debugDump(defaultLevel, defaultBase) + "\n";
            for (const auto& edge : seg.edges) {
                s += edge.debugDump(defaultLevel, defaultBase) + "\n";
            }
        }
    }
    OpDebugOut(s);
}

std::string debugDumpContours() {
    return debugGlobalContours->debugDump(DebugLevel::detailed, DebugBase::hex);
}

std::string debugDumpEdges() {
    size_t edgeCount = 0;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            edgeCount += seg.edges.size();
        }
    }
    std::string s = "edges:" + STR(edgeCount) + "\n";
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                s += edge.debugDump(DebugLevel::detailed, DebugBase::hex) + "\n";
            }
        }
    }
    s.pop_back();
    return s;
}

std::string debugDumpIntersections() {
    size_t sectCount = 0;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            sectCount += seg.sects.i.size();
        }
    }
    std::string s = "sects:" + STR(sectCount) + "\n";
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto sect : seg.sects.i) {
                s += sect->debugDump(DebugLevel::detailed, DebugBase::hex) + "\n";
            }
        }
    }
    s.pop_back();
    return s;
}

void dmpFile() {
    FILE* file = fopen("dmp.txt", "w");
    std::string s;
    s += debugGlobalContours->debugDump(DebugLevel::file, DebugBase::hex);
    int saveWidth = lineWidth;
    lineWidth = 100;
    s = stringFormat(s);
    lineWidth = saveWidth;
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
    fflush(file);
}

OpContours* fromFileContours = nullptr;

OpContours* fromFile(std::vector<PathOpsV0Lib::CurveCallBacks>* callBacks) {
    std::string buffer;
    if (FILE* file = fopen("dmp.txt", "r")) {
        int seek = fseek(file, 0, SEEK_END);
        OP_ASSERT(!seek);
        long size = ftell(file);
        fclose(file);
        file = fopen("dmp.txt", "r");
        buffer.resize(size);
        fread(&buffer[0], 1, size, file);
        fclose(file);
    }
    const char* str = buffer.c_str();
    OpContours* fileContours = new OpContours();
    if (callBacks)
        fileContours->callBacks = *callBacks;
    fileContours->dumpSet(str);  // also reads segments, which read segments' edges, etc.
    fileContours->dumpResolveAll(fileContours);
    return fileContours;
}

void verifyFile(OpContours* contours) {
	OpContours* fileContours = fromFile(&contours->callBacks);
    FILE* file = fopen("dmp2.txt", "w");
    std::string s;
    OP_ASSERT(fileContours);
    s += fileContours->debugDump(DebugLevel::file, DebugBase::hex);
    int saveWidth = lineWidth;
    lineWidth = 100;
    s = stringFormat(s);
    lineWidth = saveWidth;
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
    fflush(file);
    delete fileContours;
}

void OpContours::dumpResolve(OpContour*& ) {
    for (auto c : contours) {
        OP_ASSERT(c->contours == this);
    }
}

void OpContours::dumpResolve(OpEdge*& edgeRef) {
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
    OP_ASSERT((int) (size_t) edgeRef != edgeID);
}

void OpContours::dumpResolve(const OpEdge*& edgeRef) {
    dumpResolve(const_cast<OpEdge*&>(edgeRef));
}

void OpContours::dumpResolve(const OpLimb*& limbRef) {
    size_t limbID = (size_t) limbRef;
    if (0 == limbID)
        return;
    const OpLimb* limb = limbStorage->debugFind(limbID);
    OP_ASSERT(limb);
    limbRef = limb;
}

void OpContours::dumpResolve(OpIntersection*& sectRef) {
    size_t sectID = (size_t) sectRef;
    if (0 == sectID)
        return;
    OpIntersection* sect = sectStorage->debugFind(sectID);
    OP_ASSERT(sect);
    sectRef = sect;
}

void OpContours::dumpResolve(OpSegment*& segRef) {
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
    OP_ASSERT((int) (size_t) segRef != segID);
}

void dmpDisabled() {
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.disabled)
                    edge.dump();
            }
        }
    }
}

void dmpInOutput() {
    for (const auto c : debugGlobalContours->contours) {
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
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            s += seg.debugDumpIntersections() + "\n";
        }
    }
    OpDebugOut(s);
}

void dmpJoin() {
    dmpActive();
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (!edge.isActive() && edge.isUnsectable() && !edge.inOutput && !edge.inLinkups)
                    edge.dump(DebugLevel::detailed, defaultBase);
            }
        }
    }
    dmpUnsortable();
}

void dmpSects() {
    dmpIntersections();
}

void dmpSegments() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            seg.dump();
        }
    }
}

void dmpUnsectable() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isUnsectable())
                    edge.dump(DebugLevel::detailed, defaultBase);
            }
        }
    }
}

void dmpUnsortable() {
    for (const auto& c : debugGlobalContours->contours) {
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
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                s += "edge[" + STR(edge.id) + "] ";
                s += edge.debugDumpWinding() + "\n";
            }
        }
    }
    OpDebugOut(s);
}

// write state of edges and intersections to detect where data changes from one run to the next
// edge:
// start pt/t
// end pt/t
// seg id

// structures written by debugdumphex
struct OpDebugSectHex {
    uint32_t pt[2];
    uint32_t t;
    int segmentID;
};

struct OpDebugEdgeHex {
    uint32_t startPt[2];
    uint32_t startT;
    uint32_t endPt[2];
    uint32_t endT;
    int segmentID;
};

#if 0
// if file exists, read old state
void OpContours::dumpCount(std::string label) const {
    FILE* file = fopen(label.c_str(), "rb");
    char* buffer = nullptr;
    long size = 0;
    if (file) {
        int seek = fseek(file, 0, SEEK_END);
        OP_ASSERT(!seek);
        size = ftell(file);
        fclose(file);
        file = fopen(label.c_str(), "rb");
        buffer = (char*) malloc(size);
        fread(buffer, 1, size, file);
        fclose(file);
    }
    // if old exists, compare
    if (buffer)  {
        std::string old(buffer, size);
        debugCompare(old);
    }
    // write new state
    std::string s = debugDumpHex(label);
    file = fopen(label.c_str(), "w");
    size_t result = fwrite(s.c_str(), 1, s.length(), file);
    OP_ASSERT(result == s.length());
    fclose(file);
    fflush(file);
    free(buffer);
}
#endif

ENUM_NAME_STRUCT(OpOperator);

#define OP_OPERATOR_NAME(r) { OpOperator::r, #r }

OpOperatorName opOperatorNames[] {
    OP_OPERATOR_NAME(Subtract),
    OP_OPERATOR_NAME(Intersect),
    OP_OPERATOR_NAME(Union),
    OP_OPERATOR_NAME(ExclusiveOr),
    OP_OPERATOR_NAME(ReverseSubtract)
};

ENUM_NAME(OpOperator, opOperator)

#if OP_DEBUG
ENUM_NAME_STRUCT(OpDebugWarning);
#define OP_WARNING_NAME(r) { OpDebugWarning::r, #r }
OpDebugWarningName debugWarningNames[] {
    OP_WARNING_NAME(lastResort)
};
ENUM_NAME(OpDebugWarning, debugWarning)

ENUM_NAME_STRUCT(OpDebugExpect);
#define OP_EXPECT_NAME(r) { OpDebugExpect::r, #r }
OpDebugExpectName debugExpectNames[] {
    OP_EXPECT_NAME(unknown),
	OP_EXPECT_NAME(fail),
	OP_EXPECT_NAME(success)
};
ENUM_NAME(OpDebugExpect, debugExpect)
#endif

std::string OpPtAliases::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "aliases[\n";
    for (OpPoint pt : aliases) {
        s += pt.debugDump(l, b) + "\n";
    }
    s += "] maps[\n";
    for (OpPtAlias map : maps) {
        s += map.original.debugDump(l, b) + ":" + map.alias.debugDump(l, b) + "\n";
    }
    s += "] threshold:" + threshold.debugDump(l, b);
    return s;
}

std::string OpContours::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (aliases.maps.size())
        s += aliases.debugDump(l, b) + "\n";
    if (curveDataStorage) {
        s += "curveDataStorage:";
        s += curveDataStorage->debugDump(l, b) + "\n";
    }
    if (callerStorage) {
        s += "callerStorage:";
        s += callerStorage->debugDump(l, b) + "\n";
    }
    if (contourStorage)
        s += contourStorage->debugDump(l, b) + "\n";
    if (ccStorage)
        s += ccStorage->debugDump("ccStorage", l, b) + "\n";
    // skip contour iterator for now
    if (fillerStorage)
        s += fillerStorage->debugDump("fillerStorage", l, b) + "\n";
    if (sectStorage)
        s += sectStorage->debugDump(l, b) + "\n";
    if (limbStorage)
        s += limbStorage->debugDump(l, b) + "\n";
    s += "uniqueID:" + STR(uniqueID) + " ";
#if OP_DEBUG_VALIDATE
    s += "debugValidateEdgeIndex:" + STR(debugValidateEdgeIndex) + " ";
    s += "debugValidateJoinerIndex:" + STR(debugValidateJoinerIndex) + " ";
#endif
    s.pop_back();
    s += "\n";
#if OP_DEBUG
    if (debugCurveCurve)
        s += "debugCurveCurve:" + debugCurveCurve->debugDump(l, b) + "\n";
    if (debugJoiner)
        s += "debugJoiner:" + debugJoiner->debugDump(l, b) + "\n";
    if (debugWarnings.size()) {
        s += "debugWarnings:" + STR(debugWarnings.size()) + " [";
        for (auto& warning : debugWarnings)
            s += debugWarningName(warning) + " ";
        s += "] ";
    }
    if (debugData.testname.size())
        s += "debugTestname:" + debugData.testname + " ";
    s += "debugExpect:" + debugExpectName(debugData.expect) + " ";
    s += debugInPathOps ? "debugInPathOps " : "";
    s += debugInClearEdges ? "debugInClearEdges " : "";
    s += debugCheckLastEdge ? "debugCheckLastEdge " : "";
    s += debugFailOnEqualCepts ? "debugFailOnEqualCepts " : "";
#endif
    return s;
}

void OpContours::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "aliases"))
        OP_ASSERT(0);  // !!! incomplete
    if (OpDebugOptional(str, "curveDataStorage"))
        CurveDataStorage::DumpSet(str, &curveDataStorage);
    if (OpDebugOptional(str, "callerStorage"))
        CallerDataStorage::DumpSet(str, &callerStorage);
    if (OpDebugOptional(str, "contourStorage"))
        OpContourStorage::DumpSet(str, this);
    if (OpDebugOptional(str, "ccStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::cc);
    // skip contour iterator for now
    if (OpDebugOptional(str, "fillerStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::filler);
    if (OpDebugOptional(str, "sectStorage"))
        OpSectStorage::DumpSet(str, this);
    if (OpDebugOptional(str, "limbStorage"))
        OpLimbStorage::DumpSet(str, this);
    OpDebugRequired(str, "uniqueID");
    uniqueID = (int) OpDebugReadSizeT(str);
#if OP_DEBUG_VALIDATE
    OpDebugRequired(str, "debugValidateEdgeIndex");
    debugValidateEdgeIndex = (int) OpDebugReadSizeT(str);
    OpDebugRequired(str, "debugValidateJoinerIndex");
    debugValidateJoinerIndex = (int) OpDebugReadSizeT(str);
#endif
#if OP_DEBUG
    if (OpDebugOptional(str, "debugCurveCurve")) {
        if (!debugCurveCurve)
            debugCurveCurve = new OpCurveCurve(this);
        debugCurveCurve->dumpSet(str);
    }
    if (OpDebugOptional(str, "debugJoiner")) {
        if (!debugJoiner)
            debugJoiner = new OpJoiner(*this);
        debugJoiner->dumpSet(str);
    }
    if (OpDebugOptional(str, "debugWarnings")) {
        size_t count = OpDebugReadSizeT(str);
        for (size_t index = 0; index < count; ++index)
            debugWarnings[index] = debugWarningStr(str, "", OpDebugWarning::lastResort);
    }
    if (OpDebugOptional(str, "debugTestname"))
        debugData.testname = OpDebugLabel(str);
    debugExpect = debugExpectStr(str, "debugExpect", OpDebugExpect::fail);
    debugInPathOps = OpDebugOptional(str, "debugInPathOps");
    debugInClearEdges = OpDebugOptional(str, "debugInClearEdges");
    debugCheckLastEdge = OpDebugOptional(str, "debugCheckLastEdge");
    debugFailOnEqualCepts = OpDebugOptional(str, "debugFailOnEqualCepts");
	debugDumpInit = true;
#endif
}

void OpContours::dumpResolveAll(OpContours* self) {
    OP_ASSERT(this == self);
    contourStorage->dumpResolveAll(self);
    ccStorage->dumpResolveAll(self);
    fillerStorage->dumpResolveAll(self);
    sectStorage->dumpResolveAll(self);
    limbStorage->dumpResolveAll(self);
#if OP_DEBUG
    if (debugCurveCurve)
        debugCurveCurve->dumpResolveAll(self);
    if (debugJoiner)
        debugJoiner->dumpResolveAll(self);
#endif
}

void dmpContours() {
    dmp(*debugGlobalContours);
}

void dmpMatch(const OpPoint& pt, bool detail) {
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            if (pt == seg.c.firstPt() || pt == seg.c.lastPt()) {
                std::string str = "seg: " 
                        + (detail ? seg.debugDump(DebugLevel::detailed, defaultBase) 
                        : seg.debugDump(defaultLevel, defaultBase));
                OpDebugOut(str + "\n");
            }
            for (const auto sect : seg.sects.i) {
                if (sect->ptT.pt == pt) {
                    OpDebugOut("sect: ");
                    detail ? sect->dump(DebugLevel::detailed, defaultBase) : sect->dump();
                }
            }
            for (const auto& edge : seg.edges) {
                if (edge.startPt() == pt) {
                    OpDebugOut("edge start: ");
                    detail ? edge.dump(DebugLevel::detailed, defaultBase) : edge.dump();
                }
                if (edge.endPt() == pt) {
                    OpDebugOut("edge end: ");
                    detail ? edge.dump(DebugLevel::detailed, defaultBase) : edge.dump();
                }
            }
        }
    }
}

void dmpMatch(const OpPoint& pt) {
    dmpMatch(pt, false);
}

void dmpMatch(const OpPtT& ptT) {
    dmpMatch(ptT.pt, false);
}

std::vector<const OpIntersection*> findCoincidence(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == abs(intersection->coincidenceID) 
                        OP_DEBUG_CODE(|| ID == abs(intersection->debugCoincidenceID)))
                    result.push_back(intersection);
            }
        }
    }
    return result;
}

const OpContour* findContour(int ID) {
#if OP_DEBUG
    for (const auto c : debugGlobalContours->contours)
        if (ID == c->id)
            return c;
#endif
    return nullptr;
}

OpEdge* findEdge(int ID) {
    auto match = [ID](const OpEdge& edge) {
        return edge.id == ID ||
                edge.debugOutPath == ID || edge.debugRayMatch == ID;
    };
    for (auto c : debugGlobalContours->contours) {
        for (auto& seg : c->segments) {
            for (auto& edge : seg.edges) {
                if (match(edge))
                    return &edge;
            }
        }
    }
    if (OpEdge* filler = debugGlobalContours->fillerStorage
            ? debugGlobalContours->fillerStorage->debugFind(ID) : nullptr)
        return filler;
    // if edge intersect is active, search there too
    if (OpEdge* ccEdge = debugGlobalContours->ccStorage
            ? debugGlobalContours->ccStorage->debugFind(ID) : nullptr)
        return ccEdge;
    return nullptr;
}

std::vector<const OpEdge*> findEdgeOutput(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.debugOutPath)
                    result.push_back(&edge);
            }
        }
    }
    return result;
}

std::vector<const OpEdge*> findEdgeRayMatch(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.debugRayMatch)
                    result.push_back(&edge);
            }
        }
    }
    return result;
}

const OpIntersection* findIntersection(int ID) {
#if OP_DEBUG
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == intersection->id)
                    return intersection;
            }
        }
    }
#endif
    return nullptr;
}

const OpLimb* findLimb(int ID) {
    if (const OpLimb* limb = debugGlobalContours->limbStorage
            ? debugGlobalContours->limbStorage->debugFind(ID) : nullptr)
        return limb;
    return nullptr;
}

std::vector<const OpIntersection*> findSectUnsectable(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == abs(intersection->unsectID))
                    result.push_back(intersection);
            }
        }
    }
    return result;
}

const OpSegment* findSegment(int ID) {
    for (const auto c : debugGlobalContours->contours) {
        for (const auto& seg : c->segments) {
            if (ID == seg.id)
                return &seg;
        }
    }
    return nullptr;
}

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

void OpContours::debugCompare(std::string s) {
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

const OpLimb& OpContours::debugNthLimb(int index) const {
    OpLimbStorage* saveCurrent = limbCurrent;
    OpContours* writeable = const_cast<OpContours*>(this);
    const OpLimb& result = writeable->nthLimb(index);
    writeable->limbCurrent = saveCurrent;
    return result;
}

std::string OpContour::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "contour:";
    OP_DEBUG_CODE(s += STR(id));
    s += " ";
 //   s += "bounds:" + ptBounds.debugDump(l, b) + " ";
    if (DebugLevel::file != l)
        s += callBacks.debugDumpContourExtraFuncPtr(caller, l, b) + " ";
    s += "segments:" + STR(segments.size()) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (auto& segment : segments)
            s += STR(segment.id) + " ";
        s.pop_back();
        s += "] ";
    } else {
        for (auto& segment : segments)
            s += segment.debugDump(l, b) + "\n";
    }
    s.pop_back();
    return s;
}

void OpContour::dumpSet(const char*& str) {
    OpDebugRequired(str, "contour");
    OP_DEBUG_CODE(id = OpDebugReadSizeT(str));
    OpDebugRequired(str, "segments");
    int segmentCount = OpDebugReadSizeT(str);
    segments.resize(segmentCount);
    for (int index = 0; index < segmentCount; ++index)
        segments[index].contour = this;
    for (int index = 0; index < segmentCount; ++index)
        segments[index].dumpSet(str);
}

void OpContour::dumpResolveAll(OpContours* c) {
    for (OpSegment& segment : segments)
        segment.dumpResolveAll(c);
}

void dmp(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dump();
}

size_t OpContourStorage::debugCount() const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return 0;
#endif
    size_t result = used;
    OpContourStorage* block = next;
    while (next) {
        result += block->used;
        block = block->next;
    }
    return result;
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

OpContour* OpContourStorage::debugIndex(int index) const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return nullptr;
#endif
    const OpContourStorage* block = this;
    while (index > block->used) {
        index -= block->used;
        block = block->next;
        if (!block)
            return nullptr;
    }
    if (block->used <= index)
        return nullptr;
    return const_cast<OpContour*>(&block->storage[index]);
}

std::string OpContourStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    size_t count = debugCount();
    if (!count)
        return s;
    s += "contourStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (size_t index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
        for (size_t index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
}

void OpContourStorage::DumpSet(const char*& str, OpContours* dumpContours) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpContour* sect = dumpContours->allocateContour();
        sect->contours = dumpContours;
        sect->dumpSet(str);
    }
}

void OpContourStorage::dumpResolveAll(OpContours* c) {
    size_t count = debugCount();
    for (size_t index = 0; index < count; ++index) {
        debugIndex(index)->dumpResolveAll(c);
    }
}

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

std::string debugLabel(DebugLevel l, std::string label) {
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

std::string debugValue(DebugLevel l, DebugBase b, std::string label, float value) {
    std::string s;
    if (DebugLevel::error != l && DebugLevel::file != l && !OpMath::IsFinite(value))
        return s;
    s = debugLabel(l, label) + ":";
    return s + debugFloat(b, value);
}

std::string debugErrorValue(DebugLevel l, DebugBase b, std::string label, float value) {
    return debugValue(DebugLevel::file == l ? l : DebugLevel::error, b, label, value);
}

// returns caller curve data stored in contours as bytes encoded in string
std::string CurveDataStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "next:" + STR(next) + " ";  // only zero/nonzero is read
    s += "used:" + STR(used) + " ";
    if (DebugLevel::detailed == l || DebugLevel::file == l) {
        s += "\n";
        s += OpDebugDumpByteArray(storage, used);   // 'b' is ignored for now; always return hex
        if (next)
            s += "\n";
    }
    if (next)
        s += " " + next->debugDump(l, b);
    if (' ' == s.back())
        s.pop_back();
    return s;
}

// returns byte offset of caller curve data stored in contours
std::string CurveDataStorage::debugDump(PathOpsV0Lib::CurveData* curveData) const {
    const CurveDataStorage* test = this;
    char* data = (char*) curveData;
    size_t result = 0;
    while (data < test->storage || data >= &test->storage[sizeof(test->storage)]) {
        result += used;
        test = test->next;
        OP_ASSERT(test);
    }
    OP_ASSERT(data < test->storage + test->used);
    ptrdiff_t diff = data - test->storage;
    result += diff;
    return STR(result);
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
    OpDebugRequired(str, "next");
    storage->next = (CurveDataStorage*) OpDebugReadSizeT(str);  // non-zero means there is more
    OpDebugRequired(str, "used");
    storage->used = OpDebugReadSizeT(str);
    std::vector<uint8_t> bytes = OpDebugByteArray(str);
    OP_ASSERT(storage->used == bytes.size());
    std::memcpy(storage->storage, &bytes.front(), storage->used);
    if (storage->next)
        DumpSet(str, &storage->next);
}

std::string OpCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += contours->callBack(c.type).debugDumpCurveNameFuncPtr() + " ";
    if (DebugLevel::file == l) {
        s += "size:" + STR(c.size) + " ";
        s += "data:" + contours->curveDataStorage->debugDump(c.data) + " ";
    } else {
        s += "{ ";
        for (int i = 0; i < pointCount(); ++i) 
            s += hullPt(i).debugDump(DebugLevel::error, b) + ", ";
        s.pop_back(); s.pop_back();
        s += " }";
        s += contours->callBack(c.type).debugDumpCurveExtraFuncPtr(c, l, b);
    }
    return s;
}

void OpCurve::dumpSet(const char*& str) {
    size_t strLen = 0;
    while (isalnum(str[strLen]))
        ++strLen;
    for (size_t index = 0; index < contours->callBacks.size(); ++index) {
        std::string name = contours->callBacks[index].debugDumpCurveNameFuncPtr();
        if (name.size() == strLen && !strncmp(str, name.c_str(), strLen)) {
            str += strLen;
            if (' ' == str[0])
                ++str;
            c.type = (PathOpsV0Lib::CurveType) (index + 1);
        }
    }
    OpDebugRequired(str, "size");
    c.size = OpDebugReadSizeT(str);
    OpDebugRequired(str, "data");
    c.data = contours->curveDataStorage->dumpSet(str);  // do not allocate, just point to
}

ENUM_NAME_STRUCT(EdgeMatch);
#define MATCH_NAME(r) { EdgeMatch::r, #r }

static EdgeMatchName edgeMatchNames[] {
    MATCH_NAME(none),
    MATCH_NAME(start),
    MATCH_NAME(end),
//    MATCH_NAME(both),
};

ENUM_NAME(EdgeMatch, edgeMatch)

ENUM_NAME_STRUCT(EdgeFail);
#define FAIL_NAME(r) { EdgeFail::r, #r }

static EdgeFailName edgeFailNames[] {
    FAIL_NAME(none),
//    FAIL_NAME(center),
    FAIL_NAME(horizontal),
    FAIL_NAME(vertical),
};

ENUM_NAME(EdgeFail, edgeFail)

ENUM_NAME_STRUCT(WindZero);
#define WIND_ZERO_NAME(r) { WindZero::r, #r }

static WindZeroName windZeroNames[] {
    WIND_ZERO_NAME(unset),
    WIND_ZERO_NAME(zero),
    WIND_ZERO_NAME(nonZero),
};

ENUM_NAME(WindZero, windZero)

ENUM_NAME_STRUCT(Axis);
#define AXIS_NAME(r) { Axis::r, #r }

static AxisName axisNames[] {
    AXIS_NAME(neither),
    AXIS_NAME(vertical),
    AXIS_NAME(horizontal),
    AXIS_NAME(up),
    AXIS_NAME(left),
};

ENUM_NAME(Axis, axis)

#define EDGE_FILTER \
	OP_X(segment) \
	OP_X(ray) \
	OP_X(priorEdge) \
	OP_X(nextEdge) \
	OP_X(lastEdge) \
	OP_X(center) \
	OP_X(curve) \
	OP_X(vertical_impl) \
	OP_X(upright_impl) \
	OP_X(ptBounds) \
	OP_X(linkBounds) \
	OP_X(winding) \
	OP_X(sum) \
	OP_X(many) \
	OP_X(coinPals) \
	OP_X(unSects) \
	OP_X(pals) \
    OP_X(hulls) \
	OP_X(startT) \
	OP_X(endT) \
	OP_X(startSect) \
	OP_X(endSect) \
	OP_X(id) \
	OP_X(whichEnd_impl) \
	OP_X(rayFail) \
	OP_X(windZero) \
	OP_X(doSplit) \
	OP_X(isUnsortable) \
	OP_X(closeSet) \
	OP_X(active_impl) \
	OP_X(inLinkups) \
	OP_X(inOutput) \
	OP_X(disabled) \
	OP_X(isUnsplitable) \
	OP_X(ccEnd) \
	OP_X(ccLarge) \
	OP_X(ccOverlaps) \
	OP_X(ccSmall) \
	OP_X(ccStart) \
	OP_X(centerless) \
	OP_X(startSeen) \
	OP_X(endSeen)

#define EDGE_VIRTUAL \
    OP_X(contour)

#define EDGE_DEBUG \
	OP_X(Match) \
	OP_X(ZeroErr) \
	OP_X(OutPath) \
	OP_X(ParentID) \
	OP_X(RayMatch) \
	OP_X(Filler)

#define EDGE_IMAGE \
	OP_X(Color) \
	OP_X(Draw) \
	OP_X(Join) \
	OP_X(Custom)

#define EDGE_MAKER \
    OP_X(SetDisabled) \
	OP_X(SetMaker) \
	OP_X(SetSum)

enum class EF {
#define OP_X(Field) \
    Field,
    EDGE_FILTER
#undef OP_X
#define OP_X(Field) \
    Field,
    EDGE_VIRTUAL
#undef OP_X
#if OP_DEBUG
    #define OP_X(Field) \
        debug##Field,
        EDGE_DEBUG
    #undef OP_X
#endif
#if OP_DEBUG_IMAGE
    #define OP_X(Field) \
        debug##Field,
        EDGE_IMAGE
    #undef OP_X
#endif
#if OP_DEBUG_MAKER
    #define OP_X(Field) \
        debug##Field,
        EDGE_MAKER
    #undef OP_X
#endif
    last
};

struct EdgeFilterName {
    EdgeFilter field;
    const char* name;
} filterNames[] = {
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
#if OP_DEBUG_IMAGE
#define OP_X(s) \
    { EdgeFilter::debug##s, "debug" #s },
EDGE_IMAGE
#undef OP_X
#endif
};

struct OpSaveEF {
    OpSaveEF(std::vector<EdgeFilter>& temp) {
        save = edgeFilters[(int) defaultLevel].filter;
        for (EF ef = (EF) 0; ef < EF::last; ef = (EF)((int) ef + 1)) {
            if (temp.end() == std::find(temp.begin(), temp.end(), ef))
                edgeFilters[(int) defaultLevel].filter.push_back(ef);
        }
    }
    ~OpSaveEF() {
        edgeFilters[(int) defaultLevel].filter = save;
    }
    std::vector<EdgeFilter> save;
};

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
        s.pop_back();
        if (':' == s.back())
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
    if (fscanf(file, "lineWidth: %d\n", &lineWidth) != 1) {
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
    int saveWidth = lineWidth;
    lineWidth = 100;
    auto dmpOne = [file](std::vector<EdgeFilter>& efSet, std::string name) {
        std::string s = name + ": ";
        for (auto ef : efSet) {
            if ((size_t) ef < ARRAY_COUNT(filterNames))
                s += filterNames[(size_t) ef].name + std::string(", ");
        }
        s.pop_back();
        if (',' == s.back())
            s.pop_back();
        s = stringFormat(s);
        fprintf(file, "%s\n", s.c_str());
    };
    for (int level = 0; level < 3; ++level) {
        fprintf(file, "%s\n", !level ? "brief" : 1 == level ? "normal" : "detailed"); 
        dmpOne(edgeFilters[level].filter, "filter");
        dmpOne(edgeFilters[level].always, "always");
    }
    lineWidth = saveWidth;
    fprintf(file, "lineWidth: %d\n", lineWidth);
    fprintf(file, "defaultBase: %d\n", (int) defaultBase);
    fprintf(file, "defaultLevel: %d\n", (int) defaultLevel);
}

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

ENUM_NAME_STRUCT_ABBR(SectType);
#define SECTTYPE_NAME(w, abbr) { SectType::w, #w, #abbr }

static SectTypeName sectTypeNames[] = {
    SECTTYPE_NAME(none, none),
    SECTTYPE_NAME(endHull, end),
    SECTTYPE_NAME(controlHull, ctrl),
	SECTTYPE_NAME(midHull, mid),
	SECTTYPE_NAME(snipLo, snpL),
	SECTTYPE_NAME(snipHi, snpH),
};

ENUM_NAME_ABBR(SectType, sectType)

ENUM_NAME_STRUCT(Unsortable);
#define UNSORTABLE_NAME(w) { Unsortable::w, #w }

static UnsortableName unsortableNames[] = {
    UNSORTABLE_NAME(none),
    UNSORTABLE_NAME(addCalcFail),
    UNSORTABLE_NAME(addCalcFail2),
    UNSORTABLE_NAME(filler),
	UNSORTABLE_NAME(homeUnsectable),
	UNSORTABLE_NAME(noMidT),
	UNSORTABLE_NAME(noNormal),
	UNSORTABLE_NAME(rayTooShallow),
	UNSORTABLE_NAME(tooManyTries),
	UNSORTABLE_NAME(underflow),
};

ENUM_NAME(Unsortable, unsortable)


std::string EdgePal::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "edge:" + STR(edge->id) + " ";
    if (!OpMath::IsNaN(cept))
        s += debugValue(l, b, "cept", cept) + " ";
    if (!OpMath::IsNaN(edgeInsideT))
        s += debugValue(l, b, "edgeInsideT", edgeInsideT) + " ";
    if (reversed) 
        s += "r ";
    if (debugUID)
        s += "debugUID:" + STR(debugUID) + " ";
    if (s.size())
        s.pop_back();
    return s;
}

void EdgePal::dumpSet(const char*& str) {
    OpDebugRequired(str, "edge");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "cept"))
        cept = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "edgeInsideT"))
        edgeInsideT = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "r"))
        reversed = true;
    if (OpDebugOptional(str, "debugUID"))
        debugUID = OpDebugReadSizeT(str);
}

std::string OpEdge::debugDump(DebugLevel l, DebugBase b) const {
    auto findFilter = [](const std::vector<EdgeFilter>& set, EdgeFilter match) {
        return set.end() != std::find(set.begin(), set.end(), match);
    };
    auto dumpAlways = [findFilter](EdgeFilter match) {
        return findFilter(edgeFilters[(int) defaultLevel].always, match);
    };
    auto dumpIt = [findFilter, dumpAlways](EdgeFilter match) {
        return !findFilter(edgeFilters[(int) defaultLevel].filter, match) || dumpAlways(match);
    };
    auto strLabel = [l](std::string label) {
        return debugLabel(l, label);
    };
    auto strCurve = [b, l, strLabel](std::string label, const OpCurve& c) {
        return strLabel(label) + ":" + c.debugDump(l, b) + " ";
    };
    auto strPts = [b, l, strLabel](std::string label, const LinePts& p) {
        return strLabel(label) + " 0:" + p.pts[0].debugDump(l, b) + " 1:"
                + p.pts[1].debugDump(l, b) + " ";
    };
    auto strEdge = [dumpIt, strLabel](EdgeFilter match, std::string label, const OpEdge* edge) {
        if (!dumpIt(match))
            return std::string("");
        return strLabel(label) + ":" + (edge ? STR(edge->id) : std::string("-")) + "/";
    };
    auto strFloat = [dumpIt, dumpAlways, b](EdgeFilter match, std::string label, float t) {
        if (!dumpIt(match) || (!dumpAlways(match) && !OpMath::IsFinite(t)))
            return std::string("");
        return debugValue(DebugLevel::error, b, label, t) + " ";
    };
    auto strPtT = [dumpIt, dumpAlways, b, strLabel](EdgeFilter match, std::string label,
                OpPtT ptT, std::string suffix) {
        if (!dumpIt(match) || (!dumpAlways(match) && (!ptT.pt.isFinite() || !OpMath::IsFinite(ptT.t))))
            return std::string("");
        return strLabel(label) + ptT.debugDump(DebugLevel::error, b) + suffix;
    };
    auto strID = [dumpIt, dumpAlways, strLabel](EdgeFilter match, std::string label, int ID) {
        if (!dumpIt(match) || (!dumpAlways(match) && !ID))
            return std::string("");
        return strLabel(label) + "[" + STR(ID) + "] ";
    };
    auto strBounds = [dumpAlways, l, b, strLabel](EdgeFilter match, 
            std::string label, const OpPointBounds& bounds) {
        if (!dumpAlways(match) && !bounds.isSet())
            return std::string("");
        return strLabel(label) + bounds.debugDump(l, b)+ " ";
    };
    auto strWinding = [dumpAlways, l, b, strLabel](EdgeFilter match, std::string label,
             const OpWinding& wind) {
        std::string s;
        if (!dumpAlways(match)) {
            if (!wind.contour)
                return s;
        }
        s += strLabel(label) + ":" + wind.debugDump(l, b) + " ";
        return s;
    };
    auto strEnum = [dumpIt, dumpAlways, strLabel](EdgeFilter match, std::string label,
            bool enumHasDefault, std::string enumName) {
        if (!dumpIt(match) || (!dumpAlways(match) && enumHasDefault))
            return std::string("");
        return strLabel(label) + ":" + enumName + " ";
    };
    std::string s = strID(EdgeFilter::id, "edge", id);
    if (dumpIt(EdgeFilter::segment) && segment) 
        s += strID(EdgeFilter::segment, "segment", segment->id);
    if (dumpIt(EdgeFilter::contour) && segment && segment->contour)
        s += strID(EF::contour, "contour", segment->contour->id);
    if (ray.distances.size() && dumpIt(EdgeFilter::ray)) 
        s += ray.debugDump(l, b) + " ";
    if (priorEdge || nextEdge || lastEdge || dumpAlways(EF::priorEdge) || dumpAlways(EF::nextEdge)) { 
        s += strEdge(EdgeFilter::priorEdge, "prior", priorEdge);
        s += strEdge(EdgeFilter::nextEdge, "next", nextEdge);
        s += strEdge(EdgeFilter::lastEdge, "last", lastEdge);
        if ('/' == s.back()) s.back() = ' ';
    }
    s += strPtT(EdgeFilter::center, "center", center, " ");
    if (dumpIt(EdgeFilter::curve)) s += strCurve("curve", curve);
    if (upright_impl.pts[0].isFinite() || upright_impl.pts[1].isFinite()) {
        if (dumpIt(EdgeFilter::upright_impl))
            s += strPts("upright_impl", upright_impl);
        if (dumpIt(EdgeFilter::vertical_impl))
            s += strCurve("vertical_impl", vertical_impl);
    }
    if (dumpIt(EdgeFilter::ptBounds)) s += strBounds(EdgeFilter::ptBounds, "ptBounds", ptBounds);
    if (dumpIt(EdgeFilter::linkBounds)) s += strBounds(EF::linkBounds, "linkBounds", linkBounds);
    if (dumpIt(EdgeFilter::winding)) s += strWinding(EdgeFilter::winding, "winding", winding);
    if (dumpIt(EdgeFilter::sum)) s += strWinding(EdgeFilter::sum, "sum", sum);
    if (dumpIt(EdgeFilter::many)) s += strWinding(EdgeFilter::many, "many", many);
    if (dumpIt(EdgeFilter::coinPals) && (dumpAlways(EdgeFilter::coinPals) || coinPals.size())) {
        s += strLabel("coinPals") + "[";
        for (auto& cPal : coinPals) {
            s += "{opp:" + STR(cPal.opp->id) + " coinID:" + STR(cPal.coinID) + "} ";
        }
        s.pop_back();
        s += "] ";
    }
    if (dumpIt(EdgeFilter::unSects) && (dumpAlways(EdgeFilter::unSects) || unSects.size())) {
        s += strLabel("unSects") + "[";
        for (auto& uSect : unSects)
            s += STR(uSect->id) + " ";
        s.pop_back();
        s += "] ";
    }
    if (dumpIt(EdgeFilter::pals) && (dumpAlways(EdgeFilter::pals) || pals.size())) {
        s += strLabel("pals") + "[";
        for (auto& pal : pals) {
            s += pal.debugDump(DebugLevel::brief, b) + " ";
        }
        s.pop_back();
        s += "] ";
    }
#if 0
    if (dumpIt(EdgeFilter::lessRay) && (dumpAlways(EdgeFilter::lessRay) || lessRay.size())) {
        s += strLabel("lessRay") + "[";
        for (auto less : lessRay)
            s += STR(less->id) + " ";
        if (' ' == s.back()) s.pop_back();
        s += "] ";
    }
    if (dumpIt(EdgeFilter::moreRay) && (dumpAlways(EdgeFilter::moreRay) || moreRay.size())) {
        s += strLabel("moreRay") + "[";
        for (auto more : moreRay)
            s += STR(more->id) + " ";
        if (' ' == s.back()) s.pop_back();
        s += "] ";
    }
#endif
    if (dumpIt(EdgeFilter::hulls) && (dumpAlways(EdgeFilter::hulls) || hulls.h.size())) {
        s += "hulls[";  // don't abbreviate in brief
        for (auto& hs : hulls.h)
            s += hs.debugDump(l, b) + " ";
        if (' ' == s.back()) s.pop_back();
        s += "] ";
    }
    s += strFloat(EdgeFilter::startT, "startT", startT);
    s += strFloat(EdgeFilter::endT, "endT", endT);
    s += strEnum(EF::whichEnd_impl, "whichEnd", EdgeMatch::none == which(), edgeMatchName(which()));
    s += strEnum(EF::rayFail, "rayFail", EdgeFail::none == rayFail, edgeFailName(rayFail));
    s += strEnum(EF::windZero, "windZero", false, windZeroName(windZero));
    s += strEnum(EF::isUnsortable, "isUnsortable", Unsortable::none == isUnsortable, unsortableName(isUnsortable));
#define STR_BOOL(ef) do { if (dumpIt(EdgeFilter::ef) && (dumpAlways(EdgeFilter::ef) || ef)) { \
        s += strLabel(#ef) + " "; \
        if (1 != ((unsigned char) ef)) s += STR((size_t) ef) + " "; }} while(false)
	STR_BOOL(active_impl);
    STR_BOOL(inLinkups);
    STR_BOOL(inOutput);
    STR_BOOL(disabled);
    STR_BOOL(isUnsplitable);
    STR_BOOL(ccEnd);
    STR_BOOL(ccLarge);
    STR_BOOL(ccOverlaps);
    STR_BOOL(ccSmall);
    STR_BOOL(ccStart);
    STR_BOOL(centerless);
    STR_BOOL(startSeen);
    STR_BOOL(endSeen);
#if OP_DEBUG
    if (dumpIt(EdgeFilter::debugMatch) && (dumpAlways(EdgeFilter::debugMatch) || debugMatch))
        s += (debugMatch ? STR(debugMatch->id) : std::string("-")) + " ";
    if (dumpIt(EdgeFilter::debugZeroErr) && (dumpAlways(EdgeFilter::debugZeroErr) || debugZeroErr))  
        s += (debugZeroErr ? STR(debugZeroErr->id) : std::string("-")) + " ";
#if OP_DEBUG_MAKER
    if (dumpIt(EF::debugSetDisabled) && (dumpAlways(EdgeFilter::debugSetDisabled) 
            || debugSetDisabled.line))
        s += "debugSetDisabled:" + debugSetDisabled.debugDump() + " ";
    if (dumpIt(EF::debugSetMaker)) {
        if (DebugLevel::file == l)
            s += "debugSetMaker:";
        s += debugSetMaker.debugDump() + " ";
    }
    if (dumpIt(EF::debugSetSum) && (dumpAlways(EF::debugSetSum) || debugSetSum.line))
        s += "debugSetSum:" + debugSetSum.debugDump() + " ";
#endif
    s += strID(EF::debugOutPath, "debugOutPath", debugOutPath);
    s += strID(EF::debugParentID, "debugParentID", debugParentID);
    s += strID(EF::debugRayMatch, "debugRayMatch", debugRayMatch);
#endif
#if OP_DEBUG_IMAGE
    if (dumpIt(EF::debugColor) && (dumpAlways(EF::debugColor) || debugBlack != debugColor))
        s += debugDumpColor(debugColor) + " ";
    STR_BOOL(debugDraw);
    STR_BOOL(debugJoin);
    STR_BOOL(debugCustom);
#endif
#undef STR_BOOL
    return s;
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
        if (OpDebugOptional(str, label))
            return OpDebugReadSizeT(str);
        return (size_t) 0;
    };
    id = strID("edge");
    segment = (OpSegment*) strID("segment");
    (void) strID("contour");  // can't do anything with this here
    ray.dumpSet(str);
    priorEdge = (OpEdge*) strID("prior");  // non-zero must be replaced with pointer later
    nextEdge = (OpEdge*) strID("next");
    lastEdge = (OpEdge*) strID("last");
    if (OpDebugOptional(str, "center"))
        center.dumpSet(str);
    OpDebugRequired(str, "curve");
    curve.contours = dumpContours;
    curve.dumpSet(str);
    if (OpDebugOptional(str, "upright_impl")) {
        upright_impl.dumpSet(str);
        OpDebugRequired(str, "vertical_impl");
        vertical_impl.contours = dumpContours;
        vertical_impl.dumpSet(str);
    }
    OpDebugRequired(str, "ptBounds");
    ptBounds.dumpSet(str);
    if (OpDebugOptional(str, "linkBounds"))
        linkBounds.dumpSet(str);
    if (OpDebugOptional(str, "winding"))
        winding.dumpSet(str, dumpContours);
    if (OpDebugOptional(str, "sum"))
        sum.dumpSet(str, dumpContours);
    if (OpDebugOptional(str, "many"))
        many.dumpSet(str, dumpContours);
    // !!! add cSects
    // !!! add uSects
    // !!! add uPals
    if (OpDebugOptional(str, "pals")) {
        while (OpDebugOptional(str, "e[")) {
            pals.resize(pals.size() + 1);
            pals.back().dumpSet(str);
        }
    }
#if 0
    if (OpDebugOptional(str, "lessRay[")) {
        int edgeCount = OpDebugCountDelimiters(str, ' ', '[', ']');
        lessRay.resize(edgeCount);
        for (int index = 0; index < edgeCount; ++index)
            lessRay[index] = (OpEdge*) OpDebugReadSizeT(str);
    }
    if (OpDebugOptional(str, "moreRay[")) {
        int edgeCount = OpDebugCountDelimiters(str, ' ', '[', ']');
        moreRay.resize(edgeCount);
        for (int index = 0; index < edgeCount; ++index)
            moreRay[index] = (OpEdge*) OpDebugReadSizeT(str);
    }
#endif
    if (OpDebugOptional(str, "hulls[")) {
        int hullCount = OpDebugCountDelimiters(str, ':', '[', ']') / 2;  // ':' used twice per hull
        hulls.h.resize(hullCount);
        for (int index = 0; index < hullCount; ++index)
            hulls.h[index].dumpSet(str);
    }
    startT = OpDebugReadNamedFloat(str, "startT");
    endT = OpDebugReadNamedFloat(str, "endT");
    whichEnd_impl = edgeMatchStr(str, "whichEnd", EdgeMatch::none);
    rayFail = edgeFailStr(str, "rayFail", EdgeFail::none);
    windZero = windZeroStr(str, "windZero", WindZero::unset);
    isUnsortable = unsortableStr(str, "unsortable", Unsortable::none);
#define STR_BOOL(ef) ef = OpDebugOptional(str, #ef)
	STR_BOOL(active_impl);
    STR_BOOL(inLinkups);
    STR_BOOL(inOutput);
    STR_BOOL(disabled);
    STR_BOOL(isUnsplitable);
    STR_BOOL(ccEnd);
    STR_BOOL(ccLarge);
    STR_BOOL(ccOverlaps);
    STR_BOOL(ccSmall);
    STR_BOOL(ccStart);
    STR_BOOL(centerless);
    STR_BOOL(startSeen);
    STR_BOOL(endSeen);
#if OP_DEBUG
    debugMatch = (OpEdge*) strID("debugMatch");
    debugZeroErr = (OpEdge*) strID("debugZeroErr");
#endif
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
    if (OpDebugOptional(str, "debugSetMaker"))
        debugSetMaker.dumpSet(str);
    if (OpDebugOptional(str, "debugSetSum"))
        debugSetSum.dumpSet(str);
#endif
#if OP_DEBUG
    debugOutPath = strID("debugOutPath");
    debugParentID = strID("debugParentID");
    debugRayMatch = strID("debugRayMatch");
#endif
#if OP_DEBUG_IMAGE
    // !!! skip debug color for now
    STR_BOOL(debugDraw);
    STR_BOOL(debugJoin);
    STR_BOOL(debugCustom);
#endif
#undef STR_BOOL
    // !!! skip missing if present
}

void OpEdge::dumpResolveAll(OpContours* c) {
    c->dumpResolve(segment);
    c->dumpResolve(priorEdge);
    c->dumpResolve(nextEdge);
    c->dumpResolve(lastEdge);
//    for (auto& less : lessRay)
//        c->dumpResolve(less);
//    for (auto& more : moreRay)
//        c->dumpResolve(more);
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

// keep this in sync with op edge : is loop
std::string OpEdge::debugDumpLink(EdgeMatch which, DebugLevel l, DebugBase b) const {
    const OpEdge* looped = debugIsLoop(which, LeadingLoop::in);
    bool firstLoop = false;
    int safetyCount = 0;
    const OpEdge* link = this;
    std::string s;
    while ((link = EdgeMatch::start == which ? link->priorEdge : link->nextEdge)) {
        s += "\n" + link->debugDump(l, b);
        if (link == looped) {
            if (firstLoop)
                return s + " loop";
            firstLoop = true;
        }
        if (++safetyCount > 700) {
            OpDebugOut(std::string("!!! likely loops forever: ") + 
                    (EdgeMatch::start == which ? "prior " : "next "));
            break;
        }
    }
    if (s.size())
        s = " for:" + STR(id) + s;
    return s;
}

std::string OpEdge::debugDumpWinding() const {
    std::string s;
    if (winding.isSet())
        s += "winding: " + winding.debugDump(defaultLevel, defaultBase) + " ";
    if (sum.isSet())
        s += "sum: " + sum.debugDump(defaultLevel, defaultBase) + " ";
    if (many.isSet())
        s += "many: " + many.debugDump(defaultLevel, defaultBase);
    return s;
}

void dmpWinding(const OpEdge& edge) {
    std::string s = edge.debugDumpWinding();
    OpDebugOut(s + "\n");
}

void dmpEnd(const OpEdge& edge)  {
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
    OpPoint c = { (ptBounds.left + ptBounds.right) / 2, (ptBounds.top + ptBounds.bottom) / 2 };
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

std::string OpEdge::debugDumpPoints() const {
    std::string s = "[" + STR(id) + "]";
    s += " " + debugValue(DebugLevel::error, defaultBase, "startT", startT);
    s += " " + debugValue(DebugLevel::error, defaultBase, "endT", endT);
    s += " curve:" + curve.debugDump(defaultLevel, defaultBase);
    s += " which:" + edgeMatchName(which());
    const OpEdge* startE = debugAdvanceToEnd(EdgeMatch::start);
    if (startE != this)
        s += " start[" + STR(startE->id) + "] " + startE->whichPtT()
                .debugDump(defaultLevel, defaultBase);
    const OpEdge* endE = debugAdvanceToEnd(EdgeMatch::end);
    if (endE != this)
        s += " end[" + STR(endE->id) + "] " + endE->whichPtT(!endE->which())
                .debugDump(defaultLevel, defaultBase);
    return s;
}

OpPtT dc_ex, dc_ey, dc_ox, dc_oy;
extern void draw(const OpPtT& );

void OpCurveCurve::drawClosest(const OpPoint& originalPt) const {
    dumpClosest(originalPt);
    ::draw(dc_ex);
    ::draw(dc_ey);
    ::draw(dc_ox);
    ::draw(dc_oy);
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
#if 0 //!!! move findT to debug so it can be called here
        OpPtT result = e->findT(axis, originalPt.choice(axis));
#else
		OpPtT result;
#endif
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

void dmpLink(const OpEdge& edge) {
   std::string s = edge.debugDump(defaultLevel, defaultBase);
   OpDebugOut(s + "\n");
   s = edge.debugDumpLink(EdgeMatch::start, defaultLevel, defaultBase);
   if (s.size())
       OpDebugOut("prior" + s + "\n");
   s = edge.debugDumpLink(EdgeMatch::end, defaultLevel, defaultBase);
   if (s.size())
       OpDebugOut("next" + s + "\n");
}

void dmpPoints(const OpEdge& edge) {
    std::string s = edge.debugDumpPoints();
    OpDebugFormat(s);
}

void dmpStart(const OpEdge& edge) {
    dmpMatch(edge.startPt());
}

std::string CallerDataStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "next:" + STR(next) + " ";  // only zero/nonzero is read
    s += "used:" + STR(used) + " ";
    if (DebugLevel::detailed == l || DebugLevel::file == l) {
        s += "\n";
        s += OpDebugDumpByteArray(storage, used);   // 'b' is ignored for now; always return hex
        if (next)
            s += "\n";
    }
    if (next)
        s += " " + next->debugDump(l, b);
    if (' ' == s.back())
        s.pop_back();
    return s;
}

void CallerDataStorage::DumpSet(const char*& str, CallerDataStorage** previousPtr) {
    CallerDataStorage* storage = new CallerDataStorage;
    *previousPtr = storage;
    OpDebugRequired(str, "next");
    storage->next = (CallerDataStorage*) OpDebugReadSizeT(str);  // non-zero means there is more
    OpDebugRequired(str, "used");
    storage->used = OpDebugReadSizeT(str);
    std::vector<uint8_t> bytes = OpDebugByteArray(str);
    OP_ASSERT(storage->used == bytes.size());
    std::memcpy(storage->storage, &bytes.front(), storage->used);
    if (storage->next)
        DumpSet(str, &storage->next);
}

size_t OpEdgeStorage::debugCount() const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return 0;
#endif
    size_t result = used;
    OpEdgeStorage* block = next;
    while (block) {
        result += block->used;
        block = block->next;
    }
    return result;
}

OpEdge* OpEdgeStorage::debugFind(int ID) {
	for (size_t index = 0; index < used; index++) {
		OpEdge& test = storage[index];
        if (test.id == ID ||
                test.debugOutPath == ID || test.debugRayMatch == ID)
            return &test;
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

OpEdge* OpEdgeStorage::debugIndex(size_t index) {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return nullptr;
#endif
    OpEdgeStorage* block = this;
    while (index > block->used) {
        index -= block->used;
        block = block->next;
        if (!block)
            return nullptr;
    }
    if (block->used <= index)
        return nullptr;
    return &block->storage[index];
}

std::string OpEdgeStorage::debugDump(std::string label, DebugLevel l, DebugBase b) {
    std::string s;
    size_t count = debugCount();
    if (!count)
        return s;
    s = label + ":" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (size_t index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
	    for (size_t index = 0; index < count; index++) {
		    const OpEdge* test = debugIndex(index);
            s += test->debugDump(l, b) + "\n";
	    }
        s.pop_back();
    }
    return s;
}

std::string OpEdgeStorage::debugDump(DebugLevel l, DebugBase b) const {
    OP_ASSERT(0);  // !!! call the label version above instead
    return "";
}

void OpEdgeStorage::DumpSet(const char*& str, OpContours* dumpContours, DumpStorage type) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpEdge* edge = nullptr;
        // !!! hackery ahead: note that 'contours->allocateEdge(this)' won't compile
        if (DumpStorage::cc == type)
            edge = dumpContours->allocateEdge(dumpContours->ccStorage);
        else if (DumpStorage::filler == type)
            edge = dumpContours->allocateEdge(dumpContours->fillerStorage);
        else {
            OpDebugExit("edge storage missing");
        }
        (void) new(edge) OpEdge();
        edge->dumpContours = dumpContours;
        edge->dumpSet(str);
    }
}

void OpEdgeStorage::dumpResolveAll(OpContours* c) {
    size_t count = debugCount();
    for (size_t index = 0; index < count; ++index)
        debugIndex(index)->dumpResolveAll(c);
}

size_t OpLimbStorage::debugCount() const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return 0;
#endif
    size_t result = used;
    OpLimbStorage* block = nextBlock;
    while (nextBlock) {
        result += block->used;
        block = block->nextBlock;
    }
    return result;
}

const OpLimb* OpLimbStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index++) {
        if (storage[index].id == ID)
            return &storage[index];
    }
    if (nextBlock)
        return nextBlock->debugFind(ID);
    return nullptr;
}

OpLimb* OpLimbStorage::debugIndex(int index) {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return nullptr;
#endif
    OpLimbStorage* block = this;
    while (index > block->used) {
        index -= block->used;
        block = block->nextBlock;
        if (!block)
            return nullptr;
    }
    if (block->used <= index)
        return nullptr;
    return &block->storage[index];
}

std::string OpLimbStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    size_t count = debugCount();
    if (!count)
        return s;
    s += "limbStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
#if OP_DEBUG
        s += "[";
        for (size_t index = 0; index < count; ++index)
            s += STR(debugFind(index)->id) + " ";
        s.pop_back();
        s += "]";
#endif
    } else {
        for (size_t index = 0; index < count; ++index)
            s += debugFind(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
}

void OpLimbStorage::DumpSet(const char*& str, OpContours* dumpContours) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpLimb* limb = dumpContours->allocateLimb();
        limb->dumpSet(str);
    }
}

void OpLimbStorage::dumpResolveAll(OpContours* c) {
    size_t count = debugCount();
    for (size_t index = 0; index < count; ++index)
        debugIndex(index)->dumpResolveAll(c);
}

ENUM_NAME_STRUCT(LinkPass);
#define LINKPASS_NAME(r) { LinkPass::r, #r }

LinkPassName linkPassNames[] = {
    LINKPASS_NAME(none),
	LINKPASS_NAME(normal),
	LINKPASS_NAME(unsectable),
	LINKPASS_NAME(remaining),
};

ENUM_NAME(LinkPass, linkPass)

std::string OpJoiner::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l) {
        auto dumpEdgeIDs = [&s](const std::vector<OpEdge*>& edges, std::string name) {
            if (!edges.size())
                return;
            s += name + ":" + STR(edges.size()) + " [";
            for (auto e : edges)
                s += STR(e->id) + " ";
            s.pop_back();
            s += "]\n";
        };
        dumpEdgeIDs(byArea, "byArea");
        dumpEdgeIDs(unsectByArea, "unsectByArea");
        dumpEdgeIDs(disabled, "disabled");
        dumpEdgeIDs(disabledPals, "disabledPals");
        dumpEdgeIDs(unsortables, "unsortables");
    } else {
        auto dumpEdges = [&s, l, b](const std::vector<OpEdge*>& edges, std::string name) {
            size_t activeCount = 0;
            for (auto e : edges)
                activeCount += e->isActive();
            if (!activeCount)
                return;
            s += "-- " + name + " " + STR(activeCount);
            if (edges.size() != activeCount)
                s += " (inactive: " + STR(edges.size() - activeCount) + ")";
            s += "\n";
            for (auto e : edges)
                s += e->debugDump(l, b) + "\n";
        };
        // set up edge::debugDump to only show joiner relevant fields
        std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, EF::winding,
                EF::sum, EF::many, EF::pals, EF::whichEnd_impl };
        OpSaveEF saveEF(showFields);
        dumpEdges(byArea, "byArea");
        dumpEdges(unsectByArea, "unsectByArea");
        dumpEdges(disabled, "disabled");
        dumpEdges(disabledPals, "disabledPals");
        dumpEdges(unsortables, "unsortables");
    }
    if (found.size()) {
        if (DebugLevel::file != l)
            s += "-- ";
        s += "found:" + STR(found.size()) + "\n";
    }
    for (const FoundEdge& f : found)
        s += f.debugDump(l, b) + "\n";
    if (bestGap.edge)
        s += "bestGap:" + bestGap.debugDump(l, b) + "\n";
    if (linkups.l.size()) {
        if (DebugLevel::file == l) {
            s += "linkups:" + STR(linkups.l.size()) + " [";
            for (OpEdge* linkup : linkups.l)
                s += STR(linkup->id) + " ";
            s.pop_back();
            s += "]\n";
        } else {
            s += "";
            s += "-- linkups:" + STR(linkups.l.size()) + "\n";
            s += linkups.debugDump(l, b) + "\n";
        }
    }
    s += "linkMatch:" + edgeMatchName(linkMatch) + " ";
    s += "linkPass:" + linkPassName(linkPass) + " ";
    if (edge) {
        s += "edge:";
        if (DebugLevel::file == l)
            s += STR(edge->id) + " ";
        else
            s += "edge:" + edge->debugDump(l, b) + "\n";
    }
    if (lastLink) {
        s += "lastLink:";
        if (DebugLevel::file == l)
            s += STR(lastLink->id) + " ";
        else
            s += lastLink->debugDump(l, b) + "\n";
    }
    if (!OpMath::IsNaN(matchPt.x) && !OpMath::IsNaN(matchPt.y))
        s += "matchPt:" + matchPt.debugDump(l, b) + " ";
    if (disabledBuilt)
        s += "disabledBuilt ";
    if (disabledPalsBuilt)
        s += "disabledPalsBuilt ";
    s.pop_back();
    return s;
}

void OpJoiner::dumpSet(const char*& str) {
    auto setEdgeIDs = [&str](std::vector<OpEdge*>& edges, const char* name) {
        if (!OpDebugOptional(str, name)) 
            return;
        size_t count = OpDebugReadSizeT(str);
        edges.resize(count);
        for (size_t index = 0; index < count; ++index)
            edges[index] = (OpEdge*) OpDebugReadSizeT(str);
    };
    setEdgeIDs(byArea, "byArea");
    setEdgeIDs(unsectByArea, "unsectByArea");
    setEdgeIDs(disabled, "disabled");
    setEdgeIDs(disabledPals, "disabledPals");
    setEdgeIDs(unsortables, "unsortables");
    if (OpDebugOptional(str, "found")) {
        size_t count = OpDebugReadSizeT(str);
        for (size_t index = 0; index < count; ++index)
            found[index].dumpSet(str);
    }
    OpDebugRequired(str, "bestGap");
    bestGap.dumpSet(str);
    if (OpDebugOptional(str, "linkups")) {
        size_t count = OpDebugReadSizeT(str);
        linkups.l.resize(count);
        for (size_t index = 0; index < count; ++index)
            linkups.l[index] = (OpEdge*) OpDebugReadSizeT(str);
    }
    linkMatch = edgeMatchStr(str, "linkMatch", EdgeMatch::none);
    linkPass = linkPassStr(str, "linkPass", LinkPass::none);
    if (OpDebugOptional(str, "edge"))
        edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "lastLink"))
        lastLink = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "matchPt"))
        matchPt.dumpSet(str);
    disabledBuilt = OpDebugOptional(str, "disabledBuilt");
    disabledPalsBuilt = OpDebugOptional(str, "disabledPalsBuilt");
}

void OpJoiner::dumpResolveAll(OpContours* c) {
    auto resolveEdgeIDs = [c](std::vector<OpEdge*>& edges) {
        for (OpEdge*& e : edges)
            c->dumpResolve(e);
    };
    resolveEdgeIDs(byArea);
    resolveEdgeIDs(unsectByArea);
    resolveEdgeIDs(disabled);
    resolveEdgeIDs(disabledPals);
    resolveEdgeIDs(unsortables);
    bestGap.dumpResolveAll(c);
    for (OpEdge*& e : linkups.l)
        c->dumpResolve(e);
    c->dumpResolve(edge);
    c->dumpResolve(lastLink);
}

ENUM_NAME_STRUCT(LimbPass);
#define LIMBPASS_NAME(r) { LimbPass::r, #r }

LimbPassName limbPassNames[] = {
	LIMBPASS_NAME(none),
	LIMBPASS_NAME(linked),
    LIMBPASS_NAME(unlinked),
    LIMBPASS_NAME(unsectPair),
	LIMBPASS_NAME(disabled),
	LIMBPASS_NAME(disabledPals),
	LIMBPASS_NAME(miswound),
	LIMBPASS_NAME(disjoint),
    LIMBPASS_NAME(unlinkedPal),
};

ENUM_NAME(LimbPass, limbPass)

std::string OpLimb::debugDumpIDs(DebugLevel l, bool bracket) const {
    std::string s = (bracket ? "[" : "id:") + STR(id);
    if (edge) {
        s += (bracket ? " e:" : " edge:") + STR(edge->id);
        if (DebugLevel::file == l)
            return s;
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
        if (gapDistance)
            s += " gapD:" + STR(gapDistance);
        s += " closeD:" + STR(closeDistance);
        if (bracket)
            s += "]";
    }
    return s;
}

std::string OpLimb::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = debugDumpIDs(l, false);  // note: dumps edge
    if (bounds.isFinite())
        s += " bounds:" + bounds.debugDump(l, b);
    if (lastLimbEdge)
        s += " lastLimbEdge:" + STR(lastLimbEdge->id);
    if (parent)
        s += " parent:" + parent->debugDumpIDs(l, true);
    if (lastPtT.pt.isFinite())
        s += " lastPtT:" + lastPtT.debugDump(l, b);
    if (OpMax != linkedIndex)
        s += " linkedIndex:" + STR((int) linkedIndex);
    if (!OpMath::IsNaN(gapDistance))
        s += " gapDistance:" + STR(gapDistance);
    if (!OpMath::IsNaN(closeDistance))
        s += " closeDistance:" + STR(closeDistance);
    if (EdgeMatch::none != match)
        s += " match:" + edgeMatchName(match);
    if (EdgeMatch::none != lastMatch)
        s += " lastMatch:" + edgeMatchName(lastMatch);
    if (LimbPass::none != treePass)
        s += " treePass:" + limbPassName(treePass);
    if (deadEnd != (bool) -1)
        s += " deadEnd";
    if (looped != (bool) -1)
        s += " looped";
    if (resetPass != (bool) -1)
        s += " resetPass";
    if (debugBranches.size()) {
        s += " debugBranches:" + STR(debugBranches.size()) + " [";
        for (auto limb : debugBranches)
            s += STR(limb->id) + " ";
        s.pop_back();
        s += "]";
    }

    return s;
}

void OpLimb::dumpResolveAll(OpContours* c) {
    c->dumpResolve(edge);
    c->dumpResolve(lastLimbEdge);
    c->dumpResolve(parent);
    for (const OpLimb* limb : debugBranches)
        c->dumpResolve(limb);
}

void OpLimb::dumpSet(const char*& str) {
    OpDebugRequired(str, "id");
    id = OpDebugReadSizeT(str);
    edge = (OpEdge*) (OpDebugOptional(str, "edge") ? OpDebugReadSizeT(str) : 0);
    if (OpDebugOptional(str, "bounds"))
        bounds.dumpSet(str);
    lastLimbEdge = (OpEdge*) (OpDebugOptional(str, "lastLimbEdge") ? OpDebugReadSizeT(str) : 0);
    parent = (const OpLimb*) (OpDebugOptional(str, "parent") ? OpDebugReadSizeT(str) : 0);
    if (OpDebugOptional(str, "lastPtT"))
        lastPtT.dumpSet(str);
    linkedIndex = OpDebugOptional(str, "linkedIndex") ? OpDebugReadSizeT(str) : OpMax;
    gapDistance = OpDebugReadNamedFloat(str, "gapDistance");
    closeDistance = OpDebugReadNamedFloat(str, "closeDistance");
    match = edgeMatchStr(str, "match", EdgeMatch::none);
    lastMatch = edgeMatchStr(str, "lastMatch", EdgeMatch::none);
    treePass = limbPassStr(str, "treePass", LimbPass::unlinked);
    looped = OpDebugOptional(str, "looped");
    resetPass = OpDebugOptional(str, "resetPass");
    if (OpDebugOptional(str, "debugBranches")) {
        size_t count = OpDebugReadSizeT(str);
        for (size_t index = 0; index < count; ++index)
            debugBranches.push_back((OpLimb*) OpDebugReadSizeT(str));
    }
}

std::string OpTree::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
//    if (limbStorage)
//        s += " limbStorage:" + OpDebugPtrToHex(limbStorage) + " used:" + STR(limbStorage->used);
//    if (current)
//        s += " current:" + OpDebugPtrToHex(current) + " used:" + STR(current->used);
//    if (contours)
//        s += " contours:" + STR(contours->id);
    if (bestGapLimb)
        s += " bestGapLimb:" + bestGapLimb->debugDumpIDs(l, true);
    if (bestLimb)
        s += " bestLimb:" + bestLimb->debugDumpIDs(l, true);
    if (firstPt.isFinite())
        s += " firstPt:" + firstPt.debugDump(l, b);
    if (LimbPass::none != limbPass)
        s += " limbPass:" + limbPassName(limbPass);
    if (OpMath::IsFinite(bestDistance))
        s += " bestDistance:" + debugFloat(b, bestDistance);
    if (OpMath::IsFinite(bestPerimeter))
        s += " bestPerimeter:" + debugFloat(b, bestPerimeter);
//    if (baseIndex)
//        s += " baseIndex:" + STR(baseIndex);
    if (totalUsed)
        s += " totalUsed:" + STR(totalUsed);
    s.erase(s.begin());
    if (DebugLevel::file == l)
        return s;
    OpLimbStorage* saveCurrent = contours->limbCurrent;
    for (int index = 0; index < totalUsed; ++index) {
        const OpLimb& limb = contours->debugNthLimb(index);
        s += "\n" + limb.debugDumpIDs(l, true);
        s += " parent:" + (limb.parent ? limb.parent->debugDumpIDs(l, true) : "-");
        if (limb.debugBranches.size()) {
            s += " children:";
            for (OpLimb* child : limb.debugBranches) {
                s += child->debugDumpIDs(l, true) + " ";
            }
            s.pop_back();
        }
        s += " treePass:" + limbPassName(limb.treePass);
    }
    contours->limbCurrent = saveCurrent;
    return s;
}

std::string CoinEnd::debugDump(DebugLevel l, DebugBase b) const { 
    std::string s;
    s += "seg:" + STR(seg->id) + " opp:" + STR(opp->id) + " ptT:" + ptT.debugDump(l, b);
    s += " oppT:" + oppT.debugDump(DebugLevel::error, b);
    return s;
}

void dmp(std::array<CoinEnd, 4>& coinEndArray) {
    for (auto& cea : coinEndArray)
        OpDebugOut(cea.debugDump(defaultLevel, defaultBase) + "\n");
}

std::string OpWinder::debugDumpAxis(Axis a, DebugLevel l, DebugBase b) const {
    std::string s = "";
    for (const auto edge : Axis::vertical == a ? inY : inX) {
        s += edge->debugDump(l, b) + "\n";
    }
    return s;
}

std::string OpWinder::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (inX.size()) {
        s += "-- sorted in x --\n";
        s += debugDumpAxis(Axis::horizontal, l, b);
    }
    if (inY.size()) {
        s += "-- sorted in y --\n";
        s += debugDumpAxis(Axis::vertical, l, b);
    }
    return s;
}

std::string EdgeRun::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "e[" + STR(edge->id) + "] ";
    s += "o[" + STR(oppEdge->id) + "] ";
    s += "edgePtT:" + edgePtT.debugDump(l, b) + " ";
    s += "oppPtT:" + oppPtT.debugDump(DebugLevel::error, b) + " ";
    s += debugErrorValue(l, b, "oppDist", oppDist) + " ";
    OP_DEBUG_CODE(s += "debugBetween:" + STR(debugBetween) + " ");
    return s;
}

void EdgeRun::dumpSet(const char*& str) {
    OpDebugRequired(str, "e[");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "o[");
    oppEdge = (OpEdge*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "edgePtT:");
    edgePtT.dumpSet(str);
    OpDebugRequired(str, "oppPtT:");
    oppPtT.dumpSet(str);
    oppDist = OpDebugReadNamedFloat(str, "oppDist");
#if OP_DEBUG
    OpDebugRequired(str, "debugBetween");
    debugBetween = OpDebugReadSizeT(str);
#endif
}

void EdgeRun::dumpResolveAll(OpContours* c) {
    c->dumpResolve(edge);
    c->dumpResolve(oppEdge);
}

ENUM_NAME_STRUCT(CurveRef);

#define CURVEREF_NAME(s) { CurveRef::s, #s }

static CurveRefName curveRefNames[] {
    CURVEREF_NAME(edge),
    CURVEREF_NAME(opp),
};

ENUM_NAME(CurveRef, curveRef)

std::string FoundLimits::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (parentEdge)
        s += "parentEdge:" + STR(parentEdge->id) + " ";
    if (parentOpp)
        s += "parentOpp:" + STR(parentOpp->id) + " ";
    s += "seg:" + seg.debugDump(l, b);
    s += " opp:" + opp.debugDump(l, b);
    if (fromFoundT)
        s += " fromFoundT";
#if OP_DEBUG_MAKER
    s += " maker:" + maker.debugDump();
#endif
    return s;
}

void FoundLimits::dumpSet(const char*& str) {
    parentEdge = OpDebugOptional(str, "parentEdge") ? (const OpEdge*) OpDebugReadSizeT(str) 
            :  (const OpEdge*) 0;
    parentOpp = OpDebugOptional(str, "parentOpp") ? (const OpEdge*) OpDebugReadSizeT(str)
            :  (const OpEdge*) 0;
    OpDebugRequired(str, "seg");
    seg.dumpSet(str);
    OpDebugRequired(str, "opp");
    opp.dumpSet(str);
    fromFoundT = OpDebugOptional(str, "fromFoundT");
#if OP_DEBUG_MAKER
    OpDebugRequired(str, "maker");
    maker.dumpSet(str);
#endif
}

void FoundLimits::dumpResolveAll(OpContours* c) {
    c->dumpResolve(parentEdge);
    c->dumpResolve(parentOpp);
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

std::string SectRay::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "ray count:" + STR(distances.size()) + " ";
    s += debugValue(l, b, "normal", normal) + " ";
    s += debugValue(l, b, "homeCept", homeCept) + " ";
    s += debugValue(l, b, "homeT", homeT) + " ";
    s += "axis:" + axisName(axis) + " ";
    for (const EdgePal& dist : distances)
        s += dist.debugDump(DebugLevel::brief, b) + " ";
    if (s.size())
        s.pop_back();
    return s;
}

void SectRay::dumpSet(const char*& str) {
    if (!OpDebugOptional(str, "ray count:"))
        return;
    size_t readDistances = OpDebugReadSizeT(str);
    normal = OpDebugReadNamedFloat(str, "normal");
    homeCept = OpDebugReadNamedFloat(str, "homeCept");
    homeT = OpDebugReadNamedFloat(str, "homeT");
    axis = axisStr(str, "axis:", Axis::neither);
    distances.resize(readDistances);
    for (size_t index = 0; index < readDistances; ++index)
        distances[index].dumpSet(str);
}

ENUM_NAME_STRUCT(PtType);
#define PTTYPE_NAME(w) { PtType::w, #w }

static PtTypeName ptTypeNames[] = {
    PTTYPE_NAME(noMatch),
	PTTYPE_NAME(original),
	PTTYPE_NAME(isAlias),
	PTTYPE_NAME(mapSegment)
};

ENUM_NAME(PtType, ptType)

std::string SegPt::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "pt:" + pt.debugDump(l, b) + " ";
    s += "ptType:" + ptTypeName(ptType);
    return s;
}

std::string CcCurves::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l || DebugLevel::detailed == l) {
        if (c.size()) {
            s = "c:" + STR(c.size()) + " [";
            for (auto edge : c)
                s += STR(edge->id) + " ";
            s.pop_back();
            s += "]\n";
        }
        if (runs.size()) {
            s += "runs:" + STR(runs.size()) + "\n";
            for (auto& run : runs)
                s += run.debugDump(l, b) + "\n";
        }
        if ('\n' == s.back())
            s.pop_back();
        return s;
    }
    // set up edge::debugDump to only show curvecurve relevant fields
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::startT, EF::endT,
			EF::isUnsplitable,
            EF::ccEnd, EF::ccLarge, EF::ccOverlaps, EF::ccSmall, EF::ccStart,
            EF::hulls, 
            EF::debugSetMaker, EF::debugParentID };
    OpSaveEF saveEF(showFields);
    DebugLevel down1 = (DebugLevel) ((int) l - 1);
    for (auto& edge : c)
        s += edge->debugDump(down1, b) + "\n";
    if ('\n' == s.back())
        s.pop_back();
    return s;
}

void CcCurves::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "c")) {
        size_t count = OpDebugReadSizeT(str);
        c.resize(count);
        for (size_t index = 0; index < count; ++index)
            c[index] = (OpEdge*) OpDebugReadSizeT(str);
    }
    if (OpDebugOptional(str, "runs")) {
        size_t count = OpDebugReadSizeT(str);
        runs.resize(count);
        for (size_t index = 0; index < count; ++index)
            runs[index].dumpSet(str);
    }
}

void CcCurves::dumpResolveAll(OpContours* contours) {
    for (auto& edge : c)
        contours->dumpResolve(edge);
    for (auto& run : runs)
        run.dumpResolveAll(contours);
}

std::string OpCurveCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    DebugLevel down1 = DebugLevel::file == l ? DebugLevel::file : (DebugLevel) ((int) l - 1);
    if (DebugLevel::file == l)
        s += "seg:" + STR(seg->id) + " ";
    else {
        const OpEdge* originalEdge = &seg->edges[0];
        s += "originalEdge:" + originalEdge->debugDump(down1, b) + "\n";
    }
    if (DebugLevel::file == l)
        s += "opp:" + STR(seg->id) + "\n";
    else {
        const OpEdge* originalOpp = &opp->edges[0];
        s += "originalOpp:" + originalOpp->debugDump(down1, b) + "\n";
    }
    if (DebugLevel::file == l) {
        s += "edgeCurves:" + edgeCurves.debugDump(l, b) + "\n";
        s += "oppCurves:" + oppCurves.debugDump(l, b) + "\n";
    } else {
        std::string names[] = { "edge curves", "opp curves" };
        int count = 0;
	    for (auto edgesPtrs : { &edgeCurves, &oppCurves } ) {
            const auto& edges = *edgesPtrs;
            if (edges.c.size()) {
                s += "-- " + names[count] + ":" + STR(edges.c.size()) + " --\n";
                s += edges.debugDump(l, b) + "\n";
            }
        }
        ++count;
    }
    if (limits.size()) {
        if (DebugLevel::file == l)
            s += "limits:" + STR(limits.size()) + "\n";
        else
            s += "-- limits:" + STR(limits.size()) + " --\n";
    }
    for (const auto& limit : limits) {
        s += limit.debugDump(down1, b) + "\n";
    }
    if (OpMath::IsFinite(snipEdge.t))
        s += "snipEdge:" + snipEdge.debugDump(down1, b) + "\n";
    if (OpMath::IsFinite(snipOpp.t))
        s += "snipOpp:" + snipOpp.debugDump(down1, b) + "\n";
    s += "matchRev:" + matchRev.debugDump(l, b) + " ";
    s += "depth:" + STR(depth) + " ";
    s += "uniqueLimits:" + STR(uniqueLimits_impl) + " ";
    if (addedPoint) 
        s += "addedPoint ";
    if (rotateFailed) 
        s += "rotateFailed ";
    if (sectResult) 
        s += "sectResult ";
    if (smallTFound) 
        s += "smallTFound ";
    if (largeTFound) 
        s += "largeTFound ";
    if (foundGap) 
        s += "foundGap ";
    if (splitMid) 
        s += "splitMid ";
#if OP_DEBUG_DUMP
    if (DebugLevel::file == l)
        s += "debugLocalCall:" + STR(debugLocalCall) + " ";
#endif
#if OP_DEBUG_VERBOSE
    if (DebugLevel::file == l) {
        if (dvDepthIndex.size()) {
            s += "dvDepthIndex[" + STR(dvDepthIndex.size()) + "\n";
            for (size_t iDepth : dvDepthIndex)
                s += STR(iDepth) + " ";
            s.pop_back();
            s += "] ";
        }
        if (dvAll.size()) {
            s += "dvAll[" + STR(dvAll.size()) + "\n";
            for (auto edge : dvAll)
                s += STR(edge->id) + " ";
            s.pop_back();
            s += "] ";
        }
    }
#endif
    if (' ' >= s[0])
        s.pop_back();
    return s;
}

void OpCurveCurve::dumpSet(const char*& str) {
    OpDebugRequired(str, "seg");
    seg = (OpSegment*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "opp");
    opp = (OpSegment*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "edgeCurves");
    edgeCurves.dumpSet(str);
    OpDebugRequired(str, "oppCurves");
    oppCurves.dumpSet(str);
    if (OpDebugOptional(str, "limits")) {
        size_t count = OpDebugReadSizeT(str);
        limits.resize(count);
        for (size_t index = 0; index < count; ++index)
            limits[index].dumpSet(str);
    }
    if (OpDebugOptional(str, "snipEdge"))
        snipEdge.dumpSet(str);
    if (OpDebugOptional(str, "snipOpp"))
        snipOpp.dumpSet(str);
    OpDebugRequired(str, "matchRev");
    matchRev.dumpSet(str);
    OpDebugRequired(str, "depth");
    depth = OpDebugReadSizeT(str);
    uniqueLimits_impl = OpDebugReadNamedInt(str, "uniqueLimits");
    addedPoint = OpDebugOptional(str, "addedPoint");
    rotateFailed = OpDebugOptional(str, "rotateFailed");
    sectResult = OpDebugOptional(str, "sectResult");
    smallTFound = OpDebugOptional(str, "smallTFound");
    largeTFound = OpDebugOptional(str, "largeTFound");
    foundGap = OpDebugOptional(str, "foundGap");
    splitMid = OpDebugOptional(str, "splitMid");
#if OP_DEBUG_DUMP
    OpDebugRequired(str, "debugLocalCall");
    debugLocalCall = OpDebugReadSizeT(str);
#endif
#if OP_DEBUG_VERBOSE
    if (OpDebugOptional(str, "dvDepthIndex")) {
        size_t count = OpDebugReadSizeT(str);
        dvDepthIndex.resize(count);
        for (size_t index = 0; index < count; ++index)
            dvDepthIndex[index] = OpDebugReadSizeT(str);
    }
    if (OpDebugOptional(str, "dvAll")) {
        size_t count = OpDebugReadSizeT(str);
        dvAll.resize(count);
        for (size_t index = 0; index < count; ++index)
            dvAll[index] = (OpEdge*) OpDebugReadSizeT(str);
    }
#endif
}

void OpCurveCurve::dumpResolveAll(OpContours* c) {
    c->dumpResolve(seg);
    c->dumpResolve(opp);
    edgeCurves.dumpResolveAll(c);
    oppCurves.dumpResolveAll(c);
    for (auto& limit : limits)
        limit.dumpResolveAll(c);
#if OP_DEBUG_VERBOSE
    for (OpEdge*& edge : dvAll)
        c->dumpResolve(edge);
#endif
}

#if OP_DEBUG_VERBOSE
void OpCurveCurve::dumpDepth(int level) {
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::startT, EF::endT, 
			EF::isUnsplitable,
            EF::ccEnd, EF::ccLarge, EF::ccOverlaps, EF::ccSmall, EF::ccStart,
            EF::hulls, EF::debugParentID, EF::debugSetMaker };
    OpSaveEF saveEF(showFields);
    OpDebugOut("depth:" + STR(level) + "\n");
    size_t dvLevels = dvDepthIndex.size();
    if ((int) dvLevels <= level) {
        for (const auto e : edgeCurves.c)
            dp(e);
        for (const auto e : oppCurves.c)
            dp(e);
        return;
    }
    size_t lo = dvDepthIndex[level];
    size_t hi = (int) dvDepthIndex.size() <= level + 1 ? dvAll.size() : dvDepthIndex[level + 1];
    for (size_t index = lo; index < hi; ++index) {
        OpEdge* e = dvAll[index];
        dp(e);
    }
}

void dmpDepth(int level) {
    OpCurveCurve* cc = debugGlobalContours->debugCurveCurve;
    if (!cc)
        return OpDebugOut("!debugGlobalContours->debugCurveCurve\n");
    cc->dumpDepth(level);
}

void OpCurveCurve::dumpDepth() {
    for (size_t level = 0; level <= dvDepthIndex.size(); ++level) {
        dumpDepth(level);
    }
}

void dmpDepth() {
    OpCurveCurve* cc = debugGlobalContours->debugCurveCurve;
    if (!cc)
        return OpDebugOut("!debugGlobalContours->debugCurveCurve\n");
    cc->dumpDepth();
}
#endif

ENUM_NAME_STRUCT(MatchEnds);
#define MATCH_ENDS_NAME(r) { MatchEnds::r, #r }

MatchEndsName matchEndsNames[] {
	MATCH_ENDS_NAME(none),
    MATCH_ENDS_NAME(start),
    MATCH_ENDS_NAME(end),
    MATCH_ENDS_NAME(both)
};

ENUM_NAME(MatchEnds, matchEnds)

std::string OpIntersection::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "[" + debugDumpID() + "] ";
    if (DebugLevel::brief == l) {
        s += "{" + ptT.debugDump(l, b) + ", ";
        s += "seg:" + segment->debugDumpID() + "\n";
        return s;
    }
    s += ptT.debugDump(id ? l : DebugLevel::error, b);   // !!! may be uninitialized?
#if OP_DEBUG
    if (debugErased)
        s += " erased";
#endif
    std::string segmentID = segment ? segment->debugDumpID() : "-";
    const OpSegment* oppParent = opp ? opp->segment : nullptr;
    std::string oppID = opp ? opp->debugDumpID() : "-";
    std::string oppParentID = oppParent ? oppParent->debugDumpID() : "-";
    s += " segment:" + segmentID;
    s += " opp/sect:" + oppParentID + "/" + oppID;
    if (coincidenceID  OP_DEBUG_CODE(|| debugCoincidenceID)) {
        s += " coinID:" + STR(coincidenceID)  OP_DEBUG_CODE(+ "/" + STR(debugCoincidenceID));
        s += DebugLevel::file == l ? " coinEnd:" : " " ;
        s += matchEndsName(coinEnd);
    }
    if (unsectID) {
        s += " unsectID:" + STR(unsectID);
        s += DebugLevel::file == l ? " unsectEnd:" : " ";
        s += matchEndsName(unsectEnd);
    }
    if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && !unsectID 
            && MatchEnds::none != coinEnd)
        s += "!!! (unexpected) " + matchEndsName(coinEnd);
    if (mergeProcessed)
        s += " mergeProcessed";
    if (moved)
        s += " moved";
    if (collapsed)
        s += " collapsed";
#if OP_DEBUG_MAKER
    s += " " + debugSetMaker.debugDump();
#endif
#if OP_DEBUG
    auto edgeOrSegment = [l](int debug_id, std::string label) {
        std::string result = label + " ";
        if (DebugLevel::file != l) {
            if (::findEdge(debug_id))
                result += "(edge) ";
            else if (::findSegment(debug_id))
                result += "(segment) ";
            else
                result += "(edge/seg:" + STR(debug_id) + " not found) ";
        }
        result += STR(debug_id);
        return result;
    };
    if (debugSrcID)
        s += " " + edgeOrSegment(debugSrcID, "debugSrcID:");
    if (debugOppID)
        s += " " + edgeOrSegment(debugOppID, "debugOppID:");
#endif
    return s;
}

void OpIntersection::dumpSet(const char*& str) {
    id = OpDebugReadSizeT(str);
    ptT.dumpSet(str);
#if OP_DEBUG
    debugErased = OpDebugOptional(str, "erased");
#endif
    OpDebugRequired(str, "segment");
    segment = (OpSegment*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "opp/sect");
    const OpSegment* oppParent = (const OpSegment*) OpDebugReadSizeT(str);  // discarded
    OP_ASSERT(oppParent || true);
    opp = (OpIntersection*) OpDebugReadSizeT(str);
    auto readCoinID = [](const char*& str) {
        bool isNegative = OpDebugOptional(str, "-");
        if (OpDebugOptional(str, "-"))
            return 0;
        size_t coinID = OpDebugReadSizeT(str);
        if ('/' == str[-1])
            --str;
        return isNegative ? -(int) coinID : (int) coinID;
    };
    coincidenceID = OpDebugOptional(str, "coinID") ? readCoinID(str) : 0;
#if OP_DEBUG
    debugCoincidenceID = OpDebugOptional(str, "/") ? readCoinID(str) : 0;
#endif
    coinEnd = matchEndsStr(str, "coinEnd", MatchEnds::none);
    unsectID = OpDebugOptional(str, "unsectID") ? readCoinID(str) : 0;
    unsectEnd = matchEndsStr(str, "unsectEnd", MatchEnds::none);
    mergeProcessed = OpDebugOptional(str, "mergeProcessed");
    moved = OpDebugOptional(str, "moved");
    collapsed = OpDebugOptional(str, "collapsed");
#if OP_DEBUG_MAKER
    debugSetMaker.dumpSet(str);
#endif
#if OP_DEBUG
    debugSrcID = OpDebugOptional(str, "debugSrcID") ? OpDebugReadSizeT(str) : 0;
    debugOppID = OpDebugOptional(str, "debugOppID") ? OpDebugReadSizeT(str) : 0;
#endif
}

void OpIntersection::dumpResolveAll(OpContours* c) {
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

#if OP_DEBUG
DEBUG_DUMP_ID_DEFINITION(OpIntersection, id)
#endif

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
        s.pop_back();
        s += "] ";
    } else {
        for (OpIntersection* sect : i)
            s += sect->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
}

void OpIntersections::dumpSet(const char*& str) {
    unsorted = OpDebugOptional(str, "unsorted");
    if (!OpDebugOptional(str, "intersections"))
        return;
    int sectCount = OpDebugReadSizeT(str);
    i.resize(sectCount);
    OpDebugRequired(str, "[");
    for (int index = 0; index < sectCount; index++) {
        i[index] = (OpIntersection*) OpDebugReadSizeT(str);
    }
    OpDebugRequired(str, "]");
}

size_t OpSectStorage::debugCount() const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return 0;
#endif
    size_t result = used;
    OpSectStorage* block = next;
    while (next) {
        result += block->used;
        block = block->next;
    }
    return result;
}

OpIntersection* OpSectStorage::debugFind(int ID) const {
	for (size_t index = 0; index < used; index++) {
		const OpIntersection& test = storage[index];
        if (test.id == ID)
            return const_cast<OpIntersection*>(&test);
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

OpIntersection* OpSectStorage::debugIndex(size_t index) const {
#if OP_I_KNOW_WHAT_IM_DOING
    if (!this)
        return nullptr;
#endif
    const OpSectStorage* block = this;
    while (index > block->used) {
        index -= block->used;
        block = block->next;
        if (!block)
            return nullptr;
    }
    if (block->used <= index)
        return nullptr;
    return const_cast<OpIntersection*>(&block->storage[index]);
}

std::string OpSectStorage::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    size_t count = debugCount();
    if (!count)
        return s;
    s += "sectStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (size_t index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
        for (size_t index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
}

void OpSectStorage::DumpSet(const char*& str, OpContours* dumpContours) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpIntersection* sect = dumpContours->allocateIntersection();
        sect->dumpSet(str);
    }
}

void OpSectStorage::dumpResolveAll(OpContours* c) {
    size_t count = debugCount();
    for (size_t index = 0; index < count; ++index) {
        debugIndex(index)->dumpResolveAll(c);
    }
}

OpSegment::OpSegment() 
    : winding(WindingUninitialized::dummy) {
}

std::string OpSegment::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::brief != l) {
        std::string s = "[" + STR(id) + "] ";
        if (contour)
            s += "contour[" + STR(contour->id) + "] ";
        s += c.debugDump(l, b) + "\n";
        s += "ptBounds:" + ptBounds.debugDump(l, b) + "\n";
        s += "closeBounds:" + closeBounds.debugDump(l, b) + "\n";
        if (sects.i.size()) {
            s += "sects:" + STR(sects.i.size()) + " [";
            for (auto sect : sects.i)
                s += STR(sect->id) + " ";
            s.pop_back();
            s += "]\n";
        }
        if (edges.size()) {
            s += "edges:" + STR(edges.size());
            if (DebugLevel::normal == l) {
                s += " [";
                for (auto& edge : edges)
                    s += STR(edge.id) + " ";
                s.pop_back();
                s += "]\n";
            } else {
                s += "\n";
                for (auto& edge : edges)
                    s += edge.debugDump(l, b) + "\n";
            }
        }
        s += "winding:" + winding.debugDump(l, b) + " ";
        if (disabled)
            s += "disabled ";
        if (willDisable)
            s += "willDisable ";
        if (hasCoin)
            s += "hasCoin ";
        if (hasUnsectable)
            s += "hasUnsectable ";
        if (startMoved)
            s += "startMoved ";
        if (endMoved)
            s += "endMoved ";
#if OP_DEBUG_IMAGE
        if (debugColor != black)
            s += debugDumpColor(debugColor) + " ";
#endif
#if OP_DEBUG_MAKER
        if (debugSetDisabled.line)
            s += "debugSetDisabled:" + debugSetDisabled.debugDump() + " ";
#endif
        s.pop_back();
        return s;
    }
    return "seg:" + STR(id) + " " + c.debugDump(l, b);
}

void OpSegment::dumpSet(const char*& str) {
    id = OpDebugReadSizeT(str);
    int contourID = OpDebugOptional(str, "contour[") ? OpDebugReadSizeT(str) : 0;
    OpDebugExitOnFail("mismatched contour id", contourID == contour->id);
    c.contours = contour->contours;
    c.dumpSet(str);
    OpDebugRequired(str, "ptBounds");
    ptBounds.dumpSet(str);
    OpDebugRequired(str, "closeBounds");
    closeBounds.dumpSet(str);
    if (OpDebugOptional(str, "sects:")) {
        int sectCount = OpDebugReadSizeT(str);
        sects.i.resize(sectCount);
        for (int index = 0; index < sectCount; ++index) {
            sects.i[index] = (OpIntersection*) OpDebugReadSizeT(str);
        }
    }
    if (OpDebugOptional(str, "edges:")) {
        int edgeCount = OpDebugReadSizeT(str);
        edges.resize(edgeCount);
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpContours = contour->contours;
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpSet(str);
    }
    OpDebugRequired(str, "winding");
    winding.dumpSet(str, contour->contours);
    disabled = OpDebugOptional(str, "disabled");
    willDisable = OpDebugOptional(str, "willDisable");
    hasCoin = OpDebugOptional(str, "hasCoin");
    startMoved = OpDebugOptional(str, "startMoved");
    endMoved = OpDebugOptional(str, "endMoved");
#if OP_DEBUG_IMAGE
	if (OpDebugOptional(str, "debugColor"))
		debugColor = OpDebugReadSizeT(str);
#endif
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
#endif
}

void OpSegment::dumpResolveAll(OpContours* contours) {
    contours->dumpResolve(contour);
    for (auto& sect : sects.i)
        contours->dumpResolve(sect);
    for (auto& edge : edges)
        edge.dumpResolveAll(contours);
}

std::string OpSegment::debugDumpEdges() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDump(defaultLevel, defaultBase) + "\n";
    if (s.size())
        s.pop_back();
    return s;
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
    s += "edges:\n";
    s += debugDumpEdges();
    return s;
}

std::string OpSegment::debugDumpIntersections() const {
    std::string s;
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
    OpDebugOut("seg:" + seg.debugDumpID() + " edges:" + STR(seg.edges.size())
            + " intersections:" + STR(seg.sects.i.size()) + "\n");
}

void dmpEnd(const OpSegment& seg) {
    dmpMatch(seg.c.lastPt());
}

void dmpFull(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpFull() + "\n"); 
}

void dmpSegmentEdges(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpEdges() + "\n");
}

void dmpSegmentIntersections(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpIntersections() + "\n");
}

void dmpSegmentSects(const OpSegment& seg) {
    dmpSegmentIntersections(seg);
}

void dmpStart(const OpSegment& seg) {
    dmpMatch(seg.c.firstPt());
}

DEBUG_DUMP_ID_DEFINITION(OpSegment, id)

ENUM_NAME_STRUCT(WindingType);
#define WINDING_NAME(w) { WindingType::w, #w }

static WindingTypeName windingTypeNames[] = {
    WINDING_NAME(uninitialized),  // note: this is -1
	WINDING_NAME(temp),
	WINDING_NAME(winding),
	WINDING_NAME(sum)
};

ENUM_NAME(WindingType, windingType)

std::string OpWinding::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "contour:";
    // !!! incomplete; if debug level is file or detailed, dump raw hex for winding data
    if (!contour)
        s += "0";
    else {
        s += STR(contour->id) + " ";
        if (DebugLevel::file != l) {
            s +="w.data:";
            if (w.data && w.size)
                s += contour->callBacks.debugDumpContourOutFuncPtr(w);
            else
                s += OpDebugStr(w.data) + " w.size:" + STR(w.size);
        } else {
            s += "w.size:" + STR(w.size) + " [";
            for (size_t index = 0; index < w.size; ++index)
                s += OpDebugByteToHex(((uint8_t*) w.data)[index]) + " ";
            if (' ' == s.back())
                s.pop_back();
            s += "]";
        }
    }
    return s;
}

void OpWinding::dumpSet(const char*& str, OpContours* dumpContours) {
    OpDebugRequired(str, "contour");
    int contourID = OpDebugReadSizeT(str);
    for (auto c : dumpContours->contours) {
        if (c->id == contourID) {
            OP_ASSERT(!contour);
            contour = c;
        }
    }
    if (!contourID)
        return;
    OP_ASSERT(contour);
    OP_ASSERT(contourID == contour->id);
    OpDebugRequired(str, "w.size");
    w.size = OpDebugReadSizeT(str);
    w.data = contour->contours->allocateWinding(w.size);
    OpDebugRequired(str, "[");
    for (size_t index = 0; index < w.size; ++index)
        ((uint8_t*) w.data)[index] = OpDebugByteToInt(str);
}

void OpWinding::dumpSet(const char*& ) {
    OP_ASSERT(0);  // call version with dump contours instead
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
            OpDebugOut("!!! linkup " + STR(linkupIndex) + " [" + STR(linkup->id) + "]"
                    " with unexpected tail: count " + STR(count) 
                    + "!= priorCount " + STR(priorCount) + "\n");
        if (looped)
            priorCount = 0;
        if (s.size() && s.back() != '\n')
            s += "\n";
        s += "[" + STR(linkupIndex) + "] linkup count:" + STR(count + priorCount);
        if (priorCount && !looped)
            s += " (prior count:" + STR(priorCount) + ")";
        if (looped)
            s += " loop";
        s += " bounds:" + linkup->linkBounds.debugDump(li, b) + "\n";
        if (!looped) {
            if (1 == count + priorCount && !linkup->lastEdge)
                s += "p/n/l:-/-/- ";
            else if (priorCount) {
                s += " prior:\n" + prior->debugDump(li, b);
                prior = linkup;
                while ((prior = prior->priorEdge) && count--)
                    s += STR(prior->id) + " ";
                s += "\n next:\n";
            }
        }
        next = linkup->nextEdge;
        s += " " + linkup->debugDump(li, b);
        bool hasNext = false;
        while (next) {
            if (looped == next)
                break;
            if (next == linkup->lastEdge) {
                OP_ASSERT(!next->nextEdge);
                s += "\n " + next->debugDump(li, b);
                hasNext = false;
            } else {
                if (!hasNext)
                    s += "\n      ";
                s += STR(next->id) + " ";
                hasNext = true;
            }
            next = next->nextEdge;
        }
        if (hasNext)
            s += "(loop) \n";
    }
    return s;
}

ENUM_NAME_STRUCT(ChopUnsortable);
#define CHOP_NAME(w) { ChopUnsortable::w, #w }

static ChopUnsortableName chopUnsortableNames[] = {
    CHOP_NAME(none),
	CHOP_NAME(prior),
	CHOP_NAME(next),
};

ENUM_NAME(ChopUnsortable, chopUnsortable)

std::string FoundEdge::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l)
         s += STR(edge->id) + " ";
    else
         s += edge->debugDump(l, b) + "\n";
    bool dumpDetails = DebugLevel::detailed == l || DebugLevel::file == l;
    if (dumpDetails && !distSq)
        s += "distSq:" + STR(distSq) + " ";
    if (dumpDetails && index >= 0)
        s += "index:" + STR(index) + " ";
    if (whichEnd != EdgeMatch::none)
        s += "whichEnd:" + edgeMatchName(whichEnd) + " ";
    if (dumpDetails && connects)
        s += "connects ";
    if (dumpDetails && loops)
        s += "loops ";
    if (dumpDetails && ChopUnsortable::none != chop)
        s += "chopUnsortable:" + chopUnsortableName(chop) + " ";
    return s;
}

void FoundEdge::dumpSet(const char*& str) {
    edge = (OpEdge*) OpDebugReadSizeT(str);
    distSq = OpDebugOptional(str, "distSq") ? OpDebugHexToFloat(str) : 0;
    index = OpDebugOptional(str, "index") ? OpDebugReadSizeT(str) : 0;
    whichEnd = edgeMatchStr(str, "whichEnd", EdgeMatch::none);
    connects = OpDebugOptional(str, "connects");
    loops = OpDebugOptional(str, "loops");
    chop = chopUnsortableStr(str, "chopUnsortable", ChopUnsortable::none);
}

void FoundEdge::dumpResolveAll(OpContours* c) {
    c->dumpResolve(edge);
}

std::string HullSect::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (opp)
        s += "[" + STR(opp->id) + "] ";
    s += sectTypeName(type, l);
    s += ":" + sect.debugDump(l, b);
    return s;
}

void HullSect::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "["))
        opp = (OpEdge*) OpDebugReadSizeT(str);
    else
        opp = nullptr;
    type = sectTypeStr(str, "", SectType::none);
    OpDebugRequired(str, ":");
    sect.dumpSet(str);
}

void HullSect::dumpResolveAll(OpContours* c) {
    if (opp)
        c->dumpResolve(opp);
}

std::string OpHulls::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    for (auto& hull : h)
        s += hull.debugDump(l, b) + "\n";
    return s;
}

std::string OpVector::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !isFinite())
        return "";
    return "{" + debugFloat(b, dx) + ", " + debugFloat(b, dy) + "}";
}

void OpVector::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    dx = OpDebugHexToFloat(str);
    OpDebugRequired(str, ", ");
    dy = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
    OpDebugOptional(str, ",");
}

std::string OpPoint::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !isFinite())
        return "";
    return "{" + debugFloat(b, x) + ", " + debugFloat(b, y) + "}";
}

void OpPoint::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    x = OpDebugHexToFloat(str);
    y = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
    OpDebugOptional(str, ",");
}

std::string OpPointBounds::debugDump(DebugLevel l, DebugBase b) const {
    return OpRect::debugDump(l, b);
}

void OpPointBounds::dumpSet(const char*& str) {
    OpRect::dumpSet(str);
}

std::string OpPtT::debugDump(DebugLevel l, DebugBase b) const {
    if (DebugLevel::error != l && !pt.isFinite() && !OpMath::IsFinite(t))
        return "";
    return pt.debugDump(DebugLevel::error, b) 
            + debugValue(DebugLevel::error, b, "t", t);
}

void OpPtT::dumpSet(const char*& str) {
    pt.dumpSet(str);
    t = OpDebugReadNamedFloat(str, "t");
}

std::string OpRootPts::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += " raw[" + raw.debugDump(l, b) + "]";
    s += " valid[" + valid.debugDump(l, b) + "]";
    s += " count:" + STR(count);
    for (size_t index = 0; index < count; ++index)
        s += ptTs[index].debugDump(l, b) + ", ";
    s.pop_back();
    if (',' == s.back()) s.pop_back();
    return s;
}

std::string OpRect::debugDump(DebugLevel l, DebugBase b) const {
    return "{" + debugFloat(b, left) + ", " + debugFloat(b, top) + ", "
        + debugFloat(b, right) + ", " + debugFloat(b, bottom) + "}";
}

void OpRect::dumpSet(const char*& str) {
    OpDebugRequired(str, "{");
    left = OpDebugHexToFloat(str);
    top = OpDebugHexToFloat(str);
    right = OpDebugHexToFloat(str);
    bottom = OpDebugHexToFloat(str);
    OpDebugRequired(str, "}");
}

std::string MatchReverse::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "match:" + matchEndsName(match) + " ";
    if (reversed)
        s += "reversed ";
    s.pop_back();
    return s;
}

void MatchReverse::dumpSet(const char*& str) {
    match = matchEndsStr(str, "match", MatchEnds::none);
    reversed = OpDebugOptional(str, "reversed");
}

std::string OpRoots::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "count:" + STR(count) + " ";
    for (size_t index = 0; index < count; ++index)
        s += debugFloat(b, roots[index]) + ", ";
    s.pop_back();
    if (',' == s.back()) s.pop_back();
    return s;
}

#if OP_DEBUG_MAKER
std::string OpDebugMaker::debugDump() const {
	return ("" == file ? std::string("(no file)") : file.substr(file.find("Op"))) + ":" + STR(line);
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
    line = OpDebugReadSizeT(str);
}
#endif

std::string LinePts::debugDump(DebugLevel l, DebugBase b) const {
    return "0:" + pts[0].debugDump(l, b) + ", 1:" + pts[1].debugDump(l, b);
}

void LinePts::dumpSet(const char*& str) {
    OpDebugRequired(str, "0:");
    pts[0].dumpSet(str);
    OpDebugRequired(str, "1:");
    pts[1].dumpSet(str);
}

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

std::string debugDumpColor(uint32_t c) {
    auto result = std::find_if(debugColorArray.begin(), debugColorArray.end(), [c](auto color) {
        return color.first == c; });
    char asHex[11];
    int written = snprintf(asHex, sizeof(asHex), "0x%08x", c);
    if (written != 10)
         return "snprintf of " + STR_E(c) + " to hex failed (written:" + STR(written) + ")";
    if (debugColorArray.end() == result)
        return "color " + std::to_string(c) + " (" + std::string(asHex) + ") not found";
    return std::string(asHex) + " " + (*result).second;
}

void dmpColor(uint32_t c) {
    OpDebugOut(debugDumpColor(c) + "\n");
}

void dmpColor(const OpEdge* edge) {
    dmpColor(edge->debugColor);
}

void dmpColor(const OpEdge& edge) {
    dmpColor(edge.debugColor);
}

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
