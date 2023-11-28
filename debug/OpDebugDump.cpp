// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_DUMP
#include "OpCurveCurve.h"
#include "OpContour.h"
#include "OpDebugColor.h"
#include "OpDebugDump.h"
#include "OpEdge.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

extern std::vector<std::pair<uint32_t, std::string>> debugColorArray;
int lineWidth;

#ifdef _WIN32
#pragma optimize( "", off )
#endif

#define OP_X(Thing) \
	void dmp(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmp(thing); \
	} \
	void dmpDetail(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmpDetail(thing); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dmp(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmp(thing); \
	} \
	void dmpDetail(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			dmpDetail(thing); \
	}
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void dmp(const std::vector<Thing>* things) { \
        dmp(*things); \
    } \
	void dmpDetail(const std::vector<Thing>* things) { \
        dmpDetail(*things); \
    }
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
    void Thing::dump() const { \
        dmp(*this); \
    } \
    void Thing::dumpDetail() const { \
        dmpDetail(*this); \
    } \
    void dmp(const Thing* thing) { \
        dmp(*thing); \
    } \
    void dmpDetail(const Thing* thing) { \
        dmpDetail(*thing); \
    }
    VECTOR_STRUCTS
    OP_STRUCTS
#undef OP_X
#define OP_X(Thing) \
    void Thing::dumpHex() const { \
        dmpHex(*this); \
    } \
    void dmpHex(const Thing* thing) { \
        dmpHex(*thing); \
    }
    DUMP_HEX
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
    }
EDGE_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X
#define OP_X(Thing) \
    void dmp##Thing(const struct OpSegment* segment) { \
        dmp##Thing(*segment); \
    } \
    void dmp##Thing(const OpContour* c) { \
        dmp##Thing(*c); \
    } \
    void dmp##Thing(const OpContour& c) { \
        OpDebugOut(c.debugDump() + "\n"); \
        for (auto& segment : c.segments) \
            segment.dump##Thing(); \
    } \
    void OpContour::dump##Thing() const { \
        dmp##Thing(*this); \
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
            return dmpMatch(*intersection); \
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
        if (std::vector<const OpEdge*> uSects = findEdgeUnsectable(ID); uSects.size()) { \
            for (auto uSect : uSects) { \
                uSect->Method(); \
                found = true; \
            } \
        } \
        if (std::vector<const OpEdge*> uSects = findEdgeOutput(ID); uSects.size()) { \
            for (auto uSect : uSects) { \
                uSect->Method(); \
                found = true; \
            } \
        } \
        if (const OpIntersection* intersection = findIntersection(ID)) { \
            intersection->Method(); \
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
        if (std::vector<const OpEdge*> oEdges = findEdgeOutput(ID); oEdges.size()) { \
            for (auto oEdge : oEdges) { \
                oEdge->Method(); \
                found = true; \
            } \
        } \
        if (!found) \
            OpDebugOut("ID: " + STR(ID) + " not found\n"); \
    }
    DUMP_BY_DUMPID
#undef OP_X

#define OP_STRUCTS_2 \
OP_X(LinkUps) \
OP_X(OpContours) \
OP_X(OpCurve) \
OP_X(OpCurveCurve) \
OP_X(OpJoiner) \
OP_X(OpOutPath) \
OP_X(OpPtT) \
OP_X(OpPoint) \
OP_X(OpPointBounds) \
OP_X(OpRect) \
OP_X(OpTightBounds) \
OP_X(OpVector) \
OP_X(OpWinder)

#define OP_X(Thing) \
     void dmpDetail(const Thing& thing) { \
         dmp(thing); \
     }
     OP_STRUCTS_2
#undef OP_X

#define OWNER OpContours
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpCurveCurve
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpEdge
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpIntersection
#undef DUMP_UNIQUE
#define DUMP_UNIQUE
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpJoiner
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpSegment
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpSegments
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpWinder
#include "OpDebugDefinitions.h"
#undef OWNER

#define ENUM_NAME_STRUCT(enum) \
struct enum##Name { \
    enum element; \
    const char* name; \
}

#define ENUM_NAME(Enum, enum) \
static bool enum##OutOfDate = false; \
\
std::string enum##Name(Enum element) { \
    static bool enum##Checked = false; \
    if (!enum##Checked) { \
        for (unsigned index = 0; index < ARRAY_COUNT(enum##Names); ++index) \
           if (!enum##OutOfDate && (unsigned) enum##Names[index].element != index) { \
               OpDebugOut("!!! enum##Names out of date\n"); \
               enum##OutOfDate = true; \
               break; \
           } \
        enum##Checked = true; \
    } \
    if (enum##OutOfDate) \
        return STR_E(element); \
    return enum##Names[(int) element].name; \
}

void OpDebugFormat(std::string s) {
    if (!s.size())
        return;
    if (!lineWidth || lineWidth >= s.size())
        return OpDebugOut(s + "\n");
    const char* start = &s.front();
    const char* end = &s.back();
    while (start + lineWidth <= end) {
        const char* c = start + lineWidth - 1;
        while (' ' != *c && c > start)
            --c;
        std::string line = s.substr(start - &s.front(), c == start ? lineWidth : c - start);
        OpDebugOut(line + "\n");
        start += line.size();
    }
    if (start <= end)
        OpDebugOut(s.substr(start - &s.front()) + "\n");
}

void dmpWidth(int width) {
    lineWidth = width;
}

void dmpActive() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isActive())
                    edge.dump();
            }
        }
    }
}

void dmpCoincidences() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
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
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            s += seg.debugDump() + "\n";
            for (const auto& edge : seg.edges) {
                s += edge.debugDump() + "\n";
            }
        }
    }
    OpDebugOut(s);
}

void dmpDisabled() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.disabled)
                    edge.dump();
            }
        }
    }
}

void dmpInOutput() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.inOutput)
                    edge.dumpDetail();
            }
        }
    }

}

void dmpIntersections() {
    std::string s;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            s += "intersect " + seg.debugDump() + "\n";
            for (const auto sect : seg.sects.i) {
                s += "  " + sect->debugDumpDetail(true) + "\n";
            }
        }
    }
    OpDebugOut(s);
}

void dmpJoin() {
    dmpActive();
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (!edge.isActive() && edge.unsectableID && !edge.inOutput && !edge.inLinkups)
                    edge.dumpDetail();
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
        for (const auto& seg : c.segments) {
            seg.dump();
        }
    }
}

void dmpUnsectable() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.unsectableID)
                    edge.dumpDetail();
            }
        }
    }
}

void dmpUnsortable() {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.unsortable)
                    edge.dumpDetail();
            }
        }
    }
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

std::string OpContours::debugDumpHex(std::string label) const {
    std::string s;
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            if (seg.sects.i.size())
                s += "OpDebugSectHex " + label + "Sects" + STR(seg.id) + "[] {\n";
            for (const auto intersection : seg.sects.i) {
                s += "// " + intersection->debugDumpBrief() + "\n";
                s += intersection->debugDumpHex() + ",\n";
            }
            if (seg.sects.i.size())
                s += "};\n\n";
            if (seg.edges.size())
                s += "OpDebugEdgeHex " + label + "Edges" + STR(seg.id) + "[] {\n";
            for (const auto& edge : seg.edges) {
                s += "// " + edge.debugDumpBrief() + "\n";
                s += edge.debugDumpHex() + ",\n";
            }
            if (seg.edges.size())
                s += "};\n\n";
        }
    }
    return s;
}

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

void dmp(const OpContours& c) {
    c.dumpSegments();
}

void dmpContours() {
    dmp(*debugGlobalContours);
}

ENUM_NAME_STRUCT(SectReason);

#define SECT_REASON_NAME(r) { SectReason::r, #r }

SectReasonName sectReasonNames[] {
    SECT_REASON_NAME(coinPtsMatch),
    SECT_REASON_NAME(curveCurveUnsectable),
    SECT_REASON_NAME(degenerateCenter),
    SECT_REASON_NAME(divideAndConquer_oneT),
    SECT_REASON_NAME(divideAndConquer_noEdgeToSplit),
    SECT_REASON_NAME(divideAndConquer_noOppToSplit),
    SECT_REASON_NAME(endPt),
    SECT_REASON_NAME(inflection),
    SECT_REASON_NAME(lineCurve),
    SECT_REASON_NAME(missingCoincidence),
    SECT_REASON_NAME(resolveCoin_windingChange),
    SECT_REASON_NAME(resolveCoin_oWindingChange),
    SECT_REASON_NAME(sharedEdge),
    SECT_REASON_NAME(sharedEnd),
    SECT_REASON_NAME(sharedStart),
    SECT_REASON_NAME(startPt),
    SECT_REASON_NAME(xExtrema),
    SECT_REASON_NAME(yExtrema),
    // testing only
    SECT_REASON_NAME(test),
};

ENUM_NAME(SectReason, sectReason)

void dmpMatch(const OpPoint& pt, bool detail) {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            if (pt == seg.c.pts[0] || pt == seg.c.lastPt()) {
                std::string str = "seg: " 
                        + (detail ? seg.debugDumpDetail() : seg.debugDump());
#if OP_DEBUG
                if (pt == seg.c.pts[0] && seg.debugStart != SectReason::startPt)
                    str += "; start is " + sectReasonName(seg.debugStart);
                if (pt == seg.c.lastPt() && seg.debugEnd != SectReason::endPt)
                    str += "; end is " + sectReasonName(seg.debugEnd);
#endif
                OpDebugOut(str + "\n");
            }
            for (const auto sect : seg.sects.i) {
                if (sect->ptT.pt == pt) {
                    OpDebugOut("sect: ");
                    detail ? sect->dumpDetail() : sect->dump();
                }
            }
            for (const auto& edge : seg.edges) {
                if (edge.start.pt == pt) {
                    OpDebugOut("edge start: ");
                    detail ? edge.dumpDetail() : edge.dump();
                }
                if (edge.end.pt == pt) {
                    OpDebugOut("edge end: ");
                    detail ? edge.dumpDetail() : edge.dump();
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

void dmpMatchDetail(const OpPoint& pt) {
    dmpMatch(pt, true);
}

void dmpMatchDetail(const OpPtT& ptT) {
    dmpMatch(ptT.pt, true);
}

void dmpMatchDetail(const OpIntersection& i) {
    dmpMatch(i.ptT.pt, true);
}

std::vector<const OpIntersection*> findCoincidence(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
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
    for (const auto& c : debugGlobalContours->contours)
        if (ID == c.id)
            return &c;
#endif
    return nullptr;
}

const OpEdge* findEdge(int ID) {
    auto match = [ID](const OpEdge& edge) {
        return edge.id == ID || edge.unsectableID == ID ||
                edge.debugOutPath == ID || edge.debugRayMatch == ID;
    };
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (match(edge))
                    return &edge;
            }
        }
    }
    if (const OpEdge* filler = debugGlobalContours->fillerStorage
            ? debugGlobalContours->fillerStorage->debugFind(ID) : nullptr)
        return filler;
    // if edge intersect is active, search there too
    if (const OpEdge* ccEdge = debugGlobalContours->ccStorage
            ? debugGlobalContours->ccStorage->debugFind(ID) : nullptr)
        return ccEdge;
    return nullptr;
}

std::vector<const OpEdge*> findEdgeOutput(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (ID == abs(edge.debugOutPath))
                    result.push_back(&edge);
            }
        }
    }
    return result;
}

std::vector<const OpEdge*> findEdgeRayMatch(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.debugRayMatch)
                    result.push_back(&edge);
            }
        }
    }
    return result;
}

std::vector<const OpEdge*> findEdgeUnsectable(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (ID == abs(edge.unsectableID))
                    result.push_back(&edge);
            }
        }
    }
    return result;
}

const OpIntersection* findIntersection(int ID) {
#if OP_DEBUG
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == intersection->id)
                    return intersection;
            }
        }
    }
#endif
    return nullptr;
}

std::vector<const OpIntersection*> findSectUnsectable(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.sects.i) {
                if (ID == abs(intersection->unsectID))
                    result.push_back(intersection);
            }
        }
    }
    return result;
}

const OpSegment* findSegment(int ID) {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
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

void OpContours::debugCompare(std::string s) const {
    const char* str = s.c_str();
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
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

std::string OpContour::debugDump() const {
    std::string s = "contour: " OP_DEBUG_CODE(+ STR(id)) + std::string("\n");
    s += "bounds: " + ptBounds.debugDump() + " ";
    s += "operand: ";
    s += OpOperand::left == operand ? "left" : "right";
    return s;
}

void dmp(const OpContour& c) {
    OpDebugOut(c.debugDump() + "\n");
    for (auto& segment : c.segments)
        segment.dump();
}

void dmp(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dump();
}

void dmpDetail(const OpContour& c) {
    OpDebugOut(c.debugDump() + "\n");
    for (auto& segment : c.segments)
        segment.dumpDetail();
}

void dmpDetail(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dumpDetail();
}

void dmpHex(const OpContour& c) {
    OpDebugOut(c.debugDump() + "\n");
    for (auto& segment : c.segments)
        segment.dumpHex();
}

ENUM_NAME_STRUCT(OpType);
#define TYPE_NAME(r) { OpType::r, #r }

static OpTypeName opTypeNames[] {
    TYPE_NAME(no),
    TYPE_NAME(line),
    TYPE_NAME(quad),
    TYPE_NAME(conic),
    TYPE_NAME(cubic),
};

ENUM_NAME(OpType, opType)

std::string debugLabel(DebugLevel debugLevel, std::string label) {
    return DebugLevel::brief == debugLevel ? label.substr(0, 1) : label;
}

std::string debugValue(DebugLevel debugLevel, DebugBase debugBase, std::string label, float value) {
    std::string s;
    if (!OpMath::IsFinite(value))
        return s;
    s = debugLabel(debugLevel, label);
    if (DebugBase::hex == debugBase)
        s += OpDebugDumpHex(value) + " ";
    s += STR(value) + " ";
    return s;
};

std::string OpCurve::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    std::string s = "{ ";
    for (int i = 0; i < pointCount(); ++i) 
        s += pts[i].debugDump(debugLevel, debugBase) + ", ";
    s.pop_back(); s.pop_back();
    s += " }";
    if (OpType::conic == type)
        s += debugValue(debugLevel, debugBase, "weight", weight) + " ";
    if (centerPt)
        s += debugLabel(debugLevel, "center") 
                + pts[pointCount()].debugDump(debugLevel, debugBase) + " ";
    s += opTypeName(type);
    return s;
}

std::string OpCurve::debugDump() const {
    return debugDump(DebugLevel::normal, DebugBase::dec);
}

std::string OpCurve::debugDumpHex() const {
    return debugDump(DebugLevel::normal, DebugBase::hex);
}

void dmp(const OpCurve& curve) {
    return OpDebugOut(curve.debugDump()); 
}

void dmpHex(const OpCurve& curve) {
    return OpDebugOut(curve.debugDumpHex()); 
}

std::string OpEdge::debugDumpBrief() const {
    std::string s;
    s += "[" + STR(id) + "] ";
    s += "{" + start.debugDump() + ", ";
    s += end.debugDump() + ", ";
    s += "seg:" + STR(segment->id);
    return s;
}

ENUM_NAME_STRUCT(EdgeMatch);
#define MATCH_NAME(r) { EdgeMatch::r, #r }

static EdgeMatchName edgeMatchNames[] {
    MATCH_NAME(none),
    MATCH_NAME(start),
    MATCH_NAME(end),
    MATCH_NAME(both),
};

ENUM_NAME(EdgeMatch, edgeMatch)

ENUM_NAME_STRUCT(EdgeFail);
#define FAIL_NAME(r) { EdgeFail::r, #r }

static EdgeFailName edgeFailNames[] {
    FAIL_NAME(none),
    FAIL_NAME(center),
    FAIL_NAME(horizontal),
    FAIL_NAME(vertical),
};

ENUM_NAME(EdgeFail, edgeFail)

#if OP_DEBUG
ENUM_NAME_STRUCT(EdgeMaker);
#define EDGE_MAKER_NAME(r) { EdgeMaker::r, #r }

static EdgeMakerName edgeMakerNames[] {
    EDGE_MAKER_NAME(empty),
    EDGE_MAKER_NAME(filler),
    EDGE_MAKER_NAME(makeEdges),
    EDGE_MAKER_NAME(oppSect),
    EDGE_MAKER_NAME(segSect),
    EDGE_MAKER_NAME(split1),
    EDGE_MAKER_NAME(addTest),
    EDGE_MAKER_NAME(opTest),
};

ENUM_NAME(EdgeMaker, edgeMaker);
#endif

ENUM_NAME_STRUCT(WindZero);
#define WIND_ZERO_NAME(r) { WindZero::r, #r }

static WindZeroName windZeroNames[] {
    WIND_ZERO_NAME(noFlip),
    WIND_ZERO_NAME(normal),
    WIND_ZERO_NAME(opp),
};

ENUM_NAME(WindZero, windZero)

ENUM_NAME_STRUCT(Axis);
#define AXIS_NAME(r) { Axis::r, #r }

static AxisName axisNames[] {
    AXIS_NAME(neither),
    AXIS_NAME(vertical),
    AXIS_NAME(horizontal),
};

ENUM_NAME(Axis, axis)

ENUM_NAME_STRUCT(ZeroReason);

#define REASON_NAME(r) { ZeroReason::r, #r }

static ZeroReasonName zeroReasonNames[] {
    REASON_NAME(uninitialized),
    REASON_NAME(addedPalToOutput),
    REASON_NAME(addIntersection),
    REASON_NAME(applyOp),
    REASON_NAME(centerNaN),
    REASON_NAME(findCoincidences), 
    REASON_NAME(hvCoincidence1),
    REASON_NAME(hvCoincidence2),
    REASON_NAME(hvCoincidence3),
    REASON_NAME(hvCoincidence4),
    REASON_NAME(isPoint),
    REASON_NAME(noFlip),
    REASON_NAME(none),
    REASON_NAME(palWinding),
};

ENUM_NAME(ZeroReason, zeroReason)

ENUM_NAME_STRUCT(EdgeSplit);

#define SPLIT_NAME(s) { EdgeSplit::s, #s }

static EdgeSplitName edgeSplitNames[] {
    SPLIT_NAME(no),
    SPLIT_NAME(keep),
    SPLIT_NAME(defer),
    SPLIT_NAME(yes)
};

ENUM_NAME(EdgeSplit, edgeSplit)

#define EDGE_FILTER \
	OP_X(segment) \
	OP_X(ray) \
	OP_X(priorEdge) \
	OP_X(nextEdge) \
	OP_X(lastEdge) \
	OP_X(ctrlPts) \
	OP_X(start) \
	OP_X(center) \
	OP_X(end) \
	OP_X(curve_impl) \
	OP_X(vertical_impl) \
	OP_X(ptBounds) \
	OP_X(linkBounds) \
	OP_X(winding) \
	OP_X(sum) \
	OP_X(many) \
	OP_X(pals) \
	OP_X(lessRay) \
	OP_X(moreRay) \
	OP_X(weight) \
	OP_X(curvy) \
	OP_X(id) \
	OP_X(unsectableID) \
	OP_X(whichEnd) \
	OP_X(rayFail) \
	OP_X(windZero) \
	OP_X(doSplit) \
	OP_X(curveSet) \
	OP_X(curvySet) \
	OP_X(lineSet) \
	OP_X(palSet) \
	OP_X(verticalSet) \
	OP_X(isLine_impl) \
	OP_X(active_impl) \
	OP_X(inLinkups) \
	OP_X(inOutput) \
	OP_X(disabled) \
	OP_X(unsortable) \
	OP_X(between)

#define EDGE_VIRTUAL \
    OP_X(contour) \
    OP_X(controls)

#define EDGE_DEBUG \
	OP_X(Start) \
	OP_X(End) \
	OP_X(Match) \
	OP_X(ZeroErr) \
	OP_X(Maker) \
	OP_X(Zero) \
	OP_X(SetMaker) \
	OP_X(SetSum) \
	OP_X(OutPath) \
	OP_X(ParentID) \
	OP_X(RayMatch) \
	OP_X(Filler)

#define EDGE_IMAGE \
	OP_X(Color) \
	OP_X(Draw) \
	OP_X(Join)

enum class EdgeFilter {
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

std::vector<EdgeFilter> filter;
std::vector<EdgeFilter> always;

void addEdgeFilter(std::vector<EdgeFilter>& efSet, EdgeFilter ef) {
    if (efSet.end() != std::find(efSet.begin(), efSet.end(), ef))
        return;
    efSet.push_back(ef);
}

void addAlways(EdgeFilter ef) {
    addEdgeFilter(always, ef);
}

void addFilter(EdgeFilter ef) {
    addEdgeFilter(filter, ef);
}

void clearAlways() {
    always.clear();
}

void clearFilter() {
    filter.clear();
}

void dmpFilters() {
    auto dmpOne = [](std::vector<EdgeFilter>& efSet, std::string name) {
        std::string s = name + ": ";
        for (auto ef : efSet) {
            if ((size_t) ef < ARRAY_COUNT(filterNames))
                s += filterNames[(size_t) ef].name;
            else 
                s += "(out of range) [" + STR_E(ef) + "] ";
        }
        OpDebugFormat(s);
    };
    dmpOne(filter, "filter");
    dmpOne(always, "always");
}

std::string OpEdge::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    auto findFilter = [](const std::vector<EdgeFilter>& set, EdgeFilter match) {
        return set.end() != std::find(set.begin(), set.end(), match);
    };
    auto dumpAlways = [findFilter](EdgeFilter match) {
        return findFilter(always, match);
    };
    auto dumpIt = [findFilter, dumpAlways](EdgeFilter match) {
        return !findFilter(filter, match) || dumpAlways(match);
    };
    auto dumpEither = [dumpIt](EdgeFilter m1, EdgeFilter m2) {
        return dumpIt(m1) || dumpIt(m2);
    };
    auto strLabel = [debugLevel](std::string label) {
        return debugLabel(debugLevel, label);
    };
    auto strBracket = [strLabel](std::string label, int value) {
        return strLabel(label) + "[" + STR(value) + "] ";
    };
    auto strCurve = [debugBase, strLabel](std::string label, const OpCurve& curve) {
        return strLabel(label) + (DebugBase::dec == debugBase ? curve.debugDump()
                : curve.debugDumpHex()) + " ";
    };
    auto strEdge = [](OpEdge* edge) {
        return edge ? STR(edge->id) : "-";
    };
    auto strPt = [dumpAlways, debugBase, strLabel](EdgeFilter match, std::string label,
                OpPoint pt) {
        if (!dumpAlways(match) && !pt.isFinite())
            return std::string("");
        return strLabel(label) + pt.debugDump(DebugLevel::error, debugBase);
    };
    auto strPtT = [dumpAlways, debugBase, strLabel](EdgeFilter match, std::string label,
                OpPtT ptT) {
        if (!dumpAlways(match) && !ptT.pt.isFinite() && !OpMath::IsFinite(ptT.t))
            return std::string("");
        return strLabel(label) + ptT.debugDump(DebugLevel::error, debugBase);
    };
    auto strValue = [debugLevel, debugBase](std::string label, float value) {
        return debugValue(debugLevel, debugBase, label, value);
    };
    auto strBounds = [dumpAlways, debugBase, strLabel](EdgeFilter match, std::string label, 
            const OpPointBounds& bounds) {
        if (!dumpAlways(match) && !bounds.isSet())
            return std::string("");
        return strLabel(label) + (DebugBase::dec == debugBase ? bounds.debugDump()
                : bounds.debugDumpHex()) + " ";
    };
    auto strWinding = [dumpAlways, debugBase, strLabel](EdgeFilter match, std::string label,
             OpWinding winding) {
        std::string s;
        if (!dumpAlways(match)) {
            if (WindingType::temp == winding.debugType && !winding.left() && !winding.right())
                return s;
            if (WindingType::uninitialized == winding.debugType 
                    && OpMax == winding.left() && OpMax == winding.right())
                return s;
        }
        return strLabel(label) + ":" + STR(winding.left()) + "/" + STR(winding.right()) + " ";
    };
    std::string s;
    if (dumpIt(EdgeFilter::id)) s += strBracket("edge", id);
    if (segment && dumpIt(EdgeFilter::segment)) s += strBracket("segment", segment->id);
    if (segment && segment->contour && dumpIt(EdgeFilter::contour)) 
        s += strBracket("contour", segment->contour->id);
    if (priorEdge || nextEdge || lastEdge) {
        if (dumpIt(EdgeFilter::priorEdge)) s += strLabel("prior") + "/";
        if (dumpIt(EdgeFilter::nextEdge)) s += strLabel("next") + "/";
        if (lastEdge && dumpIt(EdgeFilter::lastEdge)) s += strLabel("last") + "/";
        if ('/' == s.back()) s.back() = ':';
        if (dumpIt(EdgeFilter::priorEdge)) s += strEdge(priorEdge) + "/";
        if (dumpIt(EdgeFilter::nextEdge)) s += strEdge(nextEdge) + "/";
        if (lastEdge && dumpIt(EdgeFilter::lastEdge)) s += strEdge(lastEdge) + "/";
        if ('/' == s.back()) s.back() = ' ';
    }
    if (ray.distances.size() && dumpIt(EdgeFilter::ray)) 
        s += ray.debugDump(debugLevel, debugBase) + " ";
    std::string p;
    if (dumpIt(EdgeFilter::start)) p += strPtT(EdgeFilter::start, "start", start) + ", ";
    if (dumpIt(EdgeFilter::controls)) {
        for (int i = 0; i < segment->c.pointCount() - 2; ++i)
            p += strPt(EdgeFilter::controls, STR(i + 1) + "ctrl", ctrlPts[i]) + ", ";
    }
    if (dumpIt(EdgeFilter::end)) p += strPtT(EdgeFilter::end, "end", end) + ", ";
    if (p.size())
        s += "{" + p.substr(0, p.size() - 2) + "} ";
    if (((OpMath::IsFinite(weight) && (weight != 1)) 
            || (segment && OpType::conic == segment->c.type)) && dumpIt(EdgeFilter::weight)) 
        s += strValue("weight", weight);
    if (dumpIt(EdgeFilter::center)) s += strPtT(EdgeFilter::center, "center", center) + " ";
    if (curveSet && (dumpEither(EdgeFilter::curve_impl, EdgeFilter::curveSet))) 
        s += strCurve("curve_impl", curve_impl);
    if (verticalSet && (dumpEither(EdgeFilter::vertical_impl, EdgeFilter::verticalSet))) 
        s += strCurve("vertical_impl", vertical_impl);
    if (dumpIt(EdgeFilter::ptBounds)) s += strBounds(EdgeFilter::ptBounds, "ptBounds", ptBounds);
    if (dumpIt(EdgeFilter::linkBounds)) s += strBounds(EdgeFilter::linkBounds, "linkBounds", linkBounds);
    if (dumpIt(EdgeFilter::winding)) s += strWinding(EdgeFilter::winding, "winding", winding);
    if (dumpIt(EdgeFilter::sum)) s += strWinding(EdgeFilter::sum, "sum", sum);
    if (dumpIt(EdgeFilter::many)) s += strWinding(EdgeFilter::many, "many", many);
    if (pals.size() && dumpIt(EdgeFilter::pals)) {
        s += strLabel("pals") + ":";
        for (auto& pal : pals)
            s += pal.debugDump(DebugLevel::brief, debugBase) + " ";
    }
    if (lessRay.size() && dumpIt(EdgeFilter::lessRay)) {
        s += strLabel("lessRay") + ":";
        for (auto less : lessRay)
            s += strEdge(less) + " ";
    }
    if (moreRay.size() && dumpIt(EdgeFilter::moreRay)) {
        s += strLabel("moreRay") + ":";
        for (auto more : moreRay)
            s += strEdge(more) + " ";
    }
    if (dumpEither(EdgeFilter::curvy, EdgeFilter::curvySet)
            && (dumpAlways(EdgeFilter::curvy) || curvySet))
        s += strValue("curvy", curvy);
    if (dumpIt(EdgeFilter::unsectableID)
            && (dumpAlways(EdgeFilter::unsectableID) || unsectableID))
        s += strBracket("unsectableID", unsectableID);
    if (dumpIt(EdgeFilter::whichEnd) 
            && (dumpAlways(EdgeFilter::whichEnd) || EdgeMatch::none != whichEnd))
        s += strLabel("which") + ":" + edgeMatchName(whichEnd) + " ";
    if (dumpIt(EdgeFilter::rayFail)
            && (dumpAlways(EdgeFilter::rayFail) || EdgeFail::none != rayFail))
        s += strLabel("rayFail") + ":" + edgeFailName(rayFail) + " ";
    if (dumpIt(EdgeFilter::windZero))
        s += strLabel("windZero") + ":" + windZeroName(windZero) + " ";
    if (dumpIt(EdgeFilter::doSplit)
            && (dumpAlways(EdgeFilter::doSplit) || EdgeSplit::no != doSplit))
        s += strLabel("doSplit") + ":" + edgeSplitName(doSplit) + " ";
    #define STR_BOOL(ef) if (dumpIt(EdgeFilter::ef) && (dumpAlways(EdgeFilter::ef) || ef)) { \
        s += strLabel(#ef) + " "; \
        if (1 != ((unsigned char) ef)) s += STR((size_t) ef) + " "; }
    STR_BOOL(curveSet);
    STR_BOOL(curvySet);
    STR_BOOL(lineSet);
	STR_BOOL(palSet);
    STR_BOOL(verticalSet);
    STR_BOOL(isLine_impl);
	STR_BOOL(active_impl);
    STR_BOOL(inLinkups);
    STR_BOOL(inOutput);
    STR_BOOL(disabled);
    STR_BOOL(unsortable);
    STR_BOOL(between);
    // !!! incomplete -- move the rest for debugDumpDetail to here
    return s;
}

void dp(const OpEdge* e) { 
    dp(*e); 
}

void dp(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::normal, DebugBase::dec));
}

void dp(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    dp(e);
}

void dp0x(const OpEdge* e) { 
    dp0x(*e); 
}

void dp0x(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::normal, DebugBase::hex));
}

void dp0x(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    dp0x(e);
}

void db(const OpEdge* e) { 
    db(*e); 
}

void db(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::brief, DebugBase::dec));
}

void db(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    db(e);
}

void db0x(const OpEdge* e) { 
    db0x(*e); 
}

void db0x(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::brief, DebugBase::hex));
}

void db0x(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    db0x(e);
}

void deets(const OpEdge* e) { 
    deets(*e); 
}
void deets(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::detailed, DebugBase::dec));
}

void deets(int id) {
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    deets(e);
}

void deets0x(const OpEdge* e) { 
    deets0x(*e); 
}

void deets0x(const OpEdge& e) { 
    OpDebugFormat(e.debugDump(DebugLevel::detailed, DebugBase::hex));
}

void deets0x(int id){
    const OpEdge* e = findEdge(id);
    if (!e)
        return OpDebugOut("id " + STR(id) + " not found");
    deets0x(e);
}

std::string OpEdge::debugDumpDetail() const {
   std::string s = "edge[" + STR(id) + "] segment[" + STR(segment->id) + "] contour["
            OP_DEBUG_CODE(+ STR(segment->contour->id)) + std::string("]\n");
    if (priorEdge || nextEdge || lastEdge) {
        s += "priorE/nextE/lastE:" + (priorEdge ? STR(priorEdge->id) : "-");
        s += "/" + (nextEdge ? STR(nextEdge->id) : "-");
        s += "/" + (lastEdge ? STR(lastEdge->id) : "-");
        s += " ";
    }
    if (priorEdge || nextEdge || lastEdge /* || priorSum_impl || loopStart || Axis::neither != sumAxis */ )
        s += "\n";
    if (ray.distances.size()) {
		const SectRay& rayRef = ray;
        s += ray.debugDump(DebugLevel::brief, DebugBase::dec);
    }
    s += "{" + start.debugDump() + ", ";
    for (int i = 0; i < segment->c.pointCount() - 2; ++i)
        s += ctrlPts[i].debugDump() + ", ";
    s += end.debugDump() + "} ";
    if (1 != weight)
        s += "w:" + STR(weight) + " ";
    s += "\ncenter:" + center.debugDump() + " ";
    s += "\n";
    if (ptBounds.isSet())
        s += "pb:" + ptBounds.debugDump() + " ";
    if (linkBounds.isSet())
        s += "lb:" + linkBounds.debugDump();
    if (ptBounds.isSet() || linkBounds.isSet())
        s += "\n";
    s += debugDumpWinding() + " ";
    if (pals.size()) {
        s += "pals: ";
        for (auto& pal : pals)
            s += STR(pal.edge->id) + " ";
    }
    if (EdgeMatch::none != whichEnd)
        s += "which:" + edgeMatchName(whichEnd) + " ";
    if (EdgeFail::none != rayFail)
        s += "rayFail:" + edgeFailName(rayFail) + " ";
    s += "windZero:" + windZeroName(windZero) + "\n";
    if (unsectableID) s += "unsectable:" + STR(unsectableID) + " ";
    if (EdgeSplit::no != doSplit)
        s += "doSplit:" + edgeSplitName(doSplit) + " ";
    if (curveSet) s += "curveSet ";
    if (lineSet) s += "lineSet ";
    if (verticalSet) s += "verticalSet ";
    if (isLine_impl) s += "isLine ";
    if (active_impl) s += "active ";
    if (inOutput) s += "inOutput ";
    if (inLinkups) s += "inLinkups ";
    if (disabled) s += "disabled ";
#if OP_DEBUG
    if (ZeroReason::uninitialized != debugZero)
        s += "reason:" + zeroReasonName(debugZero) + " ";
#endif
    if (unsortable) s += "unsortable ";
    if (between) s += "between ";
#if OP_DEBUG
    if (debugStart) s += "debugStart:" + STR(debugStart->id) + " ";
    if (debugEnd) s += "debugEnd:" + STR(debugEnd->id) + " ";
    s += "debugMaker:" + edgeMakerName(debugMaker) + " ";
    s += debugSetMaker.debugDump() + " ";
    if (debugParentID) s += "debugParentID:" + STR(debugParentID) + " ";
    if (debugRayMatch) s += "debugRayMatch:" + STR(debugRayMatch) + " ";
    if (debugSetSum.line)
        s += debugSetSum.debugDump() + " ";
    if (debugFiller)
        s += "filler ";
    if (debugBlack != debugColor)
        s += debugDumpColor(debugColor) + " ";
    if (debugDraw)
        s += "draw ";
    s += "\n";
#endif
    std::vector<OpIntersection*> startSects;
    std::vector<OpIntersection*> endSects;
    for (auto sect : segment->sects.i) {
        if (start == sect->ptT)
            startSects.push_back(sect);
        if (end == sect->ptT)
            endSects.push_back(sect);
    }
    if (!startSects.size())
        s += "!!! missing start intersection at t:" + STR(start.t);
    else {
        s += "start sects:";
        for (auto sect : startSects)
            s += OP_DEBUG_STR_ID(sect) + " ";
    }
    s += " ";
    if (!endSects.size())
        s += "!!! missing end intersection at t:" + STR(end.t);
    else {
        s += "end sects:";
        for (auto sect : endSects)
            s += OP_DEBUG_STR_ID(sect) + " ";
    }
    return s;
}

std::string OpEdge::debugDumpHex() const {
    std::string s = "[" + STR(id) + "]";
    s = "  {" + start.debugDumpHex() + " }, // " + start.debugDump() + "\n";
    for (int i = 0; i < segment->c.pointCount() - 2; ++i)
        s += "  {" + ctrlPts[i].debugDumpHex() + " }, // " + ctrlPts[i].debugDump() + "\n";
    s += "  {" + end.debugDumpHex() + " } // " + end.debugDump() + "\n";
    return s;
}

OpEdge::OpEdge(std::string s)
    : OpEdge() {
    const char* str = s.c_str();
    OpDebugSkip(str, "{");
    start = OpPtT(str);
    OpDebugSkip(str, "\n");
    end = OpPtT(str);
    OpDebugSkip(str, "/* seg:");
    char* endStr;
    long segmentID = strtol(str, &endStr, 10);
    str = endStr;
    OpDebugSkip(str, "}");
    segment = ::findSegment(segmentID);
//    OP_ASSERT(segment);
    // don't call complete because we don't want to advance debug id
}

OpEdge::OpEdge(OpHexPtT hexPtT[2])
    : OpEdge() {
    start = hexPtT[0];
    end = hexPtT[1];
}

OpEdge::OpEdge(OpPtT ptT[2])
    : OpEdge() {
    start = ptT[0];
    end = ptT[1];
}

void OpEdge::debugCompare(std::string s) const {
    OpEdge test(s);
    OP_ASSERT(segment->id == test.segment->id);
    OP_ASSERT(start == test.start);
    OP_ASSERT(end == test.end);
}

std::string OpEdge::debugDump() const {
    std::string s;
    if (priorEdge || nextEdge || lastEdge) {
        s += "p/n";
        if (lastEdge)
            s +=  "/l";
        s += ":" + (priorEdge ? STR(priorEdge->id) : "-");
        s += "/" + (nextEdge ? STR(nextEdge->id) : "-");
        if (lastEdge)
            s += "/" + STR(lastEdge->id);
        s += " ";
    }
    s += "[" + STR(id) + "] ";
    s += "{" + start.pt.debugDump() + ", ";
    for (int i = 0; i < segment->c.pointCount() - 2; ++i)
        s += ctrlPts[i].debugDump() + ", ";
    s += end.pt.debugDump() + "} ";
    if (1 != weight)
        s += "w:" + STR(weight) + " ";
    s += "t{" + STR(start.t) + ", " + STR(center.t) + ", " + STR(end.t) + "}";
#if 0   // show bounds in detail view only
    if (ptBounds.isSet())
        s += "    pb:" + ptBounds.debugDump() + " ";
    if (linkBounds.isSet())
        s += "    lb:" + linkBounds.debugDump() + " ";
#endif
    s += " wind:" + STR(winding.left()) + "/" + STR(winding.right()) + " ";
    if (OpMax != sum.left() || OpMax != sum.right()) {
        s += "l/r:" + (OpMax != sum.left() ? STR(sum.left()) : "--");
        s += "/" + (OpMax != sum.right() ? STR(sum.right()) : "--") + " ";
    }
    if (EdgeMatch::none != whichEnd)
        s += "which:" +edgeMatchName(whichEnd) + " ";
    if (EdgeFail::none != rayFail)
        s += "rayFail:" + edgeFailName(rayFail) + " ";
    if (EdgeSplit::no != doSplit)
        s += "doSplit:" + edgeSplitName(doSplit) + " ";
    if (isLine_impl) s += "isLine ";
    if (disabled) s += OP_DEBUG_CODE(debugFiller ? "filler " : ) "disabled ";
    if (unsectableID) s += "uID:" + STR(unsectableID) + " ";
    if (unsortable) s += "unsortable ";
    if (debugRayMatch) s += "debugRayMatch:" + STR(debugRayMatch) + " ";
    s += "seg:" + STR(segment->id);
    return s;
}

// keep this in sync with op edge : is loop
std::string OpEdge::debugDumpChain(WhichLoop which, bool detail) const {
    std::string s = "chain:";
    const OpEdge* looped = debugIsLoop(which, LeadingLoop::in);
    bool firstLoop = false;
    int safetyCount = 0;
    const OpEdge* chain = this;
    for (;;) {
        s += "\n" + (detail ? chain->debugDumpDetail() : chain->debugDump());
        if (chain == looped) {
            if (firstLoop)
                return s + " loop";
            firstLoop = true;
        }
        chain = WhichLoop::prior == which ? chain->priorEdge : chain->nextEdge;
		if (!chain)
			break;
        if (++safetyCount > 250) {
            OpDebugOut(std::string("!!! %s likely loops forever: ") + 
                    (WhichLoop::prior == which ? "prior " : "next "));
            break;
        }
    }
    return s;
}

std::string OpEdge::debugDumpWinding() const {
    std::string s;
    s += "winding: " + winding.debugDump();
    if (sum.isSet())
        s += "\nsum: " + sum.debugDump();
    return s;
}

void dmp(const OpEdge& edge) {
    std::string s = edge.debugDump() + "\n";
    OpDebugOut(s);
}

void OpEdge::dumpChain(bool detail) const {
    OpDebugOut("prior: " + debugDumpChain(WhichLoop::prior, detail) + "\n");
    OpDebugOut("next: " + debugDumpChain(WhichLoop::next, detail) + "\n");
}

void dmpDetail(const OpEdge& edge) { 
    std::string s = edge.debugDumpDetail() + "\n";
    OpDebugOut(s);
}

void dmpEnd(const OpEdge& edge)  {
    edge.contours()->dumpMatch(edge.end.pt);
}

void dmpFull(const OpEdge& edge) { 
    edge.segment->dumpFull(); 
}

void dmpFullDetail(const OpEdge& edge) { 
    edge.segment->dumpFullDetail(); 
}

void dmpFull(const OpEdge* edge) { 
    edge->segment->dumpFull(); 
}

void dmpFullDetail(const OpEdge* edge) { 
    edge->segment->dumpFullDetail(); 
}

void dmpHex(const OpEdge& edge) {
    OpDebugOut(edge.debugDumpHex() + "\n");
}

void dmpCenter(const OpEdge& edge) {
    edge.dumpCenter(false);
}

void dmpCenterHex(const OpEdge& edge) {
    edge.dumpCenter(true);
}

// don't just dump it, find the best theoretical one through binary search
void OpEdge::dumpCenter(bool asHex) const {
    std::string s = "[" + STR(id) + "] center:" + (asHex ? center.debugDumpHex() : center.debugDump());
    OpPoint c = { (ptBounds.left + ptBounds.right) / 2, (ptBounds.top + ptBounds.bottom) / 2 };
    s += " bounds center:" + (asHex ? c.debugDumpHex() : c.debugDump()) + "\n";
    float lo = start.t;
    float hi = end.t;
    OpPoint bestXPt, bestYPt;
    float bestXt = OpNaN;
    float bestYt = OpNaN;
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
                (XyChoice::inX == xy ? bestXPt : bestYPt) = midPt;
                (XyChoice::inX == xy ? bestXt : bestYt) = mid;
                break;
            }
            (inLo ? hi : lo) = mid;
        }
    }
    s += "bestX: {" + (asHex ? bestXPt.debugDumpHex() : bestXPt.debugDump()) + "} t:" 
            + (asHex ? OpDebugDumpHex(bestXt) : STR(bestXt));
    s += " bestY: {" + (asHex ? bestYPt.debugDumpHex() : bestYPt.debugDump()) + "} t:" 
            + (asHex ? OpDebugDumpHex(bestYt) : STR(bestYt));
    OpDebugOut(s + "\n");
}

void dmpLink(const OpEdge& edge) {
    edge.dumpChain();
}

void OpEdge::dumpLink() const {
    dmpLink(*this);
}

void dmpLinkDetail(const OpEdge& edge) {
    edge.dumpChain(true);
}

void OpEdge::dumpLinkDetail() const {
    dmpLinkDetail(*this);
}

void dmpStart(const OpEdge& edge) {
    edge.contours()->dumpMatch(edge.start.pt);
}

size_t OpEdgeStorage::debugCount() const {
    if (!this)
        return 0;
    size_t result = used;
    OpEdgeStorage* block = next;
    while (next) {
        result += block->used;
        block = block->next;
    }
    return result / sizeof(OpEdge);
}

const OpEdge* OpEdgeStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index += sizeof(OpEdge)) {
		const OpEdge& test = *(const OpEdge*) &storage[index];
        if (test.id == ID || test.unsectableID == ID ||
                test.debugOutPath == ID || test.debugRayMatch == ID)
            return &test;
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

const OpEdge* OpEdgeStorage::debugIndex(int index) const {
    if (!this)
        return nullptr;
    const OpEdgeStorage* block = this;
    int byteIndex = sizeof(OpEdge) * index;
    while (byteIndex > block->used) {
        byteIndex -= block->used;
        block = block->next;
        if (!block)
            return nullptr;
    }
    if (block->used <= byteIndex)
        return nullptr;
    return (const OpEdge*) &block->storage[byteIndex];
}

void dmp(const OpEdgeStorage& edges) {
	for (int index = 0; index < edges.used; index += sizeof(OpEdge)) {
		const OpEdge* test = (const OpEdge*) &edges.storage[index];
        test->dump();
	}
    if (edges.next)
	    dmp(*edges.next);
}

void dmpDetail(const OpEdgeStorage& edges) {
	for (int index = 0; index < edges.used; index += sizeof(OpEdge)) {
		const OpEdge* test = (const OpEdge*) &edges.storage[index];
        test->dumpDetail();
	}
    if (edges.next)
	    dmpDetail(*edges.next);
}

ENUM_NAME_STRUCT(LinkPass);
#define LINKPASS_NAME(r) { LinkPass::r, #r }

LinkPassName linkPassNames[] = {
    LINKPASS_NAME(none),
	LINKPASS_NAME(unambiguous),
	LINKPASS_NAME(unsectInX)
};

ENUM_NAME(LinkPass, linkPass)

void dmp(const OpJoiner& edges) {
    if (!edges.path.debugIsEmpty())
        edges.path.dump();
    if (edges.byArea.size()) {
        OpDebugOut("-- sorted in x --\n");
        for (auto edge : edges.byArea)
            if (edge->isActive())
                edge->dump();
    }
    if (edges.unsectByArea.size()) {
        OpDebugOut("-- unsectable sorted in x --\n");
        for (auto edge : edges.unsectByArea)
            if (edge->isActive())
                edge->dump();
    }
    if (edges.disabled.size()) {
        OpDebugOut("-- disabled --\n");
        dmp(edges.disabled);
    }
    if (edges.unsortables.size()) {
        OpDebugOut("-- unsortables --\n");
        for (auto edge : edges.unsortables)
            if (edge->isActive())
                edge->dump();
    }
    if (edges.linkups.l.size()) {
        OpDebugOut("-- linkups --\n");
        dmp(edges.linkups);
    }
    OpDebugOut("linkMatch:" + edgeMatchName(edges.linkMatch) + " linkPass:" 
            + linkPassName(edges.linkPass) + " disabledBuilt:" + STR(edges.disabledBuilt) + "\n");
}

void OpWinder::dumpAxis(Axis a) const {
    std::string s = "";
    for (const auto edge : Axis::vertical == a ? inY : inX) {
        s += edge->debugDump() + "\n";
    }
    OpDebugOut(s);
}

void dmp(const OpWinder& edges) {
    if (edges.inX.size()) {
        OpDebugOut("-- sorted in x --\n");
        edges.dumpAxis(Axis::horizontal);
    }
    if (edges.inY.size()) {
        OpDebugOut("-- sorted in y --\n");
        edges.dumpAxis(Axis::vertical);
    }
}

std::string EdgeDistance::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    if (DebugLevel::brief == debugLevel)
        return STR(edge->id) + " ";
    std::string s = edge->debugDump(debugLevel, debugBase) + " ";
    s += debugValue(debugLevel, debugBase, "cept", cept);
    s += debugValue(debugLevel, debugBase, "t", t);
    if (reversed) s += debugLabel(debugLevel, "reversed");
    if (skipPal) s += debugLabel(debugLevel, "skipPal");
    if (skipSum) s += debugLabel(debugLevel, "skipSum");
    return s;
}

void dmpDetail(const EdgeDistance& distance) {
    OpDebugFormat(distance.debugDump(DebugLevel::detailed, DebugBase::dec));
}

void dmp(const EdgeDistance& distance) {
    OpDebugFormat(distance.debugDump(DebugLevel::normal, DebugBase::dec));
}

void dmpHex(const EdgeDistance& distance) {
    OpDebugFormat(distance.debugDump(DebugLevel::normal, DebugBase::hex));
}

ENUM_NAME_STRUCT(CurveRef);

#define CURVEREF_NAME(s) { CurveRef::s, #s }

static CurveRefName curveRefNames[] {
    CURVEREF_NAME(edge),
    CURVEREF_NAME(opp),
};

ENUM_NAME(CurveRef, curveRef)

ENUM_NAME_STRUCT(CenterSet);

#define CENTERSET_NAME(s) { CenterSet::s, #s }

static CenterSetName centerSetNames[] {
    CENTERSET_NAME(splitNo),
    CENTERSET_NAME(splitKeep),
	CENTERSET_NAME(newEdge),
	CENTERSET_NAME(defer),
	CENTERSET_NAME(edgeCurvy),
	CENTERSET_NAME(oppCurvy),
	CENTERSET_NAME(edgeLineLine),
	CENTERSET_NAME(oppLineLine),
};

ENUM_NAME(CenterSet, centerSet)

std::string debugDump(const CcCenter& ccCenter, DebugLevel level) {
    std::string s = DebugLevel::detailed == level ? ccCenter.edge->debugDumpDetail()
            : ccCenter.edge->debugDump();
    s += " adjusted:" + ccCenter.center.debugDump();
	s += " which:" + curveRefName(ccCenter.which);
	s += " split:" + edgeSplitName(ccCenter.split);
	s += " centerSet:" + centerSetName(ccCenter.centerSet);
	s += " depth:" + STR(ccCenter.depth);
    s += "\n";
    return s;
}

void dmp(const CcCenter& ccCenter) {
    OpDebugOut(debugDump(ccCenter, DebugLevel::normal));
}

void dmpDetail(const CcCenter& ccCenter) {
    OpDebugOut(debugDump(ccCenter, DebugLevel::detailed));
}

std::string debugDump(const CoinPair& pair, DebugLevel level) {
    std::string s = DebugLevel::brief == level ? "start id:" + STR(pair.start->id) + " " : 
            DebugLevel::detailed == level ? "start:" + pair.start->debugDumpDetail() + "\n" :
            "start:" + pair.start->debugDump() + "\n";
    if (pair.end) s += "end:" + pair.end->debugDump() + "\n";
    if (pair.oStart) s += DebugLevel::brief == level ? "oStart id:" + STR(pair.oStart->id) + " " : 
            DebugLevel::detailed == level ? "oStart:" + pair.oStart->debugDumpDetail() + "\n" :
            "oStart:" + pair.oStart->debugDump() + "\n";
    if (pair.oEnd) s += "oEnd:" + pair.oEnd->debugDump() + "\n";
    if (pair.edge) s += "edge:" + pair.edge->debugDump() + "\n";
    if (pair.oppEdge) s += "oppEdge:" + pair.oppEdge->debugDump() + "\n";
    s += "id:" + STR(pair.id) + " ";
    if (pair.lastEdge) s += "lastEdge:" + pair.lastEdge->debugDump();
    s += "\n";
    return s;
}

std::string debugDumpHex(const CoinPair& pair, DebugLevel level) {
    std::string s = DebugLevel::brief == level ? "start id:" + STR(pair.start->id) + " " : 
            DebugLevel::detailed == level ? "start:" + pair.start->debugDumpDetail() + "\n" :
            "start:" + pair.start->debugDumpHex() + "\n";
    if (pair.end) s += "end:" + pair.end->debugDumpHex() + "\n";
    if (pair.oStart) s += DebugLevel::brief == level ? "oStart id:" + STR(pair.oStart->id) + " " : 
            DebugLevel::detailed == level ? "oStart:" + pair.oStart->debugDumpDetail() + "\n" :
            "oStart:" + pair.oStart->debugDumpHex() + "\n";
    if (pair.oEnd) s += "oEnd:" + pair.oEnd->debugDumpHex() + "\n";
    if (pair.edge) s += "edge:" + pair.edge->debugDumpHex() + "\n";
    if (pair.oppEdge) s += "oppEdge:" + pair.oppEdge->debugDumpHex() + "\n";
    s += "id:" + STR(pair.id) + " ";
    if (pair.lastEdge) s += "lastEdge:" + pair.lastEdge->debugDumpHex();
    s += "\n";
    return s;
}

void dmp(const CoinPair& pair) {
    OpDebugOut(debugDump(pair, DebugLevel::normal));
}

void dmpDetail(const CoinPair& pair) {
    OpDebugOut(debugDump(pair, DebugLevel::detailed));
}

void dmpHex(const CoinPair& pair) {
    OpDebugOut(debugDumpHex(pair, DebugLevel::normal));
}

std::string SectRay::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    std::string s = "ray count:" + STR(distances.size()) + " ";
    s += debugValue(debugLevel, debugBase, "normal", normal);
    s += debugValue(debugLevel, debugBase, "cept", homeCept);
    s += debugValue(debugLevel, debugBase, "t", homeT);
    s += "axis:" + axisName(axis) + " ";
    for (const EdgeDistance& dist : distances)
        s += dist.debugDump(debugLevel, debugBase);
    return s;

}

void dmpDetail(const SectRay& ray) {
    OpDebugFormat(ray.debugDump(DebugLevel::detailed, DebugBase::dec));
}

void dmp(const SectRay& ray) {
    OpDebugFormat(ray.debugDump(DebugLevel::normal, DebugBase::dec));
}

void dmpHex(const SectRay& ray) {
    OpDebugFormat(ray.debugDump(DebugLevel::normal, DebugBase::hex));
}

#if 0
static int debugSavedID = -1;

void OpCurveCurve::debugSaveID() {
    OP_ASSERT(-1 == debugSavedID);
    debugSavedID = debugGlobalContours->uniqueID;
    OP_ASSERT(-1 != debugSavedID);
}

// This is a half-baked idea. 
// The idea is to make debugging easier by minimizing the magnitude of edge IDs.
// But edges created by curve curve can't have their IDs reused this easily.
// Intersection creation is intermixed with the edge creation, so this would
// cause intersections and edges to have the same IDs, or for intersections to reuse IDs.
// At the moment, this doesn't seem worth the complexity required for a robust implmentation.
// It also hides the number of temporary edges created, which is a worthwhile performance metric.
void OpCurveCurve::debugRestoreID() {
    OP_ASSERT(-1 != debugSavedID);
    debugGlobalContours->uniqueID = debugSavedID;
    debugSavedID = -1;
}
#endif

void OpCurveCurve::dump(bool detail) const {
    std::string names[] = { "edge curves", "opp curves", "edge lines", "opp lines", "edge runs", "opp runs" };
    int count = 0;
	for (auto edgesPtrs : { &edgeCurves, &oppCurves, &edgeLines, &oppLines, &edgeRuns, &oppRuns } ) {
        const auto& edges = *edgesPtrs;
        if (edges.size()) {
            int splitCount = 0;
            for (auto& edge : edges)
                splitCount += EdgeSplit::yes == edge->doSplit;
            OpDebugOut("-- " + names[count]);
            if (count < 2)
                OpDebugOut("curves split: " + STR(splitCount));
            OpDebugOut(" --\n");
            detail ? dmp(edges) : dmpDetail(edges);
        }
        ++count;
    }
}

void dmp(const OpCurveCurve& cc) {
    cc.dump(false);
}

#if OP_DEBUG_VERBOSE
void dmpDepth(int level) {
    OpCurveCurve* cc = debugGlobalContours->debugCurveCurve;
    if (!cc)
        return;
    int dvLevels = cc->dvDepthIndex.size();
    if (dvLevels <= level) {
        for (const auto e : cc->edgeCurves)
            e->dump();
        for (const auto e : cc->oppCurves)
            e->dump();
        return;
    }
    int lo = (int) cc->dvDepthIndex[level];
    int hi = (int) cc->dvDepthIndex.size() <= level + 1 ? (int) cc->dvAll.size() 
            : cc->dvDepthIndex[level + 1];
    for (int index = lo; index < hi; ++index) {
        OpEdge* e = cc->dvAll[index];
        e->dump();
    }
}

void dmpDepth() {
    for (int level = 0; level <= (int) debugGlobalContours->debugCurveCurve->dvDepthIndex.size();
            ++level) {
        OpDebugOut("level:" + STR(level) + "\n");
        dmpDepth(level);
    }
}
#endif

#if OP_DEBUG
ENUM_NAME_STRUCT(IntersectMaker);
#define INTERSECT_MAKER_NAME(r) { IntersectMaker::r, #r }

IntersectMakerName intersectMakerNames[] {
	INTERSECT_MAKER_NAME(addCoincidentCheck),
	INTERSECT_MAKER_NAME(addCoincidentCheckOpp),
	INTERSECT_MAKER_NAME(addMatchingEnd),
	INTERSECT_MAKER_NAME(addMatchingEndOpp),
	INTERSECT_MAKER_NAME(addMatchingEndOStart),
	INTERSECT_MAKER_NAME(addMatchingEndOStartOpp),
	INTERSECT_MAKER_NAME(addMatchingStart),
	INTERSECT_MAKER_NAME(addMatchingStartOpp),
	INTERSECT_MAKER_NAME(addMatchingStartOEnd),
	INTERSECT_MAKER_NAME(addMatchingStartOEndOpp),
	INTERSECT_MAKER_NAME(addMix),
	INTERSECT_MAKER_NAME(addMixOpp),
	INTERSECT_MAKER_NAME(addPair_aPtT),
	INTERSECT_MAKER_NAME(addPair_bPtT),
	INTERSECT_MAKER_NAME(addPair_oppStart),
	INTERSECT_MAKER_NAME(addPair_oppEnd),
	INTERSECT_MAKER_NAME(curveCenter),
	INTERSECT_MAKER_NAME(curveCenterOpp),
	INTERSECT_MAKER_NAME(edgeIntersections),
	INTERSECT_MAKER_NAME(edgeIntersectionsOpp),
	INTERSECT_MAKER_NAME(edgeLineCurve),
	INTERSECT_MAKER_NAME(edgeLineCurveOpp),
	INTERSECT_MAKER_NAME(edgeT),
	INTERSECT_MAKER_NAME(edgeTOpp),
	INTERSECT_MAKER_NAME(oppT),
	INTERSECT_MAKER_NAME(oppTOpp),
	INTERSECT_MAKER_NAME(findIntersections_start),
	INTERSECT_MAKER_NAME(findIntersections_startOppReversed),
	INTERSECT_MAKER_NAME(findIntersections_startOpp),
	INTERSECT_MAKER_NAME(findIntersections_end),
	INTERSECT_MAKER_NAME(findIntersections_endOppReversed),
	INTERSECT_MAKER_NAME(findIntersections_endOpp),
	INTERSECT_MAKER_NAME(missingCoincidence),
	INTERSECT_MAKER_NAME(missingCoincidenceOpp),
	INTERSECT_MAKER_NAME(segEnd),
	INTERSECT_MAKER_NAME(segmentLineCurve),
	INTERSECT_MAKER_NAME(segmentLineCurveOpp),
	INTERSECT_MAKER_NAME(segStart),
	INTERSECT_MAKER_NAME(splitAtWinding),
	INTERSECT_MAKER_NAME(unsectableStart),
	INTERSECT_MAKER_NAME(unsectableEnd),
	INTERSECT_MAKER_NAME(unsectableOppStart),
	INTERSECT_MAKER_NAME(unsectableOppEnd),
	// testing only
	INTERSECT_MAKER_NAME(opTestEdgeZero1),
	INTERSECT_MAKER_NAME(opTestEdgeZero2),
	INTERSECT_MAKER_NAME(opTestEdgeZero3),
	INTERSECT_MAKER_NAME(opTestEdgeZero4),
};

ENUM_NAME(IntersectMaker, intersectMaker)
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

std::string OpIntersection::debugDump(bool fromDumpFull, bool fromDumpDetail) const {
    std::string s;
    std::string segmentID = segment ? segment->debugDumpID() : "--";
    const OpSegment* oppParent = opp ? opp->segment : nullptr;
    std::string oppID = opp ? opp->debugDumpID() : "--";
    std::string oppParentID = oppParent ? oppParent->debugDumpID() : "--";
    s = "[" + debugDumpID() + "] " + ptT.debugDump();
#if OP_DEBUG
    if (debugErased)
        s += " erased";
#endif
    if (!fromDumpFull || !segment)
        s += " segment:" + segmentID;
    s += " opp/sect:" + oppParentID + "/" + oppID;
    if (coincidenceID  OP_DEBUG_CODE(|| debugCoincidenceID)) {
        s += " coinID:" + STR(coincidenceID)  OP_DEBUG_CODE(+ "/" + STR(debugCoincidenceID));
        s += " " + matchEndsName(coinEnd);
    }
    if (unsectID) {
        s += " unsectID:" + STR(unsectID);
        s += " " + matchEndsName(unsectEnd);
    }
    if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && !unsectID 
            && MatchEnds::none != coinEnd)
        s += "!!! (unexpected) " +  matchEndsName(coinEnd);
    if (betweenID)
        s += " betweenID:" + STR(betweenID);
    if (coincidenceProcessed)
        s += " coincidenceProcessed";
#if OP_DEBUG
    s += " maker:" + intersectMakerName(debugMaker);
    if (fromDumpDetail)
        s += " " + debugSetMaker.debugDump();
    s += " reason:" + sectReasonName(debugReason);
#endif
    return s;
}

std::string OpIntersection::debugDumpBrief() const {
    std::string s;
    s += "[" + debugDumpID() + "] ";
    s += "{" + ptT.debugDump() + ", ";
    s += "seg:" + segment->debugDumpID() + "\n";

    return s;
}

std::string OpIntersection::debugDumpDetail(bool fromDumpIntersections) const {
#if OP_DEBUG
    auto edgeOrSegment = [](int debug_id, std::string label) {
        if (::findEdge(debug_id))
            return label + " (edge) " + STR(debug_id) + " ";
        if (::findSegment(debug_id))
            return label + " (segment) " + STR(debug_id) + " ";
        return STR(debug_id) + " not found ";
    };
#endif
    std::string s = debugDump(fromDumpIntersections, true);
#if OP_DEBUG
    if (debugID || debugOppID)
        s += "\n";
    if (debugID)
        s += edgeOrSegment(debugID, "debugID:");
    if (debugOppID)
        s += edgeOrSegment(debugOppID, "debugOpp:");
#endif
    return s;
}

std::string OpIntersection::debugDump() const {
    return debugDump(false, false);
}

std::string OpIntersection::debugDumpDetail() const {
    return debugDumpDetail(false);
}

std::string OpIntersection::debugDumpHex() const {
    std::string s;
    s = "/* sect:" + debugDumpID() + " */ OpPtT data[] {\n";
    s += "  " + ptT.debugDumpHex() + " // " + ptT.debugDump() + "\n";
    s += "}; // seg:" + segment->debugDumpID() + "\n";
    return s;
}

OpIntersection::OpIntersection(std::string s) {
    const char* str = s.c_str();
    OpDebugSkip(str, "{");
    ptT = OpPtT(str);
    OpDebugSkip(str, ", ");
    char* end;
    long segmentID = strtol(str, &end, 10);
    str = end;
    OpDebugSkip(str, "}");
    segment = const_cast<OpSegment*>(::findSegment(segmentID));
    OP_ASSERT(segment);
    // don't call complete because we don't want to advance debug id
    set(ptT, segment  
            OP_DEBUG_PARAMS(IntersectMaker::opTestEdgeZero1, 
            __LINE__, __FILE__, SectReason::test, 0, 0));
}

void OpIntersection::debugCompare(std::string s) const {
    OpIntersection test(s);
    OP_ASSERT(segment->id == test.segment->id);
    OP_ASSERT(ptT == test.ptT);
}

void dmp(const OpIntersection& sect) {
    OpDebugOut(sect.debugDump(false, false) + "\n");
}

void dmpDetail(const OpIntersection& sect) {
    OpDebugOut(sect.debugDumpDetail() + "\n");
}

void dmpHex(const OpIntersection& sect) { 
    OpDebugOut(sect.debugDumpHex() + "\n"); 
}

void dmpMatch(const OpIntersection& sect) {
    sect.segment->contour->contours->dumpMatch(sect.ptT.pt);
}

#if OP_DEBUG
DEBUG_DUMP_ID_DEFINITION(OpIntersection, id)
#endif

void dmp(const OpIntersections& sects) {
    dmp(sects.i);
}

void dmpDetail(const OpIntersections& sects) {
    dmpDetail(sects.i);
}

std::string OpSegment::debugDump() const {
    return "seg:" + STR(id) + " " + c.debugDump();
}

std::string OpSegment::debugDumpDetail() const {
    std::string s = debugDump() + "\n";
    s += " winding: " + winding.debugDump() + " ";
    if (disabled)
        s += "disabled ";
#if OP_DEBUG
    if (ZeroReason::uninitialized != debugZero)
        s += "reason:" + zeroReasonName(debugZero) + " ";
#endif
    if (recomputeBounds)
        s += "recomputeBounds ";
    if (sects.resort)
        s += "sects.resort ";
    s += "\n";
    s += " bounds:" + ptBounds.debugDump() + " ";
    OP_DEBUG_CODE(s += "contour:" + (contour ? STR(contour->id) : std::string("unset")) + "\n");
#if OP_DEBUG
    s += " start reason:" + sectReasonName(debugStart);
    s += " end reason:" + sectReasonName(debugEnd);
#endif
    return s;
}

// !!! probably shouldn't include trailing \n
std::string OpSegment::debugDumpEdges() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDump() + "\n";
    return s;
}

std::string OpSegment::debugDumpEdgesDetail() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDumpDetail() + "\n";
    return s;
}

std::string OpSegment::debugDumpFull() const {
    std::string s = debugDump() + "\n";
    s += debugDumpIntersections();
    s += "edges:\n";
    s += debugDumpEdges();
    return s;
}

std::string OpSegment::debugDumpFullDetail() const {
    std::string s = debugDumpDetail() + "\n";
    s += debugDumpIntersectionsDetail();
    s += debugDumpEdgesDetail();
    return s;
}

std::string OpSegment::debugDumpHex() const {
    return "/* seg:" + debugDumpID() + " */ " + c.debugDumpHex();
}

std::string OpSegment::debugDumpIntersections() const {
    std::string s;
    for (auto i : sects.i)
        s += i->debugDump(true, false) + "\n";
    return s;
}

std::string OpSegment::debugDumpIntersectionsDetail() const {
    std::string s;
    for (auto i : sects.i)
        s += i->debugDumpDetail(true) + "\n";
    return s;
}

void dmp(const OpSegment& seg) {
    OpDebugOut(seg.debugDump() + "\n");
}

void dmpCount(const OpSegment& seg) {
    OpDebugOut("seg:" + seg.debugDumpID() + " edges:" + STR(seg.edges.size())
            + " intersections:" + STR(seg.sects.i.size()) + "\n");
}

void dmpDetail(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpDetail() + "\n"); 
}

void dmpEnd(const OpSegment& seg) {
    seg.contour->contours->dumpMatch(seg.c.lastPt());
}

void dmpFull(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpFull() + "\n"); 
}

void dmpFullDetail(const OpSegment& seg) {
    OpDebugOut(seg.debugDumpFullDetail() + "\n"); 
}

void dmpHex(const OpSegment& seg) { 
    OpDebugOut(seg.debugDumpHex()); 
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
    seg.contour->contours->dumpMatch(seg.c.pts[0]);
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

std::string OpWinding::debugDump() const {
    std::string result = "type: ";
#if OP_DEBUG
        result += std::string(windingTypeNames[(int)debugType + 1].name);  // + 1: see above
#endif
    result += " left: " + (OpMax == left() ? std::string("unset") : STR(left()));
    result += " right: " + (OpMax == right() ? std::string("unset") : STR(right()));
    return result;
}

void OpWinding::dump() const {
    OpDebugOut(debugDump() + "\n");
}

void DumpLinkups(const std::vector<OpEdge*>& linkups) {
    for (const auto& linkup : linkups) {
        if (!linkup) {
           OpDebugOut("!!! expected non-null linkup\n");
           return;
        }
        int count = 0;
        auto next = linkup;
        auto looped = linkup->debugIsLoop(WhichLoop::prior, LeadingLoop::in);
        if (!looped)
            looped = linkup->debugIsLoop(WhichLoop::next, LeadingLoop::in);
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
        OP_ASSERT(!looped || count == priorCount);
        if (looped)
            priorCount = 0;
        std::string str = "linkup count:" + STR(count + priorCount);
        if (priorCount && !looped)
            str += " (prior count:" + STR(priorCount) + ")";
        if (looped)
            str += " loop";
        OpDebugOut(str + " bounds:" + linkup->linkBounds.debugDump() + "\n");
        if (!looped) {
            if (1 == count + priorCount && !linkup->lastEdge)
                OpDebugOut("p/n/l:-/-/- ");
            else if (priorCount) {
                prior->dump();
                str = "";
                while (prior != linkup) {
                    prior = prior->priorEdge;
                    if (!prior || (looped == prior && firstLoop))
                        break;
                    str += STR(prior->id) + " ";
                }
                if (str.length())
                    OpDebugOut(str + "\n");
                OpDebugOut("  chain:\n");
            }
        }
        next = linkup->nextEdge;
        linkup->dump();
        str = "";
        while (next) {
            if (looped == next)
                break;
            if (next == linkup->lastEdge) {
                OP_ASSERT(!next->nextEdge);
                if (str.length()) {
                    OpDebugOut(str + "\n");
                    str = "";
                }
                next->dump();
            } else
                str += STR(next->id) + " ";
            next = next->nextEdge;
        }
        if (str.length())
            OpDebugOut(str + "(loop) \n");
    }
}

void dmp(const LinkUps& linkups) {
    DumpLinkups(linkups.l);
}

void dmp(const FoundEdge& foundOne) {
    OpDebugOut(edgeMatchName(foundOne.whichEnd) + " " + foundOne.edge->debugDump() + "\n");
}

ENUM_NAME_STRUCT(ChopUnsortable);
#define CHOP_NAME(w) { ChopUnsortable::w, #w }

static ChopUnsortableName chopUnsortableNames[] = {
    CHOP_NAME(none),
	CHOP_NAME(prior),
	CHOP_NAME(next),
};

ENUM_NAME(ChopUnsortable, chopUnsortable)

void dmpDetail(const FoundEdge& foundOne) {
    std::string s = foundOne.edge->debugDumpDetail() + "\n";
    if (!foundOne.distSq)
        s += "distSq:" + STR(foundOne.distSq) + " ";
    if (foundOne.index >= 0)
        s += "index:" + STR(foundOne.index) + " ";
    if (foundOne.whichEnd != EdgeMatch::none)
        s += "whichEnd:" + edgeMatchName(foundOne.whichEnd) + " ";
    if (foundOne.connects)
        s += "connects ";
    if (foundOne.loops)
        s += "loops ";
    if (ChopUnsortable::none != foundOne.chop)
        s += "chopUnsortable:" + chopUnsortableName(foundOne.chop) + " ";
    OpDebugOut(s + "\n");
}

void dmpHex(float f) {
    OpDebugOut(OpDebugDumpHex(f) + "\n");
}

OpVector::OpVector(const char*& str) {
    OpDebugSkip(str, "{");
    dx = OpDebugHexToFloat(str);
    OpDebugSkip(str, ", ");
    dy = OpDebugHexToFloat(str);
    OpDebugSkip(str, "}");
}

std::string OpVector::debugDump() const {
    return "{" + STR(dx) + ", " + STR(dy) + "}";
}

std::string OpVector::debugDumpHex() const {
    return "{" + OpDebugDumpHex(dx) + ", " + OpDebugDumpHex(dy) + "}";
}

void dmp(const OpVector& pt) {
    OpDebugOut("OpVector v { " + pt.debugDump() + " };\n");
}

void dmpHex(const OpVector& pt) {
    OpDebugOut("OpVector v { " + pt.debugDumpHex() + " };  // " + pt.debugDump() + "\n");
}

OpPoint::OpPoint(const char*& str) {
    OpDebugSkip(str, "{");
    x = OpDebugHexToFloat(str);
    OpDebugSkip(str, ", ");
    y = OpDebugHexToFloat(str);
    OpDebugSkip(str, "}");
}

std::string OpPoint::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    std::string s;
    if (DebugLevel::error != debugLevel && !isFinite())
        return s;
    s = "{";
    if (DebugBase::hex == debugBase)
        s += OpDebugDumpHex(x) + " ";
    s += STR(x) + ", ";
    if (DebugBase::hex == debugBase)
        s += OpDebugDumpHex(y) + " ";
    s += STR(y) + "}";
    return s;
}

std::string OpPoint::debugDump() const {
    return debugDump(DebugLevel::normal, DebugBase::dec);
}

std::string OpPoint::debugDumpHex() const {
    return debugDump(DebugLevel::normal, DebugBase::hex);
}

void dmp(const OpPoint& pt) {
    OpDebugOut("OpPoint pt { " + pt.debugDump() + " };\n");
}

void dmpHex(const OpPoint& pt) {
    OpDebugOut("OpPoint pt { " + pt.debugDumpHex() + " };  // " + pt.debugDump() + "\n");
}

void dmp(const OpRect& r) {
    OpDebugOut(r.debugDump() + "\n");
}

void dmpHex(const OpRect& r) {
    OpDebugOut(r.debugDumpHex() + " // " + r.debugDump() + "\n");
}

void dmp(const OpPointBounds& pb) {
    dmp((const OpRect& ) pb);
}

void dmpHex(const OpPointBounds& pb) {
    dmpHex((const OpRect& ) pb);
}

void dmp(const OpTightBounds& tb) {
    std::string s = tb.debugDump();
    if (tb.debugXExtremaFailed)
        s += " debugXExtremaFailed";
    if (tb.debugYExtremaFailed)
        s += " debugYExtremaFailed";
    OpDebugOut(s + "\n");
}

void dmpHex(const OpTightBounds& tb) {
    dmpHex((const OpRect& ) tb);
}

OpPtT::OpPtT(const char*& str) {
    pt = OpPoint(str);
    OpDebugSkip(str, ", ");
    t = OpDebugHexToFloat(str);
}

std::string OpPtT::debugDump(DebugLevel debugLevel, DebugBase debugBase) const {
    std::string s;
    if (DebugLevel::error != debugLevel && !pt.isFinite() && !OpMath::IsFinite(t))
        return s;
    s = pt.debugDump(DebugLevel::error, debugBase);
    if (DebugBase::hex == debugBase)
        s += OpDebugDumpHex(t) + " ";
    s += STR(t);
    return s;
}

std::string OpPtT::debugDump() const {
    return debugDump(DebugLevel::normal, DebugBase::dec);
}

std::string OpPtT::debugDumpHex() const {
    return debugDump(DebugLevel::normal, DebugBase::hex);
}

void dmp(const OpPtT& ptT) {
    OpDebugOut(ptT.debugDump() + "\n"); 
}

void dmpHex(const OpPtT& ptT) {
    OpDebugOut(ptT.debugDumpHex() + " // " + ptT.debugDump() + "\n");
}

std::string OpRect::debugDump() const {
    return "{" + STR(left) + ", " + STR(top) + ", " + STR(right) + ", " + STR(bottom) + "}";
}

std::string OpRect::debugDumpHex() const {
    return "{" + OpDebugDumpHex(left) + ", " + OpDebugDumpHex(top) + ", "
        + OpDebugDumpHex(right) + ", " + OpDebugDumpHex(bottom) + "}";
}

void OpRoots::dump() const {
    std::string s = "OpRoots roots {\n"
        "{{ ";
    for (size_t index = 0; index < count; ++index) {
        s += STR(roots[index]);
        if (index < count - 1)
            s += ", ";
    }
    s += " }}, " + STR(count) + " };\n";
    OpDebugOut(s);
}


std::string OpDebugMaker::debugDump() const {
	return file.substr(file.find("Op")) + ":" + STR(line);
}

void dmpHex(const OpRoots& roots) {
    std::string s = "OpRoots roots { {{\n";
    for (size_t index = 0; index < roots.count; ++index) {
        s += "  " + OpDebugDumpHex(roots.roots[index]);
        if (index < roots.count - 1)
            s += ", ";
        s += "  // " + STR(roots.roots[index]) + "\n";
    }
    s += "}}, " + STR(roots.count) + " };\n";
    OpDebugOut(s);
}

void dmp(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDump() + "\n";
    }
    OpDebugOut(s);
}

void dmpDetail(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpDetail() + "\n";
    }
    OpDebugOut(s);
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

void dmpFullDetail(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpFullDetail() + "\n";
    }
    OpDebugOut(s);
}

void dmpHex(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDumpHex() + "\n";
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

#endif
