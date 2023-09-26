// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"
#include "OpDebugDump.h"

#if OP_DEBUG_DUMP
#include "OpContour.h"
#include "OpEdge.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

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

struct SectReasonName {
    SectReason reason;
    std::string name;
};

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

static bool reasonOutOfDate = false;

void checkReason() {
    static bool reasonChecked = false;
    if (!reasonChecked) {
        for (unsigned index = 0; index < ARRAY_COUNT(sectReasonNames); ++index)
           if (!reasonOutOfDate && (unsigned) sectReasonNames[index].reason != index) {
               OpDebugOut("!!! sectReasonNames out of date\n");
               reasonOutOfDate = true;
               break;
           }
        reasonChecked = true;
    }
}

void dmpMatch(const OpPoint& pt, bool detail) {
    checkReason();
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            if (pt == seg.c.pts[0] || pt == seg.c.lastPt()) {
                std::string str = "seg: " 
                        + (detail ? seg.debugDumpDetail() : seg.debugDump());
#if OP_DEBUG
                if (pt == seg.c.pts[0] && seg.debugStart != SectReason::startPt)
                    str += "; start is " + sectReasonNames[(int) seg.debugStart].name;
                if (pt == seg.c.lastPt() && seg.debugEnd != SectReason::endPt)
                    str += "; end is " + sectReasonNames[(int) seg.debugEnd].name;
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
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.id)
                    return &edge;
            }
        }
    }
    if (const OpEdge* filler = debugGlobalContours->edgeStorage
            ? debugGlobalContours->edgeStorage->debugFind(ID) : nullptr)
        return filler;
    // if edge intersect is active, search there too
    if (OpCurveCurve::debugActive) {
	    for (auto edgePtrs : { 
                &OpCurveCurve::debugActive->edgeCurves,
			    &OpCurveCurve::debugActive->oppCurves,
			    &OpCurveCurve::debugActive->edgeLines,
			    &OpCurveCurve::debugActive->oppLines,
                &OpCurveCurve::debugActive->edgeRuns,
                &OpCurveCurve::debugActive->oppRuns} ) {
            const auto& edges = *edgePtrs;
            for (const auto& edge : edges) {
                if (ID == edge.id)
                    return &edge;
            }
        }
   }
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

std::string OpCurve::debugDump() const {
    const char* names[] { "noType", "OpType::line", "OpType::quad", "OpType::conic", "OpType::cubic" };
    std::string s;
    s = "{ ";
    bool first = true;
    for (int i = 0; i < pointCount(); ++i) {
        if (!first)
            s += ", ";
        s += pts[i].debugDump();
        first = false;
    }
    s += " }";
    if (1 != weight)
        s += " w:" + OpDebugDump(weight);
    s += " " + (OpType::no <= type && type <= OpType::cubic ? names[(int) type] :
        "broken type [" + STR((int) type) + "]");
    return s;
}

std::string OpCurve::debugDumpHex() const {
    const char* names[] { "noType", "OpType::line", "OpType::quad", "OpType::conic", "OpType::cubic" };
    std::string s;
    s = "OpPoint data[] {\n";
    for (int i = 0; i < pointCount(); ++i) {
        if (i != pointCount() - 1)
            s += "  " + pts[i].debugDumpHex() + ", // " + pts[i].debugDump() + "\n";
        else
            s += "  " + pts[i].debugDumpHex() + "  // " + pts[i].debugDump() + "\n";
    }
    s += "};";
    if (1 != weight)
        s += "  // weight:" + OpDebugDumpHex(weight) + " // " + OpDebugDump(weight) + "\n";
    s += "  // type:" + (OpType::no <= type && type <= OpType::cubic ? names[(int) type] :
        "broken type [" + STR((int) type) + "]");
    return s;
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

struct DebugMatchName {
    EdgeMatch match;
    const char* name;
};

#define MATCH_NAME(r) { EdgeMatch::r, #r }

static DebugMatchName debugMatchNames[] {
    MATCH_NAME(none),
    MATCH_NAME(start),
    MATCH_NAME(end),
    MATCH_NAME(both),
};

std::string debugEdgeMatch(EdgeMatch match) {
    std::string result;
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugMatchNames); ++index) {
        if (!outOfDate && (unsigned)debugMatchNames[index].match != index) {
            OpDebugOut("debugMatchNames out of date\n");
            outOfDate = true;
        }
        if (match != debugMatchNames[index].match)
            continue;
        if (outOfDate)
            result += STR((int)match);
        else
            result += std::string(debugMatchNames[(int)match].name);
    }
    return result;
}

struct DebugFailName {
    EdgeFail fail;
    const char* name;
};

#define FAIL_NAME(r) { EdgeFail::r, #r }

static DebugFailName debugFailNames[] {
    FAIL_NAME(none),
    FAIL_NAME(center),
    FAIL_NAME(horizontal),
    FAIL_NAME(vertical),
};

std::string debugEdgeFail(EdgeFail fail) {
    std::string result;
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugFailNames); ++index) {
        if (!outOfDate && (unsigned)debugFailNames[index].fail != index) {
            OpDebugOut("debugFailNames out of date\n");
            outOfDate = true;
        }
        if (fail != debugFailNames[index].fail)
            continue;
        if (outOfDate)
            result += STR((int)fail);
        else
            result += std::string(debugFailNames[(int)fail].name);
    }
    return result;
}

#if OP_DEBUG
struct EdgeMakerName {
    EdgeMaker maker;
    const char* name;
};

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

std::string debugEdgeDebugMaker(EdgeMaker maker) {
    std::string result;
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(edgeMakerNames); ++index) {
        if (!outOfDate && (unsigned)edgeMakerNames[index].maker != index) {
            OpDebugOut("edgeMakerNames out of date\n");
            outOfDate = true;
        }
        if (maker != edgeMakerNames[index].maker)
            continue;
        if (outOfDate)
            result += STR((int) maker);
        else
            result += std::string(edgeMakerNames[(int) maker].name);
    }
    return result;
}
#endif

struct WindZeroName {
    WindZero wz;
    const char* name;
};

#define WIND_ZERO_NAME(r) { WindZero::r, #r }

static WindZeroName windZeroNames[] {
    WIND_ZERO_NAME(noFlip),
    WIND_ZERO_NAME(normal),
    WIND_ZERO_NAME(opp),
};

std::string debugEdgeWindZero(WindZero wz) {
    std::string result;
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(windZeroNames); ++index) {
        if (!outOfDate && (unsigned)windZeroNames[index].wz != index) {
            OpDebugOut("windZeroNames out of date\n");
            outOfDate = true;
        }
        if (wz != windZeroNames[index].wz)
            continue;
        if (outOfDate)
            result += STR((int) wz);
        else
            result += std::string(windZeroNames[(int) wz].name);
    }
    return result;
}

struct AxisName {
    Axis axis;
    const char* name;
};

#define AXIS_NAME(r) { Axis::r, #r }

static AxisName axisNames[] {
    AXIS_NAME(neither),
    AXIS_NAME(vertical),
    AXIS_NAME(horizontal),
};

std::string debugAxisName(Axis axis) {
    std::string result;
    bool outOfDate = false;
    for (signed index = 0; index < (signed) ARRAY_COUNT(axisNames); ++index) {
        if (!outOfDate && ((signed) axisNames[index].axis + 1) != index) {
            OpDebugOut("axisNames out of date\n");
            outOfDate = true;
        }
        if (axis != axisNames[index].axis)
            continue;
        if (outOfDate)
            result += STR((int) axis);
        else
            result += std::string(axisNames[index].name);
    }
    return result;
}

struct DebugReasonName {
    ZeroReason reason;
    const char* name;
};

#define REASON_NAME(r) { ZeroReason::r, #r }

static DebugReasonName debugReasonNames[] {
    REASON_NAME(uninitialized),
    REASON_NAME(addedPalToOutput),
    REASON_NAME(addIntersection),
    REASON_NAME(applyOp),
    REASON_NAME(centerNaN),
    REASON_NAME(findCoincidences),
    REASON_NAME(hvCoincidence1),
    REASON_NAME(hvCoincidence2),
    REASON_NAME(hvCoincidence3),
    REASON_NAME(isPoint),
    REASON_NAME(noFlip),
    REASON_NAME(none)
};

std::string OpEdge::debugDumpDetail() const {
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugReasonNames); ++index)
       if (!outOfDate && (unsigned) debugReasonNames[index].reason != index) {
           OpDebugOut("debugReasonNames out of date\n");
           outOfDate = true;
       }
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
        s += ::debugDump(rayRef, DebugLevel::brief);
    }
    s += "{" + start.debugDump() + ", ";
    for (int i = 0; i < segment->c.pointCount() - 2; ++i)
        s += ctrlPts[i].debugDump() + ", ";
    s += end.debugDump() + "} ";
    if (1 != weight)
        s += "w:" + OpDebugDump(weight) + " ";
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
        for (auto pal : pals)
            s += STR(pal.edge->id) + " ";
    }
    if (EdgeMatch::none != whichEnd)
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != rayFail)
        s += "rayFail:" + debugEdgeFail(rayFail) + " ";
    s += "windZero:" + debugEdgeWindZero(windZero) + "\n";
    if (unsectableID) s += "unsectable:" + STR(unsectableID) + " ";
    if (EdgeSplit::no != doSplit) s += "doSplit";
    if (EdgeSplit::yes == doSplit) s += ":yes";
    if (EdgeSplit::no != doSplit) s += " ";
    if (curveSet) s += "curveSet ";
    if (lineSet) s += "lineSet ";
    if (verticalSet) s += "verticalSet ";
    if (isLine_impl) s += "isLine ";
    if (active_impl) s += "active ";
    if (inOutput) s += "inOutput ";
    if (inLinkups) s += "inLinkups ";
    if (disabled) s += "disabled ";
#if OP_DEBUG
    if (ZeroReason::uninitialized != debugZero) {
        s += " reason:";
        if (outOfDate)
            s += STR((int)debugZero);
        else
            s += std::string(debugReasonNames[(int)debugZero].name);
        s += " ";
    }
#endif
    if (unsortable) s += "unsortable ";
    if (between) s += "between ";
#if OP_DEBUG
    if (debugStart) s += "debugStart:" + STR(debugStart->id) + " ";
    if (debugEnd) s += "debugEnd:" + STR(debugEnd->id) + " ";
    s += "debugMaker:" + debugEdgeDebugMaker(debugMaker) + " ";
    s += debugSetMaker.debugDump() + " ";
    if (debugParentID) s += "debugParentID:" + STR(debugParentID) + " ";
    if (debugRayMatch) s += "debugRayMatch:" + STR(debugRayMatch) + " ";
    if (debugSetSum.line)
        s += debugSetSum.debugDump() + " ";
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
        s += "w:" + OpDebugDump(weight) + " ";
    s += "t{" + OpDebugDump(start.t) + ", " +
        OpDebugDump(center.t) + ", " + OpDebugDump(end.t) + "}";
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
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != rayFail)
        s += "rayFail:" + debugEdgeFail(rayFail) + " ";
    if (EdgeSplit::no != doSplit) s += "doSplit ";
    if (EdgeSplit::yes == doSplit) s += "yes ";
    if (isLine_impl) s += "isLine ";
    if (disabled) s += "disabled ";
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
    edge.segment->contour->contours->dumpMatch(edge.end.pt);
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
    edge.segment->contour->contours->dumpMatch(edge.start.pt);
}

const OpEdge* OpEdgeStorage::debugFind(int ID) const {
	for (int index = 0; index < used; index += sizeof(OpEdge)) {
		const OpEdge* test = (const OpEdge*) &storage[index];
        if (test->id == ID)
            return test;
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
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
    OpDebugOut("linkMatch:" + STR((int) edges.linkMatch) + " linkPass:" + STR((int) edges.linkPass) 
            + " disabledBuilt:" + STR((int) edges.disabledBuilt) + "\n");
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

std::string debugDump(const EdgeDistance& distance, DebugLevel level) {
    std::string s = DebugLevel::brief == level ? "id:" + STR(distance.edge->id) + " " : 
            DebugLevel::detailed == level ? " " + distance.edge->debugDumpDetail() + "\n" :
            distance.edge->debugDump() + " ";
    s += "cept:" + STR(distance.cept) + " ";
    s += "t:" + STR(distance.t) + "\n";
    return s;
}

std::string debugDumpHex(const EdgeDistance& distance, DebugLevel level) {
    std::string s = DebugLevel::brief == level ? "id:" + STR(distance.edge->id) + " " : 
            DebugLevel::detailed == level ? " " + distance.edge->debugDumpDetail() + "\n" :
            distance.edge->debugDumpHex() + " ";
    s += "cept:" + OpDebugDumpHex(distance.cept) + " ";
    s += "t:" + OpDebugDumpHex(distance.t) + "\n";
    return s;
}

void dmpDetail(const EdgeDistance& distance) {
    OpDebugOut(debugDump(distance, DebugLevel::detailed));
}

void dmp(const EdgeDistance& distance) {
    OpDebugOut(debugDump(distance, DebugLevel::normal));
}

void dmpHex(const EdgeDistance& distance) {
    OpDebugOut(debugDumpHex(distance, DebugLevel::normal));
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

std::string debugDump(const SectRay& ray, DebugLevel level) {
    std::string s;
    s = "ray count:" + STR(ray.distances.size()) + " normal:" + STR(ray.normal);
    s += " cept:" + STR(ray.homeCept) + " axis:" + debugAxisName(ray.axis) + "\n";
    for (const EdgeDistance& dist : ray.distances) {
        s += ::debugDump(dist, level);
    }
    return s;
}

std::string debugDumpHex(const SectRay& ray, DebugLevel level) {
    std::string s;
    s = "ray count:" + STR(ray.distances.size()) + " normal:" + OpDebugDumpHex(ray.normal);
    s += " cept:" + OpDebugDumpHex(ray.homeCept) + " axis:" + debugAxisName(ray.axis) + "\n";
    for (const EdgeDistance& dist : ray.distances) {
        s += ::debugDumpHex(dist, level);
    }
    return s;
}

void dmpDetail(const SectRay& ray) {
    OpDebugOut(debugDump(ray, DebugLevel::detailed));
}

void dmp(const SectRay& ray) {
    OpDebugOut(debugDump(ray, DebugLevel::normal));
}

void dmpHex(const SectRay& ray) {
    OpDebugOut(debugDumpHex(ray, DebugLevel::normal));
}

const OpCurveCurve* OpCurveCurve::debugActive;

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

void OpCurveCurve::dump(bool detail) const {
    std::string names[] = { "edge curves", "opp curves", "edge lines", "opp lines", "edge runs", "opp runs" };
    int count = 0;
	for (auto edgesPtrs : { &edgeCurves, &oppCurves, &edgeLines, &oppLines, &edgeRuns, &oppRuns } ) {
        const auto& edges = *edgesPtrs;
        if (edges.size()) {
            int splitCount = 0;
            for (auto& edge : edges)
                splitCount += EdgeSplit::yes == edge.doSplit;
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

#if OP_DEBUG
struct IntersectMakerName {
    IntersectMaker maker;
    const char* name;
};

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

static bool makerOutOfDate = false;

void checkMaker() {
    static bool makerChecked = false;
    if (!makerChecked) {
        for (unsigned index = 0; index < ARRAY_COUNT(intersectMakerNames); ++index)
           if (!makerOutOfDate && (unsigned) intersectMakerNames[index].maker != index) {
               OpDebugOut("!!! intersectMakerNames out of date\n");
               makerOutOfDate = true;
               break;
           }
        makerChecked = true;
    }
}
#endif

struct MatchEndsName {
    MatchEnds end;
    const char* name;
};

#define MATCH_ENDS_NAME(r) { MatchEnds::r, #r }

MatchEndsName matchEndsNames[] {
	MATCH_ENDS_NAME(none),
    MATCH_ENDS_NAME(start),
    MATCH_ENDS_NAME(end),
    MATCH_ENDS_NAME(both)
};

static bool matchEndsOutOfDate = false;

// !!! macro-ize this pattern?
void checkMatchEnds() {
    static bool matchEndsChecked = false;
    if (!matchEndsChecked) {
        for (unsigned index = 0; index < ARRAY_COUNT(matchEndsNames); ++index)
           if (!matchEndsOutOfDate && (unsigned) matchEndsNames[index].end != index) {
               OpDebugOut("!!! matchEndsNames out of date\n");
               matchEndsOutOfDate = true;
               break;
           }
        matchEndsChecked = true;
    }
}

std::string OpIntersection::debugDump(bool fromDumpFull, bool fromDumpDetail) const {
    auto matchEndStr = [this]() {
        return matchEndsOutOfDate ? " (matchEnds out of date) " + STR((int)sectEnd)
                : matchEndsNames[(int)sectEnd].name;
    };
#if OP_DEBUG
    checkMaker();
#endif
    checkMatchEnds();
    checkReason();
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
        s += " " + matchEndStr();
    }
    if (unsectID) {
        s += " unsectID:" + STR(unsectID);
        s += " " + matchEndStr();
    }
    if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && !unsectID && MatchEnds::none != sectEnd)
        s += "!!! (unexpected) " +  matchEndStr();
    if (betweenID)
        s += " betweenID:" + STR(betweenID);
    s += " maker:";
#if OP_DEBUG
    if (makerOutOfDate)
        s += " (maker out of date) " + STR((int)debugMaker);
    else
        s += intersectMakerNames[(int)debugMaker].name;
    if (fromDumpDetail)
        s += " " + debugSetMaker.debugDump();
    s += " reason:";
    if (reasonOutOfDate)
        s += " (reason out of date) " + STR((int)debugReason);
    else
        s += sectReasonNames[(int)debugReason].name;
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
    // don't call comnplete because we don't want to advance debug id
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
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugReasonNames); ++index)
       if (!outOfDate && (unsigned) debugReasonNames[index].reason != index) {
           OpDebugOut("debugReasonNames out of date\n");
           outOfDate = true;
       }
    std::string s = debugDump() + "\n";
    s += " winding: " + winding.debugDump() + " ";
#if OP_DEBUG
    if (ZeroReason::uninitialized != debugZero) {
        s += " reason: ";
        if (outOfDate)
            s += STR((int)debugZero);
        else
            s += std::string(debugReasonNames[(int)debugZero].name);
    }
#endif
    if (recomputeBounds)
        s += "recomputeBounds ";
    if (sects.resort)
        s += "sects.resort ";
    s += "\n";
    s += " bounds:" + ptBounds.debugDump() + " ";
    OP_DEBUG_CODE(s += "contour:" + (contour ? STR(contour->id) : std::string("unset")) + "\n");
    checkReason();
#if OP_DEBUG
    if (reasonOutOfDate)
        s += " (start reason out of date) " + STR((int)debugStart);
    else
        s += std::string(" start reason:") + sectReasonNames[(int)debugStart].name;
    if (reasonOutOfDate)
        s += " (end reason out of date) " + STR((int)debugEnd);
    else
        s += std::string(" end reason:") + sectReasonNames[(int)debugEnd].name;
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

struct DebugWindingTypeName {
    WindingType windType;
    const char* name;
};

#define WINDING_NAME(w) { WindingType::w, #w }

static DebugWindingTypeName debugWindingTypeNames[] = {
    WINDING_NAME(uninitialized),
	WINDING_NAME(temp),
	WINDING_NAME(winding),
	WINDING_NAME(sum)
};

std::string OpWinding::debugDump() const {
    bool outOfDateWT = false;
    for (signed index = 0; index < (signed) ARRAY_COUNT(debugWindingTypeNames); ++index)
       if (!outOfDateWT && (signed) debugWindingTypeNames[index].windType != index - 1) {
           OpDebugOut("debugWindingTypeNames out of date\n");
           outOfDateWT = true;
       }
    std::string result = "type: ";
#if OP_DEBUG
    if (outOfDateWT)
        result += STR((signed)debugType);
    else
        result += std::string(debugWindingTypeNames[(int)debugType + 1].name);
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
        while (prior->priorEdge) {
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
    OpDebugOut(debugEdgeMatch(foundOne.whichEnd) + " " + foundOne.edge->debugDump() + "\n");
}

void dmpDetail(const FoundEdge& foundOne) {
    OpDebugOut(debugEdgeMatch(foundOne.whichEnd) + " " + foundOne.edge->debugDumpDetail() + "\n");
}

void dmpHex(float f) {
    OpDebugOut(OpDebugDumpHex(f) + "\n");
}

std::string OpVector::debugDump() const {
    return "{" + OpDebugDump(dx) + ", " + OpDebugDump(dy) + "}";
}

std::string OpVector::debugDumpHex() const {
    return "{" + OpDebugDumpHex(dx) + ", " + OpDebugDumpHex(dy) + "}";
}

OpPoint::OpPoint(const char*& str) {
    OpDebugSkip(str, "{");
    x = OpDebugHexToFloat(str);
    OpDebugSkip(str, ", ");
    y = OpDebugHexToFloat(str);
    OpDebugSkip(str, "}");
}

std::string OpPoint::debugDump() const {
    return "{" + OpDebugDump(x) + ", " + OpDebugDump(y) + "}";
}

std::string OpPoint::debugDumpHex() const {
    return "{" + OpDebugDumpHex(x) + ", " + OpDebugDumpHex(y) + "}";
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

std::string OpPtT::debugDump() const {
    return pt.debugDump() + " t:" + OpDebugDump(t);
}

std::string OpPtT::debugDumpHex() const {
    return pt.debugDumpHex() + ", " + OpDebugDumpHex(t);
}

void dmp(const OpPtT& ptT) {
    OpDebugOut(ptT.debugDump() + "\n"); 
}

void dmpHex(const OpPtT& ptT) {
    OpDebugOut(ptT.debugDumpHex() + " // " + ptT.debugDump() + "\n");
}

std::string OpRect::debugDump() const {
    return "{" + OpDebugDump(left) + ", " + OpDebugDump(top) + ", "
        + OpDebugDump(right) + ", " + OpDebugDump(bottom) + "}";
}

std::string OpRect::debugDumpHex() const {
    return "{" + OpDebugDumpHex(left) + ", " + OpDebugDumpHex(top) + ", "
        + OpDebugDumpHex(right) + ", " + OpDebugDumpHex(bottom) + "}";
}

void OpRoots::dump() const {
    std::string s = "OpRoots roots {\n"
        "{{ ";
    for (size_t index = 0; index < count; ++index) {
        s += OpDebugDump(roots[index]);
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
        s += "  // " + OpDebugDump(roots.roots[index]) + "\n";
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

#endif
