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
	void dump(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			thing.dump(); \
	} \
	void dumpDetail(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			thing.dumpDetail(); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dump(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			thing->dump(); \
	} \
	void dumpDetail(const std::vector<Thing>& things) { \
		for (const auto& thing : things) \
			thing->dumpDetail(); \
	}
	VECTOR_PTRS
#undef OP_X

#define OP_STRUCTS_2 \
OP_X(LinkUps) \
OP_X(OpContours) \
OP_X(OpCurveCurve) \
OP_X(OpJoiner) \
OP_X(OpOutPath) \
OP_X(OpPtT) \
OP_X(OpPoint) \
OP_X(OpRect) \
OP_X(OpWinder)

#define OP_X(Thing) \
     void Thing::dumpDetail() const { \
         dump(); \
     }
     OP_STRUCTS_2
#undef OP_X

#define OWNER OpContours
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpCurveCurve
#include "OpDebugDefinitions.h"
#undef OWNER

// #define OWNER OpIntersection
// #include "OpDebugDefinitions.h"
// #undef OWNER

#define OWNER OpJoiner
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpSegments
#include "OpDebugDefinitions.h"
#undef OWNER

#define OWNER OpWinder
#include "OpDebugDefinitions.h"
#undef OWNER

void OpContours::dumpActive() const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isActive())
                    edge.dump();
                else if (edge.unsectableID && !edge.inOutput && !edge.inOutQueue)
                    edge.dumpDetail();
            }
        }
    }
    for (auto edge : unsortables) {
        if (edge->unsortable)
            edge->dumpDetail();
    }
}

void dumpCoin(int coinID) {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto sect : seg.intersections) {
                if (coinID == sect->coincidenceID)
                    sect->dump();
            }
        }
    }
}

void dumpCoincidence(int coinID) {
    dumpCoin(coinID);
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
            if (seg.intersections.size())
                s += "OpDebugSectHex " + label + "Sects" + STR(seg.id) + "[] {\n";
            for (const auto intersection : seg.intersections) {
                s += "// " + intersection->debugDumpBrief() + "\n";
                s += intersection->debugDumpHex() + ",\n";
            }
            if (seg.intersections.size())
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

void dump(int ID) {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dump();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dump();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dump();
    const OpIntersection* intersection = findIntersection(ID);
    if (intersection)
        return intersection->dump();
}

void dump(const OpContours& c) {
    c.dumpSegments();
}

void OpContours::dumpEdges() const {
    std::string s;
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            s += seg.debugDump() + "\n";
            for (const auto& edge : seg.edges) {
                s += edge.debugDump() + "\n";
            }
        }
    }
    OpDebugOut(s);
}

void dumpEnd(int ID) {
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dumpEnd();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpEnd();
    const OpIntersection* intersection = findIntersection(ID);
    if (intersection)
        return intersection->dumpPt();
}

// to do : add full dump for edge, intersection ?
void dumpFull(int ID) {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dumpFull();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        segment->dumpFull();
}

void dumpHex(int ID) {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dumpHex();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dumpHex();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpHex();
    const OpIntersection* intersection = findIntersection(ID);
    if (intersection)
        return intersection->dumpHex();
}

void OpContours::dumpIntersections() const {
    std::string s;
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            s += "intersect " + seg.debugDump() + "\n";
            for (const auto sect : seg.intersections) {
                s += "  " + sect->debugDumpDetail(true) + "\n";
            }
        }
    }
    OpDebugOut(s);
}

void OpContours::dumpSects() const {
    dumpIntersections();
}

// to do : add detail dump for edge, intersection ?
void dumpDetail(int ID) {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dumpDetail();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dumpDetail();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpDetail();
    const OpIntersection* sect = findIntersection(ID);
    if (sect)
        return sect->dumpDetail();
}

void dumpLink(int ID) {
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpChain();
}

void dumpLinkDetail(int ID) {
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpChain(true);
}

void OpContours::dumpSegments() const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            seg.dump();
        }
    }
}

void dumpSegmentEdges(int ID) {
    findSegment(ID)->dumpSegmentEdges();
}

void dumpSegmentIntersections(int ID) {
    findSegment(ID)->dumpSegmentIntersections();
}

void dumpSegmentSects(int ID) {
    findSegment(ID)->dumpSegmentSects();
}

void dumpWinding(int ID) {
    findEdge(ID)->dumpWinding();
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

void dumpMatch(const OpPoint& pt, bool detail) {
    checkReason();
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            if (pt == seg.c.pts[0] || pt == seg.c.lastPt()) {
                std::string str = "seg: " 
                        + (detail ? seg.debugDumpDetail() : seg.debugDump());
                if (pt == seg.c.pts[0] && seg.debugStart != SectReason::startPt)
                    str += "; start is " + sectReasonNames[(int) seg.debugStart].name;
                if (pt == seg.c.lastPt() && seg.debugEnd != SectReason::endPt)
                    str += "; end is " + sectReasonNames[(int) seg.debugEnd].name;
                OpDebugOut(str + "\n");
            }
            for (const auto sect : seg.intersections) {
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

void dumpMatch(const OpPoint& pt) {
    dumpMatch(pt, false);
}

void dumpMatch(const OpPtT& ptT) {
    dumpMatch(ptT.pt, false);
}

void dumpMatchDetail(const OpPoint& pt) {
    dumpMatch(pt, true);
}

void dumpMatchDetail(const OpPtT& ptT) {
    dumpMatch(ptT.pt, true);
}

void dumpStart(int ID) {
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dumpStart();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpStart();
    const OpIntersection* intersection = findIntersection(ID);
    if (intersection)
        return intersection->dumpPt();
}

const OpIntersection* findCoincidence(int ID) {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.intersections) {
                if (ID == intersection->coincidenceID || ID == intersection->debugCoincidenceID)
                    return intersection;
            }
        }
    }
    return nullptr;
}

const OpIntersection* findCoin(int ID) {
    return findCoincidence(ID);
}

const OpContour* findContour(int ID) {
    for (const auto& c : debugGlobalContours->contours)
        if (ID == c.id)
            return &c;
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

const OpIntersection* findIntersection(int ID) {
    for (const auto& c : debugGlobalContours->contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.intersections) {
                if (ID == intersection->id)
                    return intersection;
            }
        }
    }
    return nullptr;
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
            for (const auto intersection : seg.intersections) {
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
    std::string s = "contour: " + STR(id) + "\n";
    s += "bounds: " + ptBounds.debugDump() + " ";
    s += "operand: ";
    s += OpOperand::left == operand ? "left" : "right";
    return s;
}

void OpContour::dump() const {
    OpDebugOut(debugDump() + "\n");
    for (auto& segment : segments)
        segment.dump();
}

void dump(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dump();
}

void OpContour::dumpDetail() const {
    OpDebugOut(debugDump() + "\n");
    for (auto& segment : segments)
        segment.dumpDetail();
}

void dumpDetail(std::vector<OpContour>& contours) {
    for (const auto& c : contours)
        c.dumpDetail();
}

void OpContour::dumpFull() const {
    OpDebugOut(debugDump() + "\n");
    for (auto& segment : segments)
        segment.dumpFull();
}

void OpContour::dumpHex() const {
    OpDebugOut(debugDump() + "\n");
    for (auto& segment : segments)
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
    FAIL_NAME(nextDistance),
    FAIL_NAME(priorDistance),
    FAIL_NAME(recalcCenter),
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

struct EdgeMakerName {
    EdgeMaker maker;
    const char* name;
};

#define EDGE_MAKER_NAME(r) { EdgeMaker::r, #r }

static EdgeMakerName edgeMakerNames[] {
    EDGE_MAKER_NAME(empty),
    EDGE_MAKER_NAME(intersectEdge1),
    EDGE_MAKER_NAME(intersectEdge2),
    EDGE_MAKER_NAME(makeEdges),
    EDGE_MAKER_NAME(oppSect),
    EDGE_MAKER_NAME(resolveCoin1),
    EDGE_MAKER_NAME(resolveCoin2),
    EDGE_MAKER_NAME(segSect),
    EDGE_MAKER_NAME(split1),
    EDGE_MAKER_NAME(split2),
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

std::string OpEdge::debugDumpDetail() const {
    std::string s = "edge[" + STR(id) + "] segment[" + STR(segment->id) + "] contour["
            + STR(segment->contour->id) + "]\n";
    if (priorWind.size()) {
        s += "priorW:";
        for (const WindDist& wd : priorWind)
            s += STR(wd.edge->id) + " ";
    }   
    if (nextWind.size()) {
        s += " nextW:";
        for (const WindDist& wd : nextWind)
            s += STR(wd.edge->id) + " ";
    }
    if (priorWind.size() || nextWind.size())
        s += "\n";
    if (priorEdge || nextEdge || lastEdge) {
        s += "priorE/nextE/lastE:" + (priorEdge ? STR(priorEdge->id) : "-");
        s += "/" + (nextEdge ? STR(nextEdge->id) : "-");
        s += "/" + (lastEdge ? STR(lastEdge->id) : "-");
        s += " ";
    }
    if (priorEdge || nextEdge || lastEdge /* || priorSum_impl || loopStart || Axis::neither != sumAxis */ )
        s += "\n";
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
            s += STR(pal->id) + " ";
    }
    if (EdgeMatch::none != whichEnd)
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != fail)
        s += "fail:" + debugEdgeFail(fail) + " ";
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
    if (inOutQueue) s += "inOutQueue ";
    if (disabled) s += "disabled ";
    if (unsortable) s += "unsortable ";
    if (between) s += "between ";
    if (debugStart) s += "debugStart:" + STR(debugStart->id) + " ";
    if (debugEnd) s += "debugEnd:" + STR(debugEnd->id) + " ";
    s += "debugMaker:" + debugEdgeDebugMaker(debugMaker) + " ";
    s += debugMakerFile.substr(debugMakerFile.find("Op")) + ":" + STR(debugMakerLine) + " ";
    if (debugParentID) s += "debugParentID:" + STR(debugParentID) + " ";
    if (debugSetSumLine)
        s += debugSetSumFile.substr(debugSetSumFile.find("Op")) + ":" + STR(debugSetSumLine) + " ";
    if (debugUnOpp)
        s += "debugUnOpp ";
    s += "\n";
    std::vector<OpIntersection*> startSects;
    std::vector<OpIntersection*> endSects;
    for (auto sect : segment->intersections) {
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
            s += STR(sect->id) + " ";
    }
    s += " ";
    if (!endSects.size())
        s += "!!! missing end intersection at t:" + STR(end.t);
    else {
        s += "end sects:";
        for (auto sect : endSects)
            s += STR(sect->id) + " ";
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

void OpEdge::dumpHex() const {
    OpDebugOut(debugDumpHex() + "\n");
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

const OpEdge* OpEdge::debugAdvanceToEnd(EdgeMatch match) const {
	const OpEdge* result = this;
	while (const OpEdge* edge = (EdgeMatch::start == match ? result->priorEdge : result->nextEdge)) {
        OP_ASSERT(EdgeMatch::start == match ? 
                result->priorEdge->nextEdge == result : 
                result->nextEdge->priorEdge == result);
		result = edge;
	}
    OP_ASSERT(result == const_cast<OpEdge*>(this)->advanceToEnd(match));
	return result;
}


void OpEdge::debugCompare(std::string s) const {
    OpEdge test(s);
    OP_ASSERT(segment->id == test.segment->id);
    OP_ASSERT(start == test.start);
    OP_ASSERT(end == test.end);
}

struct DebugReasonName {
    ZeroReason reason;
    const char* name;
};

#define REASON_NAME(r) { ZeroReason::r, #r }

static DebugReasonName debugReasonNames[] {
    REASON_NAME(uninitialized),
    REASON_NAME(addIntersection),
    REASON_NAME(applyOp),
    REASON_NAME(centerNaN),
    REASON_NAME(findCoincidences),
    REASON_NAME(hvCoincidence),
    REASON_NAME(isPoint),
    REASON_NAME(many),
    REASON_NAME(noFlip),
};

std::string OpEdge::debugDump() const {
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugReasonNames); ++index)
       if (!outOfDate && (unsigned) debugReasonNames[index].reason != index) {
           OpDebugOut("debugReasonNames out of date\n");
           outOfDate = true;
       }
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
        OpDebugDump(center.t) + ", " + OpDebugDump(end.t) + "}\n";
    if (ptBounds.isSet())
        s += "    pb:" + ptBounds.debugDump() + " ";
    if (linkBounds.isSet())
        s += "    lb:" + linkBounds.debugDump() + " ";
    s += "wind:" + STR(winding.left()) + "/" + STR(winding.right()) + " ";
    if (OpMax != sum.left() || OpMax != sum.right()) {
        s += "l/r:" + (OpMax != sum.left() ? STR(sum.left()) : "--");
        s += "/" + (OpMax != sum.right() ? STR(sum.right()) : "--") + " ";
    }
    if (EdgeMatch::none != whichEnd)
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != fail)
        s += "fail:" + debugEdgeFail(fail) + " ";
    if (EdgeSplit::no != doSplit) s += "doSplit ";
    if (EdgeSplit::yes == doSplit) s += "yes ";
    if (isLine_impl) s += "isLine ";
    if (unsectableID) s += "uID:" + STR(unsectableID) + " ";
    if (unsortable) s += "unsortable ";
    if (ZeroReason::uninitialized != debugZero) {
        s += " reason:";
        if (outOfDate)
            s += STR((int)debugZero);
        else
            s += std::string(debugReasonNames[(int)debugZero].name);
        s += " ";
    }
    s += "seg:" + STR(segment->id);
    return s;
}

// !!! move to OpDebug.cpp
void OpEdge::debugValidate() const {
    debugGlobalContours->debugValidateEdgeIndex += 1;
    bool loopy = debugIsLoop();
    if (loopy) {
        const OpEdge* test = this;
        do {
            OP_ASSERT(!test->priorEdge || test->priorEdge->nextEdge == test);
            OP_ASSERT(!test->nextEdge || test->nextEdge->priorEdge == test);
//            OP_ASSERT(!test->lastEdge);
            test = test->nextEdge;
        } while (test != this);
    } else if ((priorEdge || lastEdge) && debugGlobalContours->debugCheckLastEdge) {
        const OpEdge* linkStart = debugAdvanceToEnd(EdgeMatch::start);
        const OpEdge* linkEnd = debugAdvanceToEnd(EdgeMatch::end);
        OP_ASSERT(linkStart);
        OP_ASSERT(linkEnd);
        OP_ASSERT(debugGlobalContours->debugCheckLastEdge ? !!linkStart->lastEdge : !linkStart->lastEdge);
        OP_ASSERT(debugGlobalContours->debugCheckLastEdge ? linkStart->lastEdge == linkEnd : true);
        const OpEdge* test = linkStart;
        while ((test = test->nextEdge)) {
            OP_ASSERT(!test->lastEdge);
            OP_ASSERT(linkEnd == test ? !test->nextEdge : !!test->nextEdge);
        }
    }
    for (auto& edge : segment->edges) {
        if (&edge == this)
            return;
    }
    OP_ASSERT(0);
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
        if (++safetyCount > 100) {
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
    if (many.isSet())
        s += "\nmany: " + many.debugDump();
    return s;
}

void OpEdge::dump() const {
    std::string s = debugDump() + "\n";
    OpDebugOut(s);
}

void dump(const OpEdge* edge) {
    edge->dump();
}

void dump(const OpEdge& edge) {
    edge.dump();
}

void OpEdge::dumpChain(bool detail) const {
    OpDebugOut("prior: " + debugDumpChain(WhichLoop::prior, detail) + "\n");
    OpDebugOut("next: " + debugDumpChain(WhichLoop::next, detail) + "\n");
}

void OpEdge::dumpDetail() const { 
    std::string s = debugDumpDetail() + "\n";
    OpDebugOut(s);
}

void OpEdge::dumpEnd() const {
    segment->contour->contours->dumpMatch(end.pt);
}

void OpEdge::dumpFull() const { 
    dumpDetail(); 
}

void OpEdge::dumpLink() const {
    dumpChain();
}

void dumpLink(const OpEdge& edge) {
    edge.dumpLink();
}

void dumpLink(const OpEdge* edge) {
    edge->dumpLink();
}

void OpEdge::dumpLinkDetail() const {
    dumpChain(true);
}

void dumpLinkDetail(const OpEdge& edge) {
    edge.dumpLinkDetail();
}

void dumpLinkDetail(const OpEdge* edge) {
    edge->dumpLinkDetail();
}

void OpEdge::dumpStart() const {
    segment->contour->contours->dumpMatch(start.pt);
}

void OpEdge::dumpWinding() const {
    dumpDetail();
}

DUMP_STRUCT_DEFINITIONS(OpEdge)
DEBUG_DUMP_ID_DEFINITION(OpEdge, id)

// !!! also debug prev/next edges (links)
void OpJoiner::debugValidate() const {
    debugGlobalContours->debugValidateJoinerIndex += 1;
    debugGlobalContours->debugCheckLastEdge = false;
    if (LinkPass::unambiguous == linkPass) {
        for (auto edge : byArea) {
            edge->debugValidate();
            OP_ASSERT(!edge->isActive() || !edge->debugIsLoop());
        }
    }
    for (auto edge : unsectByArea) {
        edge->debugValidate();
        OP_ASSERT(!edge->isActive() || !edge->debugIsLoop());
    }
    for (auto edge : disabled) {
        edge->debugValidate();
//        OP_ASSERT(!edge->debugIsLoop());
    }
    for (auto edge : unsortables) {
        edge->debugValidate();
        OP_ASSERT(!edge->isActive() || !edge->debugIsLoop());
    }
    debugGlobalContours->debugCheckLastEdge = true;
    for (auto edge : linkups.l) {
        edge->debugValidate();
        OP_ASSERT(!edge->debugIsLoop());
    }
}

void dump(const OpJoiner& edges) {
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
        ::dump(edges.disabled);
    }
    if (edges.unsortables.size()) {
        OpDebugOut("-- unsortables --\n");
        for (auto edge : edges.unsortables)
            if (edge->isActive())
                edge->dump();
    }
    if (edges.linkups.l.size()) {
        OpDebugOut("-- linkups --\n");
        ::dump(edges.linkups);
    }
    OpDebugOut("linkMatch:" + STR((int) edges.linkMatch) + " linkPass:" + STR((int) edges.linkPass) 
            + " disabledBuilt:" + STR((int) edges.disabledBuilt) + "\n");
}

void OpWinder::debugValidate() const {
    for (auto& edge : inX)
        edge->debugValidate();
    for (auto& edge : inY)
        edge->debugValidate();
}

void OpWinder::dumpAxis(Axis axis) const {
    std::string s = "";
    for (const auto edge : Axis::vertical == axis ? inY : inX) {
        s += edge->debugDump() + "\n";
    }
    OpDebugOut(s);
}

void dump(const OpWinder& edges) {
    if (edges.inX.size()) {
        OpDebugOut("-- sorted in x --\n");
        edges.dumpAxis(Axis::horizontal);
    }
    if (edges.inY.size()) {
        OpDebugOut("-- sorted in y --\n");
        edges.dumpAxis(Axis::vertical);
    }
}

struct DistMultName {
    DistMult distMult;
    std::string name;
};

#define DIST_MULT_NAME(r) { DistMult::r, #r }

DistMultName distMultNames[] {
	DIST_MULT_NAME(none),
	DIST_MULT_NAME(first),
	DIST_MULT_NAME(mid),
	DIST_MULT_NAME(last),
};

static bool distMultOutOfDate = false;

void checkDistMult() {
    static bool distMultOutChecked = false;
    if (!distMultOutChecked) {
        for (unsigned index = 0; index < ARRAY_COUNT(distMultNames); ++index)
           if (!distMultOutOfDate && (unsigned) distMultNames[index].distMult != index) {
               OpDebugOut("!!! distMultNames out of date\n");
               distMultOutOfDate = true;
               break;
           }
        distMultOutChecked = true;
    }
}

void dump(const EdgeDistance& distance) {
    checkDistMult();
    OpDebugOut(distance.edge->debugDump() + "\n");
    OpDebugOut("distance:" + STR(distance.distance) + " ");
    OpDebugOut("normal:" + STR(distance.normal) + " ");
    OpDebugOut("t:" + STR(distance.t) + " ");
    if (distMultOutOfDate)
        OpDebugOut("(distMult out of date) " + STR((int)distance.multiple));
    else
        OpDebugOut("multiple:" + distMultNames[(int)distance.multiple].name + "\n");
}

void EdgeDistance::dump() const {
    ::dump(*this);
}

void dumpDetail(const EdgeDistance& distance) {
    checkDistMult();
    OpDebugOut(distance.edge->debugDumpDetail() + "\n");
    OpDebugOut("distance:" + STR(distance.distance) + " ");
    OpDebugOut("normal:" + STR(distance.normal) + " ");
    OpDebugOut("t:" + STR(distance.t) + " ");
    if (distMultOutOfDate)
        OpDebugOut("(distMult out of date) " + STR((int)distance.multiple));
    else
        OpDebugOut("multiple:" + distMultNames[(int)distance.multiple].name + "\n");
}

void EdgeDistance::dumpDetail() const {
    ::dumpDetail(*this);
}

const OpCurveCurve* OpCurveCurve::debugActive;

static int debugSavedID = -1;

void OpCurveCurve::debugSaveID() {
    OP_ASSERT(-1 == debugSavedID);
    debugSavedID = debugGlobalContours->id;
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
    debugGlobalContours->id = debugSavedID;
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
            detail ? ::dump(edges) : ::dumpDetail(edges);
        }
        ++count;
    }
}

void dump(const OpCurveCurve& cc) {
    cc.dump(false);
}

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

std::string OpIntersection::debugDump(bool fromDumpFull, bool fromDumpDetail) const {
    static bool makerChecked = false;
    static bool makerOutOfDate = false;
    if (!makerChecked) {
        for (unsigned index = 0; index < ARRAY_COUNT(intersectMakerNames); ++index)
           if (!makerOutOfDate && (unsigned) intersectMakerNames[index].maker != index) {
               OpDebugOut("!!! intersectMakerNames out of date\n");
               makerOutOfDate = true;
               break;
           }
        makerChecked = true;
    }
    checkReason();
    std::string s;
    std::string segmentID = segment ? segment->debugDumpID() : "--";
    const OpSegment* oppParent = opp ? opp->segment : nullptr;
    std::string oppID = opp ? opp->debugDumpID() : "--";
    std::string oppParentID = oppParent ? oppParent->debugDumpID() : "--";
    s = "[" + debugDumpID() + "] " + ptT.debugDump();
    if (debugErased)
        s += " erased";
    if (!fromDumpFull || !segment)
        s += " segment:" + segmentID;
    s += " opp/sect:" + oppParentID + "/" + oppID;
    if (coincidenceID || debugCoincidenceID)
        s += " coinID:" + STR(coincidenceID) + "/" + STR(debugCoincidenceID);
    if (unsectableID)
        s += " unsectableID:" + STR(unsectableID);
    if (betweenID)
        s += " betweenID:" + STR(betweenID);
    s += " maker:";
    if (makerOutOfDate)
        s += " (maker out of date) " + STR((int)debugMaker);
    else
        s += intersectMakerNames[(int)debugMaker].name;
    if (fromDumpDetail)
        s += " " + debugMakerFile.substr(debugMakerFile.find("Op")) + ":" + STR(debugMakerLine);
    s += " reason:";
    if (reasonOutOfDate)
        s += " (reason out of date) " + STR((int)debugReason);
    else
        s += sectReasonNames[(int)debugReason].name;
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
    auto edgeOrSegment = [](int debug_id, std::string label) {
        if (::findEdge(debug_id))
            return label + " (edge) " + STR(debug_id) + " ";
        if (::findSegment(debug_id))
            return label + " (segment) " + STR(debug_id) + " ";
        return STR(debug_id) + " not found ";
    };
    std::string s = debugDump(fromDumpIntersections, true);
    if (debugID || debugOppID)
        s += "\n";
    if (debugID)
        s += edgeOrSegment(debugID, "debugID:");
    if (debugOppID)
        s += edgeOrSegment(debugOppID, "debugOpp:");
    return s;
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

void OpIntersection::dumpDetail() const {
    OpDebugOut(debugDumpDetail() + "\n");
}

void dumpDetail(std::vector<OpIntersection>& sects) {
    for (const auto& s : sects)
        s.dumpDetail();
}

void OpIntersection::dumpHex() const { 
    OpDebugOut(debugDumpHex() + "\n"); 
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

void OpIntersection::dump() const {
    std::string s = debugDump(false, false) + "\n";
    OpDebugOut(s);
}

void dump(std::vector<OpIntersection>& sects) {
    for (const auto& s : sects)
        s.dump();
}

void OpIntersection::dumpPt() const {
    segment->contour->contours->dumpMatch(ptT.pt);
}

DEBUG_DUMP_ID_DEFINITION(OpIntersection, id)
DUMP_STRUCT_DEFINITIONS(OpIntersection)

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
    if (ZeroReason::uninitialized != debugZero) {
        s += " reason: ";
        if (outOfDate)
            s += STR((int)debugZero);
        else
            s += std::string(debugReasonNames[(int)debugZero].name);
    }
    if (recomputeBounds)
        s += "recomputeBounds ";
    if (resortIntersections)
        s += "resortIntersections ";
    s += "\n";
    s += " bounds:" + ptBounds.debugDump() + " ";
    s += "contour:" + (contour ? STR(contour->id) : std::string("unset")) + "\n";
    checkReason();
    if (reasonOutOfDate)
        s += " (start reason out of date) " + STR((int)debugStart);
    else
        s += std::string(" start reason:") + sectReasonNames[(int)debugStart].name;
    if (reasonOutOfDate)
        s += " (end reason out of date) " + STR((int)debugEnd);
    else
        s += std::string(" end reason:") + sectReasonNames[(int)debugEnd].name;
    return s;
}

// !!! probably shouldn't include trailing \n
std::string OpSegment::debugDumpEdges() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDump() + "\n";
    return s;
}

std::string OpSegment::debugDumpFull() const {
    std::string s = debugDump() + "\n";
    s += debugDumpIntersections();
    s += debugDumpEdges();
    return s;
}

std::string OpSegment::debugDumpHex() const {
    return "/* seg:" + debugDumpID() + " */ " + c.debugDumpHex();
}

std::string OpSegment::debugDumpIntersections() const {
    std::string s;
    for (auto i : intersections)
        s += i->debugDump(true, false) + "\n";
    return s;
}

// !!! move to OpDebug.cpp
void OpSegment::debugValidate() const {
    for (auto i : intersections)
        i->debugValidate();
}

void OpSegment::dump() const {
    OpDebugOut(debugDump() + "\n");
}

void OpSegment::dumpCount() const {
    OpDebugOut("seg:" + debugDumpID() + " edges:" + STR(edges.size())
            + " intersections:" + STR(intersections.size()) + "\n");
}

void OpSegment::dumpDetail() const {
    OpDebugOut(debugDumpDetail() + "\n"); 
}

void OpSegment::dumpSegmentEdges() const {
    OpDebugOut(debugDumpEdges() + "\n");
}

void OpSegment::dumpEnd() const {
    contour->contours->dumpMatch(c.lastPt());
}

void OpSegment::dumpFull() const {
    OpDebugOut(debugDumpFull() + "\n"); 
}

void dumpFull(const OpSegment& segment) {
    segment.dumpFull();
}

void dumpFull(const OpSegment* segment) {
    segment->dumpFull();
}

void OpSegment::dumpHex() const { 
    OpDebugOut(debugDumpHex()); 
}

void OpSegment::dumpSegmentIntersections() const {
    OpDebugOut(debugDumpIntersections() + "\n");
}

void OpSegment::dumpSegmentSects() const {
    OpDebugOut(debugDumpIntersections() + "\n");
}

void OpSegment::dumpStart() const {
    contour->contours->dumpMatch(c.pts[0]);
}

DEBUG_DUMP_ID_DEFINITION(OpSegment, id)
DUMP_STRUCT_DEFINITIONS(OpSegment)

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
    if (outOfDateWT)
        result += STR((signed)debugType);
    else
        result += std::string(debugWindingTypeNames[(int)debugType + 1].name);
    result += " left: " + (INT_MAX == left() ? std::string("unset") : STR(left()));
    result += " right: " + (INT_MAX == right() ? std::string("unset") : STR(right()));
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
        OpDebugOut(str + " area:" + STR(linkup->linkedArea()) + "\n");
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

void dump(const LinkUps& linkups) {
    DumpLinkups(linkups.l);
}

void LinkUps::dump() const {
    DumpLinkups(l);
}

void dump(const FoundEdge& foundOne) {
    OpDebugOut(debugEdgeMatch(foundOne.whichEnd) + " " + foundOne.edge->debugDump() + "\n");
}

void FoundEdge::dump() const {
    ::dump(*this);
}

void dumpDetail(const FoundEdge& foundOne) {
    OpDebugOut(debugEdgeMatch(foundOne.whichEnd) + " " + foundOne.edge->debugDumpDetail() + "\n");
}

void FoundEdge::dumpDetail() const {
    ::dumpDetail(*this);
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

void OpPoint::dump() const {
    OpDebugOut("OpPoint pt { " + debugDump() + " };\n");
}

void dump(const OpPoint& pt) {
    pt.dump();
}

void dump(const OpPoint* pt) {
    pt->dump();
}

void OpPoint::dumpHex() const {
    OpDebugOut("OpPoint pt { " + debugDumpHex() + " };  // " + debugDump() + "\n");
}

void dump(const OpPointBounds& pb) {
    OpDebugOut(pb.debugDump() + "\n");
}

void dump(const OpPointBounds* pb) {
    dump(*pb);
}

void dump(const OpRect& r) {
    OpDebugOut(r.debugDump() + "\n");
}

void dump(const OpRect* r) {
    dump(*r);
}

void dump(const OpTightBounds& tb) {
    OpDebugOut(tb.debugDump() + "\n");
}

void dump(const OpTightBounds* tb) {
    dump(*tb);
}

void dumpHex(const OpPointBounds& pb) {
    OpDebugOut(pb.debugDumpHex() + " // " + pb.debugDump() + "\n");
}

void dumpHex(const OpPointBounds* pb) {
    dump(*pb);
}

void dumpHex(const OpRect& r) {
    OpDebugOut(r.debugDumpHex() + " // " + r.debugDump() + "\n");
}

void dumpHex(const OpRect* r) {
    dumpHex(*r);
}

void dumpHex(const OpTightBounds& tb) {
    OpDebugOut(tb.debugDumpHex() + " // " + tb.debugDump() + "\n");
}

void dumpHex(const OpTightBounds* tb) {
    dumpHex(*tb);
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

void OpPtT::dump() const { 
    OpDebugOut(debugDump() + "\n"); 
}

void dump(const OpPtT& ptT) {
    ptT.dump();
}

void dump(const OpPtT* ptT) {
    ptT->dump();
}

void OpPtT::dumpHex() const {
    OpDebugOut(debugDumpHex() + " // " + debugDump() + "\n");
}

std::string OpRect::debugDump() const {
    return "{" + OpDebugDump(left) + ", " + OpDebugDump(top) + ", "
        + OpDebugDump(right) + ", " + OpDebugDump(bottom) + "}";
}

std::string OpRect::debugDumpHex() const {
    return "{" + OpDebugDumpHex(left) + ", " + OpDebugDumpHex(top) + ", "
        + OpDebugDumpHex(right) + ", " + OpDebugDumpHex(bottom) + "}";
}

void OpRect::dump() const {
    OpDebugOut(debugDump());
}

void OpRect::dumpHex() const {
    OpDebugOut(debugDumpHex());
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

void OpRoots::dumpHex() const {
    std::string s = "OpRoots roots { {{\n";
    for (size_t index = 0; index < count; ++index) {
        s += "  " + OpDebugDumpHex(roots[index]);
        if (index < count - 1)
            s += ", ";
        s += "  // " + OpDebugDump(roots[index]) + "\n";
    }
    s += "}}, " + STR(count) + " };\n";
    OpDebugOut(s);
}

void dump(const OpSegments& segs) {
    std::string s = "";
    for (const auto seg : segs.inX) {
        s += seg->debugDump() + "\n";
    }
    OpDebugOut(s);
}

void OpSegments::dumpDetail() const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDumpDetail() + "\n";
    }
    OpDebugOut(s);
}

void OpSegments::dumpEdges() const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDumpEdges() + "\n";
    }
    OpDebugOut(s);
}

void OpSegments::dumpIntersections() const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDumpIntersections() + "\n";
    }
    OpDebugOut(s);
}

void OpSegments::dumpFull() const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDumpFull() + "\n";
    }
    OpDebugOut(s);
}

void OpSegments::dumpHex() const {
    std::string s = "";
    for (const auto seg : inX) {
        s += seg->debugDumpHex() + "\n";
    }
    OpDebugOut(s);
}

void dump(const OpOutPath& path) {
    path.dump();
}

#endif
