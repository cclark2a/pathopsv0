#include "OpDebug.h"
#include "OpDebugDump.h"

#if OP_DEBUG_DUMP
#include "OpContour.h"
#include "OpEdge.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "PathOps.h"

#pragma optimize( "", off )

DUMP_GLOBAL_DEFINITIONS()

const OpContour* findContour(int id) {
    return debugGlobalContours->findContour(id);
}

const OpEdge* findEdge(int id) {
    return debugGlobalContours->findEdge(id);
}

const OpIntersection* findCoin(int id) {
    return debugGlobalContours->findCoincidence(id);
}

const OpIntersection* findCoincidence(int id) {
    return debugGlobalContours->findCoincidence(id);
}

const OpIntersection* findIntersection(int id) {
    return debugGlobalContours->findIntersection(id);
}

const OpSegment* findSegment(int id) {
    return debugGlobalContours->findSegment(id);
}

void OpContours::dumpCoin(int coinID) const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            for (const auto sect : seg.intersections) {
                if (coinID == sect->coincidenceID)
                    sect->dump();
            }
        }
    }
}

void OpContours::dumpCoincidence(int coinID) const {
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

// if file exists, read old state
void OpContours::dumpCount(std::string label) const {
    FILE* file = fopen(label.c_str(), "rb");
    char* buffer = nullptr;
    long size = 0;
    if (file) {
        int seek = fseek(file, 0, SEEK_END);
        assert(!seek);
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
    assert(result == s.length());
    fclose(file);
    fflush(file);
    free(buffer);
}

void OpContours::dump(int ID) const {
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

void OpContours::dumpEnd(int ID) const {
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
void OpContours::dumpFull(int ID) const {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dumpFull();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        segment->dumpFull();
}

void OpContours::dumpHex(int ID) const {
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
                s += "  " + sect->debugDump(true) + "\n";
            }
        }
    }
    OpDebugOut(s);
}

void OpContours::dumpSects() const {
    dumpIntersections();
}

// to do : add detail dump for edge, intersection ?
void OpContours::dumpDetail(int ID) const {
    const OpContour* contour = findContour(ID);
    if (contour)
        return contour->dumpDetail();
    const OpSegment* segment = findSegment(ID);
    if (segment)
        return segment->dumpDetail();
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpDetail();
}

void OpContours::dumpLink(int ID) const {
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpChain(EdgeLoop::link);
}

void OpContours::dumpLinkDetail(int ID) const {
    const OpEdge* edge = findEdge(ID);
    if (edge)
        return edge->dumpChain(EdgeLoop::link, true);
}

void OpContours::dumpSegments() const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            seg.dump();
        }
    }
}

void OpContours::dumpSegmentEdges(int ID) const {
    findSegment(ID)->dumpSegmentEdges();
}

void OpContours::dumpSegmentIntersections(int ID) const {
    findSegment(ID)->dumpSegmentIntersections();
}

void OpContours::dumpSegmentSects(int ID) const {
    findSegment(ID)->dumpSegmentSects();
}

void OpContours::dumpWinding(int ID) const {
    findEdge(ID)->dumpWinding();
}

void OpContours::dumpMatch(const OpPoint& pt) const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            if (pt == seg.c.pts[0]) {
                OpDebugOut("seg: ");
                seg.dump();
            }
            if (pt == seg.c.lastPt()) {
                OpDebugOut("seg: ");
                seg.dump();
            }
            for (int index = 0; index < 2; ++index) {
                if (pt == seg.tightBounds.xExtrema[index].pt)
                    OpDebugOut("xExtrema[" + STR(index) + "]: " + seg.debugDumpDetail());
                if (pt == seg.tightBounds.yExtrema[index].pt)
                    OpDebugOut("yExtrema[" + STR(index) + "]: " + seg.debugDumpDetail());
                if (pt == seg.tightBounds.inflections[index].pt)
                    OpDebugOut("inflections[" + STR(index) + "]: " + seg.debugDumpDetail());
            }
            for (const auto sect : seg.intersections) {
                if (sect->ptT.pt == pt) {
                    OpDebugOut("sect: ");
                    sect->dump();
                }
                if (sect->debugOriginal == pt) {
                    OpDebugOut("sect original: ");
                    sect->dump();
                }
            }
            for (const auto& edge : seg.edges) {
                if (edge.start.pt == pt) {
                    OpDebugOut("edge start: ");
                    edge.dump();
                }
                if (edge.debugOriginalStart == pt) {
                    OpDebugOut("edge original start: ");
                    edge.dump();
                }
                if (edge.end.pt == pt) {
                    OpDebugOut("edge end: ");
                    edge.dump();
                }
                if (edge.debugOriginalEnd == pt) {
                    OpDebugOut("edge original end: ");
                    edge.dump();
                }
            }
        }
    }
}

void OpContours::dumpStart(int ID) const {
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

void OpContours::dumpSum(int ID) const {
    const OpEdge* edge = findEdge(ID);
    if (!edge) {
        OpDebugOut("edge id: " + STR(ID) + " not found\n");
        return;
    }
    edge->dumpChain(EdgeLoop::sum);
}

void OpContours::dumpSumDetail(int ID) const {
    const OpEdge* edge = findEdge(ID);
    if (!edge) {
        OpDebugOut("edge id: " + STR(ID) + " not found\n");
        return;
    }
    edge->dumpChain(EdgeLoop::sum, true);
}

DEBUG_DUMP_ID_DEFINITION(OpContours, id)

const OpIntersection* OpContours::findCoincidence(int ID) const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.intersections) {
                if (ID == intersection->coincidenceID || ID == intersection->debugCoincidenceID)
                    return intersection;
            }
        }
    }
    return nullptr;
}

const OpIntersection* OpContours::findCoin(int ID) const {
    return findCoincidence(ID);
}

const OpContour* OpContours::findContour(int ID) const {
    for (const auto& c : contours)
        if (ID == c.id)
            return &c;
    return nullptr;
}

const OpEdge* OpContours::findEdge(int ID) const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            for (const auto& edge : seg.edges) {
                if (ID == edge.id)
                    return &edge;
            }
        }
    }
    // if edge intersect is active, search there too
    if (OpEdgeIntersect::debugActive) {
        for (const auto& edge : OpEdgeIntersect::debugActive->edgeParts) {
            if (ID == edge.id)
                return &edge;
        }
         for (const auto& edge : OpEdgeIntersect::debugActive->oppParts) {
            if (ID == edge.id)
                return &edge;
        }
   }
    return nullptr;
}

const OpIntersection* OpContours::findIntersection(int ID) const {
    for (const auto& c : contours) {
        for (const auto& seg : c.segments) {
            for (const auto intersection : seg.intersections) {
                if (ID == intersection->id)
                    return intersection;
            }
        }
    }
    return nullptr;
}

const OpSegment* OpContours::findSegment(int ID) const {
    for (const auto& c : contours) {
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
        assert(';' == *str++);
        if ('\r' == str[0])
            ++str;
        assert('\n' == *str++);
        if ('\r' == str[0])
            ++str;
        assert('\n' == *str++);
    }
    const char* structCheck = "OpDebug";
    if (!strncmp(structCheck, str, sizeof(structCheck) - 1)) {
        str = strchr(str, '\n');
        assert(str);
        str += 1;
    }
    assert(!strncmp("// ", str, 3));
    const char* start = strchr(str, '\n');
    assert(start);
    start += 1;
    assert('{' == start[0]);
    const char* end = strchr(start, '\n');
    assert(end);
    str = end + 1;
    if ('\r' == end[-1])
        --end;
    assert(',' == end[-1]);
    assert('/' == str[0] || '}' == str[0]);
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

void OpContour::dump() const {
    OpDebugOut("contour: " + STR(id) + "\n");
    for (auto& segment : segments)
        segment.dump();
}

void OpContour::dumpDetail() const {
    OpDebugOut("contour: " + STR(id) + "\n");
    for (auto& segment : segments)
        segment.dumpDetail();
}

void OpContour::dumpFull() const {
    OpDebugOut("contour: " + STR(id) + "\n");
    for (auto& segment : segments)
        segment.dumpFull();
}

void OpContour::dumpHex() const {
    OpDebugOut("contour: " + STR(id) + "\n");
    for (auto& segment : segments)
        segment.dumpHex();
}

std::string OpCurve::debugDump() const {
    const char* names[] { "pointType", "lineType", "quadType", "conicType", "cubicType" };
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
    s += " " + (pointType <= type && type <= cubicType ? names[type] :
        "noType [" + STR(type) + "]");
    return s;
}

std::string OpCurve::debugDumpHex() const {
    const char* names[] { "pointType", "lineType", "quadType", "conicType", "cubicType" };
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
    s += "  // type:" + (pointType <= type && type <= cubicType ? names[type] :
        "noType [" + STR(type) + "]");
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

struct DebugLinkedName {
    EdgeLink linked;
    const char* name;
};

#define LINKED_NAME(r) { EdgeLink::r, #r }

static DebugLinkedName debugLinkedNames[] {
    LINKED_NAME(unlinked),
    LINKED_NAME(single),
    LINKED_NAME(multiple),
};

std::string debugLinkedUp(EdgeLink linked) {
    std::string result;
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugLinkedNames); ++index) {
        if (!outOfDate && (unsigned)debugLinkedNames[index].linked != index) {
            OpDebugOut("debugLinkedNames out of date\n");
            outOfDate = true;
        }
        if (linked != debugLinkedNames[index].linked)
            continue;
        if (outOfDate)
            result += STR((int)linked);
        else
            result += std::string(debugLinkedNames[(int)linked].name);
    }
    return result;
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
    FAIL_NAME(nextDistance),
    FAIL_NAME(priorDistance),
    FAIL_NAME(recalcCenter),
    FAIL_NAME(winding),
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
    EDGE_MAKER_NAME(addTest),
    EDGE_MAKER_NAME(intersectEdge1),
    EDGE_MAKER_NAME(intersectEdge2),
    EDGE_MAKER_NAME(makeEdges),
    EDGE_MAKER_NAME(opTest),
    EDGE_MAKER_NAME(resolveCoin1),
    EDGE_MAKER_NAME(resolveCoin2),
    EDGE_MAKER_NAME(resolveCoin3),
    EDGE_MAKER_NAME(split1),
    EDGE_MAKER_NAME(split2),
    EDGE_MAKER_NAME(split3),
    EDGE_MAKER_NAME(split4),
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
            result += STR((int)maker);
        else
            result += std::string(edgeMakerNames[(int)maker].name);
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
            result += STR((int)wz);
        else
            result += std::string(windZeroNames[(int)wz].name);
    }
    return result;
}

std::string OpEdge::debugDumpDetail() const {
    std::string s = "edge[" + STR(id) + "] segment[" + STR(segment->id) + "] contour["
            + STR(segment->contour->id) + "]\n";
    if (priorEdge || nextEdge || lastEdge) {
        s += "p/n/l:" + (priorEdge ? STR(priorEdge->id) : "-");
        s += "/" + (nextEdge ? STR(nextEdge->id) : "-");
        s += "/" + (lastEdge ? STR(lastEdge->id) : "-");
        s += " ";
    }
    if (priorSum)
        s += "priorSum:" + STR(priorSum->id) + " axis:" 
                + (Axis::horizontal == priorAxis ? "horizontal " : "vertical ");
    if (loopStart) {
        s += "loopStart:" + STR(loopStart->id) + " ";
        if (!isSumLoop)
            s += "\n!!! loopStart set EXPECTED isSumLoop ";
    }
    if (priorEdge || nextEdge || lastEdge || priorSum || loopStart)
        s += "\n";
    s += "{" + start.debugDump() + ", ";
    for (int i = 0; i < segment->c.pointCount() - 2; ++i)
        s += ctrlPts[i].debugDump() + ", ";
    s += end.debugDump() + "} ";
    if (1 != weight)
        s += "w:" + OpDebugDump(weight) + " ";
    s += "center:" + center.debugDump() + "\n";
    if (debugAliasStartID || !OpMath::IsNaN(debugOriginalStart.x) || !OpMath::IsNaN(debugOriginalStart.y))
        s += "(start was " + debugOriginalStart.debugDump() + "; now from sect " + STR(debugAliasStartID);
    if (debugAliasEndID || !OpMath::IsNaN(debugOriginalEnd.x) || !OpMath::IsNaN(debugOriginalEnd.y))
        s += " (end was " + debugOriginalEnd.debugDump() + "; now from sect " + STR(debugAliasEndID);
    if (debugAliasStartID || !OpMath::IsNaN(debugOriginalStart.x) || !OpMath::IsNaN(debugOriginalStart.y)
            || debugAliasEndID || !OpMath::IsNaN(debugOriginalEnd.x) || !OpMath::IsNaN(debugOriginalEnd.y))
        s += "\n";
    if (ptBounds.isSet())
        s += "pb:" + ptBounds.debugDump() + " ";
    if (linkBounds.isSet())
        s += "lb:" + linkBounds.debugDump();
    if (ptBounds.isSet() || linkBounds.isSet())        
        s += "\n";
    s += debugDumpWinding() + "\n";
    if (sum.isSet()) {
        s += "priorNormal:" + STR(priorNormal) + " ";
        s += "priorT:" + STR(priorT) + " ";
    }
    if (EdgeLink::unlinked != nextLink)
        s += "next:" + debugLinkedUp(nextLink) + " ";
    if (EdgeLink::unlinked != priorLink)
        s += "prior:" + debugLinkedUp(priorLink) + " ";
    if (EdgeMatch::none != whichEnd)
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != fail)
        s += "fail:" + debugEdgeFail(fail) + " ";
    s += "windZero:" + debugEdgeWindZero(windZero) + "\n";
    if (EdgeSplit::no != doSplit) s += "doSplit ";
    if (EdgeSplit::yes == doSplit) s += "yes ";
    if (EdgeSplit::unsplittable == doSplit) s+= "unsplittable ";
    if (curveSet) s += "curveSet ";
    if (endAliased) s += "endAliased ";
    if (lineSet) s += "lineSet ";
    if (verticalSet) s += "verticalSet ";
    if (isLine_impl) s += "isLine ";
    if (isPoint) s += "isPoint ";
    if (isSumLoop) s += "isSumLoop ";
    if (active_impl) s += "active ";
    if (startAliased) s += "startAliased ";
    if (unsortable) s += "unsortable ";
    s += "debugMaker:" + debugEdgeDebugMaker(debugMaker) + " ";
    if (debugParentID) s += "debugParentID:" + STR(debugParentID);
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
    segment = findSegment(segmentID);
//    assert(segment);
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
    assert(segment->id == test.segment->id);
    assert(start == test.start);
    assert(end == test.end);
}

std::string OpEdge::debugDump() const {
    std::string s;
    if (priorEdge || nextEdge) {
        s += "p/n:" + (priorEdge ? STR(priorEdge->id) : "-");
        s += "/" + (nextEdge ? STR(nextEdge->id) : "-");
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
    s += "wind:" + STR(winding.left) + "/" + STR(winding.right) + " ";
    if (OpMax != sum.left || OpMax != sum.right) {
        s += "l/r:" + (OpMax != sum.left ? STR(sum.left) : "--");
        s += "/" + (OpMax != sum.right ? STR(sum.right) : "--") + " ";
    }
    if (EdgeLink::unlinked != priorLink)
        s += "prior:" + debugLinkedUp(priorLink) + " ";
    if (EdgeLink::unlinked != nextLink)
        s += "next:" + debugLinkedUp(nextLink) + " ";
    if (EdgeMatch::none != whichEnd)
        s += "which:" + debugEdgeMatch(whichEnd) + " ";
    if (EdgeFail::none != fail)
        s += "fail:" + debugEdgeFail(fail) + " ";
    if (EdgeSplit::no != doSplit) s += "doSplit ";
    if (EdgeSplit::yes == doSplit) s += "yes ";
    if (EdgeSplit::unsplittable == doSplit) s+= "unsplittable ";
    if (isLine_impl) s += "isLine ";
    if (isPoint) s += "isPoint ";
    if (unsortable) s += "unsortable ";
    s += "seg:" + STR(segment->id);
    return s;
}

// !!! move to OpDebug.cpp
void OpEdge::debugValidate() const {
    for (auto& edge : segment->edges) {
        if (&edge == this)
            return;
    }
    assert(0);
}

// keep this in sync with op edge : is loop
std::string OpEdge::debugDumpChain(WhichLoop which, EdgeLoop edgeLoop, bool detail) const {
    assert(WhichLoop::prior == which || EdgeLoop::link == edgeLoop);
    std::string s = "chain:";
    const OpEdge* looped = isLoop(which, edgeLoop, LeadingLoop::in);
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
        chain = WhichLoop::prior == which ? chain->priorChain(edgeLoop) : chain->nextChain(edgeLoop);
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
    s += " sum: " + sum.debugDump();
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

void OpEdge::dumpChain(EdgeLoop edgeLoop, bool detail) const {
    std::string s;
    if (EdgeLoop::link == edgeLoop)
        s += "prior: ";
    s += debugDumpChain(WhichLoop::prior, edgeLoop, detail) + "\n";
    OpDebugOut(s);
    if (EdgeLoop::sum == edgeLoop)
        return;
    s = "next: " + debugDumpChain(WhichLoop::next, edgeLoop, detail) + "\n";
    OpDebugOut(s);
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
    dumpChain(EdgeLoop::link);
}

void OpEdge::dumpLinkDetail() const {
    dumpChain(EdgeLoop::link, true);
}

void OpEdge::dumpStart() const {
    segment->contour->contours->dumpMatch(start.pt);
}

void OpEdge::dumpSum() const {
    dumpChain(EdgeLoop::sum);
}

void OpEdge::dumpSumDetail() const {
    dumpChain(EdgeLoop::sum, true);
}

void OpEdge::dumpWinding() const {
    dumpDetail();
}

DUMP_STRUCT_DEFINITIONS(OpEdge)
DEBUG_DUMP_ID_DEFINITION(OpEdge, id)

void OpEdges::debugValidate() const {
    for (auto& edge : inX)
        edge->debugValidate();
    for (auto& edge : inY)
        edge->debugValidate();
}

void OpEdges::dumpAxis(Axis axis) const {
    std::string s = "";
    for (const auto edge : Axis::vertical == axis ? inY : inX) {
        s += edge->debugDump() + "\n";
    }
    OpDebugOut(s);
}

void OpEdges::dump() const {
    if (inX.size()) {
        OpDebugOut("-- sorted in x --\n");
        dumpAxis(Axis::horizontal);
    }
    if (inY.size()) {
        OpDebugOut("-- sorted in y --\n");
        dumpAxis(Axis::vertical);
    }
}

DUMP_STRUCT_DEFINITIONS(OpEdges)

DUMP_STRUCT_DEFINITIONS(OpEdgeIntersect)

void dump(const std::vector <OpEdge>& edges) {
    for (auto& edge : edges) {
        OpDebugOut(edge.debugDump() + "\n");
    }
}

void dumpDetail(const std::vector <OpEdge>& edges) {
    for (auto& edge : edges) {
        OpDebugOut(edge.debugDumpDetail() + "\n");
    }
}

const OpEdgeIntersect* OpEdgeIntersect::debugActive;

void OpEdgeIntersect::dump() const {
    if (edgeParts.size()) {
        int splitCount = 0;
        for (auto& edge : edgeParts)
            splitCount += EdgeSplit::yes == edge.doSplit;
        OpDebugOut("-- edge parts split: " + STR(splitCount) + " --\n");
        ::dump(edgeParts);
    }
    if (oppParts.size()) {
        int splitCount = 0;
        for (auto& edge : oppParts)
            splitCount += EdgeSplit::yes == edge.doSplit;
        OpDebugOut("-- opp parts split: " + STR(splitCount) + " --\n");
        ::dump(oppParts);
    }
}

void OpEdgeIntersect::dumpDetail() const {
    if (edgeParts.size()) {
        int splitCount = 0;
        for (auto& edge : edgeParts)
            splitCount += EdgeSplit::yes == edge.doSplit;
        OpDebugOut("-- edge parts split: " + STR(splitCount) + " --\n");
        ::dumpDetail(edgeParts);
    }
    if (oppParts.size()) {
        int splitCount = 0;
        for (auto& edge : oppParts)
            splitCount += EdgeSplit::yes == edge.doSplit;
        OpDebugOut("-- opp parts split: " + STR(splitCount) + " --\n");
        ::dumpDetail(oppParts);
    }
}

struct IntersectMakerName {
    IntersectMaker maker;
    const char* name;
};

#define INTERSECT_MAKER_NAME(r) { IntersectMaker::r, #r }

IntersectMakerName intersectMakerNames[] {
    INTERSECT_MAKER_NAME(addCoincidentCheck1),
    INTERSECT_MAKER_NAME(addCoincidentCheck2),
    INTERSECT_MAKER_NAME(addCurveCoincidence1),
	INTERSECT_MAKER_NAME(addCurveCoincidence2),
	INTERSECT_MAKER_NAME(addCurveCoincidence3),
	INTERSECT_MAKER_NAME(addCurveCoincidence4),
	INTERSECT_MAKER_NAME(addIntersection1),
	INTERSECT_MAKER_NAME(addIntersection2),
	INTERSECT_MAKER_NAME(addIntersection3),
	INTERSECT_MAKER_NAME(addIntersection4),
	INTERSECT_MAKER_NAME(addIntersection5),
	INTERSECT_MAKER_NAME(addIntersection6),
	INTERSECT_MAKER_NAME(addIntersection7),
	INTERSECT_MAKER_NAME(addIntersection8),
	INTERSECT_MAKER_NAME(addMatchingEnds1),
	INTERSECT_MAKER_NAME(addMatchingEnds2),
	INTERSECT_MAKER_NAME(addMatchingEnds3),
	INTERSECT_MAKER_NAME(addMatchingEnds4),
	INTERSECT_MAKER_NAME(addMix1),
	INTERSECT_MAKER_NAME(addMix2),
	INTERSECT_MAKER_NAME(addPair1),
	INTERSECT_MAKER_NAME(addPair2),
	INTERSECT_MAKER_NAME(addPair3),
	INTERSECT_MAKER_NAME(addPair4),
	INTERSECT_MAKER_NAME(coincidentCheck1),
	INTERSECT_MAKER_NAME(coincidentCheck2),
	INTERSECT_MAKER_NAME(curveCenter1),
	INTERSECT_MAKER_NAME(curveCenter2),
	INTERSECT_MAKER_NAME(findIntersections1),
	INTERSECT_MAKER_NAME(findIntersections2),
	INTERSECT_MAKER_NAME(findIntersections3),
	INTERSECT_MAKER_NAME(findIntersections4),
	INTERSECT_MAKER_NAME(findIntersections5),
	INTERSECT_MAKER_NAME(findIntersections6),
	INTERSECT_MAKER_NAME(findIntersections7),
	INTERSECT_MAKER_NAME(findIntersections8),
	INTERSECT_MAKER_NAME(makeEdges),
	INTERSECT_MAKER_NAME(missingCoincidence1),
	INTERSECT_MAKER_NAME(missingCoincidence2),
	INTERSECT_MAKER_NAME(opCubicErrorTest1),
	INTERSECT_MAKER_NAME(opCubicErrorTest2),
	INTERSECT_MAKER_NAME(opTestEdgeZero1),
	INTERSECT_MAKER_NAME(opTestEdgeZero2),
	INTERSECT_MAKER_NAME(opTestEdgeZero3),
	INTERSECT_MAKER_NAME(opTestEdgeZero4),
	INTERSECT_MAKER_NAME(splitAtWinding),
};

struct SelfIntersectName {
    SelfIntersect self;
    const char* name;
};

#define SELF_INTERSECT_NAME(r) { SelfIntersect::r, #r }

SelfIntersectName selfIntersectNames[] {
    SELF_INTERSECT_NAME(none),
    SELF_INTERSECT_NAME(self),
    SELF_INTERSECT_NAME(missing),
    SELF_INTERSECT_NAME(split),
};

std::string OpIntersection::debugDump(bool fromDumpFull) const {
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(intersectMakerNames); ++index)
       if (!outOfDate && (unsigned) intersectMakerNames[index].maker != index) {
           OpDebugOut("intersectMakerNames out of date\n");
           outOfDate = true;
           break;
       }
    std::string s;
    std::string segmentID = segment ? segment->debugDumpID() : "--";
    const OpSegment* oppParent = opp ? opp->segment : nullptr;
    std::string oppID = opp ? opp->debugDumpID() : "--";
    std::string oppParentID = oppParent ? oppParent->debugDumpID() : "--";
    s = "[" + debugDumpID() + "] " + ptT.debugDump();
    if (debugAliasID || !OpMath::IsNaN(debugOriginal.x) || !OpMath::IsNaN(debugOriginal.y))
        s += " (was " + debugOriginal.debugDump() + "; now from sect " + STR(debugAliasID) + ")";
    if (!fromDumpFull || !segment)
        s += " segment:" + segmentID;
    s += " opp/sect:" + oppParentID + "/" + oppID;
    if (coincidenceID || debugCoincidenceID)
        s += " coinID:" + STR(coincidenceID) + "/" + STR(debugCoincidenceID);
    if (SelfIntersect::none != self)
        s += " self:" + std::string(selfIntersectNames[(int)self].name);
//    if (unsortable)
//        s += " unsortable";
    s += " maker:";
    if (outOfDate)
        s += "(out of date) " + STR((int)debugMaker);
    else
        s += intersectMakerNames[(int)debugMaker].name;
    return s;
}

std::string OpIntersection::debugDumpBrief() const {
    std::string s;
    s += "[" + debugDumpID() + "] ";
    s += "{" + ptT.debugDump() + ", ";
    s += "seg:" + segment->debugDumpID();
    return s;
}

std::string OpIntersection::debugDumpHex() const {
    std::string s;
    s = "/* sect:" + debugDumpID() + " */ OpPtT data[] {\n";
    s += "  " + ptT.debugDumpHex() + " // " + ptT.debugDump() + "\n";
    s += "}; // seg:" + segment->debugDumpID() + "\n";
    return s;
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
    segment = const_cast<OpSegment*>(findSegment(segmentID));
    assert(segment);
    // don't call comnplete because we don't want to advance debug id
}

void OpIntersection::debugCompare(std::string s) const {
    OpIntersection test(s);
    assert(segment->id == test.segment->id);
    assert(ptT == test.ptT);
}

void OpIntersection::dump() const {
    std::string s = debugDump(false) + "\n";
    OpDebugOut(s);
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
    std::string s = debugDump() + "\n";
    s += winding.debugDump() + "\n";
    s += "pb:" + ptBounds.debugDump() + " ";
    s += "tb:" + tightBounds.debugDump() + "\n";
    s += "contour:" + STR(contour->id);
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
        s += i->debugDump(true) + "\n";
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

struct DebugReasonName {
    ZeroReason reason;
    const char* name;
};

#define REASON_NAME(r) { ZeroReason::r, #r }

static DebugReasonName debugReasonNames[] {
    REASON_NAME(none),
    REASON_NAME(addIntersection),
    REASON_NAME(applyOp),
    REASON_NAME(centerNaN),
    REASON_NAME(coincidence),
    REASON_NAME(collapsed),
    REASON_NAME(failCenter),
    REASON_NAME(failNext),
    REASON_NAME(failPrior),
    REASON_NAME(findCoincidences),
    REASON_NAME(isPoint),
    REASON_NAME(linkUp),
    REASON_NAME(looped),
    REASON_NAME(loopyPair),
    REASON_NAME(matchClosest),
    REASON_NAME(matchLink),
    REASON_NAME(noFlip),
    REASON_NAME(noNormal),
    REASON_NAME(rayNormal),
    REASON_NAME(recalcCenter),
    REASON_NAME(resolveCoin),
    REASON_NAME(tangentXRay)
};

std::string OpWinding::debugDump() const {
    bool outOfDate = false;
    for (unsigned index = 0; index < ARRAY_COUNT(debugReasonNames); ++index)
       if (!outOfDate && (unsigned) debugReasonNames[index].reason != index) {
           OpDebugOut("debugReasonNames out of date\n");
           outOfDate = true;
       }
    std::string result = "left: " + STR(left) + " right: " + STR(right);
    if (debugSetter)
        result += " setter: " + STR(debugSetter);
    if (ZeroReason::none != debugReason) {
        result += " reason: ";
        if (outOfDate)
            result += STR((int)debugReason);
        else
            result += std::string(debugReasonNames[(int)debugReason].name);
    }
    return result;
}

void OpWinding::dump() const {
    OpDebugOut(debugDump() + "\n");
}

void DumpLinkups(const std::vector<OpEdge*>& linkups) {
    std::vector<int> inactive;
    for (const auto& linkup : linkups) {
        if (!linkup->isActive()) {
            inactive.push_back(linkup->id);
            continue;
        }
        int count = 0;
        auto next = linkup;
        auto looped = linkup->isLoop(WhichLoop::prior, EdgeLoop::link, LeadingLoop::in);
        if (!looped)
            looped = linkup->isLoop(WhichLoop::next, EdgeLoop::link, LeadingLoop::in);
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
        assert(!looped || count == priorCount);
        if (looped)
            priorCount = 0;
        std::string str = "linkup count: " + STR(count + priorCount);
        if (priorCount && !looped)
            str += " (prior count: " + STR(priorCount) + ")";
        if (looped)
            str += " loop";
        OpDebugOut(str + "\n");
        if (!looped) {
            if (1 == count + priorCount)
                OpDebugOut("p/n:-/- ");
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
                assert(!next->nextEdge);
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
    if (inactive.size()) {
        std::string s = "(inactive ";
        for (int i : inactive)
            s += STR(i) + " ";
        OpDebugOut(s + ")\n");
    }
}

void dump(const std::vector<OpEdge*>& linkups) {
    DumpLinkups(linkups);
}

void dump(const std::vector<const OpEdge*>& edges) {
    for (auto& edge : edges) {
        OpDebugOut(edge->debugDump() + "\n");
    }
}

void DumpFound(const std::vector<FoundEdge>& found) {
    for (auto foundOne : found) {
        OpDebugOut(debugEdgeMatch(foundOne.whichEnd) + " " + foundOne.edge->debugDump() + "\n");
    }
}

void dump(const std::vector<FoundEdge>& found) {
    DumpFound(found);
}

void dump(const std::vector<OpSegment*>& found) {
    std::string s = "";
    for (const auto seg : found) {
        s += seg->debugDump() + "\n";
    }
    OpDebugOut(s);
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

void OpSegments::dump() const {
    std::string s = "";
    for (const auto seg : inX) {
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

#if PATH_OPS_V0_FOR_SKIA == PATH_OPS_V0_FOR_SKIA

#include "include/core/SkPath.h"

void OpOutPath::dump() const {
    skPath->dump();
}

void dump(const OpOutPath& path) {
    path.dump();
}

#endif

#endif
