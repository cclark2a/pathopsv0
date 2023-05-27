#include "OpDebug.h"

#if OP_DEBUG
OpContours* debugGlobalContours;
bool OpDebugPathOpsEnable::inPathOps;
OpDebugIntersect debugGlobalIntersect;
#endif

#if OP_DEBUG || OP_RELEASE_TEST
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif

void OpPrintOut(const std::string& s) {
#ifdef _WIN32
    OutputDebugStringA(s.c_str());
    FILE* out = fopen("out.txt", "a+");
    fprintf(out, "%s", s.c_str());
    fclose(out);
#else
    printf(stderr, s.c_str());
#endif
}

uint64_t OpInitTimer() {
#ifdef _WIN32
    LARGE_INTEGER frequency;
    QueryPerformanceFrequency(&frequency);
    return frequency.QuadPart;
#else
#error "unimplmented"
#endif
}

uint64_t OpReadTimer() {
#ifdef _WIN32
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);
    return time.QuadPart;
#else
#error "unimplmented"
#endif
}

float OpTicksToSeconds(uint64_t diff, uint64_t frequency) {
#ifdef _WIN32
    return (float) (diff * 1000000 / frequency) / 1000000;
#else
#error "unimplmented"
#endif
}

#endif

#if OP_DEBUG

void OpDebugOut(const std::string& s) {
#ifdef _WIN32
    OutputDebugStringA(s.c_str());
#else
    printf(stderr, s.c_str());
#endif
}

#include "OpCurve.h"

union FloatIntUnion {
    float   f;
    int32_t i;
};

float OpDebugBitsToFloat(int32_t i) {
    FloatIntUnion d;
    d.i = i;
    return d.f;
}

std::string OpDebugDump(float f) {
    if (OpMath::IsNaN(f))
        return "NaN";
    if (!OpMath::IsFinite(f))
        return f > 0 ? "Inf" : "-Inf";
    char buffer[20];
    int precision = 8;
    float copyF = f;
    while (copyF >= 10) {
        if (0 == --precision)
            break;
        copyF /= 10;
    }
    int size = snprintf(buffer, sizeof(buffer), "%.*g", precision, f);
    std::string s(buffer, size);
//    while ('0' == s.back())
//        s.pop_back();
//    if ('.' == s.back())
//        s.pop_back();
    if ("1" == s && 1 != f)
        s = "~" + s;
    else if ("0" == s && 0 != f)
        s = "~" + s;
    return s;
}

std::string OpDebugDumpHex(float f) {
    std::string s = "0x";
    int32_t hex = OpDebugFloatToBits(f);
    for (int index = 28; index >= 0; index -= 4) {
        int nybble = (hex >> index) & 0xF;
        if (nybble <= 9)
            s += '0' + nybble;
        else 
            s += 'a' + nybble - 10;
    }
    return s;
}

std::string OpDebugDumpHexToFloat(float f) {
    return "OpDebugBitsToFloat(" + OpDebugDumpHex(f) + ")";    
}

int32_t OpDebugFloatToBits(float f) {
    FloatIntUnion d;
    d.f = f;
    return d.i;
}

float OpDebugHexToFloat(const char*& str) {
    FloatIntUnion d;
    d.i = OpDebugHexToInt(str);
    return d.f;
}

int32_t OpDebugHexToInt(const char*& str) {
    int32_t result = 0;
    OpDebugSkip(str, "0x");
    for (int index = 0; index < 8; ++index) {
        char c = *str++;
        int nybble = c - '0';
        if (nybble > 9)
            nybble = c - 'a' + 10;
        result = (result << 4) | nybble;
}
    return result;
}

void OpDebugSkip(const char*& str, const char* match) {
    size_t matchLen = strlen(match);
    size_t strLen = strlen(str);
    while (strLen) {
        if (!strncmp(match, str, matchLen)) {
            break;
        }
        str += 1;
        --strLen;
    }
    assert(strlen(str));
}

std::string OpDebugToString(float value, int precision) {
    if (precision < 0)
        return STR(value);
    std::string s(16, '\0');
    auto written = std::snprintf(&s[0], s.size(), "%.*f", precision, value);
    s.resize(written);
    return s;
}

void OpMath::DebugCompare(float a, float b) {
    float diff = fabsf(a - b);
    float max = std::max(fabsf(a), fabsf(b));
    float precision = diff / max;
    assert(precision < OpEpsilon);
}

OpVector OpCurve::debugTangent(float t) const {
    switch (type) {
    case pointType: return OpVector();
    case lineType: return asLine().tangent(t);
    case quadType: return asQuad().debugTangent(t);
    case conicType: return asConic().debugTangent(t);
    case cubicType: return asCubic().debugTangent(t);
    default:
        assert(0);
    }
    return OpVector();
}

OpVector OpQuad::debugTangent(float t) const {
    OpVector result = tangent(t);
    if ((0 == t && pts[0] == pts[1]) || (1 == t && pts[2] == pts[1]))
        return pts[2] - pts[0];
    return result;
}

OpVector OpConic::debugTangent(float t) const {
    return ((OpQuad*) this)->debugTangent(t);
}

OpVector OpCubic::debugTangent(float t) const {
    OpVector result = tangent(t);
    if (0 == t && pts[0] == pts[1]) {
        if (pts[1] == pts[2])
            return pts[3] - pts[0];
        else
            return pts[2] - pts[0];
    }
    if (1 == t && pts[3] == pts[2]) {
        if (pts[2] == pts[1])
            return pts[3] - pts[0];
        else
            return pts[3] - pts[1];
    }
    return result;
}

#endif

#if OP_DEBUG
template <typename V, typename... T>   // replace with std::to_array in c++20
constexpr auto to_array(T&&... t)->std::array < V, sizeof...(T) > {
    return { { std::forward<T>(t)... } };
}

#include "OpContour.h"
#include "OpSegment.h"

void OpIntersection::debugSetID() {
    id = segment->contour->contours->id++;
if (68 == id)
OpDebugOut("");
#if 1
    auto match = to_array<int>(68);  // c++20: std::to_array<int>({... (brace)
    if (match.end() != std::find(match.begin(), match.end(), id))
        OpDebugOut("");
#endif
}

void OpIntersection::debugValidate() const {
    assert(OpMath::Between(0, ptT.t, 1));
    OpPoint pt = segment->c.ptAtT(ptT.t);
    OpMath::DebugCompare(pt, ptT.pt);
    OpPoint oPt = opp->segment->c.ptAtT(opp->ptT.t);
    OpMath::DebugCompare(pt, oPt);
}
#endif

#if OP_DEBUG_EDGE_INTERSECT
#include "OpEdgeIntersect.h"
#include "OpSegment.h"

struct OpDebugResult {
    OpDebugResult(const OpEdge* edge_, const OpEdge* opp_, OpPtT ptT_, OpPtT oppPtT_, float dxy_)
        : edge(edge_)
        , opp(opp_)
        , ptT(ptT_)
        , oppPtT(oppPtT_)
        , dxy(dxy_) {
    }
    const OpEdge* edge;
    const OpEdge* opp;
    OpPtT ptT;
    OpPtT oppPtT;
    float dxy;
};

struct OpDebugFlip {
    OpDebugFlip(const OpDebugResult* last_, const OpDebugResult* next_, float xyBias_, float estEdgeT_,
            float estOppT_)
        : last(last_)
        , next(next_)
        , xyBias(xyBias_)
        , estEdgeT(estEdgeT_)
        , estOppT(estOppT_) {
    }
    const OpDebugResult* last;
    const OpDebugResult* next;
    float xyBias;
    float estEdgeT;
    float estOppT;
};

void OpEdgeIntersect::debugFindEdgeCrossings() {
    std::vector<OpDebugResult> hResults, vResults;
    for (const auto& edge : edgeParts) {
        OpEdge copy(edge);
        const OpCurve& edgeCurve = copy.setCurve();
	    const OpRect& ptBounds = copy.ptBounds;
        OpPtT center;
        for (Axis axis : { Axis::vertical, Axis::horizontal }) {
            Axis perpendicular = !axis;
		    float middle = (ptBounds.ltChoice(axis) + ptBounds.rbChoice(axis)) / 2;
            float t = edgeCurve.center(axis, middle);   // edge t
            if (OpMath::IsNaN(t))
                continue;
            center.t = OpMath::Interp(edge.start.t, edge.end.t, t); // segment t
            center.pt = edgeCurve.ptAtT(t);
	//	    center.pt.pin(ptBounds);
            for (const auto& opp : oppParts) {
                if (opp.ptBounds.ltChoice(axis) > middle || middle > opp.ptBounds.rbChoice(axis))
                    continue;
                OpEdge oppCopy(opp);
                rootCellar cepts;
                const OpCurve& oppCurve = oppCopy.setCurve();
                int roots = oppCurve.axisRayHit(axis, middle, cepts);
                assert(0 == roots || 1 == roots);
                for (int index = 0; index < roots; ++index) {
                    float oppT = OpMath::Interp(opp.start.t, opp.end.t, cepts[index]); // segment t
                    OpPtT oppPtT { oppCurve.ptAtT(cepts[index]), oppT };
    //              oppPtT.pt.pin(opp.ptBounds);
                    float distance = oppPtT.pt.choice(perpendicular) - center.pt.choice(perpendicular);
                    (Axis::vertical == axis ? vResults : hResults).
                            emplace_back(&edge, &opp, center, oppPtT, distance);
                }
            }
        }
    }
    // Check both axes for sign reversal on dxy. Dump estimated segment t value of reversal.
    std::vector<OpDebugFlip> hFlips, vFlips;
    auto findFlips = [](const std::vector<OpDebugResult>& results, std::vector<OpDebugFlip>& flips) {
        const OpDebugResult* last = nullptr;
        for (const auto& result : results) {
            if (last && last->dxy * result.dxy < 0) {
                float xyBias = last->dxy / fabsf(last->dxy - result.dxy);
                float absBias = fabsf(xyBias);
                float estEdgeT = result.ptT.t * absBias + last->ptT.t * (1 - absBias);
                float estOppT = result.oppPtT.t * absBias + last->oppPtT.t * (1 - absBias);
                flips.emplace_back(last, &result, xyBias, estEdgeT, estOppT);
            }
            last = &result;
        }
    };
    findFlips(hResults, hFlips);
    findFlips(vResults, vFlips);
    auto dumpFlip = [](const OpDebugFlip& flip) {
        const OpEdge* lastE = flip.last->edge;
        bool inLastE = OpMath::Between(lastE->start.t, flip.estEdgeT, lastE->end.t);
        const OpEdge* lastO = flip.last->opp;
        bool inLastO = OpMath::Between(lastO->start.t, flip.estOppT, lastO->end.t);
        OpDebugOut("edge pair[" + STR(flip.last->edge->id) + ", " + STR(flip.next->edge->id) + "]"
            + " est edge[" + STR(inLastE ? lastE->id : flip.next->edge->id) + "] T:" + STR(flip.estEdgeT)
            + " opp pair[" + STR(lastO->id) + ", " + STR(flip.next->opp->id) + "]"
            + " est opp[" + STR(inLastO ? lastO->id : flip.next->opp->id) + "] T:" + STR(flip.estOppT) + "\n");
    };
    if (hFlips.size() || vFlips.size())
        OpDebugOut("for segs[" + STR(originalEdge->segment->id) + ", " 
                + STR(originalOpp->segment->id) + "] :\n");
    if (hFlips.size()) 
        OpDebugOut("horizontal flips\n");
    for (const auto& flip: hFlips)
        dumpFlip(flip);
    if (vFlips.size())
        OpDebugOut("vertical flips\n");
    for (const auto& flip: vFlips)
        dumpFlip(flip);
    auto dumpResult = [](const OpDebugResult& result) {
        OpDebugOut("[" + STR(result.edge->id) + "] ptT:" + result.ptT.debugDump() 
                + " opp[" + STR(result.opp->id) + "] ptT:" + result.oppPtT.debugDump() 
                + " dxy:" + STR(result.dxy) + "\n");
    };
    if (hResults.size())
        OpDebugOut("horizontal results:\n");
    for (const auto& result: hResults)
        dumpResult(result);
    if (vResults.size())
        OpDebugOut("vertical results:\n");
    for (const auto& result: vResults)
        dumpResult(result);
    if (hResults.size() || vResults.size())
        OpDebugOut("\n");
}
#endif
