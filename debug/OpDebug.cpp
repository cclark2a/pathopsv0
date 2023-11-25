// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if 0
// code pattern to find one of several id values
template <typename V, typename... T>   // replace with std::to_array in c++20
constexpr auto to_array(T&&... t)->std::array < V, sizeof...(T) > {
    return { { std::forward<T>(t)... } };
}

    auto match = to_array<int>(68);  // c++20: std::to_array<int>({... (brace)
    if (match.end() != std::find(match.begin(), match.end(), id))
        OP_ASSERT(0);
#endif

#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
OpContours* debugGlobalContours;
bool debugHexFloat = false;
#endif

#if OP_DEBUG || OP_RELEASE_TEST
#include <atomic>
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif

std::atomic_int testsWarn;
int debugPrecision = -1;		// minus one means unset

union FloatIntUnion {
    float   f;
    int32_t i;
};

float OpDebugBitsToFloat(int32_t i) {
    FloatIntUnion d;
    d.i = i;
    return d.f;
}

#if 0
void OpPrintOut(const std::string& s) {
#ifdef _WIN32
    OutputDebugStringA(s.c_str());
    FILE* out = fopen("out.txt", "a+");
    fprintf(out, "%s", s.c_str());
    fclose(out);
#else
    fprintf(stderr, "%s", s.c_str());
#endif
}
#endif

uint64_t OpInitTimer() {
#ifdef _WIN32
    LARGE_INTEGER frequency;
    QueryPerformanceFrequency(&frequency);
    return frequency.QuadPart;
#else
// #error "unimplmented"
    return 0;
#endif
}

uint64_t OpReadTimer() {
#ifdef _WIN32
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);
    return time.QuadPart;
#else
// #error "unimplmented"
    return 0;
#endif
}

float OpTicksToSeconds(uint64_t diff, uint64_t frequency) {
#ifdef _WIN32
    return (float) (diff * 1000000 / frequency) / 1000000;
#else
// #error "unimplmented"
    return 0;
#endif
}

#endif

#if !defined(NDEBUG) || OP_RELEASE_TEST
void OpDebugOut(const std::string& s) {
#ifdef _WIN32
    OutputDebugStringA(s.c_str());
#else
    fprintf(stderr, "%s", s.c_str());
#endif
}
#endif

#if OP_DEBUG || OP_DEBUG_DUMP || OP_DEBUG_IMAGE

#include "OpCurve.h"

std::string OpDebugDump(float f) {
    if (OpMath::IsNaN(f))
        return "NaN";
    if (!OpMath::IsFinite(f))
        return f > 0 ? "Inf" : "-Inf";
    char buffer[20];
    int pPrecision = 8;
    float copyF = f;
    while (copyF >= 10) {
        if (0 == --pPrecision)
            break;
        copyF /= 10;
    }
    int size = snprintf(buffer, sizeof(buffer), "%.*g", pPrecision, f);
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

#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE
std::string OpDebugDumpHex(float f) {
    if (!debugHexFloat) {
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
    char buffer[256];
    int bytes = snprintf(buffer, sizeof(buffer), "%af", f);
    return std::string(buffer, bytes);
}
#endif

int32_t OpDebugFloatToBits(float f) {
    FloatIntUnion d;
    d.f = f;
    return d.i;
}

float OpDebugHexToFloat(const char*& str) {
    // !!! add support for hex float %a format
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
    OP_ASSERT(strlen(str));
}

std::string OpDebugToString(float value) {
    std::string result;
    if (debugPrecision < 0)
        result = std::to_string(value);
    else {
        std::string s(16, '\0');
        auto written = std::snprintf(&s[0], s.size(), "%.*f", debugPrecision, value);
        s.resize(written);
        result = s;
    }
    // !!! try trimming trailing zeroes to see what that's like
    size_t pos = result.find('.');
    if (std::string::npos == pos)
        return result;
    while (result.back() == '0')
        result.pop_back();
    if (result.back() == '.')
        result.pop_back();
    return result;
}

#endif

#if OP_DEBUG

void OpMath::DebugCompare(float a, float b) {
    float diff = fabsf(a - b);
    float max = std::max(fabsf(a), fabsf(b));
    float pPrecision = diff / max;
    OP_ASSERT(pPrecision < OpEpsilon);
}

OpVector OpCurve::debugTangent(float t) const {
    switch (type) {
    case OpType::line: return asLine().tangent();
    case OpType::quad: return asQuad().debugTangent(t);
    case OpType::conic: return asConic().debugTangent(t);
    case OpType::cubic: return asCubic().debugTangent(t);
    default:
        OP_ASSERT(0);
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

#include "OpContour.h"
#include "OpEdge.h"

void OpContours::addDebugWarning(OpDebugWarning warn) {
    debugWarnings.push_back(warn);
    testsWarn++;
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

#if OP_DEBUG_VALIDATE
void OpEdge::debugValidate() const {
    OpContours* contours = this->contours();
    contours->debugValidateEdgeIndex += 1;
    bool loopy = debugIsLoop();
    if (loopy) {
        const OpEdge* test = this;
        do {
            OP_ASSERT(!test->priorEdge || test->priorEdge->nextEdge == test);
            OP_ASSERT(!test->nextEdge || test->nextEdge->priorEdge == test);
//            OP_ASSERT(!test->lastEdge);
            test = test->nextEdge;
        } while (test != this);
    } else if ((priorEdge || lastEdge) && contours->debugCheckLastEdge) {
        const OpEdge* linkStart = debugAdvanceToEnd(EdgeMatch::start);
        const OpEdge* linkEnd = debugAdvanceToEnd(EdgeMatch::end);
        OP_ASSERT(linkStart);
        OP_ASSERT(linkEnd);
        OP_ASSERT(contours->debugCheckLastEdge ? !!linkStart->lastEdge : !linkStart->lastEdge);
        OP_ASSERT(contours->debugCheckLastEdge ? linkStart->lastEdge == linkEnd : true);
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
#endif

int OpEdge::debugLinkCount() const {
    int count = 1;
    std::vector<const OpEdge*> visited;
    visited.push_back(this);
    auto visit = [&visited, &count](const OpEdge* test) {
        if (visited.end() != std::find(visited.begin(), visited.end(), test))
            return;
        visited.push_back(test);
        count += 1;
    };
    const OpEdge* test = this;
    while ((test = test->priorEdge)) {
        visit(test);
    }
    test = this;
    while ((test = test->nextEdge)) {
        visit(test);
    }
    return count;
}

bool OpEdge::debugSuccess() const {
    return segment->debugSuccess();
}

// fuzz-generated test crbug_526025 generated an edge link that is invalid.
// this is currently unused
#if 0
bool OpEdge::debugValidLoop() const {
	std::vector<const OpEdge*> seen;
	const OpEdge* last = this;
	for (;;) {
		if (seen.end() != std::find(seen.begin(), seen.end(), last)) {
			return true;
		}
		seen.push_back(last);
		if (!last->nextEdge)
			return true;
		if (last != last->nextEdge->priorEdge) {
			OP_ASSERT(0);
			return false;
		}
		last = last->nextEdge;
	}
}
#endif

#include "OpJoiner.h"

void OpContours::debugRemap(int oldRayMatch, int newRayMatch) {
    for (auto& contour : contours) {
        for (auto& segment : contour.segments) {
            for (auto& edge : segment.edges) {
                if (oldRayMatch == edge.debugRayMatch)
                    edge.debugRayMatch = newRayMatch;
            }
        }
    }
}

// assign the same ID for all edges linked together
// also assign that ID to edges whose non-zero crossing rays attach to those edges
void OpJoiner::debugMatchRay(OP_DEBUG_CODE(OpContours* contours)) {
    OP_DEBUG_CODE(bool mayFail = OpDebugExpect::unknown == contours->debugExpect);
	for (auto linkup : linkups.l) {
        OP_ASSERT(!linkup->priorEdge);
        OP_ASSERT(linkup->lastEdge);
        OpEdge* firstLink = linkup;
        // search ray list for possible nonzero ray connected edge pair
        do {
            if (!linkup->ray.distances.size())
                continue;
            if (linkup->unsortable)
                continue;
            if (linkup->disabled)
                continue;
            const EdgeDistance* linkDist = nullptr;
            OpEdge* dTest = nullptr;
            OP_DEBUG_CODE(const EdgeDistance* dDist = nullptr);
            for (const EdgeDistance* dist = &linkup->ray.distances.back(); 
                    dist >= &linkup->ray.distances.front(); --dist) {
                OpEdge* test = dist->edge;
                if (test == linkup) {
                    linkDist = dist;
                    continue;
                }
                if (!linkDist)
                    continue;
                if (linkup->isPal(test))
                    continue;
                if (test->disabled)
                    continue;
                OP_DEBUG_CODE(dDist = dist);
                dTest = test;
                break;
            }
            // look to see if edge maps a non-zero ray to a prior edge
            WindZero linkZero = linkup->windZero;
            OP_ASSERT(WindZero::noFlip != linkZero);
	        NormalDirection NdotR = linkup->normalDirection(-linkup->ray.axis, linkDist->t);
            if (NormalDirection::downLeft == NdotR)
                WindZeroFlip(&linkZero);    // get wind zero for edge normal pointing left
            if (WindZero::opp == linkZero) {
#if OP_DEBUG
                if (dTest && !dTest->inLinkups && !dTest->inOutput) {
                    bool foundOne = false;
                    for (auto& pal : dTest->pals)
                        foundOne |= pal.edge->inLinkups;
                    for (auto& us : unsectByArea)
                        foundOne |= dTest == us && dTest->isActive();
                    for (auto& u : unsortables)
                        foundOne |= dTest == u && dTest->isActive();
                    OP_ASSERT(foundOne || mayFail);
                }
#endif
                if (dTest)
                    linkup->debugMatch = dTest;
// if verbose...
//                else
//                    OpDebugOut("wind zero missing: edge " + STR(linkup->id) + "\n");
            }
#if OP_DEBUG
            if (!dTest)
                continue;
            WindZero distZero = dTest->windZero;
            NdotR = dTest->normalDirection(linkup->ray.axis, dDist->t);
            if (NormalDirection::downLeft == NdotR)
                WindZeroFlip(&distZero);    // get wind zero for prior normal pointing right
                // either neither zero should be opp, or both should be 
            // !!! for now, just track when this happens. Wait until we can't ignore it to fix
            if ((WindZero::opp == distZero) != (WindZero::opp == linkZero)) {
                if (!dTest->inOutput) {
                    dTest->debugZeroErr = linkup;
// if verbose...
//                    OpDebugOut("wind zero mismatch: edges " + STR(dTest->id) + ", " 
//                            + STR(linkup->id) + "\n");
                }
                if (!linkup->inOutput) {
                    linkup->debugZeroErr = dTest;
// if verbose...
//                    OpDebugOut("wind zero mismatch: edges " + STR(linkup->id) + ", " 
//                            + STR(dTest->id) + "\n");
                }
            }
#endif
        } while ((linkup = linkup->nextEdge));
        // search links and ray matches for an ID
        linkup = firstLink;
        int nextID = 0;
        do {
            if ((nextID = linkup->debugRayMatch))
                break;
            // don't recurse: just check one level below
            const OpEdge* match = linkup->debugMatch;
            if (!match)
                continue;
            if ((nextID = match->debugRayMatch))
                break;
        } while ((linkup = linkup->nextEdge));
        if (!nextID)
            nextID = firstLink->segment->nextID();
        // apply ID to all links
        linkup = firstLink;
        do {
            OP_ASSERT(!linkup->debugRayMatch || nextID == linkup->debugRayMatch);
            linkup->debugRayMatch = nextID;
        } while ((linkup = linkup->nextEdge));
        // apply ID to all ray-matched links
        linkup = firstLink;
        do {
            OpEdge* match = linkup->debugMatch;
            if (!match)
                continue;
            if (match->debugRayMatch && nextID != match->debugRayMatch)
                // remap everything with old match to next id (don't know how to do this efficiently
                contours->debugRemap(match->debugRayMatch, nextID);
            if (match->debugRayMatch)   // !!! could assert that all linked edges are ID'd
                continue;
            if (match->debugIsLoop()) {
                OP_ASSERT(mayFail || match->inOutput);
                continue;
            }
            OpEdge* firstMatch = match;
            do {
                OP_ASSERT(!match->debugRayMatch);
                match->debugRayMatch = nextID;
            } while ((match = match->nextEdge));
            match = firstMatch;
            while ((match = match->priorEdge)) {
                OP_ASSERT(!match->debugRayMatch);
                match->debugRayMatch = nextID;
            }
        } while ((linkup = linkup->nextEdge));
    }
}

#if OP_DEBUG_VALIDATE
// !!! also debug prev/next edges (links)
void OpJoiner::debugValidate() const {
    OpEdge* anEdge = byArea.size() ? byArea[0] : unsectByArea.size() ? unsectByArea[0] : 
            disabled.size() ? disabled[0] : unsortables.size() ? unsortables[0] :
            linkups.l.size() ? linkups.l[0] : nullptr;
    if (!anEdge)
        return;
    OpContours* contours = anEdge->contours();
    contours->debugValidateJoinerIndex += 1;
    contours->debugCheckLastEdge = false;
    if (LinkPass::unambiguous == linkPass) {
        for (auto e : byArea) {
            e->debugValidate();
            OP_ASSERT(!e->isActive() || !e->debugIsLoop());
        }
    }
    for (auto e : unsectByArea) {
        e->debugValidate();
        OP_ASSERT(!e->isActive() || !e->debugIsLoop());
    }
    for (auto e : disabled) {
        e->debugValidate();
//        OP_ASSERT(!e->debugIsLoop());
    }
    for (auto e : unsortables) {
        e->debugValidate();
        OP_ASSERT(!e->isActive() || !e->debugIsLoop());
    }
    contours->debugCheckLastEdge = true;
    for (auto e : linkups.l) {
        e->debugValidate();
        OP_ASSERT(!e->priorEdge);
        OP_ASSERT(e->lastEdge);
        OP_ASSERT(!e->debugIsLoop());
    }
}

void OpSegment::debugValidate() const {
    for (auto i : sects.i)
        i->debugValidate();
}

#include "OpWinder.h"

void OpWinder::debugValidate() const {
    for (auto& edge : inX)
        edge->debugValidate();
    for (auto& edge : inY)
        edge->debugValidate();
}
#endif

#include "PathOps.h"

int OpOutPath::debugNextID(OpEdge* edge) {
    return edge->segment->contour->nextID();
}

#endif
