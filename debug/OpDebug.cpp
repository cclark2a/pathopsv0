// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG || OP_RELEASE_TEST
#include <atomic>
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif

std::atomic_int testsWarn;
int debugPrecision = 8; // -1;		// minus one means unset
bool debugSmall = true;  // set to false to show sub-epsilon values as ~0
bool debugEpsilon = false;  // set to true to show values smaller than 100 * OpEpsilon as eps 

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
#endif

#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP || OP_TINY_SKIA
bool debugHexFloat = false;
#endif

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
    if (s.size()) OutputDebugStringA(s.c_str());  // !!! printing empty strings slows visual studio!
#else
    fprintf(stderr, "%s", s.c_str());
#endif
}

#include "OpMath.h"

std::string OpDebugStr(float value) {
    if (OpMath::IsNaN(value))
        return "NaN";
    if (!OpMath::IsFinite(value))
        return value > 0 ? "Inf" : "-Inf";
    if (0 == value)
        return "0";
    if (!debugSmall && fabsf(value) < OpEpsilon)
        return "~0";
    std::string result;
    bool _small = false;
    if (debugEpsilon) {
        _small = fabsf(value) <= OpEpsilon * 100;
        if (_small)
            value = value / OpEpsilon;
    }
    if (debugPrecision < 0) {
        if (_small) {
            if (value < 0)
                result = "-";
            value = floorf((fabsf(value) * 10 + 5));  // round up to epsilon + one digit of fraction
            result += std::to_string((int) value / 10) + "." + std::to_string((int) value % 10);
        } else
            result = std::to_string(value);
    } else {
        std::string s(16, '\0');
        auto written = std::snprintf(&s[0], s.size(), "%.*g", debugPrecision, value);
        s.resize(written);
        result = s;
    }
    if (_small)
#if _WIN32
        result += "ep";
#else
        result += "\u03B5";  // epsilon (fails on Windows Visual Studio, wrong character encoding)
#endif
#if 1    // trim trailing zeroes (useful if precision is -1)
    size_t pos = result.find('.');
    if (std::string::npos == pos)
        return result;
    pos = result.find('e');
    if (std::string::npos != pos)
        return result;
    while (result.back() == '0')
        result.pop_back();
    if (result.back() == '.')
        result.pop_back();
#endif
    return result;
}

#endif

#if OP_DEBUG || OP_DEBUG_DUMP || OP_DEBUG_IMAGE

#include "OpCurve.h"

#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE
std::string OpDebugByteToHex(uint8_t hex) {
    std::string s = "0x";
    for (int index = 4; index >= 0; index -= 4) {
        int nybble = (hex >> index) & 0xF;
        if (nybble <= 9)
            s += '0' + nybble;
        else 
            s += 'a' + nybble - 10;
    }
    return s;
}
#endif


#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE || OP_TINY_SKIA
std::string OpDebugIntToHex(int32_t hex) {
    std::string s = "0x";
    for (int index = 28; index >= 0; index -= 4) {
        int nybble = (hex >> index) & 0xF;
        if (nybble <= 9)
            s += '0' + nybble;
        else 
            s += 'a' + nybble - 10;
    }
    return s;
}
#endif


#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE
std::string OpDebugPtrToHex(void* ptr) {
    uint64_t hex = (uint64_t) ptr;
    std::string s = "0x";
    for (int index = 60; index >= 0; index -= 4) {
        int nybble = (hex >> index) & 0xF;
        if (nybble <= 9)
            s += '0' + nybble;
        else 
            s += 'a' + nybble - 10;
    }
    return s;
}
#endif

#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE || OP_TINY_SKIA
std::string OpDebugDumpHex(float f) {
    if (!debugHexFloat) {
        int32_t hex = OpDebugFloatToBits(f);
        std::string s = OpDebugIntToHex(hex);
        return s;
    }
    char buffer[256];
    int bytes = snprintf(buffer, sizeof(buffer), "%af", f);
    return std::string(buffer, bytes);
}
#endif

#if OP_DEBUG_DUMP || OP_DEBUG_IMAGE
std::string OpDebugDumpByteArray(const char* bytes, size_t size) {
    std::string s = "[";
    size_t lastReturn = 0;
    for (size_t index = 0; index < size; ++index) {
        size_t lastSpace = s.size() - 1;
        s += OpDebugByteToHex((uint8_t) bytes[index]) + " ";
        if (s.size() - lastReturn > 100) {  // !!! hard-code to line length for now
            s[lastSpace] = '\n';
            lastReturn = lastSpace;
        }
    }
    if (' ' >= s.back())
        s.pop_back();
    s += "]";
    return s;
}

void playback() {
	FILE* file = fopen("OpDebugImageState.txt", "r");
	if (!file)
		return;
    OpDebugImage::playback(file);
    dmpPlayback(file);
	fclose(file);
}

void record() {
#if 0 && defined _WIN32
   char full[_MAX_PATH];
   if( _fullpath( full, ".\\", _MAX_PATH ) != NULL )
      OpDebugOut( "Full path is: %s" + std::string(full) + "\n");
   else
      OpDebugOut( "Invalid path\n" );
#endif
	FILE* recordFile = fopen("opDebugImageState.txt", "w");
	if (!recordFile) {
		OpDebugOut("failed to open opDebugImageState.txt for writing\n");
		return;
	}
    OpDebugImage::record(recordFile);
    dmpRecord(recordFile);
	fclose(recordFile);
}
#endif

int OpDebugCountDelimiters(const char* str, char delimiter, char openBracket, char closeBracket) {
    int count = 0;
    int openCount = 1;
    do {
        char ch = *str++;
        if (openBracket == ch)
            ++openCount;
        else if (delimiter == ch)
            count += 1 == openCount;
        else if (closeBracket == ch)
            --openCount;
    } while (openCount > 0);
    return count;
}

void OpDebugExit(std::string message) {
    OpDebugOut(message);
    exit(1);
}

void OpDebugExitOnFail(std::string message, bool condition) {
    if (condition)
        return;
    OpDebugExit(message);
}

int32_t OpDebugFloatToBits(float f) {
    FloatIntUnion d;
    d.f = f;
    return d.i;
}

float OpDebugHexToFloat(const char*& str) {
    // !!! add support for hex float %a format
    FloatIntUnion d;
    d.i = OpDebugHexToInt(str);
    if (')' == str[0])
        ++str;
    if (',' == str[0])
        ++str;
    if (' ' >= str[0])
        ++str;
    return d.f;
}

int32_t OpDebugHexToInt(const char*& str) {
    int32_t result = 0;
    if ('[' == str[0])
        ++str;
    OpDebugRequired(str, "0x");
    for (int index = 0; index < 8; ++index) {
        char c = *str++;
        int nybble = c - '0';
        if (nybble > 9)
            nybble = c - 'a' + 10;
        result = (result << 4) | nybble;
    }
    if (' ' >= str[0])
        ++str;
    if (']' == str[0])
        ++str;
    return result;
}

uint8_t OpDebugByteToInt(const char*& str) {
    uint8_t result = 0;
    if ('[' == str[0])
        ++str;
    OpDebugRequired(str, "0x");
    for (int index = 0; index < 2; ++index) {
        char c = *str++;
        int nybble = c - '0';
        if (nybble > 9)
            nybble = c - 'a' + 10;
        result = (result << 4) | nybble;
    }
    if (' ' >= str[0])
        ++str;
    if (']' == str[0])
        ++str;
    return result;
}

std::vector<uint8_t> OpDebugByteArray(const char*& str) {
    std::vector<uint8_t> result;
    OpDebugRequired(str, "[");
    OpDebugOptional(str, "]");
    while (']' != str[-1]) {
        result.push_back(OpDebugByteToInt(str));
    }
    if (' ' >= str[0])
        ++str;
    return result;
}

float OpDebugReadNamedFloat(const char*& str, const char* label) {
    OpDebugRequired(str, label);
    float result = OpDebugHexToFloat(str);
    return result;
}

int OpDebugReadNamedInt(const char*& str, const char* label) {
    OpDebugRequired(str, label);
    char* endPtr;
    int result = strtol(str, &endPtr, 10);
    str = endPtr;
    if (')' == str[0])
        ++str;
    if (',' == str[0])
        ++str;
    if (' ' >= str[0])
        ++str;
    return result;
}

size_t OpDebugReadSizeT(char const *& str) {
    if ('[' == str[0])
        ++str;
    unsigned long result;
    if (OpDebugOptional(str, "unset"))
        result = OpMax;
    else if (OpDebugOptional(str, "-"))
        result = 0;
    else {
        char* endPtr;
        result = strtoul(str, &endPtr, 10);
        str = endPtr;
    }
    if (']' == str[0] || '/' == str[0])
        ++str;
    if (' ' >= str[0])
        ++str;
    return (size_t) result;
}

std::string OpDebugLabel(const char*& str) {
    if (' ' >= str[0])
        ++str;
    std::string result;
    while (' ' < str[0])
        result += *str++;
    if (' ' >= str[0] || ':' == str[0])
        ++str;
    return result;
}

bool OpDebugOptional(const char*& str, const char* match) {
    size_t matchLen = strlen(match);
    while(str[0] && ' ' >= str[0])
        ++str;
    if (!strncmp(match, str, matchLen)) {
        str += matchLen;
        if (' ' >= str[0] || ':' == str[0])
            ++str;
        return true;
    }
    return false;
}

void OpDebugRequired(const char*& str, const char* match) {
    OpDebugExitOnFail(match + std::string(" failed"), OpDebugOptional(str, match));
}

#endif

#if OP_DEBUG

bool OpPoint::debugIsUninitialized() const {
	return OpMath::IsDebugNaN(x) && OpMath::IsDebugNaN(y);
}

void OpMath::DebugCompare(float a, float b) {
    float diff = fabsf(a - b);
    float max = std::max(fabsf(a), fabsf(b));
    float pPrecision = diff / max;
    OP_ASSERT(pPrecision < OpEpsilon);
}

bool OpMath::IsDebugNaN(float f) {
	if (!std::isnan(f))
		return false;
	int32_t fBits = OpDebugFloatToBits(f);
	return fBits & 1;
}

#include "OpCurveCurve.h"

// !!! debugging failure in thread_cubics8753
#if 0
OpCurve OpCurve::toVerticalDouble(const LinePts& line) const {
	OpCurve rotated;
	double adj = (double) line.pts[1].x - line.pts[0].x;
	double opp = (double) line.pts[1].y - line.pts[0].y;
	for (int n = 0; n < pointCount(); ++n) {
		double vdx = (double) pts[n].x - line.pts[0].x;
		double vdy = (double) pts[n].y - line.pts[0].y;
		rotated.pts[n].x = (float) (vdy * adj - vdx * opp);
		rotated.pts[n].y = (float) (vdy * opp + vdx * adj);
	}
	rotated.weight = weight;
	rotated.c.type = c.type;
	return rotated;
}
#endif

#if OP_DEBUG_VERBOSE
void OpCurveCurve::debugSaveState() {
	if ((int) dvDepthIndex.size() < depth)
		dvDepthIndex.push_back(dvAll.size());
	for (auto edge : edgeCurves.c)
		dvAll.push_back(edge);
	for (auto oppEdge : oppCurves.c)
		dvAll.push_back(oppEdge);
}
#endif

// return false for caller to assert
bool OpCurveCurve::debugShowImage(bool atDepth) {
	if (OpDebugSkipBreak())
		return true;
    if (contours->debugData.curveCurveDepth < 0)
        return true;
    if (atDepth && !contours->debugData.curveCurveDepth)
        return true;
    if (contours->debugData.curveCurve1 != seg->id && contours->debugData.curveCurve2 != seg->id)
        return true;
    if (contours->debugData.curveCurve1 != opp->id && contours->debugData.curveCurve2 != opp->id)
        return true;
    if (atDepth && depth < contours->debugData.curveCurveDepth)
		return true;
#if OP_DEBUG_DUMP
	if (!atDepth || contours->debugData.curveCurveDepth == depth)
		::debug();
#endif
	OP_DEBUG_IMAGE_CODE(1 == depth ? ::showSegmentEdges() : ::hideSegmentEdges());
#if OP_DEBUG_DUMP
#if OP_DEBUG_VALIDATE
	if (contours->debugData.curveCurveDepth < depth) {
		::dmpDepth(depth);
		::drawDepth(depth);
	}
#endif
	dmpFile();
	verifyFile(contours);
#endif
	return false;
}

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

// keep this in sync with op edge : debug dump chain
// ignore axis changes when detecting sum loops (for now)
// !!! if the axis change is required to detect for sum loops, document why!
// !!! either add 'stamp' or rewalk links instead of find
const OpEdge* OpEdge::debugIsLoop(EdgeMatch which, LeadingLoop leading) const {
	if (!(EdgeMatch::start == which ? priorEdge : nextEdge))
		return nullptr;
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	for (;;) {
		seen.push_back(chain);
		chain = EdgeMatch::start == which ? chain->priorEdge : chain->nextEdge;
		if (!chain)
			break;	
		if (seen.end() == std::find(seen.begin(), seen.end(), chain))
			continue;
		if (LeadingLoop::will == leading)
			return this;
		OP_ASSERT(LeadingLoop::in == leading);
		return chain;
	}
	return nullptr;
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

#if OP_DEBUG_VALIDATE
void OpHulls::debugValidate() const {
	if (h.size() < 2)
		return;
	OP_ASSERT(h[0].opp);
	OpVector threshold = h[0].opp->segment->threshold();
	for (size_t outer = 0; outer < h.size() - 1; ++outer) {
			const HullSect& o = h[outer];
		for (size_t inner = outer + 1; inner < h.size(); ++inner) {
			const HullSect& i = h[inner];
			OP_ASSERT(!o.sect.pt.isNearly(i.sect.pt, threshold));
		}
	}
}
#endif

#include "OpIntersection.h"

#if OP_DEBUG_VALIDATE
void OpIntersection::debugValidate() const {
	OP_ASSERT(OpMath::Between(0, ptT.t, 1));
	OpPoint pt = segment->c.ptAtT(ptT.t);
	OpMath::DebugCompare(pt, ptT.pt);
	OpPoint oPt = opp->segment->c.ptAtT(opp->ptT.t);
	OpMath::DebugCompare(pt, oPt);
}

void OpIntersection::debugCoinValidate() const {
	const OpIntersection* coins[4] = { nullptr, nullptr, nullptr, nullptr };
	int match[4] = { -1, -1, -1, -1 };
	int used = 0;
	int segCnt = 0;
	int oppCnt = 0;
	int segStart = 0;
	int segEnd = 0;
	int oppStart = 0;
	int oppEnd = 0;
	OP_ASSERT(coincidenceID);
	for (auto seg : { segment, opp->segment } ) {
		for (auto sect : seg->sects.i) {
			if (sect->coincidenceID != coincidenceID)
				continue;
			OP_ASSERT(used < 4);
			coins[used] = sect;
			if (sect->segment == segment) {
				++segCnt;
				segStart += MatchEnds::start == sect->coinEnd;
				segEnd += MatchEnds::end == sect->coinEnd;
			}
			if (sect->segment == opp->segment) {
				++oppCnt;
				oppStart += MatchEnds::start == sect->coinEnd;
				oppEnd += MatchEnds::end == sect->coinEnd;
			}
			for (int inner = 0; inner < used; ++inner) {
				if (coins[inner]->ptT.pt == coins[used]->ptT.pt) {
					match[inner] = used;
					match[used] = inner;
				}
			}
			++used;
		}
	}
	OP_ASSERT(4 == used);
	OP_ASSERT(2 == segCnt);
	OP_ASSERT(2 == oppCnt);
	OP_ASSERT(1 == segStart);
	OP_ASSERT(1 == segEnd);
	OP_ASSERT(1 == oppStart);
	OP_ASSERT(1 == oppEnd);
	for (int index = 0; index < 4; ++index)
		OP_ASSERT(0 <= match[index] && match[index] < 4);
	for (int index = 0; index < 4; ++index)
		OP_ASSERT(index == match[match[index]]);
	OP_ASSERT(coins[0]->ptT.t != coins[1]->ptT.t);
	if (coins[0]->ptT.t < coins[1]->ptT.t) {
		OP_ASSERT(MatchEnds::start == coins[0]->coinEnd);
		OP_ASSERT(MatchEnds::end == coins[1]->coinEnd);
	} else {
		OP_ASSERT(MatchEnds::end == coins[0]->coinEnd);
		OP_ASSERT(MatchEnds::start == coins[1]->coinEnd);
	}
	OP_ASSERT(coins[2]->ptT.t != coins[3]->ptT.t);
	if (coins[2]->ptT.t < coins[3]->ptT.t) {
		OP_ASSERT(MatchEnds::start == coins[2]->coinEnd);
		OP_ASSERT(MatchEnds::end == coins[3]->coinEnd);
	} else {
		OP_ASSERT(MatchEnds::end == coins[2]->coinEnd);
		OP_ASSERT(MatchEnds::start == coins[3]->coinEnd);
	}
}
#endif

void OpIntersection::debugSetID() {
	id = segment->nextID();
}

#if OP_DEBUG_VALIDATE
void OpIntersections::debugValidate() const {
	for (const auto sectPtr : i) {
		OP_ASSERT(sectPtr->opp->opp == sectPtr);
		OP_ASSERT(sectPtr->ptT.pt == sectPtr->opp->ptT.pt 
				|| (!!sectPtr->unsectID && !!sectPtr->opp->unsectID));
	}
}
#endif

bool OpIntersections::debugContains(const OpPtT& ptT, const OpSegment* opp) const {
	for (auto sect : i) {
		if ((sect->ptT.pt == ptT.pt || sect->ptT.t == ptT.t) 
				&& sect->opp && sect->opp->segment == opp)
			return true;
	}
	return false;
}

OpIntersection* OpIntersections::debugAlreadyContains(const OpPoint& pt, const OpSegment* oppSegment) const {
	for (auto sect : i) {
		if (oppSegment == sect->opp->segment && pt == sect->ptT.pt)
			return sect;
	}
	return nullptr;
}

#include "OpJoiner.h"

void OpContours::debugRemap(int oldRayMatch, int newRayMatch) {
    for (auto contour : contours) {
        for (auto& segment : contour->segments) {
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
            if (Unsortable::none != linkup->isUnsortable)
                continue;
            if (linkup->disabled)
                continue;
            const EdgePal* linkDist = nullptr;
            OpEdge* dTest = nullptr;
            OP_DEBUG_CODE(const EdgePal* dDist = nullptr);
            for (const EdgePal* dist = &linkup->ray.distances.back(); 
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
            OP_ASSERT(WindZero::unset != linkZero);
	        NormalDirection NdotR = linkup->normalDirection(-linkup->ray.axis, 
                    linkDist->edgeInsideT);
            if (NormalDirection::downLeft == NdotR)
                linkZero = !linkZero;    // get wind zero for edge normal pointing left
            if (WindZero::nonZero == linkZero) {
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
            NdotR = dTest->normalDirection(linkup->ray.axis, dDist->edgeInsideT);
            if (NormalDirection::downLeft == NdotR)
                distZero = !distZero;    // get wind zero for prior normal pointing right
                // either neither zero should be opp, or both should be 
            // !!! for now, just track when this happens. Wait until we can't ignore it to fix
            if ((WindZero::nonZero == distZero) != (WindZero::nonZero == linkZero)) {
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

// return false to auto-break
bool OpJoiner::DebugShowImage() {
	if (!OpDebugSkipBreak()) {
#if OP_DEBUG_IMAGE && 0  // locally defeat if test is very large (e.g., grshapearc)
		::debugImage();
		::showFill();
#endif
		return false;
	}
	return true;
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
    if (LinkPass::remaining != linkPass) {
        for (auto e : byArea) {
            e->debugValidate();
            OP_ASSERT(!e->isActive() || !e->debugIsLoop());
            OP_ASSERT(!e->disabled);
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
        if (e->debugScheduledForErasure)
            continue;
        e->debugValidate();
        OP_ASSERT(!e->priorEdge);
        OP_ASSERT(e->disabled || e->lastEdge);
        OP_ASSERT(!e->debugIsLoop());
    }
}

void OpSegment::debugValidate() const {
    for (auto i : sects.i)
        i->debugValidate();
}

OpTree::~OpTree() {
	contours->debugTree = nullptr;
}

#include "OpWinder.h"

void OpWinder::debugValidate() const {
    for (auto& edge : inX)
        edge->debugValidate();
    for (auto& edge : inY)
        edge->debugValidate();
}
#endif

#if OP_DEBUG_DUMP
std::string debugContext;

void debugImage() {
#if OP_DEBUG_IMAGE
    if ("linkRemaining" == debugContext || "linkUnambiguous" == debugContext 
            || "apply" == debugContext || "makeCoins" == debugContext
			|| "transferCoins" == debugContext) {
        ::hideOperands();
        ::showEdges();
        ::showIDs();
        ::showPoints();
        ::showValues();
        ::showWindings();
        ::showTangents();
        ::resetFocus();
        ::oo();
        return;
    }
    if ("divideAndConquer" == debugContext) {
        ::hideOperands();
        ::hideSegmentEdges();
        ::hideWindings();
        ::showEdges();
        ::showIDs();
        ::showPoints();
        ::showTangents();
        ::showValues();
        ::resetFocus();
        ::oo();
        return;
    }
    if ("findIntersections" == debugContext || "AddLineCurveIntersection" == debugContext
            || "AddEndMatches" == debugContext) {
        ::hideOperands();
        ::showSegments();
        ::showIDs();
        ::showPoints();
        ::showValues();
        ::resetFocus();
        ::oo();
        return;
    }
    ::hideOperands();
    ::showSegments();
    ::showFill();
    ::showIDs();
    ::showPoints();
    ::showTangents();
    ::showValues();
    ::resetFocus();
    ::oo();
#endif
}

void debug() {
    debugImage();
    if ("linkRemaining" == debugContext || "linkUnambiguous" == debugContext) {
        ::dmp(debugGlobalContours->debugJoiner);
        return;
    }
    if ("divideAndConquer" == debugContext) {
        if (debugGlobalContours->debugCurveCurve)
            ::dmp(debugGlobalContours->debugCurveCurve);
        return;
    }
    if ("findIntersections" == debugContext || "AddLineCurveIntersection" == debugContext
            || "AddEndMatches" == debugContext || "transferCoins" == debugContext) {
        OpSaveDump save(DebugLevel::brief, DebugBase::dec);
        ::dmpSegments();
        return;
    }
    debugGlobalContours->dump();
}

#endif

#endif
