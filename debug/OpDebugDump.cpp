// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_DUMP
#include <ctype.h>

#include "OpCurveCurve.h"
#include "OpDebugColor.h"
#include "OpDebugDump.h"
#include "OpEdge.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

int OpCurveCurve::debugCall = INT_MAX;  // which call to curve-curve was made

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
		if (' ' == s.back()) s.pop_back(); \
		OpDebugFormat(s + "]"); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dmpIDs(const std::vector<Thing>& things) { \
		std::string s = "["; \
		for (const auto& thing : things) \
			s += STR(thing->id) + " "; \
		if (' ' == s.back()) s.pop_back(); \
		OpDebugFormat(s + "]"); \
	}
	VECTOR_PTRS
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
				if (coin != coins.back()) \
					OpDebugOut("\n"); \
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
				if (output != outputs.back()) \
					OpDebugOut("\n"); \
                found = true; \
            } \
        } \
        if (std::vector<const OpEdge*> matches = findEdgeRayMatch(ID); matches.size()) { \
            for (auto match : matches) { \
                match->Method(); \
				if (match != matches.back()) \
					OpDebugOut("\n"); \
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
				if (uSect != uSects.back()) \
					OpDebugOut("\n"); \
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
} \
\
enum enum##Str(const char*& str, const char* label, enum enumDefault) { \
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
    return (enum) -1; \
}

#define ENUM_NAME_STRUCT_ABBR(enum) \
struct _##enum##Abbr { \
    enum element; \
    const char* name; \
    const char* abbr; \
}

#define ENUM_NAME_ABBR(enum) \
\
std::string enum##Abbr(enum element, DebugLevel l) { \
    for (int index = 0; index < (int) ARRAY_COUNT(enum##Abbrs); ++index) { \
        if (enum##Abbrs[index].element == element) \
            return DebugLevel::brief == l ? enum##Abbrs[index].abbr \
                    : enum##Abbrs[index].name; \
    } \
    return "missing " + std::string(#enum) + " element:" + STR((int) element); \
}

static std::string wordBounds = "\t\n\r ,:<>()[]{}";
static std::string bracketL = "<([{";
static std::string bracketR = ">)]}";

static std::string edit(std::string s, std::string skip, bool note) {
	std::string result;
	size_t first = 0;
	size_t start = 0;
    size_t end = s.size();
	while (start < end) {
		start = s.find(skip, start);
		if (std::string::npos == start)
			break;
		char prior = start > 0 ? s[start - 1] : ' ';
		if (std::string::npos == wordBounds.find(prior))
			continue;
		size_t last = start + skip.size();
		char next = last < s.size() ? s[last] : ' ';
		if (std::string::npos == wordBounds.find(next))
			continue;
#if 0 // !!! enable and debug when needded
	// (only) if prior is in bracket left, walk until balance in bracket right is found
		std::vector<size_t> brackets;
		last = start - 1;
		while (++last < end) {
			char ch = s[last];
			size_t bracketIndex = bracketL.find(ch);
			if (std::string::npos != bracketIndex) {
				brackets.push_back(bracketIndex);
				continue;
			}
			bracketIndex = bracketR.find(ch);
			if (std::string::npos != bracketIndex && brackets.back() == bracketIndex) {
				brackets.pop_back();
				continue;
			} 
			if (!brackets.size() && std::string::npos != wordBounds.find(ch))
				break;
		}
#endif
		result += s.substr(first, start - first);
		if (note) {
			result += "**";
			result += s.substr(start, last - start);
			result += "**";
		}
		start = first = last;
	}
	result += s.substr(first);
	return result;
}

std::string stringFormat(std::string s) {
    if (!s.size())
		return "";
	for (std::string skip : debugGlobalContext->debugDumpSkips) {
		s = edit(s, skip, false);
	}
	for (std::string note : debugGlobalContext->debugDumpNotes) {
		s = edit(s, note, true);
	}
    std::string result;
	const char* start = &s.front();
    const char* end = &s.back();
    while (lineWidth && start + lineWidth <= end) {
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

void addNote(int id) {
	std::string toAdd = STR(id);
	auto& notes = debugGlobalContext->debugDumpNotes;
	if (notes.end() != std::find(notes.begin(), notes.end(), toAdd))
		return;
	debugGlobalContext->debugDumpNotes.push_back(toAdd);
}

void clearNotes() {
	debugGlobalContext->debugDumpNotes.clear();
}

void addSkip(std::string s) {
	auto& notes = debugGlobalContext->debugDumpSkips;
	if (notes.end() != std::find(notes.begin(), notes.end(), s))
		return;
	debugGlobalContext->debugDumpSkips.push_back(s);
}

void clearSkips() {
	debugGlobalContext->debugDumpSkips.clear();
}

void OpDebugFormat(std::string s) {
    std::string result = stringFormat(s);
	if ('\n' != result.back())
		s += "\n";
    OpDebugOut(result);
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
    for (const auto c : contourIterator) {
        for (const auto& seg : c->segments) {
            for (const auto& edge : seg.edges) {
                if (edge.isActive())
                    edge.dump();
            }
        }
    }
}

void dmpAliases() {
    debugGlobalContext->aliases.dump();
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
    s.pop_back();
    return s;
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
    s.pop_back();
    return s;
}

void dmpFile(OpContext* context, std::string filename) {
    FILE* file = fopen(filename.c_str(), "w");
    std::string s;
    s += context->debugDump(DebugLevel::file, DebugBase::hex);
    int saveWidth = lineWidth;
    lineWidth = 100;
    s = stringFormat(s);
    lineWidth = saveWidth;
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}

static std::string fileToStr(std::string filename) {
    std::string buffer;
    FILE* file = fopen(filename.c_str(), "r");
    OP_ASSERT(file);
    int seek = fseek(file, 0, SEEK_END);
    OP_ASSERT(!seek);
    long size = ftell(file);
    fclose(file);
    file = fopen(filename.c_str(), "r");
    buffer.resize(size);
    fread(&buffer[0], 1, size, file);
    fclose(file);
    return buffer;
}

OpContext* fromFile(std::string filename) {
    std::string buffer = fileToStr(filename);
    const char* str = buffer.c_str();
    OpContext* fileContours = new OpContext(nullptr);
    fileContours->dumpSet(str);  // also reads segments, which read segments' edges, etc.
    fileContours->dumpResolveAll(fileContours);
    return fileContours;
}

void verifyFile(OpContext* context, std::string fromFilename, std::string verifyFilename) {
    dmpFile(context, fromFilename);
	OpContext* fileContext = fromFile(fromFilename);
    dmpFile(fileContext, verifyFilename);
    std::string orig = fileToStr(fromFilename);
    std::string copy = fileToStr(verifyFilename);
    OP_ASSERT(orig == copy);
    delete fileContext;
}

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
    s.pop_back();
    OpDebugFormat(s);
}

void OpContext::dumpResolve(OpContour*& contourRef) {
    int contourID = (int) (size_t) contourRef;
    if (!contourStorage) {  // !!! looks like a macro candidate to me
        OpDebugOut(__func__ + std::string(": !contourStorage"));
        exit(1);
    }
    contourRef = contourStorage->debugFind(contourID);
    if (contourRef->id != contourID) {
        OpDebugOut(__func__ + std::string(": contourRef->id != contourID"));
        exit(1);
    }
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
    OP_ASSERT((int) (size_t) edgeRef != edgeID);
}

void OpContext::dumpResolve(const OpEdge*& edgeRef) {
    dumpResolve(const_cast<OpEdge*&>(edgeRef));
}

void OpContext::dumpResolve(const OpLimb*& limbRef) {
    int limbID = (int) (size_t) limbRef;
    if (0 == limbID)
        return;
    const OpLimb* limb = limbStorage->debugFind(limbID);
    OP_ASSERT(limb);
    limbRef = limb;
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
    OP_ASSERT((int) (size_t) segRef != segID);
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

void dmpJoin() {
	std::string s;
	if (debugGlobalContext->debugJoiner) {
		dmp(debugGlobalContext->debugJoiner);
		s = "\n";
	}
    for (const auto& c : contourIterator) {
		s += c->debugDumpJoin(defaultLevel, defaultBase);
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
	if (' ' == s.back())
		s.pop_back();
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
void OpContext::dumpCount(std::string label) const {
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

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { OpDebugExpect::w, #w }
#define OpDebugExpect_Base
ENUM_NAME_STRUCT(OpDebugExpect)

#define BOOL_TO_STR(data) if (data) s += #data + std::string(" ")

std::string OpPtAliases::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "aliases[\n";
    for (OpPoint pt : aliases) {
        s += pt.debugDump(l, b) + "\n";
    }
    s += "] maps[\n";
    for (OpPtAlias map : maps) {
        s += map.original.debugDump(l, b) + ":";
        s += map.alias.debugDump(l, b) + "\n";
    }
    s += "] threshold:" + threshold.debugDump(l, b);
    return s;
}

void OpPtAliases::dumpSet(const char*& str) {
    OpDebugRequired(str, "aliases");
    size_t size = OpDebugReadSizeT(str);
    aliases.resize(size);
    for (auto& alias : aliases) {
        alias.dumpSet(str);
    }
    OpDebugRequired(str, "maps");
    size = OpDebugReadSizeT(str);
    maps.resize(size);
    for (auto& map : maps) {
        map.original.dumpSet(str);
        map.alias.dumpSet(str);
    }
    OpDebugRequired(str, "threshold");
    threshold.dumpSet(str);
}

namespace PathOpsV0Lib {

// don't want funny macros in public interface, so this is explicitly for the only public enum... 
#define CONTEXT_ERROR_NAME(r) { ContextError::r, #r }

struct ContextErrorName {
    ContextError element;
    std::string name;
} contextErrorNames[] = {
	CONTEXT_ERROR_NAME(none),
	CONTEXT_ERROR_NAME(end),
    CONTEXT_ERROR_NAME(finite),
    CONTEXT_ERROR_NAME(gap),
	CONTEXT_ERROR_NAME(intersection),
	CONTEXT_ERROR_NAME(loop),
	CONTEXT_ERROR_NAME(missing),
	CONTEXT_ERROR_NAME(toVertical),
    CONTEXT_ERROR_NAME(tree),
};

#undef CONTEXT_ERROR_NAME

static bool contextErrorOutOfDate = false;

std::string contextErrorName(ContextError element) {
    static bool contextErrorChecked = false;
    int first = (int) contextErrorNames[0].element;
    if (!contextErrorChecked) {
        for (int index = 0; index < (int) ARRAY_COUNT(contextErrorNames); ++index)
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

ContextError contextErrorStr(const char*& str, const char* label, ContextError enumDefault) {
    if (!OpDebugOptional(str, label))
        return enumDefault;
    size_t strLen = 0;
    while (isalnum(str[strLen]))
        ++strLen;
    for (int index = 0; index < (int) ARRAY_COUNT(contextErrorNames); ++index) {
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

// use static asserts throughout to ensure that all of context is serialized
#define ASSERT_SERIAL_OFFSET(inst, last, offset, thisField) \
    static_assert(offsetof(std::remove_reference_t<decltype(inst)>, last) + sizeof((inst).last) \
            + offset == offsetof(std::remove_reference_t<decltype(inst)>, thisField))

#define ASSERT_SERIAL(instance, lastField, thisField) \
    ASSERT_SERIAL_OFFSET(instance, lastField, 0, thisField)

// macro checks that function ptrs are consecutive
#define DEBUG_FIND_TAG(callback, lastField, thisField) \
    ASSERT_SERIAL(callback, lastField, thisField); \
    s += debugFindTag(reinterpret_cast<DebugFunction>(callback.thisField))

#define DEBUG_DUMP_BOOL(instance, lastField, thisBool) \
    ASSERT_SERIAL(instance, lastField, thisBool); \
    if (thisBool) s += #thisBool " "

#define DEBUG_DUMP_BOOL_INST(instance, lastField, thisBool) \
    ASSERT_SERIAL(instance, lastField, thisBool); \
    if (instance.thisBool) s += #thisBool " "

// must be written before segments' curves for curve names
static std::string debugCallbacksDump(const std::vector<PathOpsV0Lib::DebugCurveCallbacks>& 
        debugCallbacks, DebugLevel l, DebugBase b) {
    std::string s = "debugCallbacks:" + STR(debugCallbacks.size()) + "\n";
    for (auto& debugCallback : debugCallbacks) {
        static_assert(0 == offsetof(PathOpsV0Lib::DebugCurveCallbacks, scaleFuncPtr));
        s += debugFindTag(reinterpret_cast<DebugFunction>(debugCallback.scaleFuncPtr));
	    DEBUG_FIND_TAG(debugCallback, scaleFuncPtr,      curveNameFuncPtr);
	    DEBUG_FIND_TAG(debugCallback, curveNameFuncPtr,  curveExtraFuncPtr);
#if OP_DEBUG_IMAGE
	    DEBUG_FIND_TAG(debugCallback, curveExtraFuncPtr, addToPathFuncPtr);
        static_assert(sizeof(PathOpsV0Lib::DebugCurveCallbacks)  
                == offsetof(PathOpsV0Lib::DebugCurveCallbacks, addToPathFuncPtr)
                + sizeof(debugCallback.addToPathFuncPtr));
#else
        static_assert(sizeof(PathOpsV0Lib::DebugCurveCallbacks)  
                == offsetof(PathOpsV0Lib::DebugCurveCallbacks, curveExtraFuncPtr)
                + sizeof(debugCallback.curveExtraFuncPtr));
#endif
    }
    return s;
}

// must be written before contours' windings
static std::string debugContextCallbacksDump(const PathOpsV0Lib::DebugContextCallbacks& 
        debugContextCallbacks, DebugLevel l, DebugBase b) {
    std::string s = "debugContextCallbacks:";
    static_assert(0 == offsetof(PathOpsV0Lib::DebugContextCallbacks, debugDumpContextExtraFuncPtr));
    s += debugFindTag(reinterpret_cast<DebugFunction>(debugContextCallbacks.debugDumpContextExtraFuncPtr));
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpContextExtraFuncPtr, debugDumpWindingOutFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpWindingOutFuncPtr, debugDumpWindingSetFuncPtr);
#if OP_DEBUG_IMAGE
    DEBUG_FIND_TAG(debugContextCallbacks, debugDumpWindingSetFuncPtr, debugImageWindingOutXFuncPtr);
    DEBUG_FIND_TAG(debugContextCallbacks, debugImageWindingOutXFuncPtr, debugImageWindingOutFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugImageWindingOutFuncPtr) 
            + sizeof(debugContextCallbacks.debugImageWindingOutFuncPtr) == sizeof(debugContextCallbacks));

#else
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugDumpWindingSetFuncPtr) 
            + sizeof(debugContextCallbacks.debugDumpWindingSetFuncPtr) == sizeof(debugContextCallbacks));

#endif
    return s;
}

std::string OpContext::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    static_assert(0 == offsetof(OpContext, aliases));
    if (aliases.maps.size()) {
        s += "aliases:";
        s += aliases.debugDump(l, b) + "\n";
    }
    ASSERT_SERIAL(*this, aliases, callbacks);
    s += "callbacks:" + STR(callbacks.size()) + "\n";
    for (auto& callback : callbacks) {
        static_assert(0 == offsetof(PathOpsV0Lib::CurveCallbacks, axisTFuncPtr));
        s += debugFindTag(reinterpret_cast<DebugFunction>(callback.axisTFuncPtr));
	    DEBUG_FIND_TAG(callback, axisTFuncPtr,          rotateTFuncPtr);
	    DEBUG_FIND_TAG(callback, rotateTFuncPtr,        curveHullFuncPtr);
	    DEBUG_FIND_TAG(callback, curveHullFuncPtr,      curveIsFiniteFuncPtr);
	    DEBUG_FIND_TAG(callback, curveIsFiniteFuncPtr,  curveIsLineFuncPtr);
	    DEBUG_FIND_TAG(callback, curveIsLineFuncPtr,    setBoundsFuncPtr);
	    DEBUG_FIND_TAG(callback, setBoundsFuncPtr,      curveTangentFuncPtr);
	    DEBUG_FIND_TAG(callback, curveTangentFuncPtr,   curvesEqualFuncPtr);
	    DEBUG_FIND_TAG(callback, curvesEqualFuncPtr,    ptAtTFuncPtr);
	    DEBUG_FIND_TAG(callback, ptAtTFuncPtr,          ptDAtTFuncPtr);
	    DEBUG_FIND_TAG(callback, ptDAtTFuncPtr,         ptCountFuncPtr);
	    DEBUG_FIND_TAG(callback, ptCountFuncPtr,        rotateFuncPtr);
	    DEBUG_FIND_TAG(callback, rotateFuncPtr,         subDivideFuncPtr);
	    DEBUG_FIND_TAG(callback, subDivideFuncPtr,      xyAtTFuncPtr);
	    DEBUG_FIND_TAG(callback, xyAtTFuncPtr,          curveReverseFuncPtr);
	    DEBUG_FIND_TAG(callback, curveReverseFuncPtr,   crossThresholdFuncPtr);
	    DEBUG_FIND_TAG(callback, crossThresholdFuncPtr, cutFuncPtr);
	    DEBUG_FIND_TAG(callback, cutFuncPtr,            interceptFuncPtr);
	    DEBUG_FIND_TAG(callback, interceptFuncPtr,      normalLimitFuncPtr);
        static_assert(offsetof(PathOpsV0Lib::CurveCallbacks, normalLimitFuncPtr) 
                + sizeof(callback.normalLimitFuncPtr) == sizeof(callback));
    }
    ASSERT_SERIAL(*this, callbacks, nativeCurveTypes);
    s += "nativeCurveTypes:" + STR(nativeCurveTypes.size()) + " ";
    for (int nativeCurveType : nativeCurveTypes) {
        s += STR(nativeCurveType) + " ";
    }
    s += "\n";
    ASSERT_SERIAL(*this, nativeCurveTypes, contextCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::ContextCallbacks, curveOutputFuncPtr));
    s += debugFindTag(reinterpret_cast<DebugFunction>(contextCallbacks.curveOutputFuncPtr));
	DEBUG_FIND_TAG(contextCallbacks, curveOutputFuncPtr, emptyCallerPathFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, emptyCallerPathFuncPtr, setLineTypeFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, setLineTypeFuncPtr, maxSplitFuncPtr);
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
	DEBUG_FIND_TAG(contextCallbacks, maxMarginFuncPtr, maxUnsectableTFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxUnsectableTFuncPtr, maxCheckSplitFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxCheckSplitFuncPtr, maxLimbsFuncPtr);
	DEBUG_FIND_TAG(contextCallbacks, maxLimbsFuncPtr, maxGapFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::ContextCallbacks, maxGapFuncPtr) 
            + sizeof(contextCallbacks.maxGapFuncPtr) == sizeof(contextCallbacks));
    ASSERT_SERIAL(*this, contextCallbacks, windingCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::WindingCallbacks, windingAddFuncPtr));
	s += debugFindTag(reinterpret_cast<DebugFunction>(windingCallbacks.windingAddFuncPtr));
	DEBUG_FIND_TAG(windingCallbacks, windingAddFuncPtr, windingKeepFuncPtr);
	DEBUG_FIND_TAG(windingCallbacks, windingKeepFuncPtr, windingVisibleFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingVisibleFuncPtr, windingZeroFuncPtr);
	DEBUG_FIND_TAG(windingCallbacks, windingZeroFuncPtr, windingSubtractFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingSubtractFuncPtr, windingIntersectFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingIntersectFuncPtr, windingShortFuncPtr);
    DEBUG_FIND_TAG(windingCallbacks, windingShortFuncPtr, windingShortAllFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::WindingCallbacks, windingShortAllFuncPtr) 
            + sizeof(windingCallbacks.windingShortAllFuncPtr) == sizeof(windingCallbacks));
    // out of order ... but callbacks must be set before curves and windings are read
    if (!debugCallbacks.empty())
        s += debugCallbacksDump(debugCallbacks, l, b);
    s += debugContextCallbacksDump(debugContextCallbacks, l, b);
    ASSERT_SERIAL(*this, windingCallbacks, callerOutput);  // omit callerOutput
    ASSERT_SERIAL(*this, callerOutput, errorHandler);  // omit errorHandler
    ASSERT_SERIAL(*this, errorHandler, sortedContours);
    if (!sortedContours.empty()) {
        s += "sortedContours:" + STR(sortedContours.size()) + " ";
        for (auto sortedContour : sortedContours) {
            s += STR(sortedContour->id) + " ";
        }
        s += "\n";
    }
    ASSERT_SERIAL(*this, sortedContours, ccStorage);
    if (ccStorage)
        s += ccStorage->debugDump("ccStorage", l, b) + "\n";
    ASSERT_SERIAL(*this, ccStorage, curveDataStorage);
    if (curveDataStorage) {
        s += "curveDataStorage:";
        s += curveDataStorage->debugDump(l, b) + "\n";
    }
    ASSERT_SERIAL(*this, curveDataStorage, contourStorage);
    if (contourStorage)
        s += contourStorage->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, contourStorage, contours);
    if (!contours.empty()) {
        s += "contours:" + STR(contours.size()) + " ";
        for (auto contour : contours) {
            s += STR(contour->id) + " ";
        }
        s += "\n";
    }
    ASSERT_SERIAL(*this, contours, fillerStorage);
    if (fillerStorage)
        s += fillerStorage->debugDump("fillerStorage", l, b) + "\n";
    ASSERT_SERIAL(*this, fillerStorage, sectStorage);
    if (sectStorage)
        s += sectStorage->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, sectStorage, limbStorage);
    if (limbStorage)
        s += limbStorage->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, limbStorage, limbCurrent);  // omit limbCurrent
    ASSERT_SERIAL(*this, limbCurrent, callerStorage);
    if (callerStorage) {
        s += "callerStorage:";
        s += callerStorage->debugDump(l, b) + "\n";
    }
    ASSERT_SERIAL(*this, callerStorage, userData);  // omit userData
    ASSERT_SERIAL(*this, userData, maxBounds);
    if (maxBounds.isFinite()) {
        s += "maxBounds:";
        s += maxBounds.debugDump(l, b) + "\n"; 
    }
    ASSERT_SERIAL(*this, maxBounds, error);
    if (PathOpsV0Lib::ContextError::none != error)
        s += "error:" + PathOpsV0Lib::contextErrorName(error) + "\n";
    ASSERT_SERIAL(*this, error, uniqueID);
    s += "uniqueID:" + STR(uniqueID) + " ";
    DEBUG_DUMP_BOOL(*this, uniqueID, initialized);
    DEBUG_DUMP_BOOL(*this, initialized, allDiscarded);
    DEBUG_DUMP_BOOL(*this, allDiscarded, allKept);
    DEBUG_DUMP_BOOL(*this, allKept, fatalError);
    DEBUG_DUMP_BOOL(*this, fatalError, outputOne);
    DEBUG_DUMP_BOOL(*this, outputOne, linkErased);
    DEBUG_DUMP_BOOL(*this, linkErased, windingSet);
#if OP_DEBUG_VALIDATE
    ASSERT_SERIAL_OFFSET(*this, windingSet, 1, debugValidateEdgeIndex);
    s += "debugValidateEdgeIndex:" + STR(debugValidateEdgeIndex) + " ";
    ASSERT_SERIAL(*this, debugValidateEdgeIndex, debugValidateJoinerIndex);
    s += "debugValidateJoinerIndex:" + STR(debugValidateJoinerIndex) + " ";
    ASSERT_SERIAL(*this, debugValidateJoinerIndex, debugCallbacks);
#else
    ASSERT_SERIAL(*this, dumpDummy, debugCallbacks);
#endif
    s.pop_back();
    s += "\n";
    // debugCallbacks is out of order; must be set before curves are read
    // debugContextCallbacks is out of order; must be set before windings in contours are read
    ASSERT_SERIAL(*this, debugCallbacks, debugContextCallbacks);
    ASSERT_SERIAL(*this, debugContextCallbacks, debugContextData);  // omit debugContextData
    ASSERT_SERIAL(*this, debugContextData, debugData);  // omit debugData (!!! omit for now, may have uses...)
    ASSERT_SERIAL(*this, debugData, debugCurveCurve);
    if (debugCurveCurve)
        s += "debugCurveCurve:" + debugCurveCurve->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, debugCurveCurve, debugJoiner);
    if (debugJoiner)
        s += "debugJoiner:" + debugJoiner->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, debugJoiner, debugTree);
    if (debugTree)
        s += "debugTree:" + debugTree->debugDump(l, b) + "\n";
    ASSERT_SERIAL(*this, debugJoiner, debugTree);
    if (debugData.testname.size())
        s += "debugTestname:" + debugData.testname + " ";
    s += "debugExpect:" + OpDebugExpectName(debugData.expect) + " ";
	DEBUG_DUMP_BOOL(*this, debugExpect, debugInPathOps);
	DEBUG_DUMP_BOOL(*this, debugInPathOps, debugInClearEdges);
	DEBUG_DUMP_BOOL(*this, debugInClearEdges, debugCheckLastEdge);
	DEBUG_DUMP_BOOL(*this, debugCheckLastEdge, debugFailOnEqualCepts);
    static_assert(offsetof(OpContext, debugFailOnEqualCepts) + 4
            + sizeof(debugFailOnEqualCepts) == offsetof(OpContext, debugDumpNotes));
    // omit debugDumpNotes (for now)
    // omit debugDumpSkips (for now)
    return s;
}

// macro checks that function ptrs are consecutive
#define DEBUG_FIND_FUNCTION(callback, last, thisField) \
    static_assert(offsetof(std::remove_reference_t<decltype(callback)>, last) + sizeof(callback.last) \
        == offsetof(std::remove_reference_t<decltype(callback)>, thisField)); \
    callback.thisField = (decltype(callback.thisField)) debugFindFunction(str)

#define DEBUG_SET_BOOL(struc, lastField, thisField) \
    static_assert(offsetof(struc, lastField) + sizeof(lastField) == offsetof(struc, thisField)); \
    thisField = OpDebugOptional(str, #thisField)

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
#if OP_DEBUG_IMAGE
	        DEBUG_FIND_FUNCTION(debugCallback, curveExtraFuncPtr, addToPathFuncPtr);
            static_assert(offsetof(PathOpsV0Lib::DebugCurveCallbacks, addToPathFuncPtr) 
                    + sizeof(debugCallback.addToPathFuncPtr) == sizeof(debugCallback));
#else
            static_assert(offsetof(PathOpsV0Lib::DebugCurveCallbacks, curveExtraFuncPtr) 
                    + sizeof(debugCallback.curveExtraFuncPtr) == sizeof(debugCallback));
#endif
        }
    }
}

static void debugContextCallbacksDumpSet(PathOpsV0Lib::DebugContextCallbacks& debugContextCallbacks, 
        const char*& str) {
    OpDebugRequired(str, "debugContextCallbacks");
    static_assert(0 == offsetof(PathOpsV0Lib::DebugContextCallbacks, debugDumpContextExtraFuncPtr));
    debugContextCallbacks.debugDumpContextExtraFuncPtr = (PathOpsV0Lib::DebugDumpContextExtra) debugFindFunction(str);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpContextExtraFuncPtr, debugDumpWindingOutFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpWindingOutFuncPtr, debugDumpWindingSetFuncPtr);
#if OP_DEBUG_IMAGE
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugDumpWindingSetFuncPtr, debugImageWindingOutXFuncPtr);
    DEBUG_FIND_FUNCTION(debugContextCallbacks, debugImageWindingOutXFuncPtr, debugImageWindingOutFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugImageWindingOutFuncPtr) 
            + sizeof(debugContextCallbacks.debugImageWindingOutFuncPtr) == sizeof(debugContextCallbacks));

#else
    static_assert(offsetof(PathOpsV0Lib::DebugContextCallbacks, debugDumpWindingSetFuncPtr) 
            + sizeof(debugContextCallbacks.debugDumpWindingSetFuncPtr) == sizeof(debugContextCallbacks));

#endif
}

void OpContext::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "aliases"))
        aliases.dumpSet(str);
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
	    DEBUG_FIND_FUNCTION(callback, setBoundsFuncPtr,      curveTangentFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveTangentFuncPtr,   curvesEqualFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curvesEqualFuncPtr,    ptAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptAtTFuncPtr,          ptDAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptDAtTFuncPtr,         ptCountFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, ptCountFuncPtr,        rotateFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, rotateFuncPtr,         subDivideFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, subDivideFuncPtr,      xyAtTFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, xyAtTFuncPtr,          curveReverseFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, curveReverseFuncPtr,   crossThresholdFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, crossThresholdFuncPtr, cutFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, cutFuncPtr,            interceptFuncPtr);
	    DEBUG_FIND_FUNCTION(callback, interceptFuncPtr,      normalLimitFuncPtr);
        static_assert(offsetof(PathOpsV0Lib::CurveCallbacks, normalLimitFuncPtr) 
                + sizeof(callback.normalLimitFuncPtr) == sizeof(callback));
    }
    ASSERT_SERIAL(*this, callbacks, nativeCurveTypes);
    OpDebugRequired(str, "nativeCurveTypes");
    size = OpDebugReadSizeT(str);
    nativeCurveTypes.resize(size);
    for (int& nativeCurveType : nativeCurveTypes) {
       nativeCurveType = OpDebugReadSizeT(str);
    }
    ASSERT_SERIAL(*this, nativeCurveTypes, contextCallbacks);
    static_assert(0 == offsetof(PathOpsV0Lib::ContextCallbacks, curveOutputFuncPtr));
	contextCallbacks.curveOutputFuncPtr = (PathOpsV0Lib::CurveOutput) debugFindFunction(str);
    DEBUG_FIND_FUNCTION(contextCallbacks, curveOutputFuncPtr, emptyCallerPathFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, emptyCallerPathFuncPtr, setLineTypeFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, setLineTypeFuncPtr, maxSplitFuncPtr);
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
	DEBUG_FIND_FUNCTION(contextCallbacks, maxMarginFuncPtr, maxUnsectableTFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxUnsectableTFuncPtr, maxCheckSplitFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxCheckSplitFuncPtr, maxLimbsFuncPtr);
	DEBUG_FIND_FUNCTION(contextCallbacks, maxLimbsFuncPtr, maxGapFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::ContextCallbacks, maxGapFuncPtr) 
            + sizeof(contextCallbacks.maxGapFuncPtr) == sizeof(contextCallbacks));
    static_assert(0 == offsetof(PathOpsV0Lib::WindingCallbacks, windingAddFuncPtr));
	windingCallbacks.windingAddFuncPtr = (PathOpsV0Lib::WindingAdd) debugFindFunction(str);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingAddFuncPtr, windingKeepFuncPtr);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingKeepFuncPtr, windingVisibleFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingVisibleFuncPtr, windingZeroFuncPtr);
	DEBUG_FIND_FUNCTION(windingCallbacks, windingZeroFuncPtr, windingSubtractFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingSubtractFuncPtr, windingIntersectFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingIntersectFuncPtr, windingShortFuncPtr);
    DEBUG_FIND_FUNCTION(windingCallbacks, windingShortFuncPtr, windingShortAllFuncPtr);
    static_assert(offsetof(PathOpsV0Lib::WindingCallbacks, windingShortAllFuncPtr) 
            + sizeof(windingCallbacks.windingShortAllFuncPtr) == sizeof(windingCallbacks));
    // out of order ... but callbacks must be set before curves and windings are read
    debugCallbacksDumpSet(debugCallbacks, str);
    debugContextCallbacksDumpSet(debugContextCallbacks, str);
    ASSERT_SERIAL(*this, windingCallbacks, callerOutput);  // omit callerOutput
    ASSERT_SERIAL(*this, callerOutput, errorHandler);  // omit errorHandler
    ASSERT_SERIAL(*this, errorHandler, sortedContours);
    if (OpDebugOptional(str, "sortedContours")) {
        size = OpDebugReadSizeT(str);
        sortedContours.resize(size);
        for (auto& sortedContour : sortedContours) {
            sortedContour = (OpContour*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_SERIAL(*this, sortedContours, ccStorage);
    if (OpDebugOptional(str, "ccStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::cc);
    ASSERT_SERIAL(*this, ccStorage, curveDataStorage);
    if (OpDebugOptional(str, "curveDataStorage"))
        CurveDataStorage::DumpSet(str, &curveDataStorage);
    ASSERT_SERIAL(*this, curveDataStorage, contourStorage);
    if (OpDebugOptional(str, "contourStorage"))
        OpContourStorage::DumpSet(str, this);
    ASSERT_SERIAL(*this, contourStorage, contours);
    if (OpDebugOptional(str, "contours")) {
        size = OpDebugReadSizeT(str);
        contours.resize(size);
        for (auto& contour : contours) {
            contour = (OpContour*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_SERIAL(*this, contours, fillerStorage);
    if (OpDebugOptional(str, "fillerStorage"))
        OpEdgeStorage::DumpSet(str, this, DumpStorage::filler);
    ASSERT_SERIAL(*this, fillerStorage, sectStorage);
    if (OpDebugOptional(str, "sectStorage"))
        OpSectStorage::DumpSet(str, this);
    ASSERT_SERIAL(*this, sectStorage, limbStorage);
    if (OpDebugOptional(str, "limbStorage"))
        OpLimbStorage::DumpSet(str, this);
    ASSERT_SERIAL(*this, limbStorage, limbCurrent);  // omit limbCurrent
    ASSERT_SERIAL(*this, limbCurrent, callerStorage);
    if (OpDebugOptional(str, "callerStorage"))
        CallerDataStorage::DumpSet(str, &callerStorage);
    ASSERT_SERIAL(*this, callerStorage, userData);  // omit userData
    ASSERT_SERIAL(*this, userData, maxBounds);
    if (OpDebugOptional(str, "maxBounds"))
        maxBounds.dumpSet(str);
    ASSERT_SERIAL(*this, maxBounds, error);
    error = PathOpsV0Lib::contextErrorStr(str, "error", PathOpsV0Lib::ContextError::none);
    ASSERT_SERIAL(*this, error, uniqueID);
    OpDebugRequired(str, "uniqueID");
    uniqueID = (int) OpDebugReadSizeT(str);
    DEBUG_SET_BOOL(OpContext, uniqueID, initialized);
    DEBUG_SET_BOOL(OpContext, initialized, allDiscarded);
    DEBUG_SET_BOOL(OpContext, allDiscarded, allKept);
    DEBUG_SET_BOOL(OpContext, allKept, fatalError);
    DEBUG_SET_BOOL(OpContext, fatalError, outputOne);
    DEBUG_SET_BOOL(OpContext, outputOne, linkErased);
    DEBUG_SET_BOOL(OpContext, linkErased, windingSet);
#if OP_DEBUG_VALIDATE
    ASSERT_SERIAL_OFFSET(*this, windingSet, 1, debugValidateEdgeIndex);
    OpDebugRequired(str, "debugValidateEdgeIndex");
    debugValidateEdgeIndex = (int) OpDebugReadSizeT(str);
    ASSERT_SERIAL(*this, debugValidateEdgeIndex, debugValidateJoinerIndex);
    OpDebugRequired(str, "debugValidateJoinerIndex");
    debugValidateJoinerIndex = (int) OpDebugReadSizeT(str);
    ASSERT_SERIAL(*this, debugValidateJoinerIndex, debugCallbacks);
#else
    ASSERT_SERIAL(*this, dumpDummy, debugCallbacks);
#endif
// debug call backs must be set before segments' curves can be set
    ASSERT_SERIAL(*this, debugCallbacks, debugContextCallbacks);
    ASSERT_SERIAL(*this, debugContextCallbacks, debugContextData);  // omit debugContextData
    ASSERT_SERIAL(*this, debugContextData, debugData);  // omit debugData (!!! omit for now, may have uses...)
    ASSERT_SERIAL(*this, debugData, debugCurveCurve);
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
    if (OpDebugOptional(str, "debugTestname"))
        debugData.testname = OpDebugLabel(str);
    debugExpect = OpDebugExpectStr(str, "debugExpect", OpDebugExpect::fail);
    debugInPathOps = OpDebugOptional(str, "debugInPathOps");
    debugInClearEdges = OpDebugOptional(str, "debugInClearEdges");
    debugCheckLastEdge = OpDebugOptional(str, "debugCheckLastEdge");
    debugFailOnEqualCepts = OpDebugOptional(str, "debugFailOnEqualCepts");
	debugDumpInit = true;
}

void OpContext::dumpResolveAll(OpContext* self) {
    OP_ASSERT(this == self);
    for (auto& sortedContour : sortedContours) {
        self->dumpResolve(sortedContour);
    }
    if (ccStorage)
        ccStorage->dumpResolveAll(self);
    for (auto& contour : contours) {    // out of order: resolve contour array
        self->dumpResolve(contour);     //  before contents of contour storage
    }
    if (contourStorage)
        contourStorage->dumpResolveAll(self);
    if (fillerStorage)
        fillerStorage->dumpResolveAll(self);
    if (sectStorage)
        sectStorage->dumpResolveAll(self);
    if (limbStorage)
        limbStorage->dumpResolveAll(self);
#if OP_DEBUG
    if (debugCurveCurve)
        debugCurveCurve->dumpResolveAll(self);
    if (debugJoiner)
        debugJoiner->dumpResolveAll(self);
#endif
}

void dmpContext() {
    dmp(*debugGlobalContext);
}

void dmpContours() {
	std::string s;
	for (OpContour* contour : contourIterator) {
		s += contour->debugDump(defaultLevel, defaultBase) + "\n";
	}
	s.pop_back();
	OpDebugFormat(s);
}

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
        if (edge.curve.firstPt() != edge.iStart && edge.iStart == pt)
            OpDebugFormat("edge iStart: " + edge.debugDump(level, defaultBase) + "\n");
        if (edge.curve.lastPt() != edge.iEnd && edge.curve.lastPt() == pt)
            OpDebugFormat("edge iEnd: " + edge.debugDump(level, defaultBase) + "\n");
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

std::vector<const OpIntersection*> findCoincidence(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto c : contourIterator) {
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
    for (const auto c : contourIterator)
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
    return nullptr;
}

std::vector<const OpEdge*> findEdgeOutput(int ID) {
    std::vector<const OpEdge*> result;
    for (const auto c : contourIterator) {
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
    for (const auto c : contourIterator) {
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
    if (const OpLimb* limb = debugGlobalContext->limbStorage
            ? debugGlobalContext->limbStorage->debugFind(ID) : nullptr)
        return limb;
    return nullptr;
}

std::vector<const OpIntersection*> findSectUnsectable(int ID) {
    std::vector<const OpIntersection*> result;
    for (const auto c : contourIterator) {
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
    for (const auto c : contourIterator) {
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

const OpLimb& OpContext::debugNthLimb(int index) const {
    OpLimbStorage* saveCurrent = limbCurrent;
    OpContext* writeable = const_cast<OpContext*>(this);
    const OpLimb& result = writeable->nthLimb(index);
    writeable->limbCurrent = saveCurrent;
    return result;
}

enum class ShowContour {
	no,
	yes
};

static std::string segmentDebugDump(const OpSegment& seg, ShowContour showContour, 
		DebugLevel l, DebugBase b) {
	std::string lf = ShowContour::yes == showContour ? "\n" : " ";
    if (DebugLevel::brief != l) {
        std::string s = "[" + STR(seg.id) + "] ";
        static_assert(0 == offsetof(OpSegment, contour));
        if (ShowContour::yes == showContour && seg.contour)
            s += "contour[" + STR(seg.contour->id) + "] ";
        ASSERT_SERIAL(seg, contour, c);
        s += seg.c.debugDump(l, b) + lf;
        ASSERT_SERIAL(seg, c, ptBounds);
		if (ShowContour::yes == showContour)
			s += "ptBounds:" + seg.ptBounds.debugDump(l, b) + lf;
        ASSERT_SERIAL(seg, ptBounds, sects);
        if (!seg.sects.i.empty()) {
            s += "sects:" + STR(seg.sects.i.size()) + " [";
            for (auto sect : seg.sects.i)
                s += STR(sect->id) + " ";
            s.pop_back();
            s += "]" + lf;
        }
        ASSERT_SERIAL(seg, sects, edges);
        if (!seg.edges.empty()) {
            s += "edges:" + STR(seg.edges.size());
            if (DebugLevel::normal == l) {
                s += " [";
                for (auto& edge : seg.edges)
                    s += STR(edge.id) + " ";
                s.pop_back();
                s += "]" + lf;
            } else {
                s += "\n";
                for (auto& edge : seg.edges)
                    s += edge.debugDump(l, b) + "\n";
            }
        }
        ASSERT_SERIAL(seg, edges, winding);
        s += "winding:" + seg.winding.debugDump(l, b) + " ";
        ASSERT_SERIAL(seg, winding, id);  // write at front
        DEBUG_DUMP_BOOL_INST(seg, id, disabled);
        DEBUG_DUMP_BOOL_INST(seg, disabled, willDisable);
        DEBUG_DUMP_BOOL_INST(seg, willDisable, hasCoin);
        DEBUG_DUMP_BOOL_INST(seg, hasCoin, hasPals);
        DEBUG_DUMP_BOOL_INST(seg, hasPals, hasUnsectable);
        DEBUG_DUMP_BOOL_INST(seg, hasUnsectable, startMoved);
        DEBUG_DUMP_BOOL_INST(seg, startMoved, endMoved);
#if OP_DEBUG_IMAGE
        ASSERT_SERIAL_OFFSET(seg, endMoved, 1, debugColor);
        if (seg.debugColor != black)
            s += "debugColor:" + debugDumpColor(l, seg.debugColor) + " ";
        ASSERT_SERIAL(seg, debugColor, debugSetDisabled);
#else
        ASSERT_SERIAL_OFFSET(seg, endMoved, 1, debugSetDisabled);
#endif
#if OP_DEBUG_MAKER
        if (seg.debugSetDisabled.valid())
            s += "debugSetDisabled:" + seg.debugSetDisabled.debugDump() + " ";
        static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugSetDisabled) 
                + sizeof(seg.debugSetDisabled));
#elif OP_DEBUG_IMAGE
        static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugColor) 
                + sizeof(seg.debugColor));
#else
        static_assert(sizeof(OpSegment) == offsetof(OpSegment, endMoved) 
                + sizeof(seg.endMoved));
#endif
        s.pop_back();
        return s;
    }
    return "seg:" + STR(seg.id) + " " + seg.c.debugDump(l, b);
}

std::string OpContour::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "contour[" + STR(id) + "] ";
    if (DebugLevel::detailed == l) {
		auto contourExtra = debugCallbacks.debugDumpContourExtraFuncPtr;
		if (contourExtra)
			s += (*contourExtra)(debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], l, b) + " ";
	}
    static_assert(0 == offsetof(OpContour, segments));
    s += "segments:" + STR(segments.size()) + (DebugLevel::detailed == l ? "\n" : " ");
    if (DebugLevel::detailed != l && DebugLevel::file != l) {
        s += "[";
        for (auto& segment : segments)
            s += STR(segment.id) + " ";
        s.pop_back();
        s += "] ";
    } else {
		// limit segment to id / curve / sects / edges / winding / disabled
		// !!! add segment filter ala edges?  refactor this to call common code?
        for (auto& segment : segments) {
			s += segmentDebugDump(segment, ShowContour::yes, l, b);
			s += "\n";
		}
    }
	std::string closeBracket = DebugLevel::detailed == l ? "]\n" : "] ";
    ASSERT_SERIAL(*this, segments, sorted);
	if (!sorted.empty()) {
		s += "sorted:" + STR(sorted.size()) + " [";
		for (OpSegment* seg : sorted)
			s += STR(seg->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
	if (DebugLevel::detailed != l)
		s += "\n  ";
    ASSERT_SERIAL(*this, sorted, overlaps);
	if (!overlaps.empty()) {
		s += "overlaps:" + STR(overlaps.size()) + " [";
		for (OpContour* member : overlaps)
			s += STR(member->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, overlaps, merges);
	if (!merges.empty()) {
		s += "merges:" + STR(merges.size()) + " [";
		for (OpContour* member : merges)
			s += STR(member->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, merges, inX);
	if (!inX.empty()) {
		s += "inX:" + STR(inX.size()) + " [";
		for (OpEdge* e : inX)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, inX, inY);
	if (!inY.empty()) {
		s += "inY:" + STR(inY.size()) + " [";
		for (OpEdge* e : inY)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, inY, byArea);
	if (!byArea.empty()) {
		s += "byArea:" + STR(byArea.size()) + " [";
		for (OpEdge* e : byArea)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, byArea, unsectByArea);
	if (!unsectByArea.empty()) {
		s += "unsectByArea:" + STR(unsectByArea.size()) + " [";
		for (OpEdge* e : unsectByArea)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, unsectByArea, disabledBackwards);
	if (!disabledBackwards.empty()) {
		s += "disabledBackwards:" + STR(disabledBackwards.size()) + " [";
		for (OpEdge* e : disabledBackwards)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, disabledBackwards, disabledCenterless);
	if (!disabledCenterless.empty()) {
		s += "disabledCenterless:" + STR(disabledCenterless.size()) + " [";
		for (OpEdge* e : disabledCenterless)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, disabledCenterless, disabledPals);
	if (disabledPals.size()) {
		s += "disabledPals:" + STR(disabledPals.size()) + " [";
		for (OpEdge* e : disabledPals)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, disabledPals, unsortables);
	if (unsortables.size()) {
		s += "unsortables:" + STR(unsortables.size()) + " [";
		for (OpEdge* e : unsortables)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, unsortables, windingStorage);
    s += "windingStorage:" + STR(windingStorage.size()) + " ";
    s += OpDebugDumpByteArray(&windingStorage.front(), windingStorage.size()) + " ";
    ASSERT_SERIAL(*this, windingStorage, linkups);
	if (linkups.l.size()) {
		s += "linkups[";
		for (OpEdge* e : linkups.l)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, linkups, endLinks);
	if (endLinks.l.size()) {
		s += "endLinks[";
		for (OpEdge* e : endLinks.l)
			s += STR(e->id) + " ";
		s.pop_back();
		s += closeBracket;
	}
    ASSERT_SERIAL(*this, endLinks, overlapBounds);
	if (overlapBounds.isFinite())
		s += "overlapBounds:" + overlapBounds.debugDump(l, b) + " ";
    ASSERT_SERIAL(*this, overlapBounds, bounds);
	if (bounds.isFinite())
		s += "bounds:" + bounds.debugDump(l, b) + " ";
    ASSERT_SERIAL(*this, bounds, context);  // omit context
    ASSERT_SERIAL(*this, context, overlapOwner);
	if (overlapOwner) 
		s += "overlapOwner[" + STR(overlapOwner->id) + "] ";
    ASSERT_SERIAL(*this, overlapOwner, id);  // id written up front
    ASSERT_SERIAL(*this, id, treeID);
	if (treeID)
		s += "treeID[" + STR(treeID) + "] ";
    ASSERT_SERIAL(*this, treeID, winding);
#if 0  // for now, just write winding storage passed in at contour creation time
    if (winding.data && winding.size) {
		auto windingOut = context->debugContextCallbacks.debugDumpWindingOutFuncPtr;
		if (windingOut)
			s += "winding" + (*windingOut)(winding) + " ";
	}
#endif
    ASSERT_SERIAL(*this, winding, backwardsBuilt);
	if (backwardsBuilt)
		s += "backwardsBuilt ";
    ASSERT_SERIAL(*this, backwardsBuilt, centerlessBuilt);
	if (centerlessBuilt)
		s += "centerlessBuilt ";
    ASSERT_SERIAL(*this, centerlessBuilt, hasPals);
	if (hasPals)
		s += "hasPals ";
    ASSERT_SERIAL(*this, hasPals, palsBuilt);
	if (palsBuilt)
		s += "palsBuilt ";
    ASSERT_SERIAL(*this, palsBuilt, disabled);
	if (disabled)
		s += "disabled ";
    ASSERT_SERIAL(*this, disabled, overlapsMerged);
	if (overlapsMerged)
		s += "overlapsMerged ";
    ASSERT_SERIAL_OFFSET(*this, overlapsMerged, 2, debugCallbacks);
    s += debugFindTag(reinterpret_cast<DebugFunction>(debugCallbacks.debugDumpContourExtraFuncPtr));
#if OP_DEBUG_IMAGE
    DEBUG_FIND_TAG(debugCallbacks, debugDumpContourExtraFuncPtr, debugNativePathFuncPtr);
	DEBUG_FIND_TAG(debugCallbacks, debugNativePathFuncPtr, debugGetDrawFuncPtr);
	DEBUG_FIND_TAG(debugCallbacks, debugGetDrawFuncPtr, debugSetDrawFuncPtr);
	DEBUG_FIND_TAG(debugCallbacks, debugSetDrawFuncPtr, debugOperandFuncPtr);
#endif
    ASSERT_SERIAL(*this, debugCallbacks, debugContourData);  // omit debugContourData
#if OP_DEBUG_IMAGE
    ASSERT_SERIAL(*this, debugContourData, debugColor);
    if (debugColor != blue) {
        if (DebugLevel::file == l)
            s += "debugColor:";
        s += debugDumpColor(l, debugColor);
    }
#endif
    s.pop_back();
    return s;
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
    ASSERT_SERIAL(*this, segments, sorted);
    if (OpDebugOptional(str, "sorted")) {
        int count = (int) OpDebugReadSizeT(str);
        sorted.resize(count);
        for (OpSegment*& seg : sorted)
            seg = (OpSegment*) OpDebugReadSizeT(str);
    }
    ASSERT_SERIAL(*this, sorted, overlaps);
    if (OpDebugOptional(str, "overlaps")) {
        int count = (int) OpDebugReadSizeT(str);
        overlaps.resize(count);
        for (int index = 0; index < count; ++index)
            overlaps[index] = (OpContour*) OpDebugReadSizeT(str);
    }
    ASSERT_SERIAL(*this, overlaps, merges);
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
	DUMP_EDGES(*this, disabledPals, unsortables);
    ASSERT_SERIAL(*this, unsortables, windingStorage);
    OpDebugRequired(str, "windingStorage");
    size_t windingSize = OpDebugReadSizeT(str);
    windingStorage.resize(windingSize);
    OpDebugByteArray(str, windingSize, &windingStorage.front());
    DUMP_NAMED_EDGES(*this, windingStorage, "linkups", linkups.l);
    DUMP_NAMED_EDGES(*this, linkups, "endLinks", endLinks.l);
    ASSERT_SERIAL(*this, endLinks, overlapBounds);
    if (OpDebugOptional(str, "overlapBounds"))
        overlapBounds.dumpSet(str);
    ASSERT_SERIAL(*this, overlapBounds, bounds);
    if (OpDebugOptional(str, "bounds"))
        bounds.dumpSet(str);
    ASSERT_SERIAL(*this, bounds, context);  // omit context
    ASSERT_SERIAL(*this, context, overlapOwner);
    if (OpDebugOptional(str, "overlapOwner"))
        overlapOwner = (OpContour*) OpDebugReadSizeT(str);
    ASSERT_SERIAL(*this, overlapOwner, id);  // id written up front
    ASSERT_SERIAL(*this, id, treeID);
    if (OpDebugOptional(str, "treeID"))
        treeID = OpDebugReadSizeT(str);
    ASSERT_SERIAL(*this, treeID, winding);
#if 0  // maybe something like this is needed? 
    if (OpDebugOptional(str, "winding")) {
        auto windingSet = context->debugContextCallbacks.debugDumpWindingSetFuncPtr;
        if (!windingSet) {
            OpDebugOut("!missing debugDumpWindingSetFuncPtr\n");
            return;
        }
        (*windingSet)(str, winding);
    }
#else  // for now, set to value passed at contour allocation
    winding = { &windingStorage.front(), windingStorage.size() };
#endif
    ASSERT_SERIAL(*this, winding, backwardsBuilt);
    backwardsBuilt = OpDebugOptional(str, "backwardsBuilt");
    ASSERT_SERIAL(*this, backwardsBuilt, centerlessBuilt);
    centerlessBuilt = OpDebugOptional(str, "centerlessBuilt");
    ASSERT_SERIAL(*this, centerlessBuilt, hasPals);
    hasPals = OpDebugOptional(str, "hasPals");
    ASSERT_SERIAL(*this, hasPals, palsBuilt);
    palsBuilt = OpDebugOptional(str, "palsBuilt");
    ASSERT_SERIAL(*this, palsBuilt, disabled);
    disabled = OpDebugOptional(str, "disabled");
    ASSERT_SERIAL(*this, disabled, overlapsMerged);
    overlapsMerged = OpDebugOptional(str, "overlapsMerged");
    ASSERT_SERIAL_OFFSET(*this, overlapsMerged, 2, debugCallbacks);
    debugCallbacks.debugDumpContourExtraFuncPtr = (PathOpsV0Lib::DebugDumpContourExtra) debugFindFunction(str);
#if OP_DEBUG_IMAGE
    DEBUG_FIND_FUNCTION(debugCallbacks, debugDumpContourExtraFuncPtr, debugNativePathFuncPtr);
	DEBUG_FIND_FUNCTION(debugCallbacks, debugNativePathFuncPtr, debugGetDrawFuncPtr);
	DEBUG_FIND_FUNCTION(debugCallbacks, debugGetDrawFuncPtr, debugSetDrawFuncPtr);
	DEBUG_FIND_FUNCTION(debugCallbacks, debugSetDrawFuncPtr, debugOperandFuncPtr);
#endif
    ASSERT_SERIAL(*this, debugCallbacks, debugContourData);  // omit debugContourData
#if OP_DEBUG_IMAGE
    ASSERT_SERIAL(*this, debugContourData, debugColor);
    if (OpDebugOptional(str, "debugColor"))
        debugColor = OpDebugHexToInt(str);
#endif
}

#undef DUMP_EDGES
#undef DUMP_NAMED_EDGES

#define DUMP_RESOLVE_ARRAY(obj) \
    for (auto& o : obj) \
        c->dumpResolve(o)

void OpContour::dumpResolveAll(OpContext* c) {
    for (OpSegment& segment : segments)
        segment.dumpResolveAll(c);
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
	DUMP_RESOLVE_ARRAY(unsortables);
	DUMP_RESOLVE_ARRAY(linkups.l);
	DUMP_RESOLVE_ARRAY(endLinks.l);
    c->dumpResolve(overlapOwner);
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
    OpDebugOut(__func__ + std::string("missing contour"));
    exit(1);
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
    const OpContourStorage* block = this;
    while (index >= block->used) {
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
    int count = debugCount();
    if (!count)
        return s;
    s += "contourStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
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
    uint8_t* data = (uint8_t*) curveData;
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
    OpDebugByteArray(str, storage->used, storage->storage);
    if (storage->next)
        DumpSet(str, &storage->next);
}

std::string OpCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
	if ((size_t) c.type > context().debugCallbacks.size())
		s += "(missing curve name) ";
    else if (!c.type)
        s += "degenerateLine ";
    else {
		auto curveName = context().debugCallback(c).curveNameFuncPtr;
		if (curveName)
			s += (*curveName)() + " ";
	}
    if (DebugLevel::file == l) {
        s += "size:" + STR(c.size) + " ";
        s += "data:" + context().curveDataStorage->debugDump(c.data) + " ";
    } else {
		s.pop_back();  // remove trailing space
        s += "{";
        for (int i = 0; i < pointCount(); ++i) 
            s += hullPt(i).debugDump(DebugLevel::error, b) + ", ";
        s.pop_back(); s.pop_back();  // remove space, comma
        s += "}";
		if ((size_t) c.type <= context().debugCallbacks.size()) {
			auto curveExtra = context().debugCallback(c).curveExtraFuncPtr;
			if (curveExtra)
				s += (*curveExtra)(c, l, b);
		}
    }
    return s;
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

void OpCurve::dumpSet(const char*& str) {
    size_t strLen = 0;
    while (isalnum(str[strLen]))
        ++strLen;
    for (size_t index = 0; index < context().callbacks.size(); ++index) {
		auto curveName = context().debugCallbacks[index].curveNameFuncPtr;
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
    c.data = context().curveDataStorage->dumpSet(str);  // do not allocate, just point to
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

#define EDGE_FILTER \
	OP_X(segment) \
	OP_X(ray) \
	OP_X(priorEdge) \
	OP_X(nextEdge) \
	OP_X(lastEdge) \
	OP_X(center) \
	OP_X(curve) \
    OP_X(iStart) \
    OP_X(iEnd) \
	OP_X(vertical_impl) \
	OP_X(upright_impl) \
	OP_X(bounds) \
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
	OP_X(startDist) \
	OP_X(endDist) \
	OP_X(id) \
    OP_X(ccUnsectID) \
	OP_X(whichEnd_impl) \
	OP_X(rayFail) \
	OP_X(windZero) \
	OP_X(doSplit) \
	OP_X(isUnsortable) \
	OP_X(closeSet) \
	OP_X(active_impl) \
	OP_X(inLinkups) \
	OP_X(linkHead) \
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
	OP_X(Depth) \
	OP_X(RayMatch) \
	OP_X(Filler) \
	OP_X(Unordered) \
	OP_X(SumSet)

#define EDGE_IMAGE \
	OP_X(Color) \
	OP_X(Draw) \
	OP_X(Join) \
	OP_X(Limb) \
	OP_X(One)

#define EDGE_MAKER \
    OP_X(SetDisabled) \
	OP_X(SetMaker) \
	OP_X(SetSum)

#define EDGE_VALIDATE \
    OP_X(PriorID) \
    OP_X(ScheduledForErasure)

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
#if OP_DEBUG_VALIDATE
    #define OP_X(Field) \
        debug##Field,
        EDGE_VALIDATE
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
    std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, EF::iStart,
            EF::iEnd, EF::winding, EF::sum, EF::whichEnd_impl };
    OpSaveEF saveEF(showFields);
    dmpEdges();
}

void dmpPts(int ID) {
    if (findEdge(ID)) {
        std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, EF::iStart,
            EF::iEnd, EF::winding, EF::sum, EF::whichEnd_impl };
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

std::string EdgePal::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "edge[" + debugDumpID() + "] ";
	if (DebugLevel::detailed == l && edge->segment)
		s += "seg[" + STR(edge->segment->id) + "] ";
	if (edge->segment)
		s += "contour[" + STR(edge->segment->contour->id) + "] ";
    if (unsectID)
		s += "unsectID:" + STR(unsectID) + " ";
    if (reversed) 
        s += DebugLevel::brief != l ? "reversed " : "r ";
    if (DebugLevel::detailed == l)
		s += edge->debugDumpWinding() + " ";
    if (!s.empty())
        s.pop_back();
    return s;
}

std::string Distance::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "edge[" + debugDumpID() + "] ";
	if (DebugLevel::detailed == l && edge->segment)
		s += "seg[" + STR(edge->segment->id) + "] ";
	if (DebugLevel::file != l && edge->segment)
		s += "contour[" + STR(edge->segment->contour->id) + "] ";
    if (!OpMath::IsNaN(cept))
        s += debugValue(l, b, "cept", cept) + " ";
    if (!OpMath::IsNaN(edgeInsideT))
        s += debugValue(l, b, "edgeInsideT", edgeInsideT) + " ";
    if (RayOrder::uninitialized != rayOrder)
		s += "rayOrder:" + RayOrderName(rayOrder) + " ";
    if (reversed) 
        s += DebugLevel::brief != l ? "reversed " : "r ";
    if (dependent) 
        s += DebugLevel::brief != l ? "dependent " : "d ";
    if (over) 
        s += DebugLevel::brief != l ? "over " : "o ";
    if (DebugLevel::detailed == l)
		s += edge->debugDumpWinding() + " ";
    if (!s.empty())
        s.pop_back();
    return s;
}

std::string EdgePal::debugDumpID() const {
    std::string s;
    s += STR(edge->id);
    return s;
}

std::string Distance::debugDumpID() const {
    std::string s;
    s += STR(edge->id);
    return s;
}

void EdgePal::dumpSet(const char*& str) {
    OpDebugRequired(str, "edge");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "unsectID"))
        unsectID = (int) OpDebugReadSizeT(str);
}

void Distance::dumpSet(const char*& str) {
    OpDebugRequired(str, "edge");
    edge = (OpEdge*) OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "cept"))
        cept = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "edgeInsideT"))
        edgeInsideT = OpDebugHexToFloat(str);
    rayOrder = RayOrderStr(str, "rayOrder", RayOrder::uninitialized);
    reversed = OpDebugOptional(str, "reversed");
    dependent = OpDebugOptional(str, "dependent");
    over = OpDebugOptional(str, "over");
}

void Distance::dumpResolveAll(OpContext* context) {
    context->dumpResolve(edge);
}

std::string EdgeDist::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (isSet()) 
		s += "opp:" + opp.debugDump(l, b) + " ";
    s += debugValue(DebugLevel::error, b, "dist", dist) + " ";
    return s;
}

void EdgeDist::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "opp"))
        opp.dumpSet(str);
    if (OpDebugOptional(str, "dist"))
        dist = OpDebugHexToFloat(str);
}

std::string OpEdge::debugDump(DebugLevel l, DebugBase b) const {
    DebugLevel brief = DebugLevel::file != l ? DebugLevel::brief : DebugLevel::file;
    DebugLevel error = DebugLevel::file != l ? DebugLevel::error : DebugLevel::file;
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
    auto strFloat = [dumpIt, dumpAlways, b, error](EdgeFilter match, std::string label, float t) {
        if (!dumpIt(match) || (!dumpAlways(match) && !OpMath::IsFinite(t)))
            return std::string("");
        return debugValue(error, b, label, t) + " ";
    };
    auto strPoint = [dumpIt, dumpAlways, b, error, strLabel](EdgeFilter match, std::string label,
                OpPoint pt) {
        if (!dumpIt(match) || (!dumpAlways(match) && !pt.isFinite()))
            return std::string("");
        return strLabel(label) + pt.debugDump(error, b) + " ";
    };
    auto strPtT = [dumpIt, dumpAlways, b, error, strLabel](EdgeFilter match, std::string label,
                OpPtT ptT, std::string suffix) {
        if (!dumpIt(match) || (!dumpAlways(match) && (!ptT.pt.isFinite() || !OpMath::IsFinite(ptT.t))))
            return std::string("");
        return strLabel(label) + ptT.debugDump(error, b) + suffix;
    };
    auto strID = [dumpIt, dumpAlways, strLabel](EdgeFilter match, std::string label, int ID) {
        if (!dumpIt(match) || (!dumpAlways(match) && !ID))
            return std::string("");
        return strLabel(label) + "[" + STR(ID) + "] ";
    };
    auto strBounds = [dumpAlways, l, b, strLabel](EdgeFilter match, 
            std::string label, const OpPointBounds& ptBounds) {
        if (!dumpAlways(match) && !ptBounds.isSet())
            return std::string("");
        return strLabel(label) + ptBounds.debugDump(l, b)+ " ";
    };
    auto strWinding = [dumpAlways, l, b, strLabel](EdgeFilter match, std::string label,
             const OpWinding& wind) {
        if (!dumpAlways(match) && !wind.isSet())
            return std::string("");
        return strLabel(label) + ":" + wind.debugDump(l, b) + " ";
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
        s += ray.debugDump(brief, b) + " ";
    if (priorEdge || nextEdge || lastEdge || dumpAlways(EF::priorEdge) || dumpAlways(EF::nextEdge)) { 
        s += strEdge(EdgeFilter::priorEdge, "prior", priorEdge);
        s += strEdge(EdgeFilter::nextEdge, "next", nextEdge);
        s += strEdge(EdgeFilter::lastEdge, "last", lastEdge);
        if ('/' == s.back()) s.back() = ' ';
    }
	if (!center.debugIsUninitialized())
		s += strPtT(EdgeFilter::center, "center", center, " ");
    if (dumpIt(EdgeFilter::curve)) s += strCurve("curve", curve);
    if (iStart != curve.firstPt() || dumpAlways(EdgeFilter::iStart)) 
        s += strPoint(EdgeFilter::iStart, "iStart", iStart);
    if (iEnd != curve.lastPt() || dumpAlways(EdgeFilter::iEnd))
        s += strPoint(EdgeFilter::iEnd, "iEnd", iEnd);
    if (upright_impl.pts[0].isFinite() || upright_impl.pts[1].isFinite()) {
        if (dumpIt(EdgeFilter::upright_impl))
            s += strPts("upright_impl", upright_impl);
        if (dumpIt(EdgeFilter::vertical_impl))
            s += strCurve("vertical_impl", vertical_impl);
    }
    if (dumpIt(EdgeFilter::bounds)) s += strBounds(EdgeFilter::bounds, "bounds", bounds);
    if (dumpIt(EdgeFilter::linkBounds)) s += strBounds(EF::linkBounds, "linkBounds", linkBounds);
    if (dumpIt(EdgeFilter::winding)) s += strWinding(EdgeFilter::winding, "winding", winding);
    if (dumpIt(EdgeFilter::sum)) s += strWinding(EdgeFilter::sum, "sum", sum);
    if (dumpIt(EdgeFilter::many)) s += strWinding(EdgeFilter::many, "many", many);
    if (dumpIt(EdgeFilter::coinPals) && (dumpAlways(EdgeFilter::coinPals) || coinPals.size())) {
        s += strLabel("coinPals") + "{";
        for (auto& cPal : coinPals) {
            s += "{opp[" + STR(cPal.opp->id) + "] coinID[" + STR(cPal.coinID) + "]} ";
        }
        s.pop_back();
        s += "} ";
    }
    if (dumpIt(EdgeFilter::unSects) && (dumpAlways(EdgeFilter::unSects) || unSects.size())) {
        s += strLabel("unSects") + "[";
        for (auto& uSect : unSects)
            s += STR(uSect->id) + " ";
        s.pop_back();
        s += "] ";
    }
    if (dumpIt(EdgeFilter::pals) && (dumpAlways(EdgeFilter::pals) || pals.size())) {
        s += strLabel("pals") + "{";
        for (auto& pal : pals) {
            s += pal.debugDump(brief, b) + " ";
        }
        s.pop_back();
        s += "} ";
    }
    if (dumpIt(EdgeFilter::hulls) && (dumpAlways(EdgeFilter::hulls) || hulls.h.size())) {
        s += "hulls";
        if (DebugLevel::file == l)
            s += ":" + STR(hulls.h.size()) + " ";
        s += "{";  // don't abbreviate in brief
        for (auto& hs : hulls.h)
            s += hs.debugDump(l, b) + " ";
        if (' ' == s.back()) s.pop_back();
        s += "} ";
    }
    if (dumpIt(EdgeFilter::startDist) && (dumpAlways(EdgeFilter::startDist) || startDist.debugIsSet()))
        s += "startDist{" + startDist.debugDump(l, b) + "} ";
    if (dumpIt(EdgeFilter::endDist) && (dumpAlways(EdgeFilter::endDist) || endDist.debugIsSet()))
        s += "endDist{" + endDist.debugDump(l, b) + "} ";
    s += strFloat(EdgeFilter::startT, "startT", startT);
    s += strFloat(EdgeFilter::endT, "endT", endT);
    s += strID(EF::ccUnsectID, "ccUnsectID", ccUnsectID);
    s += strEnum(EF::whichEnd_impl, "whichEnd", EdgeMatch::none == which(), EdgeMatchName(which()));
    s += strEnum(EF::rayFail, "rayFail", EdgeFail::none == rayFail, EdgeFailName(rayFail));
    s += strEnum(EF::windZero, "windZero", WindZero::unset == windZero, WindZeroName(windZero));
    s += strEnum(EF::isUnsortable, "isUnsortable", Unsortable::none == isUnsortable, 
			UnsortableName(isUnsortable));
#define STR_BOOL(ef) do { if (dumpIt(EdgeFilter::ef) && (dumpAlways(EdgeFilter::ef) || ef)) { \
        s += strLabel(#ef) + " "; \
        if (1 != ((unsigned char) ef)) s += STR((size_t) ef) + " "; }} while(false)
	STR_BOOL(active_impl);
    STR_BOOL(inLinkups);
    STR_BOOL(linkHead);
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
    s += strID(EF::debugOutPath, "debugOutPath", debugOutPath);
    s += strID(EF::debugParentID, "debugParentID", debugParentID);
    s += strID(EF::debugDepth, "debugDepth", debugDepth);
    s += strID(EF::debugRayMatch, "debugRayMatch", debugRayMatch);
	STR_BOOL(debugUnordered);
	STR_BOOL(debugSumSet);
#endif
    // omit dumpContext
#if OP_DEBUG_IMAGE
    if (dumpIt(EF::debugColor) && (dumpAlways(EF::debugColor) || debugBlack != debugColor))
        s += "debugColor:" + debugDumpColor(l, debugColor) + " ";
    STR_BOOL(debugDraw);
    STR_BOOL(debugJoin);
    STR_BOOL(debugLimb);
    STR_BOOL(debugOne);
#endif
#if OP_DEBUG_MAKER
    if (dumpIt(EF::debugSetDisabled) && (dumpAlways(EdgeFilter::debugSetDisabled) 
            || debugSetDisabled.valid()))
        s += "debugSetDisabled:" + debugSetDisabled.debugDump() + " ";
    if (dumpIt(EF::debugSetMaker)) {
        if (DebugLevel::file == l)
            s += "debugSetMaker:";
        s += debugSetMaker.debugDump() + " ";
    }
    if (dumpIt(EF::debugSetSum) && (dumpAlways(EF::debugSetSum) || debugSetSum.valid()))
        s += "debugSetSum:" + debugSetSum.debugDump() + " ";
#endif
#if OP_DEBUG_VALIDATE
    s += strID(EF::debugPriorID, "debugPriorID", debugPriorID);
    STR_BOOL(debugScheduledForErasure);
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
    id = (int) strID("edge");
    segment = (OpSegment*) strID("segment");
    (void) strID("contour");  // can't do anything with this here
    ray.dumpSet(str);
    priorEdge = (OpEdge*) strID("prior");  // non-zero must be replaced with pointer later
    nextEdge = (OpEdge*) strID("next");
    lastEdge = (OpEdge*) strID("last");
    if (OpDebugOptional(str, "center"))
        center.dumpSet(str);
    OpDebugRequired(str, "curve");
    curve.c.context = (ContextPtr) dumpContext;
    curve.dumpSet(str);
    if (OpDebugOptional(str, "iStart"))
        iStart.dumpSet(str);
    if (OpDebugOptional(str, "iEnd"))
        iEnd.dumpSet(str);
    if (OpDebugOptional(str, "upright_impl")) {
        upright_impl.dumpSet(str);
        OpDebugRequired(str, "vertical_impl");
        vertical_impl.c.context = (ContextPtr) dumpContext;
        vertical_impl.dumpSet(str);
    }
    OpDebugRequired(str, "bounds");
    bounds.dumpSet(str);
    if (OpDebugOptional(str, "linkBounds"))
        linkBounds.dumpSet(str);
    if (OpDebugOptional(str, "winding"))
        winding.dumpSet(str);
    if (OpDebugOptional(str, "sum"))
        sum.dumpSet(str);
    if (OpDebugOptional(str, "many"))
        many.dumpSet(str);
    if (OpDebugOptional(str, "coinPals")) {
        coinPals.resize(OpDebugReadSizeT(str));
        for (auto& pal : coinPals) {
            pal.opp = (OpSegment*) strID("opp");
            pal.coinID = strID("coinID");
        }
    }
    if (OpDebugOptional(str, "unSects")) {
        unSects.resize(OpDebugReadSizeT(str));
        for (auto& unSect : unSects) {
            unSect = (OpIntersection*) OpDebugReadSizeT(str);
        }
    }
    if (OpDebugOptional(str, "pals")) {
        pals.resize(OpDebugReadSizeT(str));
        for (auto& pal : pals) {
            pal.dumpSet(str);
        }
    }
    if (OpDebugOptional(str, "hulls")) {
        hulls.h.resize(OpDebugReadSizeT(str));
        for (auto& hull : hulls.h)
            hull.dumpSet(str);
    }
    if (OpDebugOptional(str, "startDist"))
        startDist.dumpSet(str);
    if (OpDebugOptional(str, "endDist"))
        endDist.dumpSet(str);
    startT = OpDebugReadNamedFloat(str, "startT");
    endT = OpDebugReadNamedFloat(str, "endT");
    // id up front
    ccUnsectID = strID("ccUnsectID");
    whichEnd_impl = EdgeMatchStr(str, "whichEnd", EdgeMatch::none);
    rayFail = EdgeFailStr(str, "rayFail", EdgeFail::none);
    windZero = WindZeroStr(str, "windZero", WindZero::unset);
    isUnsortable = UnsortableStr(str, "unsortable", Unsortable::none);
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
    debugOutPath = (int) strID("debugOutPath");
    debugParentID = (int) strID("debugParentID");
    debugDepth = (int) strID("debugDepth");
    debugRayMatch = (int) strID("debugRayMatch");
    STR_BOOL(debugUnordered);
    STR_BOOL(debugSumSet);
#endif
    // omit dumpContext
#if OP_DEBUG_IMAGE
    if (OpDebugOptional(str, "debugColor"))
        debugColor = OpDebugHexToInt(str);
    STR_BOOL(debugDraw);
    STR_BOOL(debugJoin);
    STR_BOOL(debugLimb);
    STR_BOOL(debugOne);
#endif
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
    if (OpDebugOptional(str, "debugSetMaker"))
        debugSetMaker.dumpSet(str);
    if (OpDebugOptional(str, "debugSetSum"))
        debugSetSum.dumpSet(str);
#endif
#if OP_DEBUG_VALIDATE
    debugPriorID = (int) strID("debugPriorID");
    STR_BOOL(debugScheduledForErasure);
#endif
#undef STR_BOOL
}

void OpEdge::dumpResolveAll(OpContext* c) {
    c->dumpResolve(segment);
    ray.dumpResolveAll(c);
    c->dumpResolve(priorEdge);
    c->dumpResolve(nextEdge);
    c->dumpResolve(lastEdge);
    for (auto& unSect : unSects)
        c->dumpResolve(unSect);
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
	DebugLevel l = DebugLevel::detailed == defaultLevel ? DebugLevel::normal : defaultLevel;
    if (winding.isSet())
        s += "winding" + winding.debugDump(l, defaultBase) + " ";
    if (sum.isSet())
        s += "sum" + sum.debugDump(l, defaultBase) + " ";
    if (many.isSet())
        s += "many" + many.debugDump(l, defaultBase);
    return s;
}

void dmpWinding(const OpEdge& edge) {
    std::string s = edge.debugDumpWinding();
    OpDebugOut(s + "\n");
}

void dmpEnd(const OpEdge& edge)  {
    dmpMatch(edge.curve.lastPt());
    if (edge.iEnd != edge.curve.lastPt())
        dmpMatch(edge.iEnd);
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
    OpPoint c = { (bounds.left + bounds.right) / 2, (bounds.top + bounds.bottom) / 2 };
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
    if (iStart != curve.firstPt()) 
        s += "iStart:" + iStart.debugDump(defaultLevel, defaultBase);
    if (iEnd != curve.lastPt())
        s += "iEnd:" + iEnd.debugDump(defaultLevel, defaultBase);
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

void dmpCompare(OpPoint a, OpPoint b) {
    OpVector diff = a - b;
    OpVector threshold = debugGlobalContext->threshold();
    OpDebugOut("difference: " + STR(diff.dx / threshold.dx) + "x, " 
            + STR(diff.dy / threshold.dy) + "x\n");
}

void dmpCompare(const OpPtT& a, const OpPtT& b) {
    dmpCompare(a.pt, b.pt);
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

void dmpLink(const OpEdge& edge) {
    std::vector<EdgeFilter> showFields = { EF::id, EF::segment, EF::contour, 
			EF::priorEdge, EF::nextEdge, EF::lastEdge,
			EF::startT, EF::endT, 
			EF::active_impl, EF::inLinkups, EF::inOutput, EF::disabled, EF::isUnsplitable,
			EF::centerless };
    OpSaveEF saveEF(showFields);
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
    if (edge.iStart != edge.curve.firstPt())
        dmpMatch(edge.iStart);
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
    OpDebugByteArray(str, storage->used, storage->storage);
    if (storage->next)
        DumpSet(str, &storage->next);
}

// don't count curve that hasn't been built
int OpEdgeStorage::debugCount() {
	OpEdge* last = debugIndex(used - 1);
    int result = used - (PathOpsV0Lib::degenerateLine == last->curve.c.type);
    OpEdgeStorage* block = next;
    while (block) {
        result += block->used;
        block = block->next;
    }
    return result;
}

OpEdge* OpEdgeStorage::debugFind(int ID) {
	for (int index = 0; index < used; index++) {
		OpEdge& test = storage[index];
        if (test.id == ID ||
                test.debugOutPath == ID || test.debugRayMatch == ID)
            return &test;
	}
    if (!next)
        return nullptr;
    return next->debugFind(ID);
}

OpEdge* OpEdgeStorage::debugIndex(int index) {
    OpEdgeStorage* block = this;
    while (index >= block->used) {
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
    int count = debugCount();
    if (!count)
        return s;
    s = label + ":" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
	    for (int index = 0; index < count; index++) {
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

void OpEdgeStorage::DumpSet(const char*& str, OpContext* dumpContext, DumpStorage type) {
    size_t count = OpDebugReadSizeT(str);
    for (size_t index = 0; index < count; ++index) {
        OpEdge* edge = nullptr;
        // !!! hackery ahead: note that 'contours->allocateEdge(this)' won't compile
        if (DumpStorage::cc == type)
            edge = dumpContext->allocateEdge(dumpContext->ccStorage);
        else if (DumpStorage::filler == type)
            edge = dumpContext->allocateEdge(dumpContext->fillerStorage);
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
        debugIndex(index)->dumpResolveAll(c);
}

int OpLimbStorage::debugCount() const {
    int result = used;
    OpLimbStorage* block = nextBlock;
    while (nextBlock) {
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
    while (index > block->used) {
        index -= block->used;
        block = block->nextBlock;
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
        s.pop_back();
        s += "]";
#endif
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
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
	if (' ' == s.back())
		s.pop_back();
    s += "]";
	s += DebugLevel::detailed == l ? "\n" : " ";
    auto dumpEdgeIDs = [&s, l](const std::vector<OpEdge*>& edges, std::string name) {
        if (!edges.size())
            return;
		if (DebugLevel::detailed != l)
			s.back() = ' ';
        s += name + ":" + STR(edges.size()) + " [";
        for (auto e : edges)
            s += STR(e->id) + " ";
        s.pop_back();
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
            s.pop_back();
            s += "]\n";
        } else {
            s += "";
            s += "-- endLinks:" + STR(endLinks.l.size()) + "\n";
            s += endLinks.debugDump(l, b) + "\n";
        }
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
    s.pop_back();
    return s;
}

void OpJoiner::dumpSet(const char*& str) {
    OpDebugRequired(str, "bestGap");
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
#define LimbPass_Base
ENUM_NAME_STRUCT(LimbPass)

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
        s += " match:" + EdgeMatchName(match);
    if (EdgeMatch::none != lastMatch)
        s += " lastMatch:" + EdgeMatchName(lastMatch);
    if (LimbPass::none != treePass)
        s += " treePass:" + LimbPassName(treePass);
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

void OpLimb::dumpResolveAll(OpContext* c) {
    c->dumpResolve(edge);
    c->dumpResolve(lastLimbEdge);
    c->dumpResolve(parent);
    for (const OpLimb* limb : debugBranches)
        c->dumpResolve(limb);
}

void OpLimb::dumpSet(const char*& str) {
    OpDebugRequired(str, "id");
    id = (int) OpDebugReadSizeT(str);
    edge = (OpEdge*) (OpDebugOptional(str, "edge") ? OpDebugReadSizeT(str) : 0);
    if (OpDebugOptional(str, "bounds"))
        bounds.dumpSet(str);
    lastLimbEdge = (OpEdge*) (OpDebugOptional(str, "lastLimbEdge") ? OpDebugReadSizeT(str) : 0);
    parent = (const OpLimb*) (OpDebugOptional(str, "parent") ? OpDebugReadSizeT(str) : 0);
    if (OpDebugOptional(str, "lastPtT"))
        lastPtT.dumpSet(str);
    linkedIndex = (uint32_t) (OpDebugOptional(str, "linkedIndex") ? OpDebugReadSizeT(str) : OpMax);
    gapDistance = OpDebugReadNamedFloat(str, "gapDistance");
    closeDistance = OpDebugReadNamedFloat(str, "closeDistance");
    match = EdgeMatchStr(str, "match", EdgeMatch::none);
    lastMatch = EdgeMatchStr(str, "lastMatch", EdgeMatch::none);
    treePass = LimbPassStr(str, "treePass", LimbPass::unlinked);
    looped = OpDebugOptional(str, "looped");
    resetPass = OpDebugOptional(str, "resetPass");
    if (OpDebugOptional(str, "debugBranches")) {
        size_t count = OpDebugReadSizeT(str);
        for (size_t index = 0; index < count; ++index)
            debugBranches.push_back((OpLimb*) OpDebugReadSizeT(str));
    }
}

// !!! string gets truncated if it gets too big (don't know why) so output it in pieces for now...
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
        s += " limbPass:" + LimbPassName(limbPass);
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
	if (totalUsed)
		s += "\n";
    OpLimbStorage* saveCurrent = context->limbCurrent;
    for (int index = 0; index < totalUsed; ++index) {
        const OpLimb& limb = context->debugNthLimb(index);
		OpDebugFormat(s);
        s = limb.debugDumpIDs(l, true);
        s += " parent:" + (limb.parent ? limb.parent->debugDumpIDs(l, true) : "-");
        if (limb.debugBranches.size()) {
            s += " children:";
            for (OpLimb* child : limb.debugBranches) {
                s += child->debugDumpIDs(l, true) + " ";
            }
            s.pop_back();
        }
        s += " treePass:" + LimbPassName(limb.treePass) + "\n";
    }
	if ('\n' == s.back())
		s.pop_back();
    context->limbCurrent = saveCurrent;
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

void EdgeRun::dumpSet(const char*& str) {
    OpDebugRequired(str, "edgePtT:");
    edgePtT.dumpSet(str);
    OpDebugRequired(str, "oppPtT:");
    oppPtT.dumpSet(str);
    oppDist = OpDebugReadNamedFloat(str, "oppDist");
    if (OpDebugOptional(str, "fromFoundT"))
        fromFoundT = LimitFrom::yes;
    if (OpDebugOptional(str, "byZero"))
        byZero = true;
#if OP_DEBUG
    OpDebugRequired(str, "debugBetween");
//    debugBetween = (int) OpDebugReadSizeT(str);
#endif
#if OP_DEBUG_MAKER
    debugSetMaker.dumpSet(str);
#endif
}

std::string FoundLimits::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (parentEdge)
        s += "parentEdge[" + STR(parentEdge->id) + "] ";
    if (parentOpp)
        s += "parentOpp[" + STR(parentOpp->id) + "] ";
    s += "seg:" + seg.debugDump(l, b);
    s += " opp:" + opp.debugDump(l, b);
    if (LimitFrom::yes == fromFoundT)
        s += " fromFoundT";
    if (Unordered::yes == oppOutOfOrder)
        s += " oppOutOfOrder";
    if (LimitUsed::yes == used)
        s += " used";
    if (LimitMatch::yes == match)
        s += " match";
#if OP_DEBUG_MAKER
    s += " debugMaker:" + debugMaker.debugDump();
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
    fromFoundT = OpDebugOptional(str, "fromFoundT") ? LimitFrom::yes : LimitFrom::no;
    oppOutOfOrder = OpDebugOptional(str, "oppOutOfOrder") ? Unordered::yes : Unordered::no;
    used = OpDebugOptional(str, "used") ? LimitUsed::yes : LimitUsed::no;
    match = OpDebugOptional(str, "match") ? LimitMatch::yes : LimitMatch::no;
#if OP_DEBUG_MAKER
    OpDebugRequired(str, "debugMaker");
    debugMaker.dumpSet(str);
#endif
}

void FoundLimits::dumpResolveAll(OpContext* c) {
    c->dumpResolve(parentEdge);
    c->dumpResolve(parentOpp);
}

std::string SnipPtTs::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "seg:" + seg.debugDump(l, b);
    s += " opp:" + opp.debugDump(l, b);
    return s;
}

void SnipPtTs::dumpSet(const char*& str) {
    OpDebugRequired(str, "seg");
    seg.dumpSet(str);
    OpDebugRequired(str, "opp");
    opp.dumpSet(str);
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

std::string RayTarget::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	if (DebugLevel::detailed == l || DebugLevel::file == l)
		s += "contour[" + STR(contour->id) + "] bounds:" + bounds.debugDump(l, b) + " ";
	else
		s += STR(contour->id) + " ";
	return s;
}

void RayTarget::dumpSet(const char*& str) {
    OpDebugRequired(str, "contour");
    contour = (OpContour*) OpDebugReadSizeT(str);
    OpDebugRequired(str, "bounds");
    bounds.dumpSet(str);
}

void RayTarget::dumpResolveAll(OpContext* context) {
    context->dumpResolve(contour);
}

std::string RayTargets::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "t:" + STR(t.size()) + " [";
    for (const RayTarget& target : t) {
		if (DebugLevel::detailed == l)
			s += " ";  // indent
        s += target.debugDump(DebugLevel::detailed == l || DebugLevel::file == l ? l 
                : DebugLevel::brief, b) + " ";
	}
	s.pop_back();
	s += "]";
	s += DebugLevel::detailed == l ? "\n " : " ";
	if (edges && DebugLevel::file != l && edges->size()) {
		s += "edges:" + STR(edges->size()) +  " [";
		for (const OpEdge* edge : *edges) {
			s += STR(edge->id) + " ";
		}
		s.pop_back();
		s += "] ";
	}
	s += "chainBounds" + chainBounds.debugDump(l, b) + " ";
	if (SIZE_MAX != edgeIndex)
		s += "edgeIndex:" + STR(edgeIndex) +  " ";
	if (SIZE_MAX != index)
		s += "index:" + STR(index) + " ";
    if (DebugLevel::file == l && debugEdgesContour) {
        s += "debugEdgesContour:" + STR(debugEdgesContour->id) + " ";
        s += "debugEdgesAxis:" + AxisName(debugEdgesAxis) + " ";
    }
	s.pop_back();
	return s;
}

void RayTargets::dumpSet(const char*& str) {
    OpDebugRequired(str, "t");
    size_t size = OpDebugReadSizeT(str);
    t.resize(size);
    for (RayTarget& target : t) {
        target.dumpSet(str);
    }
    OpDebugRequired(str, "chainBounds");
    chainBounds.dumpSet(str);
    if (OpDebugOptional(str, "edgeIndex"))
        edgeIndex = OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "index"))
        index = OpDebugReadSizeT(str);
    if (OpDebugOptional(str, "debugEdgesContour")) {
        debugEdgesContour = (OpContour*) OpDebugReadSizeT(str);
        debugEdgesAxis = AxisStr(str, "debugEdgesAxis", Axis::neither);
    }
}

void RayTargets::dumpResolveAll(OpContext* context) {
    for (RayTarget& target : t) {
        target.dumpResolveAll(context);
    }
    if (debugEdgesContour) {
        context->dumpResolve(debugEdgesContour);
	    edges = Axis::horizontal == debugEdgesAxis 
                ? &debugEdgesContour->inX : &debugEdgesContour->inY;
    }
}

std::string SectRay::debugDumpHeader(DebugLevel l, DebugBase b) const {
    std::string s;
	if (homeTangent.isFinite())
		s += debugLabel(l, "homeTangent") + homeTangent.debugDump(l, b) + " ";
	if (OpMath::IsFinite(normal))
		s += debugValue(l, b, "normal", normal) + " ";
	if (OpMath::IsFinite(homeCept))
	    s += debugValue(l, b, "homeCept", homeCept) + " ";
	if (OpMath::IsFinite(homeT))
	    s += debugValue(l, b, "homeT", homeT) + " ";
	if (OpMath::IsFinite(interceptLimit))
	    s += debugValue(l, b, "interceptLimit", interceptLimit) + " ";
	if (.5 != mid)
	    s += debugValue(l, b, "mid", mid) + " ";
	if (.5 != midEnd)
	    s += debugValue(l, b, "midEnd", midEnd) + " ";
	if (Axis::neither != axis)
		s += "axis:" + AxisName(axis) + " ";
	if (sorted) s += "sorted ";
	if (' ' == s.back())
		s.pop_back();
	return s;
}

std::string SectRay::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "targets:" + targets.debugDump(l, b) + "\n";
    s += "distances:" + STR(distances.size()) + " ";
    for (const Distance& dist : distances) {
		if (DebugLevel::detailed == l)
			s += " ";  // indent
        s += dist.debugDump(DebugLevel::detailed == l || DebugLevel::file == l ? l 
                : DebugLevel::brief, b);
		s += DebugLevel::detailed == l || DebugLevel::file == l ? "\n" : " ";
	}
    s += "erased:" + STR(erased.size()) + " ";
    for (const Distance& erase : erased) {
		if (DebugLevel::detailed == l)
			s += " ";  // indent
        s += erase.debugDump(DebugLevel::detailed == l || DebugLevel::file == l ? l 
                : DebugLevel::brief, b);
		s += DebugLevel::detailed == l || DebugLevel::file == l ? "\n" : " ";
	}
    s += debugDumpHeader(l, b) + "\n ";  // indent
	s += DebugLevel::detailed == l || DebugLevel::file == l ? "\n" : " ";
	if (!s.empty())
		s.pop_back();
    return s;
}

void SectRay::dumpSet(const char*& str) {
    OpDebugRequired(str, "targets");
    targets.dumpSet(str);
    OpDebugRequired(str, "distances");
    size_t size = OpDebugReadSizeT(str);
    distances.resize(size);
    for (Distance& dist : distances)
        dist.dumpSet(str);
    OpDebugRequired(str, "erased");
    size = OpDebugReadSizeT(str);
    erased.resize(size);
    for (Distance& erase : erased)
        erase.dumpSet(str);
    if (OpDebugOptional(str, "homeTangent"))
        homeTangent.dumpSet(str);
    if (OpDebugOptional(str, "normal"))
        normal = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "homeCept"))
        homeCept = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "homeT"))
        homeT = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "interceptLimit"))
        interceptLimit = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "mid"))
        mid = OpDebugHexToFloat(str);
    if (OpDebugOptional(str, "midEnd"))
       midEnd = OpDebugHexToFloat(str);
    axis = AxisStr(str, "axis:", Axis::neither);
    sorted = OpDebugOptional(str, "sorted");
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

std::string SegPt::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "pt:" + pt.debugDump(l, b) + " ";
    s += "ptType:" + PtTypeName(ptType);
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
			EF::startDist, EF::endDist, EF::isUnsplitable,
            EF::ccEnd, EF::ccLarge, EF::ccOverlaps, EF::ccSmall, EF::ccStart,
            EF::hulls, 
            EF::debugSetMaker, EF::debugParentID };
    OpSaveEF saveEF(showFields);
    DebugLevel down1 = (DebugLevel) ((int) l - 1);
    for (auto& edge : c)
        s += edge->debugDump(down1, b) + "\n";
    for (auto& run : runs)
        s += run.debugDump(down1, b) + "\n";
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

void CcCurves::dumpResolveAll(OpContext* context) {
    for (auto& edge : c)
        context->dumpResolve(edge);
}

std::string OpCurveCurve::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    DebugLevel down1 = DebugLevel::file == l ? DebugLevel::file : (DebugLevel) ((int) l - 1);
    if (DebugLevel::file == l || !seg->edges.size())
        s += "seg:" + STR(seg->id) + " ";
    else  {
        const OpEdge* originalEdge = &seg->edges[0];
        s += "originalEdge:" + originalEdge->debugDump(down1, b) + "\n";
    }
    if (DebugLevel::file == l || !opp->edges.size())
        s += "opp:" + STR(opp->id) + "\n";
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
    for (const auto& snip : snips) {
        s += snip.debugDump(down1, b) + "\n";
    }
    s += "depth:" + STR(depth) + " ";
    s += "uniqueLimits:" + STR(uniqueLimits_impl) + " ";
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
#if OP_DEBUG_DUMP
    if (DebugLevel::file == l)
        s += "debugLocalCall:" + STR(debugLocalCall) + " ";
#endif
#if OP_DEBUG_VERBOSE
    if (DebugLevel::file == l) {
        if (dvDepthIndex.size()) {
            s += "dvDepthIndex[" + STR(dvDepthIndex.size()) + "\n";
            for (const DebugDepth& iDepth : dvDepthIndex)
                s += "{depth:" + STR(iDepth.depth) + " " + STR(iDepth.all) + "} ";
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
        if (dvRunIndex.size()) {
            s += "dvRunIndex[" + STR(dvRunIndex.size()) + "\n";
            for (const DebugRunSize& size : dvRunIndex)
                s += "{edgeRuns:" + STR(size.edgeRuns) + " oppRuns:" + STR(size.oppRuns) + "} ";
            s.pop_back();
            s += "] ";
        }
        if (dvRuns.size()) {
            s += "dvRuns:" + STR(dvRuns.size()) + "\n";
            for (const auto& run : dvRuns)
                s += run.debugDump(l, b) + "\n";
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
    if (OpDebugOptional(str, "snips")) {
        size_t count = OpDebugReadSizeT(str);
        snips.resize(count);
        for (size_t index = 0; index < count; ++index)
            snips[index].dumpSet(str);
    }
    OpDebugRequired(str, "matchRev");
    OpDebugRequired(str, "depth");
    depth = (int) OpDebugReadSizeT(str);
    uniqueLimits_impl = OpDebugReadNamedInt(str, "uniqueLimits");
    reversed = OpDebugOptional(str, "reversed");
    rotateFailed = OpDebugOptional(str, "rotateFailed");
    sectResult = OpDebugOptional(str, "sectResult");
    foundGap = OpDebugOptional(str, "foundGap");
    splitMid = OpDebugOptional(str, "splitMid");
#if OP_DEBUG_DUMP
    OpDebugRequired(str, "debugLocalCall");
    debugLocalCall = (int) OpDebugReadSizeT(str);
#endif
#if 0 && OP_DEBUG_VERBOSE  // out of date (update when needed)
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

void OpCurveCurve::dumpResolveAll(OpContext* c) {
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
            OpDebugFormat(e->debugDump(defaultLevel, defaultBase) + "\n");
        for (const auto e : oppCurves.c)
            OpDebugFormat(e->debugDump(defaultLevel, defaultBase) + "\n");
        OpDebugOut("edgeCurves.runs: " + STR(edgeCurves.runs.size()) + "\n");
        for (const auto& run : edgeCurves.runs)
            OpDebugFormat(run.debugDump(defaultLevel, defaultBase) + "\n");
        OpDebugOut("oppCurves.runs: " + STR(oppCurves.runs.size()) + "\n");
        for (const auto& run : oppCurves.runs)
            OpDebugFormat(run.debugDump(defaultLevel, defaultBase) + "\n");
        return;
    }
    if (dvDepthIndex[level].depth != level + 1)
        OpDebugOut("!mismatch:" + STR(dvDepthIndex[level].depth) + "!=" + STR(level + 1) + "\n");
    size_t lo = dvDepthIndex[level].all;
    size_t hi = (int) dvDepthIndex.size() <= level + 1 ? dvAll.size() : dvDepthIndex[level + 1].all;
    for (size_t index = lo; index < hi; ++index) {
        OpEdge* e = dvAll[index];
        OpDebugFormat(e->debugDump(defaultLevel, defaultBase) + "\n");
    }
    lo = dvRunIndex[level].edgeRuns;
    hi = dvRunIndex[level].oppRuns;
    OpDebugOut("edgeCurves.runs: " + STR(hi - lo) + "\n");
    for (size_t index = lo; index < hi; ++index) {
        const EdgeRun& run = dvRuns[index];
        OpDebugFormat(run.debugDump(defaultLevel, defaultBase) + "\n");
    }
    lo = hi;
    hi = (int) dvRunIndex.size() <= level + 1 ? dvRuns.size() : dvRunIndex[level + 1].edgeRuns;
    OpDebugOut("oppCurves.runs: " + STR(hi - lo) + "\n");
    for (size_t index = lo; index < hi; ++index) {
        const EdgeRun& run = dvRuns[index];
        OpDebugFormat(run.debugDump(defaultLevel, defaultBase) + "\n");
    }
}

void dmpDepth(int level) {
    OpCurveCurve* cc = debugGlobalContext->debugCurveCurve;
    if (!cc)
        return OpDebugOut("!debugGlobalContext->debugCurveCurve\n");
    cc->dumpDepth(level);
}

void OpCurveCurve::dumpDepth() {
    for (int level = 0; level <= (int) dvDepthIndex.size(); ++level) {
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

std::string OpIntersection::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "[" + debugDumpID() + "] ";
    if (DebugLevel::brief == l) {
        s += "{" + ptT.debugDump(l, b) + ", ";
        s += "seg:" + segment->debugDumpID();
        return s;
    }
    s += ptT.debugDump(id ? l : DebugLevel::error, b) + " ";   // !!! may be uninitialized?
#if OP_DEBUG
	BOOL_TO_STR(debugErased);
#endif
    std::string segmentID = segment ? segment->debugDumpID() : "-";
    const OpSegment* oppParent = opp ? opp->segment : nullptr;
    std::string oppID = opp ? opp->debugDumpID() : "-";
    std::string oppParentID = oppParent ? oppParent->debugDumpID() : "-";
    s += "segment:" + segmentID + " ";
    s += "opp/sect:" + oppParentID + "/" + oppID + " ";
    if (coincidenceID  OP_DEBUG_CODE(|| debugCoincidenceID)) {
        s += "coinID:" + STR(coincidenceID)  OP_DEBUG_CODE(+ "/" + STR(debugCoincidenceID));
        s += DebugLevel::file == l ? " coinEnd:" : " " ;
        s += MatchEndsName(coinEnd) + " ";
        s += CoinOppName(coinOpp) + " ";
    }
    if (unsectID) {
        s += "unsectID:" + STR(unsectID);
        s += DebugLevel::file == l ? " unsectEnd:" : " ";
        s += MatchEndsName(unsectEnd) + " ";
    }
    if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && !unsectID 
            && MatchEnds::none != coinEnd)
        s += "!!! (unexpected) " + MatchEndsName(coinEnd) + " ";
    if (!coincidenceID  OP_DEBUG_CODE(&& !debugCoincidenceID) && CoinOpp::yes == coinOpp)
        s += "!!! (unexpected) " + CoinOppName(coinOpp) + " ";
	BOOL_TO_STR(betweenCoins);
	BOOL_TO_STR(mergeProcessed);
	BOOL_TO_STR(moved);
	BOOL_TO_STR(collapsed);
	BOOL_TO_STR(ccSect);
	BOOL_TO_STR(ccUnsectable);
#if OP_DEBUG_MAKER
    s += debugSetMaker.debugDump() + " ";
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
        result += STR(debug_id) + " ";
        return result;
    };
    if (debugSrcID)
        s += edgeOrSegment(debugSrcID, "debugSrcID:");
    if (debugOppID)
        s += edgeOrSegment(debugOppID, "debugOppID:");
#endif
	s.pop_back();
    return s;
}

void OpIntersection::dumpSet(const char*& str) {
    id = (int) OpDebugReadSizeT(str);
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
    coinEnd = MatchEndsStr(str, "coinEnd", MatchEnds::none);
    unsectID = OpDebugOptional(str, "unsectID") ? readCoinID(str) : 0;
    unsectEnd = MatchEndsStr(str, "unsectEnd", MatchEnds::none);
    mergeProcessed = OpDebugOptional(str, "mergeProcessed");
    moved = OpDebugOptional(str, "moved");
    collapsed = OpDebugOptional(str, "collapsed");
#if OP_DEBUG_MAKER
    debugSetMaker.dumpSet(str);
#endif
#if OP_DEBUG
    debugSrcID = OpDebugOptional(str, "debugSrcID") ? (int) OpDebugReadSizeT(str) : 0;
    debugOppID = OpDebugOptional(str, "debugOppID") ? (int) OpDebugReadSizeT(str) : 0;
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
    int sectCount = (int) OpDebugReadSizeT(str);
    i.resize(sectCount);
    OpDebugRequired(str, "[");
    for (int index = 0; index < sectCount; index++) {
        i[index] = (OpIntersection*) OpDebugReadSizeT(str);
    }
    OpDebugRequired(str, "]");
}

int OpSectStorage::debugCount() const {
    int result = used;
    OpSectStorage* block = next;
    while (next) {
        result += block->used;
        block = block->next;
    }
    return result;
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

OpIntersection* OpSectStorage::debugIndex(int index) const {
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
    int count = debugCount();
    if (!count)
        return s;
    s += "sectStorage:" + STR(count) + "\n";
    if (DebugLevel::brief == l) {
        s += "[";
        for (int index = 0; index < count; ++index)
            s += STR(debugIndex(index)->id) + " ";
        s.pop_back();
        s += "]";
    } else {
        for (int index = 0; index < count; ++index)
            s += debugIndex(index)->debugDump(l, b) + "\n";
        s.pop_back();
    }
    return s;
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

std::string OpSegment::debugDump(DebugLevel l, DebugBase b) const {
	return segmentDebugDump(*this, ShowContour::yes, l, b);
}

void OpSegment::dumpSet(const char*& str) {
    id = (int) OpDebugReadSizeT(str);
    static_assert(0 == offsetof(OpSegment, contour));
    int contourID = OpDebugOptional(str, "contour[") ? (int) OpDebugReadSizeT(str) : 0;
    OpDebugExitOnFail("mismatched contour id", contourID == contour->id);
    ASSERT_SERIAL(*this, contour, c);
    c.c.context = (ContextPtr) contour->context;
    c.dumpSet(str);
    ASSERT_SERIAL(*this, c, ptBounds);
    OpDebugRequired(str, "ptBounds");
    ptBounds.dumpSet(str);
    ASSERT_SERIAL(*this, ptBounds, sects);
    if (OpDebugOptional(str, "sects:")) {
        int sectCount = (int) OpDebugReadSizeT(str);
        sects.i.resize(sectCount);
        for (int index = 0; index < sectCount; ++index) {
            sects.i[index] = (OpIntersection*) OpDebugReadSizeT(str);
        }
    }
    ASSERT_SERIAL(*this, sects, edges);
    if (OpDebugOptional(str, "edges:")) {
        int edgeCount = (int) OpDebugReadSizeT(str);
        edges.resize(edgeCount);
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpContext = contour->context;
        for (int index = 0; index < edgeCount; ++index)
            edges[index].dumpSet(str);
    }
    ASSERT_SERIAL(*this, edges, winding);
    OpDebugRequired(str, "winding");
    winding.dumpSet(str);
    ASSERT_SERIAL(*this, winding, id);  // write at front
    DEBUG_SET_BOOL(OpSegment, id, disabled);
    DEBUG_SET_BOOL(OpSegment, disabled, willDisable);
    DEBUG_SET_BOOL(OpSegment, willDisable, hasCoin);
    DEBUG_SET_BOOL(OpSegment, hasCoin, hasPals);
    DEBUG_SET_BOOL(OpSegment, hasPals, hasUnsectable);
    DEBUG_SET_BOOL(OpSegment, hasUnsectable, startMoved);
    DEBUG_SET_BOOL(OpSegment, startMoved, endMoved);

#if OP_DEBUG_IMAGE
    ASSERT_SERIAL_OFFSET(*this, endMoved, 1, debugColor);
	if (OpDebugOptional(str, "debugColor"))
		debugColor = (int) OpDebugReadSizeT(str);
#else
    ASSERT_SERIAL_OFFSET(*this, endMoved, 1, debugSetDisabled);
#endif
#if OP_DEBUG_MAKER
    if (OpDebugOptional(str, "debugSetDisabled"))
        debugSetDisabled.dumpSet(str);
    static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugSetDisabled) 
            + sizeof(debugSetDisabled));
#elif OP_DEBUG_IMAGE
    static_assert(sizeof(OpSegment) == offsetof(OpSegment, debugColor) 
            + sizeof(debugColor));
#else
    static_assert(sizeof(OpSegment) == offsetof(OpSegment, endMoved) 
            + sizeof(endMoved));
#endif
}

void OpSegment::dumpResolveAll(OpContext* context) {
//    context->dumpResolve(contour);
    context->contourStorage->debugCheck(contour);  // asserts and exists if missing
    for (auto& sect : sects.i)
        context->dumpResolve(sect);
    for (auto& edge : edges)
        edge.dumpResolveAll(context);
}

std::string OpSegment::debugDumpEdges() const {
    std::string s;
    for (auto& e : edges)
        s += e.debugDump(defaultLevel, defaultBase) + "\n";
    if (!s.empty())
        s.pop_back();
    return s;
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

DEBUG_DUMP_ID_DEFINITION(OpSegment, id)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { WindingType::w, #w }
#define WindingType_Base
ENUM_NAME_STRUCT(WindingType)

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { DebugWindingType::w, #w }
#define DebugWindingType_Base
ENUM_NAME_STRUCT(DebugWindingType)

std::string OpWinding::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (DebugLevel::file == l) {
		s += "w.size:" + STR(w.size) + " ";
        s += OpDebugDumpByteArray((const uint8_t*) w.data, w.size);
    } else {
		if (w.size) {
			auto windingOut = debugGlobalContext->debugContextCallbacks.debugDumpWindingOutFuncPtr;
			if (windingOut)
				s += (*windingOut)(w) + " ";
		}
    }
	if (DebugLevel::detailed == l || DebugLevel::file == l) {
		if (WindingType::uninitialized != type)
			s += "type:" + WindingTypeName(type) + " ";
		if (DebugWindingType::uninitialized != debugType)
			s += "debugType:" + DebugWindingTypeName(debugType) + " ";
	}
	if (' ' == s.back())
		s.pop_back();
    return s;
}

void OpWinding::dumpSet(const char*& str) {
    OpDebugRequired(str, "w.size");
    w.size = OpDebugReadSizeT(str);
    w.data = debugGlobalContext->allocateWinding(w.size);
    for (size_t index = 0; index < w.size; ++index) {
        ((uint8_t*) w.data)[index] = OpDebugByteToInt(str);
	}
    type = WindingTypeStr(str, "type", WindingType::uninitialized);
    debugType = DebugWindingTypeStr(str, "debugType", DebugWindingType::uninitialized);
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
			s += " [" + STR(linkup->id) + " ";
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
			s.pop_back();
			s += "] ";
		}
        if (hasNext)
            s += "(loop)";
        s += "\n";
    }
    return s;
}

#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { ChopUnsortable::w, #w }
#define ChopUnsortable_Base
ENUM_NAME_STRUCT(ChopUnsortable)

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

std::string HullSect::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    if (opp)
        s += "[" + STR(opp->id) + "] ";
    s += SectTypeAbbr(type, l);
    s += " sect:" + sect.debugDump(l, b);
	if (oppDist.isSet())
		s += " oppDist:" + oppDist.debugDump(l, b);
    return s;
}

void HullSect::dumpSet(const char*& str) {
    if (OpDebugOptional(str, "["))
        opp = (OpEdge*) OpDebugReadSizeT(str);
    else
        opp = nullptr;
    type = SectTypeStr(str, "", SectType::none);
    OpDebugRequired(str, ":");
    sect.dumpSet(str);
}

void HullSect::dumpResolveAll(OpContext* c) {
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
    if (DebugLevel::error != l && !pt.debugIsUninitialized() && !pt.isFinite() 
            && !OpMath::IsDebugNaN(t) && !OpMath::IsFinite(t))
        return "";
    std::string s;
    if (!pt.debugIsUninitialized())
        s += pt.debugDump(DebugLevel::error, b);
    if (!OpMath::IsDebugNaN(t))
        s += debugValue(DebugLevel::error, b, "t", t);
    return s;
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
    s.pop_back();
    if (',' == s.back()) s.pop_back();
    return s;
}
#endif

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
    s += "match:" + MatchEndsName(match) + " ";
    if (reversed)
        s += "reversed ";
    s.pop_back();
    return s;
}

void MatchReverse::dumpSet(const char*& str) {
    match = MatchEndsStr(str, "match", MatchEnds::none);
    reversed = OpDebugOptional(str, "reversed");
}

std::string OpRoots::debugDump(DebugLevel l, DebugBase b) const {
    std::string s = "count:" + STR(count()) + " ";
    for (int index = 0; index < count(); ++index)
        s += debugFloat(b, roots[index]) + ", ";
    s.pop_back();
    if (',' == s.back()) s.pop_back();
    return s;
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

std::string LinePts::debugDump(DebugLevel l, DebugBase b) const {
    return "0:" + pts[0].debugDump(l, b) + ", 1:" + pts[1].debugDump(l, b);
}

void LinePts::dumpSet(const char*& str) {
    OpDebugRequired(str, "0:");
    pts[0].dumpSet(str);
    OpDebugRequired(str, "1:");
    pts[1].dumpSet(str);
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

void dmpColor(uint32_t c) {
    OpDebugOut(debugDumpColor(DebugLevel::normal, c) + "\n");
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
