// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#include "OpDebugSerialize.h"

#if OP_DEBUG_DUMP || OP_DEBUGGER

constexpr uint32_t debugBlack = 0xFF000000;
extern std::vector<std::pair<uint32_t, std::string>> debugColorArray;

namespace PathOpsV0Lib {
struct AddCurve;
struct Curve;
struct Winding;
}

enum class DumpSerialization {
    dummy
};

#undef DUMP_DECLARATIONS
#define DUMP_DECLARATIONS \
std::string debugDump(DebugLevel , DebugBase ) const; \
std::string debugDumpID() const; \
void dump() const; \
void dump(DebugLevel, DebugBase ) const; \
void dumpBrief() const; \
void dumpDetailed() const; \
void dumpHex() const; \
void dumpResolveAll(OpContext* ); \
void dumpSet(const char*& );

// removed OP_X(ExtremaT) for now
// removed OP_X(LoopCheck) for now
#define VECTOR_STRUCTS \
OP_X(CoinPair) \
OP_X(EdgeRun) \
OP_X(EdgePal) \
OP_X(FoundEdge) \
OP_X(FoundLimits) \
OP_X(HullSect) \
OP_X(OpContour) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpPtT) \
OP_X(OpSegment)

#define STRUCT_ID \
OP_X(CoinPair) \
OP_X(OpContour) \
OP_X(OpEdge)

#define STRUCT_NO_ID \
OP_X(EdgeRun) \
OP_X(FoundLimits) \
OP_X(HullSect) \
OP_X(OpPtT)

#define OP_STRUCTS \
OP_X(CcCurves) \
OP_X(CoinEnd) \
OP_X(LinePts) \
OP_X(LinkUps) \
OP_X(OpContext) \
OP_X(OpCurve) \
OP_X(OpCurveCurve) \
OP_X(OpEdgeStorage) \
OP_X(OpHulls) \
OP_X(OpIntersections) \
OP_X(OpJoiner) \
OP_X(OpLimb) \
OP_X(OpPoint) \
OP_X(OpRect) \
OP_X(OpRoots) \
OP_X(OpTree) \
OP_X(OpVector) \
OP_X(OpWinding) \
OP_X(SectRay) \
OP_X(SegPt)

#define OP_X(Thing) \
	struct Thing;
	VECTOR_STRUCTS
#undef OP_X

#define VECTOR_PTRS \
OP_X(OpContour*) \
OP_X(OpEdge*) \
OP_X(const OpEdge*) \
OP_X(OpIntersection*) \
OP_X(const OpIntersection*) \
OP_X(OpSegment*)

#define OP_X(Thing) \
	extern void dmp(const std::vector<Thing>* ); \
	extern void dmp(const std::vector<Thing>& ); \
	extern void dmpHex(const std::vector<Thing>* ); \
	extern void dmpHex(const std::vector<Thing>& ); \
	extern void dmpIDs(const std::vector<Thing>* ); \
	extern void dmpIDs(const std::vector<Thing>& ); \
	extern std::string debugDumpID(Thing);
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X

#define OP_X(Thing) \
	extern void dmp(const struct Thing* ); \
	extern void dmp(const struct Thing& ); \
	extern void dmpHex(const struct Thing* );
	VECTOR_STRUCTS
	OP_STRUCTS
#undef OP_X

#define OP_X(Thing) \
	extern void dmpHex(const struct Thing& );
	VECTOR_STRUCTS
#undef OP_X

#define DUMP_GROUP \
OP_X(Active) \
OP_X(Coincidences) \
OP_X(Coins) \
OP_X(Context) \
OP_X(Contours) \
OP_X(Disabled) \
OP_X(Edges) \
OP_X(EdgePts) \
OP_X(File) \
OP_X(InOutput) \
OP_X(Intersections) \
OP_X(Join) \
OP_X(Links) \
OP_X(Rays) \
OP_X(Sects) \
OP_X(Segments) \
OP_X(Sorted) \
OP_X(Tree) \
OP_X(Unsectable) \
OP_X(Unsortable) \
OP_X(Windings)

extern std::string debugDmpJoin(OpContext* , DebugLevel l, DebugBase b);
extern std::string debugDmpLinks(OpContext* , DebugLevel l, DebugBase b);

#define OP_X(Thing) \
	extern void dmp##Thing();
	DUMP_GROUP
#undef OP_X

#define DUMP_POINT \
OP_X(Match)

#define DETAIL_POINTS \
OP_X(Match, Intersection) \
OP_X(Match, Point) \
OP_X(Match, PtT)

#define OP_X(Thing, Struct) \
extern void dmp##Thing(const Op##Struct* ); \
extern void dmp##Thing(const Op##Struct& );
DETAIL_POINTS
#undef OP_X

extern void dmpMatchEnd(int id);
extern void dmpMatchStart(int id);

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpEdge* ); \
extern void dmp##Thing(const OpEdge& );
DEBUG_DUMP
EDGE_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#define SEGMENT_DETAIL \
OP_X(SegmentEdges) \
OP_X(SegmentIntersections) \
OP_X(SegmentSects)

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpSegment* ); \
extern void dmp##Thing(const OpSegment& );
DEBUG_DUMP
SEGMENT_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#define OP_X(Thing) \
extern void dmp##Thing(const OpIntersection& );
EDGE_OR_SEGMENT_DETAIL
SEGMENT_DETAIL
#undef OP_X

// !!! eventually, macro-itize this:
extern void dmpFull(const OpIntersection* );
extern void dmpFull(const OpIntersection& );
extern void dmpEnd(const OpIntersection& sect);
extern void dmpStart(const OpIntersection& sect);

#define DUMP_BY_ID \
OP_X(Brief) \
OP_X(Detailed) \
OP_X(Hex)

namespace PathOpsV0Lib {
struct CurveCallbacks;
}

extern OpContext* fromFile(std::string filename);
extern void verifyFile(OpContext* , std::string fromFilename, std::string verifyFilename);

#if OP_DEBUG_VERBOSE
extern void dmpDepth(int level);  // curve-curve intermediate edges created at some recursive depth
extern void dmpDepth();  // curve-curve intermediate edges at all depths 
#endif

// for typing in immediate window as parameters to dmpBase
// commented out here to avoid declaration shadowing, but defined for real at bottom of cpp file
// const int dec = 0;
// const int hex = 1;
// const int hexdec = 2;

// for typing in immediate window as parameters to dmpLevel
// commented out here to avoid declaration shadowing, but defined for real at bottom of cpp file
// const int brief = 0;
// const int normal = 1;
// const int detailed = 2;

extern void dmp(const PathOpsV0Lib::AddCurve& );
extern void dmp(const PathOpsV0Lib::AddCurve* );
extern void dmp(const PathOpsV0Lib::Curve& );
extern void dmp(const PathOpsV0Lib::Curve* );
extern void dmpBase(int );  // set to dec, hex, hexdec
extern void dmpCompare(OpPoint , OpPoint );  // show threshold difference between points
extern void dmpCompare(const OpPtT& , const OpPtT& );
extern void dmpClosest(const OpCurveCurve& , const OpPoint& );
extern void dmp(std::array<CoinEnd, 4>& );
extern void dmp(int );
extern std::string debugDumpColor(DebugLevel, uint32_t c);
extern void dmpColor(uint32_t );
extern void dmpColor(const OpEdge* );
extern void dmpColor(const OpEdge& );
extern void dmpFilters();  // returns current filter settings
extern void dmpToHex(float );
extern void dmpToHex(uint32_t );
extern void dmpLevel(int level);  // set to brief, normal, detailed
extern void dmpPlayback(FILE* );
extern void dmpPts(int ID);
extern void dmpPts(const OpEdge* );
extern void dmpPts(const OpEdge& );
extern void dmpPts(const OpSegment* );
extern void dmpPts(const OpSegment& );
extern void dmpRecord(FILE* );
extern void dmpT(int ID, float t);
extern void dmpT(const OpEdge* s, float t);
extern void dmpT(const OpSegment* s, float t);
extern void dmpWidth(int );  // max chars before inserting linefeed

struct OpSaveEF {
    OpSaveEF(std::vector<EdgeFilter>& temp);
    ~OpSaveEF();
    std::vector<EdgeFilter> save;
};

extern void addAlways(EdgeFilter);
extern void clearAlways(EdgeFilter);
extern void addFilter(EdgeFilter);
extern void clearFilter(EdgeFilter);

// !!! working around laptop compiler bug; testing new w/o breaking old...
extern void dp(const OpEdge* );
extern void dp(const OpEdge& );
extern void dp(int id);

#define df(x) dmpFull(x)

// expand this as the need arises

extern std::string debugContext;
extern void debug();  // set debug bitmap to start and dump state using current context

// used by new interface

extern DebugFunction debugFindFunction(const char*& tag);
extern bool debugDmpIsLine(const PathOpsV0Lib::AddCurve& );
extern bool debugDmpIsLine(const PathOpsV0Lib::Curve& );
extern void DumpSet(PathOpsV0Lib::Winding&, char const*& str);
extern void DumpResolveAll(PathOpsV0Lib::Winding& , OpContext* );

enum class LimbPass : int8_t;

extern std::string debugLimbPass(LimbPass pass);

struct OpSaveDump {
    OpSaveDump(DebugLevel l, DebugBase b);
    ~OpSaveDump();

    DebugLevel saveL;
    DebugBase saveB;
};

#define DEBUG_SET_BOOL(lastField, thisBool) \
    ASSERT_ORDERED(lastField, thisBool); \
    thisBool = OpDebugOptional(str, #thisBool)

#define DEBUG_SET_FLOAT(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    thisFloat = OpDebugReadNamedFloat(str, #thisFloat)

#define DEBUG_SET_START_REQUIRED_FLOAT(thisFloat) \
    OpDebugRequired(str, #thisFloat); \
    thisFloat = OpDebugReadNamedFloat(str, #thisFloat)

#define DEBUG_SET_REQUIRED_FLOAT(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    DEBUG_SET_START_REQUIRED_FLOAT(thisFloat)

#define DEBUG_SET_ID(lastField, thisID) \
    ASSERT_ORDERED(lastField, thisID); \
    if (OpDebugOptional(str, #thisID)) \
        thisID = (decltype(thisID)) OpDebugReadSizeT(str)

#define DEBUG_SET_COMMON_STRUCT(thisStruct) \
    OpDebugRequired(str, #thisStruct); \
    thisStruct.dumpSet(str)

#define DEBUG_SET_FIRST_STRUCT(thisStruct) \
    ASSERT_FIRST(thisStruct); \
    DEBUG_SET_COMMON_STRUCT(thisStruct)

#define DEBUG_SET_STRUCT_OFFSET(lastField, thisStruct, offset) \
    ASSERT_ORDERED_OFFSET(lastField, thisStruct, offset); \
    DEBUG_SET_COMMON_STRUCT(thisStruct)

#define DEBUG_SET_STRUCT(lastField, thisStruct) \
    DEBUG_SET_STRUCT_OFFSET(lastField, thisStruct, 0)

#define DEBUG_SET_LAST_STRUCT(lastField, thisStruct) \
    DEBUG_SET_STRUCT(lastField, thisStruct); \
    ASSERT_LAST(thisStruct)

#define DEBUG_SET_OPTIONAL_COMMON_ID(thisValue) \
    if (OpDebugOptional(str, #thisValue)) \
        thisValue = (decltype(thisValue)) OpDebugReadSizeT(str)

#define DEBUG_SET_OPTIONAL_ID(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_SET_OPTIONAL_COMMON_ID(thisValue)

#define DEBUG_SET_OPTIONAL_FINITE_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (OpDebugOptional(str, #thisValue)) { \
		thisValue = (decltype(thisValue)) OpDebugHexToFloat(str); \
		while (*str && ')' != *str++) \
			; \
	}

#define DEBUG_SET_PUBLIC_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (OpDebugRequired(str, #thisValue)) \
        DumpSet(thisValue, str);

#define DEBUG_SET_OPTIONAL_PUBLIC_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (OpDebugOptional(str, #thisValue)) \
        DumpSet(thisValue, str);

#define DEBUG_SET_OPTIONAL_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (OpDebugOptional(str, #thisValue)) \
        thisValue = (decltype(thisValue)) OpDebugReadSizeT(str)

#define DEBUG_SET_START_REQUIRED_VALUE(thisValue) \
    OpDebugRequired(str, #thisValue); \
    thisValue = (decltype(thisValue)) OpDebugReadSizeT(str)

#define DEBUG_SET_REQUIRED_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_SET_START_REQUIRED_VALUE(thisValue)

#define DEBUG_SET_COMMON_VECTOR(thisVector) \
    do { \
    if (OpDebugOptional(str, #thisVector)) { \
        size_t count = OpDebugReadSizeT(str); \
        thisVector.resize(count); \
        for (auto& member : thisVector) \
            member.dumpSet(str); \
    } \
    } while (false)

#define DEBUG_SET_FIRST_VECTOR(thisVector) \
    ASSERT_FIRST(thisVector); \
    DEBUG_SET_COMMON_VECTOR(thisVector)

#define DEBUG_SET_VECTOR_OFFSET(lastField, thisVector, offset) \
    do { \
    ASSERT_ORDERED_OFFSET(lastField, thisVector, offset); \
    DEBUG_SET_COMMON_VECTOR(thisVector); \
    } while (false)

#define DEBUG_SET_VECTOR(lastField, thisVector) \
    DEBUG_SET_VECTOR_OFFSET(lastField, thisVector, 0)

#define DEBUG_SET_PUBLIC_VECTOR(thisVector) \
    do { \
    if (OpDebugOptional(str, #thisVector)) { \
        size_t count = OpDebugReadSizeT(str); \
        thisVector.resize(count); \
        for (auto& member : thisVector) \
            DumpSet(member, str); \
    } \
    } while (false)

#define DEBUG_SET_VECTOR_IDS(lastField, thisVector) \
    do { \
    ASSERT_ORDERED(lastField, thisVector); \
    if (OpDebugOptional(str, #thisVector)) { \
        size_t count = OpDebugReadSizeT(str); \
        thisVector.resize(count); \
        for (auto& member : thisVector) { \
            member = (std::remove_reference_t<decltype(member)>) OpDebugReadSizeT(str); \
        } \
    } \
    } while (false)

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
enum enum##Str(const char*& str, const char* label, enum enumDefault) { \
    while ('{' == str[0]) \
        ++str; \
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

#endif

#endif
