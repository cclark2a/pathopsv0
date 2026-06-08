// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef OpDebugSerialize_DEFINED
#define OpDebugSerialize_DEFINED

#if OP_DEBUG_SERIALIZE

struct OpContour;
struct OpEdge;
struct OpIntersection;
struct OpLimb;
struct OpSegment;

enum class DebugBase {
    dec,
    hex,
	hexdec,
};

enum class DebugLevel {
	brief,
	normal,
    ray,  // normal + extra linefeeds for distance array
	detailed,
	file,
    error      // displays uninitialized and error conditions like nan and infinities
};

inline const std::string DumpFile = "Dump";  // filenames are "Dump#.txt" where # is 1 or greater
inline const std::string TestInFile = "TestIn.txt";

typedef void (*DebugFunction)();

#if OP_DEBUG_GLOBALS
extern std::string DebugDump(int id, DebugLevel l, DebugBase b);
#endif

extern std::string debugFindTag(DebugFunction );
extern std::string dmpFileToPath(std::string filename);
extern std::string dmpFileToStr(std::string filename);
extern std::string debugFloat(DebugBase , float );
extern std::string debugFloat(DebugLevel , float );
extern std::string debugPopMatching(std::string& s, char match);
extern bool debugIfMatching(std::string& s, char match);
extern std::string debugValue(DebugLevel l, DebugBase b, std::string label, float value);
extern std::vector<const OpIntersection*> findCoincidence(int id);
extern const OpContour* findContour(int id);
extern OpEdge* findEdge(int id);
extern std::vector<const OpEdge*> findEdgeRayMatch(int id);
extern const OpIntersection* findIntersection(int id);
extern const OpLimb* findLimb(int id);
extern std::vector<const OpIntersection*> findMerge(int id);
extern std::vector<const OpIntersection*> findSectUnsectable(int id);
extern const OpSegment* findSegment(int id);

extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

// !!! add max # of lines to help debugger out until debugger text issues are solved
// add edge formatting that puts distance entries on separate lines
// add ray to debugger to trigger this formatting, and draw ray, if edge is selected in text view
extern std::string stringFormat(std::string , int lineWidth, int maxLines = 0); // = 0 to disable

namespace PathOpsV0Lib {

enum class ContextError;

struct ContextErrorName {
    ContextError element;
    std::string name;
};

extern std::vector<ContextErrorName> contextErrorNames;

}

#define DEBUG_DUMP \
OP_X(ID) \

#define EDGE_OR_SEGMENT_DETAIL \
OP_X(Edges) \
OP_X(End) \
OP_X(Full) \
OP_X(Intersections) \
OP_X(Start) \

#define EDGE_DETAIL \
OP_X(Center) \
OP_X(Hulls) \
OP_X(Link) \
OP_X(Points) \
OP_X(Ray) \
OP_X(Winding)

#define DUMP_DECLARATIONS \
std::string debugDump(DebugLevel , DebugBase ) const; \
std::string debugDumpID() const;

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
	OP_X(bounds) \
	OP_X(linkBounds) \
	OP_X(winding) \
	OP_X(sum) \
	OP_X(palMany) \
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
	OP_X(unsortable) \
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
	OP_X(endSeen) \
    OP_X(unsummable) \

#define EDGE_VIRTUAL \
    OP_X(contour)

#define EDGE_DEBUG \
	OP_X(Match) \
	OP_X(ZeroErr) \
	OP_X(OutPath) \
	OP_X(ParentID) \
	OP_X(Depth) \
	OP_X(CC) \
	OP_X(RayMatch) \
	OP_X(Filler) \
	OP_X(Unordered) \
	OP_X(SumSet)

#define EDGE_DUMP \
	OP_X(Join) \
	OP_X(Limb) \
	OP_X(Released)

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
#if OP_DEBUG_DUMP
    #define OP_X(Field) \
        debug##Field,
        EDGE_DUMP
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

typedef EF EdgeFilter;

struct EdgeFilters {
    std::vector<EdgeFilter> filter;
    std::vector<EdgeFilter> always;
};

struct EdgeFilterName {
    EdgeFilter field;
    const char* name;
};

// use static asserts throughout to ensure that all of context is serialized
#define ASSERT_SERIAL_OFFSET(inst, last, offset, thisField) \
    static_assert(offsetof(std::remove_reference_t<decltype(inst)>, last) + sizeof((inst).last) \
            + offset == offsetof(std::remove_reference_t<decltype(inst)>, thisField))

#define ASSERT_SERIAL(instance, lastField, thisField) \
    ASSERT_SERIAL_OFFSET(instance, lastField, 0, thisField)

#define ASSERT_FIRST(firstField) \
    static_assert(0 == offsetof(std::remove_reference_t<decltype(*this)>, firstField))

#define ASSERT_LAST_OFFSET(lastField, offset) \
    static_assert(sizeof(*this) == offsetof(std::remove_reference_t<decltype(*this)>, lastField) \
            + sizeof(lastField) + offset)

#define ASSERT_LAST(lastField) \
    ASSERT_LAST_OFFSET(lastField, 0)

#define ASSERT_ORDERED(lastField, thisField) \
    ASSERT_SERIAL(*this, lastField, thisField)

#define ASSERT_ORDERED_OFFSET(lastField, thisField, offset) \
    ASSERT_SERIAL_OFFSET(*this, lastField, offset, thisField)

#define DEBUG_DUMP_BOOL(lastField, thisBool) \
    ASSERT_ORDERED(lastField, thisBool); \
    if (thisBool) s += #thisBool " "

// !!! replace with debug dump bool
#define BOOL_TO_STR(data) if (data) s += #data + std::string(" ")

// macro checks that function ptrs are consecutive
#define DEBUG_FIND_TAG(callback, lastField, thisField) \
    ASSERT_SERIAL(callback, lastField, thisField); \
    s += debugFindTag(reinterpret_cast<DebugFunction>(callback.thisField))

// macro checks that function ptrs are consecutive
#define DEBUG_FIND_FUNCTION(callback, lastField, thisField) \
    ASSERT_SERIAL(callback, lastField, thisField); \
    callback.thisField = (decltype(callback.thisField)) debugFindFunction(str)

#define DEBUG_DUMP_FLOAT(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    if (!OpMath::IsDebugNaN(thisFloat)) \
        s += debugValue(DebugLevel::error, b, #thisFloat, thisFloat) + " "

#define DEBUG_DUMP_START_REQUIRED_FLOAT(thisFloat) \
    s += debugValue(DebugLevel::error, b, #thisFloat, thisFloat) + " "

#define DEBUG_DUMP_REQUIRED_FLOAT(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    DEBUG_DUMP_START_REQUIRED_FLOAT(thisFloat)

#define DEBUG_DUMP_START_REQUIRED_DOUBLE(thisFloat) \
    s += debugValue(DebugLevel::error, b, #thisFloat, (float) thisFloat) + " "

#define DEBUG_DUMP_REQUIRED_DOUBLE(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    DEBUG_DUMP_START_REQUIRED_DOUBLE(thisFloat)

#define DEBUG_DUMP_ID(lastField, thisID) \
    ASSERT_ORDERED(lastField, thisID); \
    if (thisID) s += #thisID ":" + STR(thisID->id) + " "

#define DEBUG_DUMP_COMMON_STRUCT(thisStruct) \
    s += #thisStruct ":" + thisStruct.debugDump(l, b) + "\n"

#define DEBUG_DUMP_FIRST_STRUCT(thisStruct) \
    std::string s; \
    ASSERT_FIRST(thisStruct); \
    DEBUG_DUMP_COMMON_STRUCT(thisStruct)

#define DEBUG_DUMP_STRUCT_OFFSET(lastField, thisStruct, offset) \
    ASSERT_ORDERED_OFFSET(lastField, thisStruct, offset); \
    DEBUG_DUMP_COMMON_STRUCT(thisStruct)

#define DEBUG_DUMP_STRUCT(lastField, thisStruct) \
    DEBUG_DUMP_STRUCT_OFFSET(lastField, thisStruct, 0)

#define DEBUG_DUMP_LAST_STRUCT(lastField, thisStruct) \
    DEBUG_DUMP_STRUCT(lastField, thisStruct); \
    ASSERT_LAST(hi); \
    return s

#define DEBUG_DUMP_OPTIONAL_STRUCT(lastField, thisStruct, condition) \
    ASSERT_ORDERED(lastField, thisStruct); \
    if (condition) \
        DEBUG_DUMP_COMMON_STRUCT(thisStruct)

#define DEBUG_DUMP_OPTIONAL_COMMON_ID(thisValue) \
    if (thisValue) \
        s += #thisValue ":" + STR(thisValue->id) + " "

#define DEBUG_DUMP_OPTIONAL_ID(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_DUMP_OPTIONAL_COMMON_ID(thisValue)

#define DEBUG_DUMP_OPTIONAL_POS_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (thisValue >= 0) \
        s += #thisValue ":" + STR(thisValue) + " "

#define DEBUG_DUMP_OPTIONAL_FINITE_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (!OpMath::IsDebugNaN(thisValue)) \
        s += #thisValue ":" + OpDebugDumpHex(thisValue) + " (" + STR(thisValue) + ") "

#define DEBUG_DUMP_PUBLIC_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    s += #thisValue ":" + DebugDump(thisValue, l, b) + "\n";

#define DEBUG_DUMP_OPTIONAL_PUBLIC_VALUE(lastField, thisValue, condition) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (condition) \
        s += DebugDump(thisValue, l, b) + "\n";

#define DEBUG_DUMP_OPTIONAL_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (thisValue) \
        s += #thisValue ":" + STR(thisValue) + " "

#define DEBUG_DUMP_START_REQUIRED_VALUE(thisValue) \
    s += #thisValue ":" + STR(thisValue) + " "

#define DEBUG_DUMP_REQUIRED_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_DUMP_START_REQUIRED_VALUE(thisValue)

#define DEBUG_DUMP_COMMON_VECTOR(thisVector) \
    do { \
    if (thisVector.size()) { \
        s += #thisVector ":" + STR(thisVector.size()) + "\n"; \
        for (const auto& member : thisVector) { \
            s += member.debugDump(l, b) + "\n"; \
        } \
    } \
    } while (false)

#define DEBUG_DUMP_FIRST_VECTOR(thisVector) \
    ASSERT_FIRST(thisVector); \
    DEBUG_DUMP_COMMON_VECTOR(thisVector)

#define DEBUG_DUMP_VECTOR_OFFSET(lastField, thisVector, offset) \
    do { \
    ASSERT_ORDERED_OFFSET(lastField, thisVector, offset); \
    DEBUG_DUMP_COMMON_VECTOR(thisVector); \
    } while (false)

#define DEBUG_DUMP_VECTOR(lastField, thisVector) \
    DEBUG_DUMP_VECTOR_OFFSET(lastField, thisVector, 0)

#define DEBUG_DUMP_PUBLIC_VECTOR(thisVector) \
    do { \
    if (!thisVector.empty()) { \
        s += #thisVector ":" + STR(thisVector.size()) + " "; \
        for (const auto& member : thisVector) { \
            s += DebugDump(member, l, b) + "\n"; \
        } \
    } \
    } while (false)

#define DEBUG_DUMP_VECTOR_IDS(lastField, thisVector) \
    do { \
    ASSERT_ORDERED(lastField, thisVector); \
    if (!thisVector.empty()) { \
        s += #thisVector ":" + STR(thisVector.size()) + " ["; \
        for (const auto& member : thisVector) { \
            s += STR(member->id) + " "; \
        } \
        s.pop_back(); \
        s += "] "; \
    } \
    } while (false)

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
}

#endif

#endif
