// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

namespace PathOpsV0Lib {
struct AddCurve;
struct Curve;
}


enum class DumpSerialization {
    dummy
};

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


#define DUMP_DECLARATIONS_OVERRIDE \
std::string debugDump(DebugLevel , DebugBase ) const override; \
void dump() const override; \
void dump(DebugLevel, DebugBase ) const override; \
void dumpBrief() const override; \
void dumpDetailed() const override; \
void dumpHex() const override; \
void dumpSet(const char*& ) override;

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
OP_X(OpPointBounds) \
OP_X(OpPtAliases) \
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
OP_X(Aliases) \
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

#define DEBUG_DUMP \
OP_X(ID) \

#define DEBUG_DUMP_ID_DEFINITION(OWNER, ID) \
	std::string OWNER::debugDumpID() const { \
		return std::to_string(ID); \
	}

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

#define DUMP_BY_DUMPID \
OP_X(dmp, dump) \
OP_X(dmpid, dump) \
OP_X(dmpHex, dumpHex)

#define OP_X(Global, Method) \
	extern void Global(int id);
DUMP_BY_DUMPID
#undef OP_X

#define DUMP_BY_ID \
OP_X(Brief) \
OP_X(Detailed) \
OP_X(Hex)

namespace PathOpsV0Lib {
struct CurveCallbacks;
}

extern void dmpFile(OpContext* context, std::string filename);
extern OpContext* fromFile(std::string filename);
extern void verifyFile(OpContext* , std::string fromFilename, std::string verifyFilename);

#if OP_DEBUG_VERBOSE
extern void dmpDepth(int level);  // curve-curve intermediate edges created at some recursive depth
extern void dmpDepth();  // curve-curve intermediate edges at all depths 
#endif

extern std::vector<const OpIntersection*> findCoincidence(int id);
extern const OpContour* findContour(int id);
extern OpEdge* findEdge(int id);
extern std::vector<const OpEdge*> findEdgeRayMatch(int id);
extern const OpIntersection* findIntersection(int id);
extern const OpLimb* findLimb(int id);
extern std::vector<const OpIntersection*> findSectUnsectable(int id);
extern const OpSegment* findSegment(int id);

enum class DebugBase {
    dec,
    hex,
	hexdec,
};

enum class DebugLevel {
	brief,
	normal,
	detailed,
	file,
    error      // displays uninitialized and error conditions like nan and infinities
};

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
extern std::string debugDumpColor(DebugLevel, uint32_t c);
extern void dmpColor(uint32_t );
extern void dmpColor(const OpEdge* );
extern void dmpColor(const OpEdge& );
extern std::string dmpFileToPath(std::string name);
extern std::string dmpFileToStr(std::string name);
extern void dmpFilters();  // returns current filter settings
extern void dmpHex(float );
extern void dmpHex(uint32_t );
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
	OP_X(CC) \
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

typedef EF EdgeFilter;

struct OpSaveEF {
    OpSaveEF(std::vector<EdgeFilter>& temp);
    ~OpSaveEF();
    std::vector<EdgeFilter> save;
};

extern void addAlways(EdgeFilter);
extern void clearAlways(EdgeFilter);
extern void addFilter(EdgeFilter);
extern void clearFilter(EdgeFilter);

// if output matches note, use asterisks to make that output stand out  // !!! only partially implemented
extern void addNote(int id);
extern void addNote(std::string );
extern void addNote(float );
extern void addNote(OpPoint );
extern void addNote(OpVector );
extern void addNote(OpPtT );
extern void clearNotes();

// if output matches skip, omit it in the output  // !!! only partially implemented
extern void addSkip(int id);
extern void addSkip(std::string );
extern void addSkip(float );
extern void addSkip(OpPoint );
extern void addSkip(OpVector );
extern void addSkip(OpPtT );
extern void clearSkips();

// !!! working around laptop compiler bug; testing new w/o breaking old...
extern void dp(const OpEdge* );
extern void dp(const OpEdge& );
extern void dp(int id);

#define df(x) dmpFull(x)

// expand this as the need arises

extern std::string debugContext;
extern void debug();  // set debug bitmap to start and dump state using current context

// used by new interface

typedef void (*DebugFunction)();
extern std::string debugFindTag(DebugFunction function);
extern DebugFunction debugFindFunction(const char*& tag);
extern std::string debugValue(DebugLevel l, DebugBase b, std::string label, float value);
extern bool debugDmpIsLine(const PathOpsV0Lib::AddCurve& c);
extern bool debugDmpIsLine(const PathOpsV0Lib::Curve& c);
extern std::string stringFormat(OpContext* context, std::string s, int lineWidth);

enum class LimbPass : uint8_t;

extern std::string debugLimbPass(LimbPass pass);

struct OpSaveDump {
    OpSaveDump(DebugLevel l, DebugBase b);
    ~OpSaveDump();

    DebugLevel saveL;
    DebugBase saveB;
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

#define DEBUG_SET_BOOL(lastField, thisBool) \
    ASSERT_ORDERED(lastField, thisBool); \
    thisBool = OpDebugOptional(str, #thisBool)

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

#define DEBUG_SET_FLOAT(lastField, thisFloat) \
    ASSERT_ORDERED(lastField, thisFloat); \
    thisFloat = OpDebugReadNamedFloat(str, #thisFloat)

#define DEBUG_DUMP_ID(lastField, thisID) \
    ASSERT_ORDERED(lastField, thisID); \
    if (thisID) s += #thisID ":" + STR(thisID->id) + " "

#define DEBUG_SET_ID(lastField, thisID) \
    ASSERT_ORDERED(lastField, thisID); \
    if (OpDebugOptional(str, #thisID)) \
        thisID = (decltype(thisID)) OpDebugReadSizeT(str)

#define DEBUG_DUMP_COMMON_STRUCT(thisStruct) \
    s += #thisStruct ":" + thisStruct.debugDump(l, b) + "\n"

#define DEBUG_DUMP_FIRST_STRUCT(thisStruct) \
    std::string s; \
    ASSERT_FIRST(thisStruct); \
    DEBUG_DUMP_COMMON_STRUCT(thisStruct)

#define DEBUG_DUMP_STRUCT(lastField, thisStruct) \
    ASSERT_ORDERED(lastField, thisStruct); \
    DEBUG_DUMP_COMMON_STRUCT(thisStruct)

#define DEBUG_DUMP_LAST_STRUCT(lastField, thisStruct) \
    DEBUG_DUMP_STRUCT(lastField, thisStruct); \
    ASSERT_LAST(hi); \
    return s

#define DEBUG_SET_COMMON_STRUCT(thisStruct) \
    OpDebugRequired(str, #thisStruct); \
    thisStruct.dumpSet(str)

#define DEBUG_SET_FIRST_STRUCT(thisStruct) \
    ASSERT_FIRST(thisStruct); \
    DEBUG_SET_COMMON_STRUCT(thisStruct)

#define DEBUG_SET_STRUCT(lastField, thisStruct) \
    ASSERT_ORDERED(lastField, thisStruct); \
    DEBUG_SET_COMMON_STRUCT(thisStruct)

#define DEBUG_SET_LAST_STRUCT(lastField, thisStruct) \
    DEBUG_SET_STRUCT(lastField, thisStruct); \
    ASSERT_LAST(thisStruct)

#define DEBUG_DUMP_OPTIONAL_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (thisValue) \
        s += #thisValue ":" + STR(thisValue) + " "

#define DEBUG_SET_OPTIONAL_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    if (OpDebugOptional(str, #thisValue)) \
        thisValue = (decltype(thisValue)) OpDebugReadSizeT(str)

#define DEBUG_DUMP_START_REQUIRED_VALUE(thisValue) \
    s += #thisValue ":" + STR(thisValue) + " "

#define DEBUG_DUMP_REQUIRED_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_DUMP_START_REQUIRED_VALUE(thisValue)

#define DEBUG_SET_START_REQUIRED_VALUE(thisValue) \
    OpDebugRequired(str, #thisValue); \
    thisValue = (decltype(thisValue)) OpDebugReadSizeT(str)

#define DEBUG_SET_REQUIRED_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    DEBUG_SET_START_REQUIRED_VALUE(thisValue)

#define DEBUG_DUMP_VECTOR_OFFSET(lastField, thisVector, offset) \
    do { \
    ASSERT_ORDERED_OFFSET(lastField, thisVector, offset); \
    if (thisVector.size()) { \
        s += #thisVector ":" + STR(thisVector.size()) + " "; \
        for (const auto& member : thisVector) { \
            s += member.debugDump(l, b) + "\n"; \
        } \
    } \
    } while (false)

#define DEBUG_DUMP_VECTOR(lastField, thisVector) \
    DEBUG_DUMP_VECTOR_OFFSET(lastField, thisVector, 0)

#define DEBUG_SET_VECTOR_OFFSET(lastField, thisVector, offset) \
    do { \
    ASSERT_ORDERED_OFFSET(lastField, thisVector, offset); \
    if (OpDebugOptional(str, #thisVector)) { \
        size_t count = OpDebugReadSizeT(str); \
        thisVector.resize(count); \
        for (auto& member : thisVector) \
            member.dumpSet(str); \
    } \
    } while (false)

#define DEBUG_SET_VECTOR(lastField, thisVector) \
    DEBUG_SET_VECTOR_OFFSET(lastField, thisVector, 0)

#define DEBUG_DUMP_VECTOR_IDS(lastField, thisVector) \
    do { \
    ASSERT_ORDERED(lastField, thisVector); \
    if (thisVector.size()) { \
        s += #thisVector ":" + STR(thisVector.size()) + " ["; \
        for (const auto& member : thisVector) { \
            s += STR(member->id) + " "; \
        } \
        s.pop_back(); \
        s += "] "; \
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

#endif

#endif
