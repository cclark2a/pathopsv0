// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpJoiner.h"
#include "OpTightBounds.h"
#include <vector>
#if OP_DEBUG
#include <atomic>
#endif

enum class EdgeMatch : int8_t;
struct FoundEdge;
struct OpContourIterator;
struct OpContourStorage;
struct OpCurveCurve;
struct OpJoiner;

struct OpContours;
struct OpInPath;

struct CallerDataStorage {
	CallerDataStorage()
		: next(nullptr)
		, used(0) {
		OP_DEBUG_CODE(memset(storage, 0, sizeof(storage)));
	}

	static char* Allocate(size_t size, CallerDataStorage** );
#if OP_DEBUG_DUMP
	static void DumpSet(const char*& str, CallerDataStorage** previousPtr);
	DUMP_DECLARATIONS
#endif

	CallerDataStorage* next;
	size_t used;
	char storage[2048];	// !!! size is arbitrary guess -- should measure and do better
};

struct OpContour {
	void addCallerData(PathOpsV0Lib::AddContour callerData);
	OpIntersection* addEdgeSect(const OpPtT& , OpSegment* seg
		   OP_LINE_FILE_DEF(const OpEdge* edge, const OpEdge* oEdge));
	OpIntersection* addCoinSect(const OpPtT& , OpSegment* seg, int cID, MatchEnds 
			OP_LINE_FILE_DEF(const OpSegment* oSeg));
	OpIntersection* addSegSect(const OpPtT& , OpSegment* seg
			OP_LINE_FILE_DEF(const OpSegment* oSeg));
	OpIntersection* addUnsect(const OpPtT& , OpSegment* seg, int uID, MatchEnds 
			OP_LINE_FILE_DEF(const OpSegment* oSeg));
	void addLine(OpPoint pts[2]);
	bool addQuad(OpPoint pts[3]);

	void apply() {
		for (auto& segment : segments) {
			segment.apply();
		}
	}

	void betweenIntersections() {
		for (auto& segment : segments) {
			segment.betweenIntersections();
		}
	}

	void findMissingEnds() {
		for (auto& segment : segments) {
			segment.findMissingEnds();
		}
	}

	void makeCoins() {
		for (auto& segment : segments) {
			segment.makeCoins();
		}
	}

	void makeEdges() {
		for (auto& segment : segments) {
			segment.makeEdges();
		}
	}

	void makePals() {
		for (auto& segment : segments) {
			segment.makePals();
		}
	}

	int nextID() const;

	void normalize() {
		for (auto& segment : segments) {
			segment.normalize();
		}
	}

#if 0  // !!! disable until use case appears
	void setBounds() {
		for (auto& segment : segments) {
			ptBounds.add(segment.ptBounds);
		}
	}
#endif

	void transferCoins() {
		for (auto& segment : segments) {
			segment.transferCoins();
		}
	}

#if OP_DEBUG
	void debugComplete();
#endif
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
	#define OP_X(Thing) \
	void dump##Thing() const;
	SEGMENT_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
#endif

	OpContours* contours;
	std::vector<OpSegment> segments;
	PathOpsV0Lib::ContourCallBacks callBacks;
	PathOpsV0Lib::CallerData caller;  // note: must use std::memcpy before reading
	OP_DEBUG_CODE(int id);
};

struct OpContourStorage {
	OpContourStorage()
		: next(nullptr)
		, used(0) {
	}

#if OP_DEBUG_DUMP
	size_t debugCount() const;
	OpContour* debugFind(int id) const;
	OpContour* debugIndex(int index) const;
	static void DumpSet(const char*& , OpContours* );
	DUMP_DECLARATIONS
#endif

	OpContourStorage* next;
	OpContour storage[2];
	int used;
};

struct OpContourIter {
	OpContourIter()
		: storage(nullptr)
		, contourIndex(0) {
	}

	OpContourIter(OpContours* contours);
	
	bool operator!=(OpContourIter rhs) { 
		return storage != rhs.storage || contourIndex != rhs.contourIndex; 
	}

	OpContour* operator*() {
		OP_ASSERT(storage && contourIndex < storage->used);
		return &storage->storage[contourIndex]; 
	}

	void operator++() {
		OP_ASSERT(storage && contourIndex < storage->used); 
		if (++contourIndex >= storage->used) {
			contourIndex = 0;
			storage = storage->next;
		}
	}

	void back() {
		OP_ASSERT(storage);
		while (OpContourStorage* next = storage->next) {
			storage = next;
		}
		OP_ASSERT(storage->used);
		contourIndex = storage->used - 1;
	}

	OpContourStorage* storage;
	int contourIndex;
};

struct OpContourIterator {
	OpContourIterator(OpContours* c) 
		: contours(c) {
	}

	OpContour* back() {
		OpContourIter iter;
		iter.back();
		return *iter;
	}

	OpContour* front() {
		OpContourIter iter(contours);
		return *iter;
	}

	OpContourIter begin() {
		return OpContourIter(contours);
	}

	OpContourIter end() {
		return OpContourIter(); 
	}

	bool empty() { return !(begin() != end()); }

	OpContours* contours;
};

struct SegmentIterator {
	SegmentIterator(OpContours* );
	OpSegment* next();

	OpContours* contours;
	OpContourIterator contourIterator;
	OpContourIter contourIter;
	size_t segIndex;
	OP_DEBUG_CODE(bool debugEnded);
};

struct OpPtAlias {
	OpPoint original;
	OpPoint alias;
};

struct OpPtAliases {
	bool add(OpPoint pt, OpPoint alias);
	SegPt addIfClose(OpPoint );
	bool contains(OpPoint ) const;
	OpPoint existing(OpPoint ) const;
	OpPoint find(OpPoint ) const;
	bool isSmall(OpPoint pt1, OpPoint pt2);
	void remap(OpPoint oldAlias, OpPoint newAlias);

	DUMP_DECLARATIONS

	std::vector<OpPoint> aliases;
	std::vector<OpPtAlias> maps;
	OpVector threshold;
};

struct OpContours {
	OpContours();
	~OpContours();

	bool addAlias(OpPoint pt, OpPoint alias);
	void addCallerData(PathOpsV0Lib::AddContext callerData);
//    OpEdge* addFiller(OpEdge* edge, OpEdge* lastEdge);
	OpEdge* addFiller(const OpPtT& start, const OpPtT& end);
	void addToBounds(const OpCurve& );
	OpContour* allocateContour();
	PathOpsV0Lib::CurveData* allocateCurveData(size_t );
	OpEdge* allocateEdge(OpEdgeStorage*& );
	OpIntersection* allocateIntersection();
	OpLimb* allocateLimb();
	PathOpsV0Lib::WindingData* allocateWinding(size_t );

	void apply() {
		for (auto contour : contours) {
			contour->apply();
		}
	}

	bool assemble();

	void betweenIntersections() {
	   for (auto contour : contours) {
			contour->betweenIntersections();
		}
	}

	PathOpsV0Lib::CurveCallBacks& callBack(PathOpsV0Lib::CurveType type) {
		return callBacks[(int) type - 1];
	}

	bool containsFiller(OpPoint start, OpPoint end) const;
//    WindingData* copySect(const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?
	void disableSmallSegments();

	bool empty() {
		for (auto contour : contours) {
			if (contour->segments.size())
				return false;
		}
		return true;
	}

	OpPoint existingAlias(OpPoint pt) const {
		return aliases.existing(pt);
	}

	OpPoint findAlias(OpPoint pt) const {
		return aliases.find(pt);
	}

	void findMissingEnds() {
	   for (auto contour : contours) {
			contour->findMissingEnds();
		}
	}

	void makeCoins() {
	   for (auto contour : contours) {
			contour->makeCoins();
		}
	}

	OpContour* makeContour() {
		OpContour* contour = allocateContour();
		contour->contours = this;
		OP_DEBUG_CODE(contour->debugComplete());
		return contour;
	}

	void makeEdges() {
	   OP_DEBUG_CODE(debugInClearEdges = true);
	   for (auto contour : contours) {
			contour->makeEdges();
		}
	   OP_DEBUG_CODE(debugInClearEdges = false);
	}


	void makePals() {
	   for (auto contour : contours) {
			contour->makePals();
		}
	}

	int nextID() { 
		return ++uniqueID; 
	}

	void normalize() {
		for (auto contour : contours) {
			contour->normalize();
		}
	}

	OpLimb& nthLimb(int index);

	bool pathOps();
	void release(OpEdgeStorage*& );
	OpPoint remapPts(OpPoint oldAlias, OpPoint newAlias);
	void resetLimbs();
	void reuse(OpEdgeStorage* );

	bool setError(PathOpsV0Lib::ContextError  OP_DEBUG_PARAMS(int id, int id2 = 0));
	void setThreshold();
	void sortIntersections();

	OpVector threshold() const {
		return aliases.threshold;
	}

	void transferCoins() {
	   for (auto contour : contours) {
			contour->transferCoins();
		}
	}

	bool debugFail() const;
#if OP_DEBUG
	void addDebugWarning(OpDebugWarning );
	void debugRemap(int oldRayMatch, int newRayMatch);
	bool debugSuccess() const;
#endif
#if OP_DEBUG_VALIDATE
	void debugValidateIntersections();
#endif
#if OP_DEBUG_DUMP
	void debugCompare(std::string s);
	OpLimb& debugNthLimb(int) const;
	void dumpResolve(OpContour*& contourRef);
	void dumpResolve(const OpEdge*& );
	void dumpResolve(OpEdge*& );
	void dumpResolve(OpIntersection*& );
	void dumpResolve(const OpLimb*& limbRef);
	void dumpResolve(OpSegment*& );
	#include "OpDebugDeclarations.h"
#endif

	OpPtAliases aliases;
	std::vector<PathOpsV0Lib::CurveCallBacks> callBacks;
	PathOpsV0Lib::ContextCallBacks contextCallBacks;
	PathOpsV0Lib::PathOutput callerOutput;
	PathOpsV0Lib::AddContext caller;   // note: must use std::memcpy before reading
	// these are pointers instead of inline values because the storage with empty slots is first
	OpEdgeStorage* ccStorage;
	CurveDataStorage* curveDataStorage;
	OpContourStorage* contourStorage;
	OpContourIterator contours;
	OpEdgeStorage* fillerStorage;
	OpSectStorage* sectStorage;
	OpLimbStorage* limbStorage;
	OpLimbStorage* limbCurrent;
	CallerDataStorage* callerStorage;
	OpPointBounds maxBounds;
	PathOpsV0Lib::ContextError error;
	int uniqueID;  // used for object id, unsectable id, coincidence id
#if OP_DEBUG_VALIDATE
	int debugValidateEdgeIndex;
	int debugValidateJoinerIndex;
#endif
#if OP_DEBUG
	OpDebugData debugData;
	OpCurveCurve* debugCurveCurve;
	OpJoiner* debugJoiner;
	int debugOutputID;
	int debugErrorID;
	int debugOppErrorID;
	std::vector<OpDebugWarning> debugWarnings;
	OpDebugExpect debugExpect;
	bool debugInPathOps;
	bool debugInClearEdges;
	bool debugCheckLastEdge;
	bool debugFailOnEqualCepts;
#endif
#if OP_DEBUG_DUMP
	OpTree* dumpTree;
	bool debugDumpInit;   // if true, created by dump init
#endif
};

#endif