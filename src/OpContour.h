// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpJoiner.h"
#include "OpTightBounds.h"
#if TEST_RASTER
#include "OpDebugRaster.h"
#endif

enum class EdgeMatch : int8_t;
struct FoundEdge;
struct OpContourIterator;
struct OpContourStorage;
struct OpCurveCurve;
struct OpJoiner;

struct OpContext;
struct OpInPath;

struct CallerDataStorage {
	CallerDataStorage()
		: next(nullptr)
		, used(0) {
		OP_DEBUG_CODE(memset(storage, 0, sizeof(storage)));
	}

//	static char* Allocate(size_t size, CallerDataStorage** );
#if OP_DEBUG_DUMP
	static void DumpSet(const char*& str, CallerDataStorage** previousPtr);
	DUMP_DECLARATIONS
#endif

	CallerDataStorage* next;
	size_t used;
	char storage[2048];	// !!! size is arbitrary guess -- should measure and do better
};

#if WINDER_CONTOUR_EXPERIMENT
enum class RelinkJoins {
	uninitialized,
	unchanged,
	done,
	unmatched,
	again
};
#endif

struct OpContour {
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

	void addDisjointIntersections() {
		for (auto& segment : segments) {
			segment.addDisjointIntersections();
		}
	}

#if WINDER_CONTOUR_EXPERIMENT
	void addCoinEdges(OpContour* );
	bool addEdges(OpContour* );
	void addJoinEdge(OpJoiner* , OpEdge* );
	void addLast(OpEdge* );
	void addToLinkups(OpJoiner* , OpEdge* );
	void buildDisabled();
	void buildDisabledPals();
	bool detachIfLoop(OpJoiner* , OpEdge* , EdgeMatch loopEnd);
	bool isSorted(Axis axis) const { return Axis::horizontal == axis ? isXSorted : isYSorted; }
	bool joinSetup();
	void joinSort();
	bool linkUp(OpJoiner* , OpEdge* );
	void pushLinkup(OpEdge* );
	RelinkJoins relinkUnambiguous(OpJoiner* , size_t checked);
	void removeLast(OpEdge* , InOutput );
	void removeLink(OpEdge* );
	void setLinkEdge(OpEdge* link, size_t index);
	void setSorted(Axis axis) { (Axis::horizontal == axis ? isXSorted : isYSorted) = true; }
	void unlink(OpEdge* );
	std::vector<OpEdge*>& windingEdges(Axis );

	OP_DEBUG_CODE(void debugMatchRay());
#if OP_DEBUG_VALIDATE
	void debugValidate(const OpJoiner* ) const;
#endif
#endif

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

	bool isEmpty() {
		return !segments.size();
	}

	void findMissingEnds() {
		for (auto& segment : segments) {
			segment.findMissingEnds();
		}
	}

	void fixCCSects() {
		for (auto& segment : segments) {
			segment.fixCCSects();
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

	void setSeen(int tree_id);

	void transferCoins() {
		for (auto& segment : segments) {
			segment.transferCoins();
		}
	}

	OP_DEBUG_CODE(void addDebugContourData(PathOpsV0Lib::DebugContourData );)

#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
	#define OP_X(Thing) \
	void dump##Thing() const;
	SEGMENT_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
#if WINDER_CONTOUR_EXPERIMENT
	std::string debugDumpJoin(DebugLevel l, DebugBase b) const;
#endif
#endif

	OpContext* context;
	std::vector<OpSegment> segments;
	std::vector<OpSegment*> sorted;
	std::vector<OpContour*> sects;
#if WINDER_CONTOUR_EXPERIMENT
	OpContour* winderOwner;  // the master that has intersects the same set of contours as this
	// !!! experiment; move winder data to contour for many-contours optimization
	//  populate only with edges in contour, and edges in overlapping contours
	std::vector<OpEdge*> inX;  // only good if winderOwner points to self
	std::vector<OpEdge*> inY;
	// for joiner:
	std::vector<OpEdge*> byArea;
	std::vector<OpEdge*> unsectByArea;
	std::vector<OpEdge*> disabled;
	std::vector<OpEdge*> disabledPals;
	std::vector<OpEdge*> unsortables;
	LinkUps linkups;
	LinkUps endLinks;
	int treeID = 0;  // tracks if contour has been initialized in this tree's context (for edge 'seen')
	bool disabledBuilt = false;
	bool disabledPalsBuilt = false;
	bool isXSorted = false;
	bool isYSorted = false;
#endif
	PathOpsV0Lib::Winding winding;
	OpPointBounds bounds;
	int id;

	OP_DEBUG_CODE(PathOpsV0Lib::DebugContourCallbacks debugCallbacks);
	OP_DEBUG_CODE(PathOpsV0Lib::DebugContourData debugCaller);  // note: must use std::memcpy before reading
#if TEST_RASTER
	OpDebugRaster rasterOperand;
#endif
};

struct OpContourStorage {
	OpContourStorage()
		: next(nullptr)
		, used(0) {
	}

#if OP_DEBUG_DUMP
	int debugCount() const;
	OpContour* debugFind(int id) const;
	OpContour* debugIndex(int index) const;
	static void DumpSet(const char*& , OpContext* );
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

	OpContourIter(OpContext* contours);
	
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
	OpContourIterator(OpContext* c) 
		: contours(c) {
	}

	OpContour* back() {
		OpContourIter iter(contours);
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

	OpContext* contours;
};

struct SegmentIterator {
	SegmentIterator(OpContext* );
	OpSegment* next();

	OpContext* contours;
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

struct OpContext {
	OpContext();
	~OpContext();

	bool addAlias(OpPoint pt, OpPoint alias);
//    OpEdge* addFiller(OpEdge* edge, OpEdge* lastEdge);
	OpEdge* addFiller(const OpPtT& start, const OpPtT& end);
	void addToBounds(const OpCurve& );
	char* allocateCallerData(size_t );
	OpContour* allocateContour();
	PathOpsV0Lib::CurveData* allocateCurveData(size_t );
	OpEdge* allocateEdge(OpEdgeStorage*& );
	OpIntersection* allocateIntersection();
	OpLimb* allocateLimb();
	PathOpsV0Lib::WindingData* allocateWinding(size_t );

	void addDisjointIntersections() {
		for (auto contour : contours) {
			contour->addDisjointIntersections();
		}
	}

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

	PathOpsV0Lib::CurveCallbacks& callback(PathOpsV0Lib::CurveType type) {
		return callbacks[(int) type - 1];
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

	void fixCCSects() {
	   for (auto contour : contours) {
			contour->fixCCSects();
		}
	}
	void findMissingEnds() {
	   for (auto contour : contours) {
			contour->findMissingEnds();
		}
	}

	void initOutOnce();

	void makeCoins() {
		OP_DEBUG_CONTEXT();
	    for (auto contour : contours) {
			contour->makeCoins();
		}
	}

	OpContour* makeContour() {
		OpContour* contour = allocateContour();
		contour->context = this;
		contour->id = nextID();
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
	void opsInit();
	bool pathOps();
	void release(OpEdgeStorage*& );
	OpPoint remapPts(OpPoint oldAlias, OpPoint newAlias);
	void resetLimbs();
	bool setError(PathOpsV0Lib::ContextError  OP_DEBUG_PARAMS(int id, int id2 = 0));
	void setSortedBounds();
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
	void addDebugContextData(PathOpsV0Lib::DebugContextData );

	PathOpsV0Lib::DebugCurveCallbacks& debugCallback(PathOpsV0Lib::CurveType type) {
		return debugCallbacks[(int) type - 1];
	}

	void debugRemap(int oldRayMatch, int newRayMatch);
	bool debugSuccess() const;
#endif
#if OP_DEBUG_VALIDATE
	void debugValidateIntersections();
#else
	void debugValidateIntersections() {}
#endif
#if OP_DEBUG_DUMP
	void debugCompare(std::string s);
	const OpLimb& debugNthLimb(int) const;
	void dumpResolve(OpContour*& contourRef);
	void dumpResolve(const OpEdge*& );
	void dumpResolve(OpEdge*& );
	void dumpResolve(OpIntersection*& );
	void dumpResolve(const OpLimb*& limbRef);
	void dumpResolve(OpSegment*& );
	#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugLimbClear();
	int debugLimbIndex(const OpEdge* ) const;
#endif

	OpPtAliases aliases;  // !!! consider moving to context for non-overlapping context case
	std::vector<PathOpsV0Lib::CurveCallbacks> callbacks;
	PathOpsV0Lib::ContextCallbacks contextCallbacks;
	PathOpsV0Lib::WindingCallbacks windingCallbacks;
	PathOpsV0Lib::PathOutput callerOutput;
	PathOpsV0Lib::ErrorHandler errorHandler;

#if WINDER_CONTOUR_EXPERIMENT
	std::vector<OpContour*> sorted; 
#endif
	// these are pointers instead of inline values because the storage with empty slots is first
	OpEdgeStorage* ccStorage;
	CurveDataStorage* curveDataStorage;
	OpContourStorage* contourStorage;
	std::vector<OpContour*> contours;
	OpEdgeStorage* fillerStorage;
	OpSectStorage* sectStorage;
	OpLimbStorage* limbStorage;
	OpLimbStorage* limbCurrent;
	CallerDataStorage* callerStorage;
	OpPointBounds maxBounds;
	PathOpsV0Lib::ContextError error;
	bool fatalError;
	int uniqueID;  // used for object id, unsectable id, coincidence id
	bool outputOne;
	bool linkErased;  // used to tell relinkUnambiguous to continue or not

#if OP_DEBUG_VALIDATE
	int debugValidateEdgeIndex;
	int debugValidateJoinerIndex;
#endif
#if TEST_RASTER
	OpDebugSamples sampleOperands;
	OpDebugSamples sampleOutputs;  // curve output + combined operands
	OpDebugRaster rasterOutput;
	OpDebugRaster rasterCombined;
	bool rasterEnabled;
#endif
#if OP_DEBUG
	std::vector<PathOpsV0Lib::DebugCurveCallbacks> debugCallbacks;
	PathOpsV0Lib::DebugContextCallbacks debugContextCallbacks;
	PathOpsV0Lib::DebugContextData debugContextData;
	OpDebugData debugData;
	OpCurveCurve* debugCurveCurve;
	OpJoiner* debugJoiner;
	OpTree* debugTree;
	int debugOutputID;
	int debugErrorID;
	int debugOppErrorID;
	OpDebugExpect debugExpect;
	bool debugInPathOps;
	bool debugInClearEdges;
	bool debugCheckLastEdge;
	bool debugFailOnEqualCepts;
#endif
#if OP_DEBUG_DUMP
	bool debugDumpInit;   // if true, created by dump init
#endif
};

#endif