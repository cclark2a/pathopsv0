// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef OpContext_DEFINED
#define OpContext_DEFINED

#include "OpContour.h"

#if OP_DEBUG
struct OpCurveCurve;
struct OpJoiner;
#endif

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
	bool original(OpPoint ) const;
	void remap(OpPoint oldAlias, OpPoint newAlias);

	DUMP_DECLARATIONS

	std::vector<OpPoint> aliases;
	std::vector<OpPtAlias> maps;
	OpVector threshold;
	float thresholdLength;
};

typedef PathOpsV0Lib::Context* ContextPtr;

struct OpContext {
	OpContext();
	~OpContext();

	operator ContextPtr() const {
		return (PathOpsV0Lib::Context*)(this);
	}

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
		return callbacks[type];
	}

	bool containsFiller(OpPoint start, OpPoint end) const;
	bool containsFiller(int ccUnsectableID) const;
	bool containsPals(OpEdge* , int totalLimbs);
//    WindingData* copySect(const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?
    int curveIndex(int nativeType);
    void curveIndex(PathOpsV0Lib::AddCurve& curvePtr);
//    void demotePalLinks();
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

	bool fixCCSects() {
		for (auto contour : contours) {
			if (!contour->fixCCSects())
				return false;
		}
		return true;
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

	void markInCoincidence();

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
	void rebuildOverlaps();
	void release(OpEdgeStorage*& );
	OpPoint remapPts(OpPoint oldAlias, OpPoint newAlias);
	void resetFiller();
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
	void addDebugContextData(PathOpsV0Lib::DebugContextData , PathOpsV0Lib::DebugContextType );
    PathOpsV0Lib::DebugContextData& debugGetContextData(PathOpsV0Lib::DebugContextType );
	PathOpsV0Lib::DebugCurveCallbacks& debugCallback(PathOpsV0Lib::Curve );
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
	void debugLimbColor(int lastLimbID, uint32_t color);
	int debugLimbIndex(const OpEdge* ) const;
#endif

	OpPtAliases aliases;  // !!! consider moving to contour for non-overlapping contour case
	std::vector<PathOpsV0Lib::CurveCallbacks> callbacks;
    std::vector<int> nativeCurveTypes;
	PathOpsV0Lib::ContextCallbacks contextCallbacks;
	PathOpsV0Lib::WindingCallbacks windingCallbacks;
	PathOpsV0Lib::PathOutput callerOutput;
	PathOpsV0Lib::ErrorHandler errorHandler;
	std::vector<OpContour*> sortedContours; 
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
#if OP_DEBUG
	std::vector<PathOpsV0Lib::DebugCurveCallbacks> debugCallbacks;
	PathOpsV0Lib::DebugContextCallbacks debugContextCallbacks;
	std::array<PathOpsV0Lib::DebugContextData, static_cast<std::size_t>(
            PathOpsV0Lib::DebugContextType::Count)> debugContextData;
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
	std::vector<std::string> debugDumpNotes;
	std::vector<std::string> debugDumpSkips;
	bool debugDumpInit;   // if true, created by dump init
#endif
};

#endif
