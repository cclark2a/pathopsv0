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
#if OP_DEBUG_DUMP || OP_DEBUGGER
    size_t dumpOffset(PathOpsV0Lib::ContextUserData data) const;
    void dumpResolve(PathOpsV0Lib::ContextUserData& );
	static void DumpSet(const char*& str, CallerDataStorage** previousPtr);
#endif
	DUMP_DECLARATIONS

	CallerDataStorage* next;
	size_t used;
	uint8_t storage[2048];	// !!! size is arbitrary guess -- should measure and do better
};

struct OpUserData {
    PathOpsV0Lib::ContextUserData data;
    PathOpsV0Lib::UserDataType type;
};

enum class SortSmall {
	no,
	yes
};

#if OP_DEBUG_SERIALIZE
enum class DumpRaster {
	no,
	yes
};
#endif

typedef PathOpsV0Lib::Context* ContextPtr;

struct OpContext {
	OpContext();
	~OpContext();

	operator ContextPtr() const {
		return (ContextPtr)(this);
	}

//    OpEdge* addFiller(OpEdge* edge, OpEdge* lastEdge);
	OpEdge* addFiller(OpPoint start, OpPoint end, OpSegment* parent);
	void addToBounds(const OpCurve& );
    void addUserData(PathOpsV0Lib::ContextUserData );

	void aliasIntersections() {
		for (auto contour : contours) {
			contour->aliasIntersections();
		}
	}

	uint8_t* allocateCallerData(size_t  OP_DEBUG_RASTER_PARAMS(bool raster));
	OpContour* allocateContour();
	PathOpsV0Lib::CurveData* allocateCurveData(size_t );
	OpEdge* allocateEdge(OpEdgeStorage*&   OP_DEBUG_PARAMS(std::string debugName));
	OpIntersection* allocateIntersection();
	OpLimb* allocateLimb(OpTree* );
	PathOpsV0Lib::WindingData* allocateWinding(size_t  OP_DEBUG_RASTER_PARAMS(bool usedByRaster));
	bool allowError(PathOpsV0Lib::ContextError , PathOpsV0Lib::Curve* = nullptr);

	void addDisjointIntersections() {
		for (auto contour : contours) {
			contour->addDisjointIntersections();
		}
	}

	WindingCondition apply();
	bool assemble();

	void betweenCoincidence() {
	   for (auto contour : contours) {
			contour->betweenCoincidence();
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}

	PathOpsV0Lib::CurveCallbacks& callback(PathOpsV0Lib::CurveType type) {
		return callbacks[type];
	}

	const PathOpsV0Lib::CurveCallbacks& callback(PathOpsV0Lib::CurveType type) const {
		return callbacks[type];
	}

    void clearContours() {
    	for (auto contour : contours) {
            contour->byArea.clear();
        }
    }

    void clearEdges() {
    	for (auto contour : contours) {
            contour->clearEdges();
        }
    }

    void clearSegments() {
    	for (auto contour : contours) {
            contour->clearSegments();
        }
    }

    void clear();

	bool containsFiller(OpPoint start, OpPoint end) const;
	bool containsFiller(int ccUnsectableID) const;
	bool containsPals(OpEdge* , int totalLimbs);
//    WindingData* copySect(const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?
    int curveIndex(int nativeType) const;
//    void demotePalLinks();
	void disableSmallSegments();

	bool empty() const {
		for (const OpContour* contour : contours) {
			if (contour->segments.size())
				return false;
		}
		return true;
	}

    PathOpsV0Lib::ContextUserData findUserData(PathOpsV0Lib::UserDataType );

	bool fixCCSects() {
		for (auto contour : contours) {
			if (!contour->fixCCSects())
				return false;
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
		return true;
	}

#if 0
	void findMissingEnds() {
	   for (auto contour : contours) {
			contour->findMissingEnds();
		}
	}
#endif

	void initOutOnce();

	void makeCoins() {
	    for (auto contour : contours) {
			contour->makeCoins();
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}

	OpContour* makeContour(PathOpsV0Lib::WindingData winding, size_t size) {
		OpContour* contour = allocateContour();
        contour->init(this, winding, size);
		return contour;
	}

	void makeEdges() {
		OP_DEBUG_CODE(debugInClearEdges = true);
		for (auto contour : contours) {
			contour->makeEdges();
		}
		OP_DEBUG_CODE(debugInClearEdges = false);
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}


	void makePals() {
	   for (auto contour : contours) {
			contour->makePals();
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}

	void manyCoincidences() {
	   for (auto contour : contours) {
			contour->manyCoincidences();
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}

	void markInCoincidence();
	void mergeEndPoints();  // iterate repeatedly through all contours and contour's overlaps
	void mergeIntersections();  // .. for sect points that are nearly equal and merge them
//	void mergeOpposites();  // .. then merge sect and its opposite

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
	WindingCondition pathOps();
	void rebuildOverlaps();
	void release(OpEdgeStorage*& );
	void resetFiller();
	void resetLimbs();
	bool setError(PathOpsV0Lib::ContextError  OP_DEBUG_PARAMS(int id, int id2 = 0));
	void setSortedBounds();
	void setThreshold();
	void sortIntersections(SortSmall );

	void transferCoins() {
	   for (auto contour : contours) {
			contour->transferCoins();
		}
		OP_DEBUG_DUMP_CODE(dumpFile(__func__));
	}

#if 0
	void tripleSect() {
	   for (auto contour : contours) {
			contour->tripleSect();
		}
	}
#endif

	void zeroSmall() {
		for (auto contour : contours) {
			contour->zeroSmall();
		}
	}

	bool debugFail() const;
#if OP_TEST
	PathOpsV0Lib::DebugCurveCallbacks& debugCallback(PathOpsV0Lib::Curve );
	const PathOpsV0Lib::DebugCurveCallbacks& debugCallback(PathOpsV0Lib::Curve ) const;
#endif
#if OP_DEBUG || OP_DEBUGGER
//	void addDebugContextData(PathOpsV0Lib::DebugContextData , PathOpsV0Lib::DebugContextType );
//    PathOpsV0Lib::DebugContextData& debugGetContextData(PathOpsV0Lib::DebugContextType );
	void debugRemap(int oldRayMatch, int newRayMatch);
	bool debugSuccess() const;
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
	void debugValidateContours() const;  // make sure overlaps make sense and context index is correct
	void debugValidateIntersections() const;
#endif
#if OP_DEBUG_SERIALIZE
	void dumpBaseFile(DumpRaster ) const;
	std::string debugDump(DebugLevel , DebugBase , DumpRaster ) const;
	void dumpFile(std::string description, DumpRaster dumpRaster = DumpRaster::no);
    const OpEdge* debugFindEdge(int id) const;
    const OpSegment* debugFindSegment(int id) const;
    bool dumpInitialized() const {
                return initialized || !windingSet;  }
	const OpLimb& debugNthLimb(int) const;
	void dumpString(const std::string& ) const;
#endif
#if OP_DEBUG_DUMP
	void debugCompare(std::string s);
	void dumpResolve(OpContour*& contourRef);
	void dumpResolve(const OpEdge*& );
	void dumpResolve(OpEdge*& );
	void dumpResolve(OpIntersection*& );
	void dumpResolve(const OpLimb*& limbRef);
	void dumpResolve(OpLimb*& limbRef);
	void dumpResolve(OpSegment*& );
#endif
#include "OpDebugDeclarations.h"

	std::vector<PathOpsV0Lib::CurveCallbacks> callbacks;
    std::vector<PathOpsV0Lib::ContextUserData> userData;
    std::vector<int> nativeCurveTypes;
	PathOpsV0Lib::ContextCallbacks contextCallbacks;
	PathOpsV0Lib::WindingCallbacks windingCallbacks;
	PathOpsV0Lib::ErrorHandler errorHandler = { nullptr };
	std::vector<OpContour*> sortedContours; 
	// these are pointers instead of inline values because the storage with empty slots is first
	CurveDataStorage* curveDataStorage = nullptr;
	OpEdgeStorage* ccStorage = nullptr;
	OpContourStorage* contourStorage = nullptr;
	std::vector<OpContour*> contours;
	OpEdgeStorage* fillerStorage = nullptr;
	OpSectStorage* sectStorage = nullptr;
	OpLimbStorage* limbStorage = nullptr;
	OpLimbStorage* limbCurrent = nullptr;
	CallerDataStorage* callerStorage = nullptr;
	OpPointBounds maxBounds;
	OpVector threshold;
	float thresholdLength;
	PathOpsV0Lib::ContextError error = PathOpsV0Lib::ContextError::none;
	int uniqueID = 0;  // used for object id, unsectable id, coincidence id
    bool initialized = false;
    bool allDiscarded = false;
    bool allKept = false;
	bool fatalError = false;
	bool outputOne = false;
	bool linkErased = false;  // used to tell relinkUnambiguous to continue or not
    bool windingSet = false;
#if OP_DEBUG_VALIDATE
	int debugValidateEdgeIndex = 0;
	int debugValidateJoinerIndex = 0;
#endif
#if OP_TEST
	std::vector<PathOpsV0Lib::DebugCurveCallbacks> debugCallbacks;
#endif
#if OP_DEBUG || OP_DEBUGGER
	PathOpsV0Lib::DebugContextCallbacks debugContextCallbacks;
	OpDebugData debugData;
	OpCurveCurve* debugCurveCurve = nullptr;
	OpJoiner* debugJoiner = nullptr;
	OpTree* debugTree = nullptr;
	std::vector<OpEdge*>* debugErasures = nullptr;
	int debugErrorID = 0;
	int debugOppErrorID = 0;
	OpDebugExpect debugExpect = OpDebugExpect::unknown;
	bool debugInPathOps = false;
	bool debugInClearEdges = false;
	bool debugCheckLastEdge = false;
	bool debugFailOnEqualCepts = false;
#endif
#if OP_DEBUG_SERIALIZE
	std::string debugFilename;
	std::string debugDescription;
	std::string debugOutPath;
#endif
#if OP_DEBUG_DUMP
	std::vector<OpEdge*> debugDumpErasures;  // read from flattened data
	bool debugDumpInit = false;   // if true, created by dump init
#endif
#if OP_TEST_RASTER || OP_DEBUGGER
	CallerDataStorage* rasterStorage = nullptr;
	struct DebugRaster* debugRaster = nullptr;
#endif
};

#endif
