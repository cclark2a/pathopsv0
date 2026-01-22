// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpJoiner.h"
#include "OpTightBounds.h"

enum class EdgeMatch : int8_t;
struct OpContext;

enum class RelinkJoins {
	uninitialized,
	unchanged,
	done,
	unmatched,
	again
};

typedef PathOpsV0Lib::Contour* ContourPtr;

struct OpContour {
	operator ContourPtr() const {
		return (ContourPtr)(this);
	}
	void addCoinEdges();

	void addDisjointIntersections() {
		for (auto& segment : segments) {
			segment.addDisjointIntersections();
		}
	}

	void addEdges();
	OpIntersection* addEdgeSect(const OpPtT& , OpSegment* seg
		   OP_LINE_FILE_DEF(const OpEdge* edge, const OpEdge* oEdge));
	void addJoinEdge(OpJoiner* , OpEdge* );
	void addLast(OpEdge* );
//	void addLine(OpPoint pts[2]);
	void addMerge(OpContour* );  // track coincidence that effectively extend this contour
	OpIntersection* addSegSect(const OpPtT& , OpSegment* seg
			OP_LINE_FILE_DEF(const OpSegment* oSeg));
	void addToLinkups(OpJoiner* , OpEdge* );
	OpIntersection* addUnsect(const OpPtT& , OpSegment* seg, int uID, MatchEnds 
			OP_LINE_FILE_DEF(const OpSegment* oSeg));

	WindingCondition apply() {
		for (auto& segment : segments) {
			if (WindingCondition windingCondition = segment.apply())
            return windingCondition;
		}
	    return 0;
}

	void manyCoincidences() {
		for (auto& segment : segments) {
            segment.manyCoincidences();
        }
    }

    void betweenCoincidence() {
        for (auto& segment : segments) {
			segment.betweenCoincidence();
		}
	}

	void buildBackwards();
	void buildCenterless();
	void buildPals();
    void clear();
    void clearEdges();
    void clearSegments();
	bool detachIfLoop(OpJoiner* , OpEdge* , EdgeMatch loopEnd);
	bool disabledPal(OpPoint, OpPoint) const;  // !!! bare minimum to fix cubic129075 (experiment)

	void findMissingEnds() {
		for (auto& segment : segments) {
			segment.findMissingEnds();
		}
	}

	bool fixCCSects();
    void init(OpContext* , PathOpsV0Lib::WindingData winding, size_t size);
    void init();
	bool isEmpty();
	bool isOpen() { return !merges.empty(); }
//	bool isSorted(Axis axis) const { return Axis::horizontal == axis ? isXSorted : isYSorted; }
	bool joinSetup();
	void joinSort();
	bool linkUp(OpJoiner* , OpEdge* );

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

	const std::vector<OpContour*>& members() const { return overlapOwner->overlaps; }
	int nextID() const;

	void normalize() {
		for (auto& segment : segments) {
			segment.normalize();
		}
	}

	void pushLinkup(OpEdge* );
	RelinkJoins relinkUnambiguous(OpJoiner* , size_t checked);
	void removeLast(OpEdge* /*, InOutput */);
	void removeLink(OpEdge* );
	void setLinkEdge(OpEdge* link, size_t index);
	void setSeen(int tree_id);
//	void setSorted(Axis axis) { (Axis::horizontal == axis ? isXSorted : isYSorted) = true; }

	void transferCoins() {
		for (auto& segment : segments) {
			segment.transferCoins();
		}
	}

	void tripleSect() {
		for (auto& segment : segments) {
			segment.tripleSect();
		}
	}

	void unlink(OpEdge* );
    PathOpsV0Lib::Winding winding() const { 
        return { (ContourPtr) this, (void*) &windingStorage.front(), windingStorage.size() }; }
	std::vector<OpEdge*>& windingEdges(Axis );

	// OP_DEBUG_CODE(void addDebugContourData(PathOpsV0Lib::DebugContourData , 
    //        PathOpsV0Lib::DebugContourType );)
	OP_DEBUG_CODE(void debugMatchRay());
#if OP_DEBUG_VALIDATE
	void debugValidate(const OpJoiner* ) const;
#endif
	DUMP_DECLARATIONS
#if OP_DEBUG_DUMP
	#define OP_X(Thing) \
	void dump##Thing() const;
	SEGMENT_DETAIL
	EDGE_OR_SEGMENT_DETAIL
	#undef OP_X
	std::string debugDumpJoin(DebugLevel l, DebugBase b) const;
#endif
#if OP_DEBUG_DUMP || OP_DEBUGGER || OP_TEST
	PathOpsV0Lib::Curve debugCurve(int index, std::vector<float>* extrema) const;
#endif

	std::vector<OpSegment> segments;
	std::vector<OpSegment*> sorted;
	std::vector<OpContour*> overlaps;  // intersecting contours (valid if this equals overlapOwner)
	std::vector<OpContour*> merges;	 // coincident contours added to this set
	//  populated with edges in this contour, merges and overlaps
	std::vector<OpEdge*> inX;  // edges intersecting horz rays (valid if this equals overlapOwner)
	std::vector<OpEdge*> inY;  // edges intersecting vert rays (valid if this equals overlapOwner)
	// for joiner:
	std::vector<OpEdge*> byArea;
	std::vector<OpEdge*> unsectByArea;
	std::vector<OpEdge*> disabledBackwards;
	std::vector<OpEdge*> disabledCenterless;
	std::vector<OpEdge*> disabledPals;
	std::vector<OpEdge*> unsortables;
    std::vector<uint8_t> windingStorage;
	LinkUps linkups;
	LinkUps endLinks;
	OpPointBounds overlapBounds;  // bounds of intersecting contours (overlapOwner only)
	OpPointBounds bounds;	// bounds of segments in this contour
	OpContext* context;
	OpContour* overlapOwner;  // the master that intersects the same set of contours as this
	int id;
	int treeID;  // tracks if contour has been initialized in this tree's context (for edge 'seen')
	bool backwardsBuilt;
	bool centerlessBuilt;
	bool hasPals;
	bool palsBuilt;
	bool disabled;
	bool overlapsMerged;

//	OP_DEBUG_CODE(PathOpsV0Lib::DebugContourCallbacks debugCallbacks);
#if OP_DEBUG || OP_DEBUGGER || OP_TEST
	std::vector<PathOpsV0Lib::DebugCurveData> debugCurveData;
	OpWinding debugWinding = OpWinding(WindingUninitialized::dummy);
#endif
#if OP_DEBUG_IMAGE
    // AddQuads/AddCubics/AddLine/AddConics save original curve for graphics debugger
	uint32_t debugColor = blue;
#endif
	OP_DEBUG_CODE(bool debugEmpty = false);  // if empty, when cloned, this is reused in release
};

struct OpContourStorage {
	OpContourStorage()
		: next(nullptr)
		, used(0) {
        for (OpContour& contour : storage) {
            contour.init();
        }
	}

#if OP_DEBUG_SERIALIZE
	int debugCount() const;
	OpContour* debugIndex(int index) const;
#endif
#if OP_DEBUG_DUMP
    void debugCheck(const OpContour* ); // error and exit if contour is not in storage
	OpContour* debugFind(int id) const;
	static void DumpSet(const char*& , OpContext* );
#endif
	DUMP_DECLARATIONS

	OpContourStorage* next;
	OpContour storage[2];
	int used;
};

struct OpContourIter {
	OpContourIter()
		: storage(nullptr)
		, contourIndex(0) {
	}

	OpContourIter(OpContext* );
	
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
		: context(c) {
	}

	OpContour* back() {
		OpContourIter iter(context);
		iter.back();
		return *iter;
	}

	OpContour* front() {
		OpContourIter iter(context);
		return *iter;
	}

	OpContourIter begin() {
		return OpContourIter(context);
	}

	OpContourIter end() {
		return OpContourIter(); 
	}

	bool empty() { return !(begin() != end()); }

	OpContext* context;
};

#if 0
struct SegmentIterator {
	SegmentIterator(OpContext* );
	OpSegment* next();

	OpContourIterator contourIterator;
	OpContourIter contourIter;
	size_t segIndex;
	OP_DEBUG_CODE(bool debugEnded);
};
#endif

#endif
