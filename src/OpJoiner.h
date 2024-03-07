// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpSegment.h"

struct OpContours;
struct OpOutPath;

enum class LinkPass {
	none,
	unambiguous,
	unsectInX
};

struct LinkUps {
	void sort();
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<OpEdge*> l;
};

struct OpJoiner {
	OpJoiner(OpContours& contours, OpOutPath& );
//	bool activeUnsectable(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& oppEdges);
	void addEdge(OpEdge* );
	void addToLinkups(OpEdge* );
	void buildDisabled(OpContours& );
	void buildDisabledPals(OpContours& );
    void checkDisabled();
    void checkGap();
    void checkLinkups();
    void checkNothingLeft();
    bool checkSectGap();
    void checkUnsectableGap();
    void checkUnsortableAndDisabled();
    void checkLinkupsUnsortables();
    FoundEdge* chooseSmallest();
    void detachChoppedEtc();
	bool detachIfLoop(OpEdge* );
//    bool forceSmallEdge();
	bool hookup(FoundEdge* smallest);
    bool lastLastResort();
	bool linkRemaining();
	void linkUnambiguous();
	bool linkUp(OpEdge* );
	void matchLeftover(const std::vector<OpEdge*>& leftovers);
	bool matchLinks(bool popLast);
    void matchPals();
	void relinkUnambiguous(size_t checked);
	void sort();
#if OP_DEBUG
	void debugMatchRay(OP_DEBUG_CODE(OpContours* contours));
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugDraw();
#endif

	OpOutPath& path;	// !!! move op joiner into op contours to eliminate reference?
	std::vector<OpEdge*> byArea;
	std::vector<OpEdge*> unsectByArea;
	std::vector<OpEdge*> disabled;
	std::vector<OpEdge*> disabledPals;
    std::vector<OpEdge*> unsortables;
	std::vector<FoundEdge> found;  //edges, real or constructed, with an end equal to matchPt 
    FoundEdge bestGap;
	LinkUps linkups;  // vector wrapper (allows data specific debugging / dumping)
	EdgeMatch linkMatch;
	LinkPass linkPass;
	OpEdge* edge;  // start of current link list
	OpEdge* lastEdge;  // end of current link list
	OpPoint matchPt;
	bool disabledBuilt;
	bool disabledPalsBuilt;
};

// !!! experiment: keep track of all edge possibilities to find the best closing path
enum class LimbType {
	unlinked,
	linked
};

struct OpTree;
struct OpLimbStorage;

struct OpLimb {
	OpLimb() {
#if OP_DEBUG
		edge = nullptr;
		lastEdge = nullptr;
		parent = nullptr;
		match = EdgeMatch::none;
		looped = false;
#endif
	}
	void add(OpTree& , OpEdge* , EdgeMatch , LimbType , OpEdge* first = nullptr);
	void foreach(const OpJoiner& , OpTree& );
	void set(OpTree& , OpEdge* , const OpLimb* parent, EdgeMatch , LimbType , 
			OpEdge* otherEnd, const OpPointBounds* bounds = nullptr);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpPointBounds bounds;
	OpEdge* edge;
	OpEdge* lastEdge;
	const OpLimb* parent;
	OpPoint lastPt;
	EdgeMatch match;
	LimbType type;
	bool looped;

	// !!! add debug id?
};


// !!! eventually (if this works) add tree (or limb storage) to joiner
// prefer the looped limb with the smallest perimeter 
struct OpTree {
	OpTree(const OpJoiner& );
	bool join(OpJoiner& );
	OpLimb& limb(int index);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif
	OpLimbStorage* current;
	const OpContour& contour;
	const OpEdge* edge;
	const OpLimb* bestLimb;   // index into limbStorage
	OpPoint firstPt;
	float bestPerimeter;
	int baseIndex;
	int totalUsed;
	int walker;
};

struct OpLimbStorage {
	OpLimbStorage()
		: nextBlock(nullptr)
		, prevBlock(nullptr) {
		static_assert((ARRAY_COUNT(storage) - 1 & ARRAY_COUNT(storage)) == 0);
	}
	OpLimb* allocate(OpTree& );
	OpLimb& limb(OpTree& , int index);
	void reset();

	OpLimbStorage* nextBlock;
	OpLimbStorage* prevBlock;
	OpLimb storage[256];
	int used;
};


#endif
