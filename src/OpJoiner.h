// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpSegment.h"

struct OpContours;
struct OpOutPath;

enum class LinkPass {
	none,
	normal,
	unsectable,
	remaining,
};

/* !!! consider a rewrite where a single link up is
    struct LinkUp {
		OpPointBounds bounds;
		OpEdge* last;
		OpEdge* edge;
	};
	and add any other data in OpEdge which is only relevant to a linked up list of edges (if any)
*/
struct LinkUps {
	void sort();
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<OpEdge*> l;
};

struct OpJoiner {
#if OP_TEST_NEW_INTERFACE
	OpJoiner(OpContours& contours);
#else
	OpJoiner(OpContours& contours, OpOutPath& );
#endif
//	bool activeUnsectable(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& oppEdges);
	void addEdge(OpEdge* );
	void addToLinkups(OpEdge* );
	void buildDisabled(OpContours& );
	void buildDisabledPals(OpContours& );
	bool detachIfLoop(OpEdge* , EdgeMatch loopEnd);
	bool linkRemaining(OP_DEBUG_CODE(OpContours*));
	void linkUnambiguous(LinkPass );
	bool linkUp(OpEdge* );
	bool matchLinks(bool popLast);
	bool relinkUnambiguous(size_t checked);
	bool setup();
	void sort();
	void unlink(OpEdge* ); // don't unlink edges that are in linkups
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

#if !OP_TEST_NEW_INTERFACE
	OpOutPath& path;	// !!! move op joiner into op contours to eliminate reference?
#endif
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
	OpEdge* lastLink;  // end of current link list
	OpPoint matchPt;
	bool disabledBuilt;
	bool disabledPalsBuilt;
};

// !!! experiment: keep track of all edge possibilities to find the best closing path
enum class LimbType : uint8_t {
	linked,    // in linkups list with correct winding
	unlinked,  // in unsectByArea and in unsortables
	disabled,  // in disabled
	disabledPals,  // in disabled pals
	miswound,  // in linkups list, including entries with the wrong winding
	disjoint,  // gap to closest in linkups list, or gap to edge start (loop)
};

struct OpTree;
struct OpLimbStorage;

struct OpLimb {
	OpLimb() {
#if OP_DEBUG
		edge = nullptr;
		lastLimb = nullptr;
		parent = nullptr;
		linkedIndex = 0;
		match = EdgeMatch::none;
		type = LimbType::unlinked;
		looped = false;
		debugID = 0;
#endif
	}
	void add(OpTree& , OpEdge* , EdgeMatch , LimbType , size_t index = 0, OpEdge* first = nullptr);
	void foreach(OpJoiner& , OpTree& , LimbType );
	void set(OpTree& , OpEdge* , const OpLimb* parent, EdgeMatch , LimbType , 
			size_t index, OpEdge* otherEnd, const OpPointBounds* bounds = nullptr);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
	std::string debugDumpIDs(DebugLevel , bool bracket) const;
#endif

	OpPointBounds bounds;
	OpEdge* edge;
	OpEdge* lastLimb;
	const OpLimb* parent;
	OpPoint lastPt;
	uint32_t linkedIndex;
	EdgeMatch match; // end of edge that matches last point in parent limb
	LimbType type;  // if linked or miswound: if match is end, edge is last in linked list
	bool looped;

#if OP_DEBUG
	std::vector<OpLimb*> debugBranches;
	int debugID;
#endif
};


// !!! eventually (if this works) add tree (or limb storage) to joiner
// prefer the looped limb with the smallest perimeter 
struct OpTree {
	OpTree(OpJoiner& );
	void addDisabled(OpJoiner& );
	void initialize(OpJoiner& join, LimbType limbType);
	bool join(OpJoiner& );
	OpLimb& limb(int index);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif
	OpLimbStorage* limbStorage;
	OpLimbStorage* current;
	const OpContour& contour;
	const OpEdge* edge;
	const OpLimb* bestGapLimb;  // used only by detached pass
	const OpLimb* bestLimb;   // index into limbStorage
	OpPoint firstPt;
	float bestDistance;  // used only by detached pass
	float bestPerimeter;
	int baseIndex;
	int totalUsed;
	int walker;

#if OP_DEBUG
	std::vector<OpLimb*> debugLimbs;
#endif
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
#if OP_DEBUG_DUMP
	size_t debugCount() const;
	const OpLimb* debugFind(int ID) const;
	OpLimb* debugIndex(int index);
	static void DumpSet(const char*& , OpContours* );
	DUMP_DECLARATIONS
#endif

	OpLimbStorage* nextBlock;
	OpLimbStorage* prevBlock;
	OpLimb storage[256];
	int used;
};


#endif
