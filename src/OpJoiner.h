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
	DUMP_DECLARATIONS

	std::vector<OpEdge*> l;
};

struct OpJoiner {
	OpJoiner(OpContours& contours);
	void addEdge(OpEdge* );
	void addToLinkups(OpEdge* );
	void buildDisabled(OpContours& );
	void buildDisabledPals(OpContours& );
	bool detachIfLoop(OpEdge* , EdgeMatch loopEnd);
	static bool LinkEnd(OpEdge *);
	bool linkRemaining(OP_DEBUG_CODE(OpContours*));
//	bool linkSimple(OpEdge* );
	static OpEdge* LinkStart(OpEdge *);
	void linkUnambiguous(LinkPass );
	bool linkUp(OpEdge* );
	bool matchLinks(bool popLast);
	bool relinkUnambiguous(size_t checked);
	bool setup();
	void sort();
	void unlink(OpEdge* ); // don't unlink edges that are in linkups
#if OP_DEBUG
	void debugMatchRay(OP_DEBUG_CODE(OpContours* contours));
	static bool DebugShowImage();
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

	OP_DEBUG_CODE(int debugRecursiveDepth);
};

// !!! experiment: keep track of all edge possibilities to find the best closing path
// !!! unsectable made coincident missing disabled pals check
enum class LimbPass : uint8_t {
	none,
	linked,    // in linkups list with correct winding
	unlinked,  // in unsectByArea and in unsortables
	unsectPair, // gap to other edge in unsectable pair
	disabled,  // in disabled
	disabledPals,  // in disabled pals
	miswound,  // in linkups list, including entries with the wrong winding
	disjoint,  // gap to closest in linkups list, or gap to edge start (loop)
	unlinkedPal,  // unlinked variant that permits siblings to connect to seen edges' pals
};

inline LimbPass operator++(LimbPass& limbPass) {
	return limbPass = (LimbPass) ((int) limbPass + 1);
}

struct OpTree;
struct OpLimbStorage;

struct OpLimb {
	OpLimb() {
#if OP_DEBUG
		edge = nullptr;
		lastLimbEdge = nullptr;
		parent = nullptr;
		linkedIndex = OpMax;
		gapDistance = OpNaN;
		closeDistance = OpNaN;
		match = EdgeMatch::none;
		lastMatch = EdgeMatch::none;
		treePass = LimbPass::none;
		deadEnd = (bool) -1;
		looped = (bool) -1;
		resetPass = (bool) -1;
#endif
		OP_DEBUG_DUMP_CODE(id = 0);
	}
	void addEach(OpJoiner& , OpTree& );
	void set(OpTree& , OpEdge* , OpLimb* parent, EdgeMatch , LimbPass , 
			size_t index, OpEdge* otherEnd, const OpPointBounds* bounds = nullptr);
	OpLimb* tryAdd(OpTree& , OpEdge* , EdgeMatch , LimbPass , size_t index = 0, 
			OpEdge* first = nullptr);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
	std::string debugDumpIDs(DebugLevel , bool bracket) const;
#endif

	OpPointBounds bounds;
	OpEdge* edge;
	OpEdge* lastLimbEdge;
	const OpLimb* parent;
	OpPtT lastPtT;
	uint32_t linkedIndex;
	float gapDistance;
	float closeDistance;
	EdgeMatch match; // end of edge that matches last point in parent limb
	EdgeMatch lastMatch;
	LimbPass treePass;  // if linked or miswound: if match is end, edge is last in linked list
	bool deadEnd;
	bool looped;
	bool resetPass;  // when new parent is found, restart limb pass

#if OP_DEBUG_DUMP
	std::vector<OpLimb*> debugBranches;
	int id;
#endif
};


// !!! eventually (if this works) add tree (or limb storage) to joiner
// prefer the looped limb with the smallest perimeter 
struct OpTree {
	OpTree(OpJoiner& );
	OP_DEBUG_CODE(~OpTree());
	void addDisabled(OpJoiner& );
	OpEdge* addFiller(const OpPtT& , const OpPtT& );
	void addUnsectableLoop(OpJoiner& , OpLimb* );
	bool contains(OpLimb* , OpEdge* ) const;
	bool containsFiller(OpLimb* , OpPoint , OpPoint ) const;
	bool containsParent(OpLimb* , OpEdge* , EdgeMatch ) const;
	void initialize(OpJoiner& join);
	bool join(OpJoiner& );
	OpLimb& nthLimb(int index);
	OpLimb* makeLimb();
	bool preferSibling(OpLimb*, OpEdge* );
	OpLimb* unsectableLoop() const;
	DUMP_DECLARATIONS
	OP_DEBUG_IMAGE_CODE(void debugLimbEdges(OpEdge*));

//	OpLimbStorage* limbStorage;
//	OpLimbStorage* current;
	OpContours* contours;
	OpLimb* bestGapLimb;  // used only by detached pass
	const OpLimb* bestLimb;   // index into limbStorage
	OpPoint firstPt;
	LimbPass limbPass;
	float bestDistance;  // used only by detached pass
	float bestPerimeter;
	int maxLimbs;
//	int baseIndex;
	int totalUsed;
};

struct OpLimbStorage {
	OpLimbStorage()
		: nextBlock(nullptr)
		, prevBlock(nullptr)
		, baseIndex(0)
		, used(0) {
		static_assert((ARRAY_COUNT(storage) - 1 & ARRAY_COUNT(storage)) == 0);
	}
	OpLimb* allocate();
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
	int baseIndex;
	int used;
};


#endif
