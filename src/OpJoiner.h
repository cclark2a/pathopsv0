// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpSegment.h"

struct OpContour;
struct OpContext;
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
	OpJoiner(OpContext& contours);
	static bool LinkEnd(OpEdge *);
	bool linkRemaining(OpContour* );
	void linkUnambiguous(OpContour* , LinkPass );
	static OpEdge* LinkStart(OpEdge *);
	bool matchLinks(bool popLast);
	bool setup();
	void sort();
#if OP_DEBUG
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

	FoundEdge bestGap;
	OpContext* context;
	EdgeMatch linkMatch;
	LinkPass linkPass;
	OpEdge* edge;  // start of current link list
	OpEdge* lastLink;  // end of current link list
	OpPoint matchPt;
	OP_DEBUG_CODE(int debugRecursiveDepth);
};

// keep track of all edge possibilities to find the best closing path
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
		linkedContour = nullptr;
		linkedIndex = OpMax;
		gapDistance = OpNaN;
		closeDistance = OpNaN;
		match = EdgeMatch::none;
		lastMatch = EdgeMatch::none;
		treePass = LimbPass::none;
		deadEnd = (bool) -1;
		looped = (bool) -1;
		resetPass = (bool) -1;
		deferredUnsectable = (bool) -1;
#endif
		OP_DEBUG_DUMP_CODE(id = 0);
	}
	void addEach(OpContour& , OpTree& );
	void set(OpTree& , OpEdge* , OpLimb* parent, EdgeMatch , LimbPass , OpContour* ,
			size_t index, OpEdge* otherEnd, const OpPointBounds* bounds = nullptr);
	OpLimb* tryAdd(OpTree& , OpEdge* , EdgeMatch , LimbPass , 
			OpContour* limbContour = nullptr,
			size_t index = 0, OpEdge* first = nullptr);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
	std::string debugDumpIDs(DebugLevel , bool bracket) const;
#endif

	OpPointBounds bounds;
	OpEdge* edge;
	OpEdge* lastLimbEdge;
	const OpLimb* parent;
	OpContour* linkedContour;
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
	bool deferredUnsectable;

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
	void addDisabled(OpContour& );
	OpEdge* addFiller(OpSegment* , const OpPtT& , const OpPtT& );
	void addUnsectableLoop(OpJoiner& , OpLimb* );
	bool contains(OpLimb* , OpEdge* ) const;
	bool containsFiller(OpLimb* , OpPoint , OpPoint ) const;
	bool containsDeferred(OpPoint , OpPoint ) const;
	bool containsParent(OpLimb* , OpEdge* , EdgeMatch ) const;
	void initialize(OpContour& join);
	bool join(OpJoiner& );
	OpLimb& nthLimb(int index);
	OpLimb* makeLimb();
	bool preferSibling(OpLimb*, OpEdge* );
	OpLimb* unsectableLoop() const;
	DUMP_DECLARATIONS
	OP_DEBUG_IMAGE_CODE(void debugLimbEdges(OpEdge*);)  // ; outside errors

//	OpLimbStorage* limbStorage;
//	OpLimbStorage* current;
	OpContext* context;
	OpLimb* bestGapLimb;  // used only by detached pass
	const OpLimb* bestLimb;   // index into limbStorage
	OpPoint firstPt;
	LimbPass limbPass;
	float bestDistance;  // used only by detached pass
	float bestPerimeter;
	int maxLimbs;
//	int baseIndex;
	int totalUsed;
	int id;
	bool deferUnsectable;
};

struct OpLimbStorage {
	OpLimbStorage()
		: nextBlock(nullptr)
		, prevBlock(nullptr)
		, baseIndex(0)
		, used(0) {
		static_assert(((ARRAY_COUNT(storage) - 1) & ARRAY_COUNT(storage)) == 0);
	}
	OpLimb* allocate();
	void reset();
#if OP_DEBUG_DUMP
	int debugCount() const;
	const OpLimb* debugFind(int ID) const;
	OpLimb* debugIndex(int index);
	static void DumpSet(const char*& , OpContext* );
	DUMP_DECLARATIONS
#endif

	OpLimbStorage* nextBlock;
	OpLimbStorage* prevBlock;
	OpLimb storage[256];
	int baseIndex;
	int used;
};


#endif
