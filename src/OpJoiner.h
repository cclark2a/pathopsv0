// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpSegment.h"

struct OpContour;
struct OpContext;
struct OpOutPath;

#define LinkPass_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(normal), \
	OP_ENUM_MEMBER(unsectable), \
	OP_ENUM_MEMBER(remaining),

enum class LinkPass {
    LinkPass_Enums
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
	bool contains(OpEdge* e) {
		return l.end() != std::find(l.begin(), l.end(), e); }
    void clear();
	void sort(OpContext* );

	DUMP_DECLARATIONS

	std::vector<OpEdge*> l;
};

struct OpJoiner {
    OP_DEBUG_DUMP_CODE(OpJoiner(DumpSerialization , OpContext* );)
	OpJoiner(OpContext& );
	OP_DEBUG_CODE(~OpJoiner());
	static bool LinkEnd(OpEdge *);
	bool linkRemaining(OpContour* );
	void linkUnambiguous(OpContour* , LinkPass );
	static OpEdge* LinkStart(OpEdge *);
	bool matchLinks(OpContour* , bool popLast);
	bool setup();
	void sort();
	bool unsectableLink(OpContour* , OpPoint start, OpPoint end);
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_SERIALIZE
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
	OP_DEBUG_CODE(int debugRecursiveDepth);
};

#define LimbPass_Base \
    OP_ENUM_BASE(uninitialized, -1)

//	!!! removed OP_ENUM_MEMBER(alternateEnd),  /* construct line using opposite sect edge end (e.g, testQuads25659799) */
#define LimbPass_Enums \
	OP_ENUM_MEMBER(none), \
	OP_ENUM_MEMBER(linked),    /* in linkups list with correct winding */ \
	OP_ENUM_MEMBER(unlinked),  /* in unsectByArea and in unsortables */ \
	OP_ENUM_MEMBER(unsectPair), /* gap to other edge in unsectable pair */ \
	OP_ENUM_MEMBER(disabledCenterless),  /* in disabled, and so small no center could be computed */ \
	OP_ENUM_MEMBER(disabledPals),  /* in disabled pals */ \
	OP_ENUM_MEMBER(miswound),  /* in linkups list, including entries with the wrong winding */ \
	OP_ENUM_MEMBER(disjoint),  /* gap to closest in linkups list, or gap to edge start (loop) */ \
	OP_ENUM_MEMBER(unlinkedPal),  /* unlinked variant that permits siblings to connect to seen edges' pals */ \
	OP_ENUM_MEMBER(smallEdge ),  /* edges from nearby intersections (delta t is below set value) */ \
	OP_ENUM_MEMBER(disabledBackwards),  /* undetected mis-sort may be closable (e.g, loop156850) */ \
	OP_ENUM_MEMBER(debugStop)  /* debugging aid when limb pass is advanced past final value */

// keep track of all edge possibilities to find the best closing path
enum class LimbPass : int8_t {
	LimbPass_Base,
    LimbPass_Enums
};

inline LimbPass operator++(LimbPass& limbPass) {
	return limbPass = (LimbPass) ((int) limbPass + 1);
}

struct OpTree;
struct OpLimbStorage;

struct OpLimb {
	void addEach(OpContour& , OpTree& );
	bool ptsMatch(EdgeMatch limbEnd, const std::vector<OpPoint>& ) const;
	bool ptsMatch(EdgeMatch limbEnd, const OpLimb* test, EdgeMatch testEnd) const;
	void set(OpTree& , OpEdge* , OpLimb* parent, EdgeMatch , LimbPass , OpContour* ,
			size_t index, OpEdge* otherEnd, const OpPointBounds* bounds = nullptr);
	OpLimb* tryAdd(OpTree& , OpEdge* , EdgeMatch , LimbPass , 
			OpContour* limbContour = nullptr,
			size_t index = 0, OpEdge* first = nullptr);
	DUMP_DECLARATIONS
#if OP_DEBUG_SERIALIZE
	std::string debugDumpIDs(DebugLevel , bool bracket) const;
#endif
	OpPointBounds limbBounds;  // bounds of this limb and any branches
	std::vector<OpPoint> firstPts;  // only required for first limb in tree, but eases debugging
	std::vector<OpPoint> lastPts;  // [0] is last t's point; others are sect aliases
	OpEdge* edge  OP_DEBUG_INIT_PTR(OpEdge);
	OpEdge* lastLimbEdge  OP_DEBUG_INIT_PTR(OpEdge);
	const OpLimb* parent  OP_DEBUG_INIT_PTR(const OpLimb);
	OpContour* linkedContour  OP_DEBUG_INIT_PTR(OpContour);
	float lastT  OP_DEBUG_INIT_FLOAT();
	float gapDistance  OP_DEBUG_INIT_FLOAT();
	float closeDistance  OP_DEBUG_INIT_FLOAT();
	uint32_t linkedIndex  OP_DEBUG_INIT_UINT();
	EdgeMatch match  OP_DEBUG_INIT(EdgeMatch); // end of edge that matches last point in parent limb
	EdgeMatch lastMatch  OP_DEBUG_INIT(EdgeMatch);
	LimbPass treePass  OP_DEBUG_INIT(LimbPass);  // linked/miswound: if match is end, last in linked
	bool deadEnd  OP_DEBUG_INIT_BOOL();
	bool looped  OP_DEBUG_INIT_BOOL();
	bool resetPass  OP_DEBUG_INIT_BOOL();  // when new parent is found, restart limb pass

#if OP_DEBUG_SERIALIZE
	std::vector<OpLimb*> debugBranches;
	int id = 0;
#endif
};

// !!! eventually (if this works) add tree (or limb storage) to joiner
// prefer the looped limb with the smallest perimeter 
struct OpTree {
    OP_DEBUG_DUMP_CODE(OpTree(DumpSerialization , OpContext* );)
	OpTree(OpEdge* );
	OP_DEBUG_CODE(~OpTree());
//	void addAlternateEnd();
	void addDisabled(OpContour& );
	OpEdge* addFiller(OpSegment* , OpPoint , OpPoint , bool fromCC);
//	void addUnsectableLoop(OpJoiner& , OpLimb* );
	bool contains(OpLimb* , OpEdge* ) const;
	bool containsFiller(OpLimb* , OpPoint , OpPoint ) const;
	bool containsFiller(int ccUnsectableID) const;
	bool containsParent(OpLimb* , OpEdge* , EdgeMatch ) const;
	float firstDistance(OpPoint pt) const;
	bool firstMatch(OpPoint pt) const;
	bool gap(float distance) const;
	void initialize(OpContour& join);
	bool join(OpJoiner& );
	OpLimb& nthLimb(int index);
	OpLimb* makeLimb();
	void makeTrunk(OpEdge* );
	bool preferSibling(OpLimb*, OpEdge* );
//	OpLimb* unsectableLoop() const;
	DUMP_DECLARATIONS
	OP_DEBUG_IMAGE_CODE(void debugLimbEdges(OpEdge*);)  // ; outside errors

	OpContext* context;
	OpLimb* trunk;
	OpLimb* bestGapLimb;  // used only by detached pass
	const OpLimb* bestLimb;   // index into limbStorage
	float bestDistance;  // used only by detached pass
	float bestPerimeter;
	int maxLimbs;
	int totalUsed;
	int id;
	LimbPass limbPass;
	bool disabled;  // set when found contour is proportionately made up of disabled edges
	bool smallGap;
	OP_DEBUG_CODE(int debugAddEach = 0);
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
#if OP_DEBUG_SERIALIZE
	int debugCount() const;
	OpLimb* debugFind(int ID) const;
	OpLimb* debugIndex(int index) const;
#endif
#if OP_DEBUG_DUMP
	static void DumpSet(const char*& , OpContext* );
#endif
	DUMP_DECLARATIONS

	OpLimbStorage* nextBlock;
	OpLimbStorage* prevBlock;
	OpLimb storage[256];
	int baseIndex;
	int used;
};


#endif
