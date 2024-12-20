// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegments_DEFINED
#define OpSegments_DEFINED

#include "OpMath.h"

struct OpContours;
struct OpSegment;
enum class FoundIntersections;
enum class IntersectResult;

struct OpSegments {
	OpSegments(OpContours& contours);
	static void AddEndMatches(OpSegment* seg, OpSegment* opp);
	static void AddLineCurveIntersection(OpSegment* opp, OpSegment* seg);
	void findCoincidences();
	static void FindCoincidences(OpContours* );  // new interface
//    void findLineCoincidences();
	FoundIntersections findIntersections();
//    FoundIntersections findIntersectionsX();
//    static FoundIntersections FindIntersections(OpContours* );  // new interface
	static IntersectResult LineCoincidence(OpSegment* seg, OpSegment* opp);

#if OP_DEBUG_DUMP
	#include "OpDebugDeclarations.h"
#endif

	std::vector<OpSegment*> inX;
	OP_DEBUG_CODE(int debugFailSegID);
	OP_DEBUG_CODE(int debugFailOppID);
};

#endif
