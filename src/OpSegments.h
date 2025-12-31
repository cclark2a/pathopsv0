// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegments_DEFINED
#define OpSegments_DEFINED

#include "OpMath.h"

struct OpContour;
struct OpContext;
struct OpSegment;
enum class FoundIntersections;
enum class IntersectResult;

struct OpSegments {
	OpSegments(OpContext& );
	static std::vector<OpIntersection*> AddEndMatches(OpSegment* seg, OpSegment* opp);
	static void AddLineCurveIntersection(OpSegment* opp, OpSegment* seg, 
			std::vector<OpIntersection*>& matchingSects);
    static void AddEndMatches(OpContour* seg, OpContour* opp);
	void findCoincidence(OpContour* seg, OpContour* opp);
	bool findCoincidence(OpSegment* seg, OpSegment* opp);
	void findCoincidences();
	FoundIntersections findIntersections();
	void findIntersection(OpContour* seg, OpContour* opp);
	bool findIntersection(OpSegment* seg, OpSegment* opp);
	void initInX();
	static IntersectResult LineCoincidence(OpSegment* seg, OpSegment* opp);
	static void SwapEndMatches(std::vector<OpIntersection*>& );
//	std::vector<OpSegment*> inX;
	OpContext& context;
	FoundIntersections found;
	OP_DEBUG_CODE(int debugFailSegID);
	OP_DEBUG_CODE(int debugFailOppID);
};

#endif
