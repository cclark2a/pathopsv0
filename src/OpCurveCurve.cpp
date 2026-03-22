// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

// in single-threaded debugging mode, keep every level of recursion for serializing to debugger
#if OP_DEBUG_DUMP
DumpCurveCurve dumpCurveCurve;
#endif

enum class IsCoin {
	no,
	yes
};

struct IdEnds {
	int id;
	MatchEnds matchEnds;
};

struct SectDuo {
	OpIntersection* s;
	OpIntersection* o;
	bool alreadySet;
};

bool EdgeRun::inDeleted(CcCurves* curves, CcCurves* oppCurves) const {
	return curves->deletedT(edgePtT.t) || oppCurves->deletedT(oppPtT.t);
}

void EdgeRun::set(OpEdge* edge, OpSegment* opp, EdgeMatch match, float scaledMax  OP_LINE_FILE_ARGS()) {
	edgePtT = EdgeMatch::start == match ? edge->startPtT() : edge->endPtT();
	oppPtT = edge->segment->distance(edgePtT, opp);
	if (OpMath::IsFinite(oppPtT.t)) {
		setOppDist(edge->segment, scaledMax);
		if (EdgeMatch::start == match)
			edge->startDist = { oppPtT, oppDist };
		else
			edge->endDist = { oppPtT, oppDist };
	} else
		oppDist = OpNaN;
	fromFoundT = LimitFrom::no;
	byZero = false;
//	OP_DEBUG_CODE(debugBetween = 1);
    OP_LINE_FILE_SET(debugSetMaker);
}

bool EdgeRun::set(OpPtT& sPtT, OpPtT& oPtT, OpSegment* seg, float scaledMax  OP_LINE_FILE_ARGS()) {
    OP_LINE_FILE_SET(debugSetMaker);
    if (!sPtT.isFinite())
        return false;
    if (!oPtT.isFinite())
        return false;
    edgePtT = sPtT;
    oppPtT = oPtT;
    setOppDist(seg, scaledMax);
	fromFoundT = LimitFrom::no;
	byZero = false;
//	OP_DEBUG_CODE(debugBetween = 1);
    return true;
}

// start here;
// if distance is zero, that defines an intersection (which may or may not have already been found)
// record intersection, and snip out (by marking as deleted?)
void EdgeRun::setOppDist(const OpSegment* segment, float scaledMax) {
	if (OpMath::IsNaN(edgePtT.t)) {
		oppDist = OpNaN;
		return;
	}
	OpVector oppV = edgePtT.pt - oppPtT.pt;
	oppDist = oppV.length();
	if (OpMath::IsNaN(oppDist))
		return;
	if (oppDist <= scaledMax) {
		oppDist = 0;
//		OpAssert(OpMath::NearlyEndT(edgePtT.t) && OpMath::NearlyEndT(oppPtT.t));
		return;
	}
	OpVector normal = segment->c.normal(edgePtT.t);
	float nDotOpp = normal.dot(oppV);
	if (nDotOpp < -OpEpsilon)
		oppDist = -oppDist;
	else if (nDotOpp <= OpEpsilon)
		oppDist = 0;
}

FoundLimit::FoundLimit(OpEdge* edge, OpEdge* oEdge, const OpPtT& edgePtT, const OpPtT& oEdgePtT
			OP_LINE_FILE_ARGS())
	: parentEdge(edge)
	, parentOpp(oEdge)
	, segPtT(edgePtT)
	, oppPtT(oEdgePtT)
	, fromFoundT(edge ? LimitFrom::no : LimitFrom::yes)
	, oppOutOfOrder(Unordered::no)
	, used(LimitUsed::no)
	, match(edge ? LimitMatch::no : LimitMatch::yes)
	, swapped(LimitSwapped::no)
	, bettered(LimitBettered::no) {
	OP_LINE_FILE_SET(debugMaker);
	if (!edge)
		return;
	OP_ASSERT(edge->curve.isLineSet);
	OP_ASSERT(oEdge->curve.isLineSet);
	edgeLine = edge->isLine() ? LimitLine::yes : LimitLine::no;
	oppLine = oEdge->isLine() ? LimitLine::yes : LimitLine::no;
}

// if snip supercedes existing snip, replace it
void FoundLimits::addSnip(const OpPtT& sPtT, const OpPtT& oPtT) {
	SnipPtTs snipLo { sPtT, oPtT };
	SnipPtTs snipHi { sPtT, oPtT };
	addSnipCommon(snipLo, snipHi);
}

void FoundLimits::addSnipCommon(SnipPtTs& snipLo, SnipPtTs& snipHi) {
	cutPair(snipLo, snipHi);
	auto snipPush = [this, snipLo, snipHi]() {
		snips.push_back({ snipLo.segPtT, snipLo.oppPtT, { snipLo.segCut.lo, snipHi.segCut.hi },
				{ snipLo.oppCut.lo, snipHi.oppCut.hi }} );
	};
	if (snips.empty()) {
		snipPush();
		return;
	}
	// check if added is in current snip range
	bool copySnip = true;
	for (size_t index = 0; index < snips.size(); ) {
		SnipPtTs& existing = snips[index];
		auto snipIntersects = [](CutRangeT& exists, CutRangeT& newSnip) {
			LoHi diffLoHi { exists.lo.t, exists.hi.t, DiffIntersect::intersect };  // assumed answer for exists minus snip
			if (newSnip.lo.t <= exists.lo.t && exists.hi.t <= newSnip.hi.t) {
				diffLoHi.diffSect = DiffIntersect::replace;
				return diffLoHi;
			}
			if (exists.lo.t < newSnip.lo.t && newSnip.lo.t < exists.hi.t)
				diffLoHi.hi = newSnip.lo.t;
			if (exists.lo.t < newSnip.hi.t && newSnip.hi.t < exists.hi.t)
				diffLoHi.lo = newSnip.hi.t;
			if (diffLoHi.lo != exists.lo.t && diffLoHi.hi != exists.hi.t)
				diffLoHi.diffSect = DiffIntersect::ignore;
			return diffLoHi;
		};
		LoHi segDiff = snipIntersects(existing.segCut, snipLo.segCut);
		LoHi oppDiff = snipIntersects(existing.oppCut, snipLo.oppCut);
		if (DiffIntersect::ignore == segDiff.diffSect && DiffIntersect::ignore == oppDiff.diffSect)
			copySnip = false;
		bool keptDiff = DiffIntersect::replace != segDiff.diffSect 
					&& DiffIntersect::replace != oppDiff.diffSect;
		if (keptDiff && !cc->edgeCurves.c.empty())
			keptDiff = cc->edgeCurves.keepDiff(segDiff) 
					|| cc->oppCurves.keepDiff(oppDiff);
		if (keptDiff) {
			++index;
			continue;
		}
		if (!copySnip)
			snips.erase(snips.begin() + index);
		else {
			snips[index] = { snipLo.segPtT, snipLo.oppPtT, { snipLo.segCut.lo, snipHi.segCut.hi },
					{ snipLo.oppCut.lo, snipHi.oppCut.hi } };
			++index;
			copySnip = false;
		}
	}
	if (copySnip)
		snipPush();
}

bool FoundLimits::addSnipRange(size_t start) {
	if (start == i.size())
		return false;
	if (start + 1 == i.size())
		addSnip(i[start].segPtT, i[start].oppPtT);
	else {
		std::sort(i.begin() + start, i.end(), [](const FoundLimit& a, const FoundLimit& b) {
			return a.segPtT.t < b.segPtT.t; });
		SnipPtTs snipLo { i[start].segPtT, i[start].oppPtT };
		SnipPtTs snipHi { i.back().segPtT, i.back().oppPtT };
		addSnipCommon(snipLo, snipHi);
	}
	return true;
}

bool FoundLimits::alreadyIn(const OpPtT& edgePtT, const OpPtT& oppPtT) const {
	for (const FoundLimit& limit : i) {
#if 0  // !!! this test is undoubtably required, but as written is too broad, dropping valid
	   //     intersections (e.g., loop8452, segments 8 & 4 intersect multiple times)
		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
			return true;  // reverse case already recorded
#endif
		if (OpMath::EqualT(limit.segPtT.t, edgePtT.t))
			return true;  // already recorded
		OpPointBounds bounds(limit.segPtT.pt, limit.oppPtT.pt);
		if (bounds.contains(edgePtT.pt))
			return true;
		if (bounds.contains(oppPtT.pt))
			return true;
	}
	return false;
}

struct CutStep {
	CutStep (const OpCurve& c) 
	: curve(c) {
		const OpContext& context = curve.context();
		PathOpsV0Lib::CurveConst cutFun = context.callbacks[curve.c.type].maxCutFuncPtr;
		float tStep = cutFun ? (*cutFun)(curve.c) : 16.f;
		OP_ASSERT(tStep);  // !!! error if zero?
		threshold = context.threshold * tStep;
		tDiff = tStep * OpEpsilon;
	}

	const OpCurve& curve;
	OpVector threshold;
	float tDiff;
};

struct CutUp {
	CutUp(const CutStep& c_s, OpPtT& p_t, OpVector gap, float dir)
		: cutStep(c_s)
		, ptT(p_t)
		, ptGap(gap)	// !!! caller may need to scale 
		, direction(dir) 
		{
		hasGap = gap.dx > cutStep.threshold.dx || gap.dy > cutStep.threshold.dy; 
		diff[0] = diff[1] = cutStep.tDiff;
		findCuts();
	}

	bool advanceFibonacci() {
		float sum = diff[0] + diff[1];
		diff[0] = diff[1];
		diff[1] = std::min(sum, 1.f);
		return sum < 1.f;
	}

	bool advanceFound() {
		float oldCutT = cutPtT.t;
		if (!advanceFibonacci())
			hitEnd();
		else
			findCuts();
		return oldCutT != cutPtT.t;
	}

	void findCuts() {
	// get cut locations for curve and opp; iterate on both while cut point is nearly original
		// not necessary to put in limit; fibonacci will terminate when it exceeds 1.f
		do {
			float cutT = std::max(0.f, std::min(1.f, ptT.t + diff[1] * direction));
			if (ptT.t == cutT) {
				cutPtT = ptT;
				return;
			}
			OpPtT cut = cutStep.curve.ptTAtT(cutT);
			if (!cut.pt.isNearly(ptT.pt, cutStep.threshold) 
					&& (!hasGap || !cut.pt.isNearly(ptT.pt, ptGap))) {
				cutPtT = cut;
				return;
			}
		} while (advanceFibonacci());
		hitEnd();
	}

	void hitEnd() {
		if (-1 == direction)
			cutPtT = { cutStep.curve.c.data->start, 0 };
		else
			cutPtT = { cutStep.curve.c.data->end, 1 };
	}

	void set(const CutUp& cutUp) {
		diff = cutUp.diff;
		cutPtT = cutUp.cutPtT;
	}

	std::array<float, 2> diff;
	const CutStep& cutStep;
	OpPtT ptT;
	OpPtT cutPtT;
	OpVector ptGap;
	float direction;  // -1 or 1
	bool hasGap;
};


// find extent of curve pair where they are nearly equal, so a subsequent snip can exclude from sect
void FoundLimits::cutPair(SnipPtTs& snipLo, SnipPtTs& snipHi) const {
	CutStep sStep(cc->seg->c);
	OpVector loGap { fabsf(snipLo.segPtT.pt.x - snipLo.oppPtT.pt.x),
			         fabsf(snipLo.segPtT.pt.y - snipLo.oppPtT.pt.y) };
	OpVector hiGap { fabsf(snipHi.segPtT.pt.x - snipHi.oppPtT.pt.x),
			         fabsf(snipHi.segPtT.pt.y - snipHi.oppPtT.pt.y) };
	CutUp sLo(sStep, snipLo.segPtT, loGap, -1);
	CutUp sHi(sStep, snipHi.segPtT, hiGap, +1);
    CutStep oStep(cc->opp->c);
	CutUp oLo(oStep, snipLo.oppPtT, loGap, cc->reversed ? +1 : -1);  // opp may be backwards
	CutUp oHi(oStep, snipHi.oppPtT, hiGap, cc->reversed ? -1 : +1);
	float segLen = (sHi.cutPtT.pt - sLo.cutPtT.pt).lengthSquared();
	float oppLen = (oHi.cutPtT.pt - oLo.cutPtT.pt).lengthSquared();
	OP_ASSERT(OpMath::IsFinite(segLen) && OpMath::IsFinite(oppLen));
	float largerLen = std::max(segLen, oppLen);
	// find smaller cut length of curve and opp; iterate until it is larger, then keep next smaller
	CutUp smallerLo = segLen < oppLen ? sLo : oLo;
	CutUp smallerHi = segLen < oppLen ? sHi : oHi;
	for (;;) {
		if (smallerLo.advanceFibonacci())
			smallerLo.findCuts();
		if (smallerHi.advanceFibonacci())
			smallerHi.findCuts();
		float testLen = (smallerHi.cutPtT.pt - smallerLo.cutPtT.pt).lengthSquared();
		if (testLen > largerLen)
			break;
		(segLen < oppLen ? sLo : oLo).set(smallerLo);
		(segLen < oppLen ? sHi : oHi).set(smallerHi);
	}
	// if resulting cut of curve & opp are nearly equal, increase cut of both, and try again
	OpVector maxThreshold = sStep.threshold.lengthSquared() < oStep.threshold.lengthSquared() 
			? oStep.threshold : sStep.threshold;
	OpVector maxGap;
	bool hasMaxGap = sLo.hasGap || oLo.hasGap || sHi.hasGap || oHi.hasGap;
	if (hasMaxGap)
		maxGap = loGap.lengthSquared() < hiGap.lengthSquared() ? hiGap : loGap;
	bool changed;
	do {
		changed = false;
		if (sLo.cutPtT.pt.isNearly(oLo.cutPtT.pt, maxThreshold)
				|| (hasMaxGap && sLo.cutPtT.pt.isNearly(oLo.cutPtT.pt, maxGap))) {
			changed = sLo.advanceFound();
			changed |= oLo.advanceFound();
		}
		if (sHi.cutPtT.pt.isNearly(oHi.cutPtT.pt, maxThreshold)
				|| (hasMaxGap && sHi.cutPtT.pt.isNearly(oHi.cutPtT.pt, maxGap))) {
			changed |= sHi.advanceFound();
			changed |= oHi.advanceFound();
		}
	} while (changed);
	snipLo.segCut.lo = sLo.cutPtT;
	snipLo.oppCut.lo = oLo.cutPtT;
	snipHi.segCut.hi = sHi.cutPtT;
	snipHi.oppCut.hi = oHi.cutPtT;
	if (snipLo.oppCut.lo.t > snipHi.oppCut.hi.t)
		std::swap(snipLo.oppCut.lo, snipHi.oppCut.hi);
}

	// mark unsectables with opposite t values that are not ordered
	// !!! start out conservative and only mark i outside of first/last
void FoundLimits::markOutOfOrder() {
	if (i.size() > 2) {
		float frontOppT = i.front().oppPtT.t;
		float backOppT = i.back().oppPtT.t;
		for (size_t index = 1; index < i.size() - 1; ++index) {
			FoundLimit& limit = i[index];
			if (!OpMath::Between(frontOppT, limit.oppPtT.t, backOppT))
				limit.oppOutOfOrder = Unordered::yes;
		}
	}
}

void FoundLimits::setEdge(const OpEdge* edge) {
	for (FoundLimit& limit : i) {
		OP_ASSERT(!limit.parentEdge);
		limit.parentEdge = edge;
	}
}

void FoundLimits::setEnds(std::vector<OpIntersection*>& matchingSects) {
	cc->reversed = false;
    for (OpIntersection* matchingSect : matchingSects) {
        OP_ASSERT(matchingSect->segment == cc->seg);
        OP_ASSERT(matchingSect->opp->segment == cc->opp);
        OpPtT& sPtT = matchingSect->ptT;
        OpPtT& oPtT = matchingSect->opp->ptT;
        cc->reversed |= (0 == sPtT.t && 1 == oPtT.t) || (1 == sPtT.t && 0 == oPtT.t);
	    smSegT |= 0 == sPtT.t;
	    lgSegT |= 1 == sPtT.t;
	    smOppT |= 0 == oPtT.t;
	    lgOppT |= 1 == oPtT.t;
        cc->splitMid = true;
		FoundLimit smT(nullptr, nullptr, sPtT, oPtT  OP_LINE_FILE_PARGS()); // no edges
		i.push_back(std::move(smT));
		addSnip(sPtT, oPtT);
    }
}

void FoundLimits::setOpp(const OpEdge* opp) {
	for (FoundLimit& limit : i) {
		OP_ASSERT(!limit.parentOpp);
		limit.parentOpp = opp;
	}
}

void FoundLimits::setUnique() {
	if (i.size() <= 1)
		unique = (int) i.size();
	if (unique >= 0)
		return;
	sort();
	unique = 1;
	const FoundLimit* last = &i[0];
	float lastDistSq = (last->segPtT.pt - last->oppPtT.pt).lengthSquared();
	OpVector threshold = cc->seg->threshold();
	for (size_t index = 1; index < i.size(); ++index) {
		const FoundLimit* limit = &i[index];
		bool soClose = last->segPtT.isNearly(limit->segPtT, threshold);
		soClose |= last->oppPtT.isNearly(limit->oppPtT, threshold);
		float distSq = (limit->segPtT.pt - limit->oppPtT.pt).lengthSquared();
		if (!soClose) {
			OpPtT midPtT = cc->seg->c.ptTAtT(OpMath::Average(last->segPtT.t, i[index].segPtT.t));
			OpPtT midOpp = cc->seg->distance(midPtT, cc->opp);
			float midDist = (midPtT.pt - midOpp.pt).lengthSquared();
			if (midDist > lastDistSq && midDist > distSq)
				++unique;
		}
		last = &i[index];
		lastDistSq = distSq;
	}
}

/* main cases: (where smaller, larger are distances) existing1=1, existing2=2, n=newDistance
   if existing changes sign, insert
   if existing same sign, replace high with higher, low with lower, or discard
   if new run distance is zero, return runs' pointer to that new run  
*/
EdgeRun* CcCurves::addEdgeRun(OpEdge* edge, EdgeMatch match, ClampDist clampDist  OP_LINE_FILE_ARGS()) {
	EdgeRun run;
	run.set(edge, opp, match, ClampDist::yes == clampDist ? scaledMax : 0  OP_LINE_FILE_CARGS());
    return addEdgeRun(run, match, clampDist);
}

EdgeRun* CcCurves::addEdgeRun(EdgeRun& run, EdgeMatch match, ClampDist clampDist) {
    OP_DEBUG_CODE(debugAdd(run));
	if (OpMath::IsNaN(run.oppDist))
		return nullptr;
	// skip adding new run if it is between existing run
	// otherwise, find run before, and its index
	// binary search
	OP_DEBUG_VALIDATE_CODE(debugValidate());
    int lo = runs.empty() ? 0 : insertPos(runs, run);  // binary search, then add / modify run as needed
    if (0 > lo || (0 == lo && !runs.empty() && run.edgePtT.t == runs.front().edgePtT.t))
        return nullptr;
    auto insertRun = [this, lo, run]() {
    	runs.insert(runs.begin() + lo, run);  // runs are sorted by edge pt t
	    return run.oppDist ? nullptr : &runs.front() + lo;
    };
#if 0  // consolidating runs misses sects where the curves cross over twice (e.g. loop191404)
       // need to detect best pair that point to possible intersections even though all four
       // runs have the same opposite sign (pair both pointing towards a zero crossing)
       // that is, middle of four have smaller magnitude than outer of four
	if (0 < lo && lo < (int) runs.size()) {
		EdgeRun& runStart = runs[lo - 1];
		EdgeRun& runEnd = runs[lo];
		OP_ASSERT(runStart.edgePtT.t < run.edgePtT.t && run.edgePtT.t < runEnd.edgePtT.t);
		if (runStart.oppDist * runEnd.oppDist >= 0) {  // true if start, end have same sign
			if (OpMath::Between(runStart.oppDist, run.oppDist, runEnd.oppDist)) {
				OP_DEBUG_CODE(++runStart.debugBetween);
				return nullptr;
			}
            // if run smaller / run larger are diffent sign from run, insert
            float existing = !runStart.oppDist ? runEnd.oppDist : runStart.oppDist;
            if (run.oppDist * existing < 0)
                return insertRun();
			bool startSmaller = fabs(runStart.oppDist) < fabs(runEnd.oppDist);
			bool runSmaller = fabs(run.oppDist) < fabs(runs[lo - startSmaller].oppDist); 
			OP_DEBUG_CODE(run.debugBetween += runs[lo - (runSmaller == startSmaller)].debugBetween);
			EdgeRun* runPtr = &runs[lo - (runSmaller == startSmaller)];
			*runPtr = run;
			return run.oppDist ? nullptr : runPtr;
		}
		// whichever of start or end has same sign as run
		//  if start or end is between run dist and following dist, replace start or end
		bool runSign = run.oppDist * runStart.oppDist > 0;  // check if signs are the same
		if (runSign ? lo > 1 : lo + 1 < (int) runs.size()) {
			EdgeRun& runNear = runSign ? runStart : runEnd;
			EdgeRun& runFar = runSign ? runs[lo - 2] : runs[lo + 1];
			if (runNear.oppDist * runFar.oppDist > 0  // signs are the same
					&& OpMath::Between(runFar.oppDist, runNear.oppDist, run.oppDist)) {
				OP_DEBUG_CODE(run.debugBetween += runNear.debugBetween);
				runNear = run;
				return run.oppDist ? nullptr : &runNear;
			}
		}
	} else if (runs.size() > 1) {
		if (0 == lo) {
			EdgeRun& runStart = runs[0];
			if (runStart.oppDist * run.oppDist >= 0) {
				EdgeRun& runEnd = runs[1];
				if (runStart.oppDist * runEnd.oppDist >= 0
						&& OpMath::Between(run.oppDist, runStart.oppDist, runEnd.oppDist)) {
					OP_DEBUG_CODE(run.debugBetween += runStart.debugBetween);
					runStart = run;
					return run.oppDist ? nullptr : &runStart;
				}
			}
		} else {
			OP_ASSERT(lo == (int) runs.size());
			EdgeRun& runStart = runs[runs.size() - 2];
			if (runStart.oppDist * run.oppDist >= 0) {
				EdgeRun& runEnd = runs.back();
				if (runStart.oppDist * runEnd.oppDist >= 0
						&& OpMath::Between(runStart.oppDist, runEnd.oppDist, run.oppDist)) {
					OP_DEBUG_CODE(run.debugBetween += runEnd.debugBetween);
					runEnd = run;
					return run.oppDist ? nullptr : &runEnd;
				}
			}
		}
	}
#endif
    return insertRun();
}

// note that this doesn't create an opportunity for a zero distance; it uses edge run as temporary
bool CcCurves::checkMidRun(size_t index) {
    OP_ASSERT(0 <= index && index + 1 < runs.size());
	EdgeRun& eS = runs[index];
	EdgeRun& eE = runs[index + 1];
	float midT = OpMath::Average(eS.edgePtT.t, eE.edgePtT.t);
    return checkMid(midT, eS.oppDist, eE.oppDist);
}

bool CcCurves::checkMid(float midT, float startDist, float endDist) {
	EdgeRun mid;
    OP_LINE_FILE_SET_IMMED(mid.debugSetMaker);
	mid.edgePtT = seg->c.ptTAtT(midT);
	mid.oppPtT = seg->distance(mid.edgePtT, opp);
	mid.setOppDist(seg, scaledMax);
    OP_DEBUG_CODE(debugAdd(mid));
	// ok if start oppDist > end.oppDist
	return OpMath::InUnsorted(startDist, mid.oppDist, endDist, seg->threshold().length());
}

bool CcCurves::checkMidEdge(OpEdge* edge) {
    if (!edge->startDist.isSet() || !edge->endDist.isSet())
        return true;
    if (edge->centerless)
		return false;
    return checkMid(edge->center.t, edge->startDist.dist, edge->endDist.dist);
}

void CcCurves::checkSigns() {
	if (c.empty())
		return;
	for (OpEdge* edge : c) {
		float startDst = edge->startDist.dist;
		float endDst = edge->endDist.dist;
		if (!(startDst * endDst < 0))  // allow nan
			continue;
		if (OpMath::IsNaN(edge->startDist.opp.t) || OpMath::IsNaN(edge->endDist.opp.t))
			continue;
		// use distance to weight middle of binary search
		float startT = edge->startT;
		float endT = edge->endT;
		EdgeRun run;
        OP_LINE_FILE_SET_IMMED(run.debugSetMaker);
		do {
			float rangeT = endT - startT;
			float absStart = fabs(startDst);
			float absEnd = fabs(endDst);
			float rangeDist = absStart + absEnd;
			float weight, midT;
			if (absStart < absEnd) {
				weight = absStart / rangeDist;
				midT = rangeT * weight + startT;
				if (startT == midT)
					break;
			} else {
				weight = absEnd / rangeDist;
				midT = endT - rangeT * weight;
				if (endT == midT)
					break;
			}

			run.edgePtT = seg->c.ptTAtT(midT);
			run.oppPtT = seg->distance(run.edgePtT, opp);
			run.setOppDist(seg, scaledMax);
            OP_DEBUG_CODE(debugAdd(run));
			float error = run.oppDist;
			if (0 == error) 
				break;
			if (OpMath::IsNaN(error)) 
				break;
			// !!! when estimated zero point misses, use the error (distance) to make a better guess
			// adjust the end further away, to cut down on the possible t range as much as possible

			// !!! if weight < .5, run pt is closer to start: move end close to guess

			if (startDst * error < 0) {	// guess too large
				endDst = error;
				endT = midT;
			} else {
				OP_ASSERT(endDst * error < 0);
				startDst = error;
				startT = midT;
			}
		} while (startT + OpEpsilon < endT);
		if (OpMath::IsNaN(run.oppDist)) 
			continue;
		OpPoint midPt = OpPoint::Average(run.edgePtT.pt, run.oppPtT.pt);
		float edgeLen = (run.edgePtT.pt - midPt).length();
		float oppLen = (run.oppPtT.pt - midPt).length();
		float edgeFactor = edgeLen / seg->thresholdLength();
		float oppFactor = oppLen / seg->thresholdLength();
		if (edgeFactor > 1 || oppFactor > 1)
			continue;
		run.edgePtT.pt = midPt;
		run.oppPtT.pt = midPt;
		if (seg->sects.contains(run.edgePtT, opp))
			continue;
		if (opp->sects.contains(run.oppPtT, seg))
			continue;
		OpIntersection* sect = seg->addSegBase(run.edgePtT  OP_LINE_FILE_PARAMS(opp));
		OpIntersection* oSect = opp->addSegBase(run.oppPtT  OP_LINE_FILE_PARAMS(seg));
		OP_ASSERT(sect);
		OP_ASSERT(oSect);
		cc->sectPair(sect, oSect, midPt);
	}
}

void CcCurves::clear() {
	for (auto edge : c) {
		edge->hulls.clear();
		edge->ccOverlaps = false;
	}
}

// adds edge run if distance sign flips; called when hull intersection found opposite flipped
void CcCurves::complementRun(OpEdge* oppEdge) {
    OP_ASSERT(oppEdge->startDist.dist * oppEdge->endDist.dist < 0);
    OP_ASSERT(oppEdge->segment == opp);
    auto complement = [this](EdgeDist& dist, EdgeMatch match) {
        OpPtT ptT = seg->c.ptTAtT(dist.opp.t);
        OpPtT oppPtT = seg->distance(ptT, opp);
        EdgeRun temp;
        bool setWorked = temp.set(ptT, oppPtT, seg, 0  OP_LINE_FILE_PARGS());
        if (!setWorked)
            return;
        OP_DEBUG_CODE(debugAdd(temp));
        // check if computed distance sign is not in edge run
        int lo = runs.empty() ? 0 : insertPos(runs, temp);
		OP_DEBUG_VALIDATE_CODE(debugValidate());
        if (lo > 0 && lo < (int) runs.size() && runs[lo - 1].oppDist * temp.oppDist < 0 
                && temp.oppDist * runs[lo].oppDist < 0)
            addEdgeRun(temp, match, ClampDist::no);
    };
    complement(oppEdge->startDist, EdgeMatch::start);
    complement(oppEdge->endDist, EdgeMatch::end);
}

// earlier limit may make curves discontinuous. Check to see if a gap separates lower from upper
// !!! some future optimization could keep edges rather than creating new ones with the same
//     startT/endT as existing ones
// return intersection of current edges with lower/upper
std::vector<Interval> CcCurves::continuous(const OpPtT& lower, const OpPtT& upper) const {
    OP_ASSERT(lower.t < upper.t);
    OP_ASSERT(c.size());
    std::vector<Interval> result;
    for (OpEdge* edge : c) {
        OpPtT loPtT = lower.t > edge->startT ? lower : edge->startPtT();
        OpPtT hiPtT = upper.t < edge->endT ? upper : edge->endPtT();
        if (loPtT.t < hiPtT.t)
            result.push_back({ loPtT, hiPtT });
    }
    return result;
}

bool CcCurves::deletedT(float t) const {
	for (const CutRangeT& test : deleted) {
		if (test.lo.t <= t && t <= test.hi.t)
			return true;
	}
	return false;
}

std::vector<CutRangeT> CcCurves::findGaps() const {
	std::vector<CutRangeT> gaps;
	size_t index = 0;
	while (index < c.size() && !c[index]->ccOverlaps)
		++index;
	if (index >= c.size())
		return gaps;
	OpPtT last = c[index]->endPtT();
	while (++index < c.size()) {
		const OpEdge* edgePtr = c[index];
		OP_ASSERT(edgePtr->startT >= last.t);  // if not sorted, fix
		if (!edgePtr->ccOverlaps)
			continue;
		if (edgePtr->startT != last.t)
			gaps.push_back({ last, edgePtr->startPtT() });
		last = edgePtr->endPtT();
	}
	return gaps;
}

void CcCurves::baseInit(OpCurveCurve* _cc, CcCurves* oppCs) {
    cc = _cc;
	oppCurves = oppCs;
}

void CcCurves::init(OpCurveCurve* _cc, CcCurves* oppCs, float sMax, OpEdge* parent, OpSegment* o) {
    baseInit(_cc, oppCs);
    scaledMax = sMax;
    seg = parent->segment;
    opp = o;
	initialEdgeRun(parent);
}

void CcCurves::initialEdgeRun(OpEdge* edge) {
	OP_ASSERT(runs.empty());
	for (EdgeMatch match : { EdgeMatch::start, EdgeMatch::end } ) {
		EdgeRun run;
		run.set(edge, opp, match, scaledMax  OP_LINE_FILE_PARGS());
        OP_DEBUG_CODE(debugAdd(run));
		if (OpMath::IsNaN(run.oppDist))
			continue;
		runs.push_back(run);
	}
}

int CcCurves::insertPos(std::vector<EdgeRun>& runSet, EdgeRun& run) {
	int lo = 0;  // position in runs where new run is to be inserted
	int hi = (int) runSet.size() - 1;
	do {
		int mid = (lo + hi) / 2;
        OP_ASSERT(0 <= mid && mid < (int) runSet.size());
		EdgeRun& test = runSet[mid];
		if (test.edgePtT.t == run.edgePtT.t)
			return -mid;
		if (test.edgePtT.t < run.edgePtT.t)
			lo = mid + 1;
		else
			hi = mid - 1;
	} while (lo <= hi);
    return lo;
}

bool CcCurves::keepDiff(const LoHi& diff) const {
	for (const OpEdge* edge : c) {
		if (diff.lo < edge->endT && diff.hi > edge->startT)
			return true;
	}
	return false;		
}

// returns true if caller should split edges down the middle
bool CcCurves::lopSided(size_t priorCount, float maxBias) const {
	if (c.size() < 2 + priorCount) 
		return true;
	float major = 0;
	bool doubleMajor = false;
	for (OpEdge* test : c) {
		float range = test->endT - test->startT;
		if (major < range) {
			major = range;
			doubleMajor = false;
		} else if (major == range)
			doubleMajor = true;
	}
	if (doubleMajor)
		return false;
	float minor = 0;
	for (OpEdge* test : c) {
		float range = test->endT - test->startT;
		if (minor < range && range < major)
			minor = range;
	}
	return minor < maxBias * major;
}

void CcCurves::markToDelete(float loT, float hiT) {
	for (auto edgePtr : c) {
		if (edgePtr->startT >= loT && hiT >= edgePtr->endT)
			edgePtr->ccOverlaps = false;
	}
}

int CcCurves::overlaps() const {
	int count = 0;
	for (auto curve : c)
		count += curve->ccOverlaps;
	return count;
}

void CcCurves::shareDistance() {
    auto copy = [](float oppDist, EdgeDist& dist) {
        if (dist.isSet())
            return;
        OP_ASSERT(OpMath::IsNaN(dist.dist));  // existing distance should not be overwritten
        dist.dist = -oppDist;
    };
    auto share = [copy, this](EdgeDist& dist) {
        for (OpEdge* oppEdge : oppCurves->c) {
            if (oppEdge->startT > dist.opp.t || oppEdge->endT < dist.opp.t)
                continue;
            if (oppEdge->ccStart)
                copy(dist.dist, oppEdge->startDist);
            if (oppEdge->ccEnd)
                copy(dist.dist, oppEdge->endDist);
            return;
       }
    };
    for (OpEdge* edge : c) {
        if (edge->ccStart && edge->startDist.isSet())
            share(edge->startDist);
        if (edge->ccEnd && edge->endDist.isSet())
            share(edge->endDist);
    }
}

// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void CcCurves::snipAndGo(const CutRangeT& tRange) {
	// snip distance must be large enough to differ in x/y and in t
	OP_ASSERT(tRange.lo.t < tRange.hi.t);
	// remove part or all of edges that overlap tRange
	deleted.push_back(tRange);
	const OpPtT& lo = tRange.lo;
	const OpPtT& hi = tRange.hi;
	// after snipping, don't push distance to zero if really small
	CcCurves snippedEdges;
	auto addSnipEdge = [this](const OpEdge* edge, const OpPtT& start, const OpPtT& end
            OP_LINE_FILE_ARGS()) {
        OpEdge* newEdge = cc->allocateEdge(nullptr, edge, start, end, NewEdge::none, 
                EdgeOverlaps::overlaps   OP_LINE_FILE_PARAMS(edge->id));
		return newEdge;
	};
	for (OpEdge* edge : c) {
		if (edge->startT >= hi.t || edge->endT <= lo.t) {
			OP_ASSERT(!edge->disabled);
			snippedEdges.c.push_back(edge);
			continue;
		}
		OpVector threshold = cc->context->threshold;
		if (edge->startT < lo.t && !edge->startPtT().isNearly(lo, threshold)) {
			OpEdge* snipE = addSnipEdge(edge, edge->startPtT(), lo  OP_LINE_FILE_PARGS());
			snipE->ccStart = edge->ccStart;
			snipE->ccSmall = edge->ccSmall;
			snipE->ccEnd = true;
			addEdgeRun(snipE, EdgeMatch::end, ClampDist::no  OP_LINE_FILE_PARGS());
			OP_ASSERT(!snipE->disabled);
			snippedEdges.c.push_back(snipE);
		}
		if (edge->endT > hi.t && !hi.isNearly(edge->endPtT(), threshold)) {
			OpEdge* snipS = addSnipEdge(edge, hi, edge->endPtT()  OP_LINE_FILE_PARGS());
			snipS->ccStart = true;
			snipS->ccEnd = edge->ccEnd;
			snipS->ccLarge = edge->ccLarge;
			addEdgeRun(snipS, EdgeMatch::start, ClampDist::no  OP_LINE_FILE_PARGS());
			OP_ASSERT(!snipS->disabled);
			snippedEdges.c.push_back(snipS);
		}
	}
    OP_DEBUG_CODE(snippedEdges.debugAdd(*this));
	snippedEdges.c.swap(c);
}

// !!! I wonder ... should this be a binary search of sorted entries?
// returns edge with one or more hull intersections on each side of center covering lower .. upper
// also checks if existing curve midpoint switches distance sign
OpEdge* CcCurves::twoHulls(OpPtT& lower, OpPtT& upper) {
    for (OpEdge* cEdge : c) {
        if (cEdge->startT < lower.t || upper.t < cEdge->endT) 
            continue;
        int lowerSect = 0;
        int upperSect = 0;
        for (HullSect& hull : cEdge->hulls.h) {
            lowerSect += hull.sect.t < cEdge->center.t;
            upperSect += hull.sect.t > cEdge->center.t;
        }
        if (lowerSect && upperSect)
            return cEdge;
        if (!checkMidEdge(cEdge))
            return cEdge;
    }
    return nullptr;
}

OpCurveCurve::OpCurveCurve(OpSegment* s, OpSegment* o, std::vector<OpIntersection*>& matchingSects, 
		ForCurveLineSect )
	: context(s->contour->context)
	, seg(s)
	, opp(o)
	, limits(this)
    , endMatches(matchingSects.size()) {
    limits.setEnds(matchingSects);
}

OpCurveCurve::OpCurveCurve(OpSegment* s, OpSegment* o, std::vector<OpIntersection*>& matchingSects)
	: context(s->contour->context)
	, seg(s)
	, opp(o)
	, limits(this)
    , endMatches(matchingSects.size())
	, depth(0)
	, unsplitables(0)
	, boundedEdgeFailed(false)
	, overlap(false)
	, rotateFailed(false)
	, sectResult(false)
	, lastDepthReduced(false)
	, foundGap(false)
	, splitHullFail(false)
{
#if OP_DEBUG_DUMP
	++dumpCurveCurve.nthCall;
	context->debugCurveCurve = this;
#endif
//	contours->reuse(contours->ccStorage);  // !!! consider adding starting block of edges to context
	PathOpsV0Lib::ContextCallbacks& cb = context->contextCallbacks;
    OpVector threshold = context->threshold;
	maxAngleMatch = cb.maxAngleMatchFuncPtr ? cb.maxAngleMatchFuncPtr(s->c.c, o->c.c) : 0.01f;  // !!! degrees
	maxAngleSweep = cb.maxAngleSweepFuncPtr ? cb.maxAngleSweepFuncPtr(s->c.c, o->c.c) : 0.1f;  // !!! degrees
    maxSplit = threshold * (cb.maxSplitFuncPtr ? cb.maxSplitFuncPtr(s->c.c, o->c.c) : 1.f);
    maxBoundedEdge = threshold 
        * (cb.maxBoundedEdgeFuncPtr ? cb.maxBoundedEdgeFuncPtr(s->c.c, o->c.c) : 2.0f);
    maxUnsectable = threshold * (cb.maxUnsectableFuncPtr ? cb.maxUnsectableFuncPtr(s->c.c, o->c.c) : 6.f);
	maxSignSwap = cb.maxSignSwapFuncPtr ? cb.maxSignSwapFuncPtr(s->c.c, o->c.c) : 131072.f;
	maxSignSwap *= context->thresholdLength;
	maxEdgeTSlop = OpEpsilon * (cb.maxTSlopFuncPtr ? cb.maxTSlopFuncPtr(s->c.c, o->c.c) : 4.0f);
	maxSplitBias = cb.maxSplitBiasFuncPtr ? cb.maxSplitBiasFuncPtr(s->c.c, o->c.c) : .1f;  // guess
    maxDist = cb.maxDistFuncPtr ? cb.maxDistFuncPtr(s->c.c, o->c.c) : 4.0f;
//	float maxOverlap = cb.maxOverlapFuncPtr ? cb.maxOverlapFuncPtr(s->c.c, o->c.c) : 1.5f;
	maxCheckSplit = cb.maxCheckSplitFuncPtr ? cb.maxCheckSplitFuncPtr(s->c.c, o->c.c) : 16;
	maxDeep = cb.maxDeepFuncPtr ? cb.maxDeepFuncPtr(s->c.c, o->c.c) : 64;
	maxShallow = cb.maxShallowFuncPtr ? cb.maxShallowFuncPtr(s->c.c, o->c.c) : 8;
	maxSplits = cb.maxSplitsFuncPtr ? cb.maxSplitsFuncPtr(s->c.c, o->c.c) : 8;
//	maxBoundedT = cb.maxBoundedTFuncPtr ? cb.maxBoundedTFuncPtr(s->c.c, o->c.c) : 8; // !!! was 8.f; cubic143299; cubic867777
    limits.setEnds(matchingSects);
	seg->edges.clear();
	opp->edges.clear();

	// !!! undone: if ends of segments already touch, exclude from made edge
	// take the bounds of both segments
	// limit the edge t range to the opposite bounding rectangle
	OpPointBounds curveSectBounds = seg->c.callerBounds().intersect(opp->c.callerBounds());
	if (curveSectBounds.isEmpty())
		return;
	if (curveSectBounds.widthHeight().axisAligned())
		return;
	OP_ASSERT(curveSectBounds.width() && curveSectBounds.height());
#if 0  // enable to debug errors in bounded edge
	OP_DEBUG_CODE(OpDebugData& debugData = context->debugData);
	OP_DEBUG_CODE(bool dbg1 = debugData.curveCurve1 == seg->id || debugData.curveCurve1 == opp->id);
	OP_DEBUG_CODE(bool dbg2 = debugData.curveCurve2 == seg->id || debugData.curveCurve2 == opp->id);
	OP_ASSERT(!debugData.runOneFile || !dbg1 || !dbg2 || -1 == debugData.curveCurveDepth);
#endif
	OpPtT segSingleton(SetToNaN::dummy);
	parentEdge = boundedEdge(seg, curveSectBounds, &segSingleton  OP_LINE_FILE_PARGS());
	if (parentEdge) {
		parentEdge->ccStart = parentEdge->ccSmall = limits.smSegT;
		parentEdge->ccEnd = parentEdge->ccLarge = limits.lgSegT;
		limits.setEdge(parentEdge);
		OP_ASSERT(!parentEdge->disabled);
		edgeCurves.c.push_back(parentEdge);
	} else if (OpMath::IsNaN(segSingleton.t))
		return;
	// end of seg; start of opp
	OpPtT oppSingleton(SetToNaN::dummy);
	parentOpp = boundedEdge(opp, curveSectBounds, &oppSingleton  OP_LINE_FILE_PARGS());
	if (parentOpp) {
		parentOpp->ccStart = parentOpp->ccSmall = limits.smOppT;
		parentOpp->ccEnd = parentOpp->ccLarge = limits.lgOppT;
		limits.setOpp(parentOpp);
		OP_ASSERT(!parentOpp->disabled);
		oppCurves.c.push_back(parentOpp);
	} else if (OpMath::IsNaN(oppSingleton.t))
		return;
	if (!parentEdge || !parentOpp) {
		if (!parentEdge) {
			if (parentOpp && !parentOpp->bounds().contains(segSingleton.pt, threshold))
				return;
			if (seg->sects.contains(segSingleton, opp))
				return;
		}
		if (!parentOpp) {
			if (parentEdge && !parentEdge->bounds().contains(oppSingleton.pt, threshold))
				return;
			if (opp->sects.contains(oppSingleton, seg))
				return;
		}
        OpPtT original = segSingleton;
		if (OpMath::IsNaN(segSingleton.t))
			segSingleton = { oppSingleton.pt, parentEdge->curve.match(parentEdge->startT,
					parentEdge->endT, oppSingleton.pt) };
		if (OpMath::IsNaN(oppSingleton.t))
			oppSingleton = { original.pt, parentOpp->curve.match(parentOpp->startT,
					parentOpp->endT, original.pt) };
		if (!segSingleton.isFinite() || !oppSingleton.isFinite())
			return;
        if (!segSingleton.pt.isNearly(oppSingleton.pt, context->threshold))
            return;
		// add end matches may have already added this
		if (seg->sects.contains(segSingleton, opp))
			return;
		if (opp->sects.contains(oppSingleton, seg))
			return;
		OpIntersection* sect = seg->addSegBase(segSingleton  OP_LINE_FILE_PARAMS(opp));
		OpIntersection* oSect = opp->addSegBase(oppSingleton  OP_LINE_FILE_PARAMS(seg));
		sectPair(sect, oSect, sect->ptT.pt);
		return;
	}
	overlap = true;
    edgeCurves.init(this, &oppCurves, maxDist * context->thresholdLength, parentEdge, opp);
    oppCurves.init(this, &edgeCurves, maxDist * context->thresholdLength, parentOpp, seg);
}

EdgeRun* OpCurveCurve::addEdgeRun(OpEdge* edge, CurveRef curveRef, EdgeMatch match  OP_LINE_FILE_ARGS()) {
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	return curves.addEdgeRun(edge, match, ClampDist::yes  OP_LINE_FILE_CARGS());
}

void OpCurveCurve::addIntersection(OpEdge* edge, OpEdge* oppEdge) {
	recordSect(edge, oppEdge, edge->startPtT(), oppEdge->startPtT()   OP_LINE_FILE_PARGS());
	limits.addSnip(edge->startPtT(), oppEdge->startPtT());
}

void OpCurveCurve::sectPair(OpIntersection* sect, OpIntersection* oSect, OpPoint limitPt) {
	if (sect->segment != seg)
		std::swap(sect, oSect);
	sect->pair(oSect);
#if 0  // defer aliases until all intersections are found
	OpPoint originalSeg = seg->c.ptAtT(sect->ptT.t);
	OpPoint originalOpp = opp->c.ptAtT(oSect->ptT.t);
	if (originalSeg != originalOpp || originalSeg != limitPt)
		seg->contour->addAlias(originalSeg, originalOpp, limitPt, AliasType::curveCurve);
#endif
}

bool OpCurveCurve::addUnsectable(FoundLimit& limit, FoundLimit& limitEnd) {
	OpPtT eStart = limit.segPtT;
	OpPtT eEnd = limitEnd.segPtT;
	OpPtT oStart = limit.oppPtT;
	OpPtT oEnd = limitEnd.oppPtT;
	if (eStart.isNearly(oStart, maxUnsectable))
		OpPtT::MeetInTheMiddle(eStart, oStart);
	if (eEnd.isNearly(oEnd, maxUnsectable))
		OpPtT::MeetInTheMiddle(eEnd, oEnd);
	if (eStart.isNearly(eEnd, maxUnsectable))
		return false;
	if (oStart.isNearly(oEnd, maxUnsectable))
		return false;
	MatchReverse match { MatchEnds::start, oStart.t > oEnd.t };
	IsCoin isCoin = eStart.pt == (match.reversed ? oEnd.pt : oStart.pt) 
			&& eEnd.pt == (match.reversed ? oStart.pt : oEnd.pt) ? IsCoin::yes : IsCoin::no;
	auto setSectPair = [isCoin, this](const OpPtT& ePt, const OpPtT& oPt) {
		SectDuo result;
		result.s = seg->sects.contains(ePt, opp);
		result.o = opp->sects.contains(oPt, seg);
		if (!result.s != !result.o) {
			if (result.o)
				result.s = result.o->opp;
			else
				result.o = result.s->opp;
		}
		result.alreadySet = result.s 
				&& (IsCoin::yes == isCoin ? result.s->coincidenceID : result.s->unsectID);
		return result;
	};
	SectDuo sect1 = setSectPair(eStart, oStart);
	if (sect1.alreadySet)
		return false;
	SectDuo sect2 = setSectPair(eEnd, oEnd);
	if (sect2.alreadySet)
		return false;
	if (sect1.s && sect1.s == sect2.s)
		return false;
	if (sect1.o && sect1.o == sect2.o)
		return false;
	int usectID = seg->nextID();
	auto idEnds = [usectID, &match, isCoin](CoinOpp isOpp) {
		IdEnds idEnds {
			match.reversed && (CoinOpp::yes == isOpp || IsCoin::yes == isCoin) ? -usectID : usectID,
			CoinOpp::yes == isOpp && match.reversed ? !match.match : match.match };
		return idEnds;
	};
	auto setSect = [isCoin, idEnds](OpIntersection* sect, CoinOpp isOpp) {
		IdEnds ie = idEnds(isOpp);
		if (IsCoin::yes == isCoin)
			sect->setCoin(ie.id, ie.matchEnds, isOpp);
		else {
			sect->setUnsect(ie.id, ie.matchEnds);
			sect->ccUnsectable = true;
		}
	};
	auto addSect = [isCoin, idEnds](OpSegment* segs, OpSegment* opps, const OpPtT& start, 
			CoinOpp isOpp  OP_LINE_FILE_ARGS()) {
		IdEnds ie = idEnds(isOpp);
		OpIntersection* result;
		if (IsCoin::yes == isCoin)
			result = segs->addCoin(start, ie.id, ie.matchEnds, isOpp, opps  OP_LINE_FILE_CARGS());
		else {
			result = segs->addUnsectable(start, ie.id, ie.matchEnds, opps  OP_LINE_FILE_CARGS());
			result->ccUnsectable = true;
		}
		result->ccSect = true;
		segs->sects.hasCCSects = true;
		return result;
	};
	auto addPair = [this, addSect, setSect](SectDuo& sPair, const OpPtT& ePtT, const OpPtT& oPtT) {
		if (sPair.s) {
			setSect(sPair.s, CoinOpp::no);
			setSect(sPair.o, CoinOpp::yes);
		} else {
			sPair.s = addSect(seg, opp, ePtT, CoinOpp::no  OP_LINE_FILE_PARGS());
			sPair.o = addSect(opp, seg, oPtT, CoinOpp::yes  OP_LINE_FILE_PARGS());
			sectPair(sPair.s, sPair.o, ePtT.pt);
		}
	};
	addPair(sect1, eStart, oStart);
	if (LimitLine::yes == limit.edgeLine)
		sect1.s->ccLine = true;
	if (LimitLine::yes == limit.oppLine)
		sect1.o->ccLine = true;
	match.match = MatchEnds::end;
	addPair(sect2, eEnd, oEnd);
	if (LimitLine::yes == limitEnd.edgeLine)
		sect2.s->ccLine = true;
	if (LimitLine::yes == limitEnd.oppLine)
		sect2.o->ccLine = true;
	return true; 
}

OpEdge* OpCurveCurve::allocateEdge(OpSegment* segment, const OpEdge* edge, const OpPtT& start,
        const OpPtT& end, NewEdge newEdge, EdgeOverlaps overlaps  
        OP_LINE_FILE_DEF(int parentID)) {
	void* block = context->allocateEdge(context->ccStorage  OP_DEBUG_PARAMS("ccStorage"));
	OpEdge* newE = nullptr;
    if (NewEdge::none != newEdge) {
        OP_ASSERT(!segment);
        OP_ASSERT(end.debugIsUninitialized());
        newE = new(block) OpEdge(edge, start, newEdge  OP_LINE_FILE_CARGS());
    } else if (!segment)
        newE = new(block) OpEdge(edge, start, end  OP_LINE_FILE_CARGS());
    else {
        OP_ASSERT(!edge);
        newE = new(block) OpEdge(segment, start, end  OP_LINE_FILE_CARGS());
    }
	newE->ccOverlaps = !newE->disabled && EdgeOverlaps::overlaps == overlaps;
	OP_DEBUG_CODE(newE->debugParentID = parentID);
    OP_DEBUG_CODE(newE->debugDepth = depth);
    return newE;
}

#if 0
enum class RectSide {
	left,
	top,
	right,
	bottom
};
#endif

// limit the t range of the edge made from seg m to the intersection of seg and opp's bounds
OpEdge* OpCurveCurve::boundedEdge(OpSegment* segm, const OpPointBounds& sectBounds,
		OpPtT* singleton  OP_LINE_FILE_ARGS()) {
	// while segment crosses at most two sect bounds' sides, all four must be checked
	OpCurve& c = segm->c;
	// This could use points at found t; but error in finding axis roots can cause points to miss 
	// bounds. Instead, compute t range, and at the end, outset it to account for error; then
	// compute the points from the liberal t values
	OpPtT minT(SetToNaN::dummy);
	OpPtT maxT(SetToNaN::dummy);
	OpVector threshold = segm->threshold();
	PathOpsV0Lib::ContextCallbacks& cb = segm->contour->context->contextCallbacks;
	float margin = cb.maxMarginFuncPtr ? (*cb.maxMarginFuncPtr)(segm->c.c) : 2.0f;
	int safetyLimit = cb.rootAdjustFuncPtr ? (*cb.rootAdjustFuncPtr)(segm->c.c) : 20;
	threshold *= margin;
	if (sectBounds.contains(segm->c.firstPt(), threshold))
		minT = { segm->c.firstPt(), 0 };
	if (sectBounds.contains(segm->c.lastPt(), threshold))
		maxT = { segm->c.lastPt(),  1 };
	auto saveBest = [sectBounds, threshold, &minT, &maxT](OpPtT ptT, Axis axis) {
		// if old and new points are nearly on or outside the bounds, keep inside point
		// if old point is outside bounds, and new point is nearly inside, keep new
		OP_ASSERT(!OpMath::IsNaN(ptT.t));
		bool inBounds = Axis::horizontal == axis  // check opposite axis to see if result is inside
                ? sectBounds.left - threshold.dx <= ptT.pt.x && ptT.pt.x <= sectBounds.right + threshold.dx
                : sectBounds.top - threshold.dy <= ptT.pt.y && ptT.pt.y <= sectBounds.bottom + threshold.dy;
		if (!inBounds) 
			return;
		if (!(minT.t <= ptT.t))	 // allow nan
			minT = ptT;
		if (!(maxT.t >= ptT.t))
			maxT = ptT;
	};
	auto saveRoots = [&c, saveBest, safetyLimit, this](Axis axis, float inside, float boundary) {
		OpRoots roots = c.axisRayHit(axis, boundary);
		if (RootFail::rootIsNaN == roots.fail) {
			boundedEdgeFailed = true;
			return;
		}
		if (!roots.count())
			return;
		for (float root : roots.roots) {
            // computed point axis should equal boundary or err towards outside of bounds
            // if computed point error moves it inside bounds, adjust t outwards
            OpPtT ptT;
            int safetyCount = safetyLimit;
            float adjust = 0;
            do {
			    ptT = c.ptTAtT(root);
                if ((ptT.pt.choice(axis) - boundary) * inside <= 0)
                    break;
                // adjust t so that the computed point moves outside the bounds
                if (!adjust)
                    adjust = c.tangent(root).choice(axis) * inside < 0 ? OpEpsilon : -OpEpsilon;
                else {
                    if (0 > root || root > 1)
                        break;
                    root += adjust;
                    adjust += adjust;
                }
            } while (--safetyCount > 0);
            OP_ASSERT(safetyCount > 0);
			saveBest(ptT, axis);
		}
	};
	if (segm->c.callerBounds().left < sectBounds.left) {
		OP_ASSERT(segm->c.callerBounds().right >= sectBounds.left);
		saveRoots(Axis::vertical, 1, sectBounds.left);
	}
	if (segm->c.callerBounds().top < sectBounds.top) {
		OP_ASSERT(segm->c.callerBounds().bottom >= sectBounds.top);
		saveRoots(Axis::horizontal, 1, sectBounds.top);
	}
	if (segm->c.callerBounds().right > sectBounds.right) {
		OP_ASSERT(segm->c.callerBounds().left <= sectBounds.right);
		saveRoots(Axis::vertical, -1, sectBounds.right);
	}
	if (segm->c.callerBounds().bottom > sectBounds.bottom) {
		OP_ASSERT(segm->c.callerBounds().top <= sectBounds.bottom);
		saveRoots(Axis::horizontal, -1, sectBounds.bottom);
	}
	if (boundedEdgeFailed)
		return nullptr;
#if 0
	//  debugger function determines error required for axisRayHit to 
	//  move from given result to result on the other side of the target ray
	//  note: can't call all the time: if edges do not intersect, assert may be triggered 
	OP_DEBUG_CODE(debugBoundedEdge(segm, sectBounds, minT, "in"));
	OP_DEBUG_CODE(debugBoundedEdge(segm, sectBounds, maxT, "ax"));
#endif
	if (minT.t == maxT.t || minT.pt == maxT.pt) {
		OpPtT::MeetInTheMiddle(minT, maxT);
		*singleton = minT.onEnd() ? minT : maxT;
	}
	if (!(minT.t < maxT.t))  // condition returns null if either is nan
		return nullptr;
	if (minT.pt == maxT.pt)
		return nullptr;
    OpEdge* result = allocateEdge(segm, nullptr, minT, maxT, NewEdge::none, EdgeOverlaps::no
            OP_LINE_FILE_PARAMS(segm->id));
	result->ccOverlaps = false;
	return result;
}

// if after breaking runs spacially on both edge and opp into two runs
//  and one run is connected to already found intersections, remove that run
// return true if edges connected to small and large t are marked for removal (not overlapping)
bool OpCurveCurve::checkForGaps() {
	if (edgeCurves.c.empty())
		return false;
    if (limits.empty())
        return false;
    if ((!edgeCurves.c.front()->ccOverlaps || 0 != edgeCurves.c.front()->startT) 
			&& (!edgeCurves.c.back()->ccOverlaps || 1 != edgeCurves.c.back()->endT))
		return false;
	OP_ASSERT(edgeCurves.c.size() && oppCurves.c.size());
	std::vector<CutRangeT> edgeGaps = edgeCurves.findGaps();
	if (edgeGaps.size() < limits.size())
		return false;
	std::vector<CutRangeT> oppGaps = oppCurves.findGaps();
	if (oppGaps.size() < limits.size())
		return false;
    for (FoundLimit& limit : limits.i) {
	    if (0 == limit.segPtT.t) {
		    OpPointBounds eGapBounds { edgeGaps[0].lo.pt, edgeGaps[0].hi.pt };
		    size_t oIndex = reversed ? oppGaps.size() - 1 : 0;
		    OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		    if (eGapBounds.overlaps(oGapBounds))
			    edgeCurves.markToDelete(0, edgeGaps[0].lo.t);
		    if (reversed)
			    oppCurves.markToDelete(oppGaps[oIndex].hi.t, 1);
		    else
			    oppCurves.markToDelete(0, oppGaps[0].lo.t);
	    }
	    if (1 == limit.segPtT.t) {
		    OpPointBounds eGapBounds { edgeGaps.back().lo.pt, edgeGaps.back().hi.pt };
		    size_t oIndex = reversed ? 0 : oppGaps.size() - 1;
		    OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		    if (eGapBounds.overlaps(oGapBounds))
			    edgeCurves.markToDelete(edgeGaps.back().hi.t, 1);
		    if (reversed)
			    oppCurves.markToDelete(0, oppGaps[0].lo.t);
		    else
			    oppCurves.markToDelete(oppGaps[oIndex].hi.t, 1);
	    }
    }
	return true;
}

bool OpCurveCurve::checkSect() {
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		OpPtT edgeStart = edge.startPtT();
		OpPtT edgeEnd = edge.endPtT();
		OpVector threshold = context->threshold;
		bool edgeDone = edgeStart.isNearly(edgeEnd, threshold);
		for (auto oppPtr : oppCurves.c) {
			auto& oppEdge = *oppPtr;
			if (!oppEdge.ccOverlaps)
				continue;
			OpPtT oppStart = oppEdge.startPtT();
			OpPtT oppEnd = oppEdge.endPtT();
			// check end condition
			if (edgeDone && (oppStart.isNearly(oppEnd, threshold))) {
				addIntersection(edgePtr, oppPtr);
				return true;
			} 
			if (ifExactly(edge, edgeStart, oppEdge, oppStart)
					|| ifExactly(edge, edgeEnd, oppEdge, oppStart)
					|| ifExactly(edge, edgeStart, oppEdge, oppEnd)
					|| ifExactly(edge, edgeEnd, oppEdge, oppEnd))
				return true;
			if (ifNearly(edge, edgeStart, oppEdge, oppStart)
					|| ifNearly(edge, edgeEnd, oppEdge, oppStart)
					|| ifNearly(edge, edgeStart, oppEdge, oppEnd)
					|| ifNearly(edge, edgeEnd, oppEdge, oppEnd))
				return true;
		}
	}
	return false;
}

// Scan through opposite curves and see if check point is inside deleted bounds. If so, use a
// different (but close by if possible) point to split the curve.
bool OpCurveCurve::checkSplit(float loT, float hiT, CurveRef which, OpPtT& checkPtT) const {
	OP_ASSERT(loT <= checkPtT.t && checkPtT.t <= hiT);
	const std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves.c : edgeCurves.c;
	const OpCurve& eCurve = CurveRef::edge == which ? seg->c : opp->c;
	float startingT = checkPtT.t;
	float deltaT = OpEpsilon;
	int attempts = 0;
	OpPtT original = checkPtT;
	do {
		// check edge cases (e.g., checkPtT.pt == oCurve end) first
		for (OpEdge* oCurve : oCurves) {
			if (oCurve->ccOverlaps && oCurve->bounds().contains(checkPtT.pt))
				return original != checkPtT;
		}
		// check for gap between original and edge list, and between edges in edge list
		const OpEdge& oEdge = CurveRef::edge == which ? *parentOpp : *parentEdge;
		OpPtT delLo = oEdge.startPtT();
		auto checkBounds = [checkPtT](const OpPtT& lo, const OpPtT& hi) {
			OpPointBounds delBounds(lo.pt, hi.pt);
			return delBounds.contains(checkPtT.pt);
		};
		for (OpEdge* oCurve : oCurves) {
			OpPtT delHi = oCurve->startPtT();
			OP_ASSERT(delLo.t <= delHi.t);
			if (delLo.t < delHi.t && checkBounds(delLo, delHi))  // there's a gap between edges
				goto tryAgain;
			delLo = oCurve->endPtT();
			if (!oCurve->ccOverlaps && checkBounds(delHi, delLo))  // check edge without overlap
				goto tryAgain;
		}
		if (delLo.t == oEdge.endT || !checkBounds(delLo, oEdge.endPtT()))  // check last list / original
			return original != checkPtT;
	tryAgain:
		checkPtT.t = startingT + deltaT;
		if (loT > checkPtT.t || checkPtT.t > hiT) {
			checkPtT.t = startingT - deltaT;
			if (loT > checkPtT.t || checkPtT.t > hiT) {
				deltaT /= -2;
				continue;
			}
		}
		checkPtT.pt = eCurve.ptAtT(checkPtT.t);
		deltaT *= -2;
	} while (++attempts < maxCheckSplit);
//	OP_ASSERT(0);  // decide what to do when assert fires
	checkPtT = original;
	return false;
}

void OpCurveCurve::checkUnsplitables() {
	for (OpEdge* eCurve : edgeCurves.c) {
		if (!eCurve->isUnsplitable)
			continue;
		for (OpEdge* oCurve : oppCurves.c) {
			if (!oCurve->isUnsplitable)
				continue;
			bool hullsIntersect = false;
			OpPtT ePt = eCurve->center;
			for (const HullSect& hull : eCurve->hulls.h) {
				if (hull.opp == oCurve) {
					ePt = hull.sect;
					hullsIntersect = true;
					break;
				}
			}
			OpPtT oPt = oCurve->center;
			for (const HullSect& hull : oCurve->hulls.h) {
				if (hull.opp == eCurve) {
					oPt = hull.sect;
					hullsIntersect = true;
					break;
				}
			}
			if (hullsIntersect) {
				recordSect(eCurve, oCurve, ePt, oPt  OP_LINE_FILE_PARGS());
				eCurve->ccOverlaps = false;
				oCurve->ccOverlaps = false;
				return;
			}
		}
	}
}

// Divide only by geometric midpoint. Midpoint is determined by endpoint intersection, or
// curve bounds if no intersection is found.
// try several approaches:
// 1) call existing code for baseline (bounds split on geometric middle, find curve root with line)
// 2) rotate curve to opp start/end line and use newton method to find intersection
//    if we're going to rotate, take advantage and do another bounds check
// 3) if line/line intersects, divide at that point
// 4) if line/line intersects, divide at corresponding ts
// 5) split geometric until t doesn't change (expect to be slow)
// 6) binary search on t (expect to be slower)
SectFound OpCurveCurve::divideAndConquer() {
	OP_ASSERT(1 == edgeCurves.c.size());
	OP_ASSERT(1 == oppCurves.c.size());
	// !!! testQuads5721199 segments 2 and 5 share (0, 0) but iterate depth to 24 to see if they
	// intersect a second time. Not sure what to do...
	for (depth = 1; depth < maxDeep; ++depth) {
#if OP_DEBUG_DUMP
		dumpCurveCurve.cc.push_back(*this);
        for (auto& edges : { edgeCurves.c, oppCurves.c } ) {
            for (auto& edge : edges) {
                edge->debugCC = depth;
            }
        }
#endif
#if OP_DEBUG && !OP_DEBUGGER && !OP_DEBUG_FAST_TEST
		OP_ASSERT(debugBreak(CcBreak::atDepth));
#endif
		bool snipEm = 1 == depth && !limits.snips.empty();
		if (!setOverlaps())
			return SectFound::fail;
		if (checkForGaps() || (splitMid && !endsOverlap()))
			splitMid = false;
		int edgeOverlaps = edgeCurves.overlaps();
		int oppOverlaps = oppCurves.overlaps();
		if (!edgeOverlaps || !oppOverlaps) {
			if (depth > maxShallow)  // default may not be correct (depth is 9 for testQuads3993265)
				return SectFound::noOverlapDeep;
			return limits.size() ? SectFound::add : SectFound::no;
		}
		if (edgeOverlaps >= maxSplits || oppOverlaps >= maxSplits) {
			// find crossing for every curve that has an opposite distance that changes sign
			edgeCurves.checkSigns();
			oppCurves.checkSigns();
			return SectFound::maxOverlaps;
		}
		if (checkSect())
			snipEm = true;
		else if (!snipEm) {
			size_t limitCount = limits.size();
			bool foundLimit = setHulls(CurveRef::edge);
            if (!foundLimit)
			    setHulls(CurveRef::opp);
			snipEm = limits.addSnipRange(limitCount);
		}
#if OP_DEBUG && OP_DEBUG_VERBOSE  // save state prior to split and delete
		debugSaveState();
#endif
		// if there is more than one crossover, look for unsectable
		limits.setUnique();
		if (depth > 2 && limits.unique > endMatches && !lastDepthReduced && !snipEm) {
			if (!reduceDistFlipped())
				return SectFound::add;
			lastDepthReduced = true;
			continue;
		}
		// If any intersection is found, already found or not, remove piece around 
		// both edge and opp so that remaining edges can be checked for intersection
		if (!snipEm) {
			CcCurves eSplits, oSplits;
			size_t limitCount = limits.size();
			unsplitables = 0;
			if (!splitHulls(CurveRef::edge, eSplits)) {
                OP_DEBUG_CODE(edgeCurves.debugAdd(eSplits));
				return SectFound::fail;
            }
			if (!splitHulls(CurveRef::opp, oSplits)) {
                OP_DEBUG_CODE(oppCurves.debugAdd(oSplits));
				return SectFound::fail;
            }

            OP_DEBUG_CODE(edgeCurves.debugAdd(eSplits));
            OP_DEBUG_CODE(oppCurves.debugAdd(oSplits));
			edgeCurves.c.swap(eSplits.c);
			oppCurves.c.swap(oSplits.c);
			if (!edgeCurves.c.size()) {
				if (splitHullFail)  // split hulls failed to split -- runs crossing axis is sect
					return SectFound::fail;  // note that this is very conservative and narrow
				return limits.size() ? SectFound::add : SectFound::no;
			}
			// if edge curves and opp curves have unsplittables, add limit and remove both
			if (1 < unsplitables)
				checkUnsplitables();
			snipEm = limits.addSnipRange(limitCount);
		}
		for (SnipPtTs snip : limits.snips) {
//			start here;
			// if snip-and-go convext hulls of edge and opp do not intersect, remove
			// limit and increase snip radius until they do; use larger edges to proceed
			edgeCurves.snipAndGo(snip.segCut);
			oppCurves.snipAndGo(snip.oppCut);
            // if one side computed start or end distances, and the other did not, copy distances
            // so that 'reduce dist flipped' can keep snipped edges if needed
            // !!! mistakenly thought needed -- disable until proven
    //        edgeCurves.shareDistance();
    //        oppCurves.shareDistance();
		}
		limits.snips.clear();
	}
	return SectFound::fail;  // soft fail (ignored)
}

// return true if either small t or large t belong to edge that is still available
bool OpCurveCurve::endsOverlap() const {
	if (!edgeCurves.c.size() || !oppCurves.c.size())
		return false;
	const OpEdge* lastB = edgeCurves.c.back();
	if (lastB->ccLarge && lastB->ccOverlaps)
		return true;
	lastB = reversed ? oppCurves.c.front() : oppCurves.c.back();
	if (lastB->ccLarge && lastB->ccOverlaps)
		return true;
	const OpEdge* lastF = edgeCurves.c.front();
	if (lastF->ccSmall && lastF->ccOverlaps)
		return true;
	lastF = reversed ? oppCurves.c.back() : oppCurves.c.front();
	if (lastF->ccSmall && lastF->ccOverlaps)
		return true;
	return false;
}

// (assuming its already been determined that the edge pair is likely partially unsectable
// binary search pair of curves
// look for range of unsectables and snip remaining more aggressively
// if one end t is already found, add it in to calculus of finding range of unsectable
// in any case, check the found range to see if the midpoint is also plausibly unsectable
// before treating the entire range as unsectable
// either: generate one unsectable range, or two points
// (unsectable range + one point or two unsectable ranges may be better, but wait for test case)

// rework this to keep unsectable ranges in curve-curve until all intersections are found, and then
// output intersections and intersection ranges
// start by changing curve-curve constructor to record range for small and large t found, if any
void OpCurveCurve::findUnsectable() {
	// start with existing found point
	// separate limits into unsectable and regular
	// continue as long as unsectable until entire unsectable range is found
	// then, skip regular if they are from found t
	for (size_t index = limits.size(); index-- != 0; ) {
		FoundLimit& limit = limits.i[index];
		if (LimitBettered::yes == limit.bettered)
			limits.i.erase(limits.i.begin() + index);
	}
	OP_ASSERT(!limits.empty() || endMatches);
	// find limit of where curve pair are nearly coincident
	limits.sort();
	limits.markOutOfOrder();
	// check midpoints to see if they are also on the curve
	size_t unsectLo = 0;
	size_t unsectHi = 0;
	auto addSect = [this, &unsectLo, &unsectHi]() {
		FoundLimit& limit = limits.i[unsectLo];
		if (unsectLo < unsectHi && addUnsectable(limit, limits.i[unsectHi])) {
            limit.used = LimitUsed::yes;
            limits.i[unsectHi].used = LimitUsed::yes;
			return;
        }
		if (LimitFrom::yes == limit.fromFoundT)
			return;
		OpPtT::MeetInTheMiddle(limit.segPtT, limit.oppPtT);
		if (seg->sects.contains(limit.segPtT, opp))
			return;
		if (opp->sects.contains(limit.oppPtT, seg))
			return;
        // check if sects added earlier by 'add end matches' form unsectable pair
        // for each matching sect: see if span for both is linear or nearly so
        auto candidateOnLine = [this](OpSegment* segC, OpPtT& limS, CurveRef curveRef) {
            for (FoundLimit& limit : limits.i) {
                if (LimitMatch::no == limit.match)
                    continue;
                OpPtT limitPtT = CurveRef::edge == curveRef ? limit.segPtT : limit.oppPtT;
                LinePts segEnds { limitPtT.pt, limS.pt };
                OpPoint midSPt = segC->c.ptAtT((limitPtT.t + limS.t) / 2);
                return segEnds.ptOnLine(midSPt);
            }
            return false;
        };
        if (candidateOnLine(seg, limit.segPtT, CurveRef::edge)
                && candidateOnLine(opp, limit.oppPtT, CurveRef::opp))
            return;
		OpIntersection* sect = seg->addSegBase(limit.segPtT  OP_LINE_FILE_PARAMS(opp));
		sect->ccLine = LimitLine::yes == limit.edgeLine;
		OpIntersection* oSect = opp->addSegBase(limit.oppPtT  OP_LINE_FILE_PARAMS(seg));
		oSect->ccLine = LimitLine::yes == limit.oppLine;
		sectPair(sect, oSect, limit.segPtT.pt);
	};
	float lastT = limits.i[0].segPtT.t;
	for (size_t index = 1; index < limits.size(); ++index) {
		FoundLimit& limit = limits.i[index];
		if (Unordered::yes == limit.oppOutOfOrder)
			continue;
		float t = limit.segPtT.t;
		OP_ASSERT(t > lastT);
		float midT = OpMath::Average(lastT, t);
		OpPtT midPtT = seg->c.ptTAtT(midT);
		OpPtT oppTest = seg->distance(midPtT, opp);
		OpVector oppdist = midPtT.pt - oppTest.pt;
		if (oppdist.lengthSquared() <= OpEpsilon) // !!! epsilon is unfounded guess
			unsectHi = index;
		else {
			addSect();
			unsectLo = index;
		}
		lastT = t;
	}
	addSect();
}

bool OpCurveCurve::ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& oppEdge, const OpPtT& oppPtT) {
	if (edgePtT.pt != oppPtT.pt)
		return false;
	if (edge.ccStart && edge.startT == edgePtT.t)
		return false;
	if (edge.ccEnd && edge.endT == edgePtT.t)
		return false;
	recordSect(&edge, &oppEdge, edgePtT, oppPtT  OP_LINE_FILE_PARGS());
	limits.addSnip(edgePtT, oppPtT);
	return true;
}

bool OpCurveCurve::ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& oppEdge, const OpPtT& oppPtT) {
	OpVector threshold = context->threshold;
	if (!edgePtT.pt.isNearly(oppPtT.pt, threshold))
		return false;
	if (edge.ccStart && edge.startPtT().isNearly(edgePtT, threshold))
		return false;
	if (edge.ccEnd && edge.endPtT().isNearly(edgePtT, threshold))
		return false;
	recordSect(&edge, &oppEdge, edgePtT, oppPtT  OP_LINE_FILE_PARGS());
	limits.addSnip(edgePtT, oppPtT);
	return true;
}

bool OpCurveCurve::alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, 
		const OpPtT& edgePtT, const OpPtT& oppPtT) {
		if (limits.alreadyIn(edgePtT, oppPtT))
			return true;
#if 0  // this breaks testQuads5635157; instead, disallow t values in deleted ranges
// if this edge/opEdge is a subset of an existing limit, replace it
	for (size_t index = 0; index < limits.size(); ++index) {
		FoundLimits& limit = limits[index];
		if (limit.parentEdge->segment != edge->segment)
			continue;
		OP_ASSERT(limit.parentOpp->segment == oEdge->segment);
		if (limit.parentEdge->startT > edge->startT)
			continue;
		if (limit.parentEdge->endT < edge->endT)
			continue;
		limits.erase(limits.begin() + index);  // caller replaces limit with more precise data
		break;
	}
#else
	CcCurves& curves = edge->segment == seg ? edgeCurves : oppCurves;
	if (curves.deletedT(edgePtT.t))
		return true;
#endif
	return false;  // caller adds to limit
}

bool OpCurveCurve::betweenLimits(OpSegment* segment, float lo, float hi) {
	for (FoundLimit& limit : limits.i) {
// !!! this test may be still required: but, trying to remove edge from edge runs
//		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
//			return true;  // reverse case already recorded
		if (OpMath::Between(lo, 
				LimitSwapped::yes == limit.swapped ? limit.oppPtT.t : limit.segPtT.t, hi))
			return true;  // already recorded
	}
	return false;
}

struct FoundAngles {
// !!! for now, return degrees to make debugging easier. Eventually, could return radians, maybe...
	void angle(OpCurve& c, float t, CurveRef ref) {
		OpVector cTan = c.tangent(t);
		float a = std::atan2f(cTan.dy, cTan.dx) * 180 / OpPI;
		if (OpMath::IsNaN(lesser))
			lesser = a;
		else {
			 greater = a;
			 if (lesser > greater) {
			 	std::swap(lesser, greater);
				ref = !ref;
			 }
		}
		curveRef = ref;
	}

	float lesser = OpNaN;
	float greater = OpNaN;
	CurveRef curveRef;
};

// defer meet in the middle stuff until all intersections are found
void OpCurveCurve::recordSect(OpEdge* edge, OpEdge* oEdge, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_ARGS()) {
	OP_ASSERT(parentEdge->segment == edge->segment);
	OP_ASSERT(parentOpp->segment == oEdge->segment);
    OP_ASSERT(edgePtT.isFinite());
    OP_ASSERT(oppPtT.isFinite());
	if (alreadyInLimits(edge, oEdge, edgePtT, oppPtT))
		return;
	OP_ASSERT(edge->curve.isLineSet);
	OP_ASSERT(oEdge->curve.isLineSet);
	FoundLimit newLimit(edge, oEdge, edgePtT, oppPtT  OP_LINE_FILE_CARGS());
	FoundAngles newAngles;
	for (size_t index = limits.size(); index-- != 0; ) {
		FoundLimit& old = limits.i[index];
		if (!old.parentEdge)
			continue;
		if (OpMath::IsNaN(newAngles.lesser)) {
			newAngles.angle(edge->segment->c, edgePtT.t, CurveRef::edge);
			newAngles.angle(oEdge->segment->c, oppPtT.t, CurveRef::opp);
			OP_ASSERT(!OpMath::IsNaN(newAngles.greater));
			float span = newAngles.greater - newAngles.lesser;
			if (span < maxAngleSweep)
				break;
		}
		OP_ASSERT(old.parentEdge->segment == edge->segment);
		OP_ASSERT(old.parentOpp->segment == oEdge->segment);
		FoundAngles oldAngles;
		oldAngles.angle(old.parentEdge->segment->c, old.segPtT.t, CurveRef::edge);
		oldAngles.angle(old.parentOpp->segment->c, old.oppPtT.t, CurveRef::opp);
		// if angle formed by tangents is larger than some set value, and if angles formed by 
		// old and new are the same within some threshold, keep the one with closer points
		if (oldAngles.curveRef != newAngles.curveRef)
			continue;
		if (std::fabsf(oldAngles.lesser - newAngles.lesser) > maxAngleMatch)
			continue;
		if (std::fabsf(oldAngles.greater - newAngles.greater) > maxAngleMatch)
			continue;
//		limits.i.erase(limits.i.begin() + index);
		old.bettered = LimitBettered::yes;
	}
	limits.i.push_back(std::move(newLimit));
	limits.unique = -1;
}

// remove edges that do not change distance sign in run
// keep opp edges that correspond to edges with signs that flip, even if their distance signs agree
bool OpCurveCurve::reduceDistFlipped() {
	CcCurves edgeSplits, oppSplits;
	CcCurves* curves = &edgeCurves;
	OpSegment* segment = seg;
	OpSegment* oSegment = opp;
	bool swap = false;
    auto addSplit = [this, &segment, &curves](CcCurves& splits, OpPtT& lower, OpPtT& upper) {
        if (lower.isNearly(upper, maxSplit))
            return;
        // return intersection of lower/upper and on intervals in curve
        std::vector<Interval> intervals = curves->continuous(lower, upper);
        if (intervals.empty())
            return;
        for (Interval interval : intervals) {
            OpEdge* split = allocateEdge(segment, nullptr, interval.lo, interval.hi, NewEdge::none, 
                    EdgeOverlaps::overlaps  OP_LINE_FILE_PARAMS(segment->id));
		    split->ccOverlaps = true;
		    OP_ASSERT(!split->disabled);
		    splits.c.push_back(split);
        }
    };
	do {
		EdgeRun* lower = nullptr;
		EdgeRun* upper = nullptr;
		auto keepRun = [addSplit, &lower, &upper, &curves](CcCurves& splits) {
            // if existing edge is equal to or smaller than split, use that instead
            OP_ASSERT(lower->edgePtT.t < upper->edgePtT.t);
            for (OpEdge* cEdge : curves->c) {
                if (cEdge->startT < lower->edgePtT.t || upper->edgePtT.t < cEdge->endT)
                    continue;
                if (!(cEdge->startDist.dist * cEdge->endDist.dist < 0))
                    continue;
                OP_ASSERT(cEdge->startDist.dist * lower->oppDist >= 0);
                OP_ASSERT(cEdge->endDist.dist * upper->oppDist >= 0);
                splits.c.push_back(cEdge);
                return;
            }
            addSplit(splits, lower->edgePtT, upper->edgePtT);
		};
        std::vector<EdgeRun>& runs = curves->runs;
        size_t size = runs.size();
		for (size_t index = 1; index < size; ++index) {
			if (runs[index].oppDist)
                continue;
            if (index > 1 && !(runs[index - 2].oppDist * runs[index - 1].oppDist >= 0))
                continue;
            if (!curves->checkMidRun(index - 1))
                continue;
            runs[index - 1].byZero = true;
        }
        for (size_t index = 0; index + 1 < size; ++index) {
			if (runs[index].oppDist)
                continue;
            if (index + 2 < size && !(runs[index + 2].oppDist * runs[index + 1].oppDist >= 0))
                continue;
            if (!curves->checkMidRun(index))
                continue;
            runs[index + 1].byZero = true;
        }
		// add edges with distances that switch signs but are not adjacent to very small
		CcCurves& splits = swap ? oppSplits : edgeSplits;
        EdgeRun* prior = nullptr;  // order: prior, lower, upper
		for (auto& run : curves->runs) {
			upper = &run;
            if (!lower) {
                lower = upper;
                continue;
            }
            bool switchesSides = lower->oppDist * run.oppDist < 0;
            bool byZero = lower->byZero || run.byZero;
            bool nearlyZero = fabsf(lower->oppDist) <= OpEpsilon || fabsf(run.oppDist) <= OpEpsilon;
            auto splitClose = [&segment, &oSegment, addSplit, &splits](
                    EdgeRun* prior, EdgeRun* lower) {
                OpPtT tempPtT = segment->c.ptTAtT(OpMath::Average(prior->edgePtT.t, lower->edgePtT.t));
                OpPtT oppPtT = segment->distance(tempPtT, oSegment);
                EdgeRun temp;
                temp.set(tempPtT, oppPtT, segment, 0  OP_LINE_FILE_PARGS());
                if (temp.oppDist * lower->oppDist < 0
                        || fabsf(temp.oppDist) < fabsf(prior->oppDist) 
                        || fabsf(temp.oppDist) < fabsf(lower->oppDist)) {
                    addSplit(splits, prior->edgePtT, tempPtT); 
                    addSplit(splits, tempPtT, lower->edgePtT);
               }
            };
			if (switchesSides && !nearlyZero && !byZero)
				keepRun(splits);
            else if (OpEdge* edge = curves->twoHulls(lower->edgePtT, upper->edgePtT)) {
				if (!edge->centerless) {
					OpPtT midPtT = edge->center;
					addSplit(splits, lower->edgePtT, midPtT);
					addSplit(splits, midPtT, upper->edgePtT);
				}
            }  // check for missed sects between smallest magnitude dist and adjacent dist
            else if (prior && !switchesSides && !(prior->oppDist * run.oppDist < 0)
                    && !prior->byZero && !byZero && fabsf(prior->oppDist) > fabsf(lower->oppDist)
                    && fabsf(lower->oppDist) < fabsf(run.oppDist)) {
                if (run.edgePtT.t - lower->edgePtT.t < lower->edgePtT.t - prior->edgePtT.t) {
                    splitClose(prior, lower);
                    addSplit(splits, lower->edgePtT, run.edgePtT);
                } else {
                    addSplit(splits, prior->edgePtT, lower->edgePtT);
                    splitClose(lower, &run);
                }
            }
            prior = lower;
			lower = upper;
		}
		curves = &oppCurves;
		std::swap(segment, oSegment);
	} while ((swap = !swap));
    OP_DEBUG_CODE(edgeCurves.debugAdd(edgeSplits));
    OP_DEBUG_CODE(edgeCurves.debugAdd(oppSplits));
	if (edgeSplits.c.size()) {
		edgeCurves.c = edgeSplits.c;
		oppCurves.c = oppSplits.c;
        edgeCurves.runs.clear();
        oppCurves.runs.clear();
		return true;
	}
	return false;
}

bool OpCurveCurve::rotatedIntersect(OpEdge& edge, OpEdge& oppEdge, bool sharesPoint) {
	LinePts edgePts { edge.startPt(), edge.endPt() };
	const OpCurve& edgeRotated = edge.setVertical(edgePts, MatchEnds::start);
	rotateFailed |= !edgeRotated.isFinite();
	MatchReverse match = oppEdge.matchEnds(edgePts);
	const OpCurve& oppRotated = oppEdge.setVertical(edgePts, match.match);
	rotateFailed |= !oppRotated.isFinite();
	OpPointBounds eRotBounds = edgeRotated.fullBounds();
	OpPointBounds oRotBounds = oppRotated.fullBounds();
	if (rotateFailed || !eRotBounds.intersects(oRotBounds))
		return false;
	// !!! can one have no area (e.g. horz or vert) and the other not?
	return !sharesPoint || !eRotBounds.hasArea() || !oRotBounds.hasArea() 
			|| eRotBounds.areaOverlaps(oRotBounds);
}

SectFound OpCurveCurve::runsToLimits() {
	size_t oldLimits = limits.size();
	bool swap = false;
	auto addIfNew = [this, &swap](EdgeRun* run) {
		OpEdge* parEdge = parentEdge;
		OpEdge* parOpp = parentOpp; 
		FoundLimit lowerLimit(parentEdge, parentOpp, run->edgePtT, run->oppPtT  OP_LINE_FILE_PARGS());
		lowerLimit.fromFoundT = run->fromFoundT; 
		lowerLimit.swapped = swap ? LimitSwapped::yes : LimitSwapped::no;
		if (swap) {
			std::swap(parEdge, parOpp);
			std::swap(lowerLimit.parentEdge, lowerLimit.parentOpp);
			std::swap(lowerLimit.segPtT, lowerLimit.oppPtT);
		}
		if (!alreadyInLimits(parEdge, parOpp, lowerLimit.segPtT, lowerLimit.oppPtT))
			limits.i.push_back(std::move(lowerLimit));
	};
	EdgeRun* lastUpper;
	CcCurves* curves = &edgeCurves;
	CcCurves* oCurves = &oppCurves;
	OpSegment* segment = seg;
	OpSegment* oSegment = opp;
	auto addLimit = [this, &segment, &lastUpper, &curves, &oCurves, addIfNew](
			EdgeRun* lower, EdgeRun* upper) {
//		OP_ASSERT(!upper || lower->runEdge->segment == upper->runEdge->segment);
		if (upper && betweenLimits(segment, lower->oppPtT.t, upper->oppPtT.t))
			return;
		if (!lower->inDeleted(curves, oCurves) && lower != lastUpper 
				&& (!upper || fabsf(lower->oppDist) <= fabsf(upper->oppDist)))
			addIfNew(lower);
		if (upper && !upper->inDeleted(curves, oCurves) 
				&& fabsf(lower->oppDist) > fabsf(upper->oppDist)) {
			addIfNew(upper);
			lastUpper = upper;
		}
	};
	// first pass: add edges with very small distances
	size_t lodex, hidex;
	auto markByZero = [&lodex, &hidex, &curves, addLimit]() {
            OP_ASSERT(0 <= lodex && lodex < curves->runs.size());
			EdgeRun* lower = &curves->runs[lodex];
            OP_ASSERT(OpMax == hidex || (0 <= hidex && hidex < curves->runs.size()));
			EdgeRun* upper = OpMax == hidex ? nullptr : &curves->runs[hidex];
			addLimit(lower, upper);
			// only disable prior if the mid point is equal to or smaller than its distance
			// mark prior two edges to disable sign compare
			if (lodex > 1 && curves->checkMidRun(lodex - 1)) {
				curves->runs[lodex - 1].byZero = true;
			}
			if (OpMax == hidex)
				hidex = lodex;
			// mark next two edges to disable sign compare
			if (hidex + 2 < curves->runs.size() && curves->checkMidRun(hidex))
				curves->runs[hidex + 1].byZero = true;
			lodex = hidex = OpMax;
	};
	// scan both edge curves and opp curves
	do {
		lastUpper = nullptr;
		lodex = OpMax;
		hidex = OpMax;
		for (size_t index = 0; index < curves->runs.size(); ++index) {
			EdgeRun& run = curves->runs[index];
			if (0 == run.oppDist)
				(OpMax == lodex ? lodex : hidex) = index;
			else if (OpMax != lodex)
				markByZero();
		}
		if (OpMax != lodex)
			markByZero();
		// second pass: add edges with distances that switch signs but are not adjacent to very small
		EdgeRun* lower = nullptr;
		for (auto& run : curves->runs) {
			if (lower && lower->oppDist * run.oppDist < 0 
					&& lower->oppDist && run.oppDist
				// !!! very large dist sign swaps should be ignored
				//     need to determine what this range should be
					&& fabsf(lower->oppDist) < maxSignSwap 
					&& fabsf(run.oppDist) < maxSignSwap
					&& !lower->byZero && !run.byZero) { // switches sides
				// check if dot of opposite normal at oppPtT.t switches signs (loop146475)
				OpVector lowerOppNormal = oSegment->c.normal(lower->oppPtT.t).normalize();
				OpVector upperOppNormal = oSegment->c.normal(run.oppPtT.t).normalize();
				OpVector lowerDistV = (lower->oppPtT.pt - lower->edgePtT.pt).normalize();
				OpVector upperDistV = (run.oppPtT.pt - run.edgePtT.pt).normalize();
				//!!! do the vectors need to be normalized first? (probably)
				float lowerX = lowerOppNormal.dot(lowerDistV);
				float upperX = upperOppNormal.dot(upperDistV);
				if (lowerX * upperX < 0)
					addLimit(lower, &run);
			}
			lower = &run;
		}
		curves = &oppCurves;
		std::swap(segment, oSegment);
	} while ((swap = !swap));
	return limits.size() == oldLimits ? SectFound::no : SectFound::add;
}

// finds intersections of opp edge's hull with edge, and stores them in edge's hulls
// returns true if found intersection is true curve curve intersection (or at least, close enough)
bool OpCurveCurve::setHullSects(OpEdge& edge, OpEdge& oppEdge, CurveRef curveRef) {
	int ptCount = oppEdge.curve.pointCount();
	LinePts oppPts;
	oppPts.pts[1] = oppEdge.curve.firstPt();
	OpVector threshold = context->threshold;
	for (int index = 1; index <= ptCount; ++index) {
		oppPts.pts[0] = oppPts.pts[1];
		int endHull = index < ptCount ? index : 0;
		oppPts.pts[1] = oppEdge.curve.hullPt(endHull);
		if (oppPts.pts[0].isNearly(oppPts.pts[1], threshold))
			continue;
		// since curve/curve intersection works by keeping overlapping edge bounds, it should
		// use edge, not segment, to find hull intersections
		OpRoots septs = edge.curve.lineIntersect(oppPts);
		if (RootFail::rootIsNaN == septs.fail)
			continue;
		for (int inner = 0; inner < septs.count(); ++inner) {
			OpPtT sectPtT = edge.curve.ptTAtT(septs.get(inner));
			// if point is not near opposite hull line, discard it
			if (!oppPts.ptNearLine(sectPtT.pt, threshold))
				continue;
			// set to secttype endhull iff computed point is equal to or nearly an end point
			SectType sectType;
			if ((1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(oppPts.pts[0], threshold)) {
				sectType = SectType::endHull;
				sectPtT.pt = oppPts.pts[0];
			} else if ((ptCount - 1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(oppPts.pts[1], threshold)) {
				sectType = SectType::endHull;
				sectPtT.pt = oppPts.pts[1];
			} else
				sectType = endHull ? SectType::controlHull : SectType::midHull;
			sectPtT.t = OpMath::Interp(edge.startT, edge.endT, sectPtT.t);
			OP_ASSERT(edge.startT <= sectPtT.t && sectPtT.t <= edge.endT);
			// if pt is close to existing hull sect, and both are not end, record intersection
			if (edge.hulls.add(sectPtT, context->threshold, EdgeDist(SetToNaN::dummy), sectType, 
                    &oppEdge)) {
#if 1
	// always use original segment to find points
				OpSegment* oSeg = oppEdge.segment;
				OpPtT oppPtT { oSeg->c.ptTAtT(oSeg->c.findValidT(0, 1, sectPtT.pt)) };
#else
				OpPtT oppPtT { oppEdge.curve.ptTAtT(oppEdge.curve.findValidT(0, 1, sectPtT.pt))};
				oppPtT.t = OpMath::Interp(oppEdge.startT, oppEdge.endT, oppPtT.t);
#endif
				if (!oppPtT.pt.isFinite())
					return false;
				// if computed points are not close, use linear intersection to try again
				OpVector diff = sectPtT.pt - oppPtT.pt;
				OpVector eTan = edge.curve.tangent(sectPtT.t);
				OpVector oTan = oppEdge.curve.tangent(oppPtT.t);
				LinePts eLinePts { sectPtT.pt, sectPtT.pt + eTan };
				LinePts oLinePts { oppPtT.pt, oppPtT.pt + oTan };
				auto tryIt = [this](const LinePts& eLinePts, const LinePts& oLinePts,
						OpCurve& eCurve) {
                    PathOpsV0Lib::CurveType lineType = eCurve.lineType();
					PathOpsV0Lib::Curve eLineCurve { (ContextPtr) context, 
                            (PathOpsV0Lib::CurveData*) &eLinePts, sizeof eLinePts, lineType };
					OpCurve eLine(eLineCurve, Rotated::no);
					OpCurve eRotLine = eLine.toVertical(oLinePts, MatchEnds::none);
					OpRoots eLineT = eRotLine.axisRawHit(Axis::vertical, 0, MatchEnds::none);
					if (eLineT.empty())
						return OpPtT(SetToNaN::dummy);
					if (eLineT.count() > 1)
						return OpPtT(SetToNaN::dummy);
					// don't call pt at t ; need answer to be unpinned
				//	OpPoint eLinePt = eLine.ptAtT(eLineT.get(0));
					float eT = eLineT.get(0);
                    if (!OpMath::IsFinite(eT))
						return OpPtT(SetToNaN::dummy);
					OpPoint eLinePt = (1 - eT) * eLinePts.pts[0] + eT * eLinePts.pts[1];
					float validT = eCurve.findValidT(0, 1, eLinePt);
					if (!OpMath::IsNaN(validT))
						return OpPtT(eLinePt, validT);
					return OpPtT(SetToNaN::dummy);
				};
				OpPtT eTry = tryIt(eLinePts, oLinePts, edge.curve);
				OpPtT oTry = tryIt(oLinePts, eLinePts, oppEdge.curve);
				if (eTry.isFinite() && oTry.isFinite()) {
					float tryLength = (eTry.pt - oTry.pt).length();
					if (tryLength < diff.length()) {
						sectPtT = OpPtT(eTry.pt, OpMath::Interp(edge.startT, edge.endT, eTry.t));
						oppPtT = OpPtT(oTry.pt, OpMath::Interp(oppEdge.startT, oppEdge.endT, oTry.t));
					}
				}
				if (CurveRef::edge == curveRef)
					recordSect(&edge, &oppEdge, sectPtT, oppPtT  OP_LINE_FILE_PARGS());
				else
					recordSect(&oppEdge, &edge, oppPtT, sectPtT  OP_LINE_FILE_PARGS());
				return true;
			}
		}
	}
    return false;
}

// its possible that slight error means that while edges intersect, edge/segment does not
// allow either
bool OpCurveCurve::addLineCurveIntersection(OpEdge& edge, OpEdge& oppEdge, CurveRef curveRef) {
    OpSegment* segment = CurveRef::edge == curveRef ? seg : opp;
    OpSegment* opposite = CurveRef::edge == curveRef ? opp : seg;
    OP_ASSERT(edge.curve.debugIsLine());
    auto checkIntersection = [&edge, &oppEdge, segment, opposite, curveRef, this](OpRoots& oppRoots) {
        for (float oppT : oppRoots.roots) {
            if (!OpMath::InSorted(oppEdge.startT, oppT, oppEdge.endT, OpEpsilon))  // !!! replace ep with max..
                continue;
            OpPtT oppPtT = opposite->c.ptTAtT(oppT);
            if (!edge.bounds().nearlyContains(oppPtT.pt, maxBoundedEdge))
                continue;
            float edgeT = segment->c.findValidT(std::max(0.f, edge.startT - maxEdgeTSlop), 
                    std::min(1.f, edge.endT + maxEdgeTSlop), oppPtT.pt);
            if (OpMath::IsNaN(edgeT))
                continue;
            OpPtT edgePtT = segment->c.ptTAtT(edgeT);
            if (CurveRef::edge == curveRef)  // !!! ? already swapped
                recordSect(&edge, &oppEdge, edgePtT, oppPtT  OP_LINE_FILE_PARGS());
            else
                recordSect(&oppEdge, &edge, oppPtT, edgePtT  OP_LINE_FILE_PARGS());
            return true;
        }
        return false;
    };
#if OP_DEBUG_DUMP
	debugDumpOn = false; // true;
#endif
    OpRoots oppRoots = edge.curve.lineIntersection(opposite->c);
#if OP_DEBUG_DUMP
	debugDumpOn = false;
#endif
    if (checkIntersection(oppRoots))
        return true;
    // if line and segment didn't intersect, check line and edge
    OpRoots oRoots = edge.curve.lineIntersection(oppEdge.curve);
    // normalize roots !!! do I need a common routine for this?
    for (int index = 0; index < oRoots.count(); ++index) {
        oRoots.roots[index] = OpMath::Interp(oppEdge.startT, oppEdge.endT, oRoots.roots[index]);
    }
    if (checkIntersection(oRoots))
        return true;
    return false;
}

// if ref is edge, records all intersections of edges with all hulls of opp
bool OpCurveCurve::setHulls(CurveRef curveRef) {
	std::vector<OpEdge*>& eCurves = CurveRef::edge == curveRef ? edgeCurves.c : oppCurves.c;
	std::vector<OpEdge*>& oCurves = CurveRef::edge == curveRef ? oppCurves.c : edgeCurves.c;
    bool addedSect = false;
	for (auto edgePtr : eCurves) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		for (auto oppPtr : oCurves) {
			auto& oppEdge = *oppPtr;
			if (!oppEdge.ccOverlaps)
				continue;
            // !!! does this need a 'edge is line and opp is line' case?
            if (edge.isLine())
                addedSect |= addLineCurveIntersection(edge, oppEdge, curveRef);
            else if (oppEdge.isLine())
                addedSect |= addLineCurveIntersection(oppEdge, edge, !curveRef);
			else if (!splitMid)
				addedSect |= setHullSects(edge, oppEdge, curveRef);
		}
	}
    return addedSect;
}

bool OpCurveCurve::setOverlaps() {
	edgeCurves.clear();
	oppCurves.clear();
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		OpRect edgeBounds = edge.bounds();
		for (auto oppPtr : oppCurves.c) {
			auto& oppEdge = *oppPtr;
			OpRect oppBounds = oppEdge.bounds();
			// !!! used to be close bounds; simple bounds required for testQuads1883885
			if (depth > 1 && !edgeBounds.intersects(oppBounds))
				continue;
			bool sharesPoint = (edge.ccStart || edge.ccEnd) && (oppEdge.ccStart || oppEdge.ccEnd);
			// if bounds have common edge only and already share point, they don't intersect
			if (sharesPoint) {
				// !!! can one have no area (e.g. horz or vert) and the other not?
				if (edgeBounds.hasArea() && oppBounds.hasArea() &&
						!edgeBounds.areaOverlaps(oppBounds))
					continue;
			}
			if (!rotatedIntersect(edge, oppEdge, sharesPoint))
				continue;
			if (!rotatedIntersect(oppEdge, edge, sharesPoint))
				continue;
			oppEdge.ccOverlaps = true;
			edge.ccOverlaps = true;
		}
	}
	return !rotateFailed;
}

bool OpCurveCurve::smallTFound(CurveRef curveRef) {
    for (FoundLimit& limit : limits.i) {
        if (CurveRef::edge == curveRef ? 0 == limit.segPtT.t : 0 == limit.oppPtT.t)
            return true;
    }
    return false;
}

bool OpCurveCurve::splitDownTheMiddle(const OpEdge& edge, CurveRef curveRef, CcCurves& splits) {
	// !!!? while edgeMid is in a deleted bounds, bump edgeMidT
	//      (this isn't necessarily in the intersection hull)
	//      wait until this is necessary to make it work
	// checkSplit(edge.startT, edge.endT, which, edgeMid);
	const OpPtT& edgeMid = edge.center;
	if (OpMath::EqualT(edge.startT, edgeMid.t))
		return false;
	if (OpMath::EqualT(edgeMid.t, edge.endT))
		return false;
	if (edge.centerless)
		return false;
	OP_ASSERT(edge.startT < edgeMid.t);
	OP_ASSERT(edgeMid.t < edge.endT);
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
    OpEdge* splitLeft = allocateEdge(nullptr, &edge, edgeMid, OpPtT(), NewEdge::isLeft,  
            EdgeOverlaps::overlaps  OP_LINE_FILE_PARAMS(edge.id));
	if (!splitLeft->disabled) {
		splitLeft->ccStart = edge.ccStart;
		splitLeft->ccSmall = edge.ccSmall;
		OP_ASSERT(!splitLeft->disabled);
		splits.c.push_back(splitLeft);
		curves.addEdgeRun(splitLeft, EdgeMatch::end, ClampDist::yes  OP_LINE_FILE_PARGS());
	}
	OpEdge* splitRight = allocateEdge(nullptr, &edge, edgeMid, OpPtT(), NewEdge::isRight,  
             EdgeOverlaps::overlaps  OP_LINE_FILE_PARAMS(edge.id));
	if (!splitRight->disabled) {
		splitRight->ccEnd = edge.ccEnd;
		splitRight->ccLarge = edge.ccLarge;
		OP_ASSERT(!splitRight->disabled);
		splits.c.push_back(splitRight);
        curves.addEdgeRun(splitRight, EdgeMatch::end, ClampDist::yes  OP_LINE_FILE_PARGS());  // !!! was missing; if deliberate, document why
	}
	return true;
}

// If edge has hull points that define split but opp does not, opp may be entirely inside edge hull.
// Modify below to keep opp corresponding to kept edge split to handle this case.
// where should the opp be stored?

// if end hull index == -1, discard both sides of sect (cutout via exact or nearby sect)
// if end hull >= 0, look for sect through curve
bool OpCurveCurve::splitHulls(CurveRef which, CcCurves& splits) {
	OpVector threshold = context->threshold;
	CcCurves& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	CcCurves& oCurves = CurveRef::edge == which ? oppCurves : edgeCurves;
	OpSegment* segment = CurveRef::edge == which ? seg : opp;
	for (auto edgePtr : curves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		if (edge.startPtT().isNearly(edge.center, threshold) 
				|| edge.center.isNearly(edge.endPtT(), threshold)) {
			edge.isUnsplitable = true;
			++unsplitables;
			splits.c.push_back(edgePtr);  // caller will split with snip and go
			if (CurveRef::edge == which)
				splitHullFail = true;
			continue;
		}
		if (CurveRef::edge == which)
			splitHullFail = false;
		OpHulls& hulls = edge.hulls;
		if (!hulls.h.size() || splitMid) {
			if (!splitDownTheMiddle(edge, which, splits))
				return false;
			continue;
		}
		if (2 <= hulls.h.size()) {  // see if hulls are close enough to define an intersection
			hulls.sort(smallTFound(which));
#if OP_DEBUG
			for (size_t index = 1; index < hulls.h.size(); ++index) {
				if (!hulls.debugSectCandidates((int) index, edge))
					continue;
				OpDebugOut("!!! splitHulls fail:"  OP_DEBUG_CODE(+ context->debugData.testname)
					+ std::string("\n"));
			}
#endif
		}
		OP_DEBUG_VALIDATE_CODE(hulls.debugValidate());
		hulls.add(edge.startPtT(), threshold, edge.startDist, SectType::endHull);
		hulls.add(edge.endPtT(), threshold, edge.endDist, SectType::endHull);
		if (!hulls.nudgeDeleted(edge, *this, which))
			return false;  // soft error; give up on this intersection
		size_t splitCount = splits.c.size();
		for (size_t index = 0; index + 1 < hulls.h.size(); ) {
			const HullSect& hullLo = hulls.h[index];
			const HullSect& hullHi = hulls.h[++index];
			if (edge.ccStart && edge.startPtT().isNearly(hullHi.sect, threshold))
				continue;
			if (edge.ccEnd && edge.endPtT().isNearly(hullLo.sect, threshold))
				continue;
			if (OpMath::EqualT(hullLo.sect.t, hullHi.sect.t))
				continue;
			// hull points have been aligned, and may not be on edge
			OpEdge* split = allocateEdge(segment, nullptr, hullLo.sect, hullHi.sect, NewEdge::none,  
					EdgeOverlaps::overlaps  OP_LINE_FILE_PARAMS(edge.id));
			if (split->disabled)
				continue;
			if (hullLo.oppDist.isSet()) {
				split->startDist = hullLo.oppDist;
                if (split->startDist.dist * split->endDist.dist < 0) {
                    curves.addEdgeRun(split, EdgeMatch::start, ClampDist::no  OP_LINE_FILE_PARGS());
                    oCurves.complementRun(split);
                }
            }
			if (hullHi.oppDist.isSet()) {
				split->endDist = hullHi.oppDist;
                if (split->startDist.dist * split->endDist.dist < 0) {
                    curves.addEdgeRun(split, EdgeMatch::end, ClampDist::no  OP_LINE_FILE_PARGS());
                    oCurves.complementRun(split);
                }
            }
            // !!! add edge run
			split->ccStart = edge.ccStart && edge.startPtT().isNearly(split->startPtT(), threshold);
			if (split->ccStart)
				split->ccSmall = edge.ccSmall;
			split->ccEnd = edge.ccEnd && edge.startPtT().isNearly(split->endPtT(), threshold);
			if (split->ccEnd)
				split->ccLarge = edge.ccLarge;
			if (split->endT < edge.endT) {
				EdgeRun* zeroDistance = addEdgeRun(split, which, EdgeMatch::end  OP_LINE_FILE_PARGS());
				if (zeroDistance) {
					if (CurveRef::edge == which)
						recordSect(parentEdge, parentOpp, zeroDistance->edgePtT,
								zeroDistance->oppPtT  OP_LINE_FILE_PARGS());
					else
						recordSect(parentEdge, parentOpp, zeroDistance->oppPtT,
								zeroDistance->edgePtT  OP_LINE_FILE_PARGS());
				}
			}
			splits.c.push_back(split);
		}
		if (splits.lopSided(splitCount, maxSplitBias)) {
			splits.c.resize(splitCount); // this may leave an edge was never used ... not the end of the world
			if (!splitDownTheMiddle(edge, which, splits))  // divide edge in two
				return false;
		}
		if (!splits.c.empty()) {
			OpEdge* edgeLo = splits.c[0];
			for (size_t index = 1; index < splits.c.size(); ++index) {
				OpEdge* edgeHi = splits.c[index];
				if (edgeLo->endT == edgeHi->startT) {
					if (!edgeHi->startDist.isSet()) {
						edgeHi->startDist = edgeLo->endDist;
//                        OP_DEBUG_CODE(curves.debugCheck(edgeHi, EdgeMatch::start)); // !!! should edge run be added? check to see if already there
                        if (edgeHi->startDist.dist * edgeHi->endDist.dist < 0)
                            oCurves.complementRun(edgeHi);
					} else if (!edgeLo->endDist.isSet()) {
						edgeLo->endDist = edgeHi->startDist;
//                        OP_DEBUG_CODE(curves.debugCheck(edgeLo, EdgeMatch::end));
                        if (edgeLo->startDist.dist * edgeLo->endDist.dist < 0)
                            oCurves.complementRun(edgeLo);
                    }
				}
				edgeLo = edgeHi;
			}
		}
	}
	return true;
}
