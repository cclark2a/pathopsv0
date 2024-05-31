// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpSegments.h"
#include "OpWinder.h"

static bool compareXBox(const OpSegment* s1, const OpSegment* s2) {
    const OpRect& r1 = s1->ptBounds;
    const OpRect& r2 = s2->ptBounds;
    if (r1.left < r2.left)
        return true;
    if (r1.left > r2.left)
        return false;
    if (r1.left == r2.left && r1.right < r2.right)
        return true;
    if (r1.left == r2.left && r1.right > r2.right)
        return false;
    return s1->id < s2->id;
}

OpSegments::OpSegments(OpContours& contours) {
    inX.clear();
    for (auto& contour : contours.contours) {
        for (auto& segment : contour.segments) {
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
}

// may need to adjust values in opp if end is nearly equal to seg
void OpSegments::AddEndMatches(OpSegment* seg, OpSegment* opp) {
    auto add = [seg, opp](OpPoint pt, float segT, float oppT   OP_LINE_FILE_DEF(int dummy)) {
        OpIntersection* sect = seg->addSegSect(OpPtT { pt, segT }, opp  
                OP_LINE_FILE_CALLER(SectReason::lineCurve));
        OpIntersection* oSect = opp->addSegSect(OpPtT { pt, oppT }, seg 
                OP_LINE_FILE_CALLER(SectReason::lineCurve));
        sect->pair(oSect);
    };
    auto checkEnds = [add, seg, opp](OpPoint pt, float oppT  OP_LINE_FILE_DEF(int dummy)) {
        float segT = OpNaN;
        if (seg->c.pts[0].isNearly(pt)) {
            segT = 0;
            if (pt != seg->c.pts[0]) {
                pt = seg->c.pts[0];
                opp->moveTo(oppT, pt);
            }
        } else if (seg->c.lastPt().isNearly(pt)) {
            segT = 1;
            if (pt != seg->c.lastPt()) {
                pt = seg->c.lastPt();
                opp->moveTo(oppT, pt);
            }
        }
        if (!OpMath::IsNaN(segT))
            add(pt, segT, oppT  OP_LINE_FILE_PARAMS(0));
        return segT;
    };
    float segTforOppStart = checkEnds(opp->c.pts[0], 0  OP_LINE_FILE_PARAMS(0));
    float segTforOppEnd = checkEnds(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS(0));
    auto checkOpp = [add, opp](OpPoint segPt, float segT  OP_LINE_FILE_DEF(int dummy)) {
        float oppT = opp->c.match(0, 1, segPt);
		if (OpMath::IsNaN(oppT))
            return;
        oppT = OpMath::PinNear(oppT);
        add(segPt, segT, oppT  OP_LINE_FILE_PARAMS(0));
    };
    if (0 != segTforOppStart && 0 != segTforOppEnd) 
        checkOpp(seg->c.pts[0], 0  OP_LINE_FILE_PARAMS(0));  // see if start pt is on opp curve
    if (1 != segTforOppStart && 1 != segTforOppEnd) 
		checkOpp(seg->c.lastPt(), 1  OP_LINE_FILE_PARAMS(0));
    auto checkSeg = [add, seg](OpPoint oppPt, float oppT  OP_LINE_FILE_DEF(int dummy)) {
        float segT = seg->c.match(0, 1, oppPt);
		if (OpMath::IsNaN(segT))
            return;
        segT = OpMath::PinNear(segT);
        add(oppPt, segT, oppT  OP_LINE_FILE_PARAMS(0));
    };
    if (OpMath::IsNaN(segTforOppStart))
        checkSeg(opp->c.pts[0], 0  OP_LINE_FILE_PARAMS(0));
    if (OpMath::IsNaN(segTforOppEnd))
        checkSeg(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS(0));
}

// somewhat different from winder's edge based version, probably for no reason
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OP_ASSERT(opp != seg);
    OP_ASSERT(OpType::line == seg->c.type);
    LinePts edgePts { seg->c.pts[0], seg->c.pts[1] };
    MatchReverse matchRev = seg->matchEnds(opp);
    if (matchRev.reversed) {
        if (MatchEnds::start == matchRev.match)
            matchRev.match = MatchEnds::end;
        else if (MatchEnds::end == matchRev.match)
            matchRev.match = MatchEnds::start;
    }
    // if line and curve share end point, pass hint that root finder can call
    // reduced form that assumes one root is zero or one.
    OpRoots septs = opp->c.rayIntersect(edgePts, matchRev.match);
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp->c.toVertical(edgePts);
		septs.roots[0] = rotated.tZeroX(0, 1);
		septs.count = 1;
	}
    if (OpType::line == opp->c.type && MatchEnds::both == matchRev.match) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        if (!seg->winding.visible())
            seg->setDisabled(OP_DEBUG_CODE(ZeroReason::addIntersection));
        opp->winding.zero();
        opp->setDisabled(OP_DEBUG_CODE(ZeroReason::addIntersection));
        return;
    }
    if (2 == septs.count && OpType::line == opp->c.type) {
        OpWinder::CoincidentCheck({ seg->c.pts[0], 0 }, { seg->c.pts[1], 1 },
                { opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
        return;
    }
#if 0
    if (!!(MatchEnds::start & matchRev.match))
        septs.addEnd(0);
    if (!!(MatchEnds::end & matchRev.match))
        septs.addEnd(1);
#endif
    std::vector<OpPtT> oppPtTs;
    std::vector<OpPtT> edgePtTs;
    for (unsigned index = 0; index < septs.count; ++index) {
        float oppT = septs.get(index);
        if (OpMath::NearlyEndT(oppT))
            continue;
        // if computed point is nearly end, ignore
        OpPoint oppPt = opp->c.ptAtT(oppT);
        if (oppPt.isNearly(oppT < .5 ? opp->c.pts[0] : opp->c.lastPt()))
            continue;
        if (oppPt.isNearly(seg->c.pts[0]))
            continue;
        if (oppPt.isNearly(seg->c.lastPt()))
            continue;
        OpPtT oppPtT { opp->c.ptAtT(oppT), oppT };
        float edgeT = seg->findValidT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        if (OpMath::NearlyEndT(edgeT))
            continue;
        seg->ptBounds.pin(&oppPtT.pt);
        opp->ptBounds.pin(&oppPtT.pt);
        oppPtTs.push_back(oppPtT);
        edgePtTs.emplace_back(oppPtT.pt, edgeT);
        OpPtT& edgePtT = edgePtTs.back();
        for (size_t earlier = 1; earlier < oppPtTs.size(); ++earlier) {
            if (oppPtTs[earlier - 1].t == oppPtT.t)
                continue;
        }
        for (size_t earlier = 1; earlier < edgePtTs.size(); ++earlier) {
            if (edgePtTs[earlier - 1].t == edgePtT.t)
                continue;
        }
        if (seg->sects.contains(edgePtT, opp))
            continue;
        if (opp->sects.contains(oppPtT, seg))
            continue;
        OpIntersection* sect = seg->addSegBase(edgePtT  
                OP_LINE_FILE_PARAMS(SectReason::lineCurve, opp));
        OpIntersection* oSect = opp->addSegBase(oppPtT  
                OP_LINE_FILE_PARAMS(SectReason::lineCurve, seg));
        sect->pair(oSect);
    }
    return;
}

void OpSegments::findCoincidences() {
    // take care of totally coincident segments
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (seg->disabled)
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (opp->disabled)
                continue;
            if (seg->ptBounds != opp->ptBounds)
                continue;
            MatchReverse mr = seg->matchEnds(opp);
            if (MatchEnds::both == mr.match && seg->c.type == opp->c.type) {
                // if control points and weight match, treat as coincident: transfer winding
                bool coincident = false;
                switch (seg->c.type) {
                    case OpType::no:
                        OP_ASSERT(0);
                        break;
                    case OpType::line:
                        coincident = true;
                        break;
                    case OpType::quad:
                        coincident = seg->c.pts[1] == opp->c.pts[1];
                        break;
                    case OpType::conic:
                        coincident = seg->c.pts[1] == opp->c.pts[1]
                            && seg->c.weight == opp->c.weight;
                        break;
                    case OpType::cubic:
                        coincident = seg->c.pts[1] == opp->c.pts[1 + (int) mr.reversed]
                            && seg->c.pts[2] == opp->c.pts[2 - (int) mr.reversed];
                        break;
                }
                if (coincident) {
                    seg->winding.move(opp->winding, seg->contour->contours, mr.reversed);
                    opp->winding.zero();
                    opp->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                    if (!seg->winding.visible()) {
                        seg->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                        break;
                    }
                }
            }
        }
    }
}

IntersectResult OpSegments::lineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(OpType::line == seg->c.type);
    OP_ASSERT(!seg->disabled);
    OpVector tangent = seg->c.asLine().tangent();
    if (tangent.dx && tangent.dy) {
        // special case pairs that exactly match start and end
        MatchReverse ends = seg->matchEnds(opp);
        if (MatchEnds::both == ends.match) {
            OP_ASSERT(0);  // !!! step through this code to make sure it is correct
            seg->makeEdge(OP_LINE_FILE_NPARAMS(EdgeMaker::segSect));
            OpEdge& e = seg->edges.front();
            opp->makeEdge(OP_LINE_FILE_NPARAMS(EdgeMaker::oppSect));
            OpEdge& o = opp->edges.front();
            OpVector sV = seg->c.pts[1] - seg->c.pts[0];
            OpVector oV = opp->c.pts[1] - opp->c.pts[0];
            bool segLonger = fabsf(sV.dx) + fabsf(sV.dy) > fabsf(oV.dx) + fabsf(oV.dy);
            OpVector skewBase = segLonger ? sV : oV;
            XyChoice larger = fabsf(skewBase.dx) > fabsf(skewBase.dy) ? XyChoice::inX : XyChoice::inY;
            OpWinder::AddPair(larger, e.start, e.end, o.start, o.end,
	                ends.reversed, seg, opp);
            return IntersectResult::yes;
        }
        // check for matching slope
        OpVector sV = seg->c.pts[1] - seg->c.pts[0];
        OpVector oV = opp->c.pts[1] - opp->c.pts[0];
        // if slope delta is zero, lines are parallel. Check for near zero by skewing one line and
        // seeing that the skew in both directions yields a greater slope delta
        float slopeDelta = fabsf(sV.dx * oV.dy - sV.dy * oV.dx);
        float largest = std::max({ fabsf(sV.dx), fabsf(sV.dy), fabsf(oV.dx), fabsf(oV.dy) });
        float minimum = OpMath::NextLarger(largest) - largest;
        if (slopeDelta > minimum)
            return IntersectResult::no;
        bool segLonger = fabsf(sV.dx) + fabsf(sV.dy) > fabsf(oV.dx) + fabsf(oV.dy);
        OpVector skewBase = segLonger ? sV : oV;
        XyChoice larger = fabsf(skewBase.dx) > fabsf(skewBase.dy) ? XyChoice::inX : XyChoice::inY;
        if (XyChoice::inY == larger)
            std::swap(skewBase.dx, skewBase.dy);  // always skew larger, placed in x
        OpVector skewOpp = segLonger ? oV : sV;
        // !!! change this to loop until upDelta != slopeDelta?
        OpVector skewUp = { OpMath::NextLarger(skewBase.dx), skewBase.dy };
        if (XyChoice::inY == larger)
            std::swap(skewUp.dx, skewUp.dy);
        float upDelta = fabsf(skewUp.dx * skewOpp.dy - skewUp.dy * skewOpp.dx);
        if (upDelta < slopeDelta)
            return IntersectResult::no;
        // !!! change this to loop until downDelta != slopeDelta?
        OpVector skewDown = { OpMath::NextSmaller(skewBase.dx), skewBase.dy };
        if (XyChoice::inY == larger)
            std::swap(skewDown.dx, skewDown.dy);
        float downDelta = fabsf(skewDown.dx * skewOpp.dy - skewDown.dy * skewOpp.dx);
        if (downDelta < slopeDelta)
            return IntersectResult::no;
        // at this point lines are parallel. See if they are also coincident
        OpPoint* longer = segLonger ? seg->c.pts : opp->c.pts;
        OpPoint* shorter = segLonger ? opp->c.pts : seg->c.pts;
        OpVector longS = longer[0] - shorter[0];
        OpVector longE = longer[1] - shorter[0];
        if ((!longS.dx && !longS.dy) || (!longE.dx && !longE.dy)) {
            longS = longer[0] - shorter[1];
            longE = longer[1] - shorter[1];
            OP_ASSERT((longS.dx || longS.dy) && (longE.dx || longE.dy));
        }
        float longSDelta = fabsf(longS.dx * skewOpp.dy - longS.dy * skewOpp.dx);
        if (upDelta < longSDelta)
            return IntersectResult::no;
        if (downDelta < longSDelta)
            return IntersectResult::no;
        float longEDelta = fabsf(longE.dx * skewOpp.dy - longE.dy * skewOpp.dx);
        if (upDelta < longEDelta)
            return IntersectResult::no;
        if (downDelta < longEDelta)
            return IntersectResult::no;
        // at this point, lines are coincident. Find extremes
        float sStart = longer[0].choice(larger);
        float sEnd = longer[1].choice(larger);
        float oStart = shorter[0].choice(larger);
        float oEnd = shorter[1].choice(larger);
        ends.reversed = (sEnd - sStart) * (oEnd - oStart) < 0;
#if 01
        if (ends.reversed)
            std::swap(sStart, sEnd);
        float sStartTonO = (sStart - oStart) / (oEnd - oStart);
        OpPtT oStartPtT = 0 <= sStartTonO && sStartTonO <= 1 
                ? OpPtT{ longer[ends.reversed], sStartTonO } : OpPtT{ shorter[0], 0 };
        float sEndTonO = (sEnd - oStart) / (oEnd - oStart);
        OpPtT oEndPtT = 0 <= sEndTonO && sEndTonO <= 1 
                ? OpPtT{ longer[!ends.reversed], sEndTonO } : OpPtT{ shorter[1], 1 };
        if (oStartPtT.t >= oEndPtT.t)
            return IntersectResult::no;
        if (oStartPtT.t > oEndPtT.t)
            std::swap(oStartPtT, oEndPtT);
        if (ends.reversed) {
            std::swap(sStart, sEnd);
            std::swap(oStart, oEnd);
        }
        float oStartTonS = (oStart - sStart) / (sEnd - sStart);
        OpPtT sStartPtT = 0 <= oStartTonS && oStartTonS <= 1 
                ? OpPtT{ shorter[ends.reversed], oStartTonS } : OpPtT{ longer[0], 0 };
        float oEndTonS = (oEnd - sStart) / (sEnd - sStart);
        OpPtT sEndPtT = 0 <= oEndTonS && oEndTonS <= 1 
                ? OpPtT{ shorter[!ends.reversed], oEndTonS } : OpPtT{ longer[1], 1 };
        if (sStartPtT.t > sEndPtT.t)
            std::swap(sStartPtT, sEndPtT);
        if (!segLonger) {
            std::swap(sStartPtT, oStartPtT);
            std::swap(sEndPtT, oEndPtT);
        }
#else
        OpPtT sStartPtT = { seg->c.pts[0], 0 };
        OpPtT sEndPtT = { seg->c.pts[1], 1 };
        OpPtT oStartPtT = { opp->c.pts[0], 0 };
        OpPtT oEndPtT = { opp->c.pts[1], 1 };
#endif
        return OpWinder::AddPair(larger, sStartPtT, sEndPtT, oStartPtT, oEndPtT,
	            ends.reversed, seg, opp);
    }
    OP_ASSERT(tangent.dx || tangent.dy);
    OP_ASSERT(OpType::line == opp->c.type);
    OP_ASSERT(!opp->disabled);
    OpVector oTangent = opp->c.asLine().tangent();
    if (oTangent.dx && oTangent.dy)
        return IntersectResult::no;
    OP_ASSERT(oTangent.dx || oTangent.dy);
    if (!tangent.dot(oTangent))  // if at right angles, skip
        return IntersectResult::no;
    OP_ASSERT(seg->ptBounds.intersects(opp->ptBounds));
    seg->makeEdge(OP_LINE_FILE_NPARAMS(EdgeMaker::segSect));
    opp->makeEdge(OP_LINE_FILE_NPARAMS(EdgeMaker::oppSect));
    return OpWinder::CoincidentCheck(seg->edges.front(), opp->edges.front());
}

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (seg->disabled)
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (opp->disabled)
                continue;
            // comparisons below need to be 'nearly' since adjusting opp may make sort incorrect
            // or, exact compare may miss nearly equal seg/opp pairs
            if (seg->closeBounds.right < opp->closeBounds.left)
                break;
            if (!seg->closeBounds.intersects(opp->closeBounds))
                continue;
            AddEndMatches(seg, opp);
            // for line-curve intersection we can directly intersect
            if (OpType::line == seg->c.type) {
                if (OpType::line == opp->c.type) {
                    IntersectResult lineCoin = lineCoincidence(seg, opp);
                    if (IntersectResult::fail == lineCoin)
                        return FoundIntersections::fail;
                    if (IntersectResult::yes == lineCoin)
                        continue;
                }
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (OpType::line == opp->c.type) {
                AddLineCurveIntersection(seg, opp);
                continue;
            }
            // if the bounds only share a corner, there's nothing more to do
            bool sharesHorizontal = seg->ptBounds.right == opp->ptBounds.left
                    || seg->ptBounds.left == opp->ptBounds.right;
            bool sharesVertical = seg->ptBounds.bottom == opp->ptBounds.top
                    || seg->ptBounds.top == opp->ptBounds.bottom;
            if (sharesHorizontal && sharesVertical)
                continue;
            // if the bounds share only an edge, and ends match, there's nothing more to do
            if ((sharesHorizontal || sharesVertical) 
                    && MatchEnds::none != seg->matchEnds(opp).match)
                continue;
            // look for curve curve intersections (skip coincidence already found)
            OpCurveCurve cc(seg, opp);
            SectFound ccResult = cc.divideAndConquer();
            if (SectFound::fail == ccResult)
                return FoundIntersections::fail;
            if (SectFound::add == ccResult)
                cc.findUnsectable();
            else if (SectFound::no == ccResult) {
                // capture the closest point(s) that did not result in an intersection
                // !!! eventually allow capturing more than 1, if curves hit twice
                // !!! if required, document test case that needs it
//                if (!cc.limits.size() && cc.closeBy.size())
//                    cc.tryClose();
            }
            if (!cc.addedPoint)
                continue;
            // if point was added, check adjacent to see if it is concident (issue3517)
            OpPtT segPtT = seg->sects.i.back()->ptT;
            OpPtT oppPtT = opp->sects.i.back()->ptT;
            seg->sects.sort();
            opp->sects.sort();
            // add pair, below, rarely adds additional entries to the intersection vectors
            // pre-expand the vectors so that the additions won't invalidate the loop pointers
            seg->sects.i.reserve(seg->sects.i.size() + 8);  // 4 is maximum
            opp->sects.i.reserve(opp->sects.i.size() + 8);
            OpIntersection* const* sSectPtr = seg->sects.entry(segPtT, opp);
            OpIntersection* const* oSectPtr = opp->sects.entry(oppPtT, seg);
            if (sSectPtr && oSectPtr) {
                auto testCoin = [](OpIntersection* segBase, OpIntersection* segTest, 
                        OpIntersection* oppBase, OpIntersection* oppTest, 
                        bool testGreater, bool flipped) {
                    if (segTest->ptT.pt == oppTest->ptT.pt && 
                            (segTest->opp->segment == oppTest->opp->segment || 
                            (segTest->opp->segment == oppTest->segment 
                            && segTest->segment == oppTest->opp->segment))) {
                        if (MatchEnds::start == segBase->unsectEnd 
                                && MatchEnds::end == segTest->unsectEnd)
                            return true;
                        if (segBase->ptT.isNearly(segTest->ptT))
                            return true;
                        OpVector v = segBase->ptT.pt - segTest->ptT.pt;
                        if (!testGreater) {
                            std::swap(segBase, segTest);
                            std::swap(oppBase, oppTest);
                        }
                        OP_ASSERT(segBase->ptT.t < segTest->ptT.t);
                        OP_ASSERT(flipped ? oppBase->ptT.t > oppTest->ptT.t :
                                oppBase->ptT.t < oppTest->ptT.t);
                        if (IntersectResult::fail == OpWinder::AddPair(v.larger(), segBase->ptT, 
                                segTest->ptT, oppBase->ptT, oppTest->ptT, flipped, segTest->segment,
                                oppTest->segment))
                            return false;
                    }
                    return true;
                };
                auto testOpp = [sSectPtr, oSectPtr, testCoin](OpIntersection* segTest, bool next) {
                    OpSegment* opp = (*oSectPtr)->segment;
                    OpIntersection* const* oTestPtr = oSectPtr;
                    while (oTestPtr > &opp->sects.i.front()) {
                        OpIntersection* oppPrior = *--oTestPtr;
                        OpVector diff = (*oSectPtr)->ptT.pt - oppPrior->ptT.pt;
                        if ((!diff.dx && !diff.dy) || oppPrior->ptT.t == (*oSectPtr)->ptT.t)
                            continue;
                        if (diff.dx && diff.dy)
                            break;
                        if (!testCoin(*sSectPtr, segTest, *oSectPtr, oppPrior, next, next))
                            return false;
                        break;
                    }
                    oTestPtr = oSectPtr;
                    while (oTestPtr < &opp->sects.i.back()) {
                        OpIntersection* oppNext = *++oTestPtr;
                        OpVector diff = (*oSectPtr)->ptT.pt - oppNext->ptT.pt;
                        if ((!diff.dx && !diff.dy) || oppNext->ptT.t == (*oSectPtr)->ptT.t)
                            continue;
                        if (diff.dx && diff.dy)
                            break;
                        if (!testCoin(*sSectPtr, segTest, *oSectPtr, oppNext, next, !next))
                            return false;
                        break;
                    }
                    return true;
                };
                OpIntersection* const* sTestPtr = sSectPtr;
                while (sTestPtr > &seg->sects.i.front()) {
                    OpIntersection* segPrior = *--sTestPtr;
                    OpVector diff = (*sSectPtr)->ptT.pt - segPrior->ptT.pt;
                    if ((!diff.dx && !diff.dy) || segPrior->ptT.t == (*sSectPtr)->ptT.t)
                        continue;
                    if (diff.dx && diff.dy)
                        break;
                    if (!testOpp(segPrior, false))
                        return FoundIntersections::fail;
                    break;
                }
                sTestPtr = sSectPtr;
                while (sTestPtr < &seg->sects.i.back()) {
                    OpIntersection* segNext = *++sTestPtr;
                    OpVector diff = (*sSectPtr)->ptT.pt - segNext->ptT.pt;
                    if ((!diff.dx && !diff.dy) || segNext->ptT.t == (*sSectPtr)->ptT.t)
                        continue;
                    if (diff.dx && diff.dy)
                        break;
                    if (!testOpp(segNext, true))
                        return FoundIntersections::fail;
                    break;
                }
            }
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
