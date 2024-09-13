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
    for (auto contour : contours.contours) {
        for (auto& segment : contour->segments) {
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
}

// may need to adjust values in opp if end is nearly equal to seg
void OpSegments::AddEndMatches(OpSegment* seg, OpSegment* opp) {
    auto add = [seg, opp](OpPoint pt, float segT, float oppT   OP_LINE_FILE_DEF()) {
        if (opp->disabled || seg->disabled)
            return;
        if (seg->sects.contains(OpPtT { pt, segT }, opp) 
                || opp->sects.contains(OpPtT { pt, oppT }, seg))
            return;
        OpIntersection* sect = seg->addSegSect(OpPtT { pt, segT }, opp  
                OP_LINE_FILE_CALLER());
        OpIntersection* oSect = opp->addSegSect(OpPtT { pt, oppT }, seg 
                OP_LINE_FILE_CALLER());
        sect->pair(oSect);
    };
    auto checkEnds = [add, seg, opp](OpPoint oppPt, float oppT  OP_LINE_FILE_DEF()) {
        float segT = OpNaN;
        if (seg->c.firstPt().isNearly(oppPt)) {
            segT = 0;
            if (oppPt != seg->c.firstPt()) {
                oppPt = seg->c.firstPt();
                opp->moveTo(oppT, oppPt);
            }
        } else if (seg->c.lastPt().isNearly(oppPt)) {
            segT = 1;
            if (oppPt != seg->c.lastPt()) {
                oppPt = seg->c.lastPt();
                opp->moveTo(oppT, oppPt);
            }
        }
        OP_ASSERT(opp->c.c.data->start != opp->c.c.data->end || opp->disabled);
        if (!OpMath::IsNaN(segT))
            add(oppPt, segT, oppT  OP_LINE_FILE_CALLER());
        return segT;
    };
    float startSegT = checkEnds(opp->c.firstPt(), 0  OP_LINE_FILE_PARAMS());
    float endSegT = checkEnds(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
    auto checkOpp = [add, opp](OpPoint segPt, float segT  OP_LINE_FILE_DEF()) {
        float oppT = opp->c.match(0, 1, segPt);
		if (OpMath::IsNaN(oppT))
            return oppT;
        oppT = OpMath::PinNear(oppT);
        if ((0 == oppT || 1 == oppT) && opp->c.end(oppT) != segPt)
            opp->moveTo(oppT, segPt);
        add(segPt, segT, oppT  OP_LINE_FILE_CALLER());
        return oppT;
    };
    float startOppT = OpNaN;
    float endOppT = OpNaN;
    if (0 != startSegT && 0 != endSegT) 
        startOppT = checkOpp(seg->c.firstPt(), 0  OP_LINE_FILE_PARAMS());  // see if start pt is on opp curve
    if (1 != startSegT && 1 != endSegT) 
		endOppT = checkOpp(seg->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
    auto checkSeg = [add, seg](OpPoint oppPt, float oppT  OP_LINE_FILE_DEF()) {
        float segT = seg->c.match(0, 1, oppPt);
		if (OpMath::IsNaN(segT))
            return;
        segT = OpMath::PinNear(segT);
        if ((0 == segT || 1 == segT) && seg->c.end(segT) != oppPt)
            seg->moveTo(segT, oppPt);
        add(oppPt, segT, oppT  OP_LINE_FILE_CALLER());
    };
    if (OpMath::IsNaN(startSegT) && 0 != startOppT && 0 != endOppT)
        checkSeg(opp->c.firstPt(), 0  OP_LINE_FILE_PARAMS());
    if (OpMath::IsNaN(endSegT) && 1 != startOppT && 1 != endOppT)
        checkSeg(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
}

// somewhat different from winder's edge based version, probably for no reason
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OP_ASSERT(opp != seg);
    OP_ASSERT(seg->c.debugIsLine());
    LinePts edgePts { seg->c.firstPt(), seg->c.lastPt() };
    OP_ASSERT(edgePts.pts[0] != edgePts.pts[1]);
    MatchReverse matchRev = opp->matchEnds(seg);
    // if line and curve share end point, pass hint that root finder can call
    // reduced form that assumes one root is zero or one.
    OpRoots septs = opp->c.rayIntersect(edgePts, matchRev.match);
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp->c.toVertical(edgePts, matchRev.match);
		septs.roots[0] = rotated.tZeroX(0, 1);
		septs.count = 1;
	}
    if (opp->c.isLine() && MatchEnds::both == matchRev.match) {
        seg->moveWinding(opp, matchRev.reversed);
        return;
    }
    if (2 == septs.count && opp->c.isLine()) {
        OpWinder::CoincidentCheck({ edgePts.pts[0], 0 }, { edgePts.pts[1], 1 },
                { opp->c.firstPt(), 0}, { opp->c.lastPt(), 1 }, seg, opp );
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
        OpPoint oppPt = opp->c.ptAtT(oppT);  // !!! redundant if ray intersect is rewritten to return pt
        if (oppPt.isNearly(oppT < .5 ? opp->c.firstPt() : opp->c.lastPt()))
            continue;
        if (oppPt.isNearly(edgePts.pts[0]))
            continue;
        if (oppPt.isNearly(edgePts.pts[1]))
            continue;
        float edgeT = seg->findValidT(0, 1, oppPt);
        oppPt = seg->c.ptAtT(edgeT);  // use line instead of curve to keep points on line
        OpPtT oppPtT { oppPt, oppT };
        if (OpMath::IsNaN(edgeT))
            continue;
        if (OpMath::NearlyEndT(edgeT))
            continue;
        seg->ptBounds.pin(&oppPtT.pt);  // required by testLine409
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
                OP_LINE_FILE_PARAMS(opp));
        OpIntersection* oSect = opp->addSegBase(oppPtT  
                OP_LINE_FILE_PARAMS(seg));
        sect->pair(oSect);
    }
    // if pair share two intersections, and mid t is close, mark intersections as unsectable
    OpIntersection* iStart = nullptr;
    OpIntersection* iEnd = nullptr;
    size_t index = 0;
    while (index < seg->sects.i.size()) {
        OpIntersection* test = seg->sects.i[index];
        if (test->opp->segment == opp) {
            if (iStart) {
                iEnd = test;
                break;
            }
            iStart = test;
        }
        ++index;
    }
    if (iEnd) {
        float midT = OpMath::Average(iStart->ptT.t, iEnd->ptT.t);
        // distance from seg point at midT normal to opp segment
        OpPtT midPtT = seg->c.ptTAtT(midT);
        OpPtT oppPtT = CcCurves::Dist(seg, midPtT, opp);
        float dist = (midPtT.pt - oppPtT.pt).length();
        auto endFromT = [](OpIntersection* one, OpIntersection* two, MatchEnds match) -> MatchEnds {
            return (one->ptT.t < two->ptT.t) == (MatchEnds::start == match) 
                    ? MatchEnds::start : MatchEnds::end;
        };
        if (dist < OpEpsilon * 8) { // !!! who knows what this const should be?
	        int usectID = seg->nextID();
            seg->addUnsectable(iStart->ptT, usectID, endFromT(iStart, iEnd, MatchEnds::start), opp
                    OP_LINE_FILE_PARAMS());
            seg->addUnsectable(iEnd->ptT, usectID, endFromT(iStart, iEnd, MatchEnds::end), opp
                    OP_LINE_FILE_PARAMS());
            OpIntersection* oStart = iStart->opp;
            OpIntersection* oEnd = iEnd->opp;
	        bool flipped = oStart->ptT.t > oEnd->ptT.t;
            if (flipped)
                usectID = -usectID;
            opp->addUnsectable(oStart->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::start), seg
                    OP_LINE_FILE_PARAMS());
            opp->addUnsectable(oEnd->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::end), seg
                    OP_LINE_FILE_PARAMS());
        }
    }
    return;
}

void OpSegments::FindCoincidences(OpContours* contours) {
    // take care of totally coincident segments
    SegmentIterator segIterator(contours);
    while (OpSegment* seg = segIterator.next()) {
        SegmentIterator oppIterator = segIterator;
        while (OpSegment* opp = oppIterator.next()) {
            if (seg->ptBounds != opp->ptBounds)
                continue;
            MatchReverse mr = seg->matchEnds(opp);
            if (MatchEnds::both != mr.match || seg->c.c.type != opp->c.c.type)
                continue;
                // if control points and weight match, treat as coincident: transfer winding
            if (!seg->contour->contours->callBack(seg->c.c.type).curvesEqualFuncPtr(
                    seg->c.c, opp->c.c ))
                continue;
            seg->winding.move(opp->winding, mr.reversed);
            opp->winding.zero();
            opp->setDisabled(OP_LINE_FILE_NPARAMS());
            if (!seg->winding.visible()) {
                seg->setDisabled(OP_LINE_FILE_NPARAMS());
                break;
            }
        }
    }
}

IntersectResult OpSegments::LineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(seg->c.debugIsLine());
    OP_ASSERT(!seg->disabled);
    // special case pairs that exactly match start and end
    MatchReverse ends = seg->matchEnds(opp);
    if (MatchEnds::both == ends.match) {
        seg->moveWinding(opp, ends.reversed);
        return IntersectResult::yes;
    }
    OpVector tangent = seg->c.tangent(0);
    if (!tangent.dx || !tangent.dy) {
        OP_ASSERT(tangent.dx || tangent.dy);
        OP_ASSERT(opp->c.debugIsLine());
        OP_ASSERT(!opp->disabled);
        OpVector oTangent = opp->c.tangent(0);
        if (oTangent.dx && oTangent.dy)
            return IntersectResult::no;
        OP_ASSERT(oTangent.dx || oTangent.dy);
        if (!tangent.dot(oTangent))  // if at right angles, skip
            return IntersectResult::no;
        OP_ASSERT(seg->ptBounds.intersects(opp->ptBounds));
        seg->makeEdge(OP_LINE_FILE_NPARAMS());
        opp->makeEdge(OP_LINE_FILE_NPARAMS());
        return OpWinder::CoincidentCheck(seg->edges.front(), opp->edges.front());
    }
    LinePts oppLine = opp->c.linePts();
    OpCurve vertSeg = seg->c.toVertical(oppLine, ends.match);
    if (!vertSeg.isVertical())
        return IntersectResult::no;
    LinePts segLine = seg->c.linePts();
    OpCurve vertOpp = opp->c.toVertical(segLine, ends.flipped());
    if (!vertOpp.isVertical())
        return IntersectResult::no;
    OpPtT oppInSeg[2], segInOpp[2];
    OpPtT* oppInSegPtr = oppInSeg;
    OpPtT* segInOppPtr = segInOpp;
    auto checkBetween = [](OpCurve& vert, OpCurve& base, float test, OpPtT*& inPtr) {
        if (OpMath::Between(vert.firstPt().y, test, vert.lastPt().y)
                && vert.firstPt().y != test && test != vert.lastPt().y) {
            float t = OpMath::XYRatio(vert.firstPt().y, vert.lastPt().y, test);
            *inPtr++ = OpPtT(OpMath::Interp(base.firstPt(), base.lastPt(), t), t);
            return true;
        }
        return false;
    };
    auto checkVert = [checkBetween](OpCurve& vertBase, OpCurve& segCurve, OpCurve& vertO, 
            OpCurve& oppCurve, OpPtT*& oInSegPtr, OpPtT*& sInOppPtr
            OP_DEBUG_PARAMS(OpPtT* oInSeg, OpPtT* sInOpp)) {
        if (checkBetween(vertBase, segCurve, vertO.firstPt().y, oInSegPtr))
            *sInOppPtr++ = OpPtT(oppCurve.firstPt(), 0);
        if (checkBetween(vertBase, segCurve, vertO.lastPt().y, oInSegPtr))
            *sInOppPtr++ = OpPtT(oppCurve.lastPt(), 1);
    };
    OpCurve vertSegBase = seg->c.toVertical(segLine, MatchEnds::start);
    checkVert(vertSegBase, seg->c, vertOpp, opp->c, oppInSegPtr, segInOppPtr
            OP_DEBUG_PARAMS(oppInSeg, segInOpp));
    OpCurve vertOppBase = opp->c.toVertical(oppLine, MatchEnds::start);
    checkVert(vertOppBase, opp->c, vertSeg, seg->c, segInOppPtr, oppInSegPtr
            OP_DEBUG_PARAMS(segInOpp, oppInSeg));
    if (MatchEnds::none != ends.match) {
        OP_ASSERT(oppInSegPtr < &oppInSeg[2]);
        *oppInSegPtr++ = MatchEnds::start == ends.match 
                ? OpPtT(seg->c.firstPt(), 0) : OpPtT(seg->c.lastPt(), 1);
        OP_ASSERT(segInOppPtr < &segInOpp[2]);
        *segInOppPtr++ = MatchEnds::start == ends.flipped() 
                ? OpPtT(opp->c.firstPt(), 0) : OpPtT(opp->c.lastPt(), 1);
    }
    if (oppInSegPtr - oppInSeg != 2)
        return IntersectResult::no;
    if (segInOppPtr - segInOpp != 2)
        return IntersectResult::no;
    if (oppInSeg[0].t > oppInSeg[1].t) {
        std::swap(oppInSeg[0], oppInSeg[1]);
        std::swap(segInOpp[0], segInOpp[1]);
    }
//    OP_ASSERT(oppInSeg[0].pt.isNearly(segInOpp[0].pt));
//    OP_ASSERT(oppInSeg[1].pt.isNearly(segInOpp[1].pt));
    OpPtT::MeetInTheMiddle(oppInSeg[0], segInOpp[0]);
    OpPtT::MeetInTheMiddle(oppInSeg[1], segInOpp[1]);
    OpVector segV = oppInSeg[1].pt - oppInSeg[0].pt;
    XyChoice larger = fabsf(segV.dx) > fabsf(segV.dy) ? XyChoice::inX : XyChoice::inY;
    return OpWinder::AddPair(larger, oppInSeg[0], oppInSeg[1], segInOpp[0], segInOpp[1],
	        segInOpp[0].t > segInOpp[1].t, seg, opp);
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
            // set both to lines if they are linear before using them in t calculations
            (void) seg->c.isLine();
            (void) opp->c.isLine();
            AddEndMatches(seg, opp);
            if (seg->disabled || opp->disabled)
                continue;
            // for line-curve intersection we can directly intersect
            if (seg->c.isLine()) {
                if (opp->c.isLine()) {
                    IntersectResult lineCoin = LineCoincidence(seg, opp);
                    if (seg->disabled)
                        break;
                    if (IntersectResult::fail == lineCoin)
                        return FoundIntersections::fail;
                    if (IntersectResult::yes == lineCoin)
                        continue;
                }
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (opp->c.isLine()) {
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
#if 0  // !!! fails to detect unsectable pairs
            if ((sharesHorizontal || sharesVertical) 
                    && MatchEnds::none != seg->matchEnds(opp).match)
                continue;
#endif
            // look for curve curve intersections (skip coincidence already found)
            OpCurveCurve cc(seg, opp);
            SectFound ccResult = cc.divideAndConquer();
#if OP_DEBUG_DUMP
            if (cc.dumpBreak(false))
                OP_ASSERT(0);
#endif
            if (true) { // SectFound::fail == ccResult || SectFound::maxOverlaps == ccResult
                        //        || SectFound::noOverlapDeep == ccResult
                // !!! as an experiment, search runs for small opp distances; turn found into limits
                SectFound limitsResult = cc.runsToLimits();
                if (SectFound::add == limitsResult)
                    ccResult = limitsResult;
#if 0
                else if (SectFound::fail == limitsResult) {
                    if (cc.limits.size())
                        ccResult = SectFound::add;
                    else {
                        if (SectFound::maxOverlaps != ccResult 
                                && SectFound::noOverlapDeep != ccResult) {
                            OP_DEBUG_DUMP_CODE(debugContext = "");
                            return FoundIntersections::fail;
                        }
                        ccResult = SectFound::no;
                    }
                }
#endif
            }
            if (SectFound::add == ccResult || cc.limits.size())
                cc.findUnsectable();
#if 0
            OP_DEBUG_DUMP_CODE(debugContext = "");
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
#endif
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
