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

// somewhat different from 
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OP_ASSERT(opp != seg);
    OP_ASSERT(OpType::line == seg->c.type);
    LinePts edgePts { seg->c.pts[0], seg->c.pts[1] };
    bool reversed;
    MatchEnds existing;
    MatchEnds common = seg->matchEnds(opp, &reversed, &existing, MatchSect::existing);
    if (reversed) {
        if (MatchEnds::start == existing)
            existing = MatchEnds::end;
        else if (MatchEnds::end == existing)
            existing = MatchEnds::start;
    }
    // if line and curve share end point, pass hint that root finder can call
    // reduced form that assumes one root is zero or one.
    OpRoots septs = opp->c.rayIntersect(edgePts, existing);
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp->c.toVertical(edgePts);
		septs.roots[0] = rotated.tZeroX(0, 1);
		septs.count = 1;
	}
    if (!septs.count && MatchEnds::none == common)
        return;
    if (OpType::line == opp->c.type && MatchEnds::both == common) {
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
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    MatchEnds existingMatch = seg->matchExisting(opp);
    std::vector<OpPtT> oppPtTs;
    std::vector<OpPtT> edgePtTs;
    septs.prioritize01();
    for (unsigned index = 0; index < septs.count; ++index) {
        oppPtTs.emplace_back(opp->c.ptAtT(septs.get(index)), septs.get(index));
        OpPtT& oppPtT = oppPtTs.back();
        float edgeT = seg->findValidT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        if (OpEpsilon >= edgeT) {
            edgeT = 0;
            oppPtT.pt = seg->c.pts[0];
        } else if (1 - OpEpsilon <= edgeT) {
            edgeT = 1;
            oppPtT.pt = seg->c.lastPt();
        } else {
            seg->ptBounds.pin(&oppPtT.pt);
            opp->ptBounds.pin(&oppPtT.pt);
        }
        edgePtTs.emplace_back(oppPtT.pt, edgeT);
        OpPtT& edgePtT = edgePtTs.back();
        if ((int) MatchEnds::start & (int) existingMatch && (edgePtT.pt == seg->c.pts[0] || 0 == edgeT
                || oppPtT.pt == opp->c.lastPt() || 1 == oppPtT.t))
            continue;
        if ((int) MatchEnds::end & (int) existingMatch && (edgePtT.pt == seg->c.lastPt() || 1 == edgeT
                || oppPtT.pt == opp->c.pts[0] || 0 == oppPtT.t))
            continue;
        for (size_t earlier = 1; earlier < oppPtTs.size(); ++earlier) {
            if (oppPtTs[earlier - 1].t == oppPtT.t)
                goto duplicate;
        }
        for (size_t earlier = 1; earlier < edgePtTs.size(); ++earlier) {
            if (edgePtTs[earlier - 1].t == edgePtT.t)
                goto duplicate;
        }
        if (seg->sects.contains(edgePtT, opp))
            goto duplicate;
        if (opp->sects.contains(oppPtT, seg))
            goto duplicate;
        {
            OpIntersection* sect = seg->addSegBase(edgePtT  
                    OP_LINE_FILE_PARAMS(SectReason::lineCurve, opp));
            OpIntersection* oSect = opp->addSegBase(oppPtT  
                    OP_LINE_FILE_PARAMS(SectReason::lineCurve, seg));
            sect->pair(oSect);
        }
duplicate: ;
    }
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
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed, nullptr, MatchSect::allow);
            if (MatchEnds::both == match && seg->c.type == opp->c.type) {
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
                        coincident = seg->c.pts[1] == opp->c.pts[1 + (int) reversed]
                            && seg->c.pts[2] == opp->c.pts[2 - (int) reversed];
                        break;
                }
                if (coincident) {
                    seg->winding.move(opp->winding, seg->contour->contours, reversed);
                    if (!seg->winding.visible())
                        seg->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                    else if (seg->disabled)
                        seg->reenable();
                    opp->winding.zero();
                    opp->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                }
            }
        }
    }
}

IntersectResult OpSegments::lineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(OpType::line == seg->c.type);
    OP_ASSERT(!seg->disabled);
    OpVector tangent = seg->c.asLine().tangent();
    if (tangent.dx && tangent.dy)
        return IntersectResult::no;
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
    seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
    opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
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
            if (seg->ptBounds.right < opp->ptBounds.left)
                break;
            if (!seg->ptBounds.intersects(opp->ptBounds))
                continue;
            // for line-curve intersection we can directly intersect
            if (OpType::line == seg->c.type) {
                if (OpType::line == opp->c.type) {
                    IntersectResult lineCoin = lineCoincidence(seg, opp);
                    if (IntersectResult::fail == lineCoin)
                        return FoundIntersections::fail;
                    if (IntersectResult::yes == lineCoin)
                        continue;
                }
#if OP_DEBUG_RECORD
                OpDebugRecordPause();
#endif
                AddLineCurveIntersection(opp, seg);
#if OP_DEBUG_RECORD
                OpDebugRecordResume();
#endif
                continue;
            } else if (OpType::line == opp->c.type) {
#if OP_DEBUG_RECORD
                OpDebugRecordPause();
#endif
                AddLineCurveIntersection(seg, opp);
#if OP_DEBUG_RECORD
                OpDebugRecordResume();
#endif
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds existing;
            MatchEnds match = seg->matchEnds(opp, &reversed, &existing, MatchSect::existing);
            OpIntersection* oppSect;
            if ((int) MatchEnds::start & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.pts[0], 0 }, opp
                        OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                if (sect) {
                    if (reversed)
                        oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }, seg
                            OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                    else
                        oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }, seg
                            OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                    OP_ASSERT(oppSect);
                    sect->pair(oppSect);
                }
            }
            if ((int) MatchEnds::end & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.lastPt(), 1 }, opp
                        OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                if (sect) {
                    if (reversed)
                        oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }, seg
                            OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                    else
                        oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }, seg
                            OP_LINE_FILE_PARAMS(SectReason::sharedEnd));
                    OP_ASSERT(oppSect);
                    sect->pair(oppSect);
                }
            }
            // if the bounds only share a corner, there's nothing more to do
            bool sharesHorizontal = seg->ptBounds.right == opp->ptBounds.left
                    || seg->ptBounds.left == opp->ptBounds.right;
            bool sharesVertical = seg->ptBounds.bottom == opp->ptBounds.top
                    || seg->ptBounds.top == opp->ptBounds.bottom;
            if (sharesHorizontal && sharesVertical)
                continue;
            // if the bounds share only an edge, and ends match, there's nothing more to do
            if ((sharesHorizontal || sharesVertical) && MatchEnds::none != existing)
                continue;
            // look for curve curve intersections (skip coincidence already found)
            OP_ASSERT(!seg->edges.size() || (1 == seg->edges.size() && !seg->edges[0].start.t
                    && 1 == seg->edges[0].end.t));
            OP_ASSERT(!opp->edges.size() || (1 == opp->edges.size() && !opp->edges[0].start.t
                    && 1 == opp->edges[0].end.t));
            seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
            opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
            OpCurveCurve ccx(&seg->edges.back(), &opp->edges.back());
#if CC_EXPERIMENT
            SectFound experimental = ccx.divideExperiment();
            OP_DEBUG_CODE(ccx.debugDone(seg->contour->contours));
            if (SectFound::fail == experimental)
                return FoundIntersections::fail;
            if (SectFound::no == experimental) {
                // capture the closest point(s) that did not result in an intersection
                // !!! eventually allow capturing more than 1, if curves hit twice
                SoClose closest;
                float best = OpInfinity;
                auto buildClosest = [&best, &closest](OpSegment* s, const std::vector<OpEdge*>& edgeCurves, 
                        OpSegment* o, const std::vector<OpEdge*>& oppCurves) {
                    for (const OpEdge* edge : edgeCurves) {
                        for (const OpPtT& ePtT : { edge->start, edge->end } ) {
                            for (const OpEdge* oppC : oppCurves) {
                                auto checkDistSq = [&best, &closest, &ePtT](const OpPtT& oPtT) {
                                    float distSq = (ePtT.pt - oPtT.pt).lengthSquared();
                                    if (best > distSq) {
                                        best = distSq;
                                        closest.close = ePtT;
                                        closest.oppPtT = oPtT;
                                    }
                                };
                                if (oppC->start.t <= OpEpsilon * 2 
                                        && oppC->start.pt.soClose(ePtT.pt, OpEpsilon * 8))
                                    checkDistSq(oppC->start);
                                if (oppC->end.t >= 1 - OpEpsilon * 2
                                        && oppC->end.pt.soClose(ePtT.pt, OpEpsilon * 8))
                                    checkDistSq(oppC->end);
                            }
                        }
                    }
                    if (OpMath::IsFinite(best)) {
                        auto check = [closest, s, o](OpPoint oPt) {
                            // require either pt or t to be within some metric of closeness
                            // since this specifies intersection points, some sloppiness is OK
                            // as long as the point is (nearly) on each segment
                            for (SoClose& test : s->debugClose) {
                                if (!closest.close.soClose(test.close, OpEpsilon * 8))
                                    continue;
                                if (o == test.oppSeg)
                                    continue;
                                if (!closest.oppPtT.pt.soClose(test.oppPtT.pt, OpEpsilon * 8)) {
                                    OpPoint oEnd = test.oppPtT.t < .5 
                                            ? test.oppSeg->c.pts[0] : test.oppSeg->c.lastPt();
                                    if (oEnd != oPt)
                                        continue;
                                }
                                // post pairs of s/o and s/test.oppSeg to intersections
                                OpPtT sPtT = OpPtT(oPt, closest.close.t);
			                    OpIntersection* sect = s->addSegSect(sPtT, o  
                                        OP_LINE_FILE_PARAMS(SectReason::lineCurve));
                                OpPtT oPtT = OpPtT(oPt, closest.oppPtT.t < .5 ? 0 : 1);
			                    OpIntersection* oSect = o->addSegSect(oPtT, s
					                    OP_LINE_FILE_PARAMS(SectReason::lineCurve));
			                    if (sect && oSect) 
                                    sect->pair(oSect);
                                sect = s->addSegSect(sPtT, test.oppSeg
                                        OP_LINE_FILE_PARAMS(SectReason::lineCurve));
                                OpPtT testPtT = OpPtT(oPt, test.close.t < .5 ? 0 : 1);
                                oSect = test.oppSeg->addSegSect(testPtT, s
					                    OP_LINE_FILE_PARAMS(SectReason::lineCurve));
			                    if (sect && oSect)
                                    sect->pair(oSect);
                            }
                        };
                        // if there is an existing soClose record with a ptT.t value which is
                        // within some diff of epsilon (at least 2 epsilon) then print it
                        // and show more data (normal? tangent?) to see if a pair of records find an
                        // intersection where one segment joins another
                        check(closest.oppPtT.t < .5 ? o->c.pts[0] : o->c.lastPt());
                        closest.oppSeg = o;
                        s->debugClose.push_back(closest);
                        closest.oppSeg = s;
                        std::swap(closest.close, closest.oppPtT);
                        o->debugClose.push_back(closest);
                    }
                };
                buildClosest(seg, ccx.edgeCurves, opp, ccx.oppCurves);
                buildClosest(opp, ccx.oppCurves, seg, ccx.edgeCurves);
                continue;
            }
            // !!! where does the comment below go?
            // if point was added, check adjacent to see if it is concident (issue3517)
#endif
#define CC_OLD_SCHOOL 0
#if CC_OLD_SCHOOL
            OpCurveCurve cc(&seg->edges.back(), &opp->edges.back());
            SectFound result = cc.divideAndConquer();
            OP_DEBUG_CODE(cc.debugDone(seg->contour->contours));
#if CC_EXPERIMENT
            if (experimental != result)
                OpDebugOut(STR(localExperiment) + " we disagree");
#endif
            if (SectFound::fail == result)
                return FoundIntersections::fail;  // triggered by fuzzhang_1
            // if point was added, check adjacent to see if it is concident (issue3517)
            if (!cc.sectResult)
                continue;
#endif
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
