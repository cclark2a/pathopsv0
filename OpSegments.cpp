#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpSegments.h"

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
            if (pointType == segment.c.type)
                continue;
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
}

// !!! I'm bothered that curve / curve calls a different form of this with edges
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OpRoots septs;
    assert(lineType == seg->c.type);
    LinePts edgePts { seg->c.pts[0], seg->c.pts[1] };
    septs = opp->c.rayIntersect(edgePts);
    bool reversed;
    MatchEnds common = seg->matchEnds(opp, &reversed);
    if (!septs.count && MatchEnds::none == common)
        return; // IntersectResult::no;
    if (lineType == opp->c.type && MatchEnds::both == common) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        opp->winding.zero(ZeroReason::addIntersection);
        return; // IntersectResult::yes;
    }
    if (2 == septs.count && lineType == opp->c.type)
        return (void) OpEdges::CoincidentCheck({ seg->c.pts[0], 0 }, { seg->c.pts[1], 1 },
                { opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    if (septs.count > 1)
        std::sort(&septs.roots[0], &septs.roots[septs.count]);
    for (unsigned index = 0; index < septs.count; ++index) {
        OpPtT oppPtT { opp->c.ptAtT(septs.get(index)), septs.get(index) };
        float edgeT = seg->findPtT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        // pin point to both bounds, but only if it is on edge
        opp->tightBounds.pin(&oppPtT.pt);
        seg->tightBounds.pin(&oppPtT.pt);
        OpPtT edgePtT { oppPtT.pt, edgeT };
        OpIntersection* sect = seg->addSegSect(edgePtT  
                OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurve), SectReason::lineCurve, opp));
        OpIntersection* oSect = opp->addSegSect(oppPtT  
                OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurveOpp), SectReason::lineCurve, opp));
        sect->pair(oSect);
    }
}

void OpSegments::findCoincidences() {
    // take care of totally coincident segments
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds != opp->ptBounds)
                continue;
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed);
            if (MatchEnds::both == match && seg->c.type == opp->c.type) {
                // if control points and weight match, treat as coincident: transfer winding
                bool coincident = false;
                switch (seg->c.type) {
                    case noType:
                        assert(0);
                        break;
                    case pointType:
                        break;
                    case lineType:
                        coincident = true;
                        break;
                    case quadType:
                        coincident = seg->c.pts[1] == opp->c.pts[1];
                        break;
                    case conicType:
                        coincident = seg->c.pts[1] == opp->c.pts[1]
                            && seg->c.weight == opp->c.weight;
                        break;
                    case cubicType:
                        coincident = seg->c.pts[1] == opp->c.pts[1 + (int) reversed]
                            && seg->c.pts[2] == opp->c.pts[2 - (int) reversed];
                        break;
                }
                if (coincident) {
                    seg->winding.move(opp->winding, seg->contour->contours, reversed);
                    opp->winding.zero(ZeroReason::findCoincidences);
                    if (!seg->winding.visible() || !opp->winding.visible())
                        continue;
                }
            }
        }
    }
}

FoundIntersections OpSegments::findIntersections() {
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds.right < opp->ptBounds.left)
                break;
            if (!seg->ptBounds.intersects(opp->ptBounds))
                continue;
            // for line-curve intersection we can directly intersect
            if (lineType == seg->c.type) {
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (lineType == opp->c.type) {
                AddLineCurveIntersection(seg, opp);
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed);
            OpIntersection* oppSect;
            if ((int) MatchEnds::start & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_start), 
                        SectReason::sharedEndPoint, opp));
                if (reversed)
                    oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOppReversed),
                        SectReason::sharedEndPoint, seg));
                else
                    oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOpp),
                        SectReason::sharedEndPoint, seg));
                sect->pair(oppSect);
            }
            if ((int) MatchEnds::end & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_end),
                        SectReason::sharedEndPoint, opp));
                if (reversed)
                    oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOppReversed),
                        SectReason::sharedEndPoint, seg));
                else
                    oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOpp),
                        SectReason::sharedEndPoint, seg));
                sect->pair(oppSect);
            }
            // look for curve curve intersections (skip coincidence already found)
            if (!seg->edges.size()) 
                seg->edges.emplace_back(seg, OpPtT(seg->c.pts[0], 0), OpPtT(seg->c.lastPt(), 1), false
                        OP_DEBUG_PARAMS(EDGE_MAKER(segSect), nullptr, nullptr));
            if (!opp->edges.size()) 
                opp->edges.emplace_back(opp, OpPtT(opp->c.pts[0], 0), OpPtT(opp->c.lastPt(), 1), false  
                        OP_DEBUG_PARAMS(EDGE_MAKER(oppSect), nullptr, nullptr));
            OpCurveCurve cc(&seg->edges.back(), &opp->edges.back());
            SectFound result = cc.divideAndConquer();
            if (SectFound::fail == result)
                return FoundIntersections::fail;
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
