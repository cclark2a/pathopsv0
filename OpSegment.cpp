#include "OpContour.h"
#include "OpSegment.h"

OpSegment::OpSegment(const OpPoint pts[], OpType type, OpContour* parent)
    : contour(parent)
    , c(pts, type)
    , winding(parent->operand) {
    complete();
}

OpSegment::OpSegment(const OpPoint pts[3], float w, OpContour* parent)
    : contour(parent)
    , c(pts, w)
    , winding(parent->operand) {
    complete();
}

OpSegment::OpSegment(const OpPoint& pt1, const OpPoint& pt2, OpContour* parent)
    : contour(parent)
    , c(pt1, pt2)
    , winding(parent->operand) {
    complete();
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found
// if operand is diff or reverse diff, the subtracted path has an interior of zero
// if the operand is union or intersection, both paths have an interior of one
void OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges,
        AllowReversal canReverse) const {
    assert(edge->winding.visible());
    // each prospective match normal must agree with edge, indicating direction of area outside fill
    OpPtT ptT = edge->whichPtT(match);
//    OpDebugBreak(edge, 267, EdgeMatch::start == match);
    for (auto& sect : intersections) {
        if (sect.ptT.t < ptT.t)
            continue;  // !!! could binary search for 1st if intersection list is extremely long
        if (sect.ptT.t > ptT.t)
            break;
        OpPtT oppPtT = sect.opp->ptT;
        if (ptT.pt != oppPtT.pt)
            continue;
        const OpSegment* sectSeg = sect.opp->segment;
        OpEdge* start = sectSeg->findActive(oppPtT, EdgeMatch::start);
        // flip zero if operator is diff/revdiff and edges are from both operands
        if (start && start != edge) {
            WindZero startZeroSide = start->windZero;
            if (EdgeMatch::start == match)
                flip(&startZeroSide);
            if (edge->windZero == startZeroSide) {
                if (!start->hasLinkTo(match))
                    oppEdges.emplace_back(start, EdgeMatch::start, AllowReversal::no);
                else if (AllowReversal::yes == canReverse && !start->hasLinkTo(Opposite(match)))
                    oppEdges.emplace_back(start, EdgeMatch::end, canReverse);
            }
        }
        OpEdge* end = sectSeg->findActive(oppPtT, EdgeMatch::end);
        if (end && end != edge) {
            WindZero endZeroSide = end->windZero;
            if (EdgeMatch::end == match)
                flip(&endZeroSide);
            if (edge->windZero == endZeroSide) {
                if (!end->hasLinkTo(match))
                    oppEdges.emplace_back(end, EdgeMatch::end, AllowReversal::no);
                else if (AllowReversal::yes == canReverse && !end->hasLinkTo(Opposite(match)))
                    oppEdges.emplace_back(end, EdgeMatch::start, canReverse);
            }
        }
    }
//    OpDebugBreak(edge, 285, EdgeMatch::start == match);
    if ((EdgeMatch::start == match && edge->start.t > 0)
            || (EdgeMatch::end == match && edge->end.t < 1)) {
        EdgeMatch neighbor = EdgeMatch::start == match ? Opposite(edge->whichEnd) : edge->whichEnd;
        OpEdge* nextDoor = findActive(ptT, neighbor);
        for (auto& alreadyFound : oppEdges)
            if (alreadyFound.edge == nextDoor)
                return;
        if (nextDoor) {
            if (edge->windZero == nextDoor->windZero) {
                if (!nextDoor->hasLinkTo(match))
                    oppEdges.emplace_back(nextDoor, neighbor, AllowReversal::no);
            }
        }
    }
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

// if bounds is dirty, recalc bounds from intersections
// if segment collapses to point, remove entries from intersections?
void OpSegment::calcBounds() {
    if (!recomputeBounds)
        return;
    recomputeBounds = false;
    OpPointBounds bounds;
    for (auto& sect : intersections) {
        bounds.add(sect.ptT.pt);
    }
    tightBounds.set(bounds);
    if (!tightBounds.isEmpty())
        return;
    winding.zero(ZeroReason::collapsed);
}

int OpSegment::coinID(bool flipped) {
    int coinID = ++contour->contours->coincidenceID;
    if (flipped)
        coinID = -coinID;
    return coinID;
}

void OpSegment::complete() {
    ptBounds.set(c);
    tightBounds.set(c);
    recomputeBounds = false;
#if OP_DEBUG
    id = contour->contours->id++;
#endif
}

bool OpSegment::containsSect(float t, const OpSegment* opp) const {
    for (const OpIntersection& sect : intersections) {
        if (sect.ptT.t == t && sect.segment == opp)
            return true;
    }
    return false;
}

// !!! would it be any better (faster) to split this into findStart / findEnd instead?
OpEdge* OpSegment::findActive(OpPtT ptT, EdgeMatch match) const {
    for (auto& edge : edges) {
        if (edge.isPoint)
            continue;
        if (ptT == (EdgeMatch::start == match ? edge.start : edge.end)) {
            if (!edge.active)
                return nullptr;
            return !edge.winding.visible() ? nullptr : const_cast<OpEdge*>(&edge);
        }
    }
    return nullptr;
}

void OpSegment::findCoincidences() {
}

void OpSegment::findExtrema() {
    if (pointType == c.type)
        return;
    if (!winding.left && !winding.right)
        return;
    assert(0 == edges.size());  // otherwise, call repair afterwards
    std::vector<OpPtT> selfPtTs;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.xExtrema); ++index) {
        if (OpMath::IsNaN(tightBounds.xExtrema[index].t))
            break;
        selfPtTs.push_back(tightBounds.xExtrema[index]);
    }
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.yExtrema); ++index) {
        if (OpMath::IsNaN(tightBounds.yExtrema[index].t))
            break;
        selfPtTs.push_back(tightBounds.yExtrema[index]);
    }
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.inflections); ++index) {
        if (OpMath::IsNaN(tightBounds.inflections[index].t))
            break;
        selfPtTs.push_back(tightBounds.inflections[index]);
    }
    std::sort(selfPtTs.begin(), selfPtTs.end(), [](const OpPtT& s1, const OpPtT& s2) {
        return s1.t < s2.t; });
    if (selfPtTs.size()) {
        OpPtT last(OpPoint(), 0);
        for (auto& ptT : selfPtTs) {
            if (ptT.t == last.t)
                continue;
            // don't recompute end pts:  tightbounds and segment have t=0,xtrema,1,..
            if (1 == ptT.t)
                break;
            // add self intersections (e.g., the inflection point) since the opposite is this segment
            // can't point opp at self yet, since intersections will grow later
            intersections.emplace_back(ptT, this, SelfIntersect::self
                    OP_DEBUG_PARAMS(IntersectMaker::makeEdges));
            last = ptT;
        }
    }
    sortIntersections();
#if 0 && OP_DEBUG
    if (1 != intersections.back().ptT.t) {
        // !!! figure out why this doesn't draw all segments
        showIDs();
        showPoints();
        showValues();
        ::focus(14);
        ::addFocus(20);
        ::addFocus(32);
    }
#endif
    assert(0 == intersections.front().ptT.t);
    assert(1 == intersections.back().ptT.t);
    const OpIntersection* last = &intersections.front();
    for (auto& sect : intersections) {
        if (sect.ptT.t == last->ptT.t)
            sect.unsortable = last->unsortable;
        last = &sect;
    }
}

float OpSegment::findPtT(OpPoint opp) const {
    OpVector lineSize = c.lastPt() - c.pts[0];
    if (fabsf(lineSize.dy) > fabsf(lineSize.dx))
        return (opp.y - c.pts[0].y) / lineSize.dy;
    return (opp.x - c.pts[0].x) / lineSize.dx;
}

void OpSegment::fixEdges(const OpPtT& alias, OpPoint master) {
    for (auto& edge : edges) {
        if (edge.start == alias)
            edge.start.pt = master;
        if (edge.end == alias)
            edge.end.pt = master;
        if (edge.start.pt == edge.end.pt) {
            edge.isPoint = true;
            edge.winding.zero(ZeroReason::isPoint);
        }
    }
}

void OpSegment::fixIntersections(OpPoint alias, OpPoint master) {
    for (auto& inner : intersections) {  // iterate again since cubic could cross over itself
        if (inner.ptT.pt != alias)
            continue;
        if (edges.size())
            fixEdges(inner.ptT, master);
        inner.ptT.pt = master;
        recomputeBounds = true;
        if (this == inner.segment)
            continue;
        for (auto& o : inner.segment->intersections) {
            if (o.ptT.pt != alias)
                continue;
            o.ptT.pt = master;
            o.segment->recomputeBounds = true;
            if (o.segment->edges.size())
                o.segment->fixEdges(o.ptT, master);
            o.segment->fixIntersections(alias, master);
        }
    }
}

// when called, edges contain monotonic curves; intersections contains points
// split edges until all points are accounted for
void OpSegment::intersectEdge() {
    std::vector<OpEdge> merge;
    std::vector<OpIntersection>::const_iterator sectIter = intersections.begin();
    for (std::vector<OpEdge>::const_iterator edgeIter = edges.begin();
            edgeIter != edges.end(); edgeIter++) {
        OpPtT start = edgeIter->start;
        OpPtT end = edgeIter->end;
        for ( ; sectIter != intersections.end(); sectIter++) {
            if (sectIter->ptT.t == start.t) {   // ok if points don't match 
                continue;                       // (avoids short non-zero line with equal ts on end)
            }
            if (sectIter->ptT.t >= end.t) {
                if (edgeIter->start == start)
                    merge.push_back(*edgeIter);
                else
                    merge.emplace_back(this, start, end  OP_DEBUG_PARAMS(EdgeMaker::intersectEdge1));
                break;
            }
            merge.emplace_back(this, start, sectIter->ptT  OP_DEBUG_PARAMS(EdgeMaker::intersectEdge2));
            start = sectIter->ptT;
        }
    }
    if (merge.size() > edges.size()) {
        edges.swap(merge);
    }
}

// count and sort extrema; create an edge for each extrema + 1
void OpSegment::makeEdges() {
    if (pointType == c.type)
        return;
    if (!winding.left && !winding.right)
        return;
    edges.reserve(intersections.size() - 1);
    const OpIntersection* last = &intersections.front();
    for (const auto& sect : intersections) {
        if (sect.ptT.t == last->ptT.t)
            continue;
        edges.emplace_back(this, last->ptT, sect.ptT  OP_DEBUG_PARAMS(EdgeMaker::makeEdges));
#if OP_DEBUG
        if (last->unsortable || sect.unsortable)
            OpDebugOut("");
#endif
        if (last->unsortable && sect.unsortable)
            edges.back().unsortable = true;
        last = &sect;
    }
}

MatchEnds OpSegment::matchEnds(const OpSegment* opp, bool* reversed) const {
    MatchEnds result = MatchEnds::none;
    *reversed = false;
    if (c.pts[0] == opp->c.pts[0])
        result = MatchEnds::start;
    else if (c.pts[0] == opp->c.lastPt()) {
        result = MatchEnds::start;
        *reversed = true;
    }
    bool foundEnd = false;
    if (c.lastPt() == opp->c.pts[0])
        *reversed = foundEnd = true;
    else if (c.lastPt() == opp->c.lastPt())
        foundEnd = true;
    if (foundEnd) {
        result = (MatchEnds)((int) result | (int) MatchEnds::end);
        assert(MatchEnds::end == result || MatchEnds::both == result);
    }
    return result;
}

void OpSegment::matchIntersections() {
    for (auto& i : intersections) {
        if (i.opp)
            continue;
        if (i.self) {
            assert(i.segment == this);
            i.opp = &i; // self intersection (e.g., inflection point)
            continue;
        }
        for (auto& o : i.segment->intersections) {
            if (&i == &o)
                continue;
            if (o.ptT.pt != i.ptT.pt)
                continue;
            if (o.segment != this)
                continue;
            if (i.coincidenceID != o.coincidenceID)
                continue;
            assert(!i.opp);
            assert(!o.opp);
            i.opp = &o;
            o.opp = &i;
            std::swap(i.segment, o.segment);
            break;
        }
        assert(i.opp);
    }
#if OP_DEBUG
    for (auto& i : intersections) {
        assert(i.opp);
        assert(i.opp->opp);
    }
#endif
}

// Note the set of coincident edges bounded by indentical ids may be of different lengths.
// The edge and opp winding counts may also vary. So the initial winding difference has to
// be applied to all subsequent edges, but only the initial opp winding can be zeroed.
void OpSegment::resolveCoincidence() {
    if (!intersections.size()) {
        assert(!winding.visible());
        return;
    }
    OpIntersection* sect = &intersections.front();
    do {
        int coinID = sect->coincidenceID;
        if (!coinID)
            continue;
#if 0 && OP_DEBUG
        if (6 == coinID)
            OpDebugOut("");
#endif
        OpIntersection* last = sect;
        while (coinID != (++last)->coincidenceID)
            assert(last < &intersections.back());
        bool backwards = sect->coincidenceID < 0;
        OpSegment* oSegment = sect->opp->segment;
        // find opp coinid first, last
        OpIntersection* oppSect = &oSegment->intersections.front();
        while (coinID != oppSect->coincidenceID) {
            assert(oppSect < &oSegment->intersections.back());
            ++oppSect;
        }
        OpEdge* edge = &edges.front();
        while (edge->start != sect->ptT) {
            assert(edge != &edges.back());
            ++edge;
        }
        OpIntersection* oppLast = oppSect;
        while (coinID != (++oppLast)->coincidenceID)
            assert(oppLast < &oSegment->intersections.back());
        if (backwards)
            std::swap(oppSect, oppLast);
        // The original and opposite coincident edges must have matching points for the spans with
        // identical windings.
        // !!! add preflight step that ensures that edges line up when this asserts
        OpEdge* oEdge = backwards ? &oSegment->edges.back() : &oSegment->edges.front();
        OP_DEBUG_CODE(OpEdge* oLast = backwards ? &oSegment->edges.front() : &oSegment->edges.back());
        int direction = backwards ? -1 : 1;
        EdgeMatch match = backwards ? EdgeMatch::end : EdgeMatch::start;
        while (oEdge->ptT(match) != sect->opp->ptT) {
            assert(oEdge != oLast);
            oEdge += direction;
        }
        do {
            OpEdge* firstEdge = edge;
            OpWinding _winding = edge->winding;
            while (edge->end != last->ptT) {
                OpEdge* next = edge + 1;
                if (_winding.left != next->winding.left || _winding.right != next->winding.right)
                    break;
                edge = next;
            }    
            OpEdge* firstOpp = oEdge;
            OpWinding oWinding = oEdge->winding;
            while (oEdge->ptT(Opposite(match)) != oppLast->ptT) {
                OpEdge* oNext = oEdge + direction;
                if (oWinding.left != oNext->winding.left || oWinding.right != oNext->winding.right)
                    break;
                oEdge = oNext;
            }
            assert(firstOpp->ptT(match).pt == firstEdge->start.pt);
            assert(oEdge->ptT(Opposite(match)).pt == edge->end.pt);
            firstEdge->winding.move(firstOpp->winding, contour->contours, backwards);
            OpEdge* windEdge = firstEdge;
            while (windEdge != edge) {
                ++windEdge;
                windEdge->winding = firstEdge->winding;
            }
            windEdge = firstOpp;
            while (windEdge != oEdge) {
                windEdge += direction;
                windEdge->winding = firstOpp->winding;
            }
            if (edge->end == last->ptT) {
                assert(oEdge->ptT(Opposite(match)) == oppLast->ptT);
                break;
            }
            assert(edge != &edges.back());
            ++edge;
            assert(oEdge != oLast);
            oEdge += direction;
        } while (true);
        sect->zeroCoincidenceID();
        last->zeroCoincidenceID();
    } while (++sect != &intersections.back());
}

// Different intersections and self intersections may compute the same t but different point values.
// After edges are processed, but before they are assembled, resolve this by moving duplicates
// towards the edge end.
void OpSegment::resolvePoints() {
//    OpDebugBreak(this, 8, true);
    OpPtT last;
    for (auto& sect : intersections) {
        if (sect.ptT.t == last.t && sect.ptT.pt != last.pt) {
            OpPoint masterPt = sect.ptT.t < .5 ? last.pt : sect.ptT.pt; // bias towards end
            OpPoint aliasPt = sect.ptT.t < .5 ? sect.ptT.pt : last.pt;
            // if intersection or edge equals alias pt, change to master pt
            fixIntersections(aliasPt, masterPt);
        }
        last = sect.ptT;
    }
}

void OpSegment::sortIntersections() {
    std::sort(intersections.begin(), intersections.end(),
        [](const OpIntersection& s1, const OpIntersection& s2) {
            return s1.ptT.t < s2.ptT.t 
                    || (s1.ptT.t == s2.ptT.t && !s1.unsortable < !s2.unsortable); 
        });
}

void OpSegment::flip(WindZero* windZero) {
    if (WindZero::noFlip == *windZero)
        return;
    *windZero = WindZero::normal == *windZero ? WindZero::opp : WindZero::normal;
}

bool OpSegment::validEdge(float startT, float endT) const {
    auto valid = [&](float t) {
        return !(startT < t && t < endT);
    };
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.xExtrema); ++index)
        if (!valid(tightBounds.xExtrema[index].t))
            return false;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.yExtrema); ++index)
        if (!valid(tightBounds.yExtrema[index].t))
            return false;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.inflections); ++index)
        if (!valid(tightBounds.inflections[index].t))
            return false;
    return true;
}

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

IntersectResult OpSegments::AddIntersection(OpSegment* opp, OpSegment* seg) {
    OpRoots septs;
    assert(lineType == seg->c.type);
    std::array<OpPoint, 2> edgePts = { seg->c.pts[0], seg->c.pts[1] };
#if 0 && OP_DEBUG
    if (4 == opp->id && 2 == seg->id) {
        showSegments();
        showIDs();
        OpDebugOut("");
    }
#endif
    septs.count = opp->c.rayIntersect(edgePts, septs.roots);
    bool reversed;
    MatchEnds common = seg->matchEnds(opp, &reversed);
    if (!septs.count && MatchEnds::none == common)
        return IntersectResult::no;
    if (lineType == opp->c.type && MatchEnds::both == common) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        return IntersectResult::yes;
    }
    if (2 == septs.count && lineType == opp->c.type) {
        return OpEdges::CoincidentCheck( {seg->c.pts[0], 0}, { seg->c.pts[1], 1 },
            {opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
    }
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    if (septs.count > 1)
        std::sort(&septs.roots[0], &septs.roots[septs.count]);
    int foundCount = 0;
    for (unsigned index = 0; index < septs.count; ++index) {
        OpPtT oppPtT = { opp->c.ptAtT(septs.get(index)), septs.get(index) };
        float edgeT = seg->findPtT(oppPtT.pt);
        if (OpMath::Between(0, edgeT, 1)) {
            // pin point to both bounds, but only if it is on edge
            opp->tightBounds.pin(&oppPtT.pt);
            seg->tightBounds.pin(&oppPtT.pt);
#if OP_DEBUG
			if (51 == seg->contour->contours->id)
				OpDebugOut("");
#endif
            OpPtT edgePtT = { oppPtT.pt, edgeT };
            seg->intersections.emplace_back(edgePtT, opp, 0
                OP_DEBUG_PARAMS(IntersectMaker::addIntersection3));
            opp->intersections.emplace_back(oppPtT, seg, 0
                OP_DEBUG_PARAMS(IntersectMaker::addIntersection4));
            ++foundCount;
        }
    }
    // if foundCount > 1, check each future edge pair to see if they are sortable
    for (int index = 0; index < foundCount - 1; ++index) {
        // create a pair of edges and see if one is in front of the other in x or y
        OpIntersection* oSectB = &opp->intersections.back() - index;
        OpIntersection* oSectA = oSectB - 1;
        OpPtT start = oSectA->ptT;
        OpPtT end = oSectB->ptT;
        if (start.t > end.t)
            std::swap(start, end);
        if (!opp->validEdge(start.t, end.t))
            continue;
        OpEdge oEdge(opp, start, end  OP_DEBUG_PARAMS(EdgeMaker::addTest));
        OpIntersection* sSectB = &seg->intersections.back() - index;
        OpIntersection* sSectA = sSectB - 1;
        OpEdge sEdge(seg, sSectA->ptT, sSectB->ptT  OP_DEBUG_PARAMS(EdgeMaker::addTest));
        OpEdges edges(&sEdge, &oEdge);
        FoundWindings foundWindings = edges.setWindings();
        if (FoundWindings::fail == foundWindings)
            return IntersectResult::fail;
        // this was: (edge fail names have changed)
//        if (sEdge.sum.visible() || EdgeFail::distance == sEdge.fail 
//                || oEdge.sum.visible() || EdgeFail::distance == oEdge.fail)
        // if this is right, add back with a comment why
        if (sEdge.sum.visible() || oEdge.sum.visible())
            continue;
        // if neither edge is in front of the other, they aren't sortable
        oSectA->unsortable = true;
        oSectB->unsortable = true;
        sSectA->unsortable = true;
        sSectB->unsortable = true;
    }
    return foundCount ? IntersectResult::yes : IntersectResult::no;

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
                (void)AddIntersection(opp, seg);
                continue;
            } else if (lineType == opp->c.type) {
                (void)AddIntersection(seg, opp);
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed);
            if ((int) MatchEnds::start & (int) match) {
                seg->intersections.emplace_back(OpPtT{ seg->c.pts[0], 0 }, opp, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections3));
                if (reversed)
                    opp->intersections.emplace_back(OpPtT{ opp->c.lastPt(), 1 }, seg, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections4));
                else
                    opp->intersections.emplace_back(OpPtT{ opp->c.pts[0], 0 }, seg, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections5));
            }
            if ((int) MatchEnds::end & (int) match) {
                seg->intersections.emplace_back(OpPtT{ seg->c.lastPt(), 1 }, opp, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections6));
                if (reversed)
                    opp->intersections.emplace_back(OpPtT{ opp->c.pts[0], 0 }, seg, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections7));
                else
                    opp->intersections.emplace_back(OpPtT{ opp->c.lastPt(), 1 }, seg, 0
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections8));
            }
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}

