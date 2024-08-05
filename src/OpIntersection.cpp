// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpIntersection.h"
#include "OpContour.h"
#include "OpSegment.h"

#if 0
void OpIntersection::betweenPair(OpIntersection* end) {
    for (OpEdge& edge : segment->edges) {
        if (edge.start.t < ptT.t)
            continue;
        OP_ASSERT(ptT.t <= edge.start.t && edge.start.t < end->ptT.t);
        edge.isUnsortable = true;
        edge.isUnsectable = true;
        if (end->ptT.t == edge.end.t)
            break;
    }
}
#endif

// setter to help debugging
void OpIntersection::setCoin(int cid, MatchEnds end) {
    OP_ASSERT(60 != cid);
    coincidenceID = cid;
    coinEnd = end;
}

// setter to help debugging
void OpIntersection::setUnsect(int uid, MatchEnds end) {
    unsectID = uid;
    unsectEnd = end;
}

OpIntersections::OpIntersections()
    : resort(false) {
}

OpIntersection* OpIntersections::add(OpIntersection* sect) {
    i.push_back(sect);
    resort = true;
    return sect;
}

// this matches opp with nearby ptT
OpIntersection* OpIntersections::contains(const OpPtT& ptT, const OpSegment* opp) {
	for (unsigned index = 0; index < i.size(); ++index) {
        OpIntersection* sect = i[index];
        if (!sect->opp || sect->opp->segment != opp)
            continue;
		if (ptT.isNearly(sect->ptT))
			return sect;
	}
	return nullptr;
}

// this matches sects exactly, and is for pair lookups
OpIntersection* const * OpIntersections::entry(const OpPtT& ptT, const OpSegment* opp) const {
	for (unsigned index = 0; index < i.size(); ++index) {
        OpIntersection* sect = i[index];
        if (!sect->opp || sect->opp->segment != opp)
            continue;
		if (ptT.pt == sect->ptT.pt || ptT.t == sect->ptT.t)  // !!! should this be && ?
			return &i[index];
	}
	return nullptr;
}

// count and sort extrema; create an edge for each extrema + 1
// 
// for the unsectable / between code:
//   if this set of intersections is reversed compared to the opposite, walk opposite backwards 
void OpIntersections::makeEdges(OpSegment* segment) {
    std::vector<const OpIntersection*> unsectables;
    OpIntersection* last = i.front();
    OP_ASSERT(!resort);
    for (auto sectPtr : i) {
        OpIntersection& sect = *sectPtr;
        if (sect.ptT.t != last->ptT.t) {
            segment->edges.emplace_back(segment, last->ptT, sect.ptT
                    OP_LINE_FILE_PARAMS(EdgeMaker::makeEdges, last, sectPtr));
            OpEdge& newEdge = segment->edges.back();
            if (unsectables.size())
                newEdge.isUnsectable = true;
        }
        if (sect.unsectID) {
            auto found = std::find_if(unsectables.begin(), unsectables.end(), 
                    [&sect](const OpIntersection* uT) { return uT->unsectID == sect.unsectID; });
            if (unsectables.end() == found)
                unsectables.push_back(&sect);
            else
                unsectables.erase(found);
        } 
        if (sect.ptT.t != last->ptT.t)
            last = &sect;
    }
}

#if 0
void OpIntersections::markBetween() {
// if we are inside a unsectable span, see if the pal has the other end
    OpIntersection* sectOpp = sect.opp;
    for (auto unsectable : unsectables) {
        if (unsectable->ptT.t == sect.ptT.t)
            continue;
        bool foundStart = false;
        for (OpIntersection* pal : unsectable->opp->segment->sects.i) {
        if (pal->unsectID == unsectable->unsectID) {
            if (foundStart)
                break;
            foundStart = true;
            continue;
        }
        if (!foundStart)
            continue;
        OpIntersection* palOpp = pal->opp;
        if (sectOpp->segment != palOpp->segment)
            continue;
        // found same segment intersections on both unsectables; mark as between
        if (palOpp->ptT.t > sectOpp->ptT.t)
            std::swap(palOpp, sectOpp);
        // mark the edges, if they've been allocated, as betweeners
        palOpp->betweenPair(sectOpp);
        break;
    }
}
#endif

const OpIntersection* OpIntersections::nearly(const OpPtT& ptT, OpSegment* oSeg) const {
	for (unsigned index = 0; index < i.size(); ++index) {
        OpIntersection* sect = i[index];
        if (oSeg && (!sect->opp || sect->opp->segment != oSeg))
            continue;
		if (ptT.pt.isNearly(sect->ptT.pt) 
                || (ptT.t - OpEpsilon <= sect->ptT.t && sect->ptT.t <= ptT.t + OpEpsilon))
			return sect;
	}
	return nullptr;
}

void OpIntersections::range(const OpSegment* opp, std::vector<OpIntersection*>& result) {
	if (resort)
	    sort();
    OP_DEBUG_CODE(float last = -1);
    for (auto sect : i) {
        if (sect->opp && sect->opp->segment == opp) {
            OP_ASSERT(last < sect->ptT.t);
            OP_DEBUG_CODE(last = sect->ptT.t);
            result.push_back(sect);
        }
    }
}

std::vector<OpIntersection*> OpIntersections::unsectables(const OpEdge* edge) {
    std::vector<OpIntersection*> result;
    for (auto sect : i) {
        if (sect->ptT.t > edge->start.t)
            break;
        if (sect->unsectID) {
            auto found = std::find_if(result.begin(), result.end(), 
                    [&sect](const OpIntersection* uT) { return uT->unsectID == sect->unsectID; });
            if (result.end() == found)
                result.push_back(sect);
            else
                result.erase(found);
        }
    }
    return result;
}

bool OpIntersections::UnsectablesOverlap(std::vector<OpIntersection*> set1,
        std::vector<OpIntersection*> set2) {
    for (OpIntersection* i1: set1) {
        for (OpIntersection* i2 : set2) {
            if (i1->unsectID == i2->unsectID)
                return true;
        }
    }
    return false;
}

struct SectPreferred {
    SectPreferred(OpIntersection* sect)
        : best(sect->ptT.pt)
        , bestOnEnd(sect->ptT.onEnd()) {
        OP_DEBUG_CODE(int safetyValue = 10);  // !!! no idea what this should be
        do {
            if (find(sect))  // repeat once if better point was found
                return;
            visited.clear();
            OP_ASSERT(--safetyValue);
        } while (true);
    }

    bool find(OpIntersection* );

    std::vector<OpSegment*> visited;
    OpPoint best;
    bool bestOnEnd;
};

void emergencyDump(OpContours* );

bool SectPreferred::find(OpIntersection* sect) {
    OpIntersections& sects = sect->segment->sects;
    visited.push_back(sect->segment);
    OpIntersection* firstOfRun = nullptr;
    for (OpIntersection* test : sects.i) {
        if (!test->ptT.pt.isNearly(best)) {
            if (firstOfRun)
                break;
            continue;
        }
        if (!firstOfRun) {
            firstOfRun = test;
            if (sects.i.back()->ptT.pt.isNearly(best)) {
// small segment may have first and last nearly touching
//                OP_ASSERT(0 != firstOfRun->ptT.t);  // breaks: cubic422305
                if (!firstOfRun->ptT.t)
                    sect->segment->setDisabled(OP_DEBUG_CODE(ZeroReason::collapsed));
                OP_ASSERT(1 == sects.i.back()->ptT.t);
                firstOfRun->ptT.t = 1;
            }
        }
        if (test->ptT.pt != best) {
            if (test->ptT.onEnd()) {
                if (bestOnEnd) {
                    test->segment->moveTo(test->ptT.t, best);
                } else {
                    best = test->ptT.pt;
                    bestOnEnd = test->ptT.onEnd();
                    return false; // best changed; start over
                }
            }
        } else if (test->ptT.onEnd())
            bestOnEnd = true;
        OP_ASSERT(test->ptT.pt.isNearly(best));
// if segment is very small, points may be nearly equal but t values may be comparitively large
//        OP_ASSERT(OpMath::NearlyEqualT(test->ptT.t, firstOfRun->ptT.t));  // breaks: cubic641
        test->ptT = OpPtT(best, firstOfRun->ptT.t);
        if (visited.end() == std::find(visited.begin(), visited.end(), test->opp->segment)) {
            if (!find(test->opp))
                return false;
        }
    }
    return true;
}

    // intersections may have multiple different t values with the same pt value
    // should be rare; do an exhaustive search for duplicates
void OpIntersections::mergeNear() {
    for (unsigned outer = 1; outer < i.size(); ++outer) {
        OpIntersection* oSect = i[outer - 1];
        unsigned limit = outer;
        bool nearEqual = false;
        do {
            OpIntersection* iSect = i[limit];
            if (!iSect->ptT.isNearly(oSect->ptT))
                break;
            nearEqual |= iSect->ptT != oSect->ptT;
        } while (++limit < i.size());
        if (outer == limit)
            continue;
        if (!nearEqual) {
            outer = limit;
            continue;
        }
        SectPreferred preferred(oSect);
    }
}

void OpIntersections::sort() {
    if (!resort) {
#if OP_DEBUG
        if (i.size()) {
            const OpIntersection* last = i.front();
            auto findEndIndex = [this](const OpIntersection* sect) {
                for (size_t index = 0; index < i.size(); ++index) {
                    const OpIntersection* test = i[index];
                    if (test->coincidenceID == sect->coincidenceID && MatchEnds::end == test->coinEnd)
                        return index;
                    if (test->unsectID == sect->unsectID && MatchEnds::end == test->unsectEnd)
                        return index;
                }
                OP_ASSERT(0); // match not found
                return (size_t) 0;
            };
            for (const auto s : i) {
                OP_ASSERT(last->ptT.t <= s->ptT.t);
                if (last != s && last->ptT.t == s->ptT.t
                       && (MatchEnds::start == last->unsectEnd || MatchEnds::start == last->coinEnd)
                       && (MatchEnds::start == s->unsectEnd || MatchEnds::start == s->coinEnd)) {
                    size_t lastEndIndex = findEndIndex(last);
                    size_t sEndIndex = findEndIndex(s);
                    OP_ASSERT(sEndIndex < lastEndIndex);
                }
                last = s;
            }
        }
#endif
        return;
    }
    resort = false;
    // order first in t, then put coincident start before unmarked, and finally end
    // start: put coincident, unmarked, unsectable (if t is equal)
    // end: put unsectable, unmarked, coincident (if t is equal)
    std::sort(i.begin(), i.end(), [](const OpIntersection* s1, const OpIntersection* s2) {
            if (s1->ptT.t != s2->ptT.t)
                return s1->ptT.t < s2->ptT.t;
            bool s1start = MatchEnds::start == s1->unsectEnd || MatchEnds::start == s1->coinEnd;
            bool s2end = MatchEnds::end == s2->unsectEnd || MatchEnds::end == s2->coinEnd;
            if (s1start && s2end)
                return false;
            bool s1end = MatchEnds::end == s1->unsectEnd || MatchEnds::end == s1->coinEnd;
            bool s2start = MatchEnds::start == s2->unsectEnd || MatchEnds::start == s2->coinEnd;
            if (s1end && s2start)
                return true;
            if (s1start)
                return MatchEnds::start == s1->coinEnd;
            if (s2end)
                return MatchEnds::end == s2->coinEnd;
            if (s1end)
                return MatchEnds::end == s1->unsectEnd;
            return MatchEnds::start == s2->unsectEnd;
    });
    if (3 >= i.size())
        return;
    // Two or more coincident (or unsectable) pairs with the same t may require further
    // sorting. Order them so that they nest coincidences by finding the other end.
    size_t rangeStart = 0;
    auto processStart = [this, &rangeStart](size_t rangeEnd) {
        size_t toFind = rangeEnd - rangeStart;
        std::vector<OpIntersection*> sorted(toFind);  // reserve sorted pointers copy (zeroed)
        size_t endI = rangeEnd;
        size_t found = 0;
        while (found < toFind && endI < i.size()) {  // look for ends that match found starts
            OpIntersection* end = i[endI++];
            if (MatchEnds::end != end->unsectEnd && MatchEnds::end != end->coinEnd)
                continue;
            size_t startI = rangeStart;
            do {   // for found end, find matching start
                OpIntersection* start = i[startI++];
                OP_ASSERT(MatchEnds::start == start->unsectEnd
                        || MatchEnds::start == start->coinEnd);
                if ((start->unsectID != end->unsectID || !start->unsectID) 
                        && (start->coincidenceID != end->coincidenceID || !start->coincidenceID))
                    continue;
                ++found;
                OP_ASSERT(!sorted[toFind - found]);
                sorted[toFind - found] = start;  // reverse order so ranges nest
                break;
            } while (startI < rangeEnd);
        }
        OP_ASSERT(found == toFind);
        std::copy(sorted.begin(), sorted.end(), i.begin() + rangeStart);
    };
    float t = 0;
    size_t index = 0;
    do {  // iterate through all, gathering groups of equal t start values
        OpIntersection* sect = i[index];
        bool isStart = MatchEnds::start == sect->unsectEnd || MatchEnds::start == sect->coinEnd;
        float nextT = sect->ptT.t;
        if (isStart && t == nextT)
            continue;
        if (rangeStart + 2 <= index)
            processStart(index);
        rangeStart = index + !isStart;
        t = nextT;
    } while (++index < i.size());
    // first pass (above) resorts range of start with same t. second pass (below) resorts end 
    // !!! maybe two passes can share code? not sure
    auto processEnd = [this, &rangeStart](size_t rangeEnd) {  // look for starts that match ends
        size_t toFind = rangeEnd - rangeStart;
        std::vector<OpIntersection*> sorted(toFind);  // reserve sorted pointers copy (zeroed)
        size_t startI = 0;
        size_t found = 0;
        while (found < toFind && startI < rangeStart) {  // look for starts that match found ends
            OpIntersection* start = i[startI++];
            if (MatchEnds::start != start->unsectEnd && MatchEnds::start != start->coinEnd)
                continue;
            size_t endI = rangeStart;
            do {   // for found start, find matching end
                OpIntersection* end = i[endI++];
                OP_ASSERT(MatchEnds::end == end->unsectEnd
                        || MatchEnds::end == end->coinEnd);
                if ((end->unsectID != start->unsectID || !end->unsectID)
                        && (end->coincidenceID != start->coincidenceID || !end->coincidenceID))
                    continue;
                ++found;
                OP_ASSERT(!sorted[toFind - found]);
                sorted[toFind - found] = end;  // reverse order so ranges nest
                break;
            } while (endI < rangeEnd);
        }
        OP_ASSERT(found == toFind);
        std::copy(sorted.begin(), sorted.end(), i.begin() + rangeStart);
    };
    t = 0;
    index = 0;
    do {  // iterate through all, gathering groups of equal t start values
        OpIntersection* sect = i[index];
        bool isEnd = MatchEnds::end == sect->unsectEnd || MatchEnds::end == sect->coinEnd;
        float nextT = sect->ptT.t;
        if (isEnd && t == nextT)
            continue;
        if (rangeStart + 2 <= index)
            processEnd(index);
        rangeStart = index + !isEnd;
        t = nextT;
    } while (++index < i.size());
    if (rangeStart + 2 <= index)
        processEnd(index);
}

void OpIntersections::windCoincidences(std::vector<OpEdge>& edges) {
    sort();
    std::vector<CoinPair> pairs;
    OpEdge* edge = &edges.front();
    for (auto sectPtr : i) {
        int coinID = sectPtr->coincidenceID;
        if (!coinID)
            continue;
    //    OP_ASSERT(abs(coinID) != 15 && abs(coinID) != 20 && abs(coinID) != 35);
        // start here; 
        if (sectPtr->coincidenceProcessed)
            continue;
        auto pairIter = std::find_if(pairs.begin(), pairs.end(), [coinID](CoinPair& pair) { 
            return coinID == pair.id; 
        });
        if (pairs.end() == pairIter) {  // set up start
            while (edge->start.t != sectPtr->ptT.t) {
                OP_ASSERT(edge->start.t < sectPtr->ptT.t);
                OP_ASSERT(edge < &edges.back());
                ++edge;
            }
            OP_ASSERT(edge->start.t == sectPtr->ptT.t);
            OpSegment* oppSegment = sectPtr->opp->segment;
            if (oppSegment->disabled)
                continue;
//            OP_ASSERT(tangent.dot(oppSegment->c.tangent(0))); // !!! can we safely ignore this?
            auto& oppEdges = oppSegment->edges;
            EdgeMatch match = coinID > 0 ? EdgeMatch::start : EdgeMatch::end;
            OpEdge* oppEdge = &oppEdges.front();
            while (oppEdge->ptT(match) != sectPtr->opp->ptT) {
                ++oppEdge;
                OP_ASSERT(oppEdge <= &oppEdges.back());
            }
            pairs.emplace_back(sectPtr, sectPtr->opp, edge, oppEdge, coinID
                    OP_DEBUG_PARAMS(EdgeMatch::start == match ? &oppEdges.back() : &oppEdges.front()));
        } else {    // set up end
            OP_ASSERT(!pairIter->end);
            pairIter->end = sectPtr;
            pairIter->oEnd = sectPtr->opp;
        }
    }    
    for (auto& coinPair : pairs) {
        coinPair.start->coincidenceProcessed = true;
        coinPair.end->coincidenceProcessed = true;
        coinPair.oStart->coincidenceProcessed = true;
        coinPair.oEnd->coincidenceProcessed = true;
        EdgeMatch match = coinPair.id > 0 ? EdgeMatch::start : EdgeMatch::end;
        // !!! experiment: thread_loops44793 has 2 segments with nearly identical but not equal pts
        // either: make the points the same
        // or: see if we can survive if compares are approximate (try the latter first)
        OP_ASSERT(coinPair.oppEdge->ptT(match).pt.isNearly(coinPair.start->ptT.pt));
        OP_ASSERT(coinPair.edge->start == coinPair.start->ptT);
        // In rare cases (e.g. issue1435) coincidence points may not match; one seg has extra.
        // Defer thinking about this if the winding is uniform. Assert to fix this in the future.
        edge = coinPair.edge;
        OpEdge* oppEdge = coinPair.oppEdge;
        OpEdge* edgeBack = edge;  // find final edge corresponding to final sect
        while (edgeBack->end.t < coinPair.end->ptT.t) {
            OP_ASSERT(edgeBack < &edges.back());
            ++edgeBack;
#if OP_DEBUG
            // if assert, more code to write; add point to opp edge sect list to match 
            if (!edge->disabled && !edgeBack->disabled 
                    && !edge->winding.equal(edgeBack->winding.w)) {  // example: testRect2
                // search intersection list for entry pointing to opp edge at edge end
                auto& oI = oppEdge->segment->sects.i;
                auto oIFound = std::find_if(oI.begin(), oI.end(), 
                        [&edge](auto sect) { return sect->ptT.pt.isNearly(edge->end.pt); });
                OP_ASSERT(oI.end() != oIFound);
            }
#endif
        }
        OP_ASSERT(edgeBack->end == coinPair.end->ptT);
        OpEdge* oppBack = oppEdge;
        while (oppBack->ptT(!match).t != coinPair.oEnd->ptT.t) {
            OP_ASSERT(coinPair.id > 0 ? oppBack < coinPair.lastEdge : oppBack > coinPair.lastEdge);
            coinPair.id > 0 ? ++oppBack : --oppBack;
#if OP_DEBUG
            // more code to write; add point if needed to edge list to match
            if (!oppEdge->disabled && !oppBack->disabled 
                    && !oppEdge->winding.equal(oppBack->winding.w)) {
                // search opp intersection list for entry pointing to edge at opp edge end
                OpEdge* oEdge = coinPair.id > 0 ? oppEdge : oppBack;
                auto iFound = std::find_if(i.begin(), i.end(), 
                        [&oEdge](auto sect) { return sect->ptT.pt.isNearly(oEdge->end.pt); });
                OP_ASSERT(i.end() != iFound);
            }
#endif
        }
        // for each different winding: 
        int oppBump = coinPair.id < 0 ? -1 : 1;
        // surprisingly difficult to get right ...
        for (;;) {
            OP_DEBUG_CODE(PathOpsV0Lib::Winding edgeWinding = edge->winding.copyData());
            OP_DEBUG_CODE(PathOpsV0Lib::Winding oppWinding = oppEdge->winding.copyData());
            OP_ASSERT(edge->start.pt.isNearly( 
                    oppEdge->ptT(coinPair.id > 0 ? EdgeMatch::start : EdgeMatch::end).pt));
            PathOpsV0Lib::Winding combinedWinding;
            bool bothEnabled = !edge->disabled && !oppEdge->disabled;
            bool edgeVisible = false;
            if (bothEnabled) {
                edge->winding.move(oppEdge->winding, coinPair.id < 0);
                combinedWinding = edge->winding.copyData();
                edgeVisible = edge->winding.contour->callBacks.windingVisibleFuncPtr(combinedWinding);
                if (!edgeVisible)
                    edge->setDisabled(OP_DEBUG_CODE(ZeroReason::hvCoincidence1));
                else if (edge->disabled)
                    edge->reenable();  // un-disable it; was disabled from earlier coincidence
                oppEdge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence2));
            }
            for (;;) {
                OpPoint oppEnd = oppEdge->ptT(coinPair.id > 0 
                        ? EdgeMatch::end : EdgeMatch::start).pt;
                if (edge->end.pt == oppEnd)
                    break;
                if (edge->end.pt.isNearly(oppEnd) && ((edge == edgeBack) == (oppEdge == oppBack)))
                    break;
                bool oppInEdge = edge->ptBounds.contains(oppEnd);
                OP_DEBUG_CODE(bool edgeInOpp = oppEdge->ptBounds.contains(edge->end.pt));
                OP_ASSERT(oppInEdge != edgeInOpp);
                if (oppInEdge) {
                    oppEdge += oppBump;
                    OP_ASSERT(oppEdge->winding.equal(oppWinding));
                    if (bothEnabled)
                        oppEdge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence3));
                } else {
                    ++edge;
                    OP_ASSERT(edge->winding.equal(edgeWinding));
                    if (edgeVisible) {
                        edge->winding.w = combinedWinding;
                        if (edge->disabled)
                            edge->reenable();
                    } else if (bothEnabled)
                        edge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence4));
                }
            }
            if (edge == edgeBack) {
                OP_ASSERT(oppEdge == oppBack);
                break;
            }
            ++edge;
            OP_ASSERT(oppEdge != oppBack);
            oppEdge += oppBump;
        }
    }
}

#if OP_DEBUG_VALIDATE
void OpIntersection::debugValidate() const {
    OP_ASSERT(OpMath::Between(0, ptT.t, 1));
    OpPoint pt = segment->c.ptAtT(ptT.t);
    OpMath::DebugCompare(pt, ptT.pt);
    OpPoint oPt = opp->segment->c.ptAtT(opp->ptT.t);
    OpMath::DebugCompare(pt, oPt);
}
#endif

#if OP_DEBUG
void OpIntersection::debugSetID() {
    id = segment->nextID();
}

#if OP_DEBUG_VALIDATE
void OpIntersections::debugValidate() const {
	for (const auto sectPtr : i) {
        OP_ASSERT(sectPtr->opp->opp == sectPtr);
        OP_ASSERT(sectPtr->ptT.pt == sectPtr->opp->ptT.pt 
                || (!!sectPtr->unsectID && !!sectPtr->opp->unsectID));
    }
}
#endif

bool OpIntersections::debugContains(const OpPtT& ptT, const OpSegment* opp) const {
    for (auto sect : i) {
        if ((sect->ptT.pt == ptT.pt || sect->ptT.t == ptT.t) 
                && sect->opp && sect->opp->segment == opp)
            return true;
    }
    return false;
}

OpIntersection* OpIntersections::debugAlreadyContains(const OpPoint& pt, const OpSegment* oppSegment) const {
	for (auto sectPtr : i) {
		const OpIntersection& sect = *sectPtr;
        if (!sect.opp)
            continue;
		if (oppSegment == sect.opp->segment && (pt == sect.ptT.pt || pt == sect.ptT.pt))
			return sectPtr;
	}
	return nullptr;
}
#endif
