// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpIntersection.h"
#include "OpContour.h"
#include "OpSegment.h"

void OpIntersection::setCoin(int cid, MatchEnds end) {
    coincidenceID = cid;
    coinEnd = end;
    segment->hasCoin = true;
}

void OpIntersection::setUnsect(int uid, MatchEnds end) {
    unsectID = uid;
    unsectEnd = end;
    segment->hasUnsectable = true;
}

OpIntersection* OpIntersection::coinOtherEnd() {
    auto endSect = std::find_if(segment->sects.i.begin(), segment->sects.i.end(), 
            [this](const OpIntersection* test) { 
            return test->coincidenceID == coincidenceID && test->coinEnd == !coinEnd; });
    OP_ASSERT(segment->sects.i.end() != endSect);
    return *endSect;
};

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

// if the edge is inside an unsectable range, record all sects that start that range
void OpIntersections::makeEdges(OpSegment* segment) {
    OP_ASSERT(!resort);
    std::vector<OpIntersection*> unsectables;
    auto stackUnsects = [&unsectables](OpIntersection* sect) {
        if (!sect->unsectID)
            return;
        if (MatchEnds::start == sect->unsectEnd) {
            unsectables.push_back(sect);
            return;
        }
        OP_ASSERT(MatchEnds::end == sect->unsectEnd);
        auto found = std::find_if(unsectables.begin(), unsectables.end(), 
                [sect](const OpIntersection* uT) { return uT->unsectID == sect->unsectID; });
        OP_ASSERT(unsectables.end() != found);
        unsectables.erase(found);
    };
    std::vector<CoinPal> coincidences;
    auto stackCoins = [&coincidences](OpIntersection* sect) {
        int coinID = sect->coincidenceID;
        if (!coinID)
            return;
        if (MatchEnds::start == sect->coinEnd) {
            coincidences.push_back({ sect->opp->segment, coinID });
            return;
        }
        OP_ASSERT(MatchEnds::end == sect->coinEnd);
        auto found = std::find_if(coincidences.begin(), coincidences.end(), [coinID]
                (const CoinPal& cPal) { return cPal.coinID == coinID; });
        OP_ASSERT(coincidences.end() != found);
        coincidences.erase(found);
    };
    OpIntersection* first = i.front();
    for (OpIntersection* sectPtr : i) {
        if (first->ptT.t != sectPtr->ptT.t) {
            segment->edges.emplace_back(first, sectPtr  OP_LINE_FILE_PARAMS());
            first = sectPtr;
            OpEdge& newEdge = segment->edges.back();
            if (unsectables.size())
                newEdge.unSects = unsectables;
            if (coincidences.size()) {
                newEdge.coinPals = coincidences;
            }
        }
        stackUnsects(sectPtr);
        stackCoins(sectPtr);
    }
}

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

struct SectPreferred {
    SectPreferred(OpIntersection* sect)
        : best(sect)
        , bestOnEnd(sect->ptT.onEnd()) {
        OP_DEBUG_CODE(int safetyValue = 10);  // !!! no idea what this should be
        do {
            if (find())  // repeat once if better point was found
                return;
            visited.clear();
            OP_ASSERT(--safetyValue);
        } while (true);
    }

    bool find();  // make consecutive nearly equal sects the same, and follow each opposite

    std::vector<OpSegment*> visited; // visit each segment with matching sects once
    OpIntersection* best;  // an end point, if one exists; otherwise, an arbitrary sect
    bool bestOnEnd;  // never cleared, only set (once first sect on end is found)
};

bool SectPreferred::find() {
    OpSegment* seg = best->segment;
    OpIntersections& sects = seg->sects;
    visited.push_back(seg);
    bool sawBest = false;
    for (OpIntersection* test : sects.i) {
        if (!test->ptT.isNearly(best->ptT)) {
            if (sawBest)
                break;
            continue;
        }
        test->mergeProcessed = true;  // skip this when seen by merge near
        if (test == best) {
            sawBest = true;
            if (sects.i.back()->ptT.isNearly(best->ptT)) {
// small segment may have first and last nearly touching
                if (0 == best->ptT.t) {
                    seg->setDisabled(OP_LINE_FILE_NPARAMS());
                    return false;
                }
                OP_ASSERT(1 == sects.i.back()->ptT.t);
                best->ptT.t = 1;
            }
        } else if (test->ptT != best->ptT) {
            if (test->ptT.onEnd()) {
                if (bestOnEnd) {
                    seg->moveTo(test->ptT.t, best->ptT.pt);
                } else {
                    best = test;
                    bestOnEnd = true;
                    return false; // best changed; start over
                }
            }
            test->ptT = best->ptT;
            test->opp->ptT.pt = best->ptT.pt;
        }
        if (visited.end() == std::find(visited.begin(), visited.end(), test->opp->segment)) {
            OpIntersection* save = best;
            best = test->opp;
            if (!find())
                return false;
            best = save;
        }
    }
    return true;
}

    // intersections may have multiple different t values with the same pt value
    // should be rare; do an exhaustive search for duplicates
void OpIntersections::mergeNear(OpPtAliases& aliases) {
    for (unsigned outer = 1; outer < i.size(); ++outer) {
        OpIntersection* oSect = i[outer - 1];
        if (oSect->mergeProcessed)
            continue;
        unsigned limit = outer;
        bool nearEqual = false;
        do {
            OpIntersection* iSect = i[limit];
            if (!iSect->ptT.isNearly(oSect->ptT))
                break;
            if (iSect->ptT == oSect->ptT)
                continue;
            nearEqual = true;
            if (aliases.contains(iSect->ptT.pt))
                oSect = iSect;
        } while (++limit < i.size());
        if (outer == limit)
            continue;
        if (!nearEqual) {
            outer = limit;
            continue;
        }
//        start here;
        // preferred needs to choose from contours aliases if a match exists there
        SectPreferred preferred(oSect);
    }
}

bool OpIntersections::simpleEnd() const {
    OP_ASSERT(!resort);
    OP_ASSERT(i.size() > 1);
    OP_ASSERT(i.back()->ptT.t == 1);
    return i[i.size() - 2]->ptT.t != 1;
}

bool OpIntersections::simpleStart() const {
    OP_ASSERT(!resort);
    OP_ASSERT(i.size() > 1);
    OP_ASSERT(i.front()->ptT.t == 0);
    return i[1]->ptT.t != 0;
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
