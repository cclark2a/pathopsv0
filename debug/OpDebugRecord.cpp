// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugRecord.h"

#if OP_DEBUG_RECORD

#include "OpContour.h"
#include "OpEdge.h"
#include "OpSegment.h"

// record all line/curve intersections from curve/curve to determine allowable error

struct DebugEdge {
	DebugEdge(const OpEdge& e) {
		curve.set(e.start.pt, e.ctrlPts, e.end.pt, e.segment->c.pointCount(), e.segment->c.type, e.weight);
		segmentCurve = e.segment->c;
		start = e.start.t;
		end = e.end.t;
		id = e.id;
		segmentID = e.segment->id;
	}

	OpCurve curve;
	OpCurve segmentCurve;
	float start;
	float end;
	int id;
	int segmentID;
};

struct DebugTest {
	DebugTest() 
		: t(OpNaN)
		, success(false) {
	}
	float t;
	OpPoint sect;
	OpRoots roots;
	bool success;
};

struct Fodder {
	Fodder(const OpEdge& o, const OpEdge& e) 
		: edge(e)
		, opp(o)
	    , found(0) {
		testname = e.segment->contour->contours->debugTestname;
	}
	DebugEdge edge;
	DebugEdge opp;
	OpCurve vert;
	std::string testname;
	DebugTest tests[3];
	unsigned found;
};

struct Guiness {
	Guiness() 
		: test(-1)
		, x(OpNaN) {
	}

	int test;
	std::string name;
	float x;
};

std::vector<Fodder> data;
bool recordPaused = false;
bool recordCubic = false;
bool recordAll = false;
Guiness recordVert;
Guiness recordSect;

void OpDebugRecordStart(const OpEdge& opp, const OpEdge& edge) {
	OP_ASSERT(!recordPaused);
	OP_ASSERT(edge.isLine_impl);
	if (OpType::cubic != opp.segment->c.type)
		return;
	if (OpDebugExpect::unknown == edge.segment->contour->contours->debugExpect)
		return;
	recordCubic = true;
	if (!recordAll && data.size())
		data.pop_back();
	data.push_back(Fodder(opp, edge));
}

void OpDebugRecord(const OpCurve& c, float t, OpPoint p, const OpRoots& r) {
	if (recordPaused || !recordCubic)
		return;
	OP_ASSERT(data.size());
	Fodder& back = data.back();
	OP_ASSERT(back.found < ARRAY_COUNT(back.tests));
	DebugTest& test = back.tests[back.found];
	test.t = t;
	test.sect = p;
	if (!(fabsf(p.x) <= recordSect.x)) {
		recordSect.test = data.size();
		recordSect.name = back.testname;
		recordSect.x = fabsf(p.x);
	}
	test.roots = r;
	back.vert = c;
	back.found++;

}

void OpDebugRecordEnd() {
	OP_ASSERT(!recordPaused);
	if (!recordCubic)
		return;
	const Fodder& f = data.back();
	if (!f.found) {
		data.pop_back();
		return;
	}
	// check to see if line intersects curve bounding hull
	LinePts edgePts { f.edge.curve.pts[0], f.edge.curve.lastPt() };
	OpCurve vertLine = f.edge.curve.toVertical(edgePts);
	if (!(fabsf(vertLine.lastPt().x) <= recordVert.x)) {
		recordVert.test = data.size();
		recordVert.name = f.testname;
		recordVert.x = fabsf(vertLine.lastPt().x);
	}
#define OP_DEBUG_VERBOSE 0
#if OP_DEBUG_VERBOSE
	std::string s = STR(data.size()) + ") " + f.testname 
		+ "\nedge:" + STR(f.edge.segmentID) + "/" + STR(f.edge.id)
		+ " eCurve t(" + STR(f.edge.start) + "," + STR(f.edge.end) + ") " + f.edge.curve.debugDump()
		+ "\neVert:" + vertLine.debugDump()
		+ "\nopp:" + STR(f.opp.segmentID) + "/" + STR(f.opp.id)
		+ " oCurve t(" + STR(f.opp.start) + "," + STR(f.opp.end) + ") " + f.opp.curve.debugDump()
		+ "\noVert:" + f.vert.debugDump();
	OP_ASSERT(f.found < ARRAY_COUNT(f.tests));
	for (unsigned index = 0; index < f.found; ++index) {
		const DebugTest& t = f.tests[index];
		s += "\nt:" + STR(t.t) + " vpt:" + t.sect.debugDump() + " success:" + STR(t.success);
		OpDebugOut(s + "\n");
		s = "    " + STR((int) index + 2) + ") ";
	}
#endif
	recordCubic = false;
}

void OpDebugRecordPause() {
	recordPaused = true;
}

void OpDebugRecordResume() {
	recordPaused = false;
}

void OpDebugRecordSuccess(unsigned index) {
	OP_ASSERT(!recordPaused);
	if (!recordCubic)
		return;
	OP_ASSERT(data.size());
	Fodder& back = data.back();
	OP_ASSERT(index < ARRAY_COUNT(back.tests));
	back.tests[index].success = true;
}

#endif
