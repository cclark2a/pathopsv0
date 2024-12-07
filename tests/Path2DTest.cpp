// (c) 2024, Cary Clark cclark2@gmail.com

#include "emscripten/Path2D.h"

bool overrideDebugIt = false;

static std::string typeToCommand(TwoD::Types type) { 
	return std::string(1, "MLQCZ"[(int) type]); 
}

static std::string commandsArray(std::vector<TwoD::Curve>& commands) {
	std::string s = "[";
	for (TwoD::Curve& curve : commands) {
		s += "[\"" + typeToCommand(curve.type) + "\", [";
		for (float f : curve.data)
			s += STR(f) + ", ";
		if (' ' == s.back())
			s.pop_back();
		if (',' == s.back())
			s.pop_back();
		s += "]], ";
	}
	if (' ' == s.back())
		s.pop_back();
	if (',' == s.back())
		s.pop_back();
	s += "]";
	return s;
}

void TestPath2D(bool debugIt) {
	debugIt |= overrideDebugIt;
	TwoD::Path path;
	path.moveTo(1, 2);
	path.lineTo(3, 4);
	path.quadraticCurveTo(5, 6, 7, 8);
	path.lineTo(9, 10);
	path.closePath();
	path.lineTo(11, 12);
	path.closePath();
	path.quadraticCurveTo(13, 14, 15, 16);
	path.bezierCurveTo(17, 18, 19, 20, 21, 22);
	std::vector<TwoD::Curve> commands = path.toCommands();
	std::string cmdsStr = commandsArray(commands);
	if (debugIt) OpDebugOut(cmdsStr + "\n");
	std::string svg = path.toSVG();
	if (debugIt) OpDebugOut(svg + "\n");
	TwoD::Path path2, path3;
	path2.fromCommands(commands);
	path3.fromSVG(svg);
	std::vector<TwoD::Curve> c2 = path2.toCommands();
	std::string s2 = path3.toSVG();
	std::string cmds2Str = commandsArray(c2);
	if (debugIt) OpDebugOut(cmds2Str + "\n");
	OP_ASSERT(cmdsStr == cmds2Str);
	if (debugIt) OpDebugOut(s2 + "\n");
	OP_ASSERT(svg == s2);
	path.intersect(path2);
	if (debugIt) OpDebugOut("path:" + path.toSVG() + "\n");
	path.fromSVG("M1 0 Q2 0 2 1 Q2 2 1 2 Q0 2 0 1 Q0 0 1 0");
	if (debugIt) OpDebugOut("path:" + path.toSVG() + "\n");
	path2.fromSVG("L2 2 L0 2 Z");
	if (debugIt) OpDebugOut("path2:" + path2.toSVG() + "\n");
	path.intersect(path2);
	if (debugIt) OpDebugOut("path:" + path.toSVG() + "\n");
	path.fromSVG("m0 0 1 1 2 2 3 3");
	if (debugIt) OpDebugOut("path:" + path.toSVG() + "\n");
	commands = path.toCommands();
	cmdsStr = commandsArray(commands);
	if (debugIt) OpDebugOut(cmdsStr + "\n");
//	path2.fromCommands("[['m', 0, 0, 1, 1, 2, 2, 3, 3]]");
	c2 = path.toCommands();
	cmds2Str = commandsArray(c2);
	if (debugIt) OpDebugOut(cmds2Str + "\n");
	OP_ASSERT(cmdsStr == cmds2Str);
	path.fromSVG("M 1 1 Q 2 2 3 3 T 5 5 7 7 Z");
	if (debugIt) OpDebugOut("path w/T:" + path.toSVG() + "\n");
	path.fromSVG("M 1 1 C 2 2 3 3 6 6 S 12 12 15 15 Z");
	if (debugIt) OpDebugOut("path w/S:" + path.toSVG() + "\n");
	path.fromSVG("M 1 1 h 1 v 1 h -1 v -1 z");
	if (debugIt) OpDebugOut("path box:" + path.toSVG() + "\n");
	path2.fromSVG("M 1 1 H 2 V 2 H 1 V 1 z");
	if (debugIt) OpDebugOut("path box:" + path2.toSVG() + "\n");
	svg = path.toSVG();
	s2 = path2.toSVG();
	OP_ASSERT(svg == s2);
	// editor tests
	path.clear();
	path.lineTo(3, 4);
	OP_ASSERT(1 == path.curveCount());
	TwoD::Curve lineCurve = path.getCurve(0, false);
	path2.clear();
	path2.quadraticCurveTo(5, 6, 7, 8);
	path3 = path2.clone();
	path3.insertPath(0, path);
	path.addPath(path2);
	OP_ASSERT(2 == path.curveCount());
	TwoD::Curve quadCurve = path.getCurve(1, true);
	svg = path.toSVG();
	std::string s3 = path3.toSVG();
	OP_ASSERT(svg == s3);
	OP_ASSERT(3 == path.pointCount());
	path.eraseRange(0, 1);
	svg = path.toSVG();
	s2 = path2.toSVG();
	OP_ASSERT(svg == s2);
	path3.eraseRange(1, 2);
	path.setCurve(0, lineCurve);
	svg = path.toSVG();
	s3 = path3.toSVG();
	OP_ASSERT(svg == s3);
	OP_ASSERT(1 == path.pointCount());
	path.addPath(path2);
	OP_ASSERT(3 == path.pointCount());
	path.setPoint(1, { -1, -2 });
	OpPoint pt = path.getPoint(1);
	OP_ASSERT(-1 == pt.x);
	OP_ASSERT(-2 == pt.y);
	quadCurve = path.getCurve(1, true);
	float expected[6] = { 3, 4, -1, -2, 7, 8 };
	OP_ASSERT(!std::memcmp(&quadCurve.data.front(), expected, sizeof expected));
}

#if OP_DEBUG && OP_TINY_TEST
bool OpDebugSkipBreak() {
	return true;
}
#endif