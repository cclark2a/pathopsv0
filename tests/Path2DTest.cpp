// (c) 2024, Cary Clark cclark2@gmail.com

#include "emscripten/Path2D.h"

bool overrideDebugIt = true;

void outputCommandsArray(std::vector<TwoD::Curve>& commands) {
	std::string s = "[";
	for (TwoD::Curve& curve : commands) {
		s += "[\"" + curve.command + "\", [";
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
	OpDebugOut(s + "\n");
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
	if (debugIt) outputCommandsArray(commands);
	std::string svg = path.toSVG();
	if (debugIt) OpDebugOut(svg + "\n");
	TwoD::Path path2, path3;
	path2.fromCommands(commands);
	path3.fromSVG(svg);
	std::vector<TwoD::Curve> c2 = path2.toCommands();
	std::string s2 = path3.toSVG();
	if (debugIt) outputCommandsArray(c2);
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
	if (debugIt) outputCommandsArray(commands);
//	path2.fromCommands("[['m', 0, 0, 1, 1, 2, 2, 3, 3]]");
	c2 = path.toCommands();
	if (debugIt) outputCommandsArray(c2);

}

#if OP_DEBUG && OP_TINY_TEST
bool OpDebugSkipBreak() {
	return true;
}
#endif
