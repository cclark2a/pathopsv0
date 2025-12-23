// (c) 2025, Cary Clark cclark2@gmail.com

#include "port/Path2D.h"

void HTMLCanvasExample() {
	TwoD::FillPath path, path2;
	path.rect(0, 0, 4, 4);
	path2.rect(2, 2, 6, 6);
	path.intersect(path2);
	std::vector<TwoD::Curve> commands = path.toCommands();
	std::string s;
	for (TwoD::Curve& curve : commands) {
		s += "[\"" + std::string(1, "MLQKCZ"[(int) curve.type]) + "\", [";
		for (float f : curve.data)
			s += STR(f) + ", ";
		if (!s.empty() && ' ' == s.back())
			s.pop_back();
		if (!s.empty() && ',' == s.back())
			s.pop_back();
		s += "]], ";
	}
	if (!s.empty())
		s.pop_back();
	if (!s.empty())
		s.pop_back();
    OpDebugOut(s + "\n");
}

OP_TINY_MAIN(HTMLCanvasExample)  // main() for cmake
