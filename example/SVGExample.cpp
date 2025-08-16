// (c) 2025, Cary Clark cclark2@gmail.com

#include "port/Path2D.h"

static void svgFrameIntersect(const char* frameSVG, const char* fillSVG) {
	TwoD::FillPath fillPath;
	fillPath.fromSVG(fillSVG);
	TwoD::FramePath framePath;
	framePath.fromSVG(frameSVG);
	framePath.intersect(fillPath);
	std::string svg = framePath.toSVG();
	OpDebugOut(svg + "\n");
}

void SVGExample() {
    const char* frame = "M 50 150 Q 150 50 350 50";
	const char* fill = "M 10 15 L 70 15 L 105 115 L 140 15 L 200 15 L 140 185 L 70 185 L 10 15"
		" M 295 10 Q 390 10 390 100 Q 390 190 295 190 Q 200 190 200 100 Q 200 10 295 10"
		" M 295 55 Q 265 55 265 100 Q 265 145 295 145 Q 325 145 325 100 Q 325 55 295 55 Z";
    svgFrameIntersect(frame, fill);
}
