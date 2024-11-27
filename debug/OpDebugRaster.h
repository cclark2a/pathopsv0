// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#if TEST_RASTER

#include "OpDebug.h"

namespace PathOpsV0Lib {
	struct Context;
	struct Contour;
	struct Curve;
	struct Winding;
}

struct OpContours;
struct OpContour;

struct OpDebugRaster {
	OpDebugRaster(PathOpsV0Lib::Context* context, PathOpsV0Lib::Contour* contour);
	OpDebugRaster(PathOpsV0Lib::Context* context, const OpDebugRaster& , const OpDebugRaster& );
	int compare(PathOpsV0Lib::Context* );
	void rasterize(PathOpsV0Lib::Contour* contour);

	// internal
	OpDebugRaster() {}
	void addCurve(OpContours* , PathOpsV0Lib::Curve );
	void addWinding(int x, int y);
	void doXor(int x, int y);
	void init(OpContours* );
	void combine(const OpDebugRaster& operand);

	static constexpr int bitWidth = 64;
	static constexpr int bitHeight = 64;
	std::vector<char> windings; // 1 winding data per pixel (caller sets winding size)
	uint8_t bits[bitWidth * bitHeight];  // 1 byte per pixel, black/white only
	bool windInitialized[bitWidth * bitHeight];
	OpContour* contour;  // for winding
	PathOpsV0Lib::Winding* winding;
	double scale;  // apply scale first
	double offsetX;  // then apply offset
	double offsetY;
	int width;  // for image watch
	int height;
	int stride;
	bool curveDown;
	char* data;
	OP_DEBUG_CODE(int pass = 0);
};

#endif

#endif
