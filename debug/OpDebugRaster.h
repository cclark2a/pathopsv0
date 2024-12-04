// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#if TEST_RASTER

#include "OpDebug.h"

static constexpr int bitWidth = 64;
static constexpr int bitHeight = 64;

namespace PathOpsV0Lib {
	struct Curve;
	struct Winding;
}

struct OpContours;
struct OpContour;
struct OpWinding;

struct RasterSample {
    OpWinding* winding;  // index into debug raster windings
    float x;
	float y;
	int intY;
	int parentID;  // edge or segment
	bool curveDown;  // or curve right if vertical
	bool vertical;  // true if sample is vertical
};

// creates an array of intersections of contour curves and horizontal scanlines
// this permites region-like operations to validate pathops
// one for operands; and one more for comparing combined with output (both stored in contours)
struct OpDebugSamples {
	void addCurveXatY(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr, bool curveDown = false);
	void addCurveYatX(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr, bool curveRight = false);
	float compare(OpDebugSamples& );  // return error as sum of partial-x differences
	void init(OpContours* );	// call after data is normalized
	void sample(OpContour* libContour);
	void sort();

	std::vector<RasterSample> samples;  // 1 per curve crossing scanline
	OpContours* contours;
	double scale;  // apply scale first
	double offsetX;  // then apply offset
	double offsetY;
};

// turns chosen group of debug samples into pixel array that can be visualized
// one per contour; and two in contours for combined and output
struct OpDebugRaster {
	void rasterize(const OpDebugSamples& , OpContour* contour);  // sets bits to sample coverage
	void fillScanline(float x, float endX, int y);
	void init();

    uint8_t bits[bitWidth * bitHeight];  // 1 byte per pixel, black/white only
	char* data; // for image watch
	int width; 
	int height;
	int stride;
};

#endif

#endif
