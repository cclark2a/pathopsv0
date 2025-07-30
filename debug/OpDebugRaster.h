// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#include "OpTestDrive.h"

#if TEST_RASTER

#include "OpMath.h"
#include "DebugOpsTypes.h"

// add: divisions per scanline

namespace PathOpsV0Lib {
    void debugRasterAdd(DebugContextData caller, Curve , int parentID);
}

struct OpContext;
struct OpContour;
struct OpWinding;

struct RasterSample {
    PathOpsV0Lib::Winding* winding;  // index into debug raster windings
    float x;
	float y;
	int parentID;  // edge or segment (for debugging  / something-went-wrong info)
	bool curveDown;  // or curve right if vertical
//	bool vertical;  // true if sample is vertical
};

// creates an array of intersections of contour curves and horizontal scanlines
// this permites region-like operations to validate pathops
// one for operands; and one more for comparing combined with output (both stored in contours)
struct OpDebugSamples {
	void addCurveXatY(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr);
//	void addCurveYatX(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr, bool curveRight = false);
	float compare(OpDebugSamples& );  // return error as sum of partial-x differences
	void init(OpContext* , int scanline, bool callKeep);	// call after data is normalized
	void sample(OpContour* );
	void sort();

    static constexpr int subSamples = 8;
	std::vector<RasterSample> sampleSet[subSamples];  // 1 per curve crossing scanline
	OpContext* context;
	double scale;  // apply scale first
	double offsetX;  // then apply offset
	double offsetY;
    bool callKeep;
};

// turns chosen group of debug samples into pixel array that can be visualized
// one per contour; and two in contours for combined and output
struct OpDebugRaster {
    void compare(OpDebugRaster& );
	void init();
	void rasterize(const OpDebugSamples& , int scanLine);  // sets bits to sample coverage

    static constexpr int bitWidth = 64;
    static constexpr int bitHeight = 64;
    uint8_t bits[bitWidth * bitHeight];  // 1 byte per pixel, black/white only
	char* data; // for image watch
	int width; 
	int height;
	int stride;
};

struct DebugRaster {
    void in(PathOpsV0Lib::Context*  );
    void out(PathOpsV0Lib::Context*  );

	OpDebugSamples inSamples;
	OpDebugSamples outSamples;
};

#endif

#endif
