// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#if TEST_RASTER

#include "OpMath.h"
#include "DebugOpsTypes.h"

struct OpContext;
struct OpContour;
struct OpWinding;
struct DebugRaster;

namespace PathOpsV0Lib {
    void debugRasterAdd(DebugContextData caller, Curve , int parentID);
}

constexpr int compareXY = 64;
constexpr int drawXY = 1000;
constexpr int compareSub = 8;
constexpr int drawSub = 4;

struct RasterSample {
    PathOpsV0Lib::Winding* winding;  // index into debug raster windings
    float x;
	int id;  // edge or segment (for debugging  / something-went-wrong info)
	bool curveDown;
};

typedef std::vector<RasterSample> RasterSamples;

// creates an array of intersections of contour curves and horizontal scanlines
// this permits region-like operations to validate pathops
// one for operands; and one more for comparing combined with output (both stored in contours)
struct OpDebugSamples {
	OpDebugSamples(DebugRaster* );
	void addCurveXatY(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr);
	float compare(OpDebugSamples& );  // return error as sum of partial-x differences
	void sample(OpContour* );
	void sort();

	std::vector<RasterSamples> sampleSet;  // 1 per curve crossing scanline
	DebugRaster* raster;
	double scale;  // apply scale first
	double offsetX;  // then apply offset
	double offsetY;
};

// turns chosen group of debug samples into pixel array that can be visualized
// one per contour; and two in contours for combined and output
struct OpDebugBitmap {
	OpDebugBitmap(DebugRaster* );
	~OpDebugBitmap() {
		free(bits); }
	void rasterize(const OpDebugSamples& , int row);  // sets bits to sample coverage

	DebugRaster* debugRaster;
    uint8_t* bits = nullptr;  // 1 byte per pixel, black/white only
};

struct OpDebugScanLine {
	OpDebugScanLine(DebugRaster* );
	~OpDebugScanLine() {
		free(subScan); }
	void fill(float x, float endX, int y);

	DebugRaster* debugRaster;
    uint8_t* subScan = nullptr;  // 1 byte per pixel (inverse alpha map)
};

enum class RasterType {
	compare,
	draw
};

struct DebugRaster {
	DebugRaster(OpContext* c, RasterType rt) 
		: context(c) 
		, bitWidth(RasterType::compare == rt ? compareXY : drawXY)
		, bitHeight(RasterType::compare == rt ? compareXY : drawXY)
		, subSamples(RasterType::compare == rt ? compareSub : drawSub)
		, inBits(RasterType::compare == rt ? nullptr : this)
		, outBits(RasterType::compare == rt ? nullptr : this)
		, inSamples(this)
		, outSamples(this)
		, rasterType(rt) {
	}
	
    void in();
    float out();
	void sendToDebugger(OpDebugBitmap& bits, OpDebugSamples& samples, std::string );

	OpContext* context;
	int bitWidth;
	int bitHeight;
	int subSamples;
	OpDebugBitmap inBits;
	OpDebugBitmap outBits;
	OpDebugSamples inSamples;
	OpDebugSamples outSamples;
	RasterType rasterType;
};

#endif

#endif
