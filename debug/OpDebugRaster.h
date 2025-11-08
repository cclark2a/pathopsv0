// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#if TEST_RASTER

#include "OpContext.h"
#include "DebugOpsTypes.h"

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
inline const std::string BitsFile = "DebuggerBits.txt";

enum class SampleType {
	in,
	out
};

struct RasterSample {
    PathOpsV0Lib::Winding* winding = nullptr;  // index into debug raster windings
    float x = OpDebugNaN;
	int id = -1;  // edge or segment (for debugging  / something-went-wrong info)
	bool curveDown = (bool) -1;
};

typedef std::vector<RasterSample> RasterSamples;
constexpr auto OpDoubleNaN = std::numeric_limits<double>::quiet_NaN();

// creates an array of intersections of contour curves and horizontal scanlines
// this permits region-like operations to validate pathops
// one for operands; and one more for comparing combined with output (both stored in contours)
struct OpDebugSamples {
	OpDebugSamples(DebugRaster* );
	void addCurveXatY(PathOpsV0Lib::Curve , int id, OpWinding* w = nullptr);
	float compare(OpDebugSamples& );  // return error as sum of partial-x differences
	void sample(OpContour* , SampleType );
	void sort();

	std::vector<RasterSamples> sampleSet;  // 1 per curve crossing scanline
	DebugRaster* raster = nullptr;
	double scale = OpDoubleNaN;  // apply scale first
	double offsetX = OpDoubleNaN;  // then apply offset
	double offsetY = OpDoubleNaN;
};

// turns chosen group of debug samples into pixel array that can be visualized
// one per contour; and two in contours for combined and output
struct OpDebugBitmap {
	OpDebugBitmap(DebugRaster* );
	~OpDebugBitmap() {
		free(bits); }
	void rasterize(const OpDebugSamples& , int row);  // sets bits to sample coverage

	DebugRaster* debugRaster = nullptr;
    uint8_t* bits = nullptr;  // 1 byte per pixel, black/white only
	size_t size = 0;
};

struct OpDebugScanLine {
	OpDebugScanLine(DebugRaster* );
	~OpDebugScanLine() {
		free(subScan); }
	void fill(float x, float endX, int y);

	DebugRaster* debugRaster = nullptr;
    uint8_t* subScan = nullptr;  // 1 byte per pixel (inverse alpha map)
};

enum class RasterType {
	none,
	compare,
	draw
};

struct DebugRaster {
	DebugRaster(OpContext* c, RasterType rt) 
		: context(c) 
		, bitWidth(RasterType::compare == rt ? compareXY : drawXY)
		, bitHeight(RasterType::compare == rt ? compareXY : drawXY)
		, subSamples(RasterType::compare == rt ? compareSub : drawSub)
		, sendToDebugger(context->debugData.runOneFile)
		, makeBits(RasterType::draw == rt || sendToDebugger)
		, inBits(makeBits ? this : nullptr)
		, outBits(makeBits ? this : nullptr)
		, inSamples(this)
		, outSamples(this)
		, rasterType(rt) {
	}
	
    void in();
    float out();
	void dmp(OpDebugBitmap& bits, OpDebugSamples& samples, std::string );

	OpContext* context = nullptr;
	int bitWidth = -1;
	int bitHeight = -1;
	int subSamples = -1;
	bool sendToDebugger = (bool) -1;
	bool makeBits = (bool) -1;
	OpDebugBitmap inBits;
	OpDebugBitmap outBits;
	OpDebugSamples inSamples;
	OpDebugSamples outSamples;
	RasterType rasterType = RasterType::none;
};

#endif

#endif
