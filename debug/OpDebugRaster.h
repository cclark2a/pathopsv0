// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpDebugRaster_DEFINED
#define OpDebugRaster_DEFINED

#if OP_TEST_RASTER

#include "OpContext.h"
#include "DebugOpsTypes.h"
#include "PathOpsTypes.h"

struct OpDebugSamples;
struct OpWinding;
struct DebugRaster;

namespace PathOpsV0Lib {
    void debugRasterAdd(DebugContextData caller, Curve , int parentID);
}

constexpr int compareXY = 64;
constexpr int drawXY = 1000;
constexpr int compareSub = 8;
constexpr int drawSub = 4;
#if 0 && OP_DEBUG_SERIALIZE
inline const std::string BitsFile = "DebuggerBits.txt";
#endif

// turns chosen group of debug samples into pixel array that can be visualized
// one per contour; and two in contours for combined and output
struct OpDebugBitmap {
	OpDebugBitmap() {}
	OpDebugBitmap(DebugRaster* );
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel l, DebugBase b) const;
#endif
#if OP_DEBUG_DUMP
	void dumpSet(char const*& str);
	static void DumpSet(char const*& str, uint32_t* pixels);
#endif
	void rasterize(OpDebugSamples& , int row, float sx, float dx);  // sets bits to sample coverage

    std::vector<uint8_t> bits;  // 1 byte per pixel, black/white only
	DebugRaster* raster = nullptr;
};

struct DebugOutput {
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel , DebugBase ) const;
#endif
#if OP_DEBUG_DUMP
	void dumpResolveAll(OpContext* );
	void dumpSet(OpContext* , char const*& str);
#endif
	OpCurve curve;
	OpWinding winding = OpWinding(DebugWindingRaster::dummy);
	PathOpsV0Lib::LoopAttribute loopAttr = PathOpsV0Lib::LoopAttribute::none;
	OpEdge* edge = nullptr;
};

struct RasterSample {
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel l, DebugBase b) const;
#endif
#if OP_DEBUG_DUMP
	void dumpResolveAll(OpContext* );
	void dumpSet(char const*& str);
#endif
	const OpWinding& winding() const;

	OpContour* contour = nullptr;  // if set: represents original curve (segment/edge are nullptr)
	OpSegment* segment = nullptr;  // if set: contour and edge are nullptr
	OpEdge* edge = nullptr;  // if set: contour and segment are nullptr
	int curveIndex = -1;  // for contour: index of user-provided curve
    float x  OP_DEBUG_INIT_FLOAT();
	float t  OP_DEBUG_INIT_FLOAT();  // unused by rasterizer, but useful for debugging
	bool curveDown = false;  // unused by contour curve (don't use int8_t; confuses compare diff)
	bool visible = true;
};

typedef std::vector<RasterSample> RasterSamples;
constexpr auto OpDoubleNaN = std::numeric_limits<double>::quiet_NaN();

#define SampleType_Enums \
	OP_ENUM_MEMBER(none),  \
	OP_ENUM_MEMBER(contourInput), \
	OP_ENUM_MEMBER(contourResolved), \
	OP_ENUM_MEMBER(segmentInput), \
	OP_ENUM_MEMBER(segmentResolved), \
	OP_ENUM_MEMBER(edges), \
	OP_ENUM_MEMBER(output)

enum class SampleType {
	SampleType_Enums
};

// creates an array of intersections of contour curves and horizontal scanlines
// this permits region-like operations to validate pathops
// one for operands; and one more for comparing combined with output (both stored in contours)
struct OpDebugSamples {
	OpDebugSamples() 
		: zeroWinding(DebugWindingRaster::dummy)
		, winding(DebugWindingRaster::dummy) {};
	OpDebugSamples(DebugRaster* );
	void addCurveXatY(const PathOpsV0Lib::Curve& , RasterSample& base, float tLo, float tHi);
	void addCurveXatY(OpContour* , int debugCurveIndex,
			const OpCurve& opCurve, std::vector<float>& extrema);
	void addCurveXatY(OpSegment* );
	void addCurveXatY(OpEdge* );
	bool alwaysVisible() const {
		return SampleType::edges == sampleType || SampleType::output == sampleType; }
	float compare(std::vector<RasterSamples>& );  // return error as sum of partial-x differences
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel l, DebugBase b) const;
#endif
#if OP_DEBUG_DUMP
	void dumpResolveAll(OpContext* );
	void dumpSet(char const*& str);
#endif
	void rasterize();
//	void resetAdd();
	void sample(OpContour* );
	void sample(DebugOutput& );
	void sort();
	PathOpsV0Lib::WindingKeep visibleFunc() const;

	OpWinding zeroWinding;
	OpWinding winding;
	DebugRaster* raster = nullptr;
	std::vector<RasterSamples> sampleSet;  // 1 per curve crossing scanline
	OpDebugBitmap mask;
	SampleType sampleType = SampleType::none;
	// the following is not serialized
#if 0
	int firstRow = INT_MAX;  // used to prevent double hits on first and last curve
	int lastRow = INT_MAX; // used to prevent double hits on consecutive curves
	bool firstDown = false;	// initial curve direction
	bool lastDown = false; // prior curve direction
	bool lastCurve = false;  // set when final curve of closed loop is added
#endif
};

struct OpDebugScanLine {
	OpDebugScanLine(DebugRaster* );
	void fill(float x, float endX, int y);

    std::vector<uint8_t> subScan;  // 1 byte per pixel (inverse alpha map)
	DebugRaster* raster = nullptr;
};

struct DebugRaster {
	DebugRaster(OpContext* c) 
		: context(c) 
		, bitWidth(compareXY + 2)
		, bitHeight(compareXY + 2)
		, subSamples(compareSub)
#if OP_DEBUG || OP_DEBUGGER
		, sendToDebugger(context->debugData.runOneFile)
#endif
		{
		context->debugRaster = this;
		float scaleX = compareXY / context->maxBounds.width();
		float scaleY = compareXY / context->maxBounds.height();
		scale = std::min(scaleX, scaleY);
		offsetX = -context->maxBounds.left * scale + 1;
		offsetY = (-context->maxBounds.top * scale + 1) * subSamples;
	}
	
	void addOutput(PathOpsV0Lib::Output , OpEdge* );
    void in();
    float out();
	void sample(SampleType );
	void sampleEdges();
	void sampleOutput();
	bool tooSmall() const;
	OP_DEBUG_VALIDATE_CODE(void validate();)
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel l, DebugBase b) const;
//	void deleteOld();
//	void dumpSample(const OpDebugSamples& , std::string description);
#endif
#if OP_DEBUG_DUMP || OP_DEBUGGER
//	bool playback(std::string filename);
#endif
#if OP_DEBUG_DUMP
	void dumpResolveAll(OpContext* );
	void dumpSet(char const*& str);
#endif
#if OP_DEBUG_SERIALIZE || OP_DEBUGGER
//	void record(std::string filename);
#endif
	std::vector<OpDebugSamples> sampleSets;  // one per initial winding value
	std::vector<DebugOutput> outputs;
	OpContext* context = nullptr;
	double scale = OpDoubleNaN;  // apply scale first
	double offsetX = OpDoubleNaN;  // then apply offset
	double offsetY = OpDoubleNaN;
	int bitWidth = -1;
	int bitHeight = -1;
	int subSamples = -1;
#if OP_DEBUG || OP_DEBUGGER
	int8_t sendToDebugger = -1;
#endif
	OP_DEBUG_VALIDATE_CODE(bool disableValidate = true;)
};

#endif

#endif
