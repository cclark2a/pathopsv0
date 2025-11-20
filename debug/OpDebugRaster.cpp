// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#if TEST_RASTER

#include "OpDebugRaster.h"
#include "OpContext.h"
#include "OpSegment.h"
#include "DebugOps.h"

#if 0
namespace PathOpsV0Lib {

void debugRasterAdd(DebugContextData caller, Curve curve, int parentID) {
    OP_ASSERT(sizeof(OpDebugSamples*) == caller.size);
    OpDebugSamples* samples = (OpDebugSamples*) caller.data;
    samples->addCurveXatY(curve, parentID);
}

}
#endif

using namespace PathOpsV0Lib;

void OpCurve::debugScale(double scale, double offsetX, double offsetY) {
	context().debugCallback(c).scaleFuncPtr(c, scale, offsetX, offsetY);
}

std::string RasterSample::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	ASSERT_FIRST(contour);
	DEBUG_DUMP_OPTIONAL_COMMON_ID(contour);
	DEBUG_DUMP_OPTIONAL_ID(contour, segment);
	DEBUG_DUMP_OPTIONAL_ID(segment, edge);
	DEBUG_DUMP_OPTIONAL_POS_VALUE(edge, curveIndex);
	DEBUG_DUMP_OPTIONAL_FINITE_VALUE(curveIndex, x);
	DEBUG_DUMP_BOOL(x, curveDown);
	ASSERT_LAST_OFFSET(curveDown, 7);
	return s;
}

void RasterSample::dumpSet(char const*& str) {
	ASSERT_FIRST(contour);
	DEBUG_SET_OPTIONAL_COMMON_ID(contour);
	DEBUG_SET_OPTIONAL_ID(contour, segment);
	DEBUG_SET_OPTIONAL_ID(segment, edge);
	DEBUG_SET_OPTIONAL_VALUE(edge, curveIndex);
	DEBUG_SET_OPTIONAL_FINITE_VALUE(curveIndex, x);
	DEBUG_SET_BOOL(x, curveDown);
	ASSERT_LAST_OFFSET(curveDown, 7);
	curveDown = OpDebugOptional(str, "curveDown");
}

const OpWinding& RasterSample::winding() const {
	if (contour) {
		OP_ASSERT(!segment);
		OP_ASSERT(!edge);
		return contour->debugWinding;
	}
	if (segment) {
		OP_ASSERT(!edge);
		return segment->winding;
	}
	OP_ASSERT(edge);
	return edge->winding;
}

OpDebugSamples::OpDebugSamples(DebugRaster* r) 
	: zeroWinding(r->context, DebugWindingZero::dummy)
	, winding(WindingUninitialized::dummy)
	, raster(r)
	, mask(r) {
	sampleSet.resize(r->bitHeight * r->subSamples);
//    OpDebugOut("  scanline:" + STR(scanline) + " offsetY:" + STR(offsetY) + "\n");
}

// advance 1/8th (or whatever sub samples is set to)
void OpDebugSamples::addCurveXatY(const Curve& original, RasterSample& base, float tLo, float tHi) {
	int rows = raster->bitHeight * raster->subSamples;
	auto toSubGrid = [this](float x) {
		return std::floorf(x * raster->subSamples) / raster->subSamples;
	};
	OpCurve curve(original, Rotated::no);
	curve.debugScale(raster->scale, raster->offsetX, raster->offsetY);
	OpPoint xy = curve.ptAtT(tLo);
	OpPoint xyEnd = curve.ptAtT(tHi);
    bool curveDown = xy.y <= xyEnd.y;
	if (!curveDown)
		std::swap(xy, xyEnd);
	float y = toSubGrid(xy.y);
	float yEnd = toSubGrid(xyEnd.y);
	y = std::max(0.f, y);
	yEnd = std::min((float) raster->bitHeight, yEnd);
    int row = (int) std::ceil(y * raster->subSamples);
	while (y <= yEnd) {
		float t = curve.tAtXY(tLo, tHi, XyChoice::inY, y);
		float x = curve.ptAtT(t).x;
		if (x < raster->bitWidth) {
//			OpDebugOut("id:" + STR(id) + " x:" + STR(x) + " y:" + STR(y) + " row:" + STR(row) + "\n");
			x = std::max(0.f, x);
            OP_ASSERT(row < rows);
			OP_ASSERT(row < sampleSet.size());
            std::vector<RasterSample>& samples = sampleSet[row];
			OP_ASSERT(0 <= x && x < raster->bitWidth);
			base.x = x;
			base.curveDown = curveDown;
			samples.push_back(base);
		}
        ++row;
		y = (float) row * raster->bitHeight / (float) rows;
	}
}

void OpDebugSamples::addCurveXatY(const OpContour* contour, int debugCurveIndex) {
	int baseIndex = debugCurveIndex;
	OP_ASSERT(baseIndex < contour->debugContourData.size());
	const DebugContourData& curveData = contour->debugContourData[baseIndex++];
	OP_ASSERT(DebugContourType::curveData == curveData.type);
	OP_ASSERT(baseIndex < contour->debugContourData.size());
	const DebugContourData& curveType = contour->debugContourData[baseIndex++];
	OP_ASSERT(DebugContourType::curveType == curveType.type);
	OP_ASSERT(sizeof(CurveType) == curveType.size);
	const Curve curve { (ContextPtr) contour->context, (CurveData*) curveData.data, curveData.size,
			*(CurveType*) curveType.data }; 
	OP_ASSERT(baseIndex < contour->debugContourData.size());
	const DebugContourData& curveExtrema = contour->debugContourData[baseIndex++];
	OP_ASSERT(DebugContourType::curveExtrema == curveExtrema.type);
	OpRoots tValues;
	float* extrema = (float*) curveExtrema.data;
	size_t count = curveExtrema.size / sizeof(float);
	for (size_t index = 0; index < count; ++index) {
		tValues.add(extrema[index]);
	}
	tValues.add(0);
	tValues.add(1);
	tValues.sort();
    for (int index = 0; index < tValues.count() - 1; ++index) {
		RasterSample base { contour, nullptr, nullptr, debugCurveIndex };
		addCurveXatY(curve, base, tValues.roots[index], tValues.roots[index + 1]);
	}
}

void OpDebugSamples::addCurveXatY(const OpSegment* segment) {
	const Curve& curve = segment->c.c;
	RasterSample base { nullptr, segment };
	addCurveXatY(curve, base, 0, 1);
}

void OpDebugSamples::addCurveXatY(const OpEdge* edge) {
	const Curve& curve = edge->curve.c;
	RasterSample base { nullptr, nullptr, edge };
	addCurveXatY(curve, base, 0, 1);
}

static float NextX(const std::vector<RasterSample>& samples, size_t& index, OpWinding& winding) {
	OP_ASSERT(index < samples.size());
	float x = samples[index].x;
	do {
		const RasterSample& next = samples[index];
		if (x != next.x)
			break;
		const OpWinding& w = next.winding();
		if (next.curveDown)
			winding.add(w);	// copy-on-demand
		else
			winding.subtract(w);
		++index;
	} while (index < samples.size());
	return x;
}

static float NextVisible(const OpDebugSamples& sampleSet, OpWinding& sum, size_t& index, 
		std::vector<RasterSample>& samples, WindKeep keeper) {
	OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
	float x = OpNaN;
	OpWinding winding(sampleSet.zeroWinding);  // shallow zeroed winding for this x value
	while (index < samples.size()) {  // accumulate winding of start
		size_t firstIndex = index;
		x = NextX(samples, index, winding);
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
		sum.add(winding);
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
		// choose between 'visible' and 'keep'
		// use 'visible' to say if source path shows
		// use 'keep' to say if computed path shows
		if (sampleSet.alwaysVisible())
			break;
		WindKeep keep = (*sampleSet.visibleFunc())(winding.w, sum.w);
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
		if (keeper == keep)
			break;
		OP_ASSERT(firstIndex < index);
		while (firstIndex < index) {
			samples[firstIndex++].visible = false;
		}
		OP_ASSERT(WindKeep::Discard == keep);
		x = OpNaN;
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
	}
	OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
	return x;
}

struct XRange {
	float start = 0;
	float end = 0;
};

float OpDebugSamples::compare(std::vector<RasterSamples>& outputs) {
    OpContext* context = raster->context;
	int rows = raster->subSamples * raster->bitHeight;
	float error = 0;
    for (int row = 0; row < rows; ++row) {
        std::vector<RasterSample>& subS = sampleSet[row];
        std::vector<RasterSample>& subO = outputs[row];
	    size_t inIndex = 0;
	    size_t outIndex = 0;
		OpWinding sum(zeroWinding);
		// build spans of coverage from 'in' samples, using winding
		// !!! this needs work to function with frames instead of fills
		//     put that task off until after fills work
		//     of concern: how to distinguish horizontal frames from frames with two intersections
		do {
			if (OpMath::IsNaN(NextVisible(*this, sum, inIndex, subS, WindKeep::Start)))
				break;
			if (OpMath::IsNaN(NextVisible(*this, sum, inIndex, subS, WindKeep::End)))
				break;
		} while (inIndex < subS.size());
		// note that 'out' samples do not have or need winding; they implicitly describe coverage
		auto advance = [this](const std::vector<RasterSample>& samples, size_t& index) {
			while (index < samples.size()) {
				const RasterSample& sample = samples[index];
				++index;
				if (sample.visible)
						return sample.x;
			}
			return (float) raster->bitWidth;
		};
		inIndex = 0;
		XRange inXs { advance(subS, inIndex), advance(subS, inIndex) }; 
		outIndex = 0;
		XRange outXs { advance(subO, outIndex), advance(subO, outIndex) }; 
		float x = std::min(inXs.start, outXs.start);
		auto nextX = [&error, &x](const XRange& upper, const XRange& lower) {
			float xEnd = std::min(upper.start, lower.end);
			OP_ASSERT(x < xEnd);
			error += xEnd - x;
			x = xEnd;
		};
		while (x < raster->bitWidth) {
			if (x < inXs.start)
				nextX(inXs, outXs);
			else if (x < outXs.start)
				nextX(outXs, inXs);
			else
				x = std::min(inXs.end, outXs.end);
			if (x >= inXs.end)
				inXs = { advance(subS, inIndex), advance(subS, inIndex) };
			if (x >= outXs.end)
				outXs = { advance(subO, outIndex), advance(subO, outIndex) };
		}
    }
	if (error >= 9) {
	#if OP_DEBUG_FAST_TEST
		std::lock_guard<std::mutex> guard(out_mutex);
	#endif
	    std::string testname = context->debugData.testname;
	    OpDebugOut(testname + " raster errors:" + STR(error) + "\n");
    }
	return error;
}

void OpDebugSamples::sample(OpContour* contour) {
	OP_ASSERT(WindingType::uninitialized != contour->debugWinding.type);
	if (SampleType::contourInput == sampleType || SampleType::contourResolved == sampleType) {
		for (int index = 0; index < (int) contour->debugContourData.size(); ++index) {
			if (DebugContourType::curveData == contour->debugContourData[index].type)
				addCurveXatY(contour, index);
		}
		return;
	}
	for (OpSegment& segment : contour->segments) {
		if (SampleType::segmentInput == sampleType || SampleType::segmentResolved == sampleType)
			addCurveXatY(&segment);
		else {
			OP_ASSERT(SampleType::edgeOutput == sampleType);
			for (OpEdge& edge : segment.edges) {
				if (edge.inOutput)
					addCurveXatY(&edge);
			}
		}
	}
}

void OpDebugSamples::rasterize() {
    for (int row = 0; row < raster->bitHeight; ++row) {
		mask.rasterize(*this, row);
	}
}

static void RasterSamplesSort(std::vector<RasterSamples>& sampleSet) {
    for (std::vector<RasterSample>& samps : sampleSet) {       
	    std::sort(samps.begin(), samps.end(), [](const RasterSample& a, const RasterSample& b) {
		    return a.x < b.x;
	    });
    }
}

void OpDebugSamples::sort() {
	RasterSamplesSort(sampleSet);
}

	// segment / contour input : check visible
	//     use debug-only callout in winding that returns if sum of edges is
	//     visible for one contour; e.g., for 2 operand resolve, is the sum visible in
	//     the 'left' operand, when the contour input is 'left'... (for additive winding)
	// segment / contour resolved : check keep
	// edge out : no check needed
WindingKeep OpDebugSamples::visibleFunc() const {
	OpContext* context = raster->context;
	WindingKeep visibleFunc = context->debugContextCallbacks.debugWindingVisibleFuncPtr;
	if (!visibleFunc || (SampleType::contourInput != sampleType 
			&& SampleType::segmentInput != sampleType))
		visibleFunc = context->windingCallbacks.windingKeepFuncPtr;
	return visibleFunc;
}

#define SampleType_Base
#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { SampleType::w, #w }
ENUM_NAME_STRUCT(SampleType)

std::string OpDebugSamples::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	OP_ASSERT(raster);
	ASSERT_FIRST(zeroWinding);
	DEBUG_DUMP_COMMON_STRUCT(zeroWinding);
	DEBUG_DUMP_STRUCT(zeroWinding, winding);
	ASSERT_ORDERED(winding, raster);
	ASSERT_ORDERED(raster, sampleSet);
	s += "size:" + STR(sampleSet.size()) + "\n";
	for (const auto& sample : sampleSet) {
		if (!sample.size())
			continue;
		s += "sample:" + STR(sample.size()) + " "; 
		int mPerLine = 0;
		for (const auto& member : sample) { 
			s += member.debugDump(l, b) + (++mPerLine % 4 ? " " : "\n"); 
		} 
		s.pop_back();
		s += "\n";
	}
	DEBUG_DUMP_STRUCT(sampleSet, mask);
	ASSERT_ORDERED(mask, sampleType);
    s += "sampleType:" + SampleTypeName(sampleType) + " ";
	ASSERT_LAST_OFFSET(sampleType, 4);
	return s;
}

void OpDebugSamples::dumpSet(char const*& str) {
	OP_ASSERT(raster);
	ASSERT_FIRST(zeroWinding);
	DEBUG_SET_COMMON_STRUCT(zeroWinding);
	DEBUG_SET_STRUCT(zeroWinding, winding);
	ASSERT_ORDERED(winding, raster);
	ASSERT_ORDERED(raster, sampleSet);
	OpDebugRequired(str, "size");
	size_t size = OpDebugReadSizeT(str);
	sampleSet.resize(size);
	for (auto& sample : sampleSet) {
		DEBUG_SET_COMMON_VECTOR(sample);
	}
	DEBUG_SET_STRUCT(sampleSet, mask);
	ASSERT_ORDERED(mask, sampleType);
    sampleType = SampleTypeStr(str, "sampleType", SampleType::none);
	ASSERT_LAST_OFFSET(sampleType, 4);
}

OpDebugScanLine::OpDebugScanLine(DebugRaster* r) {
	raster = r;
	int size = r->bitWidth * r->subSamples;
	subScan.resize(size);
	std::fill(subScan.begin(), subScan.end(), 0);
}

void OpDebugScanLine::fill(float x, float endX, int subLine) {
    OP_ASSERT(0 <= subLine && subLine < raster->subSamples);
	if (x >= endX)
		return;
	int intX = (int) x;
    OP_ASSERT(0 <= intX && intX < raster->bitWidth);
	OP_DEBUG_CODE(uint8_t* limit = &subScan[(subLine + 1) * raster->bitWidth]);
	uint8_t* bitsPtr = &subScan[subLine * raster->bitWidth + intX];
	int intEndX = (int) endX;
	if (intX == intEndX) {
		OP_ASSERT(bitsPtr < limit);
		bitsPtr[0] = std::min(255, bitsPtr[0] + (int) ((endX - x) * 255));
		return;
	}
	if (x != intX) {
		OP_ASSERT(bitsPtr < limit);
		bitsPtr[0] = std::min(255, bitsPtr[0] + (int) ((intX + 1 - x) * 255));
		++bitsPtr;
	}
	while (++intX < intEndX) {
		OP_ASSERT(bitsPtr < limit);
		*bitsPtr++ = 255;
	}
	if (intEndX < endX) {
		OP_ASSERT(bitsPtr < limit);
		bitsPtr[0] = (int) ((endX - intEndX) * 255);
	}
}

OpDebugBitmap::OpDebugBitmap(DebugRaster* r)
	: raster(r) {
	if (!r)
		return;
	bits.resize(r->bitWidth * r->bitHeight);
	std::fill(bits.begin(), bits.end(), 0);
}

void OpDebugBitmap::rasterize(OpDebugSamples& sampleSet, int row) {
	OP_DEBUG_VALIDATE_CODE(raster->validate());
    OpDebugScanLine subScan(raster);
	int scanLine = row * raster->subSamples;
//	OP_ASSERT(12 != row || SampleType::contourResolved != sampleSet.sampleType);
	for (int subLine = 0; subLine < raster->subSamples; ++subLine) {
		std::vector<RasterSample>& samples = sampleSet.sampleSet[scanLine + subLine];
		if (samples.empty())
			continue;
		size_t index = 0;
		OP_DEBUG_VALIDATE_CODE(raster->validate());
		OpWinding sum(sampleSet.zeroWinding);  // shallow zero winding
		// zero winding
			// accumulate winding while x is unchanged
		// add accumulated winding to sum 
		// call keep with winding, sum
		do {
			OP_DEBUG_VALIDATE_CODE(raster->validate());
			float startX = NextVisible(sampleSet, sum, index, samples, WindKeep::Start);
			OP_DEBUG_VALIDATE_CODE(raster->validate());
			if (OpMath::IsNaN(startX))
				break;
			float endX = NextVisible(sampleSet, sum, index, samples, WindKeep::End);
			if (OpMath::IsNaN(endX))
				endX = (float) raster->bitWidth;
			subScan.fill(startX, endX, subLine);
			OP_DEBUG_VALIDATE_CODE(raster->validate());
		} while (index < samples.size());
	}
    for (int x = 0; x < raster->bitWidth; ++x) {
        int pixel = 0;
        for (int y = 0; y < raster->subSamples; ++y) {
            pixel += subScan.subScan[y * raster->bitWidth + x];
        }
		size_t bitOffset = row * raster->bitWidth + x;
		OP_ASSERT(bitOffset < bits.size());
        bits[bitOffset] = pixel / raster->subSamples;
    }
    OpNop();
}

std::string OpDebugBitmap::debugDump(DebugLevel l, DebugBase b) const {
	OP_ASSERT(raster);
	std::string s;
	s += "size:" + STR(bits.size()) + "\n";
	OP_ASSERT(bits.size() == raster->bitHeight * raster->bitWidth);
	for (int y = 0; y < raster->bitHeight; ++y) {
		for (int x = 0; x < raster->bitWidth; ++x) {
			std::string hex = OpDebugByteToHex(bits[y * raster->bitWidth + x]);
			s += hex[2];
			s += hex[3];
		}
		s += "\n";
	}
	return s;
}

void OpDebugBitmap::dumpSet(char const*& str) {
	OP_ASSERT(raster);
	OpDebugRequired(str, "size");
	bits.resize(OpDebugReadSizeT(str));
	OP_ASSERT(bits.size() == raster->bitHeight * raster->bitWidth);
	char hexStr[5] { '0', 'x', '?', '?', '\0' };
	for (int y = 0; y < raster->bitHeight; ++y) {
		for (int x = 0; x < raster->bitWidth; ++x) {
			hexStr[2] = *str++; 
			hexStr[3] = *str++;
			const char* hexPtr = hexStr;
			bits[y * raster->bitWidth + x] = OpDebugByteToInt(hexPtr);
		}
		OpDebugExitOnFail("missing \\n", '\n' == *str++);
	}
	OpNop();
}

void DebugRaster::in() {
	sample(SampleType::contourResolved);
	validate();
	if (sendToDebugger) {
		sample(SampleType::contourInput);
		sample(SampleType::segmentInput);
		sample(SampleType::segmentResolved);
	}
	for (auto& s : samples) {
		s.sort();
	}
	if (!sendToDebugger)
		return;
	validate();
	for (auto& s : samples) {
		s.rasterize();
	}
	OpNop();
}

float DebugRaster::out() {
	sampleEdges();
	float result = 0;
	OP_ASSERT(samples.size());
	OpDebugSamples& output = samples.back();
	output.sort();
	if (sendToDebugger)
		output.rasterize();
	OP_ASSERT(SampleType::edgeOutput == output.sampleType);
	OpDebugSamples& allContours = samples[0];
	OP_ASSERT(SampleType::contourResolved == allContours.sampleType);
	result = allContours.compare(output.sampleSet);
	if (sendToDebugger)
		record(BitsFile);
	OpNop();
	return result;
}

bool DebugRaster::playback(std::string filename) {
    std::string buffer = dmpFileToStr(filename);
    if (buffer.empty())
        return false;
    const char* str = buffer.c_str();
	dumpSet(str);
	return true;
}

void DebugRaster::record(std::string name) {
    std::string filename = dmpFileToPath(name);
    FILE* file = fopen(filename.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filename + " to write\n");
        return;
    }
	std::string s = debugDump(DebugLevel::file, DebugBase::hex);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}

// for each master winding value
// iterate through all matching contours
// collect debugCurves
void DebugRaster::sampleEdges() {
	OpContourIterator iterator(context);
	samples.emplace_back(this);
	OpDebugSamples& addSamples = samples.back();
	addSamples.sampleType = SampleType::edgeOutput;
	for (auto contour : iterator) {
		addSamples.sample(contour);
	}
}

void DebugRaster::sample(SampleType sampleType) {
	bool gatherAll = SampleType::contourResolved == sampleType 
			|| SampleType::segmentResolved == sampleType;
	OP_ASSERT(gatherAll || SampleType::contourInput == sampleType 
			|| SampleType::segmentInput == sampleType);
	OpContourIterator iterator(context);
	for (auto contour : iterator) {
		OpDebugSamples* addSamples = nullptr;
		for (auto& test : samples) {
			OpWinding& w = test.winding;
			if (test.sampleType == sampleType && (gatherAll
					|| (contour->windingStorage.size() == w.w.size 
					&& !std::memcmp(&contour->windingStorage.front(), w.w.data, w.w.size)))) {
				addSamples = &test;
				break;
			}
		}
		if (!addSamples) {
			samples.emplace_back(this);
			addSamples = &samples.back();
			addSamples->winding = contour->winding();
			addSamples->sampleType = sampleType;
		}
		addSamples->sample(contour);
	}
}

std::string DebugRaster::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	ASSERT_FIRST(samples);
	DEBUG_DUMP_COMMON_VECTOR(samples);
	ASSERT_ORDERED(samples, context);
	DEBUG_DUMP_REQUIRED_VALUE(context, scale);
    DEBUG_DUMP_REQUIRED_VALUE(scale, offsetX);
    DEBUG_DUMP_REQUIRED_VALUE(offsetX, offsetY);
    DEBUG_DUMP_REQUIRED_VALUE(offsetY, bitWidth);
    DEBUG_DUMP_REQUIRED_VALUE(bitWidth, bitHeight);
    DEBUG_DUMP_REQUIRED_VALUE(bitHeight, subSamples);
	ASSERT_ORDERED(subSamples, sendToDebugger);
	ASSERT_ORDERED(sendToDebugger, makeBits);
	ASSERT_LAST_OFFSET(makeBits, 2);
	return s;
}

void DebugRaster::dumpSet(char const*& str) {
	ASSERT_FIRST(samples);
	DEBUG_SET_COMMON_VECTOR(samples);
	ASSERT_ORDERED(samples, context);
	DEBUG_SET_REQUIRED_VALUE(context, scale);
    DEBUG_SET_REQUIRED_VALUE(scale, offsetX);
    DEBUG_SET_REQUIRED_VALUE(offsetX, offsetY);
    DEBUG_SET_REQUIRED_VALUE(offsetY, bitWidth);
    DEBUG_SET_REQUIRED_VALUE(bitWidth, bitHeight);
    DEBUG_SET_REQUIRED_VALUE(bitHeight, subSamples);
	ASSERT_ORDERED(subSamples, sendToDebugger);
	ASSERT_ORDERED(sendToDebugger, makeBits);
	ASSERT_LAST_OFFSET(makeBits, 2);
}

#if OP_DEBUG_VALIDATE
void DebugRaster::validate() {
	for (const auto& sample : samples) {
		for (const auto& set : sample.sampleSet) {
			for (const auto& s : set) {
				if (s.contour)
					OP_ASSERT(WindingType::uninitialized != s.contour->debugWinding.type);
			}
		}
	}
}
#endif

OpWinding::OpWinding(OpContext* context, DebugWindingZero)
	: w({ nullptr, nullptr, 0 })
	, type(WindingType::caller) {  // always copy
	PathOpsV0Lib::ContextCount countFuncPtr = context->contextCallbacks.windingBytesFuncPtr;
	OP_ASSERT(context->contourStorage);  // some contour is required so code can retrieve context
	OP_ASSERT(context->contourStorage->used);  // there must be at least one contour
	w.contour = (ContourPtr) context->contourStorage->storage;
	if (countFuncPtr)
		w.size = (*countFuncPtr)((ContextPtr) context);
	else {  // if windings bytes function is not provided: then
		w.size = ((OpContour*) w.contour)->windingStorage.size();
		for (OpContour* test : context->contours)  // all contours must have the same winding size
			OP_ASSERT(w.size == test->windingStorage.size());
	}
	w.data = context->allocateWinding(w.size);
	zeroCommon();
}

void OpWinding::debugZero() {
	OP_ASSERT(WindingType::uninitialized == type);
    zeroCommon();
}

#endif
