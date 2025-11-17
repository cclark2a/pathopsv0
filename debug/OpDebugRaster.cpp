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

PathOpsV0Lib::Winding RasterSample::winding() const {
	if (contour) {
		OP_ASSERT(!segment);
		OP_ASSERT(!edge);
		return contour->winding();
	}
	if (segment) {
		OP_ASSERT(!edge);
		return segment->winding.w;
	}
	OP_ASSERT(edge);
	return edge->winding.w;
}

OpDebugSamples::OpDebugSamples(DebugRaster* r) 
	: zeroWinding(r->context, DebugWindingZero::dummy)
	, raster(r)
	, mask(r) {
	OpContext* context = r->context;
	float scaleX = r->bitWidth / context->maxBounds.width();
	float scaleY = r->bitHeight / context->maxBounds.height();
	scale = std::min(scaleX, scaleY);
	offsetX = -context->maxBounds.left * scale;
	offsetY = -context->maxBounds.top * scale;
	sampleSet.resize(r->bitHeight * r->subSamples);
//    OpDebugOut("  scanline:" + STR(scanline) + " offsetY:" + STR(offsetY) + "\n");
}

// advance 1/8th (or whatever sub samples is set to)
void OpDebugSamples::addCurveXatY(const Curve& original, RasterSample& base) {
	int rows = raster->bitHeight * raster->subSamples;
	auto toSubGrid = [this](float x) {
		return std::floorf(x * raster->subSamples) / raster->subSamples;
	};
	OpCurve curve(original, Rotated::no);
	curve.debugScale(scale, offsetX, offsetY);
	OpPoint xy = curve.firstPt();
	OpPoint xyEnd = curve.lastPt();
    bool curveDown = xy.y <= xyEnd.y;
	if (!curveDown)
		std::swap(xy, xyEnd);
	float y = toSubGrid(xy.y);
	float yEnd = toSubGrid(xyEnd.y);
	y = std::max(0.f, y);
	yEnd = std::min((float) raster->bitHeight, yEnd);
    int row = (int) std::ceil(y * raster->subSamples);
	while (y < yEnd) {
		float t = curve.tAtXY(0, 1, XyChoice::inY, y);
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
	const Curve& curve = contour->debugCurves[debugCurveIndex];
	RasterSample base { contour, nullptr, nullptr, debugCurveIndex };
	addCurveXatY(curve, base);
}

void OpDebugSamples::addCurveXatY(const OpSegment* segment) {
	const Curve& curve = segment->c.c;
	RasterSample base { nullptr, segment };
	addCurveXatY(curve, base);
}

void OpDebugSamples::addCurveXatY(const OpEdge* edge) {
	const Curve& curve = edge->curve.c;
	RasterSample base { nullptr, nullptr, edge };
	addCurveXatY(curve, base);
}

struct InSamples {
	float x;
	WindKeep keep;
};

static float NextX(const std::vector<RasterSample>& samples, size_t& index, OpWinding& winding) {
	OP_ASSERT(index < samples.size());
	float x = samples[index].x;
	do {
		const RasterSample& next = samples[index];
		if (x != next.x)
			break;
		Winding w = next.winding();
		if (next.curveDown)
			winding.add(w);	// copy-on-demand
		else
			winding.subtract(w);
		++index;
	} while (index < samples.size());
	return x;
}

static float NextVisible(const OpDebugSamples& sampleSet, OpWinding& sum, size_t& index, 
		const std::vector<RasterSample>& samples, WindKeep keeper) {
	float x = OpNaN;
	OpWinding winding(sampleSet.zeroWinding);  // shallow zeroed winding for this x value
	while (index < samples.size()) {  // accumulate winding of start
		x = NextX(samples, index, winding);
		sum.add(winding);
		// choose between 'visible' and 'keep'
		// use 'visible' to say if source path shows
		// use 'keep' to say if computed path shows
		if (sampleSet.alwaysVisible())
			break;
		WindKeep keep = (*sampleSet.visibleFunc())(winding.w, sum.w);
		if (keeper == keep)
			break;
		OP_ASSERT(WindKeep::Discard == keep);
		x = OpNaN;
	}
	return x;
}

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
		std::vector<InSamples> inSamples;
		// !!! this needs work to function with frames instead of fills
		//     put that task off until after fills work
		//     of concern: how to distinguish horizontal frames from frames with two intersections
		do {
			float xStart = NextVisible(*this, sum, inIndex, subS, WindKeep::Start);
			if (OpMath::IsNaN(xStart))
				break;
			inSamples.push_back({ xStart, WindKeep::Start });
			float xEnd = NextVisible(*this, sum, inIndex, subS, WindKeep::End);
			if (OpMath::IsNaN(xEnd)) {
				xEnd = raster->bitWidth;
				break;
			}
			if (OpMath::IsNaN(xEnd))
				break;
			inSamples.push_back({ xEnd, WindKeep::End });
		} while (inIndex < subS.size());
		// note that 'out' samples do not have or need winding; they implicitly describe coverage
		inIndex = 0;
		outIndex = 0;
		InSamples inStart, inEnd;
		float outStart, outEnd;
		auto advanceIn = [&inStart, &inEnd, &inIndex, &inSamples, this]() {
			inStart = { (float) raster->bitWidth, WindKeep::Discard };
			inEnd = inStart;
			if (inIndex < inSamples.size())  
				inStart = inSamples[inIndex++];
			if (inIndex < inSamples.size())
				inEnd = inSamples[inIndex++];
			return inStart.x;
		};
		float inPos = advanceIn();
		auto advanceOut = [&outStart, &outEnd, &outIndex, subO, this]() {
			outStart = outIndex < subO.size() ? subO[outIndex++].x : raster->bitWidth;
			outEnd = outIndex < subO.size() ? subO[outIndex++].x : raster->bitWidth;
			return outStart;
		};
		float outPos = advanceOut();
		// advance 'in' until it describes a coverage span; advance 'out' the same
		// find in/out overlap; missing overlap is error amount
		// if no overlap, advance the one of in/out to the left of the other
	    while (inIndex < inSamples.size() || outIndex < subO.size()) {
			float errorEnd = inPos < outPos ? std::min(inEnd.x, outPos) : std::min(outEnd, inPos);
			error += errorEnd - std::min(inPos, outPos);
			if (inPos < errorEnd)
				inPos = inEnd.x <= errorEnd ? advanceIn() : errorEnd;
			if (outPos < errorEnd)
				outPos = outEnd <= errorEnd ? advanceOut() : errorEnd;
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
	if (SampleType::contourInput == sampleType || SampleType::contourResolved == sampleType) {
		for (int index = 0; index < (int) contour->debugCurves.size(); ++index) {
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
	OP_ASSERT(raster);
	std::string s;
	ASSERT_FIRST(zeroWinding);
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
    DEBUG_DUMP_REQUIRED_VALUE(mask, scale);
    DEBUG_DUMP_REQUIRED_VALUE(scale, offsetX);
    DEBUG_DUMP_REQUIRED_VALUE(offsetX, offsetY);
    DEBUG_DUMP_OPTIONAL_PUBLIC_VALUE(offsetY, winding, winding.contour);
	ASSERT_ORDERED(winding, sampleType);
    s += "sampleType:" + SampleTypeName(sampleType) + " ";
	ASSERT_LAST_OFFSET(sampleType, 4);
	return s;
}

void OpDebugSamples::dumpSet(char const*& str) {
	OP_ASSERT(raster);
	ASSERT_FIRST(zeroWinding);
	OpDebugRequired(str, "size");
	size_t size = OpDebugReadSizeT(str);
	sampleSet.resize(size);
	for (auto& sample : sampleSet) {
		DEBUG_SET_COMMON_VECTOR(sample);
	}
	DEBUG_SET_STRUCT(sampleSet, mask);
    DEBUG_SET_REQUIRED_VALUE(mask, scale);
    DEBUG_SET_REQUIRED_VALUE(scale, offsetX);
    DEBUG_SET_REQUIRED_VALUE(offsetX, offsetY);
    DEBUG_SET_OPTIONAL_PUBLIC_VALUE(offsetY, winding);
	ASSERT_ORDERED(winding, sampleType);
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

void OpDebugBitmap::rasterize(const OpDebugSamples& sampleSet, int row) {
    OpDebugScanLine subScan(raster);
	int scanLine = row * raster->subSamples;
	for (int subLine = 0; subLine < raster->subSamples; ++subLine) {
		const std::vector<RasterSample>& samples = sampleSet.sampleSet[scanLine + subLine];
		if (samples.empty())
			continue;
		size_t index = 0;
		OpWinding sum(sampleSet.zeroWinding);  // shallow zero winding
		// zero winding
			// accumulate winding while x is unchanged
		// add accumulated winding to sum 
		// call keep with winding, sum
		do {
			float startX = NextVisible(sampleSet, sum, index, samples, WindKeep::Start);
			if (OpMath::IsNaN(startX))
				break;
			float endX = NextVisible(sampleSet, sum, index, samples, WindKeep::End);
			if (OpMath::IsNaN(endX))
				endX = (float) raster->bitWidth;
			subScan.fill(startX, endX, subLine);
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
    OpNop();  // dmpScan(this, scanLine)  and  dmpSub(subScan)
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
			Winding& w = test.winding;
			if (test.sampleType == sampleType && (gatherAll
					|| (contour->windingStorage.size() == w.size 
					&& !std::memcmp(&contour->windingStorage.front(), w.data, w.size)))) {
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
    DEBUG_DUMP_REQUIRED_VALUE(context, bitWidth);
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
    DEBUG_SET_REQUIRED_VALUE(context, bitWidth);
    DEBUG_SET_REQUIRED_VALUE(bitWidth, bitHeight);
    DEBUG_SET_REQUIRED_VALUE(bitHeight, subSamples);
	ASSERT_ORDERED(subSamples, sendToDebugger);
	ASSERT_ORDERED(sendToDebugger, makeBits);
	ASSERT_LAST_OFFSET(makeBits, 2);
}

extern DebugLevel defaultLevel;
extern DebugBase defaultBase;

std::string debugDmpSample(const OpDebugSamples& sampleSet) {
	std::string s = "sampleSet:" + STR(sampleSet.sampleSet.size()) + "\n";
    int sub = 0;
	for (const std::vector<RasterSample>& samples : sampleSet.sampleSet) {
        OpDebugOut("sub:" + STR(sub) + "\n");
		for (const RasterSample& sample : samples) {
			s += sample.debugDump(defaultLevel, defaultBase);  
        }
		s += "\n";
        ++sub;
	}
	return s;
}

void dmpSample(const OpDebugSamples* samples) {
	std::string s = debugDmpSample(*samples);
	OpDebugOut(s);
}

void dmpSub(const OpDebugScanLine& scans) {
    for (int y = 0; y < scans.raster->subSamples; ++y) {
        std::string s;
        for (int x = 0; x < scans.raster->bitWidth; ++x) {
            uint8_t bit = scans.subScan[y * scans.raster->bitWidth + x];
            s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
        }
        OpDebugOut(s + "\n");
    }
}

void dmpScan(const OpDebugBitmap* rasta, int line) {
    std::string s;
    for (int x = 0; x < rasta->raster->bitWidth; ++x) {
        uint8_t bit = rasta->bits[line * rasta->raster->bitWidth + x];
        s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
    }
    OpDebugOut(s + "\n");
}

void dmpScan(const OpDebugBitmap* rasta) {
    for (int y = 0; y < rasta->raster->bitHeight; ++y) {
        dmpScan(rasta, y);
    }
}

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
