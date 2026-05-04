// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#include "OpDebug.h"

#if OP_TEST_RASTER

#include <filesystem>
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

void OpCurve::debugScale(double scaleX, double scaleY, double offsetX, double offsetY) {
	context().debugCallback(c).scaleFuncPtr(c, scaleX, scaleY, offsetX, offsetY);
	start = c.data->start;
	end = c.data->end;
}

#if OP_DEBUG_SERIALIZE
std::string RasterSample::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	ASSERT_FIRST(contour);
	DEBUG_DUMP_OPTIONAL_COMMON_ID(contour);
	DEBUG_DUMP_OPTIONAL_ID(contour, segment);
	DEBUG_DUMP_OPTIONAL_ID(segment, edge);
	DEBUG_DUMP_OPTIONAL_POS_VALUE(edge, curveIndex);
	DEBUG_DUMP_OPTIONAL_FINITE_VALUE(curveIndex, x);
	DEBUG_DUMP_OPTIONAL_FINITE_VALUE(x, t);
	DEBUG_DUMP_BOOL(t, curveDown);
	ASSERT_LAST_OFFSET(curveDown, 3);
	return s;
}
#endif

#if OP_DEBUG_DUMP
void RasterSample::dumpResolveAll(OpContext* context) {
	if (contour)
		context->dumpResolve(contour);
	if (segment)
		context->dumpResolve(segment);
	if (edge)
		context->dumpResolve(edge);
}

void RasterSample::dumpSet(char const*& str) {
	ASSERT_FIRST(contour);
	DEBUG_SET_OPTIONAL_COMMON_ID(contour);
	DEBUG_SET_OPTIONAL_ID(contour, segment);
	DEBUG_SET_OPTIONAL_ID(segment, edge);
	DEBUG_SET_OPTIONAL_VALUE(edge, curveIndex);
	DEBUG_SET_OPTIONAL_FINITE_VALUE(curveIndex, x);
	DEBUG_SET_OPTIONAL_FINITE_VALUE(x, t);
	DEBUG_SET_BOOL(t, curveDown);
	ASSERT_LAST_OFFSET(curveDown, 3);
}
#endif

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
	, winding(DebugWindingRaster::dummy)
	, raster(r)
	, mask(r) {
	sampleSet.resize(r->bitHeight * r->subSamples);
//    OpDebugOut("  scanline:" + STR(scanline) + " offsetY:" + STR(offsetY) + "\n");
}

// advance 1/8th (or whatever sub samples is set to)
// track previous (and first) add so that consecutive (and first + last) points are not duplicated
void OpDebugSamples::addCurveXatY(const Curve& original, RasterSample& base, float tLo, float tHi) {
	OpCurve curve(original, Rotated::yes);
	curve.debugScale(1, raster->scale * raster->subSamples, 0, raster->offsetY);  // leave x alone
	OpPoint xy = curve.ptAtT(tLo);
	OpPoint xyEnd = curve.ptAtT(tHi);
	// error in point at t math may put points outside curve bounds; ceil then puts in wrong pixel
	// restrict answer to y-axis bounds of curve
	OpRect bounds = curve.fullBounds();
	xy.y = OpMath::PinSorted(bounds.top, xy.y, bounds.bottom);
	xyEnd.y = OpMath::PinSorted(bounds.top, xyEnd.y, bounds.bottom);
    bool curveDown = xy.y < xyEnd.y || (xy.y == xyEnd.y && xy.x < xyEnd.x);
	if (!curveDown)
		std::swap(xy, xyEnd);
	int row = (int) std::ceil(xy.y);
	OP_ASSERT(0 < row);
	int rowEnd = (int) std::ceil(xyEnd.y);
	while (row < rowEnd) {
//		OpAssert(9 != row || !base.edge || 54 != base.edge->id);
//		OpAssert(9 != row || !base.contour || 6 != base.contour->id);
		float t = curve.tAtXY(tLo, tHi, XyChoice::inY, (float) row);
		float x = curve.ptAtT(t).x;
		OP_ASSERT(row < sampleSet.size());
		std::vector<RasterSample>& samples = sampleSet[row];
		OP_DEBUG_CODE(float bitX = x * raster->scale + raster->offsetX);
		OP_ASSERT(0 < bitX && bitX < raster->bitWidth);
		base.x = x;
		base.t = t;
		base.curveDown = curveDown;
		samples.push_back(base);
		++row;
	}
}

void OpDebugSamples::addCurveXatY(OpContour* contour, int debugCurveIndex,
		const OpCurve& opCurve, std::vector<float>& extrema) {
	OpRoots tValues;
	for (float ex : extrema) {
		tValues.add(ex);
	}
	tValues.add(0);
	tValues.add(1);
	tValues.sort();
    for (int index = 0; index < tValues.count() - 1; ++index) {
		RasterSample base { contour, nullptr, nullptr, debugCurveIndex };
		addCurveXatY(opCurve.c, base, tValues.roots[index], tValues.roots[index + 1]);
	}
}

void OpDebugSamples::addCurveXatY(OpSegment* segment) {
	const Curve& curve = segment->c.c;
	RasterSample base { nullptr, segment };
	addCurveXatY(curve, base, 0, 1);
}

void OpDebugSamples::addCurveXatY(OpEdge* edge) {
	const Curve& curve = edge->curve.c;
	RasterSample base { nullptr, nullptr, edge };
	// end points for edge curve come from intersections

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
			winding.subtract(w);	// copy-on-demand
		else
			winding.add(w);
		++index;
	} while (index < samples.size());
	return x;
}

static float NextVisible(const OpDebugSamples& sampleSet, OpWinding& sum, size_t& index, 
		std::vector<RasterSample>& samples, WindKeep keeper) {
	OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
	float x = OpNaN;
	if (sampleSet.alwaysVisible()) {
		if (index < samples.size())
			x = samples[index++].x;
		return x;
	}
	while (index < samples.size()) {  // accumulate winding of start
		size_t firstIndex = index;
		OpWinding winding(sampleSet.zeroWinding, DebugWindingSum::dummy);  // shallow zeroed winding for this x value
		x = NextX(samples, index, winding);
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
		// choose between 'visible' and 'keep'
		// use 'visible' to say if source path shows
		// use 'keep' to say if computed path shows
		sum.add(winding);
		WindKeep keep = (*sampleSet.visibleFunc())(winding.w, sum.w);
		OP_DEBUG_VALIDATE_CODE(sampleSet.raster->validate());
		if (/* keeper == keep */ WindKeep::Discard != keep)
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

static float NextVisiblePixel(const OpDebugSamples& sampleSet, OpWinding& sum, size_t& index, 
		std::vector<RasterSample>& samples, WindKeep keeper, float scaleX, float offsetX) {
	float x = NextVisible(sampleSet, sum, index, samples, keeper);
	return x * scaleX + offsetX;
}
struct XRange {
	float start = 0;
	float end = 0;
};

float OpDebugSamples::compare(std::vector<RasterSamples>& outputs) {
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
		auto advance = [](const std::vector<RasterSample>& samples, size_t& index) {
			float result = OpNaN;
			while (index < samples.size()) {
				const RasterSample& sample = samples[index];
				++index;
				if (sample.visible) {
					result = sample.x;
					break;
				}
			}
			if (OpMath::IsNaN(result))
				return OpNaN;
			size_t testIndex = index;
			while (testIndex < samples.size()) {
				const RasterSample& test = samples[testIndex];
				if (test.edge || test.x != result)
					break;
				testIndex = ++index;
			}
			return result;
		};
		inIndex = 0;
		XRange inXs { advance(subS, inIndex), advance(subS, inIndex) }; 
		outIndex = 0;
		XRange outXs { advance(subO, outIndex), advance(subO, outIndex) }; 
#if OP_DEBUG_DUMP
		auto debugCheckDiff = [this](float diff) {
			OP_ASSERT(diff >= 0);
			float scaled = diff * raster->scale;
			OpAssert(scaled < .7f);
		};
#endif
		auto checkExhausted = [&inXs, &outXs, &error  OP_DEBUG_DUMP_PARAMS(debugCheckDiff)]
				(float x) {
			if (OpMath::IsNaN(inXs.end)) {
				if (!OpMath::IsNaN(outXs.end)) {
					float xStart = OpMath::IsNaN(x) ? outXs.start : std::max(x, outXs.start);
					float diff = outXs.end - xStart;
					OP_DEBUG_DUMP_CODE(debugCheckDiff(diff));
					error += diff;
				}
				return true;
			}
			if (OpMath::IsNaN(outXs.end)) {
				float xStart = OpMath::IsNaN(x) ? inXs.start : std::max(x, inXs.start);
				float diff = inXs.end - xStart;
				OP_DEBUG_DUMP_CODE(debugCheckDiff(diff));
				error += diff;
				return true;
			}
			return false;
		};
		if (checkExhausted(OpNaN))
			continue;
		float x = std::min(inXs.start, outXs.start);
		auto nextX = [&error, &x  OP_DEBUG_DUMP_PARAMS(debugCheckDiff)]
				(const XRange& upper, const XRange& lower) {
			float xEnd = std::min(upper.start, lower.end);
			float diff = xEnd - x;
			OP_DEBUG_DUMP_CODE(debugCheckDiff(diff));
			error += diff;
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
			if (checkExhausted(x))
				break;
			x = std::max(x, std::min(inXs.start, outXs.start));
		}
    }
	return error;
}

void OpDebugSamples::sample(OpContour* contour) {
	OP_ASSERT(WindingType::uninitialized != contour->debugWinding.type);
	if (SampleType::contourInput == sampleType || SampleType::contourResolved == sampleType) {
		for (int index = 0; index < (int) contour->debugCurveData.size(); ++index) {
			std::vector<float> extrema;
			OpCurve opCurve(contour->debugCurve(index, &extrema), Rotated::yes);
			if (opCurve.fullBounds().isEmpty())
				continue;
//			lastCurve = index == contour->debugCurveData.size() - 1;
			addCurveXatY(contour, index, opCurve, extrema);
		}
		return;
	}
	for (OpSegment& segment : contour->segments) {
		if (SampleType::segmentInput == sampleType || SampleType::segmentResolved == sampleType) {
//			lastCurve = &segment == &contour->segments.back();
			addCurveXatY(&segment);
		} else {
			OP_ASSERT(SampleType::edges == sampleType);
			for (OpEdge& edge : segment.edges) {
				if (edge.inOutput)
					addCurveXatY(&edge);
			}
		}
	}
}

void OpDebugSamples::sample(DebugOutput& output) {
	RasterSample base { nullptr, nullptr, output.edge };
	addCurveXatY(output.curve.c, base, 0, 1);
}

void OpDebugSamples::rasterize() {
    for (int row = 0; row < raster->bitHeight; ++row) {
		mask.rasterize(*this, row, (float) raster->scale, (float) raster->offsetX);
	}
}

#if 0
void OpDebugSamples::resetAdd() {
	firstRow = INT_MAX;
	lastRow = INT_MAX;
	firstDown = false;
	lastDown = false;
	lastCurve = false;
}
#endif

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
	WindingKeep visibleFunc = nullptr;
#if OP_DEBUG
	visibleFunc = context->debugContextCallbacks.debugWindingVisibleFuncPtr;
#endif
	if (!visibleFunc || (SampleType::contourInput != sampleType 
			&& SampleType::segmentInput != sampleType))
		visibleFunc = context->windingCallbacks.windingKeepFuncPtr;
	return visibleFunc;
}

#if OP_DEBUG_SERIALIZE

#define SampleType_Base
#undef OP_ENUM_MEMBER
#define OP_ENUM_MEMBER(w) { SampleType::w, #w }
ENUM_NAME_STRUCT(SampleType)

#if !OP_DEBUG_FAST_TEST
std::string SampleTypeName(SampleType element) {
    int first = (int) SampleTypeNames[0].element;
    return SampleTypeNames[(int) element - first].name;
}
#endif

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
//		if (!sample.size())
//			continue;
		s += "sample:" + STR(sample.size()) + " "; 
		int mPerLine = 0;
		for (const auto& member : sample) { 
			s += member.debugDump(l, b) + (++mPerLine % 4 ? " " : "\n"); 
		} 
		if (!debugIfMatching(s, '\n'))
			s += "\n";
	}
	DEBUG_DUMP_STRUCT(sampleSet, mask);
	ASSERT_ORDERED(mask, sampleType);
    s += "sampleType:" + SampleTypeName(sampleType) + " ";
//	ASSERT_LAST_OFFSET(sampleType, 4);
	return s;
}
#endif

#if OP_DEBUG_DUMP
void OpDebugSamples::dumpResolveAll(OpContext* context) {
	zeroWinding.dumpResolveAll(context);
	winding.dumpResolveAll(context);
	for (RasterSamples& samples : sampleSet) {
		for (RasterSample& sample : samples) {
			sample.dumpResolveAll(context);
		}
	}
}

void OpDebugSamples::dumpSet(char const*& str) {
	ASSERT_FIRST(zeroWinding);
	OpDebugRequired(str, "zeroWinding"); 
	zeroWinding.dumpSet(raster->context, str);
	ASSERT_ORDERED(zeroWinding, winding);
	OpDebugRequired(str, "winding");
	winding.dumpSet(raster->context, str);
	ASSERT_ORDERED(winding, raster);
	ASSERT_ORDERED(raster, sampleSet);
	OpDebugRequired(str, "size");
	size_t size = OpDebugReadSizeT(str);
	sampleSet.resize(size);
	for (auto& sample : sampleSet) {
		DEBUG_SET_COMMON_VECTOR(sample);
	}
	mask.raster = raster;
	DEBUG_SET_STRUCT(sampleSet, mask);
	ASSERT_ORDERED(mask, sampleType);
    sampleType = SampleTypeStr(str, "sampleType", SampleType::none);
//	ASSERT_LAST_OFFSET(sampleType, 4);
}
#endif

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
	OP_DEBUG_CODE(uint8_t* limit = &subScan[(subLine + 1) * raster->bitWidth - 1]);
	uint8_t* bitsPtr = &subScan[subLine * raster->bitWidth + intX];
	int intEndX = (int) endX;
	if (intX == intEndX) {
		OP_ASSERT(bitsPtr <= limit);
		bitsPtr[0] = std::min(255, bitsPtr[0] + (int) ((endX - x) * 255));
		return;
	}
	if (x != intX) {
		OP_ASSERT(bitsPtr <= limit);
		bitsPtr[0] = std::min(255, bitsPtr[0] + (int) ((intX + 1 - x) * 255));
		++bitsPtr;
	}
	while (++intX < intEndX) {
		OP_ASSERT(bitsPtr <= limit);
		*bitsPtr++ = 255;
	}
	if (intEndX < endX) {
		OP_ASSERT(bitsPtr <= limit);
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

void OpDebugBitmap::rasterize(OpDebugSamples& sampleSet, int row, float scaleX, float offsetX) {
	OP_DEBUG_VALIDATE_CODE(raster->validate());
    OpDebugScanLine subScan(raster);
	int scanLine = row * raster->subSamples;
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
		float endX = 0;  // reused by edges when sample is missing
		do {
			OP_DEBUG_VALIDATE_CODE(raster->validate());
			float startX = NextVisiblePixel(sampleSet, sum, index, samples, WindKeep::Start,
					scaleX, offsetX);
			OP_DEBUG_VALIDATE_CODE(raster->validate());
			if (OpMath::IsNaN(startX))
				break;
			float lastX = endX;
			endX = NextVisiblePixel(sampleSet, sum, index, samples, WindKeep::End,
					scaleX, offsetX);
			if (SampleType::edges == sampleSet.sampleType && OpMath::IsNaN(endX))
				endX = lastX;
			else
				OP_ASSERT(!OpMath::IsNaN(endX));
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

#if OP_DEBUG_SERIALIZE
std::string OpDebugBitmap::debugDump(DebugLevel l, DebugBase b) const {
	OP_ASSERT(raster);
	std::string s;
	s += "size:" + STR(bits.size());
	s += " height:" + STR(raster->bitHeight);
	s += " width:" + STR(raster->bitWidth) + "\n";
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
#endif

#if OP_DEBUG_DUMP
void OpDebugBitmap::dumpSet(char const*& str) {
	OpDebugRequired(str, "size");
	bits.resize(OpDebugReadSizeT(str));
	OpDebugRequired(str, "height");
	int height = OpDebugReadSizeT(str);
	OpDebugRequired(str, "width");
	int width = OpDebugReadSizeT(str);
	char hexStr[5] { '0', 'x', '?', '?', '\0' };
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			hexStr[2] = *str++; 
			hexStr[3] = *str++;
			const char* hexPtr = hexStr;
			bits[y * width + x] = OpDebugByteToInt(hexPtr);
		}
		OpDebugExitOnFail("missing \\n", '\n' == *str++);
	}
	OpNop();
}
#endif

#if OP_DEBUG_SERIALIZE
std::string DebugOutput::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	s += "curve:" + curve.debugDump(l, b) + " ";
	s += "winding:" + winding.debugDump(l, b) + " ";
	s += "loopAttr:" + STR((int) loopAttr) + " ";
	if (edge)
		s += "edge:" + STR(edge->id);
	return s;
}
#endif

#if OP_DEBUG_DUMP
void DebugOutput::dumpSet(OpContext* context, char const*& str) {
	OpDebugRequired(str, "curve");
	curve.dumpSet(str);
	curve.c.context = (ContextPtr) context;
	OpDebugRequired(str, "winding");
	winding.dumpSet(context, str);
	OpDebugRequired(str, "loopAttr");
	loopAttr = (PathOpsV0Lib::LoopAttribute) OpDebugReadSizeT(str);
	if (OpDebugOptional(str, "edge"))
		edge = (OpEdge*) OpDebugReadSizeT(str);
}

void DebugOutput::dumpResolveAll(OpContext* context) {
	winding.dumpResolveAll(context);
	context->dumpResolve(edge);
}
#endif

void DebugRaster::addOutput(PathOpsV0Lib::Output o, OpEdge* e) {
	OpCurve opCurve(o.curve, Rotated::debug);
	OpWinding opWinding(o.winding, DebugWindingRef::dummy);
	outputs.push_back({opCurve, opWinding, o.attribute, e});
}

void DebugRaster::in() {
	if (tooSmall())
		return;
	OP_DEBUG_DUMP_CODE(context->dumpFile("init"));
	sample(SampleType::contourResolved);
	OP_DEBUG_CODE(validate());
#if OP_DEBUG || OP_DEBUGGER
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
	OP_DEBUG_CODE(validate());
	for (auto& s : samples) {
		s.rasterize();
	}
	OpNop();
#endif
}

float DebugRaster::out() {
	float result = 0;
	if (tooSmall())
		return result;
#if OP_DEBUG || OP_DEBUGGER
	if (sendToDebugger) {
		sampleEdges();
		OpDebugSamples& edges = samples.back();
		edges.sort();
		edges.rasterize();
	}
#endif
	sampleOutput();
	OP_ASSERT(samples.size());
	OpDebugSamples& output = samples.back();
	output.sort();
#if OP_DEBUG || OP_DEBUGGER
	if (sendToDebugger) {
		output.rasterize();
#if OP_DEBUG_DUMP
		record(BitsFile);
#endif
	}
#endif
	OP_ASSERT(SampleType::output == output.sampleType);
	OpDebugSamples& allContours = samples[0];
	OP_ASSERT(SampleType::contourResolved == allContours.sampleType);
	result = allContours.compare(output.sampleSet);
	return result;
}

#if OP_DEBUG_DUMP || OP_DEBUGGER
bool DebugRaster::playback(std::string filename) {
    std::string buffer = dmpFileToStr(filename);
    if (buffer.empty())
        return false;
    const char* str = buffer.c_str();
	dumpSet(str);
	dumpResolveAll(context);
	return true;
}
#endif

#if OP_DEBUG_SERIALIZE || OP_DEBUGGER
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
#endif

// for each master winding value
// iterate through all matching contours
// collect debugCurves
void DebugRaster::sampleEdges() {
	OpContourIterator iterator(context);
	samples.emplace_back(this);
	OpDebugSamples& addSamples = samples.back();
	addSamples.sampleType = SampleType::edges;
	for (auto contour : iterator) {
		addSamples.sample(contour);
	}
}

void DebugRaster::sampleOutput() {
	samples.emplace_back(this);
	OpDebugSamples& addSamples = samples.back();
	addSamples.sampleType = SampleType::output;
	for (DebugOutput& output : outputs) {
//		OpAssert(!output.edge || 60 != output.edge->id);
		addSamples.sample(output);
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
			addSamples->winding.usedByRaster = true;
			addSamples->sampleType = sampleType;
		}  // else 
			// addSamples->resetAdd();
		addSamples->sample(contour);
	}
}

bool DebugRaster::tooSmall() const {
	OP_ASSERT(context);
	if (!context->maxBounds.width() || !context->maxBounds.height())
		return true;
	if (!scale || OpMath::IsNaN((float) offsetX) || OpMath::IsNaN((float) offsetY))
		return true;
	return false;
}

#if OP_DEBUG_SERIALIZE
std::string DebugRaster::debugDump(DebugLevel l, DebugBase b) const {
	std::string s;
	ASSERT_FIRST(samples);
	DEBUG_DUMP_COMMON_VECTOR(samples);
	DEBUG_DUMP_VECTOR(samples, outputs);
	ASSERT_ORDERED(outputs, context);
	DEBUG_DUMP_REQUIRED_DOUBLE(context, scale);
    DEBUG_DUMP_REQUIRED_DOUBLE(scale, offsetX);
    DEBUG_DUMP_REQUIRED_DOUBLE(offsetX, offsetY);
    DEBUG_DUMP_REQUIRED_VALUE(offsetY, bitWidth);
    DEBUG_DUMP_REQUIRED_VALUE(bitWidth, bitHeight);
    DEBUG_DUMP_REQUIRED_VALUE(bitHeight, subSamples);
	ASSERT_ORDERED(subSamples, sendToDebugger);
	ASSERT_LAST_OFFSET(sendToDebugger, 3);
	return s;
}
#endif

#if OP_DEBUG_SERIALIZE
void DebugRaster::deleteOld() {
	std::string filePath = dmpFileToPath(BitsFile);
	if (!std::filesystem::exists(filePath))
		return;
	std::filesystem::remove(filePath);
}
#endif

#if OP_DEBUG_DUMP
void DebugRaster::dumpResolveAll(OpContext* ctx) {
	OP_ASSERT(context == ctx);
	for (OpDebugSamples& s : samples)
		s.dumpResolveAll(context);
	for (DebugOutput& o : outputs)
		o.dumpResolveAll(context);
}

void DebugRaster::dumpSet(char const*& str) {
	ASSERT_FIRST(samples);
	if (OpDebugOptional(str, "samples")) { 
		size_t count = OpDebugReadSizeT(str); 
		samples.resize(count); 
		for (auto& member : samples) {
			member.raster = this;
			member.dumpSet(str); 
		}
	}
	// DEBUG_SET_VECTOR(samples, outputs);
	ASSERT_ORDERED(samples, outputs);
    if (OpDebugOptional(str, "outputs")) { 
        size_t count = OpDebugReadSizeT(str);
		outputs.resize(count); 
		for (auto& output : outputs) { 
			output.curve.c.context = (ContextPtr) context;
			output.dumpSet(context, str);
		} 
    } 
	ASSERT_ORDERED(outputs, context);
	DEBUG_SET_REQUIRED_FLOAT(context, scale);
    DEBUG_SET_REQUIRED_FLOAT(scale, offsetX);
    DEBUG_SET_REQUIRED_FLOAT(offsetX, offsetY);
    DEBUG_SET_REQUIRED_VALUE(offsetY, bitWidth);
    DEBUG_SET_REQUIRED_VALUE(bitWidth, bitHeight);
    DEBUG_SET_REQUIRED_VALUE(bitHeight, subSamples);
	ASSERT_ORDERED(subSamples, sendToDebugger);
	ASSERT_LAST_OFFSET(sendToDebugger, 3);
}
#endif

#if OP_DEBUG_VALIDATE
void DebugRaster::validate() {
	if (disableValidate)
		return;
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

#endif
