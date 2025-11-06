// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#if TEST_RASTER

#include "OpDebugRaster.h"
#include "OpContext.h"
#include "OpSegment.h"
#include "DebugOps.h"

namespace PathOpsV0Lib {

void debugRasterAdd(DebugContextData caller, Curve curve, int parentID) {
    OP_ASSERT(sizeof(OpDebugSamples*) == caller.size);
    OpDebugSamples* samples = (OpDebugSamples*) caller.data;
    samples->addCurveXatY(curve, parentID);
}

}

using namespace PathOpsV0Lib;

void OpCurve::debugScale(double scale, double offsetX, double offsetY) {
	context().debugCallback(c).scaleFuncPtr(c, scale, offsetX, offsetY);
}

OpDebugSamples::OpDebugSamples(DebugRaster* r) {
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
void OpDebugSamples::addCurveXatY(Curve original, int id, OpWinding* winding) {
	int rows = raster->bitHeight * raster->subSamples;
	auto toSubGrid = [rows](float x) {
		return std::floorf(x * rows) / rows;
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
	yEnd = std::min(1.f, yEnd);
    int row = (int) std::ceil(y * rows);
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
			samples.push_back({ &winding->w, x, id, curveDown });
		}
        ++row;
		y = (float) row / (float) rows;
	}
}

float OpDebugSamples::compare(OpDebugSamples& outputs) {
	outputs.sort();
	float lastComboX = 0;
	float lastOutX = 0;
	float comboCoverage = 0;
	float outCoverage = 0;
	float error = 0;
	bool comboVisible = false;
	bool outVisible = false;
	int rows = raster->subSamples * raster->bitHeight;

    for (int row = 0; row < rows; ++row) {
        std::vector<RasterSample>& subS = sampleSet[row];
        std::vector<RasterSample>& subO = outputs.sampleSet[row];
	    size_t comboIndex = 0;
	    size_t outIndex = 0;
		OpWinding comboSum(WindingUninitialized::dummy);
	    while (comboIndex < subS.size() || outIndex < subO.size()) {
		    int comboX = comboIndex < subS.size() ? subS[comboIndex].x : raster->bitWidth;
		    int outX = outIndex < subO.size() ? subO[outIndex].x : raster->bitWidth;
		    if (comboX < outX && comboIndex < subS.size()) {
			    RasterSample& sample = subS[comboIndex];
				if (!comboSum.isSet())
					comboSum.zeroUninitialized(*sample.winding);
			    if (sample.curveDown)
				    comboSum.add(*sample.winding);
			    else
				    comboSum.subtract(*sample.winding);
			    bool visible = comboSum.visible();
			    if (comboVisible != visible) {
				    if (visible)
					    comboCoverage += sample.x - lastComboX;
				    else
					    lastComboX = sample.x;
				    comboVisible = visible;
			    }
		    }
		    if (outX < comboX && outIndex < subO.size()) {
			    RasterSample& outSample = subO[outIndex];
			    if (outVisible)
				    outCoverage += outSample.x - lastOutX;
			    else
				    lastOutX = outSample.x;
			    outVisible ^= true;
		    }
		    if (!comboVisible && !outVisible) {
			    error += fabs(comboCoverage - outCoverage);
			    comboCoverage = 0;
			    outCoverage = 0;
		    }
	    }
    }
	if (error >= 9) {
	#if OP_DEBUG_FAST_TEST
		std::lock_guard<std::mutex> guard(out_mutex);
	#endif
	    std::string testname = raster->context->debugData.testname;
	    OpDebugOut(testname + " raster errors:" + STR(error) + "\n");
    }
	return error;
}

void OpDebugSamples::sample(OpContour* contour) {
	for (OpSegment& segment : contour->segments) {
		addCurveXatY(segment.c.c, segment.id, &segment.winding);
	}
	if (contour->segments.size())
		sort();
}

void OpDebugSamples::sort() {
    for (std::vector<RasterSample>& samps : sampleSet) {       
	    std::sort(samps.begin(), samps.end(), [](const RasterSample& a, const RasterSample& b) {
		    return a.x < b.x;
	    });
    }
}

OpDebugScanLine::OpDebugScanLine(DebugRaster* r) {
	debugRaster = r;
	int size = r->bitWidth * r->subSamples;
	subScan = (uint8_t*) malloc(size);
	memset(subScan, 0xFF, size);
}

void OpDebugScanLine::fill(float x, float endX, int subLine) {
    OP_ASSERT(0 <= subLine && subLine < debugRaster->subSamples);
	if (x >= endX)
		return;
	int intX = (int) x;
	int intEndX = (int) endX;
	int xPartial = std::min(intX + 1, intEndX);
    OP_ASSERT(0 <= intX && intX <= debugRaster->bitWidth);
	uint8_t* bitsPtr = &subScan[subLine * debugRaster->bitWidth + intX];
	if (x < xPartial)
		*bitsPtr++ -= (xPartial - x) * 255;
	while (++intX < intEndX) {
		*bitsPtr++ = 0;
	}
	if (intEndX < endX)
		*bitsPtr = 255 - (endX - intEndX) * 255;
}

OpDebugBitmap::OpDebugBitmap(DebugRaster* r)
	: debugRaster(r) {
	if (!r)
		return;
	int size = r->bitWidth * r->bitHeight * r->subSamples;
	bits = (uint8_t*) malloc(size);
	memset(bits, 0xFF, size);
}

void OpDebugBitmap::rasterize(const OpDebugSamples& sampleSet, int row) {
    OpContext* context = debugRaster->context;
    OpDebugScanLine subScan(debugRaster);
	int scanLine = row * debugRaster->subSamples;
	for (int subLine = 0; subLine < debugRaster->subSamples; ++subLine) {
		const std::vector<RasterSample>& samples = sampleSet.sampleSet[scanLine + subLine];
	    OpWinding sum(WindingUninitialized::dummy);
//		sum.zero(context);
	    float x = 0;
	    bool fillOn = false;
	    bool lastVisible = false;
		for (const RasterSample& sample : samples) {
		    if (WindingType::uninitialized == sum.type)
			    sum.zeroUninitialized(*sample.winding);
		    if (sample.curveDown)
			    sum.add(*sample.winding);
		    else
			    sum.subtract(*sample.winding);
		    bool visible;
//		    if (sampleSet.callKeep) {  // !!! unsure what this is for...
			    WindKeep keep = context->windingCallbacks.windingKeepFuncPtr(
                        (ContextPtr) context, *sample.winding, sum.w);
			    visible = WindKeep::Start == keep;
//		    } else
//			    visible = sum.visible();
		    if (lastVisible == visible)
			    continue;
		    lastVisible = visible;
		    if (fillOn) {
			    // use ids to determine that samples outside range are ok to consider?
			    subScan.fill(x, sample.x, subLine);
		    } else
			    x = sample.x;
		    fillOn ^= true;
        }
	}
    for (int x = 0; x < debugRaster->bitWidth; ++x) {
        int pixel = 0;
        for (int y = 0; y < debugRaster->subSamples; ++y) {
            pixel += subScan.subScan[y * debugRaster->bitWidth + x];
        }
        bits[scanLine * debugRaster->bitWidth + x] = pixel / debugRaster->subSamples;
    }
    OpNop();  // dmpScan(this, scanLine)  and  dmpSub(subScan)
}

void DebugRaster::in() {
    for (int row = 0; row < bitHeight; ++row) {
		for (auto contour : context->contours) {
            inSamples.sample(contour);
		}
		if (RasterType::draw == rasterType)
			inBits.rasterize(inSamples, row);
	}
    sendToDebugger(inBits, inSamples, "inBits"); // if a single test is run, serialize data so debugger can draw it
}

float DebugRaster::out() {
	for (int row = 0; row < bitHeight; ++row) {
		for (auto contour : context->contours) {
            outSamples.sample(contour);
		}
		if (RasterType::draw == rasterType)
        	outBits.rasterize(outSamples, row);
    }
	float result = 0;
	if (RasterType::compare == rasterType)
    	result = inSamples.compare(outSamples);
    sendToDebugger(outBits, outSamples, "outBits"); // if a single test is run, serialize data so debugger can draw it
	return result;
}

void DebugRaster::sendToDebugger(OpDebugBitmap& bits, OpDebugSamples& samples, std::string name) {
	if (!context->debugData.runOneFile) 
		return;
    std::string filename = dmpFileToPath(name);
	FILE* file = fopen(filename.c_str(), "wb");
	size_t written = fwrite(bits.bits, 1, sizeof(bits.bits), file);
	OP_ASSERT(sizeof(bits.bits) == written);
	fclose(file);
}

std::string debugDmpSample(const OpDebugSamples& sampleSet) {
	std::string s = "sampleSet:" + STR(sampleSet.sampleSet.size()) + "\n";
    int sub = 0;
	for (const std::vector<RasterSample>& samples : sampleSet.sampleSet) {
        OpDebugOut("sub:" + STR(sub) + "\n");
		for (const RasterSample& sample : samples) {
			s += " x:" + STR(sample.x);
			s += " id:" + STR(sample.id);
			if (sample.curveDown)
				s += " curveDown";  
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
    for (int y = 0; y < scans.debugRaster->subSamples; ++y) {
        std::string s;
        for (int x = 0; x < scans.debugRaster->bitWidth; ++x) {
            uint8_t bit = scans.subScan[y * scans.debugRaster->bitWidth + x];
            s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
        }
        OpDebugOut(s + "\n");
    }
}

void dmpScan(const OpDebugBitmap* rasta, int line) {
    std::string s;
    for (int x = 0; x < rasta->debugRaster->bitWidth; ++x) {
        uint8_t bit = rasta->bits[line * rasta->debugRaster->bitWidth + x];
        s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
    }
    OpDebugOut(s + "\n");
}

void dmpScan(const OpDebugBitmap* rasta) {
    for (int y = 0; y < rasta->debugRaster->bitHeight; ++y) {
        dmpScan(rasta, y);
    }
}

#endif
