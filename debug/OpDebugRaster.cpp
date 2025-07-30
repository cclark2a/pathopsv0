// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#include "OpTestDrive.h"
#include "OpDebugRaster.h"

#if TEST_RASTER

#include "OpContext.h"
#include "OpSegment.h"
#include "DebugOps.h"

using namespace PathOpsV0Lib;

static float toSubGrid(float x) {
	return std::floor(x * OpDebugSamples::subSamples) / OpDebugSamples::subSamples;
}

void OpCurve::debugScale(double scale, double offsetX, double offsetY) {
	context->debugCallback(c.type).scaleFuncPtr(c, scale, offsetX, offsetY);
}

// advance 1/8th (or whatever sub samples is set to)
void OpDebugSamples::addCurveXatY(Curve original, int parentID, OpWinding* winding) {
	OpCurve curve(context, original, Rotated::no);
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
    int subline = (int) std::ceil(y * OpDebugSamples::subSamples);
	while (y < yEnd) {
		float t = curve.tAtXY(0, 1, XyChoice::inY, y);
		float x = curve.ptAtT(t).x;
		if (x < OpDebugRaster::bitWidth) {
			OpDebugOut("id:" + STR(parentID) + " x:" + STR(x) + " y:" + STR(y) 
                    + " subline:" + STR(subline) + "\n");
			x = std::max(0.f, x);
            OP_ASSERT(subline < subSamples);
            std::vector<RasterSample>& samples = sampleSet[subline];
			OP_ASSERT(0 <= x && x < OpDebugRaster::bitWidth);
			samples.push_back({ &winding->w, x, y, parentID, curveDown });
		}
        ++subline;
		y += 1.f / subSamples;
	}
}

namespace PathOpsV0Lib {

void debugRasterAdd(DebugContextData caller, Curve curve, int parentID) {
    OP_ASSERT(sizeof(OpDebugSamples*) == caller.size);
    OpDebugSamples* samples = (OpDebugSamples*) caller.data;
    samples->addCurveXatY(curve, parentID);
}

}

#if 0
void OpDebugSamples::addCurveYatX(Curve original, int parentID, OpWinding* winding, bool curveRight) {
	OpCurve curve(context, original, Rotated::no);
	curve.debugScale(scale, offsetX, offsetY);
	OpPoint xy = curve.firstPt();
	OpPoint xyEnd = curve.lastPt();
	if (xy.x > xyEnd.x)
		std::swap(xy, xyEnd);
	float x = toCenter(xy.x);
	float xEnd = toLimit(xyEnd.x);
	x = std::max(0.5f, x);
	xEnd = std::min((subSamples - 1) + 0.5f, xEnd);
	while (x <= xEnd) {
		float t = curve.tAtXY(0, 1, XyChoice::inX, x);
		float y = curve.ptAtT(t).y;
		if (y < subSamples) {
//			OpDebugOut("x:" + STR(x) + " y:" + STR(y) + "\n");
			y = std::max(0.f, y);
			int intY = (int) y;
            OP_ASSERT(0 <= intY && intY < subSamples);
            std::vector<RasterSample>& samples = sampleSet[intY];
			OP_ASSERT(0 <= x && x < OpDebugRaster::bitWidth);
			OP_ASSERT(0 <= y && y < subSamples);
			samples.push_back({ winding, x, y, parentID, curveRight });
		}
		x += 1.f;
	}
}
#endif

float OpDebugSamples::compare(OpDebugSamples& outputs) {
	outputs.sort();
	OpWinding comboSum(WindingUninitialized::dummy);
	float lastComboY = -1;
	float lastOutY = -1;
	float lastComboX = 0;
	float lastOutX = 0;
	float comboCoverage = 0;
	float outCoverage = 0;
	float error = 0;
	bool comboVisible = false;
	bool outVisible = false;
    for (int sub = 0; sub < subSamples; ++sub) {
        std::vector<RasterSample>& subS = sampleSet[sub];
        std::vector<RasterSample>& subO = outputs.sampleSet[sub];
	    size_t comboIndex = 0;
	    size_t outIndex = 0;
	    while (comboIndex < subS.size() || outIndex < subO.size()) {
		    int comboX = comboIndex < subS.size() ? subS[comboIndex].x : OpDebugRaster::bitWidth;
		    int comboY = comboIndex < subS.size() ? subS[comboIndex].y : OpDebugRaster::bitHeight;
		    int outX = outIndex < subO.size() ? subO[outIndex].x : OpDebugRaster::bitWidth;
		    int outY = outIndex < subO.size() ? subO[outIndex].y : OpDebugRaster::bitHeight;
		    if ((comboY < outY || (comboY == outY && comboX < outX)) && comboIndex < subS.size()) {
			    RasterSample& sample = subS[comboIndex];
			    if (WindingType::uninitialized == comboSum.type)
				    comboSum = OpWinding(context, *sample.winding);
			    if (comboY > lastComboY) {
				    comboSum.zero(context);
				    lastComboX = 0;
				    comboVisible = false;
			    }
			    if (sample.curveDown)
				    comboSum.add(context, *sample.winding);
			    else
				    comboSum.subtract(context, *sample.winding);
			    bool visible = comboSum.visible(context);
			    if (comboVisible != visible) {
				    if (visible)
					    comboCoverage += sample.x - lastComboX;
				    else
					    lastComboX = sample.x;
			    }
			    comboVisible = visible;
			    lastComboY = comboY;
			    comboY = ++comboIndex < subS.size() ? subS[comboIndex].y : OpDebugRaster::bitHeight;
		    }
		    if ((outY < comboY || (outY == comboY && outX < comboX)) && outIndex < subO.size()) {
			    RasterSample& outSample = subO[outIndex];
			    if (outY > lastOutY) {
				    lastOutX = 0;
				    outVisible = false;
			    }
			    if (outVisible)
				    outCoverage += outSample.x - lastOutX;
			    else
				    lastOutX = outSample.x;
			    outVisible ^= true;
			    lastOutY = outY;
			    outY = ++outIndex < subO.size() ? subO[outIndex].y : OpDebugRaster::bitHeight;
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
	    std::string testname = context->debugData.testname;
	    OpDebugOut(testname + " raster errors:" + STR(error) + "\n");
    }
	return error;
}

void OpDebugSamples::init(OpContext* ctext, int scanline, bool keep) {
	context = ctext;
	float scaleX = OpDebugRaster::bitWidth / context->maxBounds.width();
	float scaleY = OpDebugRaster::bitHeight / context->maxBounds.height();
	scale = std::min(scaleX, scaleY);
	offsetX = -context->maxBounds.left * scale;
	offsetY = -context->maxBounds.top * scale - scanline;
    callKeep = keep;
    OpDebugOut("  scanline:" + STR(scanline) + " offsetY:" + STR(offsetY) + "\n");
}

void OpDebugSamples::sample(OpContour* contour ) {
	if (!contour->segments.size())
		return;
	for (OpSegment& segment : contour->segments) {
		addCurveXatY(segment.c.c, segment.id, &segment.winding);
#if 0
        addCurveYatX(segment.c.c, segment.id, &segment.winding, 
				segment.c.firstPt().x < segment.c.lastPt().x);
#endif
	}
	sort();
}

void OpDebugSamples::sort() {
    for (std::vector<RasterSample>& samps : sampleSet) {       
	    std::sort(samps.begin(), samps.end(), [](const RasterSample& a, const RasterSample& b) {
		    return a.x < b.x;
	    });
    }
}

struct OpDebugScanLine {
	void fill(float x, float endX, int y);
	void init();

    uint8_t subScan[OpDebugRaster::bitWidth * OpDebugSamples::subSamples];  // 1 byte per pixel, black/white only
	char* data; // for image watch
	int width; 
	int height;
	int stride;
};

void OpDebugScanLine::init() {
	memset(subScan, 0xFF, sizeof(subScan));
	// for image watch
	width = OpDebugRaster::bitWidth;
	height = OpDebugSamples::subSamples;
	stride = OpDebugRaster::bitWidth;
	data = (char*) subScan;
}

void OpDebugScanLine::fill(float x, float endX, int subLine) {
    OP_ASSERT(0 <= subLine && subLine < OpDebugSamples::subSamples);
	if (x >= endX)
		return;
	int intX = (int) x;
	int intEndX = (int) endX;
	int xPartial = std::min(intX + 1, intEndX);
    OP_ASSERT(0 <= intX && intX <= OpDebugRaster::bitWidth);
	uint8_t* bitsPtr = &subScan[subLine * OpDebugRaster::bitWidth + intX];
	if (x < xPartial)
		*bitsPtr++ -= (xPartial - x) * 255;
	while (++intX < intEndX) {
		*bitsPtr++ = 0;
	}
	if (intEndX < endX)
		*bitsPtr = 255 - (endX - intEndX) * 255;
}

void OpDebugRaster::init() {
	memset(bits, 0xFF, sizeof(bits));
	// for image watch
	width = bitWidth;
	height = bitHeight;
	stride = bitWidth;
	data = (char*) bits;
}

void OpDebugRaster::rasterize(const OpDebugSamples& sampleSet, int scanLine) {
    OpContext* context = sampleSet.context;
    OpDebugScanLine subScan;
    subScan.init();
	int subLine = 0;
	for (const std::vector<RasterSample>& samples : sampleSet.sampleSet) {
	    OpWinding sum(WindingUninitialized::dummy);
//		sum.zero(context);
	    float x = 0;
	    bool fillOn = false;
	    bool lastVisible = false;
		for (const RasterSample& sample : samples) {
		    if (WindingType::uninitialized == sum.type)
			    sum.zeroUninitialized(context, *sample.winding);
		    if (sample.curveDown)
			    sum.add(context, *sample.winding);
		    else
			    sum.subtract(context, *sample.winding);
		    bool visible;
		    if (sampleSet.callKeep) {
			    WindKeep keep = context->windingCallbacks.windingKeepFuncPtr(
                        *sample.winding, sum.w);
			    visible = WindKeep::Start == keep;
		    } else
			    visible = sum.visible(context);
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
        ++subLine;
	}
    for (int x = 0; x < bitWidth; ++x) {
        int pixel = 0;
        for (int y = 0; y < OpDebugSamples::subSamples; ++y) {
            pixel += subScan.subScan[y * bitWidth + x];
        }
        bits[scanLine * bitWidth + x] = pixel / OpDebugSamples::subSamples;
    }
    OpNop();  // dmpScan(this, scanLine)  and  dmpSub(subScan)
}

void OpDebugRaster::compare(OpDebugRaster& comp) {

}

void DebugRaster::in(Context* ctx) {
	OpContext* context = (OpContext*) ctx;
	OpDebugRaster inBits;
    inBits.init();
    for (int scanLine = 0; scanLine < OpDebugRaster::bitHeight; ++scanLine) {
		inSamples.init(context, scanLine, false);
		for (auto contour : context->contours) {
            inSamples.sample(contour);
            OpNop();
		}
		inBits.rasterize(inSamples, scanLine);
	}
    OpNop();  // !!! draw in bits for debugging
}

void DebugRaster::out(Context*  ) {
    inSamples.compare(outSamples);
	OpDebugRaster outBits;
    outBits.init();
    for (int scanLine = 0; scanLine < OpDebugRaster::bitHeight; ++scanLine) {
        outBits.rasterize(outSamples, scanLine);
    }
    OpNop();  // !!! draw out bits for debugging
}

void dmpSample(const OpDebugSamples& sampleSet) {
    int sub = 0;
	for (const std::vector<RasterSample>& samples : sampleSet.sampleSet) {
        OpDebugOut("sub:" + STR(sub) + "\n");
		for (const RasterSample& sample : samples) {
			OpDebugOut("id:" + STR(sample.parentID) + " x:" + STR(sample.x) + " y:" + STR(sample.y) 
					+ " curve:" + std::string(sample.curveDown ? "T" : "F") 
//					+ " vert:" + std::string(sample.vertical ? "T" : "F")
                    + "\n");  
        }
        ++sub;
	}
}

void dmpSample(const OpDebugSamples* samples) {
	dmpSample(*samples);
}

void dmpSub(const OpDebugScanLine& scans) {
    for (int y = 0; y < OpDebugSamples::subSamples; ++y) {
        std::string s;
        for (int x = 0; x < OpDebugRaster::bitWidth; ++x) {
            uint8_t bit = scans.subScan[y * OpDebugRaster::bitWidth + x];
            s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
        }
        OpDebugOut(s + "\n");
    }
}

void dmpScan(const OpDebugRaster* rasta, int line) {
    std::string s;
    for (int x = 0; x < OpDebugRaster::bitWidth; ++x) {
        uint8_t bit = rasta->bits[line * OpDebugRaster::bitWidth + x];
        s += bit < 64 ? " " : bit < 128 ?  "," :  bit < 192 ? "x" : "X";
    }
    OpDebugOut(s + "\n");
}

void dmpScan(const OpDebugRaster* rasta) {
    for (int y = 0; y < OpDebugRaster::bitHeight; ++y) {
        dmpScan(rasta, y);
    }
}

#endif
