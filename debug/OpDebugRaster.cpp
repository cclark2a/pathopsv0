// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#include "OpTestDrive.h"
#include "OpDebugRaster.h"

#if TEST_RASTER

#include "OpContour.h"
#include "OpSegment.h"

using namespace PathOpsV0Lib;

static float toCenter(float x) {
	return (int64_t) (x + .5f) + .5f;
}

static float toLimit(float x) {
	return (int64_t) (x - .5f) + .5f;
}


void OpCurve::debugScale(double scale, double offsetX, double offsetY) {
#if OP_DEBUG
	contours->debugCallBack(c.type).scaleFuncPtr(c, scale, offsetX, offsetY);
#endif
}

void OpDebugSamples::addCurveXatY(Curve original, int parentID, OpWinding* winding, bool curveDown) {
	OP_DEBUG_CODE(OpCurve orig(contours, original));
	OpCurve curve(contours, original);
	curve.debugScale(scale, offsetX, offsetY);
	OpPoint xy = curve.firstPt();
	OpPoint xyEnd = curve.lastPt();
	if (xy.y > xyEnd.y)
		std::swap(xy, xyEnd);
	float y = toCenter(xy.y);
	float yEnd = toLimit(xyEnd.y);
	y = std::max(0.5f, y);
	yEnd = std::min((bitHeight - 1) + 0.5f, yEnd);
	while (y <= yEnd) {
		float t = curve.tAtXY(0, 1, XyChoice::inY, y);
		float x = curve.ptAtT(t).x;
		if (x < bitWidth) {
//			OpDebugOut("x:" + STR(x) + " y:" + STR(y) + "\n");
			x = std::max(0.f, x);
			int intY = (int) y;
			OP_ASSERT(0 <= x && x < bitWidth);
			OP_ASSERT(0 <= intY && intY < bitHeight);
			samples.push_back({ winding, x, y, intY, parentID, curveDown, false });
		}
		y += 1.f;
	}
}

void OpDebugSamples::addCurveYatX(Curve original, int parentID, OpWinding* winding, bool curveRight) {
	OP_DEBUG_CODE(OpCurve orig(contours, original));
	OpCurve curve(contours, original);
	curve.debugScale(scale, offsetX, offsetY);
	OpPoint xy = curve.firstPt();
	OpPoint xyEnd = curve.lastPt();
	if (xy.x > xyEnd.x)
		std::swap(xy, xyEnd);
	float x = toCenter(xy.x);
	float xEnd = toLimit(xyEnd.x);
	x = std::max(0.5f, x);
	xEnd = std::min((bitHeight - 1) + 0.5f, xEnd);
	while (x <= xEnd) {
		float t = curve.tAtXY(0, 1, XyChoice::inX, x);
		float y = curve.ptAtT(t).y;
		if (y < bitHeight) {
//			OpDebugOut("x:" + STR(x) + " y:" + STR(y) + "\n");
			y = std::max(0.f, y);
			OP_ASSERT(0 <= x && x < bitWidth);
			OP_ASSERT(0 <= y && y < bitHeight);
			samples.push_back({ winding, x, y, (int) y, parentID, curveRight, true });
		}
		x += 1.f;
	}
}

float OpDebugSamples::compare(OpDebugSamples& outputs) {
	if (!contours->rasterEnabled)
		return 0;
	if (samples.size())
		return 0;
	outputs.sort();
	size_t comboIndex = 0;
	size_t outIndex = 0;
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
	while (comboIndex < samples.size() || outIndex < outputs.samples.size()) {
		int comboX = comboIndex < samples.size() ? samples[comboIndex].x : bitWidth;
		int comboY = comboIndex < samples.size() ? samples[comboIndex].y : bitHeight;
		int outX = outIndex < outputs.samples.size() ? outputs.samples[outIndex].x : bitWidth;
		int outY = outIndex < outputs.samples.size() ? outputs.samples[outIndex].y : bitHeight;
		if ((comboY < outY || (comboY == outY && comboX < outX)) && comboIndex < samples.size()) {
			RasterSample& sample = samples[comboIndex];
			if (!comboSum.contour)
				comboSum = OpWinding(sample.winding->contour, sample.winding->w);
			if (comboY > lastComboY) {
				comboSum.zero();
				lastComboX = 0;
				comboVisible = false;
			}
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
			}
			comboVisible = visible;
			lastComboY = comboY;
			comboY = ++comboIndex < samples.size() ? samples[comboIndex].y : bitHeight;
		}
		if ((outY < comboY || (outY == comboY && outX < comboX)) && outIndex < outputs.samples.size()) {
			RasterSample& outSample = outputs.samples[outIndex];
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
			outY = ++outIndex < outputs.samples.size() ? outputs.samples[outIndex].y : bitHeight;
		}
		if (!comboVisible && !outVisible) {
			error += fabs(comboCoverage - outCoverage);
			comboCoverage = 0;
			outCoverage = 0;
		}
	}
	return error;
}

void OpDebugSamples::init(OpContours* contrs) {
	contours = contrs;
	if (!contours->rasterEnabled)
		return;
	float scaleX = bitWidth / contours->maxBounds.width();
	float scaleY = bitHeight / contours->maxBounds.height();
	scale = std::min(scaleX, scaleY);
	offsetX = -contours->maxBounds.left * scale;
	offsetY = -contours->maxBounds.top * scale;
}

void OpDebugSamples::sample(OpContour* contour) {
	if (!contour->segments.size())
		return;
	for (OpSegment& segment : contour->segments) {
		addCurveXatY(segment.c.c, segment.id, &segment.winding, 
				segment.c.firstPt().y < segment.c.lastPt().y);
		addCurveYatX(segment.c.c, segment.id, &segment.winding, 
				segment.c.firstPt().x < segment.c.lastPt().x);
	}
	sort();
}

void OpDebugSamples::sort() {
	std::sort(samples.begin(), samples.end(), [](const RasterSample& a, const RasterSample& b) {
		return a.intY < b.intY || (a.intY == b.intY && a.x < b.x);
	});
}

void OpDebugRaster::fillScanline(float x, float endX, int y) {
	if (x >= endX)
		return;
	int intX = (int) x;
	int intEndX = (int) endX;
	int xPartial = std::min(intX + 1, intEndX);
	uint8_t* bitsPtr = &bits[y * bitWidth + intX];
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

void dmpSample(const OpDebugSamples& samples, int match) {
	for (size_t index = 0; index < samples.samples.size(); ++index) {
		const RasterSample& sample = samples.samples[index];
		if (sample.intY == match)
			OpDebugOut("id:" + STR(sample.parentID) + " x:" + STR(sample.x) + " y:" + STR(sample.y) 
					+ " curve:" + std::string(sample.curveDown ? "T" : "F") 
					+ " vert:" + std::string(sample.vertical ? "T" : "F") + "\n");  
	}
}

void dmpSample(const OpDebugSamples* samples, int match) {
	dmpSample(*samples, match);
}

void OpDebugRaster::rasterize(const OpDebugSamples& samples, OpContour* cntr) {
	init();
	if (!samples.contours->rasterEnabled)
		return;
	OpWinding sum(WindingUninitialized::dummy);
	int intY = -1;
	float x = 0;
	bool fillOn = false;
	bool lastVisible = false;
	for (size_t index = 0; index < samples.samples.size(); ++index) {
		const RasterSample& sample = samples.samples[index];
		if (sample.vertical) {
			// !!! add capture range of partial
			continue;
		}
		OpContour* contour = sample.winding->contour;
		if (cntr && contour != cntr)
			continue;
		
		if (!sum.contour)
			sum = OpWinding(sample.winding->contour, sample.winding->w);
		if (sample.intY > intY) {
			if (fillOn)
				fillScanline(x, bitWidth, intY);  // probably an error
			sum.zero();
			x = 0;
			intY = sample.intY;
			fillOn = false;
			lastVisible = false;
		}
		if (sample.curveDown)
			sum.add(*sample.winding);
		else
			sum.subtract(*sample.winding);
		bool visible;
		if (!cntr) {
			WindKeep keep = contour->callBacks.windingKeepFuncPtr(sample.winding->w, sum.w);
			visible = WindKeep::Start == keep;
		} else
			visible = sum.visible();
		if (lastVisible == visible)
			continue;
		lastVisible = visible;
		if (fillOn) {
			// !!! extend scanline to include partial coverage by verticals, if present
			// use ids to determine that samples outside range are ok to consider?
			fillScanline(x, sample.x, intY);
		} else
			x = sample.x;
		fillOn ^= true;
	}
}

#endif
