// (c) 2024, Cary Clark cclark2@gmail.com

// verify path ops by rasterizing curves into bitmap using xor
// each operand has its own bitmap
// perform operations on bitmaps, then compare result with rasterizing path ops result

#include "OpTestDrive.h"
#include "OpDebugRaster.h"

#if TEST_RASTER

#include "OpContour.h"
#include "OpSegment.h"

namespace PathOpsV0Lib {
	struct CallerData;
}

void OpDebugRaster::set(PathOpsV0Lib::Context* context, PathOpsV0Lib::Contour* cntour,
		RasterType t) {
	if (!((OpContours*) context)->debugData.rasterEnabled)
		return;
	init((OpContours*) context, t); 
	rasterize(cntour); 
}

void OpDebugRaster::setCombined(PathOpsV0Lib::Context* context, const OpDebugRaster& left,
		const OpDebugRaster& right) {
	if (!((OpContours*) context)->debugData.rasterEnabled)
		return;
	init((OpContours*) context, RasterType::combined);
	memcpy(bits, left.bits, sizeof(bits));
	contour = left.contour;
	combine(right);
}

static float toCenter(float x) {
	return (int64_t) (x + .5f) + .5f;
}

static float toLimit(float x) {
	return (int64_t) (x - .5f) + .5f;
}

void OpDebugRaster::addCurve(OpContours* contours, PathOpsV0Lib::Curve original) {
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
		if (x <= (float) (bitWidth - 1) + 0.5f) {
			OpDebugOut("y:" + STR(y) + " x:" + STR(x) + "\n");
			float centerX = toCenter(x);
			int bitX = std::max(0, (int) centerX);
			int bitY = (int) y;
			OP_ASSERT(0 <= bitX && bitX < bitWidth);
			OP_ASSERT(0 <= bitY && bitY < bitHeight);
			if (contour)
				addWinding(bitX, bitY);
			else
				doXor(bitX, bitY);
		}
		y += 1.f;
	}
	OP_DEBUG_CODE(++pass);
}

void OpDebugRaster::addWinding(int x, int y) {
	int index = y * bitWidth + x;
	PathOpsV0Lib::Winding result { &windings[index * winding->size], winding->size };
	if (curveDown)
		contour->callBacks.windingAddFuncPtr(result, *winding);
	else
		contour->callBacks.windingSubtractFuncPtr(result, *winding);
	windInitialized[index] = true;
}

void OpDebugRaster::combine(const OpDebugRaster& operand) {
	uint8_t* src = bits;
	const uint8_t* opp = operand.bits;
	uint8_t (*bitOperator)(PathOpsV0Lib::CallerData , uint8_t , uint8_t) 
			= contour->callBacks.debugBitOperFuncPtr;
	for (int y = 0; y < bitHeight; ++y) {
		for (int x = 0; x < bitWidth; ++x) {
			*src++ = (*bitOperator)(contour->caller, *src, *opp++);
		}
	}
}

int OpDebugRaster::compare(PathOpsV0Lib::Context* context) {
    OpContours* contours = (OpContours*) context;
	if (!contours->debugData.rasterEnabled)
		return 0;
	OpDebugRaster& output = contours->opRaster;

	uint8_t* src = bits;
	uint8_t* out = output.bits;
	int result = 0;
	for (int y = 0; y < bitHeight; ++y) {
		for (int x = 0; x < bitWidth; ++x) {
			result += *src++ != *out++;
		}
	}
	return result;
}

void OpDebugRaster::doXor(int x, int y) {
	uint8_t* bitAddr = &bits[y * bitWidth + x];
	uint8_t* bitLimit = &bits[(y + 1) * bitWidth];
	while (bitAddr < bitLimit) {
		*bitAddr++ ^= 0xFF;
	}
}

void OpDebugRaster::init(OpContours* contours, RasterType t) {
	float scaleX = bitWidth / contours->maxBounds.width();
	float scaleY = bitHeight / contours->maxBounds.height();
	memset(bits, 0xFF, sizeof(bits));
	contour = nullptr;
	winding = nullptr;
	scale = std::min(scaleX, scaleY);
	offsetX = -contours->maxBounds.left * scale;
	offsetY = -contours->maxBounds.top * scale;
	type = t;
	// for image watch
	width = bitWidth;
	height = bitHeight;
	stride = bitWidth;
	data = (char*) bits;
}

void OpDebugRaster::rasterize(PathOpsV0Lib::Contour* libContour) {
	contour = (OpContour*) libContour;
	if (!contour->segments.size())
		return;
	OpSegment& firstSeg = contour->segments[0];
	winding = &firstSeg.winding.w;
	size_t windingSize = winding->size;
	windings.resize(windingSize * bitWidth * bitHeight);
	memset(windInitialized, false, sizeof(windInitialized));
	OpContours* contours = contour->contours;
	for (OpSegment& segment : contour->segments) {
		curveDown = segment.c.firstPt().y < segment.c.lastPt().y;
		addCurve(contours, segment.c.c);
	}
	char* windingBytes = &windings.front();
	bool* inits = windInitialized;
	uint8_t* bitBytes = bits;
	for (int y = 0; y < bitHeight; ++y) {
		PathOpsV0Lib::Winding sum { nullptr, 0 };
		bool pixelOn = false;
		for (int x = 0; x < bitWidth; ++x) {
			if (*inits++) {
				if (!sum.size)
					sum = { windingBytes, windingSize };
				else
					contour->callBacks.windingAddFuncPtr(sum, {windingBytes, windingSize});
				pixelOn = contour->callBacks.windingVisibleFuncPtr(sum);
			}
			*bitBytes++ = pixelOn ? 0x00 : 0xFF;
			windingBytes += windingSize;
		}
	}
}

#endif
