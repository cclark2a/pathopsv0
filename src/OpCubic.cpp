// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpMath.h"

// The algorithm below is based on github.com/cemyuksel/cyCodeBase/blob/master/cyPolynomial.h#L1061
// the original file includes the following notice:

// Copyright (c) 2022, Cem Yuksel <cem@cemyuksel.com>
// All rights reserved.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// in the Software without restriction, including without limitation the rights 
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
// copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all 
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
// SOFTWARE.

#define BOUND_ERROR 0

#if !BOUND_ERROR
#if 0
static void quadraticRoots(const float coef[3], float x0, float x1, OpRoots* roots) {
	float a = coef[0];
	float b = coef[1];
	float c = coef[2];
	float delta = b*b - 4*a*c;
	if (delta > 0) {
		float d = std::sqrt(delta);
		float q = -0.5f * (b + d * (b < 0 ? -1 : 1));
		float rv0 = q / a;
		if (rv0 >= x0 && rv0 <= x1)
			roots->addEnd(rv0);
		float rv1 = c / q;
		if (rv1 >= x0 && rv1 <= x1)
			roots->addEnd(rv1);
		return;
	}
	if (delta < 0) 
		return;
	float r0 = -0.5f * b / a;
	if (r0 >= x0 && r0 <= x1)
		roots->addEnd(r0);
}

static void cubicDeflate(const float coefficient[4], float root, float quadCoeffs[3]) {
	quadCoeffs[0] = coefficient[0];
	quadCoeffs[1] = coefficient[1] + root * quadCoeffs[0];
	quadCoeffs[2] = coefficient[2] + root * quadCoeffs[1];
}
#endif
#endif

static float cubicXAtT(const OpCubicFloatType coefficient[4], float t) { 
	return ((coefficient[0] * t + coefficient[1]) * t + coefficient[2]) * t + coefficient[3]; 
}

static float cubicDxAtT(const OpCubicFloatType derivative[3], float t) { 
	return (derivative[0] * t + derivative[1]) * t + derivative[2]; 
}

inline bool crossesZero(float a, float b) {
	return (a < 0) != (b < 0);
}

#if 0
struct CubicState {
	float cubicRoot(float coefficient[4], float derivative[3], float t1, float t2);
	float cubicBackup(float coefficient[4], float derivative[3], float t1, float t2, float y0);

	float midT;
	float testRoot;
};

float CubicState::cubicRoot(float coefficient[4], float derivative[3], float t1, float t2) {
	midT = (t1 + t2) / 2;
	if (t2 - t1 <= 2 * OpEpsilon)
		return midT;
	testRoot = midT;
	for (int tries = 0; tries < 16; ++tries) {
		float testT = midT - cubicXAtT(coefficient, midT) / cubicDxAtT(derivative, midT);
		testT = std::max(std::min(testT, t2), t1);
		if (std::abs(midT - testT) <= OpEpsilon) 
			return testT;
		midT = testT;
	}
	return OpNaN;
}

float CubicState::cubicBackup(float coefficient[4], float derivative[3], float t1, float t2, 
		float y0) {
	if (!OpMath::IsFinite(midT)) 
		midT = testRoot;
	float yr = cubicXAtT(coefficient, midT);
	while (true) {
		bool side = crossesZero(y0, yr);
		(side ? t2 : t1) = midT;
		float dy = cubicDxAtT(derivative, midT);
		float dx = yr / dy;
		float xn = midT - dx;
		if (xn > t1 && xn < t2) { // valid Newton step
			float stepsize = std::abs(midT - xn);
			midT = xn;
			if (stepsize > OpEpsilon)
				yr = cubicXAtT(coefficient, midT );
			else {
#if BOUND_ERROR
				midT = xn + (side ? -OpEpsilon : OpEpsilon);
				OP_ASSERT(midT != xn); 
				yr = cubicXAtT(coefficient, midT);
				bool s = crossesZero(y0, yr);
				if (side != s) 
					return xn;
#else
				break;
#endif
			}
		} else { // Newton step failed
			midT = (t1 + t2) / 2;
			if (midT == t1 || midT == t2 || t2 - t1 <= 2 * OpEpsilon)
				break;
			yr = cubicXAtT(coefficient, midT);
		}
	}
	return midT;
}

static void cubicRoots(float coefficient[4], OpRoots* roots) {
#if !BOUND_ERROR
	auto quadRoots = [coefficient, roots](float t) {
		float quadCoeff[3];
		cubicDeflate(coefficient, roots->roots[0], quadCoeff);
		quadraticRoots(quadCoeff, t, 1, roots);
	};
#endif
	float xy1 = coefficient[3];  // cubicXAtT(coefficient, 0);
	float a = coefficient[0];
	float b = coefficient[1];
	float c = coefficient[2];
	float xy2 = a + b + c + xy1;  // cubicXAtT(coefficient, 1);
	a *= 3;
	float derivative[] = { a, 2 * b, c };
	auto cubicRoot = [coefficient, &derivative, roots](float t1, float t2, float y0) {
		CubicState state;
		float result = state.cubicRoot(coefficient, derivative, t1, t2);
		if (OpMath::IsFinite(result)) {
			roots->addEnd(result);
			return;
		}
		float backup = state.cubicBackup(coefficient, derivative, t1, t2, y0);
		// !!! try backup as solution to original cubic; discard if not close
		float yr = cubicXAtT(coefficient, backup);
		if (fabsf(yr) < OpEpsilon * 8)
			roots->addEnd(backup);
	};
	auto crossZeroCubic = [cubicRoot](float x1, float x2, float t1, float t2, float y0) {
		if (crossesZero(x1, x2)) {
			cubicRoot(t1, t2, y0);
			return true;
		}
		return false;
	};
	float delta = b * b - a * c;
	if (delta > 0) {
		float d_2root = std::sqrt(delta);
		float q = -(b + d_2root * (b < 0 ? -1 : 1));
		float t1 = q / a;
		float t2 = c / q;
		if (t1 > t2)
			std::swap(t1, t2);
		bool interval = t1 >= 1 || t2 <= 0 || (t1 <= 0 && t2 >= 1);
		if (crossesZero(xy1, xy2)) {		
			if (interval) {	// first, last, or middle interval only
				cubicRoot(0, 1, xy1);
				return;
			}
		} else if (interval) 
			return;
		if (t1 > 0) {
			float xya = cubicXAtT(coefficient, t1);
			if (crossZeroCubic(xy1, xya, 0, t1, xy1)) {  // first interval
#if !BOUND_ERROR
				if (crossesZero(xya, xy2) 
						|| (t2 < 1 && crossesZero(xya, cubicXAtT(coefficient, t2))))
					quadRoots(t1);
				return;
#endif
			}
			if (t2 < 1) {
				float xyb = cubicXAtT(coefficient, t2);
				if (crossZeroCubic(xya, xyb, t1, t2, xya)) {
#if !BOUND_ERROR
					if (crossesZero(xyb, xy2)) 
						quadRoots(t2);
					return;
#endif
				}
				crossZeroCubic(xyb, xy2, t2, 1, xyb);  // last interval
			} else
				crossZeroCubic(xya, xy2, t1, 1, xya);
		} else {
			float xyb = cubicXAtT(coefficient, t2);
			if (crossZeroCubic(xy1, xyb, 0, t2, xyb)) {
#if !BOUND_ERROR
				if (crossesZero(xyb, xy2))
					quadRoots(t2);
				return;
#endif
			}
			crossZeroCubic(xyb, xy2, t2, 1, xyb);	// last interval
		}
	} else 
		crossZeroCubic(xy1, xy2, 0, 1, xy1);
	return;
}
#endif

// assumption is that this can only return a single root (and that it must return that root)...
float OpMath::CubicRoot(OpCubicFloatType A, OpCubicFloatType B, OpCubicFloatType C, 
		OpCubicFloatType D) {
	float result = 0; // OpNaN;
	OpRoots quadRoots;
	if (0 == A) {
		quadRoots = QuadRootsDouble(B, C, D);
		quadRoots = quadRoots.keepValidTs();
		OP_ASSERT(1 >= quadRoots.count());
		if (quadRoots.count())
			result = quadRoots.get(0);
	} else if (0 == D) {
	//	quadRoots = QuadRootsDouble(A, B, C);
		result = 0;
	} else if (0 == A + B + C + D) {
	//	quadRoots = QuadRootsDouble(A, A + B, -D);
		result = 1;
	} else {
		OpCubicFloatType coefficient[] = { A, B, C, D };
		OpCubicFloatType derivative[] = { 3 * A, 2 * B, C };
		float midT = .5;
	//	float testRoot = midT;
		int tries = 0;
		do {
			float testT = midT - cubicXAtT(coefficient, midT) / cubicDxAtT(derivative, midT);
			testT = std::max(std::min(testT, 1.f), 0.f);
			if (std::abs(midT - testT) <= OpEpsilon) {
				result = testT;
				break;
			}
			midT = testT;
		} while (++tries < 16);
        OP_ASSERT(OpMath::IsFinite(midT));
		result = midT;
#if 0
		if (!OpMath::IsFinite(midT)) 
			midT = testRoot;
		float y0 = (float) D;
		float yr = cubicXAtT(coefficient, midT);
		float t2 = 1;
		float t1 = 0;
		while (true) {
			bool side = crossesZero(y0, yr);
			(side ? t2 : t1) = midT;
			float dy = cubicDxAtT(derivative, midT);
			float dx = yr / dy;
			float xn = midT - dx;
			if (xn > t1 && xn < t2) { // valid Newton step
				float stepsize = std::abs(midT - xn);
				midT = xn;
				if (stepsize > OpEpsilon)
					yr = cubicXAtT(coefficient, midT );
				else {
					midT = xn + (side ? -OpEpsilon : OpEpsilon);
					OP_ASSERT(midT != xn); 
					yr = cubicXAtT(coefficient, midT);
					bool s = crossesZero(y0, yr);
					if (side != s) 
						return xn;
				}
			} else { // Newton step failed
				midT = (t1 + t2) / 2;
				if (midT == t1 || midT == t2 || t2 - t1 <= 2 * OpEpsilon)
					break;
				yr = cubicXAtT(coefficient, midT);
			}
		}
		return midT;
#endif
	}
#if 0  // !!! move this to caller, only where necessary
	   //       calling it always unnecessarily breaks op curve : center()
	if (OpMath::NearlyZeroT(result))
		result = 0;
	else if (OpMath::NearlyOneT(result))
		result = 1;
#endif
	return result;
}

#include "cyPolynomial.h"

OpRoots OpMath::CubicRootsY(float A, float B, float C, float D) {
	float coef[] = { D, C, B, A };
	float roots[3];
#define NO_ERROR_ALLOWED 1
#if NO_ERROR_ALLOWED
	int numRoots = cy::CubicRoots<float>(roots, coef, 0, 1, 0);
#else
	int numRoots = cy::CubicRoots<float>(roots, coef, 0, 1, OpEpsilon);
#endif
	OpRoots result;
	for (int index = 0; index < numRoots; ++index)
		result.add(roots[index]);
	if (!D)
		result.addEnd(0);
	if (A + B + C + D == 0)
		result.addEnd(1);
	return result;
}
