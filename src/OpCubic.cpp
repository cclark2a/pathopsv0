// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpMath.h"

// Inspired by: github.com/cemyuksel/cyCodeBase/blob/master/cyPolynomial.h#L1061
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

static float cubicXAtT(const OpCubicFloatType coefficient[4], float t) { 
	return (float) (((coefficient[0] * t + coefficient[1]) * t + coefficient[2]) * t + coefficient[3]); 
}

static float cubicDxAtT(const OpCubicFloatType derivative[3], float t) { 
	return (float) ((derivative[0] * t + derivative[1]) * t + derivative[2]); 
}

// this can only return a single root (and that it must return that root)...
float OpMath::CubicRoot(OpCubicFloatType A, OpCubicFloatType B, OpCubicFloatType C, 
		OpCubicFloatType D) {
	float result = 0;
	OpRoots quadRoots;
	if (0 == A) {
		quadRoots = QuadRootsDouble((float) B, (float) C, (float) D);
		quadRoots = quadRoots.keepValidTs();
		OP_ASSERT(1 >= quadRoots.count());
		if (quadRoots.count())
			result = quadRoots.get(0);
	} else if (0 == D)
		result = 0;
	else if (0 == A + B + C + D)
		result = 1;
	else {
		OpCubicFloatType coefficient[] = { A, B, C, D };
		OpCubicFloatType derivative[] = { 3 * A, 2 * B, C };
		float midT = .5;
		int tries = 0;
		do {
			float testT = midT - cubicXAtT(coefficient, midT) / cubicDxAtT(derivative, midT);
            if (!OpMath::IsFinite(testT)) {
#if 0 && OP_DEBUG
	// This will fail when the cubic is nearly flat on the axis, because multiple values are
	// valid for zero-crossing. For now, comment this test out unless some future failure
	// suggests it is worth while maintaining this
                OpRoots roots = OpMath::CubicRootsReal(A, B, C, D, MatchEnds::none);
                OP_ASSERT(roots.empty() ? 0 == midT : roots.get(0) == midT);
#endif
                return midT;
            }
			testT = std::max(std::min(testT, 1.f), 0.f);
			if (std::abs(midT - testT) <= OpEpsilon) {
				result = testT;
				break;
			}
			midT = testT;
		} while (++tries < 16);
        OP_ASSERT(OpMath::IsFinite(midT));
		result = midT;
	}
	return result;
}

