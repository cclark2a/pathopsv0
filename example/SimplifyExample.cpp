// (c) 2023, Cary Clark cclark2@gmail.com

/* Given a pair of overlapping loops, compute the filled path which encloses exactly once. 
   The loops are added using a clockwise winding rule; if the accumulated winding is non-zero,
   the loop describes a fill. The coordinates of the simplified path are output to the console. */

#include "curves/Line.h"  // rules for lines
#include "curves/QuadBezier.h"  // rules for quadratic Beziers
#include "curves/UnaryWinding.h"  // rules for single operand w/ winding (e.g. Simplify)

using namespace PathOpsV0Lib;  // pathopsV0 public interfaces are here

constexpr CurveType lineType = 1;  // caller curve type for lines
constexpr CurveType quadType = 2;  // caller curve type for quadratic Beziers
constexpr size_t lineSize = sizeof(OpPoint) * 2;  // caller size of line
constexpr size_t quadSize = sizeof(OpPoint) * 3;  // caller size of quadratic Bezier

// Called by the engine for each output curve. Write the curve description to the console
static void pathOutput(Curve c, bool firstPt, bool lastPt, PathOutput ) {
    if (firstPt)
        OpDebugOut("contour start --\n");  // portable printf() for std::string
    std::string outStr = lineType == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");  // first point of line or quadratic
    if (quadType == c.type)
        addPtStr(quadControlPt(c), ", ");  // control point of quadratic
    addPtStr(c.data->end, "\n");  // last point of line or quadratic
    OpDebugOut(outStr);
    if (lastPt)
        OpDebugOut("-- contour end\n");
}

// Given points describing a pair of closed loops with quadratic Beziers, find their intersection
void SimplifyExample() {
    Context* context = unaryContext(pathOutput);  // an instance of the pathopsv0 engine
    lineCallbacks(context, lineType);  // add the rules to handle lines
    quadCallbacks(context, quadType);  // add the rules to handle quadratic Beziers
    UnaryWinding winding(context);  // winding value for each line or quad (each contributes '1')

    // note that the data below omits start points for curves that match the previous end point
                      //  start     control     end
    OpPoint contour1[] { { 2, 0 }, { 0, 2 }, { 1, 2 },  // quad: start, control, end
                                             { 2, 3 },  // line:                 end
                                             { 2, 0 },  // line:                 end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(winding.contour, { context, &contour1[0], quadSize, quadType } );  // add curve to loop
    AddLine( winding.contour, { context, &contour1[2], lineSize, lineType } );
    AddLine( winding.contour, { context, &contour1[3], lineSize, lineType } );

    OpPoint contour2[] { { 0, 0 },           { 1, 1 },  // line: start,          end
                                   { 1, 3 }, { 0, 3 },  // quad:        control, end
                                             { 0, 0 },  // line:                 end
    };
    AddLine( winding.contour, { context, &contour2[0], lineSize, lineType } );  // add to second loop
    AddQuads(winding.contour, { context, &contour2[1], quadSize, quadType } );
    AddLine( winding.contour, { context, &contour2[3], lineSize, lineType } );

    Resolve(context, nullptr);  // compute the output; for each curve, call pathOutput()
    DeleteContext(context);  // release memory allocated by context and contour
}

OP_TINY_MAIN(SimplifyExample)  // main() for cmake
