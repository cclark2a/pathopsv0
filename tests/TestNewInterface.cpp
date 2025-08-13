// (c) 2023, Cary Clark cclark2@gmail.com

#include "curves/QuadBezier.h"
#include "curves/UnaryWinding.h"

// curve types
static PathOpsV0Lib::CurveType lineType = 1;
static PathOpsV0Lib::CurveType quadType = 2;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;

static void pathOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, 
        PathOpsV0Lib::PathOutput output) {
    if (firstPt)
        OpDebugOut("contour start --\n");
    std::string outStr = lineType == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");
    if (quadType == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
    if (lastPt)
        OpDebugOut("-- contour end\n");
}

void SimpleTest() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext();
    SetContextCallbacks(context, { pathOutput } );
    unaryCallbacks(context);

	OP_DEBUG_DUMP_CODE(SetDebugContextCallbacks(context, { nullptr, unaryWindingDumpOutFunc }) );

    SetCurveCallbacks(context, lineType, { } );
    quadCallbacks(context, quadType);

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    UnaryWinding windingData(1);
    Winding winding { &windingData, sizeof(windingData) };
    Contour* contour = CreateContour(context, winding);

    // note that the data below omits start points for curves that match the previous end point
                      //  start      end      control
    OpPoint contour1[] { { 2, 0 }, { 0, 2 }, { 1, 2 },  // quad: start, control, end
                                             { 2, 3 },  // line:                 end
                                             { 2, 0 },  // line:                 end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(contour, { &contour1[0], quadSize, quadType } );
    Add(     contour, { &contour1[2], lineSize, lineType } );
    Add(     contour, { &contour1[3], lineSize, lineType } );

    OpPoint contour2[] { { 0, 0 },           { 1, 1 },  // line: start,          end
                                   { 1, 3 }, { 0, 3 },  // quad:        control, end
                                             { 0, 0 },  // line:                 end
    };
    Add(     contour, { &contour2[0], lineSize, lineType } );
    AddQuads(contour, { &contour2[1], quadSize, quadType } );
    Add(     contour, { &contour2[3], lineSize, lineType } );

	Normalize(context);
    Resolve(context, nullptr);
	ContextError error = Error(context);
    DeleteContext(context);
    if (ContextError::none != error)
        exit(1);
}
