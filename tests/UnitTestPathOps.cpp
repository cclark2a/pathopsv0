// (c) 2023, Cary Clark cclark2@gmail.com
//       1         2         3         4         5         6         7         8         9         0
//34567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#include <cstdlib>
#include "OpTestDrive.h"

// code is placeholder; it's considerably out of date
#if OP_INTERACTIVE
#include "OpDebug.h"

struct PointIndex {
    int index;
    int close;
};

struct PointsVerbs {
    // !!! add way to visualize edge runs
    SkPath makePath() {
        return SkPath::Make(&points.front(), points.size(),
            &verbs.front(), verbs.size(), &weights.front(), weights.size(), fillType);
    }
    
    void set(OpInPath& inPath) {
        SkPath* path = (SkPath*) inPath.externalReference;
        int count = path->countPoints();
        points.resize(count);
        path->getPoints(&points.front(), count);
        count = path->countVerbs();
        verbs.resize(count);
        path->getVerbs(&verbs.front(), count);
        curves = 0;
        fillType = path->getFillType();
        SkPath::RawIter iter(*path);
        SkPath::Verb verb;
        do {
            SkPoint pts[4];
            verb = iter.next(pts);
            curves += SkPath::kQuad_Verb <= verb && verb <= SkPath::kCubic_Verb;
            if (SkPath::kConic_Verb == verb)
                weights.push_back(iter.conicWeight());
        } while (verb != SkPath::kDone_Verb);
    }

    std::vector<SkPoint> points;
    std::vector<uint8_t> verbs;
    std::vector<SkScalar> weights;
    int curves;
    SkPathFillType fillType;
};

PointsVerbs leftPath;
PointsVerbs rightPath;
PointsVerbs* activePtV;

void readFromFile() {
    OpContext* fileContours = fromFile();
    std::swap(fileContours, debugGlobalContext);
    // preserve original contour data so it can be edited/restored later (incomplete)
    resetPaths();
    OpDebugImage::init();
    resetFocus();
    showEdges();
    showIDs();
}

PointIndex activeIndex;
int activeFocus;
int activeLeft;
int activeRight;

	namespace skiatest {
	struct Reporter;
	}

void resetPaths() {
    OP_ASSERT(0); // !!! incomplete
    activeIndex = { -1, -1 };
    activeFocus = -1;
    activeLeft = 0;
    activeRight = 0;
}

PointIndex pointIndex(PointsVerbs& ptVerbs, int curve, int focus) {
    PointIndex result {-1, -1};
    size_t vIndex = 0;
    size_t ptIndex = 0;
    size_t firstInContour = 0;
    int cIndex = curve;
    OP_ASSERT(focus > 0 && focus <= 4);
    while (vIndex < ptVerbs.verbs.size() && cIndex >= 0) {
        SkPath::Verb v = (SkPath::Verb) ptVerbs.verbs[vIndex++];
        if (SkPath::kMove_Verb == v || SkPath::kLine_Verb == v) {
            ++ptIndex;
            continue;
        }
        if (SkPath::kClose_Verb == v || SkPath::kDone_Verb == v) {
            if (firstInContour == (size_t) result.index && 1 < ptIndex) {
                size_t closePtIndex = ptIndex - 1;
                if (ptVerbs.points[firstInContour] == ptVerbs.points[closePtIndex])
                    result.close = closePtIndex;
                break;
            }
            if (SkPath::kDone_Verb == v)
                break;
        }
        if (SkPath::kClose_Verb == v) {
            firstInContour = ptIndex;
            continue;
        }
        int ptCount = 0;
        switch (v) {
            case SkPath::kQuad_Verb:
                ptCount = 3;
            break;
            case SkPath::kConic_Verb:
                ptCount = 3;
            break;
            case SkPath::kCubic_Verb:
                ptCount = 4;
            break;
            default:
                OP_ASSERT(0);
        }
       if (0 == cIndex) {
           if (focus > ptCount)
               return result;
           int offset = focus - 2;  // first point is 1, so -1 is last point from prior 
           OP_ASSERT((int) ptIndex >= offset);
           result.index = ptIndex + offset;
       }
       --cIndex;
       ptIndex += ptCount - 1;
       OP_ASSERT(ptIndex < ptVerbs.points.size());
    }
    return result;
}

void opOnChar(char c) {
    switch (c) {
    case 'b' : toggleBounds(); break;
    case 'c' : toggleControls(); break;
    case 'C' : toggleControlLines(); break;
    case 'd' : adjustCCDepth(+1); break;
    case 'D' : adjustCCDepth(-1); break;
    case 'e' : toggleEdges(); break;
    case 'E' : toggleEndToEnd(); break;
    case 'f' : toggleFill(); break;
    case 'g' : toggleGrid(); break;
    case 'G' : toggleGuides(); break;
    case 'h' : toggleHex(); break;
    case 'H' : toggleHulls(); break;
    case 'i' : toggleIDs(); break;
    case 'I' : toggleIntersections(); break;
    case 'n' : toggleNormals(); break;
    case 'N' : toggleIn(); break;
    case 'o' : togglePathsOut(); break;
    case 'O' : toggleOperands(); break;
    case 'p' : togglePoints(); break;
    case 'r' : resetPaths(); break;  // !!! add way to (probably toggle) original path and current edit
    case 's' : toggleSegments(); break;
    case 't' : toggleTangents(); break;
    case 'v' : toggleValues(); break;
    case 'w' : toggleWindings(); break;
    case '0' : 
        activeFocus = 0; 
//        activePtV = nullptr; 
        activeIndex = { -1, -1 };
        break;
    default:
        if ('1' <= c && c <= '8') {
            activeFocus = c <= '4' ? c - '0' : c - '4';
#if 0
            activePtV = c <= '4' ? &leftPath : &rightPath;
            int activeCurve = c <= '4' ? activeLeft : activeRight;
            activeIndex = pointIndex(*activePtV, activeCurve, activeFocus); 
#endif
        }
        break;
    }
}

// change indicated point. If this point is the first in contour, change last also if same
void setPoint(PointsVerbs& ptVerbs, PointIndex pi, SkPoint pt) {
    OP_ASSERT(0 <= pi.index && pi.index < (int) ptVerbs.points.size());
    OP_ASSERT(-1 == pi.close || (0 < pi.close && pi.close < (int) ptVerbs.points.size()));
    SkPoint oldValue = ptVerbs.points[pi.index];
    OP_ASSERT(-1 == pi.close || oldValue == ptVerbs.points[pi.close]);
    ptVerbs.points[pi.index] = pt;
    if (0 < pi.close)
        ptVerbs.points[pi.close] = pt;
}

// associates 1:4 with first curve; 5:8 with second curve
void movePoint(float x, float y) {
    switch (activeFocus) {
        case 1: break;
        case 2: break;
        case 3: break;
        case 4: break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
    }
    // !!! start here
    // edit skia path
    OP_DEBUG_CODE(int ptIndex = activeFocus % 3);
    OP_ASSERT(ptIndex);  // !!! suppress warning; incomplete
    // regenerate segments, edges, curve/curve data (esp. edge runs, exiting when some depth is reached)
}

void opOnMouse(int lastX, int lastY, int x, int y) {
    int deltaX = x - lastX;
    int deltaY = y - lastY;
    if (deltaX > 0)
        activeFocus <= 0 ? l(.02f) : movePoint(-.02f, 0);
    else if (deltaX < 0)
        activeFocus <= 0 ? r(.02f) : movePoint(.02f, 0);
    if (deltaY > 0)
        activeFocus <= 0 ? u(.02f) : movePoint(0, -.02f);
    else if (deltaY < 0)
        activeFocus <= 0 ? d(.02f) : movePoint(.02f, .02f);
}

bool opWheel(float delta) {
    if (delta > 0)
        i(1);
    else if (delta < 0)
        oo(1);
}

#endif

#define VALIDATE_HTML 0

#if VALIDATE_HTML
#include "OpMath.h"

enum class State {
	none,
	start,
	in,
	angle,
	property,
	equal,
	out,

	sQuote,
	dQuote,

	_class,
	href,
	id,
	src,
	type,

	a,
	br,
	code,
	p,
	pre,
	script,
	table,
	td,
	tr,

};

struct Name {
	const char* n;
	State state;
} names[] = { 
	{"a", State::a}, 
	{"br", State::br}, 
	{"code", State::code}, 
	{"p", State::p},
	{"pre", State::pre},
	{"script", State::script},
	{"table", State::table},
	{"td", State::td},
	{"tr", State::tr},
};

struct Property {
	const char* n;
	State state;
} properties[] = {
	{"class", State::_class},
	{"href", State::href}, 
	{"id", State::id}, 
	{"src", State::src},
	{"type", State::type},
};

struct Tag {
	Name* name;
	const char* s;
	int len;
};

struct StateStr {
	const char* s;
	std::string str;
};

static void setBackState(std::vector<Tag>& tags) {
	OP_ASSERT(tags.size());
	Tag& back = tags.back();
	OP_ASSERT(nullptr == back.name);
	OP_ASSERT('<' == back.s[0]);
	std::string tagName(&back.s[1], back.len - 1);
	for (size_t index = 0; index < ARRAY_COUNT(names); ++index) {
		Name* name = &names[index];
		if (std::string(name->n) == tagName) {
			if (State::br == name->state)
				tags.pop_back();
			else
				back.name = name;
			return;
		}
	}
	OP_ASSERT(0);
};

void validateHTML() {
    std::string buffer;
	const char* name = "D:/ErikSom/Pathopsv0-WASM/pathopsv0-wasm/examples/es/readme.html";
	FILE* html = fopen(name, "rb");
    int seek = fseek(html, 0, SEEK_END);
    OP_ASSERT(!seek);
    long size = ftell(html);
    fclose(html);
    html = fopen(name, "rb");
    buffer.resize(size);
    fread(&buffer[0], 1, size, html);
    fclose(html);
	std::vector<Tag> tags;
	std::vector<StateStr> ids;
	std::vector<StateStr> hrefs;
	State tag = State::none;
	Property* prop = nullptr;
	const char* propStr = nullptr;
	const char* valStr = nullptr;
	const char* outStr = nullptr;
	const char* bodyStr = "<body>";
	const char* s = strstr(&buffer.front(), bodyStr);
	OP_ASSERT(s);
	s += strlen(bodyStr);
	const char* e = &buffer.back() + 1;
	auto setBackProp = [&propStr, &s, &prop]() {
		OP_ASSERT(propStr);
		std::string propName(propStr, s - propStr - 1);
		for (Property property : properties) {
			if (std::string(property.n) == propName) {
				prop = &property;
				return;
			}
		}
		OP_ASSERT(0);
	};
	auto verifyPropValue = [&prop, &valStr, &s, &hrefs, &ids]() {
		std::string val(valStr, s - valStr - 1);
		OP_ASSERT(prop);
		switch (prop->state) {
			case State::_class:
				OP_ASSERT(std::string("c1") == val || std::string("c3") == val);
			break;
			case State::href:
				OP_ASSERT('#' == val[0]);
				hrefs.push_back({valStr, std::string(&valStr[1], val.size() - 1) });
			break; 
			case State::id:
				ids.push_back({valStr, std::string(valStr, val.size()) });
			break; 
			case State::src:
				// don't check for now
			break;
			case State::type:
				OP_ASSERT(std::string("module") == val);
			break;
			default:
				OP_ASSERT(0);
		}
	};
	auto verifyBackState = [&tags, &outStr, &s]() {
		OP_ASSERT(tags.size());
		Tag& back = tags.back();
		std::string out(outStr, s - outStr - 1);
		OP_ASSERT(out == std::string(back.name->n));
		tags.pop_back();
	};
	while (s < e) {
		char c = *s++;
		switch (tag) {
			case State::start:
				if ('/' == c) {
					outStr = s;
					tag = State::out;
					break;
				}
				tag = State::in;
				tags.push_back({nullptr, s - 2, 2});
			break;
			case State::in:
				if ('>' == c) {
					setBackState(tags);
					tag = State::none;
					break;
				}
				OP_ASSERT('<' != c);
				if (' ' == c) {
					setBackState(tags);
					tag = State::angle;
					break;
				}
				OP_ASSERT(isalpha(c));
				tags.back().len += 1;
			break;
			case State::angle:
				OP_ASSERT('<' != c);
				if (' ' == c)
					break;
				if (!propStr && isalpha(c)) {
					propStr = s - 1;
					tag = State::property;
					break;
				}
				OP_ASSERT('>' == c);
				tag = State::none;
			break;
			case State::property:
				if (isalpha(c))
					break;
				OP_ASSERT('=' == c);
				tag = State::equal;
				setBackProp();
				propStr = nullptr;
			break;
			case State::equal:
				if ('\'' == c)
					tag = State::sQuote;
				else if ('"' == c)
					tag = State::dQuote;
				else
					OP_ASSERT(0);
				valStr = s;
			break;
			case State::sQuote:
				if ('\'' == c) {
					verifyPropValue();
					tag = State::angle;
				}
				OP_ASSERT(' ' <= c);
			break;
			case State::dQuote:
				if ('"' == c) {
					verifyPropValue();
					tag = State::angle;
				}
				OP_ASSERT(' ' <= c);
			break;
			case State::out:
				if ('>' == c) {
					if (std::string(outStr, s - outStr - 1) == "body") {
						s = e;
						break;
					}
					verifyBackState();
					tag = State::none;
					break;
				}
				OP_ASSERT(isalpha(c));

			break;
			case State::none:
				if ('<' == c)
					tag = State::start;
			break;
			default:
				OP_ASSERT(0);
		}
	}
	std::sort(ids.begin(), ids.end(), [](const StateStr& a, const StateStr& b) {
		return a.str < b.str;
	});
	std::sort(hrefs.begin(), hrefs.end(), [](const StateStr& a, const StateStr& b) {
		return a.str < b.str;
	});
	// verify that each ID is used only once
	// verify that every href has an ID
	size_t idIndex = 0;
	size_t hrefIndex = 0;
	while (idIndex < ids.size() && hrefIndex < hrefs.size()) {
		while (idIndex < ids.size() && ids[idIndex].str < hrefs[hrefIndex].str)
			++idIndex;
		OP_ASSERT(idIndex < ids.size() && ids[idIndex].str == hrefs[hrefIndex].str);
		OP_ASSERT(idIndex + 1 >= ids.size() || ids[idIndex].str < ids[idIndex + 1].str);
		++idIndex;
		while (hrefIndex + 1 < hrefs.size() 
				&& hrefs[hrefIndex].str == hrefs[hrefIndex + 1].str)
			++hrefIndex;
		++hrefIndex;
	}
	OP_ASSERT(0);
}
#endif

#define TEST_SMALL_EXAMPLES 0

extern void ContainsTest();
extern void testFrame();
extern void TestPath2D(bool debugIt);
extern void SimpleTest();
extern void runTests();

void OpTest(bool terminateEarly) {
#if VALIDATE_HTML
	validateHTML();
#endif
#if 0 && OP_DEBUG_IMAGE
    if (GENERATE_COLOR_FILES) {
        OpDebugGenerateColorFiles();
        return;
    }
#endif
#if TEST_SMALL_EXAMPLES
    ContainsTest();
 	TestPath2D(true);
  	testFrame();
    SimpleTest();
#endif
	runTests();
	if (terminateEarly)
		exit(0);
}
