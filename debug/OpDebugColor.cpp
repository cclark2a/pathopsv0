// (c) 2023, Cary Clark cclark2@gmail.com
// This file is generated. Edit OpDebugColor.txt instead.

#include "OpDebugColor.h"

#if OP_DEBUG_IMAGE

#define OP_X(Thing) \
bool color##Thing##On = false; \
uint32_t color##Thing##Color = 0xFF000000; 
COLOR_LIST
#undef OP_X
int colorID;
uint32_t colorIDColor;
uint32_t OP_DEBUG_MULTICOLORED = 0xAbeBeBad;

uint32_t maroon = 0xFF800000;
uint32_t Maroon = 0xFF800000;
uint32_t darkred = 0xFF8B0000;
uint32_t Darkred = 0xFF8B0000;
uint32_t darkRed = 0xFF8B0000;
uint32_t DarkRed = 0xFF8B0000;
uint32_t dark_red = 0xFF8B0000;
uint32_t brown = 0xFFA52A2A;
uint32_t Brown = 0xFFA52A2A;
uint32_t firebrick = 0xFFB22222;
uint32_t Firebrick = 0xFFB22222;
uint32_t crimson = 0xFFDC143C;
uint32_t Crimson = 0xFFDC143C;
uint32_t red = 0xFFFF0000;
uint32_t Red = 0xFFFF0000;
uint32_t tomato = 0xFFFF6347;
uint32_t Tomato = 0xFFFF6347;
uint32_t coral = 0xFFFF7F50;
uint32_t Coral = 0xFFFF7F50;
uint32_t indianred = 0xFFCD5C5C;
uint32_t Indianred = 0xFFCD5C5C;
uint32_t indianRed = 0xFFCD5C5C;
uint32_t IndianRed = 0xFFCD5C5C;
uint32_t indian_red = 0xFFCD5C5C;
uint32_t lightcoral = 0xFFF08080;
uint32_t Lightcoral = 0xFFF08080;
uint32_t lightCoral = 0xFFF08080;
uint32_t LightCoral = 0xFFF08080;
uint32_t light_coral = 0xFFF08080;
uint32_t darksalmon = 0xFFE9967A;
uint32_t Darksalmon = 0xFFE9967A;
uint32_t darkSalmon = 0xFFE9967A;
uint32_t DarkSalmon = 0xFFE9967A;
uint32_t dark_salmon = 0xFFE9967A;
uint32_t salmon = 0xFFFA8072;
uint32_t Salmon = 0xFFFA8072;
uint32_t lightsalmon = 0xFFFFA07A;
uint32_t Lightsalmon = 0xFFFFA07A;
uint32_t lightSalmon = 0xFFFFA07A;
uint32_t LightSalmon = 0xFFFFA07A;
uint32_t light_salmon = 0xFFFFA07A;
uint32_t orangered = 0xFFFF4500;
uint32_t Orangered = 0xFFFF4500;
uint32_t orangeRed = 0xFFFF4500;
uint32_t OrangeRed = 0xFFFF4500;
uint32_t orange_red = 0xFFFF4500;
uint32_t darkorange = 0xFFFF8C00;
uint32_t Darkorange = 0xFFFF8C00;
uint32_t darkOrange = 0xFFFF8C00;
uint32_t DarkOrange = 0xFFFF8C00;
uint32_t dark_orange = 0xFFFF8C00;
uint32_t orange = 0xFFFFA500;
uint32_t Orange = 0xFFFFA500;
uint32_t gold = 0xFFFFD700;
uint32_t Gold = 0xFFFFD700;
uint32_t darkgoldenrod = 0xFFB8860B;
uint32_t Darkgoldenrod = 0xFFB8860B;
uint32_t darkGoldenRod = 0xFFB8860B;
uint32_t DarkGoldenRod = 0xFFB8860B;
uint32_t dark_golden_rod = 0xFFB8860B;
uint32_t goldenrod = 0xFFDAA520;
uint32_t Goldenrod = 0xFFDAA520;
uint32_t goldenRod = 0xFFDAA520;
uint32_t GoldenRod = 0xFFDAA520;
uint32_t golden_rod = 0xFFDAA520;
uint32_t palegoldenrod = 0xFFEEE8AA;
uint32_t Palegoldenrod = 0xFFEEE8AA;
uint32_t paleGoldenRod = 0xFFEEE8AA;
uint32_t PaleGoldenRod = 0xFFEEE8AA;
uint32_t pale_golden_rod = 0xFFEEE8AA;
uint32_t darkkhaki = 0xFFBDB76B;
uint32_t Darkkhaki = 0xFFBDB76B;
uint32_t darkKhaki = 0xFFBDB76B;
uint32_t DarkKhaki = 0xFFBDB76B;
uint32_t dark_khaki = 0xFFBDB76B;
uint32_t khaki = 0xFFF0E68C;
uint32_t Khaki = 0xFFF0E68C;
uint32_t olive = 0xFF808000;
uint32_t Olive = 0xFF808000;
uint32_t yellow = 0xFFFFFF00;
uint32_t Yellow = 0xFFFFFF00;
uint32_t yellowgreen = 0xFF9ACD32;
uint32_t Yellowgreen = 0xFF9ACD32;
uint32_t yellowGreen = 0xFF9ACD32;
uint32_t YellowGreen = 0xFF9ACD32;
uint32_t yellow_green = 0xFF9ACD32;
uint32_t darkolivegreen = 0xFF556B2F;
uint32_t Darkolivegreen = 0xFF556B2F;
uint32_t darkOliveGreen = 0xFF556B2F;
uint32_t DarkOliveGreen = 0xFF556B2F;
uint32_t dark_olive_green = 0xFF556B2F;
uint32_t olivedrab = 0xFF6B8E23;
uint32_t Olivedrab = 0xFF6B8E23;
uint32_t oliveDrab = 0xFF6B8E23;
uint32_t OliveDrab = 0xFF6B8E23;
uint32_t olive_drab = 0xFF6B8E23;
uint32_t lawngreen = 0xFF7CFC00;
uint32_t Lawngreen = 0xFF7CFC00;
uint32_t lawnGreen = 0xFF7CFC00;
uint32_t LawnGreen = 0xFF7CFC00;
uint32_t lawn_green = 0xFF7CFC00;
uint32_t chartreuse = 0xFF7FFF00;
uint32_t Chartreuse = 0xFF7FFF00;
uint32_t greenyellow = 0xFFADFF2F;
uint32_t Greenyellow = 0xFFADFF2F;
uint32_t greenYellow = 0xFFADFF2F;
uint32_t GreenYellow = 0xFFADFF2F;
uint32_t green_yellow = 0xFFADFF2F;
uint32_t darkgreen = 0xFF006400;
uint32_t Darkgreen = 0xFF006400;
uint32_t darkGreen = 0xFF006400;
uint32_t DarkGreen = 0xFF006400;
uint32_t dark_green = 0xFF006400;
uint32_t green = 0xFF008000;
uint32_t Green = 0xFF008000;
uint32_t forestgreen = 0xFF228B22;
uint32_t Forestgreen = 0xFF228B22;
uint32_t forestGreen = 0xFF228B22;
uint32_t ForestGreen = 0xFF228B22;
uint32_t forest_green = 0xFF228B22;
uint32_t lime = 0xFF00FF00;
uint32_t Lime = 0xFF00FF00;
uint32_t limegreen = 0xFF32CD32;
uint32_t Limegreen = 0xFF32CD32;
uint32_t limeGreen = 0xFF32CD32;
uint32_t LimeGreen = 0xFF32CD32;
uint32_t lime_green = 0xFF32CD32;
uint32_t lightgreen = 0xFF90EE90;
uint32_t Lightgreen = 0xFF90EE90;
uint32_t lightGreen = 0xFF90EE90;
uint32_t LightGreen = 0xFF90EE90;
uint32_t light_green = 0xFF90EE90;
uint32_t palegreen = 0xFF98FB98;
uint32_t Palegreen = 0xFF98FB98;
uint32_t paleGreen = 0xFF98FB98;
uint32_t PaleGreen = 0xFF98FB98;
uint32_t pale_green = 0xFF98FB98;
uint32_t darkseagreen = 0xFF8FBC8F;
uint32_t Darkseagreen = 0xFF8FBC8F;
uint32_t darkSeaGreen = 0xFF8FBC8F;
uint32_t DarkSeaGreen = 0xFF8FBC8F;
uint32_t dark_sea_green = 0xFF8FBC8F;
uint32_t mediumspringgreen = 0xFF00FA9A;
uint32_t Mediumspringgreen = 0xFF00FA9A;
uint32_t mediumSpringGreen = 0xFF00FA9A;
uint32_t MediumSpringGreen = 0xFF00FA9A;
uint32_t medium_spring_green = 0xFF00FA9A;
uint32_t springgreen = 0xFF00FF7F;
uint32_t Springgreen = 0xFF00FF7F;
uint32_t springGreen = 0xFF00FF7F;
uint32_t SpringGreen = 0xFF00FF7F;
uint32_t spring_green = 0xFF00FF7F;
uint32_t seagreen = 0xFF2E8B57;
uint32_t Seagreen = 0xFF2E8B57;
uint32_t seaGreen = 0xFF2E8B57;
uint32_t SeaGreen = 0xFF2E8B57;
uint32_t sea_green = 0xFF2E8B57;
uint32_t mediumaquamarine = 0xFF66CDAA;
uint32_t Mediumaquamarine = 0xFF66CDAA;
uint32_t mediumAquaMarine = 0xFF66CDAA;
uint32_t MediumAquaMarine = 0xFF66CDAA;
uint32_t medium_aqua_marine = 0xFF66CDAA;
uint32_t mediumseagreen = 0xFF3CB371;
uint32_t Mediumseagreen = 0xFF3CB371;
uint32_t mediumSeaGreen = 0xFF3CB371;
uint32_t MediumSeaGreen = 0xFF3CB371;
uint32_t medium_sea_green = 0xFF3CB371;
uint32_t lightseagreen = 0xFF20B2AA;
uint32_t Lightseagreen = 0xFF20B2AA;
uint32_t lightSeaGreen = 0xFF20B2AA;
uint32_t LightSeaGreen = 0xFF20B2AA;
uint32_t light_sea_green = 0xFF20B2AA;
uint32_t darkslategray = 0xFF2F4F4F;
uint32_t Darkslategray = 0xFF2F4F4F;
uint32_t darkSlateGray = 0xFF2F4F4F;
uint32_t DarkSlateGray = 0xFF2F4F4F;
uint32_t dark_slate_gray = 0xFF2F4F4F;
uint32_t teal = 0xFF008080;
uint32_t Teal = 0xFF008080;
uint32_t darkcyan = 0xFF008B8B;
uint32_t Darkcyan = 0xFF008B8B;
uint32_t darkCyan = 0xFF008B8B;
uint32_t DarkCyan = 0xFF008B8B;
uint32_t dark_cyan = 0xFF008B8B;
uint32_t aqua = 0xFF00FFFF;
uint32_t Aqua = 0xFF00FFFF;
uint32_t cyan = 0xFF00FFFF;
uint32_t Cyan = 0xFF00FFFF;
uint32_t lightcyan = 0xFFE0FFFF;
uint32_t Lightcyan = 0xFFE0FFFF;
uint32_t lightCyan = 0xFFE0FFFF;
uint32_t LightCyan = 0xFFE0FFFF;
uint32_t light_cyan = 0xFFE0FFFF;
uint32_t darkturquoise = 0xFF00CED1;
uint32_t Darkturquoise = 0xFF00CED1;
uint32_t darkTurquoise = 0xFF00CED1;
uint32_t DarkTurquoise = 0xFF00CED1;
uint32_t dark_turquoise = 0xFF00CED1;
uint32_t turquoise = 0xFF40E0D0;
uint32_t Turquoise = 0xFF40E0D0;
uint32_t mediumturquoise = 0xFF48D1CC;
uint32_t Mediumturquoise = 0xFF48D1CC;
uint32_t mediumTurquoise = 0xFF48D1CC;
uint32_t MediumTurquoise = 0xFF48D1CC;
uint32_t medium_turquoise = 0xFF48D1CC;
uint32_t paleturquoise = 0xFFAFEEEE;
uint32_t Paleturquoise = 0xFFAFEEEE;
uint32_t paleTurquoise = 0xFFAFEEEE;
uint32_t PaleTurquoise = 0xFFAFEEEE;
uint32_t pale_turquoise = 0xFFAFEEEE;
uint32_t aquamarine = 0xFF7FFFD4;
uint32_t Aquamarine = 0xFF7FFFD4;
uint32_t aquaMarine = 0xFF7FFFD4;
uint32_t AquaMarine = 0xFF7FFFD4;
uint32_t aqua_marine = 0xFF7FFFD4;
uint32_t powderblue = 0xFFB0E0E6;
uint32_t Powderblue = 0xFFB0E0E6;
uint32_t powderBlue = 0xFFB0E0E6;
uint32_t PowderBlue = 0xFFB0E0E6;
uint32_t powder_blue = 0xFFB0E0E6;
uint32_t cadetblue = 0xFF5F9EA0;
uint32_t Cadetblue = 0xFF5F9EA0;
uint32_t cadetBlue = 0xFF5F9EA0;
uint32_t CadetBlue = 0xFF5F9EA0;
uint32_t cadet_blue = 0xFF5F9EA0;
uint32_t steelblue = 0xFF4682B4;
uint32_t Steelblue = 0xFF4682B4;
uint32_t steelBlue = 0xFF4682B4;
uint32_t SteelBlue = 0xFF4682B4;
uint32_t steel_blue = 0xFF4682B4;
uint32_t cornflowerblue = 0xFF6495ED;
uint32_t Cornflowerblue = 0xFF6495ED;
uint32_t cornFlowerBlue = 0xFF6495ED;
uint32_t CornFlowerBlue = 0xFF6495ED;
uint32_t corn_flower_blue = 0xFF6495ED;
uint32_t deepskyblue = 0xFF00BFFF;
uint32_t Deepskyblue = 0xFF00BFFF;
uint32_t deepSkyBlue = 0xFF00BFFF;
uint32_t DeepSkyBlue = 0xFF00BFFF;
uint32_t deep_sky_blue = 0xFF00BFFF;
uint32_t dodgerblue = 0xFF1E90FF;
uint32_t Dodgerblue = 0xFF1E90FF;
uint32_t dodgerBlue = 0xFF1E90FF;
uint32_t DodgerBlue = 0xFF1E90FF;
uint32_t dodger_blue = 0xFF1E90FF;
uint32_t lightblue = 0xFFADD8E6;
uint32_t Lightblue = 0xFFADD8E6;
uint32_t lightBlue = 0xFFADD8E6;
uint32_t LightBlue = 0xFFADD8E6;
uint32_t light_blue = 0xFFADD8E6;
uint32_t skyblue = 0xFF87CEEB;
uint32_t Skyblue = 0xFF87CEEB;
uint32_t skyBlue = 0xFF87CEEB;
uint32_t SkyBlue = 0xFF87CEEB;
uint32_t sky_blue = 0xFF87CEEB;
uint32_t lightskyblue = 0xFF87CEFA;
uint32_t Lightskyblue = 0xFF87CEFA;
uint32_t lightSkyBlue = 0xFF87CEFA;
uint32_t LightSkyBlue = 0xFF87CEFA;
uint32_t light_sky_blue = 0xFF87CEFA;
uint32_t midnightblue = 0xFF191970;
uint32_t Midnightblue = 0xFF191970;
uint32_t midnightBlue = 0xFF191970;
uint32_t MidnightBlue = 0xFF191970;
uint32_t midnight_blue = 0xFF191970;
uint32_t navy = 0xFF000080;
uint32_t Navy = 0xFF000080;
uint32_t darkblue = 0xFF00008B;
uint32_t Darkblue = 0xFF00008B;
uint32_t darkBlue = 0xFF00008B;
uint32_t DarkBlue = 0xFF00008B;
uint32_t dark_blue = 0xFF00008B;
uint32_t mediumblue = 0xFF0000CD;
uint32_t Mediumblue = 0xFF0000CD;
uint32_t mediumBlue = 0xFF0000CD;
uint32_t MediumBlue = 0xFF0000CD;
uint32_t medium_blue = 0xFF0000CD;
uint32_t blue = 0xFF0000FF;
uint32_t Blue = 0xFF0000FF;
uint32_t royalblue = 0xFF4169E1;
uint32_t Royalblue = 0xFF4169E1;
uint32_t royalBlue = 0xFF4169E1;
uint32_t RoyalBlue = 0xFF4169E1;
uint32_t royal_blue = 0xFF4169E1;
uint32_t blueviolet = 0xFF8A2BE2;
uint32_t Blueviolet = 0xFF8A2BE2;
uint32_t blueViolet = 0xFF8A2BE2;
uint32_t BlueViolet = 0xFF8A2BE2;
uint32_t blue_violet = 0xFF8A2BE2;
uint32_t indigo = 0xFF4B0082;
uint32_t Indigo = 0xFF4B0082;
uint32_t darkslateblue = 0xFF483D8B;
uint32_t Darkslateblue = 0xFF483D8B;
uint32_t darkSlateBlue = 0xFF483D8B;
uint32_t DarkSlateBlue = 0xFF483D8B;
uint32_t dark_slate_blue = 0xFF483D8B;
uint32_t slateblue = 0xFF6A5ACD;
uint32_t Slateblue = 0xFF6A5ACD;
uint32_t slateBlue = 0xFF6A5ACD;
uint32_t SlateBlue = 0xFF6A5ACD;
uint32_t slate_blue = 0xFF6A5ACD;
uint32_t mediumslateblue = 0xFF7B68EE;
uint32_t Mediumslateblue = 0xFF7B68EE;
uint32_t mediumSlateBlue = 0xFF7B68EE;
uint32_t MediumSlateBlue = 0xFF7B68EE;
uint32_t medium_slate_blue = 0xFF7B68EE;
uint32_t mediumpurple = 0xFF9370DB;
uint32_t Mediumpurple = 0xFF9370DB;
uint32_t mediumPurple = 0xFF9370DB;
uint32_t MediumPurple = 0xFF9370DB;
uint32_t medium_purple = 0xFF9370DB;
uint32_t darkmagenta = 0xFF8B008B;
uint32_t Darkmagenta = 0xFF8B008B;
uint32_t darkMagenta = 0xFF8B008B;
uint32_t DarkMagenta = 0xFF8B008B;
uint32_t dark_magenta = 0xFF8B008B;
uint32_t darkviolet = 0xFF9400D3;
uint32_t Darkviolet = 0xFF9400D3;
uint32_t darkViolet = 0xFF9400D3;
uint32_t DarkViolet = 0xFF9400D3;
uint32_t dark_violet = 0xFF9400D3;
uint32_t darkorchid = 0xFF9932CC;
uint32_t Darkorchid = 0xFF9932CC;
uint32_t darkOrchid = 0xFF9932CC;
uint32_t DarkOrchid = 0xFF9932CC;
uint32_t dark_orchid = 0xFF9932CC;
uint32_t mediumorchid = 0xFFBA55D3;
uint32_t Mediumorchid = 0xFFBA55D3;
uint32_t mediumOrchid = 0xFFBA55D3;
uint32_t MediumOrchid = 0xFFBA55D3;
uint32_t medium_orchid = 0xFFBA55D3;
uint32_t purple = 0xFF800080;
uint32_t Purple = 0xFF800080;
uint32_t thistle = 0xFFD8BFD8;
uint32_t Thistle = 0xFFD8BFD8;
uint32_t plum = 0xFFDDA0DD;
uint32_t Plum = 0xFFDDA0DD;
uint32_t violet = 0xFFEE82EE;
uint32_t Violet = 0xFFEE82EE;
uint32_t fuchsia = 0xFFFF00FF;
uint32_t Fuchsia = 0xFFFF00FF;
uint32_t magenta = 0xFFFF00FF;
uint32_t Magenta = 0xFFFF00FF;
uint32_t orchid = 0xFFDA70D6;
uint32_t Orchid = 0xFFDA70D6;
uint32_t mediumvioletred = 0xFFC71585;
uint32_t Mediumvioletred = 0xFFC71585;
uint32_t mediumVioletRed = 0xFFC71585;
uint32_t MediumVioletRed = 0xFFC71585;
uint32_t medium_violet_red = 0xFFC71585;
uint32_t palevioletred = 0xFFDB7093;
uint32_t Palevioletred = 0xFFDB7093;
uint32_t paleVioletRed = 0xFFDB7093;
uint32_t PaleVioletRed = 0xFFDB7093;
uint32_t pale_violet_red = 0xFFDB7093;
uint32_t deeppink = 0xFFFF1493;
uint32_t Deeppink = 0xFFFF1493;
uint32_t deepPink = 0xFFFF1493;
uint32_t DeepPink = 0xFFFF1493;
uint32_t deep_pink = 0xFFFF1493;
uint32_t hotpink = 0xFFFF69B4;
uint32_t Hotpink = 0xFFFF69B4;
uint32_t hotPink = 0xFFFF69B4;
uint32_t HotPink = 0xFFFF69B4;
uint32_t hot_pink = 0xFFFF69B4;
uint32_t lightpink = 0xFFFFB6C1;
uint32_t Lightpink = 0xFFFFB6C1;
uint32_t lightPink = 0xFFFFB6C1;
uint32_t LightPink = 0xFFFFB6C1;
uint32_t light_pink = 0xFFFFB6C1;
uint32_t pink = 0xFFFFC0CB;
uint32_t Pink = 0xFFFFC0CB;
uint32_t antiquewhite = 0xFFFAEBD7;
uint32_t Antiquewhite = 0xFFFAEBD7;
uint32_t antiqueWhite = 0xFFFAEBD7;
uint32_t AntiqueWhite = 0xFFFAEBD7;
uint32_t antique_white = 0xFFFAEBD7;
uint32_t beige = 0xFFF5F5DC;
uint32_t Beige = 0xFFF5F5DC;
uint32_t bisque = 0xFFFFE4C4;
uint32_t Bisque = 0xFFFFE4C4;
uint32_t blanchedalmond = 0xFFFFEBCD;
uint32_t Blanchedalmond = 0xFFFFEBCD;
uint32_t blanchedAlmond = 0xFFFFEBCD;
uint32_t BlanchedAlmond = 0xFFFFEBCD;
uint32_t blanched_almond = 0xFFFFEBCD;
uint32_t wheat = 0xFFF5DEB3;
uint32_t Wheat = 0xFFF5DEB3;
uint32_t cornsilk = 0xFFFFF8DC;
uint32_t Cornsilk = 0xFFFFF8DC;
uint32_t cornSilk = 0xFFFFF8DC;
uint32_t CornSilk = 0xFFFFF8DC;
uint32_t corn_silk = 0xFFFFF8DC;
uint32_t lemonchiffon = 0xFFFFFACD;
uint32_t Lemonchiffon = 0xFFFFFACD;
uint32_t lemonChiffon = 0xFFFFFACD;
uint32_t LemonChiffon = 0xFFFFFACD;
uint32_t lemon_chiffon = 0xFFFFFACD;
uint32_t lightgoldenrodyellow = 0xFFFAFAD2;
uint32_t Lightgoldenrodyellow = 0xFFFAFAD2;
uint32_t lightGoldenRodYellow = 0xFFFAFAD2;
uint32_t LightGoldenRodYellow = 0xFFFAFAD2;
uint32_t light_golden_rod_yellow = 0xFFFAFAD2;
uint32_t lightyellow = 0xFFFFFFE0;
uint32_t Lightyellow = 0xFFFFFFE0;
uint32_t lightYellow = 0xFFFFFFE0;
uint32_t LightYellow = 0xFFFFFFE0;
uint32_t light_yellow = 0xFFFFFFE0;
uint32_t saddlebrown = 0xFF8B4513;
uint32_t Saddlebrown = 0xFF8B4513;
uint32_t saddleBrown = 0xFF8B4513;
uint32_t SaddleBrown = 0xFF8B4513;
uint32_t saddle_brown = 0xFF8B4513;
uint32_t sienna = 0xFFA0522D;
uint32_t Sienna = 0xFFA0522D;
uint32_t chocolate = 0xFFD2691E;
uint32_t Chocolate = 0xFFD2691E;
uint32_t peru = 0xFFCD853F;
uint32_t Peru = 0xFFCD853F;
uint32_t sandybrown = 0xFFF4A460;
uint32_t Sandybrown = 0xFFF4A460;
uint32_t sandyBrown = 0xFFF4A460;
uint32_t SandyBrown = 0xFFF4A460;
uint32_t sandy_brown = 0xFFF4A460;
uint32_t burlywood = 0xFFDEB887;
uint32_t Burlywood = 0xFFDEB887;
uint32_t burlyWood = 0xFFDEB887;
uint32_t BurlyWood = 0xFFDEB887;
uint32_t burly_wood = 0xFFDEB887;
uint32_t tanbrown = 0xFFD2B48C;
uint32_t Tanbrown = 0xFFD2B48C;
uint32_t tanBrown = 0xFFD2B48C;
uint32_t TanBrown = 0xFFD2B48C;
uint32_t tan_brown = 0xFFD2B48C;
uint32_t rosybrown = 0xFFBC8F8F;
uint32_t Rosybrown = 0xFFBC8F8F;
uint32_t rosyBrown = 0xFFBC8F8F;
uint32_t RosyBrown = 0xFFBC8F8F;
uint32_t rosy_brown = 0xFFBC8F8F;
uint32_t moccasin = 0xFFFFE4B5;
uint32_t Moccasin = 0xFFFFE4B5;
uint32_t navajowhite = 0xFFFFDEAD;
uint32_t Navajowhite = 0xFFFFDEAD;
uint32_t navajoWhite = 0xFFFFDEAD;
uint32_t NavajoWhite = 0xFFFFDEAD;
uint32_t navajo_white = 0xFFFFDEAD;
uint32_t peachpuff = 0xFFFFDAB9;
uint32_t Peachpuff = 0xFFFFDAB9;
uint32_t peachPuff = 0xFFFFDAB9;
uint32_t PeachPuff = 0xFFFFDAB9;
uint32_t peach_puff = 0xFFFFDAB9;
uint32_t mistyrose = 0xFFFFE4E1;
uint32_t Mistyrose = 0xFFFFE4E1;
uint32_t mistyRose = 0xFFFFE4E1;
uint32_t MistyRose = 0xFFFFE4E1;
uint32_t misty_rose = 0xFFFFE4E1;
uint32_t lavenderblush = 0xFFFFF0F5;
uint32_t Lavenderblush = 0xFFFFF0F5;
uint32_t lavenderBlush = 0xFFFFF0F5;
uint32_t LavenderBlush = 0xFFFFF0F5;
uint32_t lavender_blush = 0xFFFFF0F5;
uint32_t linen = 0xFFFAF0E6;
uint32_t Linen = 0xFFFAF0E6;
uint32_t oldlace = 0xFFFDF5E6;
uint32_t Oldlace = 0xFFFDF5E6;
uint32_t oldLace = 0xFFFDF5E6;
uint32_t OldLace = 0xFFFDF5E6;
uint32_t old_lace = 0xFFFDF5E6;
uint32_t papayawhip = 0xFFFFEFD5;
uint32_t Papayawhip = 0xFFFFEFD5;
uint32_t papayaWhip = 0xFFFFEFD5;
uint32_t PapayaWhip = 0xFFFFEFD5;
uint32_t papaya_whip = 0xFFFFEFD5;
uint32_t seashell = 0xFFFFF5EE;
uint32_t Seashell = 0xFFFFF5EE;
uint32_t seaShell = 0xFFFFF5EE;
uint32_t SeaShell = 0xFFFFF5EE;
uint32_t sea_shell = 0xFFFFF5EE;
uint32_t mintcream = 0xFFF5FFFA;
uint32_t Mintcream = 0xFFF5FFFA;
uint32_t mintCream = 0xFFF5FFFA;
uint32_t MintCream = 0xFFF5FFFA;
uint32_t mint_cream = 0xFFF5FFFA;
uint32_t slategray = 0xFF708090;
uint32_t Slategray = 0xFF708090;
uint32_t slateGray = 0xFF708090;
uint32_t SlateGray = 0xFF708090;
uint32_t slate_gray = 0xFF708090;
uint32_t lightslategray = 0xFF778899;
uint32_t Lightslategray = 0xFF778899;
uint32_t lightSlateGray = 0xFF778899;
uint32_t LightSlateGray = 0xFF778899;
uint32_t light_slate_gray = 0xFF778899;
uint32_t lightsteelblue = 0xFFB0C4DE;
uint32_t Lightsteelblue = 0xFFB0C4DE;
uint32_t lightSteelBlue = 0xFFB0C4DE;
uint32_t LightSteelBlue = 0xFFB0C4DE;
uint32_t light_steel_blue = 0xFFB0C4DE;
uint32_t lavender = 0xFFE6E6FA;
uint32_t Lavender = 0xFFE6E6FA;
uint32_t floralwhite = 0xFFFFFAF0;
uint32_t Floralwhite = 0xFFFFFAF0;
uint32_t floralWhite = 0xFFFFFAF0;
uint32_t FloralWhite = 0xFFFFFAF0;
uint32_t floral_white = 0xFFFFFAF0;
uint32_t aliceblue = 0xFFF0F8FF;
uint32_t Aliceblue = 0xFFF0F8FF;
uint32_t aliceBlue = 0xFFF0F8FF;
uint32_t AliceBlue = 0xFFF0F8FF;
uint32_t alice_blue = 0xFFF0F8FF;
uint32_t ghostwhite = 0xFFF8F8FF;
uint32_t Ghostwhite = 0xFFF8F8FF;
uint32_t ghostWhite = 0xFFF8F8FF;
uint32_t GhostWhite = 0xFFF8F8FF;
uint32_t ghost_white = 0xFFF8F8FF;
uint32_t honeydew = 0xFFF0FFF0;
uint32_t Honeydew = 0xFFF0FFF0;
uint32_t ivory = 0xFFFFFFF0;
uint32_t Ivory = 0xFFFFFFF0;
uint32_t azure = 0xFFF0FFFF;
uint32_t Azure = 0xFFF0FFFF;
uint32_t snow = 0xFFFFFAFA;
uint32_t Snow = 0xFFFFFAFA;
uint32_t black = 0xFF000000;
uint32_t Black = 0xFF000000;
uint32_t dimgray = 0xFF696969;
uint32_t Dimgray = 0xFF696969;
uint32_t dimGray = 0xFF696969;
uint32_t DimGray = 0xFF696969;
uint32_t dim_gray = 0xFF696969;
uint32_t dimgrey = 0xFF696969;
uint32_t Dimgrey = 0xFF696969;
uint32_t dimGrey = 0xFF696969;
uint32_t DimGrey = 0xFF696969;
uint32_t dim_grey = 0xFF696969;
uint32_t gray = 0xFF808080;
uint32_t Gray = 0xFF808080;
uint32_t grey = 0xFF808080;
uint32_t Grey = 0xFF808080;
uint32_t darkgray = 0xFFA9A9A9;
uint32_t Darkgray = 0xFFA9A9A9;
uint32_t darkGray = 0xFFA9A9A9;
uint32_t DarkGray = 0xFFA9A9A9;
uint32_t dark_gray = 0xFFA9A9A9;
uint32_t darkgrey = 0xFFA9A9A9;
uint32_t Darkgrey = 0xFFA9A9A9;
uint32_t darkGrey = 0xFFA9A9A9;
uint32_t DarkGrey = 0xFFA9A9A9;
uint32_t dark_grey = 0xFFA9A9A9;
uint32_t silver = 0xFFC0C0C0;
uint32_t Silver = 0xFFC0C0C0;
uint32_t lightgray = 0xFFD3D3D3;
uint32_t Lightgray = 0xFFD3D3D3;
uint32_t lightGray = 0xFFD3D3D3;
uint32_t LightGray = 0xFFD3D3D3;
uint32_t light_gray = 0xFFD3D3D3;
uint32_t lightgrey = 0xFFD3D3D3;
uint32_t Lightgrey = 0xFFD3D3D3;
uint32_t lightGrey = 0xFFD3D3D3;
uint32_t LightGrey = 0xFFD3D3D3;
uint32_t light_grey = 0xFFD3D3D3;
uint32_t gainsboro = 0xFFDCDCDC;
uint32_t Gainsboro = 0xFFDCDCDC;
uint32_t whitesmoke = 0xFFF5F5F5;
uint32_t Whitesmoke = 0xFFF5F5F5;
uint32_t whiteSmoke = 0xFFF5F5F5;
uint32_t WhiteSmoke = 0xFFF5F5F5;
uint32_t white_smoke = 0xFFF5F5F5;
uint32_t white = 0xFFFFFFFF;
uint32_t White = 0xFFFFFFFF;

std::vector<uint32_t> debugColorArray = {
    0xFF800000, // maroon
    0xFF8B0000, // darkred
    0xFFA52A2A, // brown
    0xFFB22222, // firebrick
    0xFFDC143C, // crimson
    0xFFFF0000, // red
    0xFFFF6347, // tomato
    0xFFFF7F50, // coral
    0xFFCD5C5C, // indianred
    0xFFF08080, // lightcoral
    0xFFE9967A, // darksalmon
    0xFFFA8072, // salmon
    0xFFFFA07A, // lightsalmon
    0xFFFF4500, // orangered
    0xFFFF8C00, // darkorange
    0xFFFFA500, // orange
    0xFFFFD700, // gold
    0xFFB8860B, // darkgoldenrod
    0xFFDAA520, // goldenrod
    0xFFEEE8AA, // palegoldenrod
    0xFFBDB76B, // darkkhaki
    0xFFF0E68C, // khaki
    0xFF808000, // olive
    0xFFFFFF00, // yellow
    0xFF9ACD32, // yellowgreen
    0xFF556B2F, // darkolivegreen
    0xFF6B8E23, // olivedrab
    0xFF7CFC00, // lawngreen
    0xFF7FFF00, // chartreuse
    0xFFADFF2F, // greenyellow
    0xFF006400, // darkgreen
    0xFF008000, // green
    0xFF228B22, // forestgreen
    0xFF00FF00, // lime
    0xFF32CD32, // limegreen
    0xFF90EE90, // lightgreen
    0xFF98FB98, // palegreen
    0xFF8FBC8F, // darkseagreen
    0xFF00FA9A, // mediumspringgreen
    0xFF00FF7F, // springgreen
    0xFF2E8B57, // seagreen
    0xFF66CDAA, // mediumaquamarine
    0xFF3CB371, // mediumseagreen
    0xFF20B2AA, // lightseagreen
    0xFF2F4F4F, // darkslategray
    0xFF008080, // teal
    0xFF008B8B, // darkcyan
    0xFF00FFFF, // aqua
    0xFFE0FFFF, // lightcyan
    0xFF00CED1, // darkturquoise
    0xFF40E0D0, // turquoise
    0xFF48D1CC, // mediumturquoise
    0xFFAFEEEE, // paleturquoise
    0xFF7FFFD4, // aquamarine
    0xFFB0E0E6, // powderblue
    0xFF5F9EA0, // cadetblue
    0xFF4682B4, // steelblue
    0xFF6495ED, // cornflowerblue
    0xFF00BFFF, // deepskyblue
    0xFF1E90FF, // dodgerblue
    0xFFADD8E6, // lightblue
    0xFF87CEEB, // skyblue
    0xFF87CEFA, // lightskyblue
    0xFF191970, // midnightblue
    0xFF000080, // navy
    0xFF00008B, // darkblue
    0xFF0000CD, // mediumblue
    0xFF0000FF, // blue
    0xFF4169E1, // royalblue
    0xFF8A2BE2, // blueviolet
    0xFF4B0082, // indigo
    0xFF483D8B, // darkslateblue
    0xFF6A5ACD, // slateblue
    0xFF7B68EE, // mediumslateblue
    0xFF9370DB, // mediumpurple
    0xFF8B008B, // darkmagenta
    0xFF9400D3, // darkviolet
    0xFF9932CC, // darkorchid
    0xFFBA55D3, // mediumorchid
    0xFF800080, // purple
    0xFFD8BFD8, // thistle
    0xFFDDA0DD, // plum
    0xFFEE82EE, // violet
    0xFFFF00FF, // fuchsia
    0xFFDA70D6, // orchid
    0xFFC71585, // mediumvioletred
    0xFFDB7093, // palevioletred
    0xFFFF1493, // deeppink
    0xFFFF69B4, // hotpink
    0xFFFFB6C1, // lightpink
    0xFFFFC0CB, // pink
    0xFFFAEBD7, // antiquewhite
    0xFFF5F5DC, // beige
    0xFFFFE4C4, // bisque
    0xFFFFEBCD, // blanchedalmond
    0xFFF5DEB3, // wheat
    0xFFFFF8DC, // cornsilk
    0xFFFFFACD, // lemonchiffon
    0xFFFAFAD2, // lightgoldenrodyellow
    0xFFFFFFE0, // lightyellow
    0xFF8B4513, // saddlebrown
    0xFFA0522D, // sienna
    0xFFD2691E, // chocolate
    0xFFCD853F, // peru
    0xFFF4A460, // sandybrown
    0xFFDEB887, // burlywood
    0xFFD2B48C, // tanbrown
    0xFFBC8F8F, // rosybrown
    0xFFFFE4B5, // moccasin
    0xFFFFDEAD, // navajowhite
    0xFFFFDAB9, // peachpuff
    0xFFFFE4E1, // mistyrose
    0xFFFFF0F5, // lavenderblush
    0xFFFAF0E6, // linen
    0xFFFDF5E6, // oldlace
    0xFFFFEFD5, // papayawhip
    0xFFFFF5EE, // seashell
    0xFFF5FFFA, // mintcream
    0xFF708090, // slategray
    0xFF778899, // lightslategray
    0xFFB0C4DE, // lightsteelblue
    0xFFE6E6FA, // lavender
    0xFFFFFAF0, // floralwhite
    0xFFF0F8FF, // aliceblue
    0xFFF8F8FF, // ghostwhite
    0xFFF0FFF0, // honeydew
    0xFFFFFFF0, // ivory
    0xFFF0FFFF, // azure
    0xFFFFFAFA, // snow
    0xFF000000, // black
    0xFF696969, // dimgray
    0xFF808080, // gray
    0xFFA9A9A9, // darkgray
    0xFFC0C0C0, // silver
    0xFFD3D3D3, // lightgray
    0xFFDCDCDC, // gainsboro
    0xFFF5F5F5, // whitesmoke
    0xFFFFFFFF  // white
};

#endif