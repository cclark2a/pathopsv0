// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"
#include "OpDebugImage.h"
#include <algorithm>

#if OP_DEBUG_IMAGE

enum class DebugGen {
    none,
    prefix,
    body,
    postfix
};

void OpDebugGenerateColorFiles() {
    if (!GENERATE_COLOR_FILES)
        return;
    DebugGen colorH = DebugGen::none;
    DebugGen colorCpp = DebugGen::none;
	FILE* colorFile = fopen("../../../example/OpDebugColor.txt", "r");
	FILE* headerFile = fopen("../../../example/xOpDebugColor.h", "w");
    FILE* sourceFile = fopen("../../../example/xOpDebugColor.cpp", "w");
    std::vector<std::string> colors;
    std::vector<uint32_t> colorValues;
    do {
	    char str[255];
        char* result = fgets(str, sizeof(str), colorFile);
        if (!result)
            break;
        if (str[0] == '$') {
            std::string s(str);
            if (s == "$ prefix for OpDebugColor.h\n") {
                colorH = DebugGen::prefix;
                colorCpp = DebugGen::none;
            } else if (s == "$ prefix for OpDebugColor.cpp\n") {
                colorH = DebugGen::none;
                colorCpp = DebugGen::prefix;
            } else if (s == "$ postfix for OpDebugColor.h\n") {
                colorH = DebugGen::postfix;
                colorCpp = DebugGen::none;
            } else if (s == "$ postfix for OpDebugColor.cpp\n") {
                colorH = DebugGen::none;
                colorCpp = DebugGen::postfix;
            } else if (s == "$ list of colors\n") {
                for (auto c : colors)
                    fprintf(sourceFile, "%s\n", c.c_str());
            } else {
                colorH = DebugGen::body;
                colorCpp = DebugGen::body;
            }
            continue;
        }
        if (DebugGen::prefix == colorH || DebugGen::postfix == colorH) {
            fprintf(headerFile, "%s", str);
            continue;
        }
        if (DebugGen::prefix == colorCpp || DebugGen::postfix == colorCpp) {
            fprintf(sourceFile, "%s", str);
            continue;
        }
        if ('\n' == str[0])
            continue;
        std::vector<std::string> names;
        std::string name;
        std::string color;
        bool expectHex = false;
        for (char c = *result; '\0' != (c = *result); ++result) {
            OP_ASSERT('\0' != c);
            if ((' ' == c || '\t' == c) && name.size()) {
                names.push_back(name);
                name = "";
            } else if ('a' <= c && c <= 'z') {
                OP_ASSERT(!expectHex);
                name += c;
            } else if ('#' == c) {
                expectHex = true;
                OP_ASSERT(!name.size());
            } else if (('0' <= c && c <= '9') || ('A' <= c && c <= 'F')) {
                OP_ASSERT(expectHex);
                OP_ASSERT(color.size() < 6);
                color += c;
            }
        }
        OP_ASSERT(names.size());
        OP_ASSERT(expectHex);
        OP_ASSERT(6 == color.size());
        name = "";
        for (auto s : names)
            name += s;
        uint32_t colorValue = std::strtoul(color.c_str(), nullptr, 16);
        if (colorValues.end() == std::find(colorValues.begin(), colorValues.end(), colorValue)) {
            std::string arrayEntry = "    0xFF" + color + (("white" != name) ? "," : " ") 
                    + std::string(" // ") + name;
            colors.push_back(arrayEntry);
            colorValues.push_back(colorValue);
        }
        fprintf(headerFile, "extern uint32_t %s;\n", name.c_str());
        fprintf(sourceFile, "uint32_t %s = 0xFF%s;\n", name.c_str(), color.c_str());
        // upper case
        std::string upper = name;
        upper[0] -= 'a' - 'A';
        OP_ASSERT('A' <= upper[0] && upper[0] <= 'Z');
        fprintf(headerFile, "extern uint32_t %s;\n", upper.c_str());
        fprintf(sourceFile, "uint32_t %s = 0xFF%s;\n", upper.c_str(), color.c_str());
        if (1 == names.size())
            continue;
        // do camelCase
        name = names[0];
        for (size_t index = 1; index < names.size(); ++index) {
            name += std::toupper(names[index][0]);
            name += names[index].substr(1);
        }
        fprintf(headerFile, "extern uint32_t %s;\n", name.c_str());
        fprintf(sourceFile, "uint32_t %s = 0xFF%s;\n", name.c_str(), color.c_str());
        // do CamelCase
        name = "";
        for (auto s : names) {
            name += std::toupper(s[0]);
            name += s.substr(1);
        }
        fprintf(headerFile, "extern uint32_t %s;\n", name.c_str());
        fprintf(sourceFile, "uint32_t %s = 0xFF%s;\n", name.c_str(), color.c_str());
        // do two_or_more
        name = names[0];
        for (size_t index = 1; index < names.size(); ++index) {
            name += '_';
            name += names[index];
        }
        fprintf(headerFile, "extern uint32_t %s;\n", name.c_str());
        fprintf(sourceFile, "uint32_t %s = 0xFF%s;\n", name.c_str(), color.c_str());
    } while (!feof(colorFile));
    fclose(colorFile);
    fclose(headerFile);
    fclose(sourceFile);
    exit(0);
}

#endif