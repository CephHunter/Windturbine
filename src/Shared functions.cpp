#include <Shared functions.h>

String addTrailingSpaces(String text, int TotalLength) {
    String res = text;
    int blocksToAdd = TotalLength - text.length();
    if (blocksToAdd < 0) {blocksToAdd = 0;}
    for (uint8_t i = 0; i < blocksToAdd; i++) {
        res += " ";
    }
    return res;
}

double clip(double n, double lower, double upper) {
    return max(lower, min(n, upper));
}

double roundDepth(double digit, int precision) {
    return round(digit * pow(10, precision)) / pow(10, precision);
}