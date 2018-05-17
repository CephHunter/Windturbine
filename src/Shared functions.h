#include <arduino.h>

String addTrailingSpaces(String text, int TotalLength = 16);
long clip(long n, long lower, long upper);
double roundDepth(double digit, int precision = 0);
double stringToDouble(char* str);
int stringToInt(char* str);
uint32_t stringIdentifier(char* str);
// void sendRFMessage(String message, int rID);