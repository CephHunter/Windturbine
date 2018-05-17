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

long clip(long n, long lower, long upper) {
    return max(lower, min(n, upper));
}

double roundDepth(double digit, int precision) {
    return round(digit * pow(10, precision)) / pow(10, precision);
}

double stringToDouble(char* str) {
    double number = 0;
    bool isnegative = false;
    int decimalpos = -1;
    int stringlength = strlen(str);
    for (int pos = 0; pos < stringlength; pos++) {
        char strpart = str[pos];

        if (strpart == 45 && pos == 0) {
            isnegative = true;
        } else if (strpart == 46 && decimalpos == -1) {
            decimalpos = pos;
        } else if (48 <= strpart && strpart <= 57) {
            number *= 10;
            number += strpart - 48;
        } else {
            return 0;
        }
    }

    if (decimalpos != -1) {
        number /= pow(10, stringlength - decimalpos - 1);
    }
    if (isnegative) {
        number *= -1;
    }
    return number;
}

int stringToInt(char* str) {
    return (int)stringToDouble(str);
}

uint32_t stringIdentifier(char* str) {
    uint32_t res = 0;
    int stringlength = strlen(str);
    for (int pos = 0; pos < stringlength; pos++) {
        char strpart = str[pos];
        if ((strpart >= 65 && strpart <= 90) || (strpart >= 97 && strpart <= 122)) {
            res *= 10;
            res += strpart;
        } else {
            return res;
        }
    }

    return res;
}

// void sendRFMessage(String message, int rID) {
//     char message_out[64];
//     message.toCharArray(message_out, 64);
//     int stringlength = strlen(message_out);
//     connection.receiverID = rID;
//     IPControl_Write(&connection, message_out, stream, stringlength);
// }