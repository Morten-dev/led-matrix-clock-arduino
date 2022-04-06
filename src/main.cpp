#include <Arduino.h>
#include <Arduino_Helpers.h>
#include <DFRobot_RGBMatrix.h> // Hardware-specific library
#include <Time.h>
#include <AH/STL/bitset>

#define OE    9
#define LAT   10
#define CLK   11
#define A     A0
#define B     A1
#define C     A2
#define D     A3
#define E     A4
#define WIDTH 64
#define HEIGHT  64


//Point struct
struct point {
    byte x;
    byte y;
};

enum armType {SEC,MIN,HR};

//Preset coordinates for clock face
const point PROGMEM notchPoints[16] = {{46, 6}, {45, 7}, {56, 16}, {55, 17}, {57, 46}, {56, 45}, {47, 56}, {46, 55}, {16, 56}, {17, 55}, {6, 46}, {7, 45}, {6, 17}, {7, 18}, {16, 7}, {17, 8}};
const point PROGMEM numberPoints[73] = { {5, 28}, {6, 28}, {7, 28}, {4, 29}, {8, 29}, {4, 30}, {8, 30}, {5, 31}, {6, 31}, {7, 31}, {8, 31}, {8, 32}, {7, 33}, {6, 34}, {5, 34}, {4, 34}, {31, 53}, {32, 53}, {33, 53}, {30, 54}, {29, 55}, {29, 56}, {30, 56}, {31, 56}, {32, 56}, {29, 57}, {33, 57}, {29, 58}, {33, 58}, {30, 59}, {31, 59}, {32, 59}, {55, 28}, {56, 28}, {57, 28}, {58, 28}, {59, 28}, {59, 29}, {58, 30}, {57, 31}, {58, 31}, {59, 32}, {59, 33}, {58, 34}, {57, 34}, {56, 34}, {55, 33}, {29, 4}, {34, 4}, {35, 4}, {36, 4}, {28, 5}, {29, 5}, {33, 5}, {37, 5}, {29, 6}, {37, 6}, {29, 7}, {34, 7}, {35, 7}, {36, 7}, {29, 8}, {33, 8}, {29, 9}, {33, 9}, {28, 10}, {29, 10}, {30, 10}, {33, 10}, {34, 10}, {35, 10}, {36, 10}, {37, 10} };

std::bitset<64> pixelState[64];

//Center coordinate
point c = {31, 31};

//Lengths of arms
const byte r_sec = 28;  // Length of second-hand
const byte r_min = 28;  // Length of minute-hand
const byte r_hr  = 16;  // Length of hour-hand

//Arrays to store coordinates of arm pixels, so we can remove them again
point lastSec[60];
point lastMin[60];
point lastHr[60];


//Initialize the RGBMatrix
DFRobot_RGBMatrix matrix(A, B, C, D, E, CLK, LAT, OE, false, WIDTH, HEIGHT);



//Helper methods for drawing the clock face
void drawNotchesManually() {
    for (int i = 0; i < 16; i++) {
        matrix.drawPixel(pgm_read_byte(&notchPoints[i].x), pgm_read_byte(&notchPoints[i].y), matrix.Color333(7, 0, 0));

        //RECORD
        pixelState[pgm_read_byte(&notchPoints[i].x)][pgm_read_byte(&notchPoints[i].y)] = true;

    }
}
void drawCircle(int x, int y, int r){
    double i, angle, x1, y1;
    for (i = 0; i < 360; i++)
    {
        angle = i;
        x1 = r * cos(angle * PI / 180);
        y1 = r * sin(angle * PI / 180);
        matrix.drawPixel(x + x1, y + y1, matrix.Color333(7, 7, 7));
    }
}
void drawNumbersManually() {
    for (int i = 0; i < 73; i++) {
        matrix.drawPixel(pgm_read_byte(&numberPoints[i].x), pgm_read_byte(&numberPoints[i].y), matrix.Color333(7, 7, 7));

        //RECORD
        pixelState[pgm_read_byte(&numberPoints[i].x)][pgm_read_byte(&numberPoints[i].y)] = true;
    }
}

//Convert degrees to radians
float deg2rad(float deg) {
    return (deg * 3.1415 / 180);
}

//Clears the arm pixels
void clearPixels(armType arm) {
    for (int i = 0; i < 60; ++i) {
        if (arm == SEC) {
            matrix.drawPixel(lastSec[i].x,lastSec[i].y, matrix.Color333(0, 0, 0));
        }

        if (arm == MIN) {
            matrix.drawPixel(lastMin[i].x,lastMin[i].y, matrix.Color333(0, 0, 0));
        }

        if (arm == HR) {
            matrix.drawPixel(lastHr[i].x,lastHr[i].y, matrix.Color333(0, 0, 0));
        }

    }
}


void drawAwareLine(byte x0, byte y0, byte x1, byte y1, armType arm) {
    byte i = 0; //Increment for every pixel in a specific arm


    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    byte newx, newy;
    if (steep) {
        swapTwo(x0, y0);
        swapTwo(x1, y1);
    }

    if (x0 > x1) {
        swapTwo(x0, x1);
        swapTwo(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            newx = y0;
            newy = x0;
        }
        else {
            newx = x0;
            newy = y0;
        }

        if (pixelState[newx][newy] == 0) {

            if (arm == SEC) {

                matrix.drawPixel(newx, newy, matrix.Color333(7, 0, 0));
                lastSec[i] = {newx, newy};
            }
            if (arm == MIN) {

                matrix.drawPixel(newx, newy, matrix.Color333(7, 7, 7));
                lastMin[i] = {newx, newy};
            }
            if (arm == HR) {

                matrix.drawPixel(newx, newy, matrix.Color333(7, 7, 7));
                lastHr[i] = {newx, newy};
            }
            i++;
        }




        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}




point calculatePoint(int radius, float timeCurrency, int modifier) {
    point p;
    p.x = radius * cos(deg2rad((timeCurrency - 15) * (360 / modifier))) + c.x;
    p.y = radius * sin(deg2rad((timeCurrency - 15) * (360 / modifier))) + c.y;
    return p;
}


void setup() {
    Serial.begin(115200);
    matrix.begin();
    matrix.setTextSize(1);

    //Draw the clock face
    drawCircle(32, 32, 32);
    drawCircle(32, 32, 31);
    drawNumbersManually();
    drawNotchesManually();


    while (Serial.available() == 0) {}; //Wait for input




}

void loop() {
    //Set the far points for each arm
    point farPointSec = {calculatePoint(r_sec, second(), 60).x, calculatePoint(r_sec, second(), 60).y};
    point farPointMin = {calculatePoint(r_min, minute(), 60).x, calculatePoint(r_min, minute(), 60).y};
    point farPointHr = {calculatePoint(r_hr, hour() + (float) minute() / 60, 12).x,calculatePoint(r_hr, hour() + (float) minute() / 60, 12).y};


    if (Serial.available())
    {
        String oneLine = Serial.readString();
        oneLine.trim();
        setTime(oneLine.substring(0, 2).toInt(), oneLine.substring(2, 4).toInt(), oneLine.substring(4).toInt(), 01, 01, 2022);

        //Draw every arm, clear previous
        clearPixels(SEC);
        drawAwareLine(c.x, c.y, farPointSec.x, farPointSec.y, SEC);
        clearPixels(MIN);
        drawAwareLine(c.x, c.y, farPointMin.x, farPointMin.y, MIN);
        clearPixels(HR);
        drawAwareLine(c.x, c.y, farPointHr.x, farPointHr.y, HR);
    }




    //Redraw second arm
    clearPixels(SEC);
    drawAwareLine(c.x, c.y, farPointSec.x, farPointSec.y, SEC);


    if (second() == 0) { //Redraw minute arm

        clearPixels(MIN);
        drawAwareLine(c.x, c.y, farPointMin.x, farPointMin.y, MIN);


        if (minute() % 15 == 0) { //Redraw hour arm
            clearPixels(HR);
            drawAwareLine(c.x, c.y, farPointHr.x, farPointHr.y, HR);
        }
    }
    delay(1000);
}