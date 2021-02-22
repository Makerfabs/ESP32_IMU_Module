#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <Arduino.h>

#define TFT_CS 26
#define TFT_RST 14 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 27
#define TFT_MOSI 12
#define TFT_SCLK 13

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#define MAKEPYTHON_ESP32_SDA 4
#define MAKEPYTHON_ESP32_SCL 5
#define INTERRUPT_PIN 35

MPU6050 mpu;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
char AcXStr[7];
char AcYStr[7];
char AcZStr[7];
char GyXStr[7];
char GyYStr[7];
char GyZStr[7];

bool blinkState = false;
int balance_flag = 0;

/* MrAbalogu/Smart-Watch/MPU6050_3D_Cube */
/* Reference from https://github.com/MrAbalogu/Smart-Watch/blob/master/MPU6050_3D_Cube/MPU6050_3D_Cube.ino */

float xx, xy, xz;
float yx, yy, yz;
float zx, zy, zz;

float fact;

int Xan, Yan;
int last_Xan, last_Yan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
    int x;
    int y;
    int z;
};

struct Point2d
{
    int x;
    int y;
};

int LinestoRender;    // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.

struct Line3d
{
    Point3d p0;
    Point3d p1;
};

struct Line2d
{
    Point2d p0;
    Point2d p1;
};

Line3d Lines[12]; //Number of lines to render
Line2d Render[12];
Line2d ORender[12];

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/

void initLine()
{
    fact = 180 / 3.14159265358979323846264338327950; // conversion from degrees to radians.

    Xoff = 64; // positions the center of the 3d conversion space into the center of the OLED screen. This is usally screen_x_size / 2.
    Yoff = 64; // screen_y_size /2
    //Zoff = 750; //Size of cube, larger no. = smaller cube
    Zoff = 350;

    // line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.

    // Front Face.

    Lines[0].p0.x = -50;
    Lines[0].p0.y = -50;
    Lines[0].p0.z = 50;
    Lines[0].p1.x = 50;
    Lines[0].p1.y = -50;
    Lines[0].p1.z = 50;

    Lines[1].p0.x = 50;
    Lines[1].p0.y = -50;
    Lines[1].p0.z = 50;
    Lines[1].p1.x = 50;
    Lines[1].p1.y = 50;
    Lines[1].p1.z = 50;

    Lines[2].p0.x = 50;
    Lines[2].p0.y = 50;
    Lines[2].p0.z = 50;
    Lines[2].p1.x = -50;
    Lines[2].p1.y = 50;
    Lines[2].p1.z = 50;

    Lines[3].p0.x = -50;
    Lines[3].p0.y = 50;
    Lines[3].p0.z = 50;
    Lines[3].p1.x = -50;
    Lines[3].p1.y = -50;
    Lines[3].p1.z = 50;

    //back face.

    Lines[4].p0.x = -50;
    Lines[4].p0.y = -50;
    Lines[4].p0.z = -50;
    Lines[4].p1.x = 50;
    Lines[4].p1.y = -50;
    Lines[4].p1.z = -50;

    Lines[5].p0.x = 50;
    Lines[5].p0.y = -50;
    Lines[5].p0.z = -50;
    Lines[5].p1.x = 50;
    Lines[5].p1.y = 50;
    Lines[5].p1.z = -50;

    Lines[6].p0.x = 50;
    Lines[6].p0.y = 50;
    Lines[6].p0.z = -50;
    Lines[6].p1.x = -50;
    Lines[6].p1.y = 50;
    Lines[6].p1.z = -50;

    Lines[7].p0.x = -50;
    Lines[7].p0.y = 50;
    Lines[7].p0.z = -50;
    Lines[7].p1.x = -50;
    Lines[7].p1.y = -50;
    Lines[7].p1.z = -50;

    // now the 4 edge lines.

    Lines[8].p0.x = -50;
    Lines[8].p0.y = -50;
    Lines[8].p0.z = 50;
    Lines[8].p1.x = -50;
    Lines[8].p1.y = -50;
    Lines[8].p1.z = -50;

    Lines[9].p0.x = 50;
    Lines[9].p0.y = -50;
    Lines[9].p0.z = 50;
    Lines[9].p1.x = 50;
    Lines[9].p1.y = -50;
    Lines[9].p1.z = -50;

    Lines[10].p0.x = -50;
    Lines[10].p0.y = 50;
    Lines[10].p0.z = 50;
    Lines[10].p1.x = -50;
    Lines[10].p1.y = 50;
    Lines[10].p1.z = -50;

    Lines[11].p0.x = 50;
    Lines[11].p0.y = 50;
    Lines[11].p0.z = 50;
    Lines[11].p1.x = 50;
    Lines[11].p1.y = 50;
    Lines[11].p1.z = -50;

    LinestoRender = 12;
    OldLinestoRender = LinestoRender;
}

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void)
{
    float Xan2, Yan2, Zan2;
    float s1, s2, s3, c1, c2, c3;

    Xan2 = Xan / fact; // convert degrees to radians.
    Yan2 = Yan / fact;

    // Zan is assumed to be zero

    s1 = sin(Yan2);
    s2 = sin(Xan2);

    c1 = cos(Yan2);
    c2 = cos(Xan2);

    xx = c1;
    xy = 0;
    xz = -s1;

    yx = (s1 * s2);
    yy = c2;
    yz = (c1 * s2);

    zx = (s1 * c2);
    zy = -s2;
    zz = (c1 * c2);
}

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but not worth the effort.
void ProcessLine(struct Line2d *ret, struct Line3d vec)
{
    float zvt1;
    int xv1, yv1, zv1;

    float zvt2;
    int xv2, yv2, zv2;

    int rx1, ry1;
    int rx2, ry2;

    int x1;
    int y1;
    int z1;

    int x2;
    int y2;
    int z2;

    int Ok;

    x1 = vec.p0.x;
    y1 = vec.p0.y;
    z1 = vec.p0.z;

    x2 = vec.p1.x;
    y2 = vec.p1.y;
    z2 = vec.p1.z;

    Ok = 0; // defaults to not OK

    xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
    yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
    zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

    zvt1 = zv1 - Zoff;

    if (zvt1 < -5)
    {
        rx1 = 256 * (xv1 / zvt1) + Xoff;
        ry1 = 256 * (yv1 / zvt1) + Yoff;
        Ok = 1; // ok we are alright for point 1.
    }

    xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
    yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
    zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

    zvt2 = zv2 - Zoff;

    if (zvt2 < -5)
    {
        rx2 = 256 * (xv2 / zvt2) + Xoff;
        ry2 = 256 * (yv2 / zvt2) + Yoff;
    }
    else
    {
        Ok = 0;
    }

    if (Ok == 1)
    {
        ret->p0.x = rx1;
        ret->p0.y = ry1;

        ret->p1.x = rx2;
        ret->p1.y = ry2;
    }
    // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines that will be way out of whack, so they dont get drawn and cause screen garbage.
}

/************ MrAbalogu/Smart-Watch/MPU6050_3D_Cube*********************************************************************/
int calculateCube()
{
    //For cube rotation
    int xOut = 0;
    int yOut = 0;

    // xOut = map(AcX, -17000, 17000, -50, 50);
    // yOut = map(AcY, -17000, 17000, -50, 50);

    //测试可以
    // xOut = map(AcX, -32768, 0, -10, 10);
    // yOut = map(AcY, -30000, 3000, -10, 10);

    //xOut = map(GyX, -32768, 30000, -20, 20);
    //yOut = map(GyY, -32768, 30000, 20, -20);

    //绝对坐标
    yOut = map(AcX, -32768, 0, -90, 90);
    xOut = map(AcY, -30000, 3000, -90, 90);

    /*
    Serial.print("AcX  ");
    Serial.println(AcX);
    Serial.print("AcY  ");
    Serial.println(AcY);
    Serial.print("GyX  ");
    Serial.println(GyX);
    Serial.print("GyY  ");
    Serial.println(GyY);
    Serial.print("xOut  ");
    Serial.println(xOut);
    Serial.print("yOut  ");
    Serial.println(yOut);
    */

    //这是增量计算，配合Gy
    // Xan += xOut;
    // Yan += yOut;

    Xan = xOut;
    Yan = yOut;

    Yan = Yan % 360;
    Xan = Xan % 360; // prevents overflow.

    if (last_Xan == Xan && last_Yan == Yan)
        return 0;
    else
    {
        last_Xan = Xan;
        last_Yan = Yan;
    }

    SetVars(); //sets up the global vars to do the conversion.

    for (int i = 0; i < LinestoRender; i++)
    {
        ORender[i] = Render[i];            // stores the old line segment so we can delete it later.
        ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
    }

    if (Xan >= -3 && Xan <= 3 && Yan >= -3 && Yan <= 3)
        balance_flag = 1;

    return 1;
}

void drawScreen(void)
{
    //Serial.println("drawScreen");
    for (int i = 0; i < OldLinestoRender; i++)
    {
        tft.drawLine(ORender[i].p0.x, ORender[i].p0.y, ORender[i].p1.x, ORender[i].p1.y, ST77XX_BLACK); // erase the old lines.
        switch (i)
        {
        case 0:
            tft.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y, ST77XX_WHITE);
            break;
        case 1:
        case 2:
        case 3:
            tft.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y, ST77XX_YELLOW);
            break;

        default:
            tft.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y, ST77XX_BLUE);
            break;
        }
    }
    OldLinestoRender = LinestoRender;
}

void setup()
{

    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    st7735_init();

    Wire.begin(MAKEPYTHON_ESP32_SDA, MAKEPYTHON_ESP32_SCL);
    Wire.setClock(400000);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Updating internal sensor offsets...");
    Serial.print(mpu.getXAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getYAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getZAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getXGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(mpu.getYGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(mpu.getZGyroOffset());
    Serial.print("\t"); // 0
    Serial.print("\n");

    mpu.setXAccelOffset(-1782);
    mpu.setYAccelOffset(-889);
    mpu.setZAccelOffset(1427);
    mpu.setXGyroOffset(11);
    mpu.setYGyroOffset(-5);
    mpu.setZGyroOffset(23);

    Serial.print(mpu.getXAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getYAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getZAccelOffset());
    Serial.print("\t"); //
    Serial.print(mpu.getXGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(mpu.getYGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(mpu.getZGyroOffset());
    Serial.print("\t"); // 0
    Serial.print("\n");

    initLine();
}

void loop()
{
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    if (calculateCube())
        drawScreen();
    if (balance_flag == 1)
    {
        tft.drawCircle(64, 64, 20, ST77XX_GREEN);
        balance_flag = 2;
    }
    else if (balance_flag == 2)
    {
        tft.drawCircle(64, 64, 20, ST77XX_BLACK);
        balance_flag = 0;
    }
}

void st7735_init()
{
    tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
    tft.fillScreen(ST77XX_BLACK);
}