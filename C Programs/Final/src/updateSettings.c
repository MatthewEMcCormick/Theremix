#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "IO_DSP.h"
#include "DSP.h"
#include "global.h"
#include "lcd.h"
#include "updateSettings.h"


#define TC GREEN
#define BC RED
#define BS BLUE
void nano_wait(unsigned int n) {
    // Calibrate the delay based on the processor speed.
    while (n--) {
        for (volatile int i = 0; i < 1; i++) {
            // Just a small loop to waste time
            // continue;
        }
    }
}
void drawvert(int x, int y)
{
    LCD_DrawFillRectangle(x, y, x+5, y+20, TC);
}
void drawhorz(int x, int y)
{
    LCD_DrawFillRectangle(x, y, x+20, y+5, TC);
}
// Main function
void DrawS(int offsetx, int offsety)
{
    drawvert(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety + 15);
    drawhorz(offsetx + 0, offsety + 30);
}
void DrawA(int offsetx, int offsety)
{
    drawvert(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety);

    drawvert(offsetx + 15, offsety + 15);
    drawvert(offsetx, offsety + 15);
    // drawhorz(offsetx + 0, offsety + 30);
}
void DrawT(int offsetx, int offsety)
{
    drawhorz(offsetx + 0, offsety + 0);
    drawvert(offsetx + 7, offsety + 0);
    drawvert(offsetx + 7, offsety + 15);
  
}
void DrawU(int offsetx, int offsety)
{
    
    drawvert(offsetx + 0, offsety + 0);
    drawvert(offsetx + 15, offsety + 0);
    drawvert(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety + 15);
    drawhorz(offsetx + 0, offsety + 30);
  
}
void DrawR(int offsetx, int offsety)
{
    drawvert(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety);
    drawvert(offsetx, offsety + 15);
    
    LCD_DrawFillRectangle(offsetx+5,offsety+20,offsetx+15,offsety+25, TC);
    LCD_DrawFillRectangle(offsetx+15,offsety+25,offsetx+20,offsety+35, TC);
    LCD_DrawFillRectangle(offsetx+15,offsety+25,offsetx+20,offsety+35, TC);
}
void DrawP(int offsetx, int offsety)
{
    drawvert(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 0);
    drawhorz(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety);
    drawvert(offsetx, offsety + 15);
}
void DrawO(int offsetx, int offsety)
{
    drawhorz(offsetx+0, offsety+0);
    drawvert(offsetx + 0, offsety + 0);
    drawvert(offsetx + 15, offsety + 0);
    drawvert(offsetx + 0, offsety + 15);
    drawvert(offsetx + 15, offsety + 15);
    drawhorz(offsetx + 0, offsety + 30);
}
void DrawC(int offsetx, int offsety)
{
    drawhorz(offsetx+0, offsety+0);
    drawvert(offsetx + 0, offsety + 0);
    drawvert(offsetx + 0, offsety + 15);
    drawhorz(offsetx + 0, offsety + 30);
}
void DrawE(int offsetx, int offsety)
{
    drawhorz(offsetx+0, offsety+0);
    drawvert(offsetx + 0, offsety + 0);
    drawhorz(offsetx+0, offsety+ 15);
    drawvert(offsetx + 0, offsety + 15);
    drawhorz(offsetx + 0, offsety + 30);
}
void DrawM(int offsetx, int offsety)
{
    // LCD_DrawFillRectangle(offsetx,offsety, offsetx+3, offsety+35, TC);
    // LCD_DrawFillRectangle(offsetx+6,offsety, offsetx+9, offsety+35, TC);
    // LCD_DrawFillRectangle(offsetx+12,offsety, offsetx+15, offsety+35, TC);

    // LCD_DrawFillRectangle(offsetx+4,offsety+3, offsetx+5, offsety+10, TC);
    // LCD_DrawFillRectangle(offsetx+10,offsety+3, offsetx+11, offsety+10, TC);
    drawvert(offsetx +0, offsety + 0);
    drawvert(offsetx +0, offsety + 15);

    drawvert(offsetx + 9, offsety + 0);
    drawvert(offsetx + 9, offsety + 15);

    drawvert(offsetx + 18, offsety + 0);
    drawvert(offsetx + 18, offsety + 15);

    LCD_DrawFillRectangle(offsetx+6, offsety+3, offsetx+9, offsety+10, TC);
    LCD_DrawFillRectangle(offsetx+15, offsety+3, offsetx+21, offsety+10, TC);
  
    
}
void Display_Top(char soc)
{   
    if(soc == 's')
    {
        DrawS(0, 0);
        DrawA(30, 0);
        DrawT(60, 0);
        DrawU(90, 0);
        DrawR(120, 0);
        DrawA(150, 0);
        DrawT(180, 0);
        DrawO(210, 0);
        DrawR(240, 0);
    }
    if (soc == 'c')
    {
        DrawC(0,0);
        DrawO(30, 0);
        DrawM(60, 0);
        DrawP(90, 0);
        DrawR(120,0);
        DrawE(150,0);
        DrawS(180, 0);
        DrawS(210, 0);
        DrawO(240, 0);
        DrawR(270, 0);
    }
}
void switchToCom()
{
  
    LCD_DrawFillRectangle(0,0,320,45,BLACK);
    nano_wait(10000);
    disp_com(100, 50, 70, 75, 100);
    Display_Top('c');
}
void switchToSat()
{
    
    LCD_DrawFillRectangle(0,0,320,45,BLACK);
    nano_wait(10000);
    disp_sat(50, 100, 70, 75, 5);
    Display_Top('s');
}
void disp_sat(int setting1, int setting2, int setting3, int setting4, int setting5)
{
    LCD_DrawFillRectangle(0, 240-setting1, 20, 240, BS);
    LCD_DrawFillRectangle(0, 140, 20, 140+(100-setting1), BLACK);

    /*LCD_DrawFillRectangle(30, 240-setting2, 50, 240, BS);
    LCD_DrawFillRectangle(30, 140, 50 , 140+(100-setting2), BLACK);

    LCD_DrawFillRectangle(60, 240-setting3, 80, 240, BS);
    LCD_DrawFillRectangle(60, 140, 80 , 140+(100-setting3), BLACK);

    LCD_DrawFillRectangle(90, 240-setting4, 110, 240, BS);
    LCD_DrawFillRectangle(90, 140, 110 , 140+(100-setting4), BLACK);
    
    LCD_DrawFillRectangle(120, 240-setting5, 140, 240, BS);
    LCD_DrawFillRectangle(120, 140, 140, 140+(100-setting5), BLACK);*/
}

void disp_com(int setting1, int setting2, int setting3, int setting4, int setting5)
{
    LCD_DrawFillRectangle(0, 240-setting1, 20, 240, BC);
    LCD_DrawFillRectangle(0, 140, 20, 140+(100-setting1), BLACK);

    LCD_DrawFillRectangle(30, 240-setting2, 50, 240, BC);
    LCD_DrawFillRectangle(30, 140, 50 , 140+(100-setting2), BLACK);

    LCD_DrawFillRectangle(60, 240-setting3, 80, 240, BC);
    LCD_DrawFillRectangle(60, 140, 80 , 140+(100-setting3), BLACK);

    LCD_DrawFillRectangle(90, 240-setting4, 110, 240, BC);
    LCD_DrawFillRectangle(90, 140, 110 , 140+(100-setting4), BLACK);
    
    LCD_DrawFillRectangle(120, 240-setting5, 140, 240, BC);
    LCD_DrawFillRectangle(120, 140, 140, 140+(100-setting5), BLACK);
}








/* Use this to test timings tomorrow:

#define SCALE 100
int main(void) {
    internal_clock();         // Initialize system clock

    // SystemCoreClockUpdate(); //Get clock in the debugger
    
    test_pin();
    nano_wait(100000000/SCALE);
    //Need to run this on the STM32F091 and match up the timings
    GPIOB->ODR &=~ GPIO_ODR_OD8;
    while(1)
    {
        asm("wfi");
    }

*/