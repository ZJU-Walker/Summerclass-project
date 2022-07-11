//We always have to include the library
#include "LedControl.h"

/*
  Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
  pin 12 is connected to the DataIn/din
  pin 11 is connected to the CLK
  pin 10 is connected to LOAD/cs
  We have only a single MAX72XX.
*/
LedControl lc = LedControl(12, 11, 10, 1);

/* we always wait a bit between updates of the display */
unsigned long delaytime = 250;

void setup() {
  /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
  */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 8);
  /* and clear the display */
  lc.clearDisplay(0);
  scrollDigits();
}



/*
  This method will scroll all the hexa-decimal
  numbers and letters on the display. You will need at least
  four 7-Segment digits. otherwise it won't really look that good.
*/
void scrollDigits() {
int  i = 1;
int  j = 2;
int  k = 3;
  lc.setDigit(0, 7, 1, false);
  lc.setDigit(0, 6, 2, false);
  lc.setDigit(0, 5, 3, false);
  lc.setChar(0, 4, '-', false);// -
  lc.setDigit(0, 3, i, false);
  lc.setDigit(0, 2, j, false);
  lc.setDigit(0, 1,  k, false);
  lc.setChar(0, 0, ' ', false);//空格


}

void loop() {


}
