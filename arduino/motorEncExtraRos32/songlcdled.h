//buzzer control
#define   BUZZER_CH  2
#define   BUZZER     13
#define   MAX_SONG   5
#define   MAX_ANIM   3
#define   MAX_COLOR  6

// LED control pins
#define   LED_L     19
#define   LED_R     18
#define   LED_B     5

#define   LEDBACK  0 //back on
#define   LEDLEFT  1 //left on
#define   LEDRIGHT 2 //right on
#define   LEDFRONT 3 //left, right on
#define   ALL_ON   4 //all on
#define   ALL_OFF  5 //all off

void playsong(int song);
void RGB(int ledbehav);
void showAnimation(int idx);
void beginLcd(void);