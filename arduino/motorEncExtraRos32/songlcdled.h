//buzzer control
#define   BUZZER_CH  2
#define   BUZZER     13
#define   MAX_SONG   5
#define   MAX_ANIM   3
#define   MAX_COLOR  6

// LED control pins
#define   LED_L     19
#define   LED_R     18
#define   LED_F     5

#define   LEDLEFT  0 //rear left on
#define   LEDRIGHT 1 //rear right on 
#define   LEDREAR  2 //rear left and right on
#define   LEDFRONT 3 //front right on
#define   ALL_ON   4 //all on
#define   ALL_OFF  5 //all off

void playsong(int song);
void RGB(int ledbehav);
void showAnimation(int idx);
void beginLcd(void);
