// Shift, ALT and CTRL modifiers can be added by adding: 
//     0.1 = SHIFT
//     0.2 = CTRL
//     0.3 = ALT
// Example: KEY_X + 0.1 prints a capital "X"
//
// M1, M2, M3 are mouse buttons
// S1, S2 scroll up/down

float keybinding[16] ={
0,  
KEY_F5,        // 1
KEY_F6,        // 2
S1,            // 3
KEY_F8,        // 4
KEY_F9,        // 5
KEYPAD_1,      // 6
M1,            // 7
M2,            // 8
M3,            // 9
KEYPAD_2,      // 10
KEYPAD_3,      // 11
KEYPAD_4,      // 12
S2,            // 13
KEYPAD_6,      // 14
KEYPAD_7,      // 15
};

byte scrollDelay = 75; // milliseconds between scroll presses. shorter=faster scroll

//which button is wired to which physical port on microcontroller
buttonMap buttons[] PROGMEM = { 
{6,1},
{23,2},
{20,3},
{21,4},
{18,5},
{13,6},
{22,7},
{11,8},
{15,9},
{17,10},
{16,11},
{24,12},
{7,13},
{14,14},
{19,15}
};


