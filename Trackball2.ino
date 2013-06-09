#include <SPI.h>
#include <avr/pgmspace.h>

#define M1 1
#define M2 2
#define M3 3
#define S1 4
#define S2 5
#define ResButton 8

#define redLED 4
#define greenLED 9
#define blueLED 10
#define LEDintensity 225

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

typedef struct {
  byte pin;
  byte number;
  } buttonMap;

extern buttonMap buttons[];  //maps pin numbers to buttons on keyboard
byte buttonstatus[17]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0};
byte bchk=0;
byte keyseq[16]; //tracks which keyboard keys are pressed; index=key number; value=1-6 keyboard stroke
byte kbpress[7]; //tracks which keyboard slots are taken or available
byte mbuttons[4]; //tracks mouse buttons pushed or not
unsigned long debounce[16]; //debounce keyboard presses; based upon millis() timing
#define debounceTime 75  //how long between key changes allowed
unsigned long currTime;     //tracks current milliseconds for timing based operations
unsigned long scrolltimer;  //tracks next scroll push according to the scrollDelay
byte currResolution = 4;  //resolution CPI
unsigned long resTimer = 0;  //tracks next check of resolution button status
unsigned long ResLastZero=0; //used in detecting double-tap on resolution button
#define resdubclick 250 //how quick a double-tap on reslution button has to happen

byte initComplete=0;

const int ncs = 0;
volatile byte newptrdata=0;

extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];
extern float keybinding[16];
extern byte scrollDelay;

void setup() {
  Serial.begin(38400);

  pinMode(pgm_read_byte(&(buttons[0].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[1].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[2].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[3].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[4].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[5].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[6].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[7].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[8].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[9].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[10].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[11].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[12].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[13].pin)),INPUT_PULLUP);
  pinMode(pgm_read_byte(&(buttons[14].pin)),INPUT_PULLUP);
  
  pinMode (ncs, OUTPUT);
  
  attachInterrupt(0, UpdatePointer, FALLING);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(2);

  performStartup();  
  dispRegisters();
  delay(100);
  initComplete=9;
  currResolution=5;
  changeResolution(1);

}

void adns_com_begin(){
  digitalWrite(ncs, LOW);
}

void adns_com_end(){
  digitalWrite(ncs, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
  }

void performStartup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(1);

  Serial.println("Optical Chip Initialized");
  }

void UpdatePointer(void){
  if(initComplete==9){
    int xydat[2];
    readXY(&xydat[0]);
    Mouse.move(xydat[0],xydat[1]);
    }
  //Serial.print("x");
  }

void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(ncs,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs,HIGH);
}

int readXY(int *xy){
  digitalWrite(ncs,LOW);
  
  xy[0] = (int)adns_read_reg(REG_Delta_X_L);
  xy[1] = (int)adns_read_reg(REG_Delta_Y_L);

  //Convert from 2's complement
  if(xy[1] & 0x80){
    xy[1] = -1 * ((xy[1] ^ 0xff) + 1);
    }
  if(xy[0] & 0x80){
    xy[0] = -1 * ((xy[0] ^ 0xff) + 1);
    }
  xy[0] = xy[0] * -1;
  digitalWrite(ncs,HIGH);     
  } 

void pushKey(byte ctr, byte num){
   switch(ctr) {
     case 1:
       Keyboard.set_key1(num);
       break;
     case 2:
       Keyboard.set_key2(num);
       break;
     case 3:
       Keyboard.set_key3(num);
       break;
     case 4:
       Keyboard.set_key4(num);
       break;
     case 5:
       Keyboard.set_key5(num);
       break;
     case 6:
       Keyboard.set_key6(num);
       break;       
     }
  }

void changeKey(byte stat, byte bnum){

  int keystroke = int(keybinding[bnum]);
  float keymod = (keybinding[bnum] - keystroke) * 10;

  if(stat==0){  
    byte mod=byte(lround(keymod));
    switch (mod) {
      case 1:
        Keyboard.set_modifier(MODIFIERKEY_SHIFT);      
        break;
      case 2:
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        break;
      case 3:
        Keyboard.set_modifier(MODIFIERKEY_ALT);        
        break;
      }  

    //find and available key slot to assign this letter to
    int ctr=0;
    for(ctr=1;ctr<=6;ctr++){
      if(kbpress[ctr] == 0){
        kbpress[ctr]=1;
        keyseq[bnum]=ctr;     //tracked to release correct KB slot (1-6) when stat is 1
        break;
        }
      }

    pushKey(ctr,keystroke);
    Keyboard.send_now();
    Keyboard.set_modifier(0);
    }  

  else{
    pushKey(keyseq[bnum],0);
    Keyboard.send_now();
    kbpress[keyseq[bnum]]=0;
    keyseq[bnum]=0;
    }
  }

void chResolution(byte newRes){
  digitalWrite(ncs,LOW);
  adns_write_reg(REG_Configuration_I, newRes);
  digitalWrite(ncs,HIGH);
  }



void changeResolution(int resDirection){

/*  
Start with Red and ramp up Green.  (x > 0)
When Green is max, ramp down red to zero. (x > 13)
When red is zero, ramp up blue. (x > 25)
When blue is max, ramp down green to zero. (x > 38)
When green is zero, ramp up red. (x > 50)

When red is is max, ramp down blue to zero. (50-60)
Now you have a rainbow. 
*/

  //five light phases; Avago 9800 has 41 resolution steps @ 200 CPI each
  byte stepsize = LEDintensity / (41/5);

  if( (currResolution > 0 && resDirection < 0) || (currResolution < 41 && resDirection > 0) ){
    currResolution = currResolution + resDirection;
    }
  chResolution(currResolution);

  if(currResolution > 32){
    analogWrite(blueLED,LEDintensity);
    analogWrite(greenLED,0);
    analogWrite(redLED, ledsafe( stepsize * (currResolution - 32) ) );
    }
  else if(currResolution > 24){
    analogWrite(blueLED,LEDintensity);
    analogWrite(greenLED, ledsafe( stepsize * (8 - (currResolution - 24) ) ) ) ;
    analogWrite(redLED,0);
    }
  else if(currResolution > 16){
    analogWrite(greenLED,LEDintensity);
    analogWrite(redLED,0);
    analogWrite(blueLED, ledsafe( stepsize * (currResolution - 16) ) );
    }
  else if(currResolution > 8){
    analogWrite(greenLED,LEDintensity);
    analogWrite(redLED, ledsafe( stepsize * (8 - (currResolution - 8) ) ) ) ;
    analogWrite(blueLED,0);
    }
  else{
    analogWrite(redLED,LEDintensity);
    analogWrite(blueLED,0);
    analogWrite(greenLED, ledsafe( stepsize * currResolution ) );
    }
  }



byte ledsafe(byte pwmval){
  if(pwmval <= LEDintensity && pwmval >=0 ){
    return pwmval;
    }
  else {
    return LEDintensity;
    }
  }

int getButtonStat(int b){
  cli();  //disable interrupts for more accurate read
  byte x;
  int capCharge=0;
  for(x=0;x<10;x++){
    pinMode(b,INPUT);   
    digitalWrite(b,HIGH);
    while(digitalRead(b) == 0){
      capCharge++;
      }
    pinMode(b,OUTPUT);
    digitalWrite(b,LOW);
    delay(1);
    }
  sei(); //re-enable interrupts
  
  //if(b==14){riseAvg-=12;} //pin 14 needs a slight offset to work properly
  //if(b==12){Serial.println(capCharge);}
  return(capCharge);
  }
  
void loop() {
  currTime = millis();

  if(currTime > resTimer){
    resTimer = currTime + 125;
    byte resBstat = getButtonStat(ResButton);
    if(buttonstatus[16] != resBstat || buttonstatus[16] == 2){
      buttonstatus[16] = 2;
      if(resBstat){
        if( currTime - ResLastZero < resdubclick){
          Serial.println("down");
          changeResolution(-1);
          ResLastZero = currTime;
          }
        else {
          Serial.println("up");
          changeResolution(1);
          }
        }   
        else{
          ResLastZero = currTime;
          buttonstatus[16] = 0;
          }             
      }
    }


  for(bchk=0;bchk<15;bchk++){
    byte newbstat = digitalRead(pgm_read_byte(&(buttons[bchk].pin)));
    if( (newbstat != buttonstatus[pgm_read_byte(&(buttons[bchk].number))] && currTime > debounce[pgm_read_byte(&(buttons[bchk].number))] )|| 
       buttonstatus[pgm_read_byte(&(buttons[bchk].number))] == 2 ){ 
      buttonstatus[pgm_read_byte(&(buttons[bchk].number))] = newbstat;
      debounce[pgm_read_byte(&(buttons[bchk].number))] = currTime + debounceTime;

      Serial.println(pgm_read_byte(&(buttons[bchk].number)));

      if(keybinding[pgm_read_byte(&(buttons[bchk].number))] == M1){
        mbuttons[1]=newbstat ^ 1;
        Mouse.set_buttons(mbuttons[1],mbuttons[2],mbuttons[3]);
        }
      else if(keybinding[pgm_read_byte(&(buttons[bchk].number))] == M2){
        mbuttons[2]=newbstat ^ 1;
        Mouse.set_buttons(mbuttons[1],mbuttons[2],mbuttons[3]);
        }
      else if(keybinding[pgm_read_byte(&(buttons[bchk].number))] == M3){
        mbuttons[3]=newbstat ^ 1;
        Mouse.set_buttons(mbuttons[1],mbuttons[2],mbuttons[3]);
        }
      else if(keybinding[pgm_read_byte(&(buttons[bchk].number))] == S1){
        if (newbstat == 0) buttonstatus[pgm_read_byte(&(buttons[bchk].number))] = 2;
        if(currTime > scrolltimer){
          Mouse.scroll(1);
          scrolltimer = currTime + scrollDelay;
          }
        }
      else if(keybinding[pgm_read_byte(&(buttons[bchk].number))] == S2){
        if (newbstat == 0) buttonstatus[pgm_read_byte(&(buttons[bchk].number))] = 2;
        if(currTime > scrolltimer){
          Mouse.scroll(-1);
          scrolltimer = currTime + scrollDelay;
          }
        }
       else if(keybinding[pgm_read_byte(&(buttons[bchk].number))] != 0){
          changeKey(newbstat,pgm_read_byte(&(buttons[bchk].number)));
          }  

      }
    }
  }
