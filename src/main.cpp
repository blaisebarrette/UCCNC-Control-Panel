
/*################ Libraries ################*/
  #include <Arduino.h>
  #include <U8g2lib.h>
  #include <ModbusRTU.h> // https://github.com/emelianov/modbus-esp8266
  #include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder

/*################ ESP32 Pin asignement ################*/
  #define Work_DRO_LED 2 /*__________________LED that shows that the DRO is set to "Work"    */
  #define Transmit_enable_pin 4 /*___________RTU Enable pin                                  */
  #define LCD3 5 /*__________________________LCD CS (RS)                                     */
  #define Encode1_Switch_Pin 12 /*___________Encoder 1 Switch pin                            */
  #define Machine_DRO_Select_Switch 13 /*____Switch that selects the "Machine" DRO           */
  #define Encode2_Switch_Pin 14 /*___________Encoder 2 Switch pin                            */
  #define Work_DRO_Select_Switch 15 /*_______Switch that selects the "Work" DRO              */
  #define rxdPin 16 /*_______________________Modbus Serial RX pin                            */
  #define txdPin 17 /*_______________________Modbus Serial TX pin                            */
  #define LCD1 18 /*_________________________LCD Clock (E)                                   */
  #define DTG_DRO_Select_Switch 19 /*________Switch that selects the "DTG" DRO               */
  #define DTG_DRO_LED 21 /*__________________LED that shows that the DRO is set to "Machine" */ 
  #define LCD4 22 /*_________________________LCD Reset (RST)                                 */
  #define LCD2 23 /*_________________________LCD data (R/W)                                  */
  #define Encode2_A_Pin 25 /*________________Encoder 2 A pin                                 */  
  #define Encode2_B_Pin 26 /*________________Encoder 2 B pin                                 */
  #define Machine_DRO_LED 27 /*______________LED that shows that the DRO is set to "DTG"     */ 
  #define Encode1_A_Pin 32 /*________________Encoder 1 B pin                                 */
  #define Encode1_B_Pin 33 /*________________Encoder 1 A pin                                 */
  #define MPG_Multiplicaton_pin 34 /*________MPG Axis Selection switch pin                   */
  #define MPG_Axis_Select_pin 35 /*__________MPG Multiplication Selection switch pin         */
  #define CoolantStartPin 36 /*______________Available / Not used in this code (Input Only)  */
  #define SpindleStartPin 39 /*______________Available / Not used in this code (Input Only)  */

/*################ Constant ################*/
  #define SLAVE_ID 1 // Used for modbus

/*################ Object Initialisation ################*/
  U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, LCD1, LCD2, LCD3, LCD4); //  LCD Initialisation  //
  ModbusRTU mb; // Modbus Initialisation //
  ESP32Encoder encoder1; // Feedrate Override encoder //
  ESP32Encoder encoder2; // Spindle Speed Override encoder //

/*################ Core asignement ################*/
  TaskHandle_t Task1; // Accessed via the function : SecondCoreCode //

/*################ Variables ################*/
  int E1prev, E2prev, MPG_Axis_Select_val, MPG_Multiplicaton;
  int E1current = 100;
  int E2current = 100;
  uint16_t selected_DRO = 1; /* 1 = Machine, 2 = Work, 3 = Distance to go (DTG) */
  uint16_t droX1, droX2, droY1, droY2, droZ1, droZ2, droC1, droC2, spindleLoad, SpindleSpeed;
  bool hard_MPG = true;
  bool HardOrSoftMPG_Changed = false;
  bool FeedOverrideChanged = false;
  bool SpindleSpeedOverrideChanged = false;
  String MPG_Selected_Axis;
  float MPG_Selected_Multiplication;
  unsigned long currentMillis;
  unsigned long previousMillis = 0;
  bool spindleStartCLicked = false;
  bool coolantStartCLicked = false;
  bool machine_DRO_Clicked = false;
  bool work_DRO_Clicked = false;
  bool DTG_DRO_Clicked = false;
  bool isMoving = false;
  bool Debuging_Mode = false;

/*################ Functions ################*/
  /* ################ CALLED BY CORE 0 (LCD) ################ */
    //-------------------  DRO_Format_Converter -------------------//
      float DRO_Format_Converter(unsigned int x1, unsigned int y1) { // Function called by the LCD_Print_DRO function
        union {
          uint32_t x;
          float f;
        } u;
        u.x = (((unsigned long)x1 << 16) | y1);
        float z = u.f;
        return z;
      }

    //-------------------  LCD_Print_DRO -------------------//
      void LCD_Print_DRO() {

        float X = DRO_Format_Converter(droX1, droX2);
        float Y = DRO_Format_Converter(droY1, droY2);
        float Z = DRO_Format_Converter(droZ1, droZ2);
        float C = DRO_Format_Converter(droC1, droC2);

        byte positionX = 0;
        byte positionY = 1;
        byte characterHeight = 8;

        int startingSpace = 42; int characterWidth = 6;
        int Xspace = startingSpace; int Yspace = startingSpace; int Zspace = startingSpace; int Cspace = startingSpace; int count;
        if (X < 0) {
          Xspace = Xspace - 7;
        }
        if (Y < 0) {
          Yspace = Yspace - 7;
        }
        if (Z < 0) {
          Zspace = Zspace - 7;
        }
        if (C < 0) {
          Cspace = Cspace - 7;
        }

        count = round( (X < 0) ? (X * -1) : X ); scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        Xspace = Xspace - (characterWidth * count);

        count = round( (Y < 0) ? (Y * -1) : Y ); scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        Yspace = Yspace - (characterWidth * count);

        count = round( (Z < 0) ? (Z * -1) : Z ); scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        Zspace = Zspace - (characterWidth * count);

        count = round( (C < 0) ? (C * -1) : C ); scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        Cspace = Cspace - (characterWidth * count);

        static char CharBuffer[10];
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.drawStr(positionY, positionX + characterHeight, "X:");
        sprintf(CharBuffer, "%.4f", X);
        u8g2.drawStr(Xspace, positionX + characterHeight, CharBuffer); //X
        u8g2.drawStr(positionY, positionX + (characterHeight * 2), "Y:");
        sprintf(CharBuffer, "%.4f", Y);
        u8g2.drawStr(Yspace, positionX + (characterHeight * 2), CharBuffer); //Y
        u8g2.drawStr(positionY, positionX + (characterHeight * 3), "Z:");
        sprintf(CharBuffer, "%.4f", Z);
        u8g2.drawStr(Zspace, positionX + (characterHeight * 3), CharBuffer); //Z
        u8g2.drawStr(positionY, positionX + (characterHeight * 4), "C:");
        sprintf(CharBuffer, "%.4f", C);
        u8g2.drawStr(Cspace, positionX + (characterHeight * 4), CharBuffer); //C

        u8g2.drawLine(69, 0, 69, positionX + (characterHeight * 4 + 2));
        u8g2.drawLine(0, positionX + (characterHeight * 4 + 2), 128, positionX + (characterHeight * 4 + 2));
      }

    //-------------------  LCD_Feed_Override -------------------//
      void LCD_Feed_Override() {
        byte positionX = 36;
        byte positionY = 1;
        byte characterHeight = 8;
        byte neededSpace = 41;

        int count = E1current; scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        int space = (positionY + neededSpace) - (6 * count);
        static char CharBuffer[5];
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.drawStr(positionY, positionX + characterHeight, "FOR:");
        sprintf(CharBuffer, "%d %%", E1current);
        u8g2.drawStr(space, positionX + characterHeight, CharBuffer);
      }

    //-------------------  LCD_Spindle_Override -------------------//
      void LCD_Spindle_Override() {
        byte positionX = 36;
        byte positionY = 62;
        byte characterHeight = 8;
        byte neededSpace = 41;

        int count = E2current; scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        int space = (positionY + neededSpace) - (6 * count);
        static char CharBuffer[5];
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.drawStr(positionY, positionX + characterHeight, "SOR:");
        sprintf(CharBuffer, "%d %%", E2current);
        u8g2.drawStr(space, positionX + characterHeight, CharBuffer);
      }

    //-------------------  LCD_Spindle_Speed_Override -------------------//
      void LCD_Spindle_Speed_Override() {
        byte positionX = 45;
        byte positionY = 1;
        byte characterHeight = 8;
        byte neededSpace = 102;

        float val = 99; //------------------------------------- à modifier! 
        int count = val; scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        int space = (positionY + neededSpace) - (6 * count);
        static char CharBuffer[5];
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.drawStr(positionY, positionX + characterHeight, "Spindle Load:");
        sprintf(CharBuffer, "%d %%", spindleLoad);
        u8g2.drawStr(space, positionX + characterHeight, CharBuffer);
        //u8g2.drawStr(space, positionX + characterHeight, "N/A");
      }

    //-------------------  LCD_Spindle_Speed_RPM -------------------//
      void LCD_Spindle_Speed_RPM() {
        byte positionX = 54;
        byte positionY = 1;
        byte characterHeight = 8;
        byte neededSpace = 102;
        float val = 99; //------------------------------------- à modifier!

        int count = val; scanf("%d", &count);
        count = (count == 0) ? 1  : (log10(count) + 1);
        int space = (positionY + neededSpace) - (6 * count);
        static char CharBuffer[9];
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.drawStr(positionY, positionX + characterHeight, "Spindle RPM:");
        sprintf(CharBuffer, "%d RPM", SpindleSpeed);
        u8g2.drawStr(space, positionX + characterHeight, CharBuffer);
        //u8g2.drawStr(space, positionX + characterHeight, "N/A");
      }

    //-------------------  JOG_Display -------------------//
      void JOG_Display() {
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(88, 11, "JOG");
        static char CharBuffer[10];
        if (!isMoving) {
          u8g2.setFont(u8g2_font_7x13B_tf);
          sprintf(CharBuffer, "%s: %.3f", MPG_Selected_Axis, MPG_Selected_Multiplication);
          u8g2.drawStr(72, 24, CharBuffer);
        } else {
          u8g2.setFont(u8g2_font_7x13B_tf);
          u8g2.drawStr(72, 24, "DISABLED");
        }
      }

    //-------------------  SecondCoreCode -------------------//
      void SecondCoreCode( void * parameter) {
        //--- Setup Section Start ----//
        u8g2.begin();
        //--- Setup Section End ----//
        
        while (true) {
          //--- Loop Section Start ----//
          u8g2.clearBuffer();
          LCD_Print_DRO();
          JOG_Display();
          LCD_Feed_Override();
          LCD_Spindle_Override();
          LCD_Spindle_Speed_Override();
          LCD_Spindle_Speed_RPM();
          u8g2.sendBuffer();
          delay(15);
          //--- Loop Section End ----//
        }
      }

  /* ################ CALLED BY CORE 1 (Modbus) ################ */
    //------------------- DRO_Select_LED -------------------//
      void DRO_Select_LED() {
        switch (selected_DRO){
          case 1:
            digitalWrite(Machine_DRO_LED, HIGH);
            digitalWrite(Work_DRO_LED, LOW);
            digitalWrite(DTG_DRO_LED, LOW);
            break;
          case 2:
            digitalWrite(Machine_DRO_LED, LOW);
            digitalWrite(Work_DRO_LED, HIGH);
            digitalWrite(DTG_DRO_LED, LOW);
            break;
          case 3:
            digitalWrite(Machine_DRO_LED, LOW);
            digitalWrite(Work_DRO_LED, LOW);
            digitalWrite(DTG_DRO_LED, HIGH);
            break;
        }
      }
    
    //------------------- Set_DRO_Variables -------------------//
      void Set_DRO_Variables() {
            droX1 = mb.Hreg(0);
            droX2 = mb.Hreg(1);
            droY1 = mb.Hreg(2);
            droY2 = mb.Hreg(3);
            droZ1 = mb.Hreg(4);
            droZ2 = mb.Hreg(5);
            droC1 = mb.Hreg(6);
            droC2 = mb.Hreg(7);
      }

    //------------------- FeedOverRide -------------------//
      void FeedOverRide() {
        if (digitalRead(Encode1_Switch_Pin) == LOW) {
          E1current = 100;
          E1prev = encoder1.getCountRaw();
          FeedOverrideChanged = true;
        }
        
        if (encoder1.getCountRaw() != E1prev) {
          if (encoder1.getCountRaw() < E1prev) {
            if (E1current > 0) {
              E1current = E1current - 10;
            }
          }
          if (encoder1.getCountRaw() > E1prev) {
            if (E1current < 300) {
              E1current = E1current + 10;
            }
          }
          E1prev = encoder1.getCountRaw();
          FeedOverrideChanged = true;
        }

        // Change made in UCCNC
        if(mb.Hreg(13) == 1){ // UCCNC watchdog signals a change
          E1current = mb.Hreg(10); // Place value from UCCNC into Control Panel
          mb.Hreg(56, 2); // Confirm to UCCNC that message is recieved
          while (mb.Hreg(13) != 0){ //Whait for confirmation from UCCNC
            mb.Hreg(53, mb.Hreg(10)); // Equalize Value in both registers
            mb.task(); yield(); // Keep modbus engine running
          }
          mb.Hreg(56, 0); // All is equalised, set watchdog to sleep!
        }

        // Change made in Control Panel
        if(FeedOverrideChanged){ 
          FeedOverrideChanged = false;
          mb.Hreg(56, 1); // Control panel watch dog sends signal to UCCNC that someting has changed
          while (mb.Hreg(13) != 2){ //Whait for confirmation from UCCNC that fields have been updated
            mb.Hreg(53, E1current); // Place new value from Control Panel in register
            mb.task(); yield(); // Keep modbus engine running
          }
          mb.Hreg(56, 0); // All is equalised, set watchdog to sleep!
        }
      }
    //------------------- SpindleSpeedOverRide -------------------//
      void SpindleSpeedOverRide() {  
        if (digitalRead(Encode2_Switch_Pin) == LOW) {
          E2current = 100;
          E2prev = encoder2.getCountRaw();
          SpindleSpeedOverrideChanged = true;
        }
        
        if (encoder2.getCountRaw() != E2prev) {
          if (encoder2.getCountRaw() < E2prev) {
            if (E2current > 0) {
              E2current = E2current - 10;
            }
          }
          if (encoder2.getCountRaw() > E2prev) {
            if (E2current < 300) {
              E2current = E2current + 10;
            }
          }
          E2prev = encoder2.getCountRaw();
          SpindleSpeedOverrideChanged = true;
        }

        // Change made in UCCNC
        if(mb.Hreg(14) == 1){ // UCCNC watchdog signals a change
          E2current = mb.Hreg(11); // Place value from UCCNC into Control Panel
          mb.Hreg(57, 2); // Confirm to UCCNC that message is recieved
          while (mb.Hreg(14) != 0){ //Whait for confirmation from UCCNC
            mb.Hreg(54, mb.Hreg(11)); // Equalize Value in both registers
            mb.task(); yield(); // Keep modbus engine running
          }
          mb.Hreg(57, 0); // All is equalised, set watchdog to sleep!
        }

        // Change made in Control Panel
        if(SpindleSpeedOverrideChanged){ 
          SpindleSpeedOverrideChanged = false;
          mb.Hreg(57, 1); // Control panel watch dog sends signal to UCCNC that someting has changed
          while (mb.Hreg(14) != 2){ //Whait for confirmation from UCCNC that fields have been updated
            mb.Hreg(54, E2current); // Place new value from Control Panel in register
            mb.task(); yield(); // Keep modbus engine running
          }
          mb.Hreg(57, 0); // All is equalised, set watchdog to sleep!
        }
      }

    //------------------- MPG_control_Select -------------------//
      int previous_MPG_Axis_Select_val, previous_MPG_Multiplicaton;
      void MPG_control_Select() {
        MPG_Axis_Select_val = analogRead(MPG_Axis_Select_pin);
        MPG_Multiplicaton = analogRead(MPG_Multiplicaton_pin);
        if (MPG_Axis_Select_val != previous_MPG_Axis_Select_val || MPG_Multiplicaton != previous_MPG_Multiplicaton) {
          previous_MPG_Axis_Select_val = MPG_Axis_Select_val;
          if (MPG_Axis_Select_val < 500) {
            mb.Hreg(51, 1);
            MPG_Selected_Axis = "X";
          } else if (MPG_Axis_Select_val >= 500 && MPG_Axis_Select_val < 1800) {
            mb.Hreg(51, 2);
            MPG_Selected_Axis = "Y";
          } else if (MPG_Axis_Select_val >= 1800 && MPG_Axis_Select_val < 3300) {
            mb.Hreg(51, 3);
            MPG_Selected_Axis = "Z";
          } else if (MPG_Axis_Select_val >= 3300) {
            mb.Hreg(51, 4);
            MPG_Selected_Axis = "C";
          }

          previous_MPG_Multiplicaton = MPG_Multiplicaton;
          if (MPG_Multiplicaton < 500) {
            mb.Hreg(52, 4);
            MPG_Selected_Multiplication = 1.00;
          } else if (MPG_Multiplicaton >= 500 && MPG_Multiplicaton < 1800) {
            mb.Hreg(52, 3);
            MPG_Selected_Multiplication = 0.10;
          } else if (MPG_Multiplicaton >= 1800 && MPG_Multiplicaton < 3300) {
            mb.Hreg(52, 2);
            MPG_Selected_Multiplication = 0.01;
          } else if (MPG_Multiplicaton >= 3300) {
            mb.Hreg(52, 1);
            MPG_Selected_Multiplication = 0.001;
          }
        }
      }

    //------------------- Test_DRO_Botton_State -------------------//
      void Test_DRO_Botton_State() {
        if (digitalRead(Machine_DRO_Select_Switch) == LOW) {
          machine_DRO_Clicked = true;
          previousMillis = millis();
          while(digitalRead(Machine_DRO_Select_Switch) == LOW && mb.Hreg(60) == 0){
            currentMillis = millis();
              if(currentMillis - previousMillis >= 2000 && machine_DRO_Clicked){
                previousMillis = currentMillis;
                machine_DRO_Clicked = false;
                mb.Hreg(60, 1);
                while(mb.Hreg(17) != 1){
                  mb.task();
                  yield();
                }
              }
              mb.Hreg(60, 0);
              mb.task();
              yield();
              Set_DRO_Variables();
            }
        }else{
          if(machine_DRO_Clicked){
            machine_DRO_Clicked = false;
            selected_DRO = 1;
             mb.Hreg(50, selected_DRO);
          }
        }

        if(digitalRead(Work_DRO_Select_Switch) == LOW) {
          work_DRO_Clicked = true;
          previousMillis = millis();
          while(digitalRead(Work_DRO_Select_Switch) == LOW && mb.Hreg(60) == 0){
            currentMillis = millis();
            if(currentMillis - previousMillis >= 2000 && work_DRO_Clicked){
              previousMillis = currentMillis;
              work_DRO_Clicked = false;
              mb.Hreg(60, 2);
              while(mb.Hreg(17) != 1){
                mb.task();
                yield();
              }
            }
            mb.Hreg(60, 0);
            mb.task();
            yield();
            Set_DRO_Variables();
          }
        }else{
          if(work_DRO_Clicked){
            work_DRO_Clicked = false;
            selected_DRO = 2;
             mb.Hreg(50, selected_DRO);
          }
        }

        if (digitalRead(DTG_DRO_Select_Switch) == LOW){
          DTG_DRO_Clicked = true;
          previousMillis = millis();
          while(digitalRead(DTG_DRO_Select_Switch) == LOW && mb.Hreg(60) == 0){
            currentMillis = millis();
            if(currentMillis - previousMillis >= 2000 && DTG_DRO_Clicked){
              previousMillis = currentMillis;
              DTG_DRO_Clicked = false;
              mb.Hreg(60, 3);
              while(mb.Hreg(17) != 1){
                mb.task();
                yield();
              }
            }
            mb.Hreg(60, 0);
            mb.task();
            yield();
            Set_DRO_Variables();
          }
        }else{
          if(DTG_DRO_Clicked){
            DTG_DRO_Clicked = false;
            selected_DRO = 3;
             mb.Hreg(50, selected_DRO);
          }
        }
      }

    //------------------- SpindleStartAndStop -------------------//
      void SpindleStartAndStop(){
        if (digitalRead(SpindleStartPin) == LOW){
          spindleStartCLicked = true;
          previousMillis = millis();
          while(digitalRead(SpindleStartPin) == LOW){
            currentMillis = millis();
            if(currentMillis - previousMillis >= 2000 && spindleStartCLicked){
              previousMillis = currentMillis;
              spindleStartCLicked = false;
              mb.Hreg(58, 2);
              while(mb.Hreg(15) != 1){
                mb.task();
                yield();
              }
              mb.Hreg(58, 0);
            }
          } 
        }else{
          if(spindleStartCLicked){
            spindleStartCLicked = false;
            mb.Hreg(58, 1);
            while(mb.Hreg(15) != 1){
              mb.task();
              yield();
            }
            mb.Hreg(58, 0);
          }
        }
      }
      
    //------------------- CoolantStartAndStop -------------------//
      void CoolantStartAndStop(){
        if (digitalRead(CoolantStartPin) == LOW){
          coolantStartCLicked = true;
        }else{
          if(coolantStartCLicked){
            coolantStartCLicked = false;
            mb.Hreg(59, 1);
            while(mb.Hreg(16) != 1){
              mb.task();
              yield();
            }
            mb.Hreg(59, 0);
          }
        }
      }

    //------------------- Debug -------------------//
      void Debug() {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= 250) {
          previousMillis = currentMillis;
          //Serial.printf("E1prev = %d E2prev = %d MPG_Axis_Select_val =  %d MPG_Multiplicaton = %d E1current = %d E2current = %d selected_DRO = %d \n", E1prev, E2prev, MPG_Axis_Select_val, MPG_Multiplicaton, E1current, E2current, selected_DRO);
        }
      }

/*################ Setup and loop ################*/
  void setup(void) {
    xTaskCreatePinnedToCore(SecondCoreCode, "Task1", 10000, NULL, 0, &Task1, 0); /* Second core implementation */
    btStop();    

    // Feedrate and Spindle Override encoders relevant setup //
      ESP32Encoder::useInternalWeakPullResistors = true;
      encoder1.clearCount();
      encoder2.clearCount();
      encoder1.attachSingleEdge(Encode1_A_Pin, Encode1_B_Pin);
      encoder2.attachSingleEdge(Encode2_A_Pin, Encode2_B_Pin);
      E1prev = encoder1.getCountRaw(); // Feed Override //
      E2prev = encoder2.getCountRaw(); // Spindle Override //
      pinMode(Encode1_Switch_Pin, INPUT_PULLUP); // Encoder 1 switch pin //
      pinMode(Encode2_Switch_Pin, INPUT_PULLUP); // Encoder 2 switch pin //
    
    // Pin Modes //
      pinMode(Machine_DRO_Select_Switch, INPUT_PULLUP);
      pinMode(Work_DRO_Select_Switch, INPUT_PULLUP);
      pinMode(DTG_DRO_Select_Switch, INPUT_PULLUP);
      pinMode(Machine_DRO_LED, OUTPUT);
      pinMode(Work_DRO_LED, OUTPUT);
      pinMode(DTG_DRO_LED, OUTPUT);
      pinMode(SpindleStartPin, INPUT);
      pinMode(CoolantStartPin, INPUT);

    // Modbus stuf //
      Serial1.begin(115200, SERIAL_8N1, rxdPin, txdPin);
      mb.begin(&Serial1, Transmit_enable_pin);
      mb.slave(SLAVE_ID);

    /*################ From Master to slave ################*/
      mb.addHreg(0);    // DRO_X_1
      mb.addHreg(1);    // DRO_X_2
      mb.addHreg(2);    // DRO_Y_1
      mb.addHreg(3);    // DRO_Y_2
      mb.addHreg(4);    // DRO_Z_1
      mb.addHreg(5);    // DRO_Z_2
      mb.addHreg(6);    // DRO_C_1
      mb.addHreg(7);    // DRO_C_2
      mb.addHreg(8);    // Spindle Load (%)
      mb.addHreg(9);    // Spindle RPM
      mb.addHreg(10);    // Feed Override
      mb.addHreg(11);    // Spindle Speed Override (%)
      mb.addHreg(12);    // Not used
      mb.addHreg(13);    // Feed override Watchdog
      mb.addHreg(14);    // Spindle Speed override Watchdog
      mb.addHreg(15);    // Spindle Start / stop Watchdog
      mb.addHreg(16);    // coolant Start / stop Watchdog      
      mb.addHreg(17);    // setXYZ Watchdog
      mb.addHreg(18);    // IsMoving Watchdog      
    
    /*################ From slave to master ################*/
      mb.addHreg(50);    // selected_DRO
      mb.addHreg(51);    // MPG_Axis_Select
      mb.addHreg(52);    // MPG_Multiplicaton
      mb.addHreg(53);    // Feed Override
      mb.addHreg(54);    // Spindle Speed Override
      mb.addHreg(55);    // Not used
      mb.addHreg(56);    // Feed Override Watchdog
      mb.addHreg(57);    // Spindle Speed Override Watchdog
      mb.addHreg(58);    // Spindle Watchdog
      mb.addHreg(59);    // Coolant Watchdog
      mb.addHreg(60);    // setXYZ Watchdog

    // Serial debug. Runs only if variable "Debuging_Mode" is set to true //
      if (Debuging_Mode) {
        Serial.begin(115200);
        Serial.println("Debug mode activated!");
      }
  }

  void loop(void) {
    if (Debuging_Mode){Debug();}
    Test_DRO_Botton_State();
    Set_DRO_Variables();
    DRO_Select_LED();
    MPG_control_Select();
    FeedOverRide();
    SpindleSpeedOverRide();
    SpindleStartAndStop();
    CoolantStartAndStop();
    spindleLoad = mb.Hreg(8);    // Spindle Load (%)
    SpindleSpeed = mb.Hreg(9);    // Spindle RPM
    if(mb.Hreg(18) == 1){isMoving = true;}else{isMoving = false;}
    mb.task();
    yield();
    delay(1);
  }