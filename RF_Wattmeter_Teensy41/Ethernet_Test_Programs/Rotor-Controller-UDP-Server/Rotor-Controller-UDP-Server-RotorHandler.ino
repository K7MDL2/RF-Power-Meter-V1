/*
   Rotoator control functions, part of Rotor-Controller-UDP-Server
   
 This sketch receives UDP message commands from a client to turn a rotator and sends rotator position
 information back to the client.

January 2021
K7MDL

 This code is in the public domain.
 */
// 
//____________________________________ Rotor_commands _________________________________________________
//
    // parse the commands in packetBuffer (global string)for format of "cmd1","cmd2".  
    // Reply with "OK" in ReplyBuffer string (also global) for successful commands, "??" for no success or unknown requests
    //Rotor_commands(); 


/*   Yeasu Commands
B              - Report elevation
C              - Report azimuth
C2             - Report azimuth and elevation
S              - Stop all rotation
A              - Stop azimuth rotation
E              - Stop elevation rotation
L              - Rotate azimuth left (CCW)
R              - Rotate azimuth right (CW)
D              - Rotate elevation down
U              - Rotate elevation up
Mxxx           - Move to azimuth
Wxxx yyy       - Move to azimuth xxxx and elevation yyy
X1             - Change to azimuth rotation speed setting 1
X2             - Change to azimuth rotation speed setting 2
X3             - Change to azimuth rotation speed setting 3
X4             - Change to azimuth rotation speed setting 4
O              - Azimuth offset calibration
F              - Azimuth full scale calibration
O2             - Elevation offset calibration
F2             - Elevation full scale calibration
P36            - Switch to 360 degree mode
P45            - Switch to 450 degree mode
Z              - Toggle north / south centered mode
H              - Help
*/ 
void Rotor_commands(void)
{
  /*
    char cmd1;
    char cmd2;
    //packetBuffer;
    // get UDP commands for rotor movement and calibration, presets and manual limits 
    switch(cmd1)
    {
        // Select Yeasu commands
        case "C":
        case "S":
        case "A":
        case "L":
        case "R":
        case "M":  // Mxxx
        case "O":
        case "F":
        case "P36":
        case "Z":
        case "H":
        // custom commands
        case "\P":
        case "\Y":
        case "\?GAx":  //  \?GAx[x][x][.x][x]  - go to AZ xxx.x   Setup as \?GA in cmd1, heading in cmd2
        case "PR":   // go to preset # in cmd2        
        default:
        break;
    }
    */
}    
// 
//____________________________________ Rotor_state _________________________________________________
//
void Rotor_state(void)
{  
  //Serial.print("AZCnt:");
  //rotor_position_counts();
  //Serial.print(RotorPosCounts);

  if (DBG==1) Serial.print(" AZSt:");
  if (DBG==1) Serial.print(RotorAZ_StartPos);

  if (DBG==1) Serial.print(" AZV:");
  rotor_position_volts();
  if (DBG==1) Serial.print(RotorPosV);
    
  if (DBG==1) Serial.print(" MLimCW:");
  if (DBG==1) Serial.print( manual_limit_CW);

  if (DBG==1) Serial.print(" MLimCCW:");
  if (DBG==1) Serial.print( manual_limit_CCW);

  if (DBG==1) Serial.print(" AZRaw:");
  rotor_position_AZ();
  if (DBG==1) Serial.print(RotorAZ_raw);
  
  if (DBG==1) Serial.print(" AZTarg_raw:");
  if (DBG==1) Serial.print(RotorTargetAZ);

  if (DBG==1) Serial.print(" AZTarg:");
  if (RotorTargetAZ > 360) {
      if (DBG==1) Serial.print(RotorTargetAZ-360);
  }    
  else
      if (DBG==1) Serial.print(RotorTargetAZ);
  
  if (DBG==1) Serial.print(" AZ:");
  rotor_position_AZ();
  if (DBG==1) Serial.println(RotorAZ);

  if (RotorTargetAZ > 360) sprintf((char *) tx_buffer, "Rotor Status: AZSt:%.1f AZVDC:%.1f CWLim:%.1f CCWLim:%.1f AZRaw:%.1f AZTargRaw:%.1f AZTarg:%.1f AZPos:%.1f", RotorAZ_StartPos, RotorPosV, manual_limit_CW, manual_limit_CCW, RotorAZ_raw, RotorTargetAZ, RotorTargetAZ-360, RotorAZ);
  else sprintf((char *) tx_buffer, "Rotor Status: AZSt:%.1f AZVDC:%.1f CWLim:%.1f CCWLim:%.1f AZRaw:%.1f AZTargRaw:%.1f AZTarg:%.1f AZPos:%.1f", RotorAZ_StartPos, RotorPosV, manual_limit_CW, manual_limit_CCW, RotorAZ_raw, RotorTargetAZ, RotorTargetAZ, RotorAZ);
  send_status();
}
// 
//____________________________________ get_remote_cmd _________________________________________________
//
void get_remote_cmd()   // parser from wattmeter and Desktop app.  Modify to use Yeasu commands
{
    uint8_t cmd1 = 0;
    float cmd2 = 0;
    uint8_t cmd_str_len, i = 0, j = 0;
    char cmd_str[BUF_LEN] = {};

    cmd2 = 0;  // do something to remove compiler warning for unused variable
    if (rx_count >=0) 
    { 
        pSdata = (uint8_t *) strchr((char *) pSdata1, '\n');   // find string terminator position 
        if (pSdata) { 
            *pSdata = '\0';
            cmd_str_len = pSdata - pSdata1;                     
            strncpy(cmd_str, (char *) pSdata1, cmd_str_len);   // copy chars between p1 and the terminator
            pSdata1 += (cmd_str_len+1);  // reset ch pointer back to strat of string for char parsing

            if (pSdata1 >= pSdata2 || cmd_str_len > BUF_LEN)  // if we have caught up to the end p2 then reset to beginning of buffer position.
                pSdata1 = pSdata2 = sdata;      

            if (strlen((char *) sdata) >= BUF_LEN-1) {
                //pSdata = sdata;
                sdata[0] = '\0';
                //printf("BUFFER OVERRRUN\n");
                //Serial_ClearRxBuffer();            
                return;
            }             
            // filter out unwanted characters             
            // Now we have a full comma delimited command string - validate and break it down           
            j = 0;
            for (i=0; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                if (isalnum(sdata[i]))
                    cmd_str[j++] = (sdata[i]);                 
            }
            cmd_str[j] = '\0';  
            //DBG_Serial.print("> Meter ID  ");
            //DBG_Serial.println("%s\n",cmd_str);
            if (atoi(cmd_str) == METERID) {
                if (i < cmd_str_len) {
                    j = 0;
                    // Look for Msg_Type now
                    for (i +=1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {   // i+1 to skip over the comma the var 'i' was left pointing at
                        cmd_str[j++] = sdata[i];                 
                    }
                    cmd_str[j] = '\0';
                    //printf(" Msg Type:  ");
                    //printf("%s",cmd_str);
                  
                    if (atoi(cmd_str) == 120)  {   // Vaidate this message type for commands.  120 = coammand, 170 is data output                              
                        j = 0;
                        if (i < cmd_str_len) {                               
                            for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                                cmd_str[j++] = sdata[i];
                            }  
                            cmd_str[j] = '\0';
                            //printf(" CMD1:  ");
                            //printf("%s",cmd_str);
                            cmd1 = atoi(cmd_str);
                        }
                        else 
                            cmd1 = 0;
                        
                        j = 0;
                        if (i < cmd_str_len) {                                                
                            for (i += 1; (sdata[i] != ',') && (i <= cmd_str_len); i++) {
                                cmd_str[j++] = sdata[i];                          
                            }
                            cmd_str[j] = '\0';
                            //printf(" CMD2:  ");
                            //printf("%s",cmd_str);
                            cmd2 = atof(cmd_str);
                        }
                        else 
                            cmd2 = 0.0;
                  
                        // Now do the commands     
                        // add code to limit variables received to allowed range
                        if (cmd1 == 241) {
                            Serial.print("Cmd1=");
                            Serial.print(cmd1);
                            MoveRotor = CW; // valid options are CW, CCW or STOP
                            MovetoPreset = OFF;
                            RotorTargetAZ = cmd2; 
                            allOff();  
                            delay(200);
                            Serial.print("  Move CW to ");   
                            Serial.println(RotorTargetAZ);                      
                            if (RotorTargetAZ < RotorAZ_StartPos)
                                RotorTargetAZ += 360; 
                            stall_detect_timer = millis();  // reset timer  
                            RotorAZ_raw_last = RotorAZ_raw;  // update new position                           
                            compute_rotor_move(); 
                        } 
                        if (cmd1 == 240) { 
                            Serial.print("Cmd1=");
                            Serial.print(cmd1);
                            MoveRotor = CCW; // valid options are CW, CCW or STOP
                            MovetoPreset = OFF;
                            RotorTargetAZ = cmd2;
                            allOff();  
                            delay(200); 
                            Serial.print("  Move CCW to ");    
                            Serial.println(RotorTargetAZ);                         
                            if (RotorTargetAZ < RotorAZ_StartPos)
                                RotorTargetAZ += 360;
                            stall_detect_timer = millis();  // reset timer  
                            RotorAZ_raw_last = RotorAZ_raw;  // update new position                           
                            compute_rotor_move(); 
                        }
                        if (cmd1 == 242) { 
                            Serial.print("Cmd1=");
                            Serial.print(cmd1);
                            MoveRotor = STOP; // valid options are CW, CCW or STOP
                            MovetoPreset = OFF;
                            Serial.println("  STOP ROTOR");
                            allOff(); 
                        }
                        if (cmd1 == 254) { 
                            Serial.print("Cmd1=");
                            Serial.print(cmd1);
                            MovetoPreset = ON; // valid options are CW, CCW or STOP 
                            constrain((uint8_t)cmd2, 0, 9); 
                            RotorTargetAZ = Preset[(uint8_t)cmd2];
                            Serial.print("  Move to Preset #");
                            Serial.print((uint8_t) cmd2);
                            Serial.print(" at "); 
                            Serial.println(RotorTargetAZ);   
                            if (RotorTargetAZ < RotorAZ_StartPos)
                                RotorTargetAZ += 360;
                            allOff();
                            delay(200);
                            stall_detect_timer = millis();  // reset timer  
                            RotorAZ_raw_last = RotorAZ_raw;  // update new position                           
                            compute_rotor_move();
                        }
                        if (cmd1 == 252) {                             
                            Serial.print("Cmd1=");
                            Serial.print(cmd1);                                                    
                            Serial.print("  Rotor Position is ");                            
                            Serial.println((uint8_t) RotorAZ);
                            sprintf((char*) tx_buffer, "%.2f", RotorAZ);                     
                            enet_write(tx_buffer, 1);
                        }                        
                        // New commands here
                   } // end of msg_type 120                                      
                } // end of msg_type length check
            } // end of meter ID OK    
        } // end of \n found
    } //end rx_count not 0
} // end get_remote_cmd function                       
// 
//____________________________________ stall_detect _________________________________________________
//
void stall_detect(void)
{
    // read position during movement and if the position does not change dor X duration,
    //    issue stop command, send back "STALL TIMEOUT" satus message.
    // Only called or works if movement command issued.  
    //    Could let all the time but causes lots of calls to the Alloff function
    if (!MoveRotor && !MovetoPreset)
        return;
    if (DBG==3)
    {
        Serial.print("Reset Stall Detector elapsed time =");
        Serial.println(RotorAZ_raw - RotorAZ_raw_last);
    }
    if ( abs(RotorAZ_raw - RotorAZ_raw_last) > STALL_DETECT_DISTANCE)
    {
        RotorAZ_raw_last = RotorAZ_raw;  // update new position
        stall_detect_timer = millis();   // reset timer
        if (DBG==3) Serial.println("Reset Stall Detector Timer");
    }
    else if (millis()- stall_detect_timer > STALL_TIMEOUT)  // if not moved far enough then check for timeout
    {
        if (DBG==3)
        {
            Serial.print("Timer now at ");
            Serial.println(millis() - stall_detect_timer);
        }
        MoveRotor = STOP;
        MovetoPreset = STOP;
        allOff();        
        sprintf((char *)tx_buffer, "\nERROR - Stall Detect Timer Expired, All Off! Timer= %lu\n", (unsigned long)STALL_TIMEOUT);
        send_status();
    }
}
// 
//____________________________________ compute_rotor_move _________________________________________________
//   
// compute what direction needed to take to avoid hard and manual limits.
// This is non-blocking.  Use stall_detect_timer().
void compute_rotor_move(void)
{ 
  float man_lim_tmp_CW;
  float man_lim_tmp_CCW;

  man_lim_tmp_CW = manual_limit_CW;
  if (man_lim_tmp_CW < RotorAZ_StartPos)
      man_lim_tmp_CW += 360;
  man_lim_tmp_CCW = manual_limit_CCW;
  if (man_lim_tmp_CCW < RotorAZ_StartPos)
      man_lim_tmp_CCW += 360;
      
  if (MoveRotor)  // for manual movement commands
  {    
      if (MoveRotor == CW)    
          move_CW();
      if (MoveRotor == CCW)
          move_CCW();
      if (MoveRotor == STOP)
      {
          MovetoPreset = OFF;
          allOff();      
          sprintf((char*) tx_buffer, "All OFF Called");
          send_status();
      }
  }
  if (MovetoPreset) // for preset or MoveTo commands where we need to decide which direction
  {                      //    to turn relative to where we are and dead zones blocking path there.
      // Start with computing dead zone and adjust target to not go into any dead zone      
      if (RotorTargetAZ > RotorAZ_raw)
      {   // Test for CW move to target possible   
          if (DBG == 4) Serial.print("Requested Move to Preset CW");
          if (DBG == 4) Serial.print(" - Manual Limit CW is ");    
          if (DBG == 4) Serial.println(manual_limit_CW);    
          // should not be in the dead zone or move out of it, only do that in manual mode          
          if ((man_lim_tmp_CCW < RotorAZ_raw) && (RotorTargetAZ < man_lim_tmp_CW))  
          {
              if (DBG == 4) Serial.println("Attempting Move to Preset CW");              
              if ( RotorTargetAZ < (RotorAZ_StartPos+360)) // check for hard stop limit
              {
                  if (DBG == 4) Serial.println(" Moving to Preset CW");
                  MoveRotor = CW;
                  move_CW();
              }
          }
          else
          {
              sprintf((char*) tx_buffer, " Current position inside limits, movement not allowed");              
              send_status();
              MovetoPreset = OFF;
          }
      }        
      else
      {
          if (DBG == 4) Serial.print("Requested Move to Preset CCW");  
          if (DBG == 4) Serial.print(" - Manual Limit CCW is ");    
          if (DBG == 4) Serial.println(manual_limit_CCW);
               
          if ((man_lim_tmp_CW > RotorAZ_raw) && (RotorTargetAZ > man_lim_tmp_CCW)) // try CCW path to target
          {    
              if (DBG == 4) Serial.println("Attempting Move to Preset CCW");
              if ( RotorTargetAZ  > RotorAZ_StartPos)
                  {
                      if (DBG == 4) Serial.println("Moving to Preset CCW");
                      MoveRotor = CCW;
                      move_CCW();
                  }
          }
          else
          {              
              sprintf((char*) tx_buffer, " Current position inside limits, movement not allowed");              
              send_status();
              MovetoPreset = OFF;
          }
      }
      if (abs(RotorTargetAZ - RotorAZ) <= Rotor_StopBand)   // stop rotor, we have arrived
      {
          allOff();
          MovetoPreset = OFF;
          MoveRotor = STOP;
          sprintf((char*) tx_buffer, "Destination Reached");
          send_status();
          return;
      }
  }
  else
      MovetoPreset = OFF;    
}
// 
//____________________________________ move_CW _________________________________________________
//
void move_CW(void)
{   
    float man_lim_tmp;

    man_lim_tmp = manual_limit_CW;
    if ( man_lim_tmp < RotorAZ_StartPos)
        man_lim_tmp += 360;
        
    if (RotorAZ_raw >= RotorTargetAZ)
    {
        allOff();
        sprintf((char*) tx_buffer, "Destination Reached");
        send_status();
    }
    else if (RotorAZ_raw > man_lim_tmp)
    {
        sprintf((char*) tx_buffer, "CW Manual limit reached! %.1f", manual_limit_CW);
        send_status();
        allOff();
    }
    else if (abs(RotorAZ_raw - 360 - RotorAZ_StartPos) <= Rotor_StopBand)
    {
        allOff();
        sprintf((char*) tx_buffer, " AZ Motion Stopped ");
        send_status();
    }
    else if (RotorTargetAZ - RotorAZ_raw < SlowdownDegrees || man_lim_tmp - RotorAZ_raw < SlowdownDegrees)  
    {
        rightSlow();
        sprintf((char*) tx_buffer, " Moving CW SLOW ");
        send_status();
    }
    else     
    {
        rightFast();
        sprintf((char*) tx_buffer, " Moving CW FAST ");              
        send_status();
    }
}
// 
//____________________________________ move_CCW _________________________________________________
//
void move_CCW(void)
{
    float man_lim_tmp;
    
    man_lim_tmp = manual_limit_CCW;
    if ( man_lim_tmp < RotorAZ_StartPos)
        man_lim_tmp += 360;
        
    if (RotorAZ_raw <= RotorTargetAZ)
    {
        allOff();
        sprintf((char*) tx_buffer, "Destination Reached");
        send_status();
    }
    else if (RotorAZ_raw < man_lim_tmp) 
    {
        sprintf((char*) tx_buffer, "CCW Manual limit reached! %.1f", manual_limit_CCW);
        send_status();
        allOff();
    }
    else if (abs(RotorAZ_raw - RotorAZ_StartPos) <= Rotor_StopBand)
    {
        allOff();
        sprintf((char*) tx_buffer, " AZ Motion Stopped");
        send_status();
    }
    else if ((RotorAZ_raw - RotorTargetAZ) < SlowdownDegrees || RotorAZ_raw - man_lim_tmp < SlowdownDegrees)
    {
        leftSlow();        
        sprintf((char*) tx_buffer, " Moving CCW SLOW"); 
        send_status();
    }
    else 
    {          
        leftFast();        
        sprintf((char*) tx_buffer, " Moving CCW FAST"); 
        send_status();
    }
}
// 
//____________________________________ leftFast _________________________________________________
//
void leftFast(void)
{
  digitalWrite(ROTOR_AC_POWER_PIN, 1);
  digitalWrite(LEFT_FAST_PIN, 0);
  digitalWrite(LEFT_SLOW_PIN, 1);
  digitalWrite(RIGHT_FAST_PIN, 1);
  digitalWrite(RIGHT_SLOW_PIN, 1);
  if(DBG==1) Serial.println("AC On, Turn LEFT FAST, Pin 1 On");
  digitalWrite(LED, OFF);
  send_pin_Status();
}
// 
//____________________________________ leftSlow _________________________________________________
//
void leftSlow(void)
{
  digitalWrite(ROTOR_AC_POWER_PIN, 1);
  digitalWrite(LEFT_FAST_PIN, 1);
  digitalWrite(LEFT_SLOW_PIN, 0);
  digitalWrite(RIGHT_FAST_PIN, 1);
  digitalWrite(RIGHT_SLOW_PIN, 1);
  if(DBG==1)Serial.println("AC On, Turn LEFT SLOW, Pin 2 On");
  digitalWrite(LED, ON);
  send_pin_Status(); 
}
// 
//____________________________________ rightFast _________________________________________________
//
void rightFast(void)
{
  digitalWrite(ROTOR_AC_POWER_PIN, 1);
  delay(100);
  digitalWrite(LEFT_FAST_PIN, 1);
  digitalWrite(LEFT_SLOW_PIN, 1);
  digitalWrite(RIGHT_FAST_PIN, 0);
  digitalWrite(RIGHT_SLOW_PIN, 1);
  if(DBG==1) Serial.println("AC On, Turn RIGHT FAST, Pin 3 On");
  digitalWrite(LED, OFF);
  send_pin_Status();
}
// 
//____________________________________ rightSlow _________________________________________________
//
void rightSlow(void)
{
  digitalWrite(ROTOR_AC_POWER_PIN, 1);
  digitalWrite(LEFT_FAST_PIN, 1);
  digitalWrite(LEFT_SLOW_PIN, 1);
  digitalWrite(RIGHT_FAST_PIN, 1);
  digitalWrite(RIGHT_SLOW_PIN, 0);
  if(DBG==1) Serial.println("AC On, Turn RIGHT SLOW, Pin 4 On");
  digitalWrite(LED, ON);
  send_pin_Status();
}
// 
//____________________________________ allOff _________________________________________________
//
void allOff(void)
{
  digitalWrite(ROTOR_AC_POWER_PIN, 0);
  digitalWrite(LEFT_FAST_PIN, 1);
  digitalWrite(LEFT_SLOW_PIN, 1);
  digitalWrite(RIGHT_FAST_PIN, 1);
  digitalWrite(RIGHT_SLOW_PIN, 1);
  if(DBG==1) Serial.println("All Off");
  digitalWrite(LED, OFF);
  send_pin_Status();
}
// 
//____________________________________ send_pin_Status _________________________________________________
//
void send_pin_Status(void)
{
    if(DBG==1)
    {
      Serial.print("AC Power=");
      Serial.print(digitalRead(ROTOR_AC_POWER_PIN));
      Serial.print("  Left Fast=");
      Serial.print(digitalRead(LEFT_FAST_PIN));
      Serial.print("  Left Slow=");
      Serial.print(digitalRead(LEFT_SLOW_PIN));
      Serial.print("  Right Fast=");
      Serial.print(digitalRead(RIGHT_FAST_PIN));
      Serial.print("  Right Slow=");
      Serial.println(digitalRead(RIGHT_SLOW_PIN));        
   }
}
// 
//____________________________________ init_relay_state _________________________________________________
//
void init_relay_state(void)
{
  if(DBG==1) Serial.println("Initializing relay driver state");
  pinMode(ROTOR_AC_POWER_PIN, OUTPUT);
  digitalWrite(ROTOR_AC_POWER_PIN, 0);
  pinMode(LEFT_FAST_PIN, OUTPUT);
  digitalWrite(LEFT_FAST_PIN, 1);
  pinMode(LEFT_SLOW_PIN, OUTPUT);
  digitalWrite(LEFT_SLOW_PIN, 1);
  pinMode(RIGHT_FAST_PIN, OUTPUT);
  digitalWrite(RIGHT_FAST_PIN, 1);
  pinMode(RIGHT_SLOW_PIN, OUTPUT);
  digitalWrite(RIGHT_SLOW_PIN, 1);
  if(DBG) Serial.println("Completed relay driver port config");
  send_pin_Status();
  delay(1000);
}
// 
//____________________________________ stress_test_relays _________________________________________________
//
void stress_test_relays(void)
{
  // Test section to operate the rotator control relays and turn on the motor transfomer 24VAC.
  digitalWrite(LED, ON);
 
  leftFast();
  delay(400);
  
  allOff();
  delay(200);
  
  leftSlow();
  delay(400);

  allOff();
  delay(200);
  
  rightFast();
  delay(400);
  
  allOff();
  delay(200);
  
  rightSlow();
  delay(400);

  allOff();
  delay(500);
}
// 
//____________________________________ set_pins _________________________________________________
//
void set_pins(void)
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, OFF);
  pinMode(ROTOR_AC_POWER_PIN, INPUT);
  pinMode(LEFT_FAST_PIN, INPUT);
  pinMode(LEFT_SLOW_PIN, INPUT);
  pinMode(RIGHT_FAST_PIN, INPUT);
  pinMode(RIGHT_SLOW_PIN, INPUT);
  pinMode(ROTOR_ANALOG_AZ_PIN, INPUT);
  digitalWrite(LED, ON);
  delay(500);
  digitalWrite(LED, OFF);
  delay(500);
  digitalWrite(LED, ON);
  delay(500);
}
