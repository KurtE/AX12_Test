#ifdef INCLUDE_TIMED_TESTS
//=======================================================================================
void TimedMove(void) {
  word wID;
  word wFrom;
  word wTo;
  word wSpeed;
  word wCnt;
  unsigned long ulTimes[50];  
  byte iTimes;

  if (!FGetNextCmdNum(&wID))
    return;
  if (!FGetNextCmdNum(&wFrom))
    return;
  if (!FGetNextCmdNum(&wTo))
    return;
  if (!FGetNextCmdNum(&wSpeed))
    return;
  if (!FGetNextCmdNum(&wCnt))
    return;

  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print(" ");
  Serial.print(wFrom, DEC);
  Serial.print("-");
  Serial.print(wTo, DEC);
  Serial.print(" Speed: ");
  Serial.print(wSpeed, DEC);
  Serial.print(" Cnt: ");
  Serial.println(wCnt, DEC);

  // Print out some Compliance information...
  Serial.print("CW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_SLOPE, 1), DEC);
  Serial.print("CCW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CCW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_SLOPE, 1), DEC);

  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.
  ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
  WaitForMoveToComplete(wID);
  iTimes = 0;
  ulTimes[iTimes++] = millis();
  while (wCnt-- > 0) {
    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
  }
  unsigned long ulServoDelta4;
  unsigned long ulTimeDelta;
  if (wFrom >= wTo)
    ulServoDelta4 = (unsigned long)(wFrom-wTo)*1000L;
  else
    ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
  for (byte i=1; i < iTimes; i++) {
    Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
    Serial.print(" ");
    Serial.print(ulServoDelta4, DEC);
    Serial.print(" ");
    Serial.println(ulServoDelta4/ulTimeDelta, DEC);
  }
  Serial.print("Total Delta Time:");
  Serial.println(ulTimeDelta = ulTimes[iTimes-1]-ulTimes[0], DEC);
  Serial.print("Total Distance:");
  Serial.println(ulServoDelta4*(iTimes-1), DEC);
  Serial.print("Guess Time: ");
  // Units per ms = ((114*1024*360)/300))/(60*1000)
  Serial.println((((unsigned long)((ulServoDelta4/16)*(unsigned long)(iTimes-1))*3125L))/((unsigned long)(456L*(unsigned long)wSpeed)), DEC);
  // 

}


//=======================================================================================
void TimedMove2(void) {
  word wID;
  word wFrom;
  word wTo;
  word wSpeed;
  word wCnt;
  word wSlope;
  unsigned long ulTimes[10];
  unsigned long ulTotalDists4[10];
  unsigned long ulTotalTimes[10];
  byte iTotals;  
  byte iTimes;

  Serial.println("TimedMove 2");
  if (!FGetNextCmdNum(&wID))
    return;

  if (!FGetNextCmdNum(&wSpeed))
    return;

  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print(" Speed: ");
  Serial.println(wSpeed, DEC);
  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.

  // Print out some Compliance information...
  Serial.print("CW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CW Slope: ");
  wSlope = ax12GetRegister(wID, AX_CW_COMPLIANCE_SLOPE, 1);
  Serial.println(wSlope, DEC);
  Serial.print("CCW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CCW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_SLOPE, 1), DEC);
  Serial.print("Punch: ");
  Serial.println(ax12GetRegister(wID, AX_PUNCH_L, 2), DEC);
  Serial.print("Torque Limit: ");
  Serial.println(ax12GetRegister(wID, AX_TORQUE_LIMIT_L, 2), DEC);
  iTotals = 0;

  for (;wSlope <= 256; wSlope <<= 1) {
    wFrom = 512 - wSlope/2;
    wTo = wFrom + wSlope;
    wCnt = 2;    // twice each one should be enough...

    Serial.print(wFrom, DEC);
    Serial.print("-");
    Serial.print(wTo, DEC);
    Serial.flush();  // Make sure everything has been written out...

    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    delay(100);    // also give time to stabilize, so first move does not impact...
    iTimes = 0;
    ulTimes[iTimes++] = millis();
    while (wCnt-- > 0) {
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
      WaitForMoveToComplete(wID);
      ulTimes[iTimes++] = millis();
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
      WaitForMoveToComplete(wID);
      ulTimes[iTimes++] = millis();
    }
    unsigned long ulServoDelta4;
    unsigned long ulTimeDelta;

    ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
    for (byte i=1; i < iTimes; i++) {
      Serial.print(" ");
      Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
      Serial.print(":");
      Serial.print(ulServoDelta4/ulTimeDelta, DEC);
    }
    Serial.print(" Totals:");
    Serial.print(ulTotalTimes[iTotals] = ulTimes[iTimes-1]-ulTimes[0], DEC);
    ulTotalDists4[iTotals] = ulServoDelta4*(iTimes-1);
    Serial.print(":");
    Serial.println(ulTotalDists4[iTotals]/ulTotalTimes[iTotals], DEC);
    iTotals++;
  }

  // Now lets print out some total dists and some speeds...
  for (byte i=1; i < iTotals; i++) {
    Serial.print("DDist: ");
    Serial.print(ulTotalDists4[i-1], DEC);
    Serial.print("-");
    Serial.print(ulTotalDists4[i], DEC);
    Serial.print("=");
    Serial.print(ulTotalDists4[i]-ulTotalDists4[i-1], DEC);
    Serial.print(" DT: ");
    Serial.print(ulTotalTimes[i-1], DEC);
    Serial.print("-");
    Serial.print(ulTotalTimes[i], DEC);
    Serial.print("=");
    Serial.print(ulTotalTimes[i]-ulTotalTimes[i-1], DEC);
    Serial.print(" ? ");
    Serial.print((ulTotalDists4[i]-ulTotalDists4[i-1])/(ulTotalTimes[i]-ulTotalTimes[i-1]), DEC);
    Serial.print(" - ");  
    Serial.println(((ulTotalDists4[i]-ulTotalDists4[i-1])/(ulTotalTimes[i]-ulTotalTimes[i-1]))/wSpeed, DEC);
  }
}

//=======================================================================================
void TimedMove3(void) {
  word wID;
  word wFrom;
  word wTo;
  word wDT;
  word wDist;
  word wSpeed;
  word wCnt;
  word wSlope;
  unsigned long ulTimes[10];
  unsigned long ulStart;
  byte abT[10];    // only needed 4 for first test of this...
  uint8_t iCur;
  uint8_t i;
  word awCurPos[100];
  word awCurSpeed[100];

  byte iTotals;  
  byte iTimes;

  // <Servo> <Dist> <Time>");
  //speed value = (10000xRange(deg))/(6xTime(ms)).
  Serial.println("TimedMove 3");
  if (!FGetNextCmdNum(&wID))
    return;

  if (!FGetNextCmdNum(&wDist))
    return;

  if (!FGetNextCmdNum(&wDT))
    return;

  // now lets calculate a from/to and a guess on speed...
  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print("Dist ");
  wFrom = 512 - wDist/2;
  wTo = wFrom + wDist;
  Serial.print(wFrom, DEC);
  Serial.print("-");
  Serial.print(wTo, DEC);
  Serial.print("=");
  Serial.println(wDist, DEC);

  // From Zenta - speed value = (10000xRange(deg))/(6xTime(ms)).
  // 500 ms @ 180 deg range = 600
  // Guess a speed - The movement in degrees is: dist/1024 * 300 so
  // Speed = *wDist/1024)*10000 / (6 * wDT); 
  unsigned long ulT = (unsigned long)wDist*15625;
  wSpeed = max(ulT / ((unsigned long)wDT * 32L), 1);
  Serial.print("Desired Time: ");
  Serial.print(wDT, DEC);
  Serial.print(" ");
  Serial.print(ulT, DEC);
  Serial.print(" Speed: ");
  Serial.println(wSpeed, DEC);
  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.

  wCnt = 3;    

  Serial.flush();  // Make sure everything has been written out...

  ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
  WaitForMoveToComplete(wID);
  delay(100);    // also give time to stabilize, so first move does not impact...
  iTimes = 0;
  ulStart = ulTimes[iTimes++] = millis();
  iCur = 0;
  while (wCnt-- > 0) {
    do {
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
      while (millis()-ulStart < 25) 
        ;
      ulStart = millis();    
      GetMultax12Registers(wID, AX_PRESENT_POSITION_L, 4, abT);
      awCurPos[iCur] = abT[0] + (abT[1] << 8);    // Get the current position and speed.
      awCurSpeed[iCur] = abT[2] + (abT[3] << 8);
    } 
    while (awCurSpeed[iCur++]) ;    // See if checking the speed will work ok...
    ulTimes[iTimes++] = millis();

    // Pass 1 only do for one direction.
    for (i=0; i < iCur; i++) {
      Serial.print(awCurPos[i], DEC);
      Serial.print("=");
      Serial.print(awCurSpeed[i],DEC);
      if ((i % 10) == 9)
        Serial.println("");
      else
        Serial.print(" ");
    }  
    Serial.println("");
    Serial.flush();  // Make sure everything has been written out...



    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
  }
  unsigned long ulServoDelta4;
  unsigned long ulTimeDelta;

  ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
  for (byte i=1; i < iTimes; i++) {
    Serial.print(" ");
    Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
  }
  Serial.println("");
}
#endif


