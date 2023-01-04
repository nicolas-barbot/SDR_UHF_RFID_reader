/*This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.*/

#define TARI 20//between 6.25 and 25 micro second (int)
#define PW (unsigned)TARI/2		//between max(0.265*TARI,2) and 0.525*TARI micro second (int)
#define L0 (unsigned)TARI 		//TARI
#define L1 (unsigned)(2*TARI) 		//1.5 to 2 TARI
#define RTCAL L0+L1 			// 2.5 to 3 TARI
#define TRCAL (unsigned)(3*(RTCAL)) 	//1.1 to 3 RTCAL
#define DR 8000 			//divide ratio: 8 or 64/3
#define BLF (unsigned)(DR/TRCAL)
#define TRESH 16000/BLF 		//16e6 is the clock freq of the ATmega328P
#define TRESH_L (unsigned) (0.75*TRESH) //should be 0.75 (min 0.5, max 1)
#define TRESH_H (unsigned) (1.4*TRESH) //should be 1.25 (min 1, max 1.5)
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

byte command[22];
byte answer[160];
unsigned int timing[320];
int i_glob, j_glob;

void preamble();
void sync();
void send_data_0();
void send_data_1();
void RTcal();
void TRcal();
void Query();
//void Query(int DR, int M, int TRext, int Sel, int Session, char Target, int Q);
void QueryAdjust(int session, int updn);
void QueryRep(int session);
void ACK(byte frame[]);
void NACK(byte frame[]);
int check_crc16(byte PC[], byte EPC[], byte CRC[]);
void read_answer(int timeout);
void send_command(byte frame[], int length);
void print_command(byte frame[], int length);
void timer1_setup (byte mode, int prescale, byte outmode_A, byte outmode_B, byte capture_mode);

void setup()
{
  int i;
  
  Serial.begin(9600);
  pinMode(13, OUTPUT); //ENTX
  pinMode(12, OUTPUT); //ASKD
  digitalWrite(13,HIGH);
  digitalWrite(12,HIGH);

  pinMode(8, INPUT);    
  digitalWrite(8, HIGH);
  for (i = 0; i<150; i++)
    answer[i] = 0;
  for (i = 0; i<300; i++)
    timing[i] = 0;
  timer1_setup (0, 1, 0, 0, 0);
  Serial.println("RFID Reader V1.0");
  Serial.print("TARI = ");
  Serial.println(TARI);
  Serial.print("PW = ");
  Serial.println(PW);
  Serial.print("L0 = ");
  Serial.println(L0);
  Serial.print("L1 = ");
  Serial.println(L1);
  Serial.print("RTCAL = ");
  Serial.println(RTCAL);
  Serial.print("TRCAL = ");
  Serial.println(TRCAL);
  Serial.print("PIE Rate = ");
  Serial.print(2./(RTCAL)*1e3);
  Serial.println(" kb/s");
  Serial.print("BLF = ");
  Serial.print(BLF);
  Serial.println(" kb/s");

}

void loop()
{
  int i, j;
  byte *pt = NULL;
  unsigned int time2 = 0, ptime2 = 0xFFFF;
  float T1rn, T1epc, T1tmp;
  float T2;
  float Tpri0, Tpri1, Tpriacc, Tpri0acc, Tpri1acc;
  int nTpri, nTpri0, nTpri1, state;
  float min, min0, min1, avg, avg0, avg1, max, max0, max1;//, mdev;
  int L;
  byte RN16[16];
  byte PC[16];
  byte EPC[96];
  byte CRC[16];
    
  delay(500); // 3000 for ocsilloscope trigger (since reset also raised the trigger)

  Query();

  read_answer(int((10+22.5)*1./BLF*1e3));

  //FM0 preamble 10101v1 appears as 310121 in answer (or 30121 or 3121...).
  //Due to FM0 end of signaling (dummy bit), RN16 could be 16 or 17 bits...

  for (i = 0; i < 7; i++)
  {
    if (answer[i] == 2) // find violation
    {
      pt = &answer[i+2];
      break;
    }
  }
  if (i == 7)
  {
    //Serial.println("No RN16 detected");
    for (i = 0; i<150; i++)
      answer[i] = 0;
    i_glob = 0;
    j_glob = 0;
    return;
  }
  //check_ack()
  //NAK();
  ACK(pt);  //Can read answer to decrease T2
  
  read_answer(5000);  //timeout of 5 ms

  //CLR(PORTB, 5); //at the end of the round swith of the carrier

  for (i = 0; i < 7; i++)
  {
    if (answer[i] == 2) // find violation
    {
      pt = &answer[i+2];
      break;
    }
  }
  for (i = 0; i < 16; i++)
    RN16[i] = pt[i];
  
  for (i = 22; i < 29; i++)
  {
    if (answer[i] == 2) // find violation
    {
      pt = &answer[i+2];
      break;
    }
  }
  if (i == 29)
  {
    //Serial.println("No EPC detected");
    for (i = 0; i<150; i++)
      answer[i] = 0;
    i_glob = 0;
    j_glob = 0;
    return;
  }
  
  for (i = 0; i < 16; i++)
    PC[i] = pt[i];
  L = (PC[0]+2*PC[1]+4*PC[2]+8*PC[3]+16*PC[4])*8;

  for (i = 0; i < L; i++)
    EPC[i] = pt[i+16];

  for (i = 0; i < 16; i++)
    CRC[i] = pt[16+L+i];
  
  T1rn = timing[0]/16;
  Serial.print("T1 (RN16) = ");
  Serial.print(T1rn);
  Serial.println(" us");
  Serial.print("RN16[] = ");
  for (i = 0; i<16; i++)
  {
     Serial.print(RN16[i]);
     Serial.print(" ");
  }
  Serial.println("");
  for (i = 22; i < 38; i++)
  {
    if (timing[i] < timing[i-1])
    {
      T1epc = timing[i]/16;
      break;
    }
  }
  T2 = (10+22.5)*1./BLF*1e3 - timing[i-1]/16 + 245;
  Serial.print("T2 (approx) = ");
  Serial.print(T2);
  Serial.println(" us");
  Serial.print("T1 (EPC) = ");
  Serial.print(T1epc);
  Serial.println(" us");
  Serial.print("PC[] = ");
  for (i = 0; i<16; i++)
  {
     Serial.print(PC[i]);
     Serial.print(" ");
  }
  Serial.println("");
  Serial.print("L = ");
  Serial.println(L);
  Serial.print("EPC[] = ");
  for (i = 0; i<L; i++)
  {
     Serial.print(EPC[i]);
     Serial.print(" ");
  }
  Serial.println("");
  Serial.print("CRC[] = ");
  for (i = 0; i<16; i++)
  {
     Serial.print(CRC[i]);
     Serial.print(" ");
  }
  Serial.println("");
  if (check_crc16(PC, EPC, CRC)==0)
    Serial.println("CRC 0K");
  else
    Serial.println("CRC failed");

  min = min0 = min1 = 1e6;
  avg = avg0 = avg1 = 0;
  max = max0 = max1 = 0;
  Tpriacc = Tpri0acc = Tpri1acc = 0;
  nTpri = nTpri0 = nTpri1 = 0;
  
  i = 0;
  j = 0;
  while(timing[i] != 0)
  {
    time2 = timing[i] - timing[i-1];
    if (time2 < TRESH_L && ptime2 < TRESH_L)
    {
      Tpri0 = (ptime2 + time2)/16.;
      nTpri0 = nTpri0 + 1;
      Tpri0acc = Tpri0acc + Tpri0;
      if (min0 > Tpri0)
        min0 = Tpri0;
      if (max0 < Tpri0)
        max0 = Tpri0;
      ptime2 = 0xFFFF;
      goto end;
    }
    else if (time2 > TRESH_L && time2 < TRESH_H)
    {
      Tpri1 = time2/16.;
      nTpri1 = nTpri1 + 1;
      Tpri1acc = Tpri1acc + Tpri1;
      if (min1 > Tpri1)
        min1 = Tpri1;
      if (max1 < Tpri1)
        max1 = Tpri1;
    }
    ptime2 = time2;
end:i = i+1;
  }  
  avg0 = Tpri0acc/nTpri0;
  avg1 = Tpri1acc/nTpri1;
  min = min0 < min1 ? min0 : min1;
  max = max0 > max1 ? max0 : max1;
  Tpriacc = Tpri0acc + Tpri1acc;
  nTpri = nTpri0 + nTpri1;
  avg = Tpriacc/nTpri;
  Serial.print("Tpri (theo) = ");
  Serial.print(1e3/BLF);
  Serial.println(" us");
  Serial.print("Tpri0 min/avg/max = ");
  Serial.print(min0);
  Serial.print("/");
  Serial.print(avg0);
  Serial.print("/");
  Serial.print(max0);
  Serial.println(" us");
  Serial.print("Tpri1 min/avg/max = ");
  Serial.print(min1);
  Serial.print("/");
  Serial.print(avg1);
  Serial.print("/");
  Serial.print(max1);
  Serial.println(" us");
  Serial.print("Tpri min/avg/max = ");
  Serial.print(min);
  Serial.print("/");
  Serial.print(avg);
  Serial.print("/");
  Serial.print(max);
  Serial.println(" us");
  Serial.print("BLF (exp) = ");
  Serial.print(1e3/avg);
  Serial.println(" kb/s");
  
  for (i = 0; i<160; i++)
  {
     Serial.print(answer[i]);
     Serial.print(" ");
  }
  Serial.println("");
  
  Serial.print("THRESH_L = ");
  Serial.println(TRESH_L);
  Serial.print("THRESH_H = ");
  Serial.println(TRESH_H);
  Serial.print(timing[0]);
  for (i = 1; i<320; i++)
  {
    Serial.print(" ");
    Serial.print(timing[i]-timing[i-1]);
  }
  Serial.println("");
  for (i = 0; i<160; i++)
      answer[i] = 0;
  i_glob = 0;
  j_glob = 0;
  //while(1);
  return;
}

inline void preamble()
{
  CLR(PORTB, 4);
  delayMicroseconds(12);
  send_data_0();
  RTcal();
  TRcal();
  SET(PORTB, 4);
}

inline void sync()
{
  CLR(PORTB, 4);
  delayMicroseconds(12);
  send_data_0();
  RTcal();
  SET(PORTB, 4);
}

inline void send_data_0() //duration: TARI
{
  SET(PORTB, 4);
  delayMicroseconds(TARI-PW+2);
  CLR(PORTB, 4);
  delayMicroseconds(PW);
  SET(PORTB, 4);
}

inline void send_data_1() //duration: 1.5 to 2 TARI
{
  SET(PORTB, 4);
  delayMicroseconds(L1-PW+2);
  CLR(PORTB, 4);
  delayMicroseconds(PW);
  SET(PORTB, 4);
}

inline void RTcal() //duration: 2.5 to 3 TARI
{
  SET(PORTB, 4);
  delayMicroseconds(RTCAL-PW+2);  //data_0 + data_1
  CLR(PORTB, 4);
  delayMicroseconds(PW);
  SET(PORTB, 4);
}

inline void TRcal() //duration: 1.1 to 3 RTcal
{
  SET(PORTB, 4);
  delayMicroseconds(TRCAL-PW+2); 
  CLR(PORTB, 4);
  delayMicroseconds(PW);
  SET(PORTB, 4);
}

void Query()
{
  // QUERY 1000 0 00 0 00 00 0 0000 10000 (S=1)
  // QUERY 1000 0 00 0 00 00 0 0001 11001 (S=1)
  // QUERY 1000 0 00 0 00 00 0 0010 00010 (S=3)
  // QUERY 1000 0 00 0 00 00 0 0011 01011 (S=7)
  // QUERY 1000 0 00 0 00 00 0 0100 11101 (S=15)
  int i;
  
  for (i = 0; i < 22; i++)
    command[i] = 0;
  command[0] = 1;
  command[17] = 1;

  preamble();
  send_command(command, 22);
}

void ACK(byte frame[])
{
  command[0] = 0;
  command[1] = 1;
  sync();
  send_command(command, 2);
  send_command(frame, 16);
} 

void NAK()
{
  int i;
  command[0] = 1;
  command[1] = 1;
  for (i = 2; i < 8; i++)
    command[i] = 0;
  sync();
  send_command(command, 8);
}

int check_crc16(byte PC[], byte EPC[], byte CRC[])
{
  int i, j;
  byte reg[16];
  byte gen[17] = {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  byte res[16] = {0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1};
  byte pre[16] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  
  byte trame[144];
  int error = 0;
  
  for (i = 0; i < 144; i++)
    trame[i] = 0;
  for (i = 0; i < 16; i++)
    trame[i] = PC[i];
  for (i = 0; i < 96; i++)
    trame[i+16] = EPC[i];
  for (i = 0; i < 16; i++)
    trame[i+112] = CRC[i]; 

  for (i = 0; i < 16; i++)
    if (pre[i])
      reg[i] = trame[i] ? 0:1;  //preset

  for (i = 16; i < 144; i++)
  {
    if (reg[0] == 1)
    {
      for (j= 0; j < 15; j++)
        reg[j] = (reg[j+1] + gen[j+1])%2;
      reg[15] = (trame[i]+ gen[16])%2;
    }
    else
    {
      for (j = 0; j < 15; j++)
        reg[j] = reg[j+1];
      reg[15] = trame[i];
    }
  }
  for (i = 0; i < 16; i++)
  {
    if (reg[i] != res[i])
    {
      error = 1;
      break;
    }
  }
  return error;
}

void send_command(byte frame[], int length)
{
  int i;
  
  for (i = 0; i < length; i++)
  {
    if (frame[i] == 0)
      send_data_0();
    else if (frame[i] == 1)
      send_data_1();
  }
}

void print_command(byte frame[], int length)
{
  int i;
  
  Serial.print("frame[] = ");
  for (i = 0; i < length; i++)
  {
    Serial.print(frame[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void timer1_setup (byte mode, int prescale, byte outmode_A, byte outmode_B, byte capture_mode)
{
  // enforce field widths for sanity
  mode &= 15 ;
  outmode_A &= 3 ;
  outmode_B &= 3 ;
  capture_mode &= 3 ;

  byte clock_mode = 0 ; // 0 means no clocking - the counter is frozen.
  switch (prescale)
  {
    case 1: clock_mode = 1 ; break ;
    case 8: clock_mode = 2 ; break ;
    case 64: clock_mode = 3 ; break ;
    case 256: clock_mode = 4 ; break ;
    case 1024: clock_mode = 5 ; break ;
    default:
      if (prescale < 0)
        clock_mode = 7 ; // external clock
  }
  TCNT1 = 0;
  TCCR1A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
  TCCR1B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
}

void read_answer(int timeout)
{
  TIFR1 = 1<<5;
  TIMSK1 = 1<<5;
  TCNT1 = 0;
  delayMicroseconds(timeout);
  TIFR1 = 0<<5;
  TIMSK1 = 0<<5;
}

ISR(TIMER1_CAPT_vect)
{
  static unsigned int time = 0, ptime = 0xFFFF;

  TCCR1B = TCCR1B ^ (1<<6);
  timing[i_glob] = ICR1;
  time = timing[i_glob] - timing[i_glob-1];
  i_glob = i_glob + 1;
  if (time < TRESH_L && ptime < TRESH_L)
  {
    answer[j_glob++] = 0;
    ptime = 0xFFFF;
    return;
  }
  else if (time > TRESH_L && time < TRESH_H)
    answer[j_glob++] = 1;
  else if (time > TRESH_H && time < 2*TRESH_H)
    answer[j_glob++] = 2; // violation
  else if (time > 2*TRESH_H)
    answer[j_glob++] = 3; // carrier
  ptime = time;
}
