#include <Servo.h>
#include <SoftwareSerial.h>


#define SECOND_PERIOD_ms	1000
#define STEP_PERIOD_MAX		20
#define STEP_PERIOD_MIN		2
#define PEN_PIN				5
#define PEN_UP				20
#define PEN_DN				0
#define SER_CONN			1
int8_t ui8_StepPeriod_ms = STEP_PERIOD_MAX;

int16_t i16_AngleSteps;
uint16_t ui16_DistanceSteps;
float fCurrentX;
float fCurrentY;
float fTargetX;
float fTargetY;
float fCurrentAngle;
float fTargetAngle;

Servo PenServo;		// Servo for Pen Up/Down
int8_t i8_StepMot1;	//on Port B0-3
int8_t i8_StepMot2;	//on Port C0-3
#define MAX_STEPS 4
uint8_t ui8_StepsArray[MAX_STEPS];

SoftwareSerial SoftSerial(2, 3); // RX, TX

void StepMotors(void)
{	//There will allways be Angle first then Distance (after angle finished)
	if (i16_AngleSteps != 0)
	{
		if (i16_AngleSteps > 0)
		{
			i8_StepMot1++;
			if (i8_StepMot1 >= MAX_STEPS) { i8_StepMot1 = 0; }
			i8_StepMot2--;
			if (i8_StepMot2 < 0) { i8_StepMot2 = MAX_STEPS - 1; }
			i16_AngleSteps--;
		}
		else
		{
			i8_StepMot1--;
			if (i8_StepMot1 < 0) { i8_StepMot1 = MAX_STEPS - 1; }
			i8_StepMot2++;
			if (i8_StepMot2 >= MAX_STEPS) { i8_StepMot2 = 0; }
			i16_AngleSteps++;
		}
		PORTB = ui8_StepsArray[i8_StepMot1];
		PORTC = ui8_StepsArray[i8_StepMot2];
	}
	else if (ui16_DistanceSteps)
	{
		i8_StepMot1++;
		if (i8_StepMot1 >= MAX_STEPS) { i8_StepMot1 = 0; }
		i8_StepMot2++;
		if (i8_StepMot2 >= MAX_STEPS) { i8_StepMot2 = 0; }
		PORTB = ui8_StepsArray[i8_StepMot1];
		PORTC = ui8_StepsArray[i8_StepMot2];
		ui16_DistanceSteps--;
		if (ui16_DistanceSteps == 0) 
		{
			fCurrentX		= fTargetX;
			fCurrentY		= fTargetY;
			fCurrentAngle	= fTargetAngle;
			SoftSerial.print("\r\n>");
		}
	}
	else
	{
//		SoftSerial.print("\r\n>");
	}
}

#define K_DISTANCE_STEPS	13.8888888
//KA= D1/(2*KD) where D1 = distance between wheels and KD = K_DISTANCE_STEPS = 79/(2*13.8(8)) = 2.844
#define K_ANGLE_STEPS		555	//274.71		
void ui16CalculateSteps()
{
	if ((ui16_DistanceSteps == 0) && (i16_AngleSteps == 0))
	{
		ui16_DistanceSteps = K_DISTANCE_STEPS * sqrt(pow(fTargetX - fCurrentX, 2) + pow(fTargetY - fCurrentY, 2));
		fTargetAngle = atan2(fTargetY - fCurrentY, fTargetX - fCurrentX);
		i16_AngleSteps = K_ANGLE_STEPS * (fTargetAngle - fCurrentAngle);
		SoftSerial.print("D: "); SoftSerial.print(ui16_DistanceSteps); SoftSerial.print(" A: "); SoftSerial.print(i16_AngleSteps);
	}
	else
	{	SoftSerial.println("Err1>");	}
}

unsigned long previousMillisStep = millis();
unsigned long previousMillisStep2 = millis();

void setup()
{
	fCurrentX = 0;
	fCurrentY = 0;
	fTargetX = 0;
	fTargetY = 0;
	PenServo.attach(PEN_PIN);  // attaches the servo on pin 9 to the servo object
	PenServo.write(PEN_UP);
	//pinMode(PEN_PIN, OUTPUT);
	//digitalWrite(PEN_PIN, LOW);
	pinMode(A0, OUTPUT);
	pinMode(A1, OUTPUT);
	pinMode(A2, OUTPUT);
	pinMode(A3, OUTPUT);
	for (int i = 8; i <= 11; i++)
	{
		pinMode(i, OUTPUT);
	}
	i16_AngleSteps = 0;
	ui16_DistanceSteps = 0;
	if (SER_CONN) { Serial.begin(9600); }
	SoftSerial.begin(2400);	//57600
//	SoftSerial.write("AT+BAUD4\r\n");
//	SoftSerial.println("AT+UART=1200,0,0");
	SoftSerial.listen();
	i8_StepMot1 = 0;
	i8_StepMot2 = 0;
	ui8_StepsArray[0] = 0b0001;	ui8_StepsArray[1] = 0b0010;	ui8_StepsArray[2] = 0b0100;	ui8_StepsArray[3] = 0b1000;
	SoftSerial.print(">");
//	SoftSerial.print("@");
}

void parseCoordinate(char * cp_Cmd)
{
	char * cp_Tmp;
	char * cp_Str;
	cp_Str = strtok_r(cp_Cmd, " ", &cp_Tmp);	//now the number is in "cp_Str"
	while (cp_Str != NULL) {					//Subsequent calls, the first number (above) is gone
		cp_Str = strtok_r(0, " ", &cp_Tmp);		//Here we get X, Y or Z in a G1 command
		//Serial.printf("%s;",str);
		
		if (tolower(cp_Str[0]) == 'x')
		{	fTargetX = atof(cp_Str + 1);	}
		else if (tolower(cp_Str[0]) == 'y')
		{	fTargetY = atof(cp_Str + 1);	}
		else if (tolower(cp_Str[0]) == 'z')
		{	/*tarZ = atof(cp_Str + 1);	*/	}
	}
		ui16CalculateSteps();	//Also starts movement on next rotation/line
}

void parseGcode(char * cp_Cmd)
{
	int iCode;
	iCode = atoi(cp_Cmd);
	switch (iCode)
	{
	case 1: // xyz move
		parseCoordinate(cp_Cmd);
		break;
	case 28: // home
		fTargetX = 0; fTargetY = 0;
		//		mPenUp();
		///		prepareMove();
		///		initPosition();
		break;
	}
}

void parsePen(char * cmd)
{
	char * tmp;
	char * str;
	str = strtok_r(cmd, " ", &tmp);
	int pos = atoi(tmp);
	if (pos == 0)
	{	PenServo.write(PEN_UP);	}	//Pin Up
	else
	{	PenServo.write(PEN_DN);	}		//Pin Down
	SoftSerial.print("\r\n>");
}

void parseMcode(char * cmd)
{
	int code;
	code = atoi(cmd);
	switch (code) {
	case 1:
		parsePen(cmd);
		break;
	}
}

#define ACC_DEC_DIST		60
#define STEP_PERIOD_ACC		20		//How often (ms) we accellerate/decellerate
#define STEP_PERIOD_STEP	1
void vStepAccDec(void)
{
	previousMillisStep2 = millis();
	if (i16_AngleSteps != 0)
	{
		if ((abs(i16_AngleSteps) < ACC_DEC_DIST) && (ui8_StepPeriod_ms < STEP_PERIOD_MAX))
		{
			ui8_StepPeriod_ms += STEP_PERIOD_STEP;
		}
		else if (ui8_StepPeriod_ms > STEP_PERIOD_MIN)
		{
			ui8_StepPeriod_ms -= STEP_PERIOD_STEP;
		}
	}
	else if (ui16_DistanceSteps > 0)
	{
		if ((ui16_DistanceSteps < ACC_DEC_DIST) && (ui8_StepPeriod_ms < STEP_PERIOD_MAX))
		{
			ui8_StepPeriod_ms += STEP_PERIOD_STEP;
		}
		else if (ui8_StepPeriod_ms > STEP_PERIOD_MIN)
		{
			ui8_StepPeriod_ms -= STEP_PERIOD_STEP;
		}
		//			Serial.println(ui8_StepPeriod_ms);
	}

}

void parseCmd(char * cmd)
{
	if (SER_CONN) { Serial.write(cmd); }
	if (tolower(cmd[0]) == 'g')
	{ // gcode
		parseGcode(cmd + 1);
//		Serial.println("OK");
	}
	else if (tolower(cmd[0]) == 'm')
	{ // mcode
		parseMcode(cmd + 1);
//		Serial.println("OK");
	}
 else
 {  SoftSerial.print("\r\n>");  }
}

char	buf[64];
uint8_t	bufindex;
char	buf2[64];
uint8_t	bufindex2;
char cChar;

void loop()
{
	if ((unsigned long)(millis() - previousMillisStep) >= ui8_StepPeriod_ms)	//SECOND_PERIOD_ms MOT_STEP_PERIOD_ms
	{	previousMillisStep = millis();	StepMotors();	/*Serial.print(">"); */ }

	if ((unsigned long)(millis() - previousMillisStep2) >= STEP_PERIOD_ACC)	//We accelerate/decelerate
	{		vStepAccDec();	}




	if (SoftSerial.available() > 0)
	{
		cChar = SoftSerial.read();

		if ((cChar == '\n'))
		{
			buf[bufindex] = 0;
			parseCmd(buf);
//			SoftSerial.print("\r\n>");
			bufindex = 0;
		}
		else
		{
			buf[bufindex++] = cChar;
		}
	}




/*	if (SoftSerial.available() > 0)
	{
		char cChar = SoftSerial.read();
		if (cChar == 'n')
		{	SoftSerial.println("V0.0.4\r\n>");	}
		if (SER_CONN) { Serial.write(cChar); }
		SoftSerial.print(cChar);
		

		if ((cChar == '\n') || (cChar == '#')) 
		{
//			SoftSerial.print('n');
//			if (SER_CONN) { Serial.print("n"); }
//			if (SER_CONN) { Serial.write("x"); }
			if (SER_CONN) { Serial.print(buf); }
			parseCmd(buf);
			memset(buf, 0, 64);
			bufindex = 0;
		}
   else
	   {
	      buf[bufindex++] = cChar;
	   }
	}	*/

}
