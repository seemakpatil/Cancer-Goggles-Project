#include "stdafx.h"
#include <stdio.h>
#include <conio.h>

using namespace System;
using namespace System::IO::Ports;
using namespace std;

enum
{
	KEY_ESC = 27,
	ARROW_UP = 256 + 72,
	ARROW_DOWN = 256 + 80,
	ARROW_LEFT = 256 + 75,
	ARROW_RIGHT = 256 + 77
};

public ref class Arduino
{
private:
	SerialPort^ sp;
public:
	Arduino() {
		sp = gcnew SerialPort("com3", 115200);
	}
	void digitalWrite(int motorPin, int lh) {
		String^ s = Convert::ToString((Char)50);
		s += Convert::ToString((Char)(97 + motorPin));
		s += Convert::ToString((Char)(48 + lh));
		sp->WriteLine(s);
	}
	void forward();
	void backward();
	void pinMode(int motorPin, int lh) {
		sp->Open();
		String^ s = Convert::ToString((Char)48);
		s += Convert::ToString((Char)(97 + motorPin));
		s += Convert::ToString((Char)(48 + lh));
		sp->WriteLine(s);
		sp->Close();
	}
	void digitalRead(int motorPin) {
		sp->Open();
		String^ s = Convert::ToString((Char)49);
		s += Convert::ToString((Char)(97 + motorPin));
		sp->WriteLine(s);
		sp->Close();
	}
};

void Arduino::forward() {

	sp->Open();
	int motorPin1 = 8;
	int motorPin2 = 9;
	int motorPin3 = 10;
	int motorPin4 = 11;
	int LOW = 0;
	int HIGH = 1;

	digitalWrite(motorPin4, HIGH);
	digitalWrite(motorPin3, HIGH);
	digitalWrite(motorPin2, LOW);
	digitalWrite(motorPin1, LOW);

	digitalWrite(motorPin4, LOW);
	digitalWrite(motorPin3, HIGH);
	digitalWrite(motorPin2, HIGH);
	digitalWrite(motorPin1, LOW);

	digitalWrite(motorPin4, LOW);
	digitalWrite(motorPin3, LOW);
	digitalWrite(motorPin2, HIGH);
	digitalWrite(motorPin1, HIGH);

	digitalWrite(motorPin4, HIGH);
	digitalWrite(motorPin3, LOW);
	digitalWrite(motorPin2, LOW);
	digitalWrite(motorPin1, HIGH);
	sp->Close();
}

void Arduino::backward() {
	sp->Open();
	int motorPin1 = 8;
	int motorPin2 = 9;
	int motorPin3 = 10;
	int motorPin4 = 11;
	int LOW = 0;
	int HIGH = 1;

	digitalWrite(motorPin1, HIGH);
	digitalWrite(motorPin2, HIGH);
	digitalWrite(motorPin3, LOW);
	digitalWrite(motorPin4, LOW);

	digitalWrite(motorPin1, LOW);
	digitalWrite(motorPin2, HIGH);
	digitalWrite(motorPin3, HIGH);
	digitalWrite(motorPin4, LOW);

	digitalWrite(motorPin1, LOW);
	digitalWrite(motorPin2, LOW);
	digitalWrite(motorPin3, HIGH);
	digitalWrite(motorPin4, HIGH);

	digitalWrite(motorPin1, HIGH);
	digitalWrite(motorPin2, LOW);
	digitalWrite(motorPin3, LOW);
	digitalWrite(motorPin4, HIGH);
	sp->Close();
}


static int get_code(void)
{
	int ch = getch();

	if (ch == 0 || ch == 224)
		ch = 256 + getch();

	return ch;
}

int main()
{
	Arduino a;
	int ch;
	try
	{
		while ((ch = get_code()) != KEY_ESC) {
			switch (ch) {
			case ARROW_UP:
				printf("UP\n");
				break;
			case ARROW_DOWN:
				printf("DOWN\n");
				break;
			case ARROW_LEFT:
				a.backward();
				break;
			case ARROW_RIGHT:
				a.forward();
				break;
			}
		}
	}
	catch (IO::IOException^ e)
	{
		Console::WriteLine(e->GetType()->Name + ": Port is not ready, check line 90 of code");
	}
	catch (ArgumentException^ e)
	{
		Console::WriteLine(e->GetType()->Name + ": incorrect port name syntax, must start with COM/com,"
			+ "check line 90 of code");
	}
	Console::Write("Press enter to close the program");
	Console::Read();
	return 0;
}
