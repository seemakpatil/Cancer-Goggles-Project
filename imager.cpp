//get both full frames of imagers, by aligning the incoming data, and save the last portion to an additional buffer
#include <Windows.h>
#include <process.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <conio.h>
#include <sstream>
#include <string>
#include <vector>
#include "opencv/build/include/opencv2/opencv.hpp"
#include "opencv/build/include/opencv2/core/core.hpp"
#include "opencv/build/include/opencv2/imgproc/imgproc.hpp"
#include "opencv/build/include/opencv2/highgui/highgui.hpp"
//#include <opencv/highgui.h>
#include <omp.h>
#include "pvf.h"
#include "colormap.cpp"
//#include <cmath>
using namespace std;
using namespace cv;

// Yu's hotfix on the code
typedef struct __hotfix_t {
	// buf_b2 buffer over flow, when startpoint is not correct
	// add MAX constrain to make sure in bound.
	static const long BUF_MAX = 406080; // avoid out of bound exception at line 977

	// add a constant offset to x and y disparity
	// then the NIR image could perfectly overlay on the original image
	static const int DISPARITY_FIX_X = -100; // line 757
	static const int DISPARITY_FIX_Y = -100; 

	// tune the video rate
	static const int FRAME_RATE = 15; // line 2015
}HOTFIX;



#include "okFrontPanelDLL.h"
#define MICRON_CLOCK_DIVIDER		70
#define CONFIGURATION_FILE         "../verilogCode/goggleimager_flip.bit"//goggleimager_master_single.bit"//goggleimager_slave_single.bit"
#define FRAMES						
#define SDRAM_SIZE                 (1*1692*480)	 // (32*1024*1024)=32MB
#define READBUF_SIZE               (1*1692*480)// (8*1024*1024)=8MB read buffer size
#define IMAGE_ROWS					480//480
#define IMAGE_COLS					1692
#define HEIGHT						480//480
#define WIDTH						1692

unsigned char *ImageBuffer;
unsigned int frameT0[HEIGHT][WIDTH]={0};
unsigned int frameT1[HEIGHT][WIDTH]={0};
unsigned int frameT2[HEIGHT][WIDTH]={0};
unsigned int frameT3[HEIGHT][WIDTH]={0};
int videoFileCount=1;
int videoNumber=1;
int oldFrameNumber=10;
int newFrameNumber=0;
bool save=false;
bool video=false;
bool validImage=true;
bool firstVideoFrame=true;
//checkbox for superimpose//
bool SuperImposeEnable=false;
bool SuperImposeRecordEnable=false;
bool AutoDisEnable=false;
bool BilinearEnable=false;
int disparity_x;
int disparity_y;
int disp_threshold;
int disp_upthreshold;
int transparency;
int sizeratio;
int pressedKey = 0;

bool g_buf1done=true;
bool g_buf2done=true;
bool g_bufdone=true;
bool g_buf1busy=false;
bool g_buf2busy=false;
bool g_bufbusy=false;

struct corXY {
	int xaxis;
	int yaxis;
	int Sxaxis;
	int Syaxis;
};


pvfHeader vHeader; 

FILE *fw;
char headerInfo[100];
unsigned int regAGC, regAER;
vector<char> x_data(HEIGHT*WIDTH);
vector<char*> x(HEIGHT);
vector<int> xy_data(HEIGHT*WIDTH);
vector<int*> xy(HEIGHT);
///
vector<int> threshold_data(HEIGHT*WIDTH);
vector<int*> xyth(HEIGHT);

int num_threads=omp_get_max_threads();
//ofstream outFile("../videoData/videoData.bin",ios::out|ios::binary);
ofstream outFile;

unsigned char *g_buf,*g_buf1,*g_buf2;
unsigned char *g_buf_1,*g_buf_2;
long long g_nMems, g_nMemSize;
int rows=IMAGE_ROWS;
int columns=IMAGE_COLS/2;
int integration1, integration2;
int integration1_temp, integration2_temp;
Mat raw,raw_align;
Mat raw_visible (rows,columns,DataType<Vec3b>::type);
Mat logo;

Mat raw_overlay(rows,columns,DataType<Vec3b>::type);
Mat raw_overlay_HSV (rows,columns,DataType<Vec3f>::type);
Mat raw_overlay_RGB (rows,columns,DataType<Vec3b>::type);
Mat raw_data1;
Mat raw_align_jet;
Rect RoI;

Mat raw_save(rows,columns*2,DataType<Vec3b>::type);;

unsigned char *x_buf;

int ImageData_B, ImageData_G,ImageData_R, ImageData_G2;

int const_B, const_G, const_R;

int const_B1, const_B2, const_B3;
int const_G1, const_G2, const_G3;
int const_R1, const_R2, const_R3;
int const_gain, const_gamma;

	okCFrontPanel *dev;	

int getBinFileNumber(void)
{
	int currentFileNumber=1;
	int nextFileNumber=1;
	char str[2000];
	ifstream read_file("./videoData/binFileNumber.dat");
	while (!read_file.eof() /*&& read_file!='\0'*/){
		read_file.getline(str,2000);
	}
	read_file.close();
	currentFileNumber=atoi(str);
	nextFileNumber=currentFileNumber+1;
	ofstream write_file;
	write_file.open("./videoData/binFileNumber.dat");
	write_file << nextFileNumber;
	write_file.close();
	return currentFileNumber;
}



void recordProgrammingEvents(unsigned int regAddress, unsigned int regValue)
{
	time_t rawtime;
	time ( &rawtime );
	std::fstream videoInfo("./videoData/videoInfo.dat");  
	videoInfo.seekp(0, std::ios::end); 
	int pos = videoInfo.tellp(); 
	if(pos > 0) 
	{ 
		videoInfo.seekp(pos - 1, std::ios::beg); 
	} 
	videoInfo << "\n\n";
	videoInfo << "Program time: " << ctime(&rawtime);
	videoInfo << "Register address: " << regAddress ;
	videoInfo << "\t Register value: " << regValue ;
	videoInfo << ".";
	videoInfo.close();
}

void recordProgrammingEventsDual(unsigned int regAddress, unsigned int regValue, unsigned int regSensor)
{
	time_t rawtime;
	time ( &rawtime );
	std::fstream videoInfo("./videoData/videoInfo.dat");  
	videoInfo.seekp(0, std::ios::end); 
	int pos = videoInfo.tellp(); 
	if(pos > 0) 
	{ 
		videoInfo.seekp(pos - 1, std::ios::beg); 
	} 
	videoInfo << "\n\n";
	videoInfo << "Program time: " << ctime(&rawtime);
	videoInfo << "Register address: " << regAddress ;
	videoInfo << "\t Register value: " << regValue ;
	videoInfo << "\t Corresponding Sensors: " << regSensor ;
	videoInfo << ".";
	videoInfo.close();
}



void readRegisterDual(okCFrontPanel *dev, unsigned int regAddress, unsigned int regSensor)
{
	unsigned int Status;
	unsigned int commandvalue, commandaddr;

	dev->SetWireInValue(0x06, 0x0001); 
	dev->SetWireInValue(0x08, regAddress); 
	//dev->SetWireInValue(0x0B, regSensor);
	if (regSensor==1) {
	dev->SetWireInValue(0x0B, 0xB8B9);
	} else {
	dev->SetWireInValue(0x0B, 0x9091);
	}
	dev->UpdateWireIns();

	do {
		dev->UpdateWireOuts();
		Status = dev->GetWireOutValue(0x23);
	} while (!(Status & 0x0002));

	dev->SetWireInValue(0x06, 0x0002);
	dev->UpdateWireIns();

	dev->UpdateWireOuts();
	commandvalue=dev->GetWireOutValue(0x27);
	commandaddr=dev->GetWireOutValue(0x28);
	cout<<hex<< commandvalue <<"\t"<<hex<<commandaddr<<"\t"<<regSensor<<endl;

	dev->SetWireInValue(0x06, 0x0000); 
	dev->UpdateWireIns();
	//recordProgrammingEvents(regAddress, regValue);

}


void programRegisterDual(okCFrontPanel *dev, unsigned int regAddress, unsigned int regValue, unsigned int regSensor)
{
	unsigned int Status;
	unsigned int commandvalue, commandaddr;

	//dev->SetWireInValue(0x06, 0x0000); 
	dev->SetWireInValue(0x07, 0x0001); 
	dev->SetWireInValue(0x08, regAddress); 
	dev->SetWireInValue(0x09, regValue);
	//dev->SetWireInValue(0x0B, regSensor);
	if (regSensor==1) {
	dev->SetWireInValue(0x0B, 0xB8B9);
	} else {
	dev->SetWireInValue(0x0B, 0x9091);
	}
	dev->UpdateWireIns();

	do {
		dev->UpdateWireOuts();
		Status = dev->GetWireOutValue(0x23);
	} while (!(Status & 0x0001));

	dev->SetWireInValue(0x07, 0x0000); 
	dev->UpdateWireIns();

	dev->UpdateWireOuts();
	Status = dev->GetWireOutValue(0x23);
	cout<<Status<<endl;
	
	dev->SetWireInValue(0x06, 0x0001);
	dev->SetWireInValue(0x08, regAddress);
	dev->UpdateWireIns();

	//read the command from the imager
	do {
		dev->UpdateWireOuts();
		Status = dev->GetWireOutValue(0x23);
		//cout<<Status<<"!"<<endl;
	} while (!(Status & 0x0002));

	dev->SetWireInValue(0x06, 0x0002);
	dev->UpdateWireIns();

	cout<<Status<<endl;

	dev->UpdateWireOuts();
	commandvalue=dev->GetWireOutValue(0x27);
	commandaddr=dev->GetWireOutValue(0x28);
	cout<<hex<< commandvalue <<"\t"<<hex<<commandaddr<<"\t"<<hex<<regSensor<<endl;

	dev->SetWireInValue(0x06, 0x0000); 
	dev->UpdateWireIns();
	recordProgrammingEvents(regAddress, regValue);
}


int readUserInput(void){
	string entry;
	int userInput;
	
	cin >> entry;
	char *input=new char[entry.size()+1];
	input[entry.size()]=0;
	memcpy(input,entry.c_str(),entry.size());
	userInput=atoi(input);
	cout << "You entered: " << userInput << "\n\n";

	return userInput;
}


void readImagerCommandDual(okCFrontPanel *dev)
{
	unsigned int reg_address;
	
	////Register address: 0x00 Chip Version
	//reg_address=0x00B9;   
	//readRegisterDual(dev, reg_address, 1);

	//Register address: 0xB9 LVDS Data Output
	reg_address=0x00B9;   
	readRegisterDual(dev, reg_address, 0);
	
	//Check the error flag
	reg_address=0x00B8;   
	readRegisterDual(dev, reg_address, 0);

}




void programImagerStereo(okCFrontPanel *dev)
{
	unsigned int reg_address, reg_value; 
	
	//Broadcast write to de_assert LVDS power_down
	//master
	reg_address=0x00B1; reg_value=0x0000;   
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	//Individual write to the master clock
	reg_address=0x00B1; reg_value=0x0001;   
	programRegisterDual(dev, reg_address, reg_value, 0);

	//To both sensor to set stereoscopic mode
	reg_address=0x0007; reg_value=0x0038;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	////////////////////////////////////////////////////////////

	// Register address: 0x0F Pixel Operation Mode
	// Value: Default=0x0011 monochrome & linear operation
	// Value: 0x0015 color & linear operation
	// Value: 0x0051 monochrome & high dynamic range
	// Value: 0x0055 color & high dynamic range
	reg_address=0x000F; reg_value=0x0002;  //034 0x0102 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	
	reg_address=0x000F; reg_value=0x0102;  //034 0x0002 
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	// Register address: R0x0D Read Mode register
	// Value: Nomal=0x0300, RowFlip=0x0310, 
	// ColFlip=0x0320 Row&ColFlips=0x0330	
	reg_address=0x000D; reg_value=0x0300;  
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	// Register address: R0xAF AGC/AEC Enable
	// Value: on_AGC&AEC=0x0003, off_AGC&AEC=0x0000
	// Value: on_AGConly=0x0002, on_AEConly=0x0001 
	reg_address=0x00AF; reg_value=0x0000; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	
	reg_address=0x00AF; reg_value=0x0000; 
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	// Register address: R0xA5 AEC/AGC Desired Bin [set brightness level during auto]
	// Value: Default=0x003A(58), Range=[0x0001(1)] to [0x0040(64)]
	reg_address=0x00A5; reg_value=0x003A; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	// Register address: R0x35 Analog Gain [set manual gain]
	// Value: Default=0x0010(16), Range=[0x0010(16)] to [0x0040(64)] 
	reg_address=0x0035; reg_value=0x0010; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	// Register address: R0x70 Row Noise Correction Control 1
	// Value: Default=0x0034 (Enable), 0x0014 (Disable)
	reg_address=0x0070; reg_value=0x0001; //034 0x0001
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	
	reg_address=0x0070; reg_value=0x0001; //034 0x0000
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	// Register address: R0x72 Row Noise Constant
	// Value: Default=42, Range=0to255
	/*reg_address=0x0072; reg_value=10; 
	programRegister(dev, reg_address, reg_value);*/

	// Register address: R0x0B Total Shutter Width [set manual exposure]
	// Value: Default=0x01E0(480), Range=[0x0001(1)] to [0x7FFF(32767)] 
	reg_address=0x000B; reg_value=0x01E0; //0x01E0; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	
	reg_address=0x0003; reg_value=0x01E0; //0x01E0; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	reg_address=0x0004; reg_value=0x02F0; //0x01E0; 
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	//////////////////////////////////////////////////////
	
	reg_address=0x00B3; reg_value=0x0000;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	reg_address=0x00B2; reg_value=0x0000;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	//enable the bypassing PLL in master sensor
	reg_address=0x00B1; reg_value=0x0000;   
	programRegisterDual(dev, reg_address, reg_value, 0);

	//enable the slave sensor internal PLL
	reg_address=0x00B1; reg_value=0x0000;   
	programRegisterDual(dev, reg_address, reg_value, 1);

	////////////////////////////////////
	//set slave as a stereo slave
	//reg_address=0x0007; reg_value=0x0028;   
	//programRegisterDual(dev, reg_address, reg_value, 0);
	////////////////////////////////////////

	//set slave as a stereo slave
	reg_address=0x0007; reg_value=0x0078;   
	programRegisterDual(dev, reg_address, reg_value, 1);

	//shift clock out
	reg_address=0x00B2; reg_value=0x0000; //old 0000, new 0007  
	//master
	programRegisterDual(dev, reg_address, reg_value, 1);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 0);

	//master//control ser_data_in
	reg_address=0x00B3; reg_value=0x0000;   //old 0000, new 0004
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	reg_address=0x00B4; reg_value=0x0000;   //old 0000, new 0000
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	//
	reg_address=0x00B5; reg_value=0x0001;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	////slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	
	

	reg_address=0x00B7; reg_value=0x0007;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);
	// individual write to B2 B3 B4 appropriately, and use B7, B8 to get lockstep feedback froms stereo error flag
	reg_address=0x00B7; reg_value=0x0003;   
	////master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);


	//a soft reset
	reg_address=0x000C; reg_value=0x0001;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);

	reg_address=0x000C; reg_value=0x0000;  
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	//slave
	programRegisterDual(dev, reg_address, reg_value, 1);


}



void programImagerErrorFlag(okCFrontPanel *dev) {

	unsigned int reg_address, reg_value; 
	
	reg_address=0x00B7; reg_value=0x0007;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	////slave
	//programRegisterDual(dev, reg_address, reg_value, 1);
	// individual write to B2 B3 B4 appropriately, and use B7, B8 to get lockstep feedback froms stereo error flag
	reg_address=0x00B7; reg_value=0x0003;   
	////master
	programRegisterDual(dev, reg_address, reg_value, 0);
	////slave
	//programRegisterDual(dev, reg_address, reg_value, 1);
}


int getVideoNumber(void)
{
	int currentVideoNumber=1;
	int nextVideoNumber=1;
	char str[2000];
	ifstream read_file("./videoData/videoNumber.dat");
	while (!read_file.eof() /*&& read_file!='\0'*/){
		read_file.getline(str,2000);
	}
	read_file.close();
	currentVideoNumber=atoi(str);
	nextVideoNumber=currentVideoNumber+1;
	ofstream write_file;
	write_file.open("./videoData/videoNumber.dat");
	write_file << nextVideoNumber;
	write_file.close();
	return currentVideoNumber;
}



int LoadOpalKellyDLL(void)
{
	char dll_date[32], dll_time[32];

    if (FALSE == okFrontPanelDLL_LoadLib(NULL))  return(-1);

  okFrontPanelDLL_GetVersion(dll_date, dll_time);
  printf("FrontPanel DLL loaded.  Built: %s  %s\n", dll_date, dll_time);
}

okCFrontPanel *initializeFPGA()
{
	okCFrontPanel *dev;

	// Open the first XEM - try all board types.
	dev = new okCFrontPanel;
	if (okCFrontPanel::NoError != dev->OpenBySerial()) {
		delete dev;
		printf("Device could not be opened.  Is one connected?\n");
		return(NULL);
	}
	
	switch (dev->GetBoardModel()) {
		case okCFrontPanel::brdXEM3001v1:
			printf("Found a device: XEM3001v1\n");
			break;
		case okCFrontPanel::brdXEM3001v2:
			printf("Found a device: XEM3001v2\n");
			break;
		case okCFrontPanel::brdXEM3005:
			printf("Found a device: XEM3005\n");
			break;
		case okCFrontPanel::brdXEM3010:
			printf("Found a device: XEM3010\n");
			break;
		case okCFrontPanel::brdXEM3020:
			printf("Found a device: XEM3020\n");
			break;
		case okCFrontPanel::brdXEM3050:
			printf("Found a device: XEM3050\n");
			break;
		case okCFrontPanel::brdXEM5010:
			printf("Found a device: XEM5010\n");
			break;
		default:
			printf("Unsupported device.\n");
			delete dev;
			return(NULL);
	}


	// Configure the PLL appropriately
	//dev->LoadDefaultPLLConfiguration();

    // initialize pll
	okCPLL22393 *pll = new okCPLL22393;
	pll->SetReference(48.0f);
	pll->SetPLLParameters(0, 400, 48);
	pll->SetOutputSource(0, okCPLL22393::ClkSrc_PLL0_0);
	pll->SetOutputDivider(0, 4); //set sdram clock to 100MHz (400MHz/4)
	pll->SetOutputEnable(0, true);

	pll->SetPLLParameters(1, 400, 48);
	pll->SetOutputSource(1, okCPLL22393::ClkSrc_PLL1_0);
	pll->SetOutputDivider(1, MICRON_CLOCK_DIVIDER);
	pll->SetOutputEnable(1, true);

	pll->SetPLLParameters(2, 400, 48);
	pll->SetOutputSource(2, okCPLL22393::ClkSrc_PLL2_0);
	pll->SetOutputDivider(2, 3);   
	pll->SetOutputEnable(2, true);

	dev->SetPLL22393Configuration (*pll);

	// Get some general information about the XEM.
	printf("Device firmware version: %d.%d\n", dev->GetDeviceMajorVersion(), dev->GetDeviceMinorVersion());
	printf("Device serial number: %s\n", dev->GetSerialNumber().c_str());
	printf("Device ID string: %s\n", dev->GetDeviceID().c_str());

	// Download the configuration file.
	if (okCFrontPanel::NoError != dev->ConfigureFPGA(CONFIGURATION_FILE)) {
		printf("FPGA configuration failed.\n");
		std::cout << "error code" << dev->ConfigureFPGA(CONFIGURATION_FILE) << std::endl;
		delete dev;
		return(NULL);
	}

	// Check for FrontPanel support in the FPGA configuration.
	if (false == dev->IsFrontPanelEnabled()) {
		printf("FrontPanel support is not enabled.\n");
		delete dev;
		return(NULL);
	}

	printf("FrontPanel support is enabled.\n");

	return(dev);
}



int InitFPGA(okCFrontPanel **dev)
{
	int LoadOpalKellySuccess=LoadOpalKellyDLL();
	if (LoadOpalKellySuccess==-1){
		printf("FrontPanel DLL could not be loaded.\n");
		return(-1);
	}

	*dev = initializeFPGA();
	if (NULL == *dev) {
		printf("FPGA could not be initialized.\n");
		return(-2);
	}

	return 0;
}




void saveFrame(int state, void *dummy)
{
	cout << "Should save snapshot" << endl;
	save=TRUE;
}

void saveVideo(int state, void *dummy)
{
	cout << "Should save video" << endl;
	video=!video;
}

void changeWB_B(int constvalue, void *dummy){
	const_B=constvalue;
}

void changeWB_G(int constvalue, void *dummy){
	const_G=constvalue;
}

void changeWB_R(int constvalue, void *dummy){
	const_R=constvalue;
}


void change_B1(int constvalue, void *dummy){
	const_B1=constvalue;
}

void change_B2(int constvalue, void *dummy){
	const_B2=constvalue;
}

void change_B3(int constvalue, void *dummy){
	const_B3=constvalue;
}
void change_G1(int constvalue, void *dummy){
	const_G1=constvalue;
}

void change_G2(int constvalue, void *dummy){
	const_G2=constvalue;
}

void change_G3(int constvalue, void *dummy){
	const_G3=constvalue;
}
void change_R1(int constvalue, void *dummy){
	const_R1=constvalue;
}

void change_R2(int constvalue, void *dummy){
	const_R2=constvalue;
}

void change_R3(int constvalue, void *dummy){
	const_R3=constvalue;
}

void change_Gain(int constvalue, void *dummy){
	const_gain=constvalue;
}

void change_Gamma(int constvalue, void *dummy){
	const_gamma=constvalue;
}

void changedisparityX(int disparity, void *dummy)
{
	disparity_x=disparity + HOTFIX::DISPARITY_FIX_X; // forcely adjust
}

void changedisparityY(int disparity, void *dummy)
{
	disparity_y=disparity + HOTFIX::DISPARITY_FIX_Y; // // forcely adjust
}

void changedisparityTh(int disparity, void *dummy)
{
	disp_threshold=disparity;
}

void changedisparityupTh(int disparityup, void *dummy)
{
	disp_upthreshold=disparityup;
}

void changetransparency(int disparityup, void *dummy)
{
	transparency=disparityup;
}

void resizeratio(int disparityup, void *dummy)
{
	sizeratio=disparityup;
}

int integration_diff1, integration_diff2;
bool integration_diff1_trigger=false;
bool integration_diff2_trigger=false;

void changeMasterExp(int integration, void *dummy)
{ 
	if (integration!=integration_diff1) {
		integration_diff1_trigger=true;
		integration1=integration;
	}
	else
	integration_diff1_trigger=false;

}


void changeMasterExp1(int integration, void * devP)
{
	//int integration_diff1;
	okCFrontPanel *dev = (okCFrontPanel *)devP;
	
	if (integration > integration2) {
	//programRegisterDual_ex(dev);
	programRegisterDual(dev, 0x000B, integration, 1);
	integration_diff1=integration-integration2-1;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration1_temp=integration;
	} else if (integration == integration2) {
	//programRegisterDual_ex(dev);
	programRegisterDual(dev, 0x000B, integration, 1);
	integration_diff1=integration-integration2;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration1_temp=integration;
	}else {
	cout<<"The master exposure is not allowed to be smaller than the slave exposure!"<<endl;
	//programRegisterDual_ex(dev);
	programRegisterDual(dev, 0x000B, integration2, 1);
	integration_diff1=0;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration1_temp=integration2;
	integration1=integration2;
	}

}

void changeSlaveExp(int integration, void * dummy)
{
	if (integration!=integration_diff2){
		integration_diff2_trigger=true;
		integration2=integration;
	}
	else
	integration_diff2_trigger=false;
}

void changeSlaveExp1(int integration, void * devP)
{
	//int integration_diff1;
	okCFrontPanel *dev = (okCFrontPanel *)devP;
	
	if (integration<integration1){
	programRegisterDual(dev, 0x000B, integration, 0);
	integration_diff1=integration1-integration-1;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration2_temp=integration;
	} else if (integration==integration1){
	programRegisterDual(dev, 0x000B, integration, 0);
	integration_diff1=integration1-integration;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration2_temp=integration;
	}else {
	cout<<"The slave exposure is not allowed to be larger than the master exposure!"<<endl;
	programRegisterDual(dev, 0x000B, integration1, 0);
	integration_diff1=0;
	dev->SetWireInValue(0x0F,integration_diff1);
	dev->UpdateWireIns();
	integration2_temp=integration1;
	integration2=integration1;
	}
}



void superimpose(int state, void *dummy)
{
	cout << "Should superimpose" << endl;
	if (state==1) SuperImposeEnable=TRUE;
	else SuperImposeEnable=FALSE;
}


void superimpose_record(int state, void *dummy)
{
	cout << "Should superimpose recording" << endl;
	if (state==1) SuperImposeRecordEnable=TRUE;
	else SuperImposeRecordEnable=FALSE;
}

void AutoDis(int state, void *dummy)
{
	cout << "Should enable auto-disparity" << endl;
	if (state==1) AutoDisEnable=TRUE;
	else AutoDisEnable=FALSE;
}


void bliinter(int state, void *dummy)
{
	cout << "Should bilinearly interpolate" << endl;
	if (state==1) BilinearEnable=TRUE;
	else BilinearEnable=FALSE;
}


void SortDataRecordTestHorizonCropColor1(void)
{
	//fw=fopen("../videoData/frameData.dat","w");
	int ImageData;
	//int ImageData_B, ImageData_G,ImageData_R;
	char fileName[50], fileName_Raw[50], fileName_Color[50], fileName_NIR[50],fileName_superimpose[50];
	long long DataPoint=0;
	long long startpoint=0;
	//int lineoffset = (gScreen->pitch / 4);
	//unsigned int *FrameBuffer = (unsigned int*)gScreen->pixels;
	int enable=0;

	
	float const_Bs=const_B;
	float const_Gs=const_G;
	float const_Rs=const_R;

	for (int i=0; i<SDRAM_SIZE; ++i){  
		if (i%2==0){
			g_buf1[i/2]=g_buf[i];
		} else {
			g_buf2[i/2]=g_buf[i];
		}
	}
	//createButton("Snap",saveFrame,0,CV_PUSH_BUTTON,0);

	for (int i=0; i<HEIGHT; ++i){  
		for (int j=0; j<WIDTH/2; ++j)
		{
			if (DataPoint<=(SDRAM_SIZE/2)){
				ImageData=g_buf1[DataPoint];
			} else {
				ImageData=0;
			};
			
			//FrameBuffer[i*lineoffset+j]=ImageData*(1+256+256*256);
			DataPoint=DataPoint+1;
			x[HEIGHT-1-i][j] = ImageData;
			xy[HEIGHT-1-i][j] = ImageData;
			//raw_align.at<uchar>(i,j)=ImageData;
			//raw_data1.at<uchar>(i,j)=ImageData;
			//if(i==230 && j==443)
			   //printf("%d \t", ImageData);
		}
		//fprintf(fw,"\n");
	}

	
		DataPoint=0;
	startpoint=0;
	enable=0;
	
	bool skipflag=false;
	if (integration1_temp>integration2_temp) {
		while (g_buf2[startpoint]!=1) {
		startpoint++;
		skipflag=true;
		}
	} else if (integration1_temp==integration2_temp) {
		startpoint+=846;
		while (g_buf2[startpoint]!=1) {
		skipflag=true;
		startpoint++;
		}
	}
	cout<<"start at startpoint: "<<dec<<startpoint<<endl;
	
	float original_b,original_r,original_g;
	float new_b,new_r,new_g;
	float b1=const_B1;
	float g1=const_G1;
	float r1=const_R1;
	float b2=const_B2;
	float g2=const_G2;
	float r2=const_R2;
	float b3=const_B3;
	float g3=const_G3;
	float r3=const_R3;
	//float constant=const_gain;
	float gain=const_gain;
	for (int i=0; i<HEIGHT-1; i+=2 ){  
		for (int j=0; j<WIDTH/2-1; j+=2)
		{	
			const long MAX = HOTFIX::BUF_MAX;
				original_b=g_buf2[(startpoint+847) % MAX];
				original_g=g_buf2[(startpoint+846) % MAX];
				original_r=g_buf2[startpoint % MAX]; // g_buf2 = 406080
				
				//Matrix exploration
				new_b=	original_b*(b1/100-3)+original_g*(b2/100-3)+original_r*(b3/100-3);
				new_g=	original_b*(g1/100-3)+original_g*(g2/100-3)+original_r*(g3/100-3);
				new_r=	original_b*(r1/100-3)+original_g*(r2/100-3)+original_r*(r3/100-3);
				
				//gamma correction
			/* 	new_b=gain/100*pow(new_b,constant/100);
				new_g=gain/100*pow(new_g,constant/100);
				new_r=gain/100*pow(new_r,constant/100); */
			/*	new_b=min(int(gain/100*new_b),255);
				new_g=min(int(gain/100*new_g),255);
				new_r=min(int(gain/100*new_r),255);
				*/
				new_b=(new_b<0.0)?1.0:new_b;
				new_g=(new_g<0.0)?1.0:new_g;
				new_r=(new_r<0.0)?1.0:new_r;

				new_b=(new_b>255.0)?255.0:new_b;
				new_g=(new_g>255.0)?255.0:new_g;
				new_r=(new_r>255.0)?255.0:new_r;
				


				ImageData_B=new_b;
				ImageData_G=new_g;
				ImageData_R=new_r;


				ImageData_B=min(int(gain/100*ImageData_B),255);
				ImageData_G=min(int(gain/100*ImageData_G),255);
				ImageData_R=min(int(gain/100*ImageData_R),255);
				
				//
				///* raw_data1.at<uchar>(i,j+WIDTH/2)=ImageData_G;
				//raw_data1.at<uchar>(i,j+1+WIDTH/2)=ImageData_B;
				//raw_data1.at<uchar>(i+1,j+WIDTH/2)=ImageData_R;
				//raw_data1.at<uchar>(i+1,j+1+WIDTH/2)=g_buf2[startpoint+847]; */
				x[i][j+WIDTH/2] = ImageData_G;
				x[i][j+1+WIDTH/2] = ImageData_B;
				x[i+1][j+WIDTH/2] = ImageData_R;
				x[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				xy[i][j+WIDTH/2] = ImageData_G;
				xy[i][j+1+WIDTH/2] = ImageData_B;
				xy[i+1][j+WIDTH/2] = ImageData_R;
				xy[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				///* raw_visible.at<Vec3b>(i,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j+1)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j+1)[2] = ImageData_R; */
				//startpoint
				if (j==(WIDTH/2-2)) {
				startpoint=startpoint+848;
				}else {
				startpoint=startpoint+2;
				}
			


		}
		//fprintf(fw,"\n");
	}
	enable=0;
/* 			if(video){

		if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
			firstVideoFrame=false;
		}
		if(videoFileCount%2000==0){
			outFile.close();
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.clear();
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
		}

		headerInfo[0]='\0';
		outFile.write(x[0], HEIGHT*WIDTH);
		outFile.flush();
		cout<<"saving video"<<endl;
		videoFileCount++;
	}
	 */
/* 	FILE *fp;
	string reserved; */
	/* if(video){

		if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.pvf", getBinFileNumber());
			fp = fopen(fileName,"w+b");
			writePvfHeader(&vHeader, &reserved,fp);
			firstVideoFrame=false;
		}
		if(videoFileCount%2000==0){
			
			writeNumberOfFramesToPvfHeader(&vHeader,fp);
			fclose(fp);
			sprintf(fileName, "./videoData/experiment/videoData%d.pvf", getBinFileNumber());
			fp = fopen(fileName,"w+b");
			writePvfHeader(&vHeader, &reserved,fp);
		}

		//headerInfo[0]='\0';
		fwrite(x[0],sizeof(unsigned char),(812160),fp);
		cout<<"saving video"<<endl;
		videoFileCount++;
	} */
	
/* 	if(video){

		if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
			firstVideoFrame=false;
		}
		if(videoFileCount%2000==0){
			outFile.close();
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.clear();
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
		}

		headerInfo[0]='\0';
		outFile.write(x[0], HEIGHT*WIDTH);
		outFile.flush();
		cout<<"saving video"<<endl;
		videoFileCount++;
	} */


}


void SortDataRecordTestHorizonCropColor1Gbuf1(void)
{
	//fw=fopen("../videoData/frameData.dat","w");
	int ImageData;
	//int ImageData_B, ImageData_G,ImageData_R;
	char fileName[50], fileName_Raw[50], fileName_Color[50], fileName_NIR[50],fileName_superimpose[50];
	long long DataPoint=0;
	long long startpoint=0;
	//int lineoffset = (gScreen->pitch / 4);
	//unsigned int *FrameBuffer = (unsigned int*)gScreen->pixels;
	int enable=0;

	
	float const_Bs=const_B;
	float const_Gs=const_G;
	float const_Rs=const_R;

	for (int i=0; i<SDRAM_SIZE; ++i){  
		if (i%2==0){
			g_buf1[i/2]=g_buf_1[i];
		} else {
			g_buf2[i/2]=g_buf_1[i];
		}
	}
	//createButton("Snap",saveFrame,0,CV_PUSH_BUTTON,0);

	for (int i=0; i<HEIGHT; ++i){  
		for (int j=0; j<WIDTH/2; ++j)
		{
			if (DataPoint<=(SDRAM_SIZE/2)){
				ImageData=g_buf1[DataPoint];
			} else {
				ImageData=0;
			};
			
			//FrameBuffer[i*lineoffset+j]=ImageData*(1+256+256*256);
			DataPoint=DataPoint+1;
			x[HEIGHT-1-i][j] = ImageData;
			xy[HEIGHT-1-i][j] = ImageData;
			//raw_align.at<uchar>(i,j)=ImageData;
			//raw_data1.at<uchar>(i,j)=ImageData;
			//if(i==230 && j==443)
			   //printf("%d \t", ImageData);
		}
		//fprintf(fw,"\n");
	}
	
		DataPoint=0;
	startpoint=0;
	enable=0;

	
	bool skipflag=false;
	if (integration1_temp>integration2_temp) {
		while (g_buf2[startpoint]!=1) {
		startpoint++;
		skipflag=true;
		}
	} else if (integration1_temp==integration2_temp) {
		startpoint+=846;
		while (g_buf2[startpoint]!=1) {
		skipflag=true;
		startpoint++;
		}
	}
	cout<<"start at startpoint: "<<dec<<startpoint<<endl;
	
	float original_b,original_r,original_g;
	float new_b,new_r,new_g;
	float b1=const_B1;
	float g1=const_G1;
	float r1=const_R1;
	float b2=const_B2;
	float g2=const_G2;
	float r2=const_R2;
	float b3=const_B3;
	float g3=const_G3;
	float r3=const_R3;
	//float constant=const_gain;
	float gain=const_gain;
	for (int i=0; i<HEIGHT-1; i+=2 ){  
		for (int j=0; j<WIDTH/2-1; j+=2)
		{
			
				original_b=g_buf2[startpoint+847];
				original_g=g_buf2[startpoint+846];
				original_r=g_buf2[startpoint];
				
				//Matrix exploration
				new_b=	original_b*(b1/100-3)+original_g*(b2/100-3)+original_r*(b3/100-3);
				new_g=	original_b*(g1/100-3)+original_g*(g2/100-3)+original_r*(g3/100-3);
				new_r=	original_b*(r1/100-3)+original_g*(r2/100-3)+original_r*(r3/100-3);
				
				//gamma correction
			/* 	new_b=gain/100*pow(new_b,constant/100);
				new_g=gain/100*pow(new_g,constant/100);
				new_r=gain/100*pow(new_r,constant/100); */
			/*	new_b=min(int(gain/100*new_b),255);
				new_g=min(int(gain/100*new_g),255);
				new_r=min(int(gain/100*new_r),255);
				*/
				new_b=(new_b<0.0)?1.0:new_b;
				new_g=(new_g<0.0)?1.0:new_g;
				new_r=(new_r<0.0)?1.0:new_r;

				new_b=(new_b>255.0)?255.0:new_b;
				new_g=(new_g>255.0)?255.0:new_g;
				new_r=(new_r>255.0)?255.0:new_r;
				


				ImageData_B=new_b;
				ImageData_G=new_g;
				ImageData_R=new_r;


				ImageData_B=min(int(gain/100*ImageData_B),255);
				ImageData_G=min(int(gain/100*ImageData_G),255);
				ImageData_R=min(int(gain/100*ImageData_R),255);
				
				//
				///* raw_data1.at<uchar>(i,j+WIDTH/2)=ImageData_G;
				//raw_data1.at<uchar>(i,j+1+WIDTH/2)=ImageData_B;
				//raw_data1.at<uchar>(i+1,j+WIDTH/2)=ImageData_R;
				//raw_data1.at<uchar>(i+1,j+1+WIDTH/2)=g_buf2[startpoint+847]; */
				x[i][j+WIDTH/2] = ImageData_G;
				x[i][j+1+WIDTH/2] = ImageData_B;
				x[i+1][j+WIDTH/2] = ImageData_R;
				x[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				xy[i][j+WIDTH/2] = ImageData_G;
				xy[i][j+1+WIDTH/2] = ImageData_B;
				xy[i+1][j+WIDTH/2] = ImageData_R;
				xy[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				///* raw_visible.at<Vec3b>(i,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j+1)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j+1)[2] = ImageData_R; */
				//startpoint
				if (j==(WIDTH/2-2)) {
				startpoint=startpoint+848;
				}else {
				startpoint=startpoint+2;
				}
			


		}
		//fprintf(fw,"\n");
	}
	enable=0;
			if(video){

		if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
			firstVideoFrame=false;
		}
		if(videoFileCount%2000==0){
			outFile.close();
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.clear();
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
		}

		headerInfo[0]='\0';
		outFile.write(x[0], HEIGHT*WIDTH);
		outFile.flush();
		cout<<"saving video"<<endl;
		videoFileCount++;
	}
/* 	if (SuperImposeEnable) {
		//raw_overlay.data=raw_visible.data;
		for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j)
				{
				raw_overlay.at<Vec3b>(i,j) [0]=raw_visible.at<Vec3b>(i,j) [0];
				raw_overlay.at<Vec3b>(i,j) [1]=raw_visible.at<Vec3b>(i,j) [1];
				raw_overlay.at<Vec3b>(i,j) [2]=raw_visible.at<Vec3b>(i,j) [2];
				if (j+disparity_x<WIDTH/2 && i-disparity_y>0  && raw_align.at<uchar>(i,j)>disp_threshold) {
					raw_overlay.at<Vec3b>(i-disparity_y,j+disparity_x)[0] = 64;
					raw_overlay.at<Vec3b>(i-disparity_y,j+disparity_x)[1] = 255;
					raw_overlay.at<Vec3b>(i-disparity_y,j+disparity_x)[2] = 0;
				} 
					
			}
		} 
	} */


	

}


void SortDataRecordTestHorizonCropColor1Gbuf2(void)
{
	//fw=fopen("../videoData/frameData.dat","w");
	int ImageData;
	//int ImageData_B, ImageData_G,ImageData_R;
	char fileName[50], fileName_Raw[50], fileName_Color[50], fileName_NIR[50],fileName_superimpose[50];
	long long DataPoint=0;
	long long startpoint=0;
	//int lineoffset = (gScreen->pitch / 4);
	//unsigned int *FrameBuffer = (unsigned int*)gScreen->pixels;
	int enable=0;

	
	float const_Bs=const_B;
	float const_Gs=const_G;
	float const_Rs=const_R;

	for (int i=0; i<SDRAM_SIZE; ++i){  
		if (i%2==0){
			g_buf1[i/2]=g_buf_2[i];
		} else {
			g_buf2[i/2]=g_buf_2[i];
		}
	}
	//createButton("Snap",saveFrame,0,CV_PUSH_BUTTON,0);

	for (int i=0; i<HEIGHT; ++i){  
		for (int j=0; j<WIDTH/2; ++j)
		{
			if (DataPoint<=(SDRAM_SIZE/2)){
				ImageData=g_buf1[DataPoint];
			} else {
				ImageData=0;
			};
			
			//FrameBuffer[i*lineoffset+j]=ImageData*(1+256+256*256);
			DataPoint=DataPoint+1;
			x[HEIGHT-1-i][j] = ImageData;
			xy[HEIGHT-1-i][j] = ImageData;
			//raw_align.at<uchar>(i,j)=ImageData;
			//raw_data1.at<uchar>(i,j)=ImageData;
			//if(i==230 && j==443)
			   //printf("%d \t", ImageData);
		}
		//fprintf(fw,"\n");
	}

		DataPoint=0;
	startpoint=0;
	enable=0;

	
	bool skipflag=false;
	if (integration1_temp>integration2_temp) {
		while (g_buf2[startpoint]!=1) {
		startpoint++;
		skipflag=true;
		}
	} else if (integration1_temp==integration2_temp) {
		startpoint+=846;
		while (g_buf2[startpoint]!=1) {
		skipflag=true;
		startpoint++;
		}
	}
	cout<<"start at startpoint: "<<dec<<startpoint<<endl;
	
	float original_b,original_r,original_g;
	float new_b,new_r,new_g;
	float b1=const_B1;
	float g1=const_G1;
	float r1=const_R1;
	float b2=const_B2;
	float g2=const_G2;
	float r2=const_R2;
	float b3=const_B3;
	float g3=const_G3;
	float r3=const_R3;
	//float constant=const_gain;
	float gain=const_gain;
	for (int i=0; i<HEIGHT-1; i+=2 ){  
		for (int j=0; j<WIDTH/2-1; j+=2)
		{
			
				original_b=g_buf2[startpoint+847];
				original_g=g_buf2[startpoint+846];
				original_r=g_buf2[startpoint];
				
				//Matrix exploration
				new_b=	original_b*(b1/100-3)+original_g*(b2/100-3)+original_r*(b3/100-3);
				new_g=	original_b*(g1/100-3)+original_g*(g2/100-3)+original_r*(g3/100-3);
				new_r=	original_b*(r1/100-3)+original_g*(r2/100-3)+original_r*(r3/100-3);
				
				//gamma correction
			/* 	new_b=gain/100*pow(new_b,constant/100);
				new_g=gain/100*pow(new_g,constant/100);
				new_r=gain/100*pow(new_r,constant/100); */
			/*	new_b=min(int(gain/100*new_b),255);
				new_g=min(int(gain/100*new_g),255);
				new_r=min(int(gain/100*new_r),255);
				*/
				new_b=(new_b<0.0)?1.0:new_b;
				new_g=(new_g<0.0)?1.0:new_g;
				new_r=(new_r<0.0)?1.0:new_r;

				new_b=(new_b>255.0)?255.0:new_b;
				new_g=(new_g>255.0)?255.0:new_g;
				new_r=(new_r>255.0)?255.0:new_r;
				


				ImageData_B=new_b;
				ImageData_G=new_g;
				ImageData_R=new_r;


				ImageData_B=min(int(gain/100*ImageData_B),255);
				ImageData_G=min(int(gain/100*ImageData_G),255);
				ImageData_R=min(int(gain/100*ImageData_R),255);
				
				//
				///* raw_data1.at<uchar>(i,j+WIDTH/2)=ImageData_G;
				//raw_data1.at<uchar>(i,j+1+WIDTH/2)=ImageData_B;
				//raw_data1.at<uchar>(i+1,j+WIDTH/2)=ImageData_R;
				//raw_data1.at<uchar>(i+1,j+1+WIDTH/2)=g_buf2[startpoint+847]; */
				x[i][j+WIDTH/2] = ImageData_G;
				x[i][j+1+WIDTH/2] = ImageData_B;
				x[i+1][j+WIDTH/2] = ImageData_R;
				x[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				xy[i][j+WIDTH/2] = ImageData_G;
				xy[i][j+1+WIDTH/2] = ImageData_B;
				xy[i+1][j+WIDTH/2] = ImageData_R;
				xy[i+1][j+1+WIDTH/2] = ImageData_G;//g_buf2[startpoint+847];
				///* raw_visible.at<Vec3b>(i,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i,j+1)[2] = ImageData_R;
				//raw_visible.at<Vec3b>(i+1,j+1)[0] = ImageData_B;
				//raw_visible.at<Vec3b>(i+1,j+1)[1] = ImageData_G;
				//raw_visible.at<Vec3b>(i+1,j+1)[2] = ImageData_R; */
				//startpoint
				if (j==(WIDTH/2-2)) {
				startpoint=startpoint+848;
				}else {
				startpoint=startpoint+2;
				}
			


		}
		//fprintf(fw,"\n");
	}
	enable=0;
		if(video){

		if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
			firstVideoFrame=false;
		}
		if(videoFileCount%2000==0){
			outFile.close();
			sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
			outFile.clear();
			outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
		}

		headerInfo[0]='\0';
		outFile.write(x[0], HEIGHT*WIDTH);
		outFile.flush();
		cout<<"saving video"<<endl;
		videoFileCount++;
	}

}

Mat rotateImage(const Mat& source, double angle)
{
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}

void SortDataRecordTestHorizonCropColor1Subthread(void)
{
	//fw=fopen("../videoData/frameData.dat","w");
	int ImageData;
	//int ImageData_B, ImageData_G,ImageData_R;
	char fileName[50], fileName_Raw[50], fileName_Color[50], fileName_NIR[50],fileName_superimpose[50];
	long long DataPoint=0;
	long long startpoint=0;
	//int lineoffset = (gScreen->pitch / 4);
	//unsigned int *FrameBuffer = (unsigned int*)gScreen->pixels;
	int enable=0;

	
	float const_Bs=const_B;
	float const_Gs=const_G;
	float const_Rs=const_R;


	for (int i=0; i<HEIGHT; ++i){  
		for (int j=0; j<WIDTH/2; ++j)
		{
		
			raw_align.at<uchar>(i,j)=xy[i][j];
			raw_data1.at<uchar>(i,j)=xy[i][j];
		}
	}
	if (BilinearEnable==true) {
		for (int i=1; i<HEIGHT-2; i+=2){  
			for (int j=1; j<WIDTH/2-2; j+=2)
			{
					raw_data1.at<uchar>(i,j+WIDTH/2)=xy[i][j+WIDTH/2];
					raw_data1.at<uchar>(i,j+1+WIDTH/2)=xy[i][j+1+WIDTH/2];
					raw_data1.at<uchar>(i+1,j+WIDTH/2)=xy[i+1][j+WIDTH/2];
					raw_data1.at<uchar>(i+1,j+1+WIDTH/2)=xy[i][j+WIDTH/2]; 
					raw_visible.at<Vec3b>(i,j)[0] = xy[i-1][j+WIDTH/2]/2+xy[i+1][j+WIDTH/2]/2;  //B
					raw_visible.at<Vec3b>(i,j)[1] = xy[i][j+WIDTH/2]; //G
					raw_visible.at<Vec3b>(i,j)[2] = xy[i][j-1+WIDTH/2]/2+xy[i][j+1+WIDTH/2]/2;//R
					raw_visible.at<Vec3b>(i+1,j)[0] = xy[i+1][j+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j)[1] = xy[i][j+WIDTH/2]/4+xy[i+1][j-1+WIDTH/2]/4+xy[i+1][j+1+WIDTH/2]/4+xy[i+2][j+WIDTH/2]/4 ; 
					raw_visible.at<Vec3b>(i+1,j)[2] = xy[i][j-1+WIDTH/2]/4+ xy[i][j+1+WIDTH/2]/4+ xy[i+2][j-1+WIDTH/2]/4+ xy[i+2][j+1+WIDTH/2]/4;
					raw_visible.at<Vec3b>(i,j+1)[0] = xy[i-1][j+WIDTH/2]/4+xy[i-1][j+2+WIDTH/2]/4+xy[i+1][j+WIDTH/2]/4+xy[i+1][j+2+WIDTH/2]/4;
					raw_visible.at<Vec3b>(i,j+1)[1] = xy[i][j+WIDTH/2]/4+xy[i-1][j+1+WIDTH/2]/4+xy[i+1][j+1+WIDTH/2]/4+xy[i][j+2+WIDTH/2]/4;
					raw_visible.at<Vec3b>(i,j+1)[2] = xy[i][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j+1)[0] = xy[i+1][j+WIDTH/2]/2+xy[i+1][j+2+WIDTH/2]/2;
					raw_visible.at<Vec3b>(i+1,j+1)[1] = xy[i+1][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j+1)[2] = xy[i][j+1+WIDTH/2]/2+xy[i+2][j+1+WIDTH/2]/2;
					
			}
		}
		//fprintf(fw,"\n");
	} else {
		for (int i=0; i<HEIGHT-1; i+=2){  
			for (int j=0; j<WIDTH/2-1; j+=2)
			{
				
					raw_data1.at<uchar>(i,j+WIDTH/2)=xy[i][j+WIDTH/2];
					raw_data1.at<uchar>(i,j+1+WIDTH/2)=xy[i][j+1+WIDTH/2];
					raw_data1.at<uchar>(i+1,j+WIDTH/2)=xy[i+1][j+WIDTH/2];
					raw_data1.at<uchar>(i+1,j+1+WIDTH/2)=xy[i][j+WIDTH/2]; 
					raw_visible.at<Vec3b>(i,j)[0] = xy[i][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i,j)[1] = xy[i][j+WIDTH/2];
					raw_visible.at<Vec3b>(i,j)[2] = xy[i+1][j+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j)[0] = xy[i][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j)[1] = xy[i][j+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j)[2] = xy[i+1][j+WIDTH/2];
					raw_visible.at<Vec3b>(i,j+1)[0] = xy[i][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i,j+1)[1] = xy[i][j+WIDTH/2];
					raw_visible.at<Vec3b>(i,j+1)[2] = xy[i+1][j+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j+1)[0] = xy[i][j+1+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j+1)[1] = xy[i][j+WIDTH/2];
					raw_visible.at<Vec3b>(i+1,j+1)[2] = xy[i+1][j+WIDTH/2];


			}
		}
	}
	enable=0;
	//superimpose disparity found/////////////
	//int xaxis=0;
	//int yaxis=0;
	int disparityX;
	int disparityY;
	Mat raw_rotate;
	int validnum;
	raw_rotate.create(rows,columns,CV_8UC1);
	if (AutoDisEnable) {
		int num=0;
		corXY cor_temp;
		int vecnum=0;
		int average_tempX=0;
		int average_tempY=0;
		int firstflag=0;
		int overlap_flag=0;
		vector <corXY> cord;
		//vector <corXY> cord1;
		cord.clear();
		
		for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j){
				raw_rotate.at<uchar>(i,j)=xy[i][j];
			}
		}
		raw_rotate=rotateImage(raw_rotate,1.5);
		for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j)
			{
				if (num){
					average_tempX=cor_temp.xaxis/num;
					average_tempY=cor_temp.yaxis/num;
				}
				if ((vecnum==0) && (firstflag==0)) {               //no light dot yet
					average_tempX=j;
					average_tempY=i;
				}
				if (raw_rotate.at<uchar>(i,j)==255) {
					firstflag=1;
					if ((abs(average_tempX-j)<30) && (abs(average_tempY-i)<30)){
						cor_temp.xaxis+=j;
						cor_temp.yaxis+=i;
						num++;
					} else {
						overlap_flag=0;
						cout<<dec<<average_tempX<<"\t"<<average_tempY<<endl;
						if(vecnum){ //check the overlap point
							for (int k=0;k<vecnum;k++){
								if ((abs(average_tempX-cord[k].xaxis)<20) && (abs(average_tempY-cord[k].yaxis)<20))
								{
									overlap_flag=1;	
									//break;
								}
							}
						}
						cout<<overlap_flag<<endl;
						if (overlap_flag==0) {
							cor_temp.xaxis=average_tempX;
							cor_temp.yaxis=average_tempY;
							cord.push_back(cor_temp);
							average_tempX=0;
							average_tempY=0;
							cor_temp.xaxis=0;
							cor_temp.yaxis=0;
							num=0;
							vecnum++;
							cor_temp.xaxis+=j;
							cor_temp.yaxis+=i;
							num++;
						}	
					}
				}
				if ((i==HEIGHT-1) && (j==WIDTH/2-1) && (firstflag==1)){  //has vecnum point
					vecnum++;
					cor_temp.xaxis=average_tempX;
					cor_temp.yaxis=average_tempY;
					cord.push_back(cor_temp);
					cout<<"the corrdinates are"<<cor_temp.xaxis<<"\t"<<cor_temp.yaxis<<endl;
				}
			}
		}
		//vecnum=3;
		int xmin_scale,xmax_scale,ymin_scale,ymax_scale;
		corXY corS_temp;
		int num2=0;
		disparityX=0;
		disparityY=0;
		cout<<"vecnum is "<<vecnum<<endl;
		int flag=0;
		for (int k=0; k<vecnum;k++){
			cout<<"The NIR point coordinates are"<<cord[k].xaxis<<"\t"<< cord[k].yaxis<<endl;
			xmin_scale=max(WIDTH/2, cord[k].xaxis+WIDTH/2-50);
			xmax_scale=min(WIDTH-1, cord[k].xaxis+WIDTH/2+100);
			ymin_scale=max(0, cord[k].yaxis-30);
			ymax_scale=min(HEIGHT-2, cord[k].yaxis+30);
			cout<<"the scales are	"<<xmin_scale<<"\t"<<xmax_scale<<"\t"<<ymin_scale<<"\t"<<ymax_scale<<endl;
			flag=0;
			cord[k].Sxaxis=0;
			cord[k].Syaxis=0;
			corS_temp.xaxis=0;
			corS_temp.yaxis=0;
			num2=0;
			for (int i=ymin_scale; i<ymax_scale; ++i){  
				for (int j=xmin_scale; j<xmax_scale; ++j)
				{
					if (xy[i][j]==255) {
							corS_temp.xaxis+=j-846;
							corS_temp.yaxis+=i;
							num2++;
							flag=1;
					}
				}
			}
			if (flag==1) {
				//if (num2==0) num2=1;
				cout<<"num2 is :"<<num2<<endl;
				corS_temp.xaxis/=num2;
				corS_temp.yaxis/=num2;
				cout<<"The corresponding pixels in color channels"<<dec<<corS_temp.xaxis<<"\t"<<corS_temp.yaxis<<endl;
				cord[k].Sxaxis=corS_temp.xaxis;
				cord[k].Syaxis=corS_temp.yaxis;
			}
		}
		
		validnum=0;
		for (int k=0; k<vecnum;k++){
			if ((cord[k].Sxaxis!=0) && (cord[k].Syaxis!=0)){
				disparityX+=(cord[k].xaxis-cord[k].Sxaxis);
				disparityY+=(cord[k].yaxis-cord[k].Syaxis);
				cout<<k+1<<"\t"<<disparityX<<"\t"<<disparityY<<endl;
				validnum++;
				//break;
			}
		}
		if (validnum) {
			disparityX/=validnum;
			disparityY/=validnum;
		}
		disparityX=-disparityX;     
		disparityY=-disparityY;
		cout<<dec<<validnum<<"\t"<<disparityX<<"\t"<<dec<<disparityY<<"\t"<<dec<<num<<endl;
	} else {
		disparityX=disparity_x;
		disparityY=disparity_y;
		raw_rotate=raw_align;
		validnum=1;
	}
	//////////////////////////////////////////
	//user 2.4.5 function
	//applyColorMap(raw_align,raw_align_jet,2); 
	applyColorMap(raw_align, raw_align_jet, COLORMAP_JET);
	if (SuperImposeEnable) {
		for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j)
				{
				 raw_overlay_HSV.at<Vec3b>(i,j) [0]=raw_visible.at<Vec3b>(i,j) [0];
				raw_overlay_HSV.at<Vec3b>(i,j) [1]=raw_visible.at<Vec3b>(i,j) [1];
				raw_overlay_HSV.at<Vec3b>(i,j) [2]=raw_visible.at<Vec3b>(i,j) [2]; 
				raw_overlay.at<Vec3b>(i,j) [0]=raw_overlay_HSV.at<Vec3b>(i,j) [0];
				raw_overlay.at<Vec3b>(i,j) [1]=raw_overlay_HSV.at<Vec3b>(i,j) [1];
				raw_overlay.at<Vec3b>(i,j) [2]=raw_overlay_HSV.at<Vec3b>(i,j) [2];
				raw_overlay_RGB.at<Vec3b>(i,j) [0]= 0;
				raw_overlay_RGB.at<Vec3b>(i,j) [1]=0;
				raw_overlay_RGB.at<Vec3b>(i,j) [2]= 0;
					
			}
		} 
				//for the right pair board
		 for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j){
				if (j+disparityX<WIDTH/2 && i+disparityY<480 && j+disparityX>0 && i+disparityY>0 && raw_rotate.at<uchar>(i,j)>disp_threshold  && disp_upthreshold>disp_threshold && validnum!=0) {
						if (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)/8){
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= min((128+raw_align.at<uchar>(i,j)*127/(disp_upthreshold-disp_threshold)/8),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=0;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=0;
						} else if ((raw_align.at<uchar>(i,j)>(disp_upthreshold-disp_threshold)/8) && (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)*3/8)) {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= 255;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]= min(((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)/8)*255/(disp_upthreshold-disp_threshold)*4),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= 0;
						}else if ((raw_align.at<uchar>(i,j)>(disp_upthreshold-disp_threshold)*3/8) && (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)*5/8)) {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= min(abs(255-((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)*3/8)*255/(disp_upthreshold-disp_threshold)*4)),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]= 255;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= min(((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)*3/8)*255/(disp_upthreshold-disp_threshold)*4),255);
						} else if ((raw_align.at<uchar>(i,j)>(disp_upthreshold-disp_threshold)*5/8) && (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)*7/8)) {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= 0;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]= min(abs(255-((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)*5/8)*255/(disp_upthreshold-disp_threshold)*4)),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= 255;
						} else {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= 0;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=0;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= min((128+(255-raw_align.at<uchar>(i,j))*127/(disp_upthreshold-disp_threshold)*8),255);
						}

						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [0]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [0]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]*transparency/100,255);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [1]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]*transparency/100,255);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [2]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]*transparency/100,255);						
					} 
					
			}
		 }     
		 //for the right pair board
		/*  for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j){
					
				if (j+disparityX<WIDTH/2 && i+disparityY<480 && j+disparityX>0 && i+disparityY>0 && raw_rotate.at<uchar>(i,j)>disp_threshold  && disp_upthreshold>disp_threshold && validnum!=0) {
						if (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)/3+disp_threshold){
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= min((128+raw_align.at<uchar>(i,j)*127/(disp_upthreshold-disp_threshold)/3),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=0;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=0;
						} else if ((raw_align.at<uchar>(i,j)>(disp_upthreshold-disp_threshold)/3+disp_threshold) && (raw_align.at<uchar>(i,j)<=(disp_upthreshold-disp_threshold)*2/3+disp_threshold)) {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= 255;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]= min(((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)/3)*255/(disp_upthreshold-disp_threshold)/3),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= 0;
						}else if ((raw_align.at<uchar>(i,j)>(disp_upthreshold-disp_threshold)*2/3+disp_threshold)) {
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]= min(255-((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)*2/3)*255/(disp_upthreshold-disp_threshold)/3),255);
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]= 255;
							raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=0;
							//raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]= min(((raw_align.at<uchar>(i,j)-(disp_upthreshold-disp_threshold)*3/4)*255/(disp_upthreshold-disp_threshold)/4),255);
							
						}

						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [0]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [0]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [0]*transparency/100,255);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [1]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [1]*transparency/100,255);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=min(raw_overlay_HSV.at<Vec3b>(i+disparity_y,j+disparity_x) [2]*(100-transparency)/100+raw_overlay_RGB.at<Vec3b>(i+disparity_y,j+disparity_x) [2]*transparency/100,255); 
					} 
					
			}
		 }   */
//overlay/////////////////////////
		 /*for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j){
				raw_overlay.at<Vec3b>(i,j) [0]=raw_overlay_HSV.at<Vec3b>(i,j) [0]/2+raw_overlay_RGB.at<Vec3b>(i,j) [0]/2;
				raw_overlay.at<Vec3b>(i,j) [1]=raw_overlay_HSV.at<Vec3b>(i,j) [1]/2+raw_overlay_RGB.at<Vec3b>(i,j) [1]/2;
				raw_overlay.at<Vec3b>(i,j) [2]=raw_overlay_HSV.at<Vec3b>(i,j) [2]/2+raw_overlay_RGB.at<Vec3b>(i,j) [2]/2; 
			}
		 }*/
		/*  int range=0;
		 for (int i=0; i<HEIGHT; ++i){  
			for (int j=0; j<WIDTH/2; ++j){
				if (j+disparityX<WIDTH/2 && i+disparityY<480 && j+disparityX>0 && i+disparityY>0 && raw_rotate.at<uchar>(i,j)>disp_threshold  && disp_upthreshold>disp_threshold && validnum!=0) {
						range=raw_align.at<uchar>(i,j);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [0]=raw_visible.at<Vec3b>(i,j) [0];
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [1]=min(raw_visible.at<Vec3b>(i,j) [1]+range*raw_align.at<uchar>(i,j)/255+disp_threshold,255);
						raw_overlay.at<Vec3b>(i+disparity_y,j+disparity_x) [2]=raw_visible.at<Vec3b>(i,j) [2]; 
						
					} 
					
			}
		 } */
	}

	//png option
	vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

	if (save) {
		sprintf(fileName_Raw, "./videoData/snapshot/Raw%d.png", getBinFileNumber());
		sprintf(fileName_Color, "./videoData/snapshot/Color%d.png", getBinFileNumber());
		sprintf(fileName_NIR, "./videoData/snapshot/NIR%d.png", getBinFileNumber());
		sprintf(fileName_superimpose, "./videoData/snapshot/superimpose%d.png", getBinFileNumber());
		try {
			 imwrite(fileName_Color, raw_visible, compression_params);
			 imwrite(fileName_NIR, raw_align, compression_params);
			 imwrite(fileName_Raw, raw_data1, compression_params);
			 imwrite(fileName_Raw, raw_overlay, compression_params);
		}
		catch (runtime_error& ex) {
			fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
		}
		/*sprintf(fileName, "../videoData/snapshot/videoData%d.bin", getBinFileNumber());
		outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app); 
		headerInfo[0]='\0';
		outFile.write(x[0], HEIGHT*WIDTH);
		outFile.flush();
		outFile.close();*/
		cout<<"saving image"<<endl;
		save=FALSE;
	}
	

}

DWORD WINAPI dataThread(LPVOID lpParameter) {
		


		
		//clock_t init, final;
	while(1){
	//init=clock();
		if (g_bufbusy){
			SortDataRecordTestHorizonCropColor1();
			//cout<<"In display 0"<<endl;
			g_bufbusy=false;
			g_bufdone=true;
		} else if (g_buf1busy) {
			SortDataRecordTestHorizonCropColor1Gbuf1();
			//cout<<"In display 1"<<endl;
			g_buf1busy=false;
			g_buf1done=true;
		} else  if (g_buf2busy) {
			SortDataRecordTestHorizonCropColor1Gbuf2();
			//cout<<"In display 2"<<endl;
			g_buf2busy=false;
			g_buf2done=true;
		}

		
		//if(video){

		//	for (int i=0; i<HEIGHT-1; i++){  
		//		for (int j=0; j<WIDTH-1; j++)
		//		{
		//				raw_save.at<Vec3b>(i,j)[0]=xy[i][j];
		//				raw_save.at<Vec3b>(i,j)[1]=xy[i][j];
		//				raw_save.at<Vec3b>(i,j)[2]=xy[i][j];
		//		}
		//	}
		//	cout<<"saving video"<<endl;
		//	writer1.write(raw_save);
	
		//}

/* 		final=clock()-init;
		cout <<  ((double)CLOCKS_PER_SEC)/(double)final <<endl; */
		if (pressedKey == (int)'2') {return 0;}
	}
}

DWORD WINAPI displayThread(LPVOID lpParameter) {

	int *clk,*clk2;
	clk=&integration1;
	clk2=&integration2;
	int *constB_int, *constG_int, *constR_int;
	constB_int=&const_B;
	constG_int=&const_G;
	constR_int=&const_R;
	int *dis_x, *dis_y;
	dis_x=&disparity_x;
	dis_y=&disparity_y;
	int *dis_th;
	int *dis_upth;
	dis_th=&disp_threshold;
	dis_upth=&disp_upthreshold;
	int *transp;
	transp=&transparency;
	int *ratiocon;
	ratiocon=&sizeratio;
	//////////////matrix/////////////
	int *constB1_int, *constG1_int, *constR1_int;
	int *constB2_int, *constG2_int, *constR2_int;
	int *constB3_int, *constG3_int, *constR3_int;
	int *constGain_int,*constGamma_int;
	constB1_int=&const_B1;
	constG1_int=&const_G1;
	constR1_int=&const_R1;
	constB2_int=&const_B2;
	constG2_int=&const_G2;
	constR2_int=&const_R2;
	constB3_int=&const_B3;
	constG3_int=&const_G3;
	constR3_int=&const_R3;
	constGain_int=&const_gain;
	constGamma_int=&const_gamma;
	//namedWindow("CTL");
	
	namedWindow("NIR",CV_WINDOW_NORMAL|CV_WINDOW_FREERATIO|CV_GUI_EXPANDED);
	//namedWindow("Color",CV_WINDOW_NORMAL| CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
	//resizeWindow("NIR",1600,1200);
	createTrackbar("exp1","",clk, 8192, changeMasterExp, dev);
	createTrackbar("exp2","",clk2, 1000, changeSlaveExp, dev);
	createButton("Snap",saveFrame,0,CV_PUSH_BUTTON,0);
	createButton("Capture Video",saveVideo,0,CV_PUSH_BUTTON,0);
	createTrackbar("B1 correction","",constB1_int, 1000, change_B1, dev);
	createTrackbar("B2 correction","",constB2_int, 600, change_B2, dev);
	createTrackbar("B3 correction","",constB3_int, 600, change_B3, dev);
	createTrackbar("G1 correction","",constG1_int, 600, change_G1, dev);
	createTrackbar("G2 correction","",constG2_int, 600, change_G2, dev);
	createTrackbar("G3 correction","",constG3_int, 600, change_G3, dev);
	createTrackbar("R1 correction","",constR1_int, 600, change_R1, dev);
	createTrackbar("R2 correction","",constR2_int, 600, change_R2, dev);
	createTrackbar("R3 correction","",constR3_int, 600, change_R3, dev);
	createTrackbar("Gain","",constGain_int, 200, change_Gain, dev);
	createButton("Bilinear",bliinter,"Bilinear",CV_CHECKBOX,0);
	createButton("SuperImpose",superimpose,"SuperImpose",CV_CHECKBOX,0);
	createButton("SI_Record",superimpose_record,"SI_record",CV_CHECKBOX,0);
	createButton("Auto-Disparity",AutoDis,"SuperImpose",CV_CHECKBOX,0);
	createTrackbar("disparityX","",dis_x, 200, changedisparityX, dev);
	createTrackbar("disparityY","",dis_y, 200, changedisparityY, dev);
	createTrackbar("threshold","",dis_th, 255, changedisparityTh, dev);
	createTrackbar("dispupthreshold","",dis_upth, 255, changedisparityupTh, dev);
	createTrackbar("transparency","",transp, 100, changetransparency, dev);
	createTrackbar("RessizeRatio","",ratiocon, 400, resizeratio, dev);
	int dispCtr = 0;

		//SortDataRecordTestHorizonCropColor1();
		////////setup the video for opencv////////
		//the avi file will directly generate/////
		VideoWriter writer;

		//filename string
		char fileName[50];
		sprintf(fileName, "./videoData/experiment/videoData%d.avi", getBinFileNumber());
		//string filenamestring="./videoData/experiment/videoData.avi";
		string filenamestring=fileName;
		//fourcc integer
		int fcc=CV_FOURCC('D','I','V','3');
		//frames per sec integer
		int fps=HOTFIX::FRAME_RATE;
		//frame size
		Size frameSize(WIDTH/2,HEIGHT);
		writer=VideoWriter(filenamestring,fcc, fps, frameSize);
		if (!writer.isOpened()){
		cout<<"ERROR OPENING FILE FOR WRITE"<<endl;
		getchar();
		//return -1;
		}
	
	clock_t init, final;
	int a,b;

	while(1) {
		//imshow("CTL",logo);
		init=clock();
		SortDataRecordTestHorizonCropColor1Subthread();
		//waitKey(40);
		cout<<"In display"<<endl;
		pressedKey = waitKey(1);
		
		

		//cout<<" a "<<int(846*(float(sizeratio)/100.0f))<<" b "<<int(480*(float(sizeratio)/100.0f));
		
		if (pressedKey == (int)'1' || pressedKey == (int)'!' ){
			cvResizeWindow("Color",846,480);
			cvResizeWindow("NIR",846,480);
			cvResizeWindow("Superimpose",846,480);
			//cvResizeWindow("Superimpose",,);
			pressedKey = 0;
		}
		//imshow("Color",raw_visible);
		//cvResizeWindow("NIR",int(846*(float(sizeratio)/100.0f)),int(480*(float(sizeratio)/100.0f)));
		//resize(raw_align_jet, raw_align_jet, Size(int(846*(float(sizeratio)/100.0f)),int(480*(float(sizeratio)/100.0f))), 0, 0, INTER_NEAREST);
		imshow("NIR",raw_align_jet);

		
		
		if (SuperImposeEnable==true) {
			namedWindow("Superimpose",CV_WINDOW_NORMAL|CV_WINDOW_FREERATIO|CV_GUI_EXPANDED);
			namedWindow("Superimpose1",CV_WINDOW_NORMAL|CV_WINDOW_FREERATIO|CV_GUI_EXPANDED);
			cvResizeWindow("Superimpose",int(846*(float(sizeratio)/100.0f)),int(480*(float(sizeratio)/100.0f)));
			cvResizeWindow("Superimpose1",int(846*(float(sizeratio)/100.0f)),int(480*(float(sizeratio)/100.0f)));
			imshow("Superimpose",raw_overlay);
			imshow("Superimpose1",raw_overlay);
			if (SuperImposeRecordEnable) writer.write(raw_overlay);
			
		}else {
		destroyWindow("Superimpose");
		destroyWindow("Superimpose1");
		}
		//printf("Displaying.%d\n", dispCtr++);
		///give a proper delay!
		//waitKey(1); 
		final=clock()-init;
		cout << "display thread FPS "<< ((double)CLOCKS_PER_SEC)/(double)final <<endl;
			if (pressedKey == (int)'2') break;
	}
	 return 0;
}

//This code snippet will help you to read data from arduino

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"

using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char *port_name = "\\\\.\\COM3";

//String for incoming data
char incomingData[MAX_DATA_LENGTH];

// Entry point
int wmain( int, wchar_t** )
{
	//Arduino code
	SerialPort arduino(port_name);
	if (arduino.isConnected())
		cout << "Arduino Connection Established" << endl;
	else
		cout << "ERROR, check Arduino port" << endl;
	int value=0,lineoffset;
	int FrameNumber=1;
	int NewFrame=0;	
	char disp[100];
	char frameRate[100];
	char videoStatus[100];
	int frame=0;
	int videoFrame=0;
	int imageCount=1;
	double timeDif=0;
	float fps=0;
	int tickOld, tickDif;
	int checkFPS=1;

	//g_nMemSize =8388576;// 8*1024*1024;
	g_nMemSize=SDRAM_SIZE;
	// Allocate some buffer memory.
	g_buf = new unsigned char[g_nMemSize];
	g_buf_1 = new unsigned char[g_nMemSize];
	g_buf_2 = new unsigned char[g_nMemSize];
	
	g_buf1 = new unsigned char[g_nMemSize/2];
	g_buf2 = new unsigned char[g_nMemSize/2];

	//okCFrontPanel *dev;	

	// Initialize OpalKelly board
	int InitTemp=InitFPGA(&dev);
	if (InitTemp<0) return (InitTemp);

	// Set micronclk=sdram(100MHZ)/value
	dev->SetWireInValue(0x0A,4); // try 3 or 4
	dev->UpdateWireIns();
    printf("Finished initializing fpga \n");

	////int integration1, integration2;
	//cout<<"Please input the NIR integration time: (120-10000) rows (typical 480)" <<endl;
	//cin>>integration1;
	//cout<<"Please input the Visible integration time: (120-10000) rows (typical 480)" <<endl;
	//cin>>integration2;

	programImagerStereo(dev);
	printf("Finished programming imager \n");


	logo = imread("logo.jpg");
	if (logo.empty()) {
		logo.create(400,400,CV_8UC1);
		logo.setTo(128);
	}

	
	int statusvalue,statusvalue1, statusstate;
	///////
	int reg_address, reg_value;


	//initializeVideo();
	// power up the deserializer
	dev->SetWireInValue(0x0C,0); 
	dev->UpdateWireIns();
	dev->SetWireInValue(0x0C,1); 
	dev->UpdateWireIns();
	/////////////////////////////////

	///stop LVDS test//////////////
	reg_address=0x00B5; reg_value=0x0000;   
	//master
	programRegisterDual(dev, reg_address, reg_value, 0);
	programRegisterDual(dev, reg_address, reg_value, 1);
	//////////////////////////////////
	integration1=470;
	integration2=360;
	integration1_temp=470;
	integration2_temp=360;
	programRegisterDual(dev, 0x000B, integration1, 1);
	programRegisterDual(dev, 0x000B, integration2, 0);


	int integration_diff=integration1-integration2-1;
	// Reset write FIFOs
	dev->SetWireInValue(0x00, 0x0004);
	dev->UpdateWireIns();
	dev->SetWireInValue(0x00, 0x0000);
	dev->SetWireInValue(0x03,67); //68
	dev->SetWireInValue(0x04,751);//753
	dev->SetWireInValue(0x05,93);//91
	dev->SetWireInValue(0x02,22);//21
	dev->SetWireInValue(0x0D,480);//21
	dev->SetWireInValue(0x0E,844);//21
	/////////3.0 port/////////////////////
	//dev->SetWireInValue(0x03,67); //68
	//dev->SetWireInValue(0x04,845);//753  new: 846/2
	//dev->SetWireInValue(0x05,480);//91   new 480 lines
	//dev->SetWireInValue(0x0E,845);//21
	//////////////////////////////////
	dev->SetWireInValue(0x0F,integration_diff);
	dev->UpdateWireIns();
	
	for (int y = 0; y < HEIGHT; y++){
		x[y] = &x_data[y*WIDTH];	
		xy[y] = &xy_data[y*WIDTH];	
	}
	

	const_B=244, const_G=111, const_R=100;
	disparity_x=112;
	disparity_y=28;
	disp_threshold=33;
	disp_upthreshold=80;
	
	const_B1=830, const_B2=240, const_B3=245;
	const_G1=280, const_G2=435, const_G3=275;
	const_R1=300, const_R2=290, const_R3=410;
	const_gain=100,const_gamma=120;
	transparency=50;
	sizeratio=100;

	raw_align.create(rows,columns,CV_8UC1);
	raw_data1.create(rows,columns*2,CV_8UC1);
	raw_align_jet.create(rows,columns,CV_8UC3);
	raw_save.create(rows,columns*2,CV_8UC3);

	

	printf("1");

	
	int i=0;
	int status;

	
	HANDLE dispThrd = CreateThread(NULL,0,displayThread,NULL,0,NULL);
	//HANDLE featureThrd = CreateThread(NULL,0,featureThread,NULL,0,NULL);
	HANDLE dataThrd = CreateThread(NULL,0,dataThread,NULL,0,NULL);
	char fileName[50];
	
	while(1)
	{
		//printf("2");

		i++;
		//g_buf = new unsigned char[g_nMemSize];

			dev->SetWireInValue(0x00, 0x0004);
			dev->UpdateWireIns();
			dev->SetWireInValue(0x00, 0x0000);
			dev->UpdateWireIns();
			
			
	
			//cout<<"hi"<<endl;
			if (g_bufdone){
			dev->ReadFromBlockPipeOut(0xa1, 1024, g_nMemSize, g_buf);
			//cout<<"I am in buf 0"<<endl;
			g_bufbusy=true;
			g_bufdone=false;
			} else if (g_buf1done) {
			dev->ReadFromBlockPipeOut(0xa1, 1024, g_nMemSize, g_buf_1);
			//cout<<"I am in buf 1"<<endl;
			g_buf1busy=true;
			g_buf1done=false;
			} else if (g_buf2done){
			dev->ReadFromBlockPipeOut(0xa1, 1024, g_nMemSize, g_buf_2);
			//cout<<"I am in buf 2"<<endl;
			g_buf2busy=true;
			g_buf2done=false;
			}
			//SortDataRecordTestHorizonCrop();
			FILE *fp;
			string reserved;
			bool grabbingimage=false;
			if(video){
/* 			for (int i=0; i<SDRAM_SIZE; ++i){  
				x[i/WIDTH][i%WIDTH]=g_buf[i]; 
			} */

			/*if(firstVideoFrame){
				sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
				outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
				firstVideoFrame=false;
			}
			if(videoFileCount%2000==0){
				outFile.close();
				sprintf(fileName, "./videoData/experiment/videoData%d.bin", getBinFileNumber());
				outFile.clear();
				outFile.open(fileName,fstream::out|fstream::in|fstream::binary|fstream::app);
			}

				headerInfo[0]='\0';
				outFile.write(x[0], HEIGHT*WIDTH);
				outFile.flush();
				cout<<"saving video"<<endl;
				videoFileCount++; */
				
			///pvf trial
			vHeader.pixelSize = 1;
			vHeader.rows = HEIGHT;
			vHeader.cols = WIDTH;

			if(firstVideoFrame){
			sprintf(fileName, "./videoData/experiment/videoData%d.pvf", getBinFileNumber());
			fp = fopen(fileName,"w+b");
			writePvfHeader(&vHeader, &reserved,fp);
			firstVideoFrame=false;
			}
			if(videoFileCount%1000==0){
				
				writeNumberOfFramesToPvfHeader(&vHeader,fp);
				fclose(fp);
				sprintf(fileName, "./videoData/experiment/videoData%d.pvf", getBinFileNumber());
				fp = fopen(fileName,"w+b");
				videoFileCount=0;
				writePvfHeader(&vHeader, &reserved,fp);
			}
			//headerInfo[0]='\0';
			fwrite(x[0],vHeader.pixelSize,(HEIGHT*WIDTH),fp);
			vHeader.numberOfFrames = videoFileCount;
			grabbingimage=true;
			cout<<"saving video"<<endl;
			videoFileCount++;		
			}
			if (grabbingimage && !video) {
				writeNumberOfFramesToPvfHeader(&vHeader,fp);
				fclose(fp);
				videoFileCount=0;
				grabbingimage=false;
			}
			do {
				//dev->ReadFromBlockPipeOut(0xa1, 1024, g_nMemSize, g_buf);
				dev->UpdateWireOuts();
				status = dev->GetWireOutValue(0x26);
			} while (!(status & 0x0001));
			

		//	SortDataRecordTestHorizonCropColor1();
			
			if (integration_diff1_trigger) {
				changeMasterExp1(integration1, dev);
				integration_diff1_trigger=false;
			}
			if (integration_diff2_trigger) {
				changeSlaveExp1(integration2, dev);
				integration_diff2_trigger=false;
			}
				
				if (pressedKey == (int)'2') break;
		//	}
			

			
	}

	
	while(1){;}
		// close output binary file
		outFile.close();	
	
	// Free allocated storage.
	delete [] ImageBuffer;	
	delete [] g_buf;		
	
    return(0);
}