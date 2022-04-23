//#include "arm_const_structs.h"
//#include "arm_math.h"

#include "ppgSensor.h"
#include "imuSensor.h"
#include "BLEService.h"
#include "common.h"

#include <math.h>
#include <cstdio>
#define LED_GREEN 1

uint8_t timeWindow=50;
uint8_t blePktPPG_noFilter[ble_ppg_noFilter_byteLength];
uint8_t blePktPPG_Filter[ble_ppg_Filter_byteLength];
float32_t std_ppgThreshold_lower = 120;

uint8_t low_ch1 =0,up_ch1 = 0x3f;
uint8_t low_ch2 =0,up_ch2 = 0xff;
uint8_t adapt_counterCh1=0,adapt_counterCh2=0;
uint8_t adapt_Ch1=0,adapt_Ch2=0;


float32_t runningMeanCh1a =0.0f, runningMeanCh1aFil=0.0f, runningSquaredMeanCh1aFil=0.0f;
float32_t runningMeanCh1b=0.0f, runningMeanCh1bFil=0.0f, runningSquaredMeanCh1bFil=0.0f;
float32_t runningMeanCh2a=0.0f, runningMeanCh2aFil=0.0f, runningSquaredMeanCh2aFil=0.0f;
float32_t runningMeanCh2b=0.0f, runningMeanCh2bFil=0.0f, runningSquaredMeanCh2bFil=0.0f;
uint8_t goodCh1=1,goodCh2=1;
uint8_t badDataCounterCh1=0,badDataCounterCh2=0; 
uint32_t chLED_upperBound=390000;
uint32_t chLED_lowerBound=200000;
uint8_t prevFlag= 0,prevFlag2= 0;
uint8_t counterCheck=0;
float32_t std_ppgThreshold =150;

//#endif // #ifdef MOTIONSENSE_MAGNETO
uint8_t adaptIterMax = 30;
uint8_t binarySteps=0;

//static float ppg_num[400];

/*
const float firCoeffs32[NUM_TAPS] = {
0.006010912178927826700, 0.010363012813119724000 , 0.012556172512003569000 , 0.011034554116985092000 , 0.006690267346201923600 , 0.002513618458951131000 ,0.001426666415669687300,  
0.003871929193533284100, 0.007197771887541251800 , 0.007617434477145259300 , 0.003376433563263605800 , -0.003482040166678553800 , -0.008476412315024920200 ,-0.008361509784599709500,  
-0.004154768641755593600, -0.000737001166881119420 , -0.002932282184898419100 , -0.011103967255963519000 , -0.020163801946157165000 , -0.023395268351078545000 ,-0.018379828368095524000,  
-0.010000739003190312000, -0.007302648306080928100 , -0.015942441129921527000 , -0.032208879035078485000 , -0.044287878880340401000 , -0.041129409005509598000 ,-0.022691238322669009000,  
-0.002900614098316127100, -0.001123955975679971300 , -0.026690156725177258000 , -0.067595597616828887000 , -0.093306577324949902000 , -0.072408057407812684000 ,0.005229811227874942900,  
0.116902285272605680000, 0.215569954470445720000 , 0.254860768015382270000 , 0.215569954470445720000 , 0.116902285272605680000 , 0.005229811227874942900 ,-0.072408057407812684000,  
-0.093306577324949902000, -0.067595597616828887000 , -0.026690156725177258000 , -0.001123955975679971300 , -0.002900614098316127100 , -0.022691238322669009000 ,-0.041129409005509598000,  
-0.044287878880340401000, -0.032208879035078485000 , -0.015942441129921527000 , -0.007302648306080928100 , -0.010000739003190312000 , -0.018379828368095524000 ,-0.023395268351078545000,  
-0.020163801946157165000, -0.011103967255963519000 , -0.002932282184898419100 , -0.000737001166881119420 , -0.004154768641755593600 , -0.008361509784599709500 ,-0.008476412315024920200,  
-0.003482040166678553800, 0.003376433563263605800 , 0.007617434477145259300 , 0.007197771887541251800 , 0.003871929193533284100 , 0.001426666415669687300 ,0.002513618458951131000,  
0.006690267346201923600, 0.011034554116985092000 , 0.012556172512003569000 , 0.010363012813119724000 , 0.006010912178927826700 };

*/
/*
const float firCoeffs32[NUM_TAPS]={0.006472,0.011553,0.008098,0.001543,0.003484,0.011371,0.011303,0.001802,-0.001942,0.006467,
0.011921,0.002491,-0.008549,-0.003595,0.007925,0.003195,-0.014319,-0.017678,-0.002243,0.002865,
-0.016935,-0.032973,-0.019191,-0.000164,-0.014191,-0.045205,-0.042638,-0.008331,-0.003738,-0.048396,
-0.073158,-0.026925,0.020820,-0.028855,-0.125639,-0.090593,0.139589,0.376075,0.376075,0.139589,
-0.090593,-0.125639,-0.028855,0.020820,-0.026925,-0.073158,-0.048396,-0.003738,-0.008331,-0.042638,
-0.045205,-0.014191,-0.000164,-0.019191,-0.032973,-0.016935,0.002865,-0.002243,-0.017678,-0.014319,
0.003195,0.007925,-0.003595,-0.008549,0.002491,0.011921,0.006467,-0.001942,0.001802,0.011303,
0.011371,0.003484,0.001543,0.008098,0.011553,0.006472};

float firCoeffs32_50[NUM_TAPS]={0.002757,0.000648,-0.004372,-0.010298,-0.014513,-0.015054,-0.011655,-0.006013,-0.001101,0.000222,
-0.003339,-0.010728,-0.018857,-0.023971,-0.023473,-0.017342,-0.008398,-0.001174,0.000176,-0.005994,
-0.017706,-0.029921,-0.036676,-0.033946,-0.021915,-0.005499,0.007362,0.009118,-0.003634,-0.027726,
-0.053584,-0.068239,-0.060219,-0.024466,0.034737,0.104180,0.165573,0.201534,0.201534,0.165573,
0.104180,0.034737,-0.024466,-0.060219,-0.068239,-0.053584,-0.027726,-0.003634,0.009118,0.007362,
-0.005499,-0.021915,-0.033946,-0.036676,-0.029921,-0.017706,-0.005994,0.000176,-0.001174,-0.008398,
-0.017342,-0.023473,-0.023971,-0.018857,-0.010728,-0.003339,0.000222,-0.001101,-0.006013,-0.011655,
-0.015054,-0.014513,-0.010298,-0.004372,0.000648,0.002757};
float firCoeffs32_100[NUM_TAPS]={-0.004840,-0.007288,-0.010290,-0.013518,-0.016595,-0.019136,-0.020799,-0.021330,-0.020596,-0.018616,
-0.015564,-0.011760,-0.007646,-0.003735,-0.000558,0.001398,0.001756,0.000297,-0.002994,-0.007912,
-0.014031,-0.020736,-0.027266,-0.032787,-0.036473,-0.037584,-0.035557,-0.030066,-0.021073,-0.008854,
0.006014,0.022687,0.040112,0.057120,0.072518,0.085192,0.094208,0.098893,0.098893,0.094208,
0.085192,0.072518,0.057120,0.040112,0.022687,0.006014,-0.008854,-0.021073,-0.030066,-0.035557,
-0.037584,-0.036473,-0.032787,-0.027266,-0.020736,-0.014031,-0.007912,-0.002994,0.000297,0.001756,
0.001398,-0.000558,-0.003735,-0.007646,-0.011760,-0.015564,-0.018616,-0.020596,-0.021330,-0.020799,
-0.019136,-0.016595,-0.013518,-0.010290,-0.007288,-0.004840};
float firCoeffs32_200[NUM_TAPS]={-0.006486,-0.007891,-0.009428,-0.011060,-0.012742,-0.014426,-0.016062,-0.017597,-0.018979,-0.020155,
-0.021076,-0.021696,-0.021972,-0.021869,-0.021360,-0.020423,-0.019047,-0.017230,-0.014979,-0.012311,
-0.009253,-0.005842,-0.002123,0.001852,0.006021,0.010320,0.014677,0.019019,0.023271,0.027358,
0.031207,0.034748,0.037916,0.040652,0.042906,0.044634,0.045805,0.046396,0.046396,0.045805,
0.044634,0.042906,0.040652,0.037916,0.034748,0.031207,0.027358,0.023271,0.019019,0.014677,
0.010320,0.006021,0.001852,-0.002123,-0.005842,-0.009253,-0.012311,-0.014979,-0.017230,-0.019047,
-0.020423,-0.021360,-0.021869,-0.021972,-0.021696,-0.021076,-0.020155,-0.018979,-0.017597,-0.016062,
-0.014426,-0.012742,-0.011060,-0.009428,-0.007891,-0.006486};
*/
const float firCoeffs32[NUM_TAPS]={0.000650,0.000794,0.000688,0.000533,0.000627,0.000954,0.001155,0.000982,0.000659,0.000638,
0.000992,0.001266,0.001036,0.000506,0.000300,0.000646,0.001017,0.000771,0.000028,-0.000415,
-0.000105,0.000403,0.000214,-0.000710,-0.001411,-0.001155,-0.000455,-0.000482,-0.001513,-0.002469,
-0.002269,-0.001312,-0.001052,-0.002091,-0.003278,-0.003138,-0.001861,-0.001181,-0.002117,-0.003518,
-0.003459,-0.001821,-0.000597,-0.001330,-0.002954,-0.003046,-0.001047,0.000820,0.000372,-0.001532,
-0.001923,0.000370,0.002954,0.002859,0.000566,-0.000365,0.002066,0.005411,0.005744,0.002926,
0.001114,0.003427,0.007553,0.008441,0.004977,0.001864,0.003685,0.008599,0.010270,0.006131,
0.001249,0.002087,0.007779,0.010617,0.005951,-0.001162,-0.001932,0.004496,0.009097,0.004324,
-0.005423,-0.008594,-0.001543,0.005674,0.001603,-0.011090,-0.017716,-0.010309,0.000720,-0.001287,
-0.017156,-0.028798,-0.021629,-0.005033,-0.002684,-0.021962,-0.041388,-0.035822,-0.010622,0.000566,
-0.022634,-0.056333,-0.056281,-0.015048,0.018012,-0.010410,-0.084063,-0.112514,-0.017492,0.173084,
0.329006,0.329006,0.173084,-0.017492,-0.112514,-0.084063,-0.010410,0.018012,-0.015048,-0.056281,
-0.056333,-0.022634,0.000566,-0.010622,-0.035822,-0.041388,-0.021962,-0.002684,-0.005033,-0.021629,
-0.028798,-0.017156,-0.001287,0.000720,-0.010309,-0.017716,-0.011090,0.001603,0.005674,-0.001543,
-0.008594,-0.005423,0.004324,0.009097,0.004496,-0.001932,-0.001162,0.005951,0.010617,0.007779,
0.002087,0.001249,0.006131,0.010270,0.008599,0.003685,0.001864,0.004977,0.008441,0.007553,
0.003427,0.001114,0.002926,0.005744,0.005411,0.002066,-0.000365,0.000566,0.002859,0.002954,
0.000370,-0.001923,-0.001532,0.000372,0.000820,-0.001047,-0.003046,-0.002954,-0.001330,-0.000597,
-0.001821,-0.003459,-0.003518,-0.002117,-0.001181,-0.001861,-0.003138,-0.003278,-0.002091,-0.001052,
-0.001312,-0.002269,-0.002469,-0.001513,-0.000482,-0.000455,-0.001155,-0.001411,-0.000710,0.000214,
0.000403,-0.000105,-0.000415,0.000028,0.000771,0.001017,0.000646,0.000300,0.000506,0.001036,
0.001266,0.000992,0.000638,0.000659,0.000982,0.001155,0.000954,0.000627,0.000533,0.000688,
0.000794,0.000650};

float firCoeffs32_50[NUM_TAPS]={-0.000086,0.000232,0.000523,0.000722,0.000788,0.000727,0.000598,0.000490,0.000496,0.000679,
0.001038,0.001508,0.001970,0.002295,0.002390,0.002236,0.001904,0.001537,0.001305,0.001342,
0.001693,0.002292,0.002967,0.003504,0.003713,0.003509,0.002944,0.002206,0.001562,0.001260,
0.001445,0.002090,0.002996,0.003845,0.004313,0.004185,0.003438,0.002272,0.001051,0.000180,
-0.000032,0.000502,0.001602,0.002863,0.003789,0.003964,0.003207,0.001658,-0.000250,-0.001930,
-0.002835,-0.002666,-0.001491,0.000248,0.001871,0.002688,0.002246,0.000511,-0.002092,-0.004798,
-0.006755,-0.007322,-0.006316,-0.004109,-0.001535,0.000379,0.000768,-0.000749,-0.003899,-0.007802,
-0.011244,-0.013092,-0.012697,-0.010172,-0.006402,-0.002786,-0.000778,-0.001367,-0.004688,-0.009907,
-0.015435,-0.019443,-0.020486,-0.018060,-0.012856,-0.006628,-0.001653,0.000021,-0.002665,-0.009313,
-0.018055,-0.026067,-0.030460,-0.029298,-0.022390,-0.011570,-0.000318,0.007233,0.007636,-0.000590,
-0.016143,-0.034909,-0.050800,-0.057301,-0.049318,-0.024809,0.014283,0.062123,0.110132,0.148932,
0.170597,0.170597,0.148932,0.110132,0.062123,0.014283,-0.024809,-0.049318,-0.057301,-0.050800,
-0.034909,-0.016143,-0.000590,0.007636,0.007233,-0.000318,-0.011570,-0.022390,-0.029298,-0.030460,
-0.026067,-0.018055,-0.009313,-0.002665,0.000021,-0.001653,-0.006628,-0.012856,-0.018060,-0.020486,
-0.019443,-0.015435,-0.009907,-0.004688,-0.001367,-0.000778,-0.002786,-0.006402,-0.010172,-0.012697,
-0.013092,-0.011244,-0.007802,-0.003899,-0.000749,0.000768,0.000379,-0.001535,-0.004109,-0.006316,
-0.007322,-0.006755,-0.004798,-0.002092,0.000511,0.002246,0.002688,0.001871,0.000248,-0.001491,
-0.002666,-0.002835,-0.001930,-0.000250,0.001658,0.003207,0.003964,0.003789,0.002863,0.001602,
0.000502,-0.000032,0.000180,0.001051,0.002272,0.003438,0.004185,0.004313,0.003845,0.002996,
0.002090,0.001445,0.001260,0.001562,0.002206,0.002944,0.003509,0.003713,0.003504,0.002967,
0.002292,0.001693,0.001342,0.001305,0.001537,0.001904,0.002236,0.002390,0.002295,0.001970,
0.001508,0.001038,0.000679,0.000496,0.000490,0.000598,0.000727,0.000788,0.000722,0.000523,
0.000232,-0.000086};

float firCoeffs32_100[NUM_TAPS]={0.002664,0.002612,0.002435,0.002135,0.001723,0.001222,0.000664,0.000090,-0.000459,-0.000940,
-0.001316,-0.001556,-0.001642,-0.001570,-0.001349,-0.001003,-0.000570,-0.000097,0.000362,0.000754,
0.001028,0.001140,0.001062,0.000781,0.000300,-0.000356,-0.001147,-0.002017,-0.002902,-0.003732,
-0.004441,-0.004969,-0.005271,-0.005321,-0.005114,-0.004670,-0.004030,-0.003258,-0.002428,-0.001628,
-0.000943,-0.000455,-0.000229,-0.000310,-0.000717,-0.001439,-0.002438,-0.003646,-0.004973,-0.006316,
-0.007564,-0.008609,-0.009357,-0.009736,-0.009705,-0.009258,-0.008425,-0.007275,-0.005908,-0.004448,
-0.003035,-0.001812,-0.000911,-0.000441,-0.000479,-0.001060,-0.002171,-0.003749,-0.005686,-0.007836,
-0.010025,-0.012064,-0.013766,-0.014966,-0.015529,-0.015373,-0.014473,-0.012868,-0.010666,-0.008032,
-0.005183,-0.002369,0.000144,0.002098,0.003262,0.003458,0.002580,0.000605,-0.002398,-0.006262,
-0.010732,-0.015475,-0.020103,-0.024195,-0.027325,-0.029098,-0.029176,-0.027306,-0.023346,-0.017277,
-0.009214,0.000593,0.011773,0.023851,0.036274,0.048440,0.059742,0.069599,0.077492,0.083002,
0.085833,0.085833,0.083002,0.077492,0.069599,0.059742,0.048440,0.036274,0.023851,0.011773,
0.000593,-0.009214,-0.017277,-0.023346,-0.027306,-0.029176,-0.029098,-0.027325,-0.024195,-0.020103,
-0.015475,-0.010732,-0.006262,-0.002398,0.000605,0.002580,0.003458,0.003262,0.002098,0.000144,
-0.002369,-0.005183,-0.008032,-0.010666,-0.012868,-0.014473,-0.015373,-0.015529,-0.014966,-0.013766,
-0.012064,-0.010025,-0.007836,-0.005686,-0.003749,-0.002171,-0.001060,-0.000479,-0.000441,-0.000911,
-0.001812,-0.003035,-0.004448,-0.005908,-0.007275,-0.008425,-0.009258,-0.009705,-0.009736,-0.009357,
-0.008609,-0.007564,-0.006316,-0.004973,-0.003646,-0.002438,-0.001439,-0.000717,-0.000310,-0.000229,
-0.000455,-0.000943,-0.001628,-0.002428,-0.003258,-0.004030,-0.004670,-0.005114,-0.005321,-0.005271,
-0.004969,-0.004441,-0.003732,-0.002902,-0.002017,-0.001147,-0.000356,0.000300,0.000781,0.001062,
0.001140,0.001028,0.000754,0.000362,-0.000097,-0.000570,-0.001003,-0.001349,-0.001570,-0.001642,
-0.001556,-0.001316,-0.000940,-0.000459,0.000090,0.000664,0.001222,0.001723,0.002135,0.002435,
0.002612,0.002664};

float firCoeffs32_200[NUM_TAPS]={-0.005035,-0.004868,-0.004660,-0.004416,-0.004140,-0.003839,-0.003521,-0.003192,-0.002860,-0.002534,
-0.002222,-0.001933,-0.001675,-0.001455,-0.001280,-0.001157,-0.001089,-0.001083,-0.001139,-0.001259,
-0.001444,-0.001692,-0.001999,-0.002361,-0.002772,-0.003225,-0.003712,-0.004222,-0.004747,-0.005274,
-0.005792,-0.006291,-0.006757,-0.007182,-0.007552,-0.007861,-0.008097,-0.008255,-0.008329,-0.008314,
-0.008208,-0.008012,-0.007727,-0.007357,-0.006908,-0.006387,-0.005805,-0.005172,-0.004503,-0.003811,
-0.003112,-0.002421,-0.001755,-0.001132,-0.000566,-0.000074,0.000330,0.000633,0.000822,0.000888,
0.000824,0.000625,0.000290,-0.000182,-0.000786,-0.001516,-0.002362,-0.003311,-0.004347,-0.005453,
-0.006607,-0.007788,-0.008971,-0.010130,-0.011239,-0.012271,-0.013199,-0.013996,-0.014638,-0.015101,
-0.015363,-0.015404,-0.015209,-0.014763,-0.014058,-0.013087,-0.011850,-0.010348,-0.008588,-0.006582,
-0.004345,-0.001896,0.000741,0.003540,0.006472,0.009502,0.012596,0.015718,0.018829,0.021890,
0.024863,0.027710,0.030393,0.032878,0.035130,0.037121,0.038823,0.040214,0.041274,0.041988,
0.042348,0.042348,0.041988,0.041274,0.040214,0.038823,0.037121,0.035130,0.032878,0.030393,
0.027710,0.024863,0.021890,0.018829,0.015718,0.012596,0.009502,0.006472,0.003540,0.000741,
-0.001896,-0.004345,-0.006582,-0.008588,-0.010348,-0.011850,-0.013087,-0.014058,-0.014763,-0.015209,
-0.015404,-0.015363,-0.015101,-0.014638,-0.013996,-0.013199,-0.012271,-0.011239,-0.010130,-0.008971,
-0.007788,-0.006607,-0.005453,-0.004347,-0.003311,-0.002362,-0.001516,-0.000786,-0.000182,0.000290,
0.000625,0.000824,0.000888,0.000822,0.000633,0.000330,-0.000074,-0.000566,-0.001132,-0.001755,
-0.002421,-0.003112,-0.003811,-0.004503,-0.005172,-0.005805,-0.006387,-0.006908,-0.007357,-0.007727,
-0.008012,-0.008208,-0.008314,-0.008329,-0.008255,-0.008097,-0.007861,-0.007552,-0.007182,-0.006757,
-0.006291,-0.005792,-0.005274,-0.004747,-0.004222,-0.003712,-0.003225,-0.002772,-0.002361,-0.001999,
-0.001692,-0.001444,-0.001259,-0.001139,-0.001083,-0.001089,-0.001157,-0.001280,-0.001455,-0.001675,
-0.001933,-0.002222,-0.002534,-0.002860,-0.003192,-0.003521,-0.003839,-0.004140,-0.004416,-0.004660,
-0.004868,-0.005035};

float firCoeffsMA32[NUM_TAPS_MA_FIL] ;


struct fs_file_t fileData;
char fname[255];

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
        .mnt_point = "/lfs",
	.fs_data = &storage,
	.storage_dev = (void *)FLASH_AREA_ID(storage)
};

uint32_t blockSize = BLOCK_SIZE;
/* Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
float firStateF32_chan1A[BLOCK_SIZE + NUM_TAPS - 1], firStateF32_chan1B[BLOCK_SIZE + NUM_TAPS - 1], firStateF32_chan2A[BLOCK_SIZE + NUM_TAPS - 1], firStateF32_chan2B[BLOCK_SIZE + NUM_TAPS - 1];
float firStateF32_chan1AMA[BLOCK_SIZE + NUM_TAPS_MA_FIL - 1],firStateF32_chan1BMA[BLOCK_SIZE + NUM_TAPS_MA_FIL - 1], firStateF32_chan2AMA[BLOCK_SIZE + NUM_TAPS_MA_FIL - 1], firStateF32_chan2BMA[BLOCK_SIZE + NUM_TAPS_MA_FIL - 1];

arm_fir_instance_f32 S_chan1A, S_chan1B, S_chan2A, 
  S_chan2B ,S_chan1AMA, S_chan1BMA, S_chan2AMA, S_chan2BMA ;

arm_fir_instance_f32 S_chan1A_fil, S_chan1B_fil, 
  S_chan2A_fil, S_chan2B_fil ,S_chan1AMA_fil, 
  S_chan1BMA_fil, S_chan2AMA_fil, S_chan2BMA_fil ;

// Channel 1A - IR 1
// Channel 1B - IR 2
// Channel 2A - Green 1
// Channel 2B - Green 2

void spiRead_registerPPG(uint8_t * tx_buffer, 
  uint8_t txLen, uint8_t * rx_buffer, uint8_t rxLen){
  int err;
  const struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = txLen
  };
  const struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
  };
	
  struct spi_buf rx_buf = {
    .buf = rx_buffer,
    .len = rxLen
  };
  const struct spi_buf_set rx = {
    .buffers = &rx_buf,
    .count = 1
  };
  err = spi_transceive(spi_dev_ppg, &spi_cfg_ppg, &tx, &rx);
  if (err) 
    printk("SPI error: %d\n", err);
}

void spiWrite_registerPPG(uint8_t * tx_buffer, uint8_t txLen){
  int err;
  const struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = txLen
  };
  const struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
  };
  err = spi_transceive(spi_dev_ppg, &spi_cfg_ppg, &tx,NULL);
  if (err) 
    printk("SPI error: %d\n", err);
	
}

void high_pass_filter_init_25(void){
/* Call FIR init function to initialize the instance structure. */
  arm_fill_f32(1.0/NUM_TAPS_MA_FIL, (float *) &firCoeffsMA32[0],
    NUM_TAPS_MA_FIL);
  arm_fir_init_f32(&S_chan1A, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0],
    &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2BMA[0], blockSize);
	
  arm_fir_init_f32(&S_chan1A_fil, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B_fil, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A_fil, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B_fil, NUM_TAPS, (float *)&firCoeffs32[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2BMA[0], blockSize);
}

void high_pass_filter_init_50(void){
/* Call FIR init function to initialize the instance structure. */
  arm_fill_f32(1.0/NUM_TAPS_MA_FIL, (float *) &firCoeffsMA32[0],
    NUM_TAPS_MA_FIL);
  arm_fir_init_f32(&S_chan1A, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0],
    &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2BMA[0], blockSize);
	
  arm_fir_init_f32(&S_chan1A_fil, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B_fil, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A_fil, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B_fil, NUM_TAPS, (float *)&firCoeffs32_50[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2BMA[0], blockSize);
}

void high_pass_filter_init_100(void){
/* Call FIR init function to initialize the instance structure. */
  arm_fill_f32(1.0/NUM_TAPS_MA_FIL, (float *) &firCoeffsMA32[0],
    NUM_TAPS_MA_FIL);
  arm_fir_init_f32(&S_chan1A, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0],
    &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2BMA[0], blockSize);
	
  arm_fir_init_f32(&S_chan1A_fil, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B_fil, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A_fil, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B_fil, NUM_TAPS, (float *)&firCoeffs32_100[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2BMA[0], blockSize);
}

void high_pass_filter_init_200(void){
/* Call FIR init function to initialize the instance structure. */
  arm_fill_f32(1.0/NUM_TAPS_MA_FIL, (float *) &firCoeffsMA32[0],
    NUM_TAPS_MA_FIL);
  arm_fir_init_f32(&S_chan1A, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0],
    &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA, NUM_TAPS_MA_FIL, (float *)&firCoeffsMA32[0], 
    &firStateF32_chan2BMA[0], blockSize);
	
  arm_fir_init_f32(&S_chan1A_fil, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan1A[0], blockSize);
  arm_fir_init_f32(&S_chan1B_fil, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan1B[0], blockSize);
  arm_fir_init_f32(&S_chan2A_fil, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan2A[0], blockSize);
  arm_fir_init_f32(&S_chan2B_fil, NUM_TAPS, (float *)&firCoeffs32_200[0], 
    &firStateF32_chan2B[0], blockSize);

  arm_fir_init_f32(&S_chan1AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1AMA[0], blockSize);
  arm_fir_init_f32(&S_chan1BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan1BMA[0], blockSize);
  arm_fir_init_f32(&S_chan2AMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2AMA[0], blockSize);
  arm_fir_init_f32(&S_chan2BMA_fil, NUM_TAPS_MA_FIL, 
    (float *)&firCoeffsMA32[0], &firStateF32_chan2BMA[0], blockSize);
}

void ppg_config(void){
  if (ppgConfig.isEnabled) {
    fileOpen();
    uint32_t dataFlash = fileRead();
    fileClose();
    ppgConfig.green_intensity = (dataFlash&0x0000FF00) >>8;
    ppgConfig.infraRed_intensity = (dataFlash&0x000000FF);
    
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, READMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
    spiRead_registerPPG(cmd_array, txLen, read_array, rxLen);

    // Resetting PPG sensor
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_RESET;
    spiWrite_registerPPG(cmd_array, txLen);

    // Shutting down PPG sensor
    cmd_array[2] = PPG_SHUTDOWN;
    spiWrite_registerPPG(cmd_array, txLen);

    // Reading interrupt status register 1
    cmd_array[0] = PPG_INT_STAT_1;
    cmd_array[1] = READMASTER;
    spiRead_registerPPG(cmd_array, txLen, read_array, rxLen);

    // Reading interrupt status register 2	
    cmd_array[0] = PPG_INT_STAT_2;
    spiRead_registerPPG(cmd_array, txLen, read_array, rxLen);

    #if  defined(LED_RED )
    //nrf_delay_ms(5);
		cmd_array[0] = 0x11;		
		cmd_array[1] = 0x00;
		cmd_array[2] = 0x03; // Red LED 0x3F
		nrf_drv_spi_transfer(&spi_ppg, cmd_array, 3, read_array, 3);
    while (!spi_ppg_xfer_done)
    {
        __WFE();
    }
		spi_ppg_xfer_done = false;
    #endif
  // PPG configuration register - ALC enabled + 
  // PPG ADC Range - 4096 nA, 117.3us integration time
    #if  defined(LED_GREEN )
      cmd_array[0] = PPG_CONFIG_1;		
      cmd_array[1] = WRITEMASTER;
      cmd_array[2] = PPG_TINT_117_3us; 
      spiWrite_registerPPG(cmd_array, txLen);
    #endif
		
// Change Sampling rate PPG
    cmd_array[0] = PPG_CONFIG_2;
    cmd_array[2] = PPG_SR_400_1 | ppgConfig.sample_avg;
    spiWrite_registerPPG(cmd_array, txLen);
      
    //PPG coniguration 3- LED settling time =12us
    cmd_array[0] = PPG_CONFIG_3;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_LED_SETLNG_12us;
    spiWrite_registerPPG(cmd_array, txLen);
      
    // Photo-diode Bias 0 to 65pF 
    cmd_array[0] = PPG_PHOTODIODE_BIAS;
    cmd_array[2] = (uint8_t)(PPG_PDBIAS_65pF<<4) | (uint8_t)PPG_PDBIAS_65pF; 
    spiWrite_registerPPG(cmd_array, txLen);
      
      // Configuring LED drive 3 (Green) range 124 mA
      //            LED drive 2 (Green) range 124 mA	
      //            LED drive 1 (IR) range 31 mA
    #if  defined(LED_GREEN )
      cmd_array[0] = PPG_LED_RANGE_1; 
      cmd_array[2] = (uint8_t)(PPG_LED_CURRENT_124mA <<4) | (uint8_t)(PPG_LED_CURRENT_124mA <<2)|PPG_LED_CURRENT_31mA; 
      spiWrite_registerPPG(cmd_array, txLen);
    #endif
		
	  #if  defined(LED_RED )
		cmd_array[0] = 0x2A;
		cmd_array[2] = 0x00; // change back to x15
    //cmd_array[2] = 0x03;
		nrf_drv_spi_transfer(&spi_ppg, cmd_array, 3, read_array, 3);
    while (!spi_ppg_xfer_done)
    {
        __WFE();
    }
		spi_ppg_xfer_done = false;
	  //nrf_delay_ms(5);
		#endif
                
    // LED 1 Driver current setting (IR )
    cmd_array[0] = PPG_LED1_PA;
    cmd_array[2] = ppgConfig.infraRed_intensity; 
    spiWrite_registerPPG(cmd_array, txLen);
    
    // LED 2 Driver current setting (Green )
    cmd_array[0] = PPG_LED2_PA;
    cmd_array[2] = ppgConfig.green_intensity; 
    spiWrite_registerPPG(cmd_array, txLen);
    
    // LED 3 Driver current setting (Green )
    cmd_array[0] = PPG_LED3_PA;
    spiWrite_registerPPG(cmd_array, txLen);

    // System control reqister - Low Power Mode + shutoddown 
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[2] = PPG_LP_MODE | PPG_SHUTDOWN;
    spiWrite_registerPPG(cmd_array, txLen);    
    
    // FIFO configuration - 15 samples stored in FIFO
    cmd_array[0] = PPG_FIFO_CONFIG_1;
    cmd_array[2] = 0x0F; 
    spiWrite_registerPPG(cmd_array, txLen);    

    // FIFO configuration 2 Push enable when FIFO is full
    cmd_array[0] = PPG_FIFO_CONFIG_2;
    cmd_array[2] = PPG_FIFO_PUSH_ENABLE;
    spiWrite_registerPPG(cmd_array, txLen);    

    // Interrupt Enable A_full interrupt is enabled
    cmd_array[0] = PPG_INT_EN_1;
    cmd_array[2] = PPG_INT_A_FULL_EN;
    spiWrite_registerPPG(cmd_array, txLen);    

    // Green LED 2 and LED 3 is pulsed simultaneously first 
    // then IR LED is pulsed next
    cmd_array[0] = PPG_LED_SEQ_1;
    cmd_array[2] = PPG_LEDC2_LED2_LED3_SIMULT | PPG_LEDC1_LED1; 
    spiWrite_registerPPG(cmd_array, txLen);    

    // System control Dual PPG + Low power mode enabled
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[2] = PPG_LP_MODE;
    spiWrite_registerPPG(cmd_array, txLen);  
    
    low_ch1 =0; up_ch1 = 0x3f;
    low_ch2 =0; up_ch2 = 0xff;
    adapt_counterCh1=0;
    adapt_counterCh2=0;
    counterCheck =0;
    badDataCounterCh1=0;  
    badDataCounterCh2=0;
    prevFlag=0;prevFlag2=0;
    adapt_Ch1=1;
    adapt_Ch2=1;
    goodCh1=0;
    goodCh2=0;
    ppgData1.bufferIndex=0;
  }
}
void ppg_changeIntensity(void){
  if (ppgConfig.isEnabled) {
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
   // LED 1 Driver current setting (IR )
    cmd_array[0] = PPG_LED1_PA;
    cmd_array[2] = ppgConfig.infraRed_intensity; 
    spiWrite_registerPPG(cmd_array, txLen);
    
    // LED 2 Driver current setting (Green )
    cmd_array[0] = PPG_LED2_PA;
    cmd_array[2] = ppgConfig.green_intensity; 
    spiWrite_registerPPG(cmd_array, txLen);
    
    // LED 3 Driver current setting (Green )
    cmd_array[0] = PPG_LED3_PA;
    spiWrite_registerPPG(cmd_array, txLen);
  }
}
void ppg_changeSamplingRate(void){
  if (ppgConfig.isEnabled) {
    uint8_t rxLen,txLen; 
    // Read chip ID 
    uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
    uint8_t read_array[5] = {0};
    txLen=3;
    rxLen=3;
  // Change Sampling rate PPG
    cmd_array[0] = PPG_CONFIG_2;
    cmd_array[2] = PPG_SR_400_1 | ppgConfig.sample_avg;
    spiWrite_registerPPG(cmd_array, txLen);
  }
}

/**	
 * @brief Function for configuring the ppg sensor to shutdown mode.
 *
 * @details  Configures shutdown bit in mode registers .
 */
void ppg_sleep(void){
  uint8_t txLen=3; 
  if (ppgConfig.isEnabled) {
    uint8_t cmd_array[] = {0xFF, 0x80, 0xFF};
   
    // System control Shutdown
    cmd_array[0] = PPG_SYS_CTRL;
    cmd_array[1] = WRITEMASTER;
    cmd_array[2] = PPG_SHUTDOWN;
    spiWrite_registerPPG(cmd_array, txLen);  
    
  }
}

uint8_t searchStep( uint8_t adapt_counter, float meanCha,float stdCha_fil,
uint8_t* low_ch,uint8_t* up_ch, uint8_t midVal,uint8_t stepSize)
{
  printk("in f search step, mean: %d\n", meanCha);
  if(meanCha < chLED_upperBound-40000 ){
    if(stdCha_fil > std_ppgThreshold ){
	
    }
    else{ // room for improvement so increase led current
      if(adapt_counter <binarySteps ){
        *low_ch = (*low_ch + *up_ch)/2+1;
	midVal = (*low_ch + *up_ch)/2;
      }
      else{
        if(midVal +  stepSize < 0xFF)
          midVal=midVal+ stepSize;
      }
    }						
  }
  else{ // close to saturation so decrease led current
    if(adapt_counter < binarySteps){
      *up_ch = (*low_ch + *up_ch)/2-1;
      midVal = (*low_ch + *up_ch)/2;
    }
    else{
      if(midVal-stepSize >= 0)
        midVal=midVal-stepSize;
    }
  }
return midVal;
}

void ppg_led_currentUpdate(void){
  //Ch1a - IR1, Ch1b - IR2, Ch2a - G1, Ch2b - G2 
  float meanCh1=0,meanCh2=0;	
  float stdCh1_fil=0,stdCh2_fil=0; 
  uint8_t IR_steps = 1, green_steps = 3;
  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t txLen=3;
  if (ppgConfig.isEnabled){
    if(counterCheck == timeWindow){
      if(gyroData1.movingFlag ==0){ // Motion is minimal
        arm_sqrt_f32(runningSquaredMeanCh1aFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh1aFil*runningMeanCh1aFil, &ppgData1.stdChanIR_1);
	arm_sqrt_f32(runningSquaredMeanCh1bFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh1bFil*runningMeanCh1bFil, &ppgData1.stdChanIR_2);
	arm_sqrt_f32(runningSquaredMeanCh2aFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh2aFil*runningMeanCh2aFil, &ppgData1.stdChanGreen_1);
	arm_sqrt_f32(runningSquaredMeanCh2bFil - timeWindow/(timeWindow-1.0f)
          *runningMeanCh2bFil*runningMeanCh2bFil, &ppgData1.stdChanGreen_2);

	ppgData1.meanChanIR_1 = runningMeanCh1a;
	ppgData1.meanChanIR_2 = runningMeanCh1b;
	ppgData1.meanChanGreen_1 = runningMeanCh2a;
	ppgData1.meanChanGreen_2 = runningMeanCh2b;

	if(ppgData1.meanChanIR_1>= ppgData1.meanChanIR_2){
          meanCh1 = ppgData1.meanChanIR_1;
          stdCh1_fil = ppgData1.stdChanIR_1;	
	}
        else{
          meanCh1 = ppgData1.meanChanIR_2;
          stdCh1_fil = ppgData1.stdChanIR_2;	
	}
	if(ppgData1.meanChanGreen_1>= ppgData1.meanChanGreen_2){
          meanCh2 = ppgData1.meanChanGreen_1;
          stdCh2_fil = ppgData1.stdChanGreen_1;	
        }
        else{
          meanCh2 = ppgData1.meanChanGreen_2;
          stdCh2_fil = ppgData1.stdChanGreen_2;	
        }		
      }
      // If the adaptation flag is disabled and data quality is bad when the sensor is not moving
      printk("adapt_flag: %d\n", adapt_Ch2);
      printk("bad counter: %d\n", badDataCounterCh2);
      if(gyroData1.movingFlag ==0){
        if(adapt_Ch1==0){
          if(meanCh1 > chLED_upperBound || meanCh1 < chLED_lowerBound )
            badDataCounterCh1++;
          if(badDataCounterCh1 > 10){
            adapt_counterCh1=0;
            adapt_Ch1=1;
            badDataCounterCh1 = 0;
          }						
        }
        if(adapt_Ch2==0){
          if(meanCh2 > chLED_upperBound || meanCh2 < chLED_lowerBound )
            badDataCounterCh2++;
          if(badDataCounterCh2 > 10){
            adapt_counterCh2=0;
            adapt_Ch2=1;
            badDataCounterCh2 = 0;
          }						
	}
      }
      printk("moving flag: %d\n", gyroData1.movingFlag);
      if(gyroData1.movingFlag ==0 && adapt_Ch1 ==1){ // Motion is minimal an adaptation is required
        ppgConfig.infraRed_intensity = searchStep( 
                adapt_counterCh1,meanCh1, stdCh1_fil,
		&low_ch1,&up_ch1,ppgConfig.infraRed_intensity, IR_steps);
			 
	cmd_array[0] = PPG_LED1_PA;
	cmd_array[2] = ppgConfig.infraRed_intensity; // changing it to 0x10 from 0x20
        spiWrite_registerPPG(cmd_array, txLen);  
				
	adapt_counterCh1++;
	printk("adapt counter length: %d\n", adapt_counterCh1);
	if(adapt_counterCh1 >adaptIterMax) {
          adapt_counterCh1=adaptIterMax;
          adapt_Ch1=0;
          goodCh1 = 1;
          badDataCounterCh1=0;
        }
      }
      if(gyroData1.movingFlag ==0 && adapt_Ch2 ==1){
        ppgConfig.green_intensity = searchStep( 
          adapt_counterCh2,meanCh2, stdCh2_fil,
          &low_ch2,&up_ch2,ppgConfig.green_intensity, green_steps);
	txLen=3;			
	cmd_array[0] = PPG_LED2_PA;
        printk("new ppg intensity: %d\n", ppgConfig.green_intensity);
	cmd_array[2] = ppgConfig.green_intensity; // Green 49.9mA
        spiWrite_registerPPG(cmd_array, txLen);  
				
	cmd_array[0] = PPG_LED3_PA;
        spiWrite_registerPPG(cmd_array, txLen);  
	adapt_counterCh2++;
	if(adapt_counterCh2 >adaptIterMax) {
          adapt_counterCh2=adaptIterMax;
          adapt_Ch2=0;
          goodCh2 =1;
          badDataCounterCh2=0;
	}
      }
      if(adapt_counterCh2 ==adaptIterMax && adapt_counterCh2 ==adaptIterMax ){
        fileOpen();
        uint32_t dataFlash = (ppgConfig.green_intensity)<<8 + ppgConfig.infraRed_intensity;
        fileWrite(dataFlash);
        fileClose();
      }
    }
  }
}

void read_ppg_fifo_buffer(struct k_work *item){
  struct ppgInfo* the_device=  ((struct ppgInfo *)(((char *)(item)) 
    - offsetof(struct ppgInfo, work)));
  
  uint16_t pktCounter = the_device->pktCounter;
  bool movingFlag = the_device->movingFlag;
  bool ppgTFPass = the_device->ppgTFPass;
  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t read_array[128*2*2*3] = {0};
  uint8_t txLen,rxLen;
  uint32_t led1A[32];
  uint32_t led1B[32];
  uint32_t led2A[32];
  uint32_t led2B[32];
  uint8_t tag1A, tag1B, tag2A, tag2B, tag;
  float channel1A_in, channel1B_in, channel2A_in, channel2B_in;
  float meanChannel1A, meanChannel1B, meanChannel2A, meanChannel2B;

  float channel1A_out, channel1B_out, channel2A_out, channel2B_out;
  float_cast buff_val_filtered;
  uint32_cast buff_val_raw;
  int i;
  uint8_t j, k;
  uint8_t sampleCnt[5] = {0};
		
  // Reading the total number of PPG samples
  cmd_array[0] = PPG_FIFO_DATA_COUNTER;
  cmd_array[1] = READMASTER;
  txLen=3;
  rxLen=3;
  spiRead_registerPPG(cmd_array, txLen, sampleCnt, rxLen);
    
  // Reading the PPG samples
  cmd_array[0] = PPG_FIFO_DATA;

  
  spiRead_registerPPG(cmd_array, txLen, read_array,sampleCnt[2]*3+2);
  for (i=0; i<sampleCnt[2]; i++){
    for (j=0; j <= 9; j=j+3){
      tag = (read_array[i*12+j+2] & 0xF8) >>3;
      switch(tag){
        case PPG1_LEDC1_DATA: // IR 1
          led1A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG1_LEDC2_DATA: // Green 1
          led2A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC1_DATA: // IR 2
          led1B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC2_DATA: // Green 2
          led2B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
      }
    }
    k = i;
    k = 0;
    tag1A = (read_array[k*12+0+2] & 0xF8) >>3;
    tag1B = (read_array[k*12+3+2] & 0xF8) >>3;
    tag2A = (read_array[k*12+6+2] & 0xF8) >>3;
    tag2B = (read_array[k*12+9+2] & 0xF8) >>3;
  }

  if(counterCheck ==0){
    runningMeanCh1a=0.0f;
    runningMeanCh1b=0.0f;
    runningMeanCh2a=0.0f;
    runningMeanCh2b=0.0f;
    runningMeanCh1aFil= 0.0f;
    runningMeanCh1bFil =0.0f;
    runningMeanCh2aFil =0.0f;
    runningMeanCh2bFil =0.0f;
    runningSquaredMeanCh1aFil= 0.0f;
    runningSquaredMeanCh1bFil =0.0f;
    runningSquaredMeanCh2aFil =0.0f;
    runningSquaredMeanCh2bFil =0.0f;
  }
  runningMeanCh1a = runningMeanCh1a +led1A[0]*1.0f/timeWindow;
  runningMeanCh1b = runningMeanCh1b +led1B[0]*1.0f/timeWindow;
  runningMeanCh2a = runningMeanCh2a +led2A[0]*1.0f/timeWindow;
  runningMeanCh2b = runningMeanCh2b +led2B[0]*1.0f/timeWindow;
		
  buff_val_raw.integer = led1A[0];
  blePktPPG_noFilter[0] = ((buff_val_raw.intcast[2]&0x07)<<5)|((buff_val_raw.intcast[1])&0xF8)>>3;
  blePktPPG_noFilter[1] = ((buff_val_raw.intcast[1]&0x07)<<5)|((buff_val_raw.intcast[0]&0xF8)>>3);
  blePktPPG_noFilter[2] =  (buff_val_raw.intcast[0]&0x07)<<5;
        
  buff_val_raw.integer = led1B[0];
  blePktPPG_noFilter[2] = blePktPPG_noFilter[2] | ((buff_val_raw.intcast[2]&0x07)<<2) |((buff_val_raw.intcast[1]&0xC0)>>6);
  blePktPPG_noFilter[3] = ((buff_val_raw.intcast[1]&0x3F)<<2) | ((buff_val_raw.intcast[0]&0xC0) >>6);
  blePktPPG_noFilter[4] = ((buff_val_raw.intcast[0]&0x3F) <<2) ;
        
  buff_val_raw.integer = led2A[0];
  blePktPPG_noFilter[4] = blePktPPG_noFilter[4] | ((buff_val_raw.intcast[2]&0x06)>>1);
  blePktPPG_noFilter[5] = ((buff_val_raw.intcast[2]&0x01)<<7) | ((buff_val_raw.intcast[1]&0xFE)>>1);
  blePktPPG_noFilter[6] = ((buff_val_raw.intcast[1]&0x01)<<7) | ((buff_val_raw.intcast[0]&0xFE)>>1);
  blePktPPG_noFilter[7] = ((buff_val_raw.intcast[0]&0x01)<<7);
        
  buff_val_raw.integer = led2B[0];
  blePktPPG_noFilter[7] = blePktPPG_noFilter[7] | ((buff_val_raw.intcast[2]&0x07) <<4)|((buff_val_raw.intcast[1]&0xF0) >>4);
  blePktPPG_noFilter[8] = ((buff_val_raw.intcast[1]&0x0F) <<4) | ((buff_val_raw.intcast[0]&0xF0)>>4);
  blePktPPG_noFilter[9] =  (buff_val_raw.intcast[0]&0x0F) <<4;
  blePktPPG_noFilter[10] = (pktCounter&0xFF00) >> 8;
  blePktPPG_noFilter[11] = (pktCounter&0x00FF);
    
  channel1A_in = led1A[0]*FLOAT_CON_16UA;
  arm_fir_f32(&S_chan1A_fil, &channel1A_in, &channel1A_out, blockSize);
  arm_fir_f32(&S_chan1AMA_fil, &channel1A_out, &meanChannel1A, blockSize);
  buff_val_filtered.float_val = channel1A_out - meanChannel1A;
  runningMeanCh1aFil = runningMeanCh1aFil +(channel1A_out - meanChannel1A)/(timeWindow*FLOAT_CON_16UA);
  runningSquaredMeanCh1aFil = runningSquaredMeanCh1aFil +(channel1A_out -meanChannel1A)*
    (channel1A_out - meanChannel1A)/(FLOAT_CON_16UA*FLOAT_CON_16UA*timeWindow);
  ppgData1.infraRed_ch1 = buff_val_filtered.float_val;                  
  blePktPPG_Filter[0] = buff_val_filtered.floatcast[0];
  blePktPPG_Filter[1] = buff_val_filtered.floatcast[1];
  blePktPPG_Filter[2] = buff_val_filtered.floatcast[2];
  blePktPPG_Filter[3] = buff_val_filtered.floatcast[3];
        
  channel1B_in = led1B[0]*FLOAT_CON_16UA;
  arm_fir_f32(&S_chan1B_fil, &channel1B_in, &channel1B_out, blockSize);
  arm_fir_f32(&S_chan1BMA_fil, &channel1B_out, &meanChannel1B, blockSize);
  buff_val_filtered.float_val = channel1B_out - meanChannel1B;
  runningMeanCh1bFil = runningMeanCh1bFil +(channel1B_out -channel1B_out)/(timeWindow*FLOAT_CON_16UA);      
  runningSquaredMeanCh1bFil = runningSquaredMeanCh1bFil +(channel1B_out - channel1B_out)*
    (channel1B_out - channel1B_out)/(FLOAT_CON_16UA*FLOAT_CON_16UA*timeWindow);
  ppgData1.infraRed_ch2 = buff_val_filtered.float_val;                            
  blePktPPG_Filter[4] = buff_val_filtered.floatcast[0];
  blePktPPG_Filter[5] = buff_val_filtered.floatcast[1];
  blePktPPG_Filter[6] = buff_val_filtered.floatcast[2];
  blePktPPG_Filter[7] = buff_val_filtered.floatcast[3];
        
  channel2A_in = led2A[0]*FLOAT_CON_16UA;
  arm_fir_f32(&S_chan2A_fil, &channel2A_in, &channel2A_out, blockSize);
  arm_fir_f32(&S_chan2AMA_fil, &channel2A_out, &meanChannel2A, blockSize);
  buff_val_filtered.float_val = channel2A_out - meanChannel2A;
  runningMeanCh2aFil = runningMeanCh2aFil +(channel2A_out - meanChannel2A)/(FLOAT_CON_16UA*timeWindow);         
  runningSquaredMeanCh2aFil = runningSquaredMeanCh2aFil +(channel2A_out - meanChannel2A)*
    (channel2A_out - meanChannel2A)/(timeWindow*FLOAT_CON_16UA*FLOAT_CON_16UA);
  ppgData1.green_ch1 = buff_val_filtered.float_val; 
  blePktPPG_Filter[8] = buff_val_filtered.floatcast[0];
  blePktPPG_Filter[9] = buff_val_filtered.floatcast[1];
  blePktPPG_Filter[10] = buff_val_filtered.floatcast[2];
  blePktPPG_Filter[11] = buff_val_filtered.floatcast[3];
        
  channel2B_in = led2B[0]*FLOAT_CON_16UA;
  arm_fir_f32(&S_chan2B_fil, &channel2B_in, &channel2B_out, blockSize);
  arm_fir_f32(&S_chan2BMA_fil, &channel2B_out, &meanChannel2B, blockSize);
  buff_val_filtered.float_val = channel2B_out - meanChannel2B;
  runningMeanCh2bFil = runningMeanCh2bFil +(channel2B_out -meanChannel2B)/(FLOAT_CON_16UA*timeWindow);		
  runningSquaredMeanCh2bFil = runningSquaredMeanCh2bFil +(channel2B_out - meanChannel2B)*
    (channel2B_out - meanChannel2B)/(FLOAT_CON_16UA*FLOAT_CON_16UA*timeWindow);
  ppgData1.green_ch2 = buff_val_filtered.float_val; 
  counterCheck++;
  if(counterCheck >timeWindow) 
    counterCheck =0;
  blePktPPG_Filter[12] = buff_val_filtered.floatcast[0];
  blePktPPG_Filter[13] = buff_val_filtered.floatcast[1];
  blePktPPG_Filter[14] = buff_val_filtered.floatcast[2];
  blePktPPG_Filter[15] = buff_val_filtered.floatcast[3];
  blePktPPG_Filter[16] = (pktCounter&0xFF00) >> 8;
  blePktPPG_Filter[17] = (pktCounter&0x00FF);  

  // Transmitting the un-filtered data on BLE 
  if(ppgConfig.txPacketEnable == true){
    my_ppgDataSensor.dataPacket = blePktPPG_noFilter;
    my_ppgDataSensor.packetLength = PPG_DATA_UNFILTER_LEN;
    k_work_submit(&my_ppgDataSensor.work);
  }
  if(ppgTFPass){
    ppgData1.green_ch1_buffer[ppgData1.bufferIndex] = ppgData1.green_ch1;
    ppgData1.green_ch2_buffer[ppgData1.bufferIndex] = ppgData1.green_ch2;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch1;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch2;
    ppgData1.bufferIndex = (ppgData1.bufferIndex +1 )%400;
    if(ppgData1.bufferIndex ==0 ){
      ppgData1.dataReadyTF = true;
    }
  }



  ppg_led_currentUpdate();
}



/* This function reads and fills bleSendArr with unfiltered ppg according
to the desired packet format */
void ppg_bluetooth_fill(uint8_t* bleSendArr){
  struct ppgInfo* the_device=  ((struct ppgInfo *)(((char *)(item)) 
    - offsetof(struct ppgInfo, work)));
  
  uint16_t pktCounter = the_device->pktCounter;
  bool movingFlag = the_device->movingFlag;
  bool ppgTFPass = the_device->ppgTFPass;
  uint8_t cmd_array[] = {PPG_CHIP_ID_1, WRITEMASTER, SPI_FILL};
  uint8_t read_array[128*2*2*3] = {0};
  uint8_t txLen,rxLen;
  uint32_t led1A[32];
  uint32_t led1B[32];
  uint32_t led2A[32];
  uint32_t led2B[32];
  uint8_t tag1A, tag1B, tag2A, tag2B, tag;
  float channel1A_in, channel1B_in, channel2A_in, channel2B_in;
  float meanChannel1A, meanChannel1B, meanChannel2A, meanChannel2B;

  float channel1A_out, channel1B_out, channel2A_out, channel2B_out;
  float_cast buff_val_filtered;
  uint32_cast buff_val_raw;
  int i;
  uint8_t j, k;
  uint8_t sampleCnt[5] = {0};
		
  // Reading the total number of PPG samples
  cmd_array[0] = PPG_FIFO_DATA_COUNTER;
  cmd_array[1] = READMASTER;
  txLen=3;
  rxLen=3;
  spiRead_registerPPG(cmd_array, txLen, sampleCnt, rxLen);
    
  // Reading the PPG samples
  cmd_array[0] = PPG_FIFO_DATA;

  
  spiRead_registerPPG(cmd_array, txLen, read_array,sampleCnt[2]*3+2);
  for (i=0; i<sampleCnt[2]; i++){
    for (j=0; j <= 9; j=j+3){
      tag = (read_array[i*12+j+2] & 0xF8) >>3;
      switch(tag){
        case PPG1_LEDC1_DATA: // IR 1
          led1A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG1_LEDC2_DATA: // Green 1
          led2A[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC1_DATA: // IR 2
          led1B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
        case PPG2_LEDC2_DATA: // Green 2
          led2B[i] = ((read_array[i*12+j+2]<<16) | (read_array[i*12+j+1+2]<<8)
           | (read_array[i*12+j+2+2])) & 0x7ffff;
          break;
      }
    }
    k = i;
    k = 0;
    tag1A = (read_array[k*12+0+2] & 0xF8) >>3;
    tag1B = (read_array[k*12+3+2] & 0xF8) >>3;
    tag2A = (read_array[k*12+6+2] & 0xF8) >>3;
    tag2B = (read_array[k*12+9+2] & 0xF8) >>3;
  }

  if(counterCheck ==0){
    runningMeanCh1a=0.0f;
    runningMeanCh1b=0.0f;
    runningMeanCh2a=0.0f;
    runningMeanCh2b=0.0f;
    runningMeanCh1aFil= 0.0f;
    runningMeanCh1bFil =0.0f;
    runningMeanCh2aFil =0.0f;
    runningMeanCh2bFil =0.0f;
    runningSquaredMeanCh1aFil= 0.0f;
    runningSquaredMeanCh1bFil =0.0f;
    runningSquaredMeanCh2aFil =0.0f;
    runningSquaredMeanCh2bFil =0.0f;
  }
  runningMeanCh1a = runningMeanCh1a +led1A[0]*1.0f/timeWindow;
  runningMeanCh1b = runningMeanCh1b +led1B[0]*1.0f/timeWindow;
  runningMeanCh2a = runningMeanCh2a +led2A[0]*1.0f/timeWindow;
  runningMeanCh2b = runningMeanCh2b +led2B[0]*1.0f/timeWindow;
		
  buff_val_raw.integer = led1A[0];
  blePktPPG_noFilter[0] = ((buff_val_raw.intcast[2]&0x07)<<5)|((buff_val_raw.intcast[1])&0xF8)>>3;
  blePktPPG_noFilter[1] = ((buff_val_raw.intcast[1]&0x07)<<5)|((buff_val_raw.intcast[0]&0xF8)>>3);
  blePktPPG_noFilter[2] =  (buff_val_raw.intcast[0]&0x07)<<5;
        
  buff_val_raw.integer = led1B[0];
  blePktPPG_noFilter[2] = blePktPPG_noFilter[2] | ((buff_val_raw.intcast[2]&0x07)<<2) |((buff_val_raw.intcast[1]&0xC0)>>6);
  blePktPPG_noFilter[3] = ((buff_val_raw.intcast[1]&0x3F)<<2) | ((buff_val_raw.intcast[0]&0xC0) >>6);
  blePktPPG_noFilter[4] = ((buff_val_raw.intcast[0]&0x3F) <<2) ;
        
  buff_val_raw.integer = led2A[0];
  blePktPPG_noFilter[4] = blePktPPG_noFilter[4] | ((buff_val_raw.intcast[2]&0x06)>>1);
  blePktPPG_noFilter[5] = ((buff_val_raw.intcast[2]&0x01)<<7) | ((buff_val_raw.intcast[1]&0xFE)>>1);
  blePktPPG_noFilter[6] = ((buff_val_raw.intcast[1]&0x01)<<7) | ((buff_val_raw.intcast[0]&0xFE)>>1);
  blePktPPG_noFilter[7] = ((buff_val_raw.intcast[0]&0x01)<<7);
        
  buff_val_raw.integer = led2B[0];
  blePktPPG_noFilter[7] = blePktPPG_noFilter[7] | ((buff_val_raw.intcast[2]&0x07) <<4)|((buff_val_raw.intcast[1]&0xF0) >>4);
  blePktPPG_noFilter[8] = ((buff_val_raw.intcast[1]&0x0F) <<4) | ((buff_val_raw.intcast[0]&0xF0)>>4);
  blePktPPG_noFilter[9] =  (buff_val_raw.intcast[0]&0x0F) <<4;
  blePktPPG_noFilter[10] = (pktCounter&0xFF00) >> 8;
  blePktPPG_noFilter[11] = (pktCounter&0x00FF);
   
  // Transmitting the un-filtered data on BLE 
  if(ppgConfig.txPacketEnable == true){
    
  }
  /*
  if(ppgTFPass){
    ppgData1.green_ch1_buffer[ppgData1.bufferIndex] = ppgData1.green_ch1;
    ppgData1.green_ch2_buffer[ppgData1.bufferIndex] = ppgData1.green_ch2;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch1;
    ppgData1.infraRed_ch1_buffer[ppgData1.bufferIndex] = ppgData1.infraRed_ch2;
    ppgData1.bufferIndex = (ppgData1.bufferIndex +1 )%400;
    if(ppgData1.bufferIndex ==0 ){
      ppgData1.dataReadyTF = true;
    }
  }
  */
  ppg_led_currentUpdate();

}




void fileOpen(){
  int rc = fs_open(&fileData, fname, FS_O_CREATE | FS_O_RDWR);
  if (rc < 0) 
    printk("FAIL: open %s: %d\n", fname, rc);

}
void fileOpenAppend(void){

  int rc = fs_open(&fileData, fname, FS_O_APPEND | FS_O_RDWR);
  if (rc < 0) 
    printk("FAIL: open for append %s: %d\n", fname, rc);

}

void fileClose(){
  int rc = fs_close(&fileData);
  getFileSysSize();
}
void fileOpenRead(){
  int rc = fs_open(&fileData, fname, FS_O_CREATE | FS_O_RDWR);
 
  if (rc < 0) 
    printk("FAIL: open for read %s: %d\n", fname, rc);
}
void fs_mount_init(){
  unsigned int id = (uintptr_t)lfs_storage_mnt.storage_dev;
  
  struct fs_statvfs sbuf;
  const struct flash_area *pfa;
  int rc;


  snprintf(fname, sizeof(fname), "%s/dataradar",lfs_storage_mnt.mnt_point);
  printk("%s is filename\n",fname);
  rc = flash_area_open(id, &pfa);
  if (rc < 0) {
    printk("FAIL: unable to find flash area %u: %d\n",id, rc);
    return;
  }

  printk("Area %u at 0x%x on %s for %u bytes\n",
         id, (unsigned int)pfa->fa_off, pfa->fa_dev_name,
         (unsigned int)pfa->fa_size);

  
  printk("Erasing flash area %d... ",id);
  rc = flash_area_erase(pfa, 0, pfa->fa_size);
  printk("%d\n", rc);


  flash_area_close(pfa);

  rc = fs_mount(&lfs_storage_mnt);
  if (rc < 0) {
    printk("FAIL: mount id %u at %s: %d\n",
           (unsigned int)lfs_storage_mnt.storage_dev, lfs_storage_mnt.mnt_point,
           rc);
    return;
  }
  printk("%s mount: %d\n", lfs_storage_mnt.mnt_point, rc);

  rc = fs_statvfs(lfs_storage_mnt.mnt_point, &sbuf);
  if (rc < 0) 
    printk("FAIL: statvfs: %d\n", rc);
  

  printk("%s: bsize = %lu ; frsize = %lu ;"
         " blocks = %lu ; bfree = %lu\n",
         lfs_storage_mnt.mnt_point,
         sbuf.f_bsize, sbuf.f_frsize,
         sbuf.f_blocks, sbuf.f_bfree);

  snprintf(fname, sizeof(fname), "%s/dataLevel",lfs_storage_mnt.mnt_point);

}
void getFileSysSize(void){
  struct fs_statvfs sbuf;
  int rc;

  rc = fs_statvfs(lfs_storage_mnt.mnt_point, &sbuf);
  if (rc < 0) 
    printk("FAIL: statvfs: %d\n", rc);
  

  printk("Flie system size after closing file %s: bsize = %lu ; frsize = %lu ;"
         " total blocks = %lu ; blocks free = %lu\n",
         lfs_storage_mnt.mnt_point,
         sbuf.f_bsize, sbuf.f_frsize,
         sbuf.f_blocks, sbuf.f_bfree);
}

void fs_umountFilesys(){
  int rc = fs_unmount(&lfs_storage_mnt);
  printk("%s unmount: %d\n", lfs_storage_mnt.mnt_point, rc);
  
  
  fs_mount_init();
}

void fileWrite(uint32_t dataFlash){
  int rc = fs_write(&fileData, &dataFlash, sizeof(dataFlash));
  
  if(rc<0){
    printk("error in file write %d\n",rc);
  }
}
uint32_t fileRead(void){
  uint32_t dataRead2;
  int rc = fs_read(&fileData, &dataRead2, sizeof(dataRead2));
  if(rc<0){
    printk("error in file write %d\n",rc);
  }
  return dataRead2;
}
