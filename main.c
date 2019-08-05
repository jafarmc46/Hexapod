/*
 * Bismillahirrohmaanirrohiim
 * ABINARA-1 KRPAI ITS JUARA NASIONAL
 * Doa Usaha Tawakal Tetap Semangat
 * Sesungguhnya bersama kesulitan ada kemudahan
 *
 */
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_dma.h>
#include <misc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "delay.h"
#include "lcd_biasa.h"
#include "invers.h"
#include "RX28.h"
#include "pengolahanKakiRX28.h"
#include "motion.h"
#include "interruptTxUsart.h"
#include "ADC_DMA2.h"

//#define out1 TIM3->CCR1
#define out2 TIM4->CCR2
//#define out3 TIM3->CCR3
//#define out4 TIM3->CCR4

#define tes_co 1500
#define tutup_co 800
#define buka_co 2200

GPIO_InitTypeDef cio;
USART_InitTypeDef cusart;
NVIC_InitTypeDef nvic;
TIM_TimeBaseInitTypeDef tim;
TIM_OCInitTypeDef pwm;

#define sin20 0.34
#define sin10 0.17

//=======================TOMBOL================================//
#define start_on 			(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)==0)
#define tombol2_on			(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==0)
#define tombol1_on			(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)
#define tombol3_on			(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14)==0)

//=======================POMPA================================//
//#define pompa_on	(GPIO_WriteBit(GPIOE,GPIO_Pin_14,1))
//#define pompa_off	(GPIO_WriteBit(GPIOE,GPIO_Pin_14,0))

//=======================SOUND ACTIVATOR========================//
//#define sound_on 	(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)==0)

//=======================UV-TRON========================//
#define uv1_off 		(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)
#define uv1_on 	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)
//#define uv2_on 		(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==0) //peka
//#define uv2_off 	(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==1) //peka

//=======================LED API==============================//
#define led_api_on	(GPIO_WriteBit(GPIOD,GPIO_Pin_14,1))
#define led_api_off	(GPIO_WriteBit(GPIOD,GPIO_Pin_14,0))

//=======================LED START/SOUND======================//
#define led_sound_on	(GPIO_WriteBit(GPIOD,GPIO_Pin_12,1))
#define led_sound_off	(GPIO_WriteBit(GPIOD,GPIO_Pin_12,0))

#define kalibrasi_utara		15 //cmps kalibrasi
#define kalibrasi_selatan	227
#define kalibrasi_barat		303
#define kalibrasi_timur		120

char tampil[16];
int i;
int penanda_ganti_muka;
int posisi_anjing;
int penanda_buzzer;
int status_api = 0;
int eksistansi_api2;
int eksistansi_api1;
int ruang_api;
int ruang_start;
int state_jalan;
int next_state_jalan;
int arah_deteksi_lost;
int state_kembalidarilost;
int langkahScan;
int ruangScan;
int konversi_scan;
int konversi_loncat;
int keluar_kanan;
int keluar_kiri;
int penanda205;
int penanda_keluar_4a_4b_1b;
int penanda_keluar_1a;

int pewaktu;

char buffer_lcd[100];
float error0, error1, error2, error1s, error2s, error3s, rotasij, rotasii;
int state_telusur, state_serong, state_sebelum, state_lanjut;
int lock_state_telusur;
int rotasi, rotasis;
int selisih_serong, regresi_samping, batas_samping;
float sudutBelok, kecepatan, langkah, tinggi;
float geser;

float kape1 = 5.504, kape2 = 0.045;
float deriv1 = 0.22, deriv2 = 0.1;
float p_rot1 = 0, p_rot2 = 0;
float deriv_rot1 = 0, deriv_rot2 = 0;

int ganti_muka;
int ganti_kanan;
int ganti_serong;
int ganti_kiri;
int setPoint0z = 200; //160
int setPoint1z = 25; //120
int setPoint2z = 5; //90
int setPointS = 12;

//KP2z=0.045/0.02, KP1z=0, KPrz=0.014/0.008, KIrz=0;
float KP2z = 0.02, KP1z = 0, KPrz = 0.002, KIrz = 0;
//KDz=0.1,KDrz=0;
float KDz = 0.2, KDrz = 0 ;
float Kp_rot =5.054 , Kd_rot=0.13;
//kontrol telusur ruangan
float KP2zR = 0.04, KP1zR = 0.01, KPrzR = 0.02, KP1rzR = 0.0001;
float KDzR = 0.1, KDrzR = 0.1; //KDz=5,KDrz=15;

#define UTARA	0
#define BARAT	1
#define TIMUR	2
#define	SELATAN	3

#define DEADZONE 4
#define SEARAH_JARUM_JAM	0
#define LAWAN_JARUM_JAM		1

int mataangin;
int upper[4];
int lower[4];
int bataskhusus_up[4];
int bataskhusus_low[4];
int aktivasi_up[4];
int aktivasi_low[4];
int atas[4];
int bawah[4];

int khusus = 330;

int lock_acuan;

int sudah_lurus;
int setcmps;
int setOrientasi;
int orientasi_Now;
int fungsi_orientasi;
int gunakan_cmps = 1;
int change;

int kan = 1, kir = 0;
int thd_tpa = 35, state_tpa = 0, posisi_tpa = 0;

void inisialisasiIO()
{
	GPIO_InitTypeDef configIo;
	//buzzer
	configIo.GPIO_Pin = GPIO_Pin_4;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Mode = GPIO_Mode_OUT;
	configIo.GPIO_PuPd = GPIO_PuPd_NOPULL;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &configIo);

	//tombol start
	configIo.GPIO_Mode = GPIO_Mode_IN;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Pin = GPIO_Pin_14;
	configIo.GPIO_PuPd = GPIO_PuPd_UP;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &configIo);

	//tombol1
	configIo.GPIO_Mode = GPIO_Mode_IN;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Pin = GPIO_Pin_10;
	configIo.GPIO_PuPd = GPIO_PuPd_UP;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &configIo);

	//tombol2
	configIo.GPIO_Mode = GPIO_Mode_IN;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Pin = GPIO_Pin_12;
	configIo.GPIO_PuPd = GPIO_PuPd_UP;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &configIo);

	//tombol3
	configIo.GPIO_Mode = GPIO_Mode_IN;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Pin = GPIO_Pin_14;
	configIo.GPIO_PuPd = GPIO_PuPd_UP;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &configIo);

//	//UV-TRON 1
	configIo.GPIO_Mode = GPIO_Mode_IN;
	configIo.GPIO_OType = GPIO_OType_PP;
	configIo.GPIO_Pin = GPIO_Pin_15;
	configIo.GPIO_PuPd = GPIO_PuPd_UP;
	configIo.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &configIo);
}

void inisialisasi_timer6() //Cartesian Trajectory Planning
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	tim.TIM_Prescaler = 84;//2.4ms, yg lama itu 80ms
	tim.TIM_Period = 40000-1;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &tim);

	TIM_ARRPreloadConfig(TIM6, ENABLE);

	TIM_Cmd(TIM6, ENABLE);

	nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	//definisi_NVIC.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&nvic);
	//
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void initBrush()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitTypeDef definisi_pin;

	definisi_pin.GPIO_Pin = GPIO_Pin_13;
	definisi_pin.GPIO_Mode = GPIO_Mode_AF;
	definisi_pin.GPIO_OType = GPIO_OType_PP;
	definisi_pin.GPIO_Speed = GPIO_Speed_50MHz;
	definisi_pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &definisi_pin);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

	TIM_OCInitTypeDef definisi_OC;
	TIM_TimeBaseInitTypeDef definisi_timebase;

	definisi_timebase.TIM_Prescaler = 83;
	definisi_timebase.TIM_Period = 20000-1 ;
	definisi_timebase.TIM_ClockDivision = TIM_CKD_DIV1;
	definisi_timebase.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM4, &definisi_timebase);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	definisi_OC.TIM_OCMode = TIM_OCMode_PWM1;
	definisi_OC.TIM_OutputState = TIM_OutputState_Enable;
	definisi_OC.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM4, &definisi_OC);

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	out2 = 800;//900-2000
}

unsigned char penanda, data_ke, data_slave[23];
void inisialisasi_USART1() //USART Atmega
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

		  /* Configure USART6 pins: Tx ----------------------------*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&data_slave;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(data_slave);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	//USART_DMACmd(UART5, USART_DMAReq_Tx, DISABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

	  /* Enable USART1 IRQ */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream5, ENABLE);
	USART_Cmd(USART1, ENABLE);

	//	  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	//
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

char dataKompas, sign, dataSudut[3], *join_sudut;
int flag_data, kompas, statusKompas, statusKalibrasi, kalibrasi;

int sensor_gp[8];
unsigned char rx_buffer[4], rx_index;
unsigned char dataSlave;
unsigned char data_srf[8], data_api, data_max_api, data_min_api, data_cmpsL, data_cmpsH, tpa[8];
int nilai_api, data_cmpsgabung, posisi_api;

void USART1_IRQHandler(void) //USART Slave
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE))
	{
		dataSlave = USART_ReceiveData(USART1);
		if(penanda == 0 && dataSlave == 'x')
		{
			penanda = 0;
			USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		}
		else penanda = 0;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void DMA2_Stream5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
	{
//		GPIO_WriteBit(GPIOD, GPIO_Pin_15,Bit_SET);

		USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
	}
}

int thd_api = 150;
int deteksi_api(int arah)
{
	for (i = 0; i < arah; i++)
	{
		if (uv1_on || nilai_api > thd_api)
		{
			eksistansi_api1 = 1;
			break;
		}
		else eksistansi_api1 = 0;
		delay_ms(1);
	}
	return eksistansi_api1;
}

int api_siap;
int setPoint_api = 24;
int error_api;
float sudutBelok_api;
float sudutBelok_api_statis;
int putar_api;
int PID_api()
{
	if (posisi_api == 8)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 30);
		api_siap = 1;
	}

	else if ((posisi_api > 8&&posisi_api <= 15)&&nilai_api > 220)
	{
		gerak(0, 0, -9, 1, 25);
		putar_api = 1;
	}
	else if ((posisi_api<8 && posisi_api>=0)&&nilai_api > 220)
	{
		gerak(0, 0, 9, 1, 25);
		putar_api = 0;
	}
	else
	{
		if (putar_api == 0)
		{
			gerak(0, 0, 9, 1, 25);
		}
		else if (putar_api == 1)
		{
			gerak(0, 0, -9, 1, 25);
		}
	}

	return api_siap;
}

int putar_kepala_semprot;
int count_pompa;
void padamkan_api()
{
	out2 = buka_co;
	while (deteksi_api(5) == 1 || count_pompa < 2) //&&
	{
		gerakStatic(0, 0, 0, -10, 0, 0, 10);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1 || statusGerak[4] == 1 || statusGerak[5] == 1)
			;

		gerakStatic(0, 0, 0, 10, 0, 0, 10);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1 || statusGerak[4] == 1 || statusGerak[5] == 1)
			;
		count_pompa++;
	}
	eksistansi_api1 = 0;
	count_pompa = 0;
	out2 = tutup_co;
	penanda_buzzer = 0;
	gerakStatic(0, 0, 0, 0, 0, 0, 10);
	delay_ms(2000); //9000
}

void cobaPosisi()
{
	RX_move(1,3148,1023);
	RX_move(2,2048,512);
	RX_move(3,2648,512);

	RX_move(4,2048,1023);
	RX_move(5,2048,512);
	RX_move(6,2048,512);

	RX_move(7,2048,1023);
	RX_move(8,2048,512);
	RX_move(9,1048,512);

	RX_move(10,2048,1023);
	RX_move(11,2048,512);
	RX_move(12,2048,512);

	RX_move(13,2048,1023);
	RX_move(14,2048,512);
	RX_move(15,2048,512);

	RX_move(16,2048,1023);
	RX_move(17,2048,512);
	RX_move(18,2048,512);
}

void cobaKalibrasi()
{
	//0
	RX_move(1,512,512);//kanan depan luar //512 +masuk -keluar 450-391
	delay_ms(100);
	RX_move(2,512,512);//512 +atas -bawah 374-300
	delay_ms(100);
	RX_move(3,512,512);//512 +kanan -kiri 512-768
	delay_ms(100);
	//1
	RX_move(4,512,512);//kanan belakang luar 512 391-450
	delay_ms(100);
	RX_move(5,512,512);//512 372-298
	delay_ms(100);
	RX_move(6,512,512);//512 512-256
	delay_ms(100);
	//2
	RX_move(10,512,512);//kiri depan luar 512 630-571
	delay_ms(100);
	RX_move(11,512,512);//512 654-728
	delay_ms(100);
	RX_move(12,512,512);//512 512-256
	delay_ms(100);
	//3
	RX_move(7,512,512);//kiri belakang luar 512 395-454
	delay_ms(100);
	RX_move(8,512,512);//512 375-301
	delay_ms(100);
	RX_move(9,512,512);//512 512-768
	delay_ms(100);
}

void TIM6_DAC_IRQHandler()
{
//	if(TIM_GetITStatus(TIM6,TIM_IT_Update))//tes timer
//	{
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_7);
//	}
	trajectoriLinier(speedServo);
	kirimInstruksiGerak(1023);
//	cobaKalibrasi();
//	cobaPosisi();
	if (penanda_buzzer == 0)(GPIO_SetBits(GPIOE, GPIO_Pin_4));
	else if (penanda_buzzer >= 5)(GPIO_ResetBits(GPIOE, GPIO_Pin_4));
	penanda_ganti_muka++;
	pewaktu++;
	penanda_buzzer++;
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void maju(int arah)//langkah kecepatan default_servo
{
	kecepatan = 20; //40
	langkah = 1; //3
	tinggi = -1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
//	PIDkr();
	if (arah == 2)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
	else if (arah == 0)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
//	penanda_ganti_muka = 0;
}

void maju_serong(int arah)//langkah kecepatan default_servo
{
	kecepatan = 30; //40
	langkah = 1.5; //3
	tinggi = -1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
	//PIDkr();
	if (arah == 2)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3)gerak(langkah, 0, sudutBelok+0.3, tinggi, kecepatan);
	else if (arah == 0)gerak(0, -langkah, sudutBelok-0.5, tinggi, kecepatan);
	else if (arah == 1)gerak(-langkah, 0, sudutBelok+1, tinggi, kecepatan);
//	penanda_ganti_muka = 0;
}

void maju_kompas(int arah, int acuan)//langkah kecepatan default_servo
{
//	head(arah);
	kecepatan = 50; //40 //60 //80
	langkah = 2; //3 //2.5 //2.3
	tinggi = -1; //-2
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
//	setPointK = acuan;
//	PIDk();
	if (arah == 2)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
	else if (arah == 0)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
//	penanda_ganti_muka = 0;
}

void jalan_langkah(int arah, int jumlah_langkah)//langkah kecepatan default_servo
{
	kecepatan = 30; //40
	langkah = 1.5; //3
	tinggi = -1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
	//PIDkr();
	hitung_langkah = 0;
	while(1)
	{
		if (arah == 2)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(langkah, 0, sudutBelok+0.3, tinggi, kecepatan);
		else if (arah == 0)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		if(hitung_langkah == jumlah_langkah)break;
	}
	hitung_langkah = 0;
	//state_telusur=arah;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void geserKanan(int arah)
{
	kecepatan = 30; //40
	langkah = 1.5; //3
	tinggi = -1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
	//PIDkr();
	if (arah == 1)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2)gerak(langkah, 0, sudutBelok+0.8, tinggi, kecepatan);
	else if (arah == 3)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 0)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
//	penanda_ganti_muka = 0;
}

void geserKiri(int arah)
{
	kecepatan = 20; //40
	langkah = 1; //3
	tinggi = -1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
	//PIDkr();
	if (arah == 3)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 0)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
	else if (arah == 1)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
//	penanda_ganti_muka = 0;
}

void serong(int arah, int arah_serong)//-1kanan 1kiri
{
	kecepatan = 20; //60.2
	langkah = 1; //3
	tinggi = -1;
//	state_telusur = arah;

//	if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
//	else sudutBelok = 0;

	if (arah == 0&&arah_serong == -1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == -1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == -1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == -1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);

	else if (arah == 0&&arah_serong == 1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == 1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == 1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == 1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
}

void muter_(int arah, int sudut, int kec)
{
	if(arah==0){//kiri
		gerak(0, 0, -sudut, -1, kec);
	}
	if(arah==1){//kanan
		gerak(0, 0, sudut, -1, kec);
	}
}

void putar_biasa(int sudutputar)
{
	int i;
	float arah_putar;
	if(sudutputar==90){i=40;arah_putar=6;}
	else if (sudutputar==180){i=80;arah_putar=6;}
	else if (sudutputar==270){i=120;arah_putar=6;}
	else if (sudutputar==-90){i=40;arah_putar=-6;}
	else if (sudutputar==-180){i=80;arah_putar=-6;}
	else if (sudutputar==-270){i=120;arah_putar=-6;}
	else if (sudutputar==360){i=155;arah_putar=6;}

	if(pewaktu>=i)goto label;
	pewaktu=0;
	while(pewaktu<i)
	{
		gerak(0,0,arah_putar,1,25);
	}
	label:
	gerakStatic(0,0,0,0,0,0,30);
}

void berhenti()
{
	gerakStatic(0, 0, 0, 0, 0, 0, 30);
}

void buzzer()
{
	int i = 0;
	for (i = 0; i < 30; i++)
	{
		GPIO_ToggleBits(GPIOE, GPIO_Pin_4);
		delay_ms(30);
	}
}

float KP=2, TI=0, TD=0.1, error_sblmI=0, error_sblmD=0;
float set_point=0, Tc=0.01, error, errorI, errorD, outP, outI, outD;
void telusurKanan()
{
	kecepatan = 25; //70 40 25
	langkah = 1; //1.5
	tinggi = -1;
	//statusKompas = 0;
	switch(state_telusur)
	{
	case 0://muka0
		selisih_serong = (data_slave[2] - data_slave[0]);

		error = set_point - selisih_serong;//sp=6

		if (error >= 4)error = 4;
		if (error <= -4)error = -4;

		sudutBelok = (KP/10)*error  + (TD/10)*(error - error_sblmD);
		error_sblmD = error;

		if (sudutBelok >= 2.3)sudutBelok = 2.3; //15.2
		if (sudutBelok <= -2.3)sudutBelok = -2.3;

		if(data_slave[3]<10)
		{
			state_telusur=1;
			hitung_zigzag = 0;
			hitung_langkah = 0;
		}

		gerak(0, -langkah, -sudutBelok, tinggi, kecepatan);
		lcd_gotoxy(0,0);
		sprintf(tampil,"%2d %.0f %.1f",selisih_serong,error,sudutBelok);
		lcd_puts(tampil);
		break;
	case 1:
		muter_(0,4,20);
		if(hitung_langkah > 7)state_telusur = 0;
		break;
	}
}

int PID_tpa()
{
	int i;
	for(i=7;i<23;i++)
	{
		if(data_slave[i]<50)posisi_tpa=i;
	}
	if(posisi_tpa==15||14)
	{
		berhenti();
		api_siap=1;
	}
	else if(posisi_tpa>15)
	{
		putar_api=1;
	}
	else if(posisi_tpa<14)
	{
		putar_api=0;
	}
	else
	{
		if(putar_api==1)
		{
			muter_(kir,5,20);
		}
		else if(putar_api==0)
		{
			muter_(kan,5,20);
		}
	}
	return api_siap;
}
/*
 * Variable Ruang Start
 * 1A	=	1
 * 2	=	2
 * 3	=	3
 * 4A	=	4
 * 1B	=	5
 * 1C	=	6
 * 4B	=	7
 */

int count_penanda_dinding2; //penanda dinding dekat anjing 2
int count_penanda_anjing3;
int count_penanda_anjing1;
int penanda_api_1; // 0 jika api di dinding luar, 1 jika api di dinding dalam
int penanda_khusus_1a;
int penanda_khusus_2;
int penanda_khusus_2api;
int penanda_telusur_api;
int penanda_pintu1; //0 = 1a, 1=1b
int state_start;
int penanda_langkah;
int thd_dinding = 400;
int batas_serong;

int count;
int count_pompa;
int count_ceper;
int menu;
int count_putar_kalibrasi;
int count_garis_tinggi;

void jalan()
{
	lcd_gotoxy(0, 0);
	lcd_puts("<<<BISMILLAH>>>");
	lcd_gotoxy(0, 1);
	sprintf(tampil, "State : %4d %d", state_jalan, posisi_tpa);
	lcd_puts(tampil);
	switch(state_jalan)
	{
		case 0:
			telusurKanan();
			if(uv1_on)
			{
				penanda_buzzer = 0;
				state_jalan = 1;
			}
			break;
		case 1:
			muter_(0,5,20);
			if(data_slave[15]<50)state_jalan = 2;
			break;
		case 2:
			padamkan_api();
			state_jalan = 3;
			eksistansi_api1 = 0;
			status_api = 1;
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			break;
		case 3:
			if (deteksi_api(5) == 1)
			{
				state_jalan = 2;
				penanda_buzzer = 0;
				break;
			}
			else
			{
				state_jalan = 4;
			}
			break;
		case 4:
			berhenti();
			break;
	}
}

void start()
{
	if ((start_on)&&state_start != 3)
	{
		while (start_on)
			;
		if (menu != 2)
		{
			state_start = 3;
			menu = 8;
			lcd_clear();
			pewaktu = 0;
			led_sound_on;
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1 || statusGerak[5] == 1 || statusGerak[5] == 1)
				;
		}
	}
	else if (tombol2_on)
	{
		while (tombol2_on)
		{
			count_ceper++;
			delay_ms(10);
			if (count_ceper > 300)
			{
//				maju_speed(2,10);
				lcd_gotoxy(0,0);
				lcd_puts("langkah kecil");
				lcd_clear();
			}
		}

		if (state_start == 1&&count_ceper <= 300)
		{
			state_start = 5;
			lcd_clear();
		}
		else if (state_start == 5&&count_ceper <= 300)
		{
			state_start = 1;
			lcd_clear();
		}
		else
		{
			menu++;
			if (menu > 3)menu = 0;
			lcd_gotoxy(0,0);
			lcd_puts("set kompas");
			lcd_clear();
//			setKompas();
		}
		count_ceper = 0;
	}
	else if (tombol1_on)
	{

		while (tombol1_on)
		{
			count_putar_kalibrasi++;
			delay_ms(10);
			if (count_putar_kalibrasi > 300)
			{
//				gerakStatic(0, 0, 0, 0, 0, 0, 30);
//				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
//					;
//				putar(360, 9, 25);
//				gerakStatic(0, 0, 0.7, 0, 0, 0, 30);
//				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
//					;
				pewaktu=0;
				lcd_clear();
				lcd_gotoxy(0,0);
				lcd_puts("Putar kalib api");
				putar_biasa(360);
			}
		}
		count_putar_kalibrasi = 0;

		if (state_start == 1)
		{
			state_start = 4;
			lcd_clear();
		}
		else if (state_start == 4)
		{
			state_start = 1;
			lcd_clear();
		}
		else
		{
			menu--;
			if (menu < 0)menu = 3;
			lcd_clear();
//			setKompas();
		}
	}
	else if (tombol3_on)
	{
		while (tombol3_on)
		{
			count_pompa++;
			delay_ms(10);
			if (count_pompa > 300)
			{
//				pompa_on;
				lcd_clear();
				lcd_gotoxy(0,0);
				lcd_puts("Pompa on");
			}
		}
//		pompa_off;
		lcd_clear();
		lcd_gotoxy(0,0);
		lcd_puts("Pompa off");
		count_pompa = 0;
		if (state_start == 1)state_start = 2;
		else if (state_start == 2)state_start = 1;
		else if (state_start == 5)state_start = 1;

		lcd_clear();
	}
	switch (state_start)
	{
	case 0 :
//		if (kalibrasi == 1)
//		{
//			statusKalibrasi = 0;
			state_start = 1;
//			setKompas();
			buzzer();
//		}
		break;
	case 1 :
		lcd_gotoxy(0, 0);
		lcd_puts("  <<<BISMILLAH!>>>");
		lcd_gotoxy(0, 1);
		lcd_puts("     <HEXA MRI>");
		break;
	case 2:
		switch (menu)
		{
		case 0 :
			lcd_gotoxy(0, 0);
			lcd_putsf("SRF");
			lcd_gotoxy(0, 1);
			sprintf(tampil,"%3d %3d %3d %3d",data_slave[0],data_slave[1],data_slave[2],data_slave[3]);
			lcd_puts(tampil);
			lcd_gotoxy(0, 2);
			sprintf(tampil,"%3d %3d %3d",data_slave[4],data_slave[5],data_slave[6]);
			lcd_puts(tampil);
			break;
		case 1 :
			lcd_gotoxy(0, 0);
			lcd_putsf("PHOTO");
			lcd_gotoxy(7, 0);
			sprintf(tampil,"%3d",data_slave[7]);
			lcd_puts(tampil);
			lcd_gotoxy(0, 1);
			sprintf(tampil,"%3d %3d %3d %3d %3d",data_slave[8],data_slave[9],data_slave[10],data_slave[11],data_slave[12]);
			lcd_puts(tampil);
			lcd_gotoxy(0, 2);
			sprintf(tampil,"%3d %3d %3d %3d %3d",data_slave[13],data_slave[14],data_slave[15],data_slave[16],data_slave[17]);
			lcd_puts(tampil);
			lcd_gotoxy(0, 3);
			sprintf(tampil,"%3d %3d %3d %3d %3d",data_slave[18],data_slave[19],data_slave[20],data_slave[21],data_slave[22]);
			lcd_puts(tampil);
			break;
		case 2 :
			lcd_gotoxy(0, 0);
			lcd_puts("UV");
			if (uv1_on)
			{
				(GPIO_SetBits(GPIOE, GPIO_Pin_4)); //buzzer on
				led_api_on;
			}
			else
			{
				(GPIO_ResetBits(GPIOE, GPIO_Pin_4)); //buzzer off
				led_api_off;
			}
			lcd_gotoxy(0, 1);
			sprintf(tampil, "UV1 :%1d", GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15));
			lcd_puts(tampil);
			break;
		case 3:
			lcd_gotoxy(0, 0);
			lcd_puts("Cek Servo CO");
			while (tombol2_on)
			{
				count_pompa++;
				delay_ms(10);
				if (count_pompa > 300);out2 = buka_co;
			}
			out2 = tutup_co;
			count_pompa = 0;
			break;
		}
		break;
	case 3 ://ganteng
		//maju(0);
		//telusurKanan();
		//putar_biasa(180);
		jalan();
		//padamkan_api();
		break;
	case 4 :
//		lcd_clear();
//		lcd_gotoxy(0, 0);
//		lcd_puts("KALIBRASI GARIS");
//		kalibrasi_garis();
		break;
	case 5 :
//		lcd_clear();
//		gerak(0, 1, 0, -1, 10);
		break;
	}
}

int main(void)
{
	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //enable interrupt
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	inisialisasiIO();
	lcd_init(20);
	lcd_clear();
	initBrush();
	inisialisasiRX28();//AX servo
	inisialisasiPosisiKaki();
	inisialisasi_timer6();//Cartesian Trajectory Planning
	inisialisasi_USART1(); //USART Slave
	//berhenti();

	buzzer();
	delay_ms(300);
	while (1)
	{
		start();
		//cobaPosisi();
		//RX_move(1,2048,512);
		//lcd_gotoxy(0,0);
		//lcd_puts("coba");
	}
}
