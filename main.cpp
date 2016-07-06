#include <pthread.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <mqueue.h>
#include <unistd.h>
#include <pigpio.h>
#include <signal.h>

#define qnSetPoint  "/setPoint"
#define qnReading  "/reading"
#define qnControlSignal  "/controlSignal"
#define qnInterruptSignal  "/controlSignal"

using namespace cv;
using namespace std;

void sig_handler(int sig)
{
	gpioSetMode(18, PI_OUTPUT);
	gpioTerminate();
}

void *procImage(void *arg)
{
	Mat frame, frameOpenClose;
	mqd_t  qReading, qReadingShow;         // descritores das filas
	Moments oMoments;
	int    msgReading=0;	// mensagens a enviar ou receber
	int iLowH = 35;
	int iHighH = 45;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 30;
	int iHighV = 255;

	if((qReading = mq_open(qnReading, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	VideoCapture cap(0);

	while(1)
	{
		bool bSuccess = cap.read(frame);
		if (!bSuccess)
			continue;

		//Acha posição da bolinha
		cvtColor(frame, frame, COLOR_BGR2HSV);
		inRange(frame, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), frame);
		//opening
		erode(frame, frameOpenClose, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		dilate(frameOpenClose, frameOpenClose, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		//closing
		dilate(frameOpenClose, frameOpenClose, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		erode(frameOpenClose, frameOpenClose, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		oMoments = moments(frameOpenClose);

		msgReading = oMoments.m01/oMoments.m00;

		mq_send(qReading, (char*) &msgReading, sizeof(int), 0);
	}
	cap.release();
}

void *control(void *arg)
{
	mqd_t  qSetPoint, qReading, qControlSignal;         // descritores das filas
	int    msgSetPoint=50, msgReading, msgControlSignal=40000;	// mensagens a enviar ou receber
	double read=0, setPoint=50;
	double kp, ki, kd, u=0, u1=0, e=0, e1=0, e2=0, b0, b1, b2, h1=0.08, h2=12;
	struct mq_attr attrNoBlock;				// atributos das filas de mensagens

	attrNoBlock.mq_maxmsg  = 4;			//capacidade para 4 mensagens
	attrNoBlock.mq_msgsize = sizeof(int);	// tamanho de cada mensagem
	attrNoBlock.mq_flags = O_NONBLOCK;
	if (qSetPoint = mq_open(qnSetPoint, O_RDWR|O_CREAT, 0666, &attrNoBlock) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	//Cria as filas a serem usadas pela Thread
	if((qReading = mq_open(qnReading, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	if((qControlSignal= mq_open(qnControlSignal, O_RDWR)) < 0)
	{
		perror ("mq_open ControlSignal");
		exit (1);
	}
	kp=2.5;
	ki=0.1;
	kd=1;

	b0=kp + ki*h1 + kd*h2;
	b1=ki*h1 - kp - 2*kd*h2;
	b2=kd*h2;

	mq_send(qControlSignal, (char*) &msgControlSignal, sizeof(int), 0);
	for(int i=0; i<=10; i++) {
		mq_receive(qReading, (char*) &msgReading, sizeof(int), 0);	//Espera pela leitura da altura
	}
	msgControlSignal=13700;

	while(1)
	{
		if(mq_receive(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0) > -1) {
			setPoint = msgSetPoint;
		}//Verifica se recebeu setpoint
		mq_receive(qReading, (char*) &msgReading, sizeof(int), 0);	//Espera pela leitura da altura
		//Controle
		read = (double) msgReading;
		read = (480 - read)/4.8;
		e = setPoint - read;
		u = e*b0 + e1*b1 + e2*b2 + u1;
		if(u < -200) {
			u=-200;
		}
		else if(u > 200) {
			u=200;
		}
		msgControlSignal = (int) (u*20)+13700;
		printf("u=%10.4f read=%4.4f e=%10.4f\n",u, read, e);
		mq_send(qControlSignal, (char*) &msgControlSignal, sizeof(int), 0);
		e2 = e1;
		e1 = e;
		u1 = u;
	}
}

void *PWM(void *arg)
{
	mqd_t qControlSignal;
	int msgControlSignal;

	if((qControlSignal= mq_open(qnControlSignal, O_RDWR)) < 0)
	{
		perror ("mq_open ControlSignal");
		exit (1);
	}

	gpioInitialise();
	gpioSetMode(18, PI_OUTPUT);
	gpioSetSignalFunc(SIGINT, &sig_handler);

	while(1)
	{
		mq_receive(qControlSignal, (char*) &msgControlSignal, sizeof(int), 0);
		//cout << msgControlSignal << endl;
		//Seta PWM para valor calculado na Thread de controle
		gpioHardwarePWM(18, 1000, msgControlSignal*10);
	}
}

void *setPoint(void *arg)
{
	mqd_t qSetPoint;
	int msgSetPoint=50;

	if((qSetPoint = mq_open(qnSetPoint, O_RDWR)) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	while(1)
	{
		//cout << "Digite a altura desejada da bolinha." << endl;
		//cin >> msgSetPoint;
		//mq_send(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0);
	}
}

int main(int argc, char** argv )
{
	pthread_t tProcImage, tControl, tSetPoint, tPWM;
	int msgStop;
	long status;
	struct mq_attr attr, attrNoBlock, attrSize1;				// atributos das filas de mensagens
	mqd_t qInterruptSignal;


	//Cria as filas a serem usadas pela Thread
	attr.mq_maxmsg  = 4;			//capacidade para 4 mensagens
	attr.mq_msgsize = sizeof(int);	// tamanho de cada mensagem
	attr.mq_flags   = 0;

	if (mq_open(qnReading, O_RDWR|O_CREAT, 0666, &attr) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	if (mq_open(qnControlSignal, O_RDWR|O_CREAT, 0666, &attr) < 0)
	{
		perror ("mq_open ControlSignal");
		exit (1);
	}

	attrNoBlock.mq_maxmsg  = 4;			//capacidade para 4 mensagens
	attrNoBlock.mq_msgsize = sizeof(int);	// tamanho de cada mensagem
	attrNoBlock.mq_flags = O_NONBLOCK;
	if (mq_open(qnSetPoint, O_RDWR|O_CREAT, 0666, &attrNoBlock) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	//Cria as Threads
	status = pthread_create(&tProcImage, NULL, procImage, NULL);
	if (status) {
		perror("pthread_create procImage");
		exit (1) ;
	}

	status = pthread_create(&tControl, NULL, control, NULL);
	if (status) {
		perror("pthread_create procImage");
		exit (1) ;
	}

	status = pthread_create(&tSetPoint, NULL, setPoint, NULL);
	if (status) {
		perror("pthread_create procImage");
		exit (1) ;
	}

	status = pthread_create(&tPWM, NULL, PWM, NULL);
	if (status) {
		perror("pthread_create procImage");
		exit (1) ;
	}

	pause();
	return 0;
}
