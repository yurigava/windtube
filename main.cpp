#include <pthread.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <mqueue.h>
#include <unistd.h>
#include <pigpio.h>

#define qnSetPoint  "/setPoint"
#define qnReading  "/reading"
#define qnControlSignal  "/controlSignal"

using namespace cv;
using namespace std;


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

	int iLowV = 180;
	int iHighV = 255;

	if((qReading = mq_open(qnReading, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	VideoCapture cap(0);
	namedWindow("MyImage", WINDOW_AUTOSIZE);

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
		dilate(frame, frameOpenClose, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

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
	int    msgSetPoint=50, msgReading, msgControlSignal;	// mensagens a enviar ou receber
	double read=0, setPoint=50;
	double kp=1, ki=1, kd=1, u=0, u1=0, e=0, e1=0, e2=0, b0, b1, b2, h1=0.05, h2=10;

	//Cria as filas a serem usadas pela Thread
	if((qReading = mq_open(qnReading, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	if((qSetPoint = mq_open(qnSetPoint, O_RDWR)) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	if((qControlSignal= mq_open(qnControlSignal, O_RDWR)) < 0)
	{
		perror ("mq_open ControlSignal");
		exit (1);
	}

	b0=kp + ki*h1 + kd*h2;
	b1=ki*h1 - kp - 2*kd*h2;
	b2=kd*h2;

	while(1)
	{
		mq_receive(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0);//Verifica se recebeu setpoint
		mq_receive(qReading, (char*) &msgReading, sizeof(int), 0);	//Espera pela leitura da altura
		//Controle
		setPoint = msgSetPoint;
		read = (double) (480 - msgReading)/4.8;
		e = setPoint - read;
		u = e*b0 + e1*b1 + e2*b2 + u1;
		msgControlSignal = (int) u;
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

	while(1)
	{
		mq_receive(qControlSignal, (char*) &msgControlSignal, sizeof(int), 0);
		cout << msgControlSignal << endl;
		//Seta PWM para valor calculado na Thread de controle
	}
}

void *setPoint(void *arg)
{
	mqd_t qSetPoint, qReadingShow;
	int msgSetPoint=0, msgReadingShow;

	if((qSetPoint = mq_open(qnSetPoint, O_RDWR)) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	while(1)
	{
		//cout << "Digite a altura desejada da bolinha." << endl;
		//cin >> msgSetPoint;
		mq_send(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0);
	}
}

int main(int argc, char** argv )
{
	pthread_t tProcImage, tControl, tSetPoint, tPWM;
	long status;
	struct mq_attr attr, attrNoBlock;				// atributos das filas de mensagens

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
	if (mq_open(qnSetPoint, O_RDWR|O_CREAT, 0666, &attr) < 0)
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

	gpioInitialise();
	gpioSetMode(17, PI_OUTPUT);
	gpioPWM(17, 10);

	while(1);
}
