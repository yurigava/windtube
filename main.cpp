#include <pthread.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <mqueue.h>
#include <unistd.h>

#define qnSetPoint  "/setPoint"
#define qnReading  "/reading"
#define qnReadingShow  "/readingShow"
#define qnControlSignal  "/controlSignal"

using namespace cv;
using namespace std;


void *procImage(void *arg)
{
	Mat frame;
	mqd_t  qReading, qReadingShow;         // descritores das filas
	int    msgReading;	// mensagens a enviar ou receber

	if((qReading = mq_open(qnReading, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	if((qReadingShow = mq_open(qnReadingShow, O_RDWR)) < 0)
	{
		perror ("mq_open ReadingShow");
		exit (1);
	}

	VideoCapture cap(0);

	while(1)
	{
		bool bSuccess = cap.read(frame);
		if (!bSuccess)
		{
			break;
		}

		//Acha posição da bolinha

		msgReading = 0;
		mq_send(qReading, (char*) &msgReading, sizeof(int), 0);
		mq_send(qReadingShow, (char*) &msgReading, sizeof(int), 0);
	}
	cap.release();
}

void *control(void *arg)
{
	mqd_t  qSetPoint, qReading, qControlSignal;         // descritores das filas
	int    msgSetPoint, msgReading, msgControlSignal;	// mensagens a enviar ou receber

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

	while(1)
	{
		mq_receive(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0);//Verifica se recebeu setpoint
		mq_receive(qReading, (char*) &msgReading, sizeof(int), 0);	//Espera pela leitura da altura
		//Controle
		msgControlSignal = 0;
		mq_send(qControlSignal, (char*) &msgControlSignal, sizeof(int), 0);
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
		//Seta PWM para valor calculado na Thread de controle
	}
}

void *setPoint(void *arg)
{
	mqd_t qSetPoint, qReadingShow;
	int msgSetPoint, msgReadingShow;

	if((qSetPoint = mq_open(qnSetPoint, O_RDWR)) < 0)
	{
		perror ("mq_open SetPoint");
		exit (1);
	}

	if((qReadingShow = mq_open(qnReadingShow, O_RDWR)) < 0)
	{
		perror ("mq_open Reading");
		exit (1);
	}

	while(1)
	{
		cout << "Digite a altura desejada da bolinha." << endl;
		cin >> msgSetPoint;
		mq_send(qSetPoint, (char*) &msgSetPoint, sizeof(int), 0);
		mq_receive(qReadingShow, (char*) &msgReadingShow, sizeof(int), 0);//Verifica se recebeu setpoint
		cout << "Valor Lido: " << msgReadingShow << endl;
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

	if (mq_open(qnReadingShow, O_RDWR|O_CREAT, 0666, &attr) < 0)
	{
		perror ("mq_open ReadingShow");
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
	while(1);
}
