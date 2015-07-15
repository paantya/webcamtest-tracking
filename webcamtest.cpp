#include <opencv2\highgui\highgui.hpp>

#include "WebcamCapture.h"
#include "ProcessingThread.h"
#include "TSDataHandler.h"

int main(int argc, char* argv[])
{
	// инициализация очереди на обработку изображения
	TSDataHandler* cap2proc = new TSDataHandler();
	// инициализация очереди для вывода 
	TSDataHandler* proc2out = new TSDataHandler();
	cv::Mat img;

	// инициализация и старт потоков считывания и обработки данных
	WebcamCapture capThread(cap2proc);
	ProcessingThread procThread(cap2proc, proc2out);
	capThread.start();
	procThread.start();

	// цикл вывода обработанных изображений
	forever
	{
		int key = -1;
		if (proc2out->ReadFrame(img))
		{
			cv::imshow("Output", img);
			key = cv::waitKey(1);
			// если нажат пробел, то break
			if (key == 27)
				break;
		}
	}

	// закрытие потоков
	capThread.exit();
	procThread.exit();
	return 0;
}