#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
    timer = new QTimer(this);
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCounter()));
}

void ejemplo1::doButton()
{
    if(!stop)
    {
        timer->start(1000);
        stop=true;
    }
    else
    {
        timer->stop();
        stop=false;
    }

}
void ejemplo1::updateCounter()
{
    contador++;
    lcdNumber->display(contador);
}





