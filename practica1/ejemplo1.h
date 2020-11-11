#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        int contador=0;
        bool stop=0;
        QTimer *timer;
        ejemplo1();

    public slots:
        void doButton();
        void updateCounter();
};

#endif // ejemplo1_H
