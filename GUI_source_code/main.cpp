#include "fangcontrol.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    FangControl w;
    w.show();

    return a.exec();
}
