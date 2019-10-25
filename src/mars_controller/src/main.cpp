#include <QApplication>
#include "mars_controller/mainlogic.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainLogic *model = new MainLogic(argc, argv, &a);

    return a.exec();
}


