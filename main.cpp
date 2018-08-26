#include <QApplication>
#include <QDateTime>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
    qRegisterMetaTypeStreamOperators<Pose>("Pose");

    QApplication a(argc, argv);

    qsrand(QDateTime::currentMSecsSinceEpoch());

    MainWindow w;
    w.show();

    if(argc == 2) w.loadMap(argv[1]);

    return a.exec();
}
