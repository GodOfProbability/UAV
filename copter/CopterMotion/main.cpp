#include "MainWindow.h"
#include <QApplication>
#include <QtCore/QDebug>
 
QT_USE_NAMESPACE

int main(int argc, char *argv[]) {

	
	QApplication application(argc, argv);
	MainWindow   window;

	window.show();	

	return application.exec();

}
