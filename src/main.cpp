#include "include/horus_patrol/main_window.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    HorusPatrol::LoginDialog *logindialog = new HorusPatrol::LoginDialog;
    logindialog->exec();

    if(logindialog->result())
    {
        HorusPatrol::ROSNode ros_node(argc,argv,"horus_patrol");
        HorusPatrol::MainWindow window(&ros_node,logindialog->getUserProfile());
        delete logindialog;
        window.show();
        app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

        return app.exec();
    }
    else
        app.quit();
}
