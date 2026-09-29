#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "UDPHandler.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    UDPHandler udpHandler;

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("udpHandler", &udpHandler);
    engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));

    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
