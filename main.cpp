#include <QApplication>
#include <QCommandLineParser>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include "MainWindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("Drive-vehicle");

    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    // Optional command-line options (handy for demos and batch screenshots), e.g.
    //   --map city --vehicle sedan --speed 40 --run --screenshot out.png --screenshot-after 8000 --quit
    QCommandLineParser parser;
    parser.setApplicationDescription("Road-hazard load / damage simulation (Qt + VTK)");
    parser.addHelpOption();
    QCommandLineOption mapOpt("map", "Map: highway | city | national", "map", "highway");
    QCommandLineOption vehicleOpt("vehicle", "Vehicle: compact | sedan | suv | truck", "vehicle", "truck");
    QCommandLineOption startOpt("start", "Index of the 'Start at' section of the map", "index", "0");
    QCommandLineOption startAtOpt("start-at", "Start at an arbitrary course position [m] (overrides --start)", "metres");
    QCommandLineOption speedOpt("speed", "Fixed speed [km/h] (default: auto, obey speed limits)", "kmh");
    QCommandLineOption scaleOpt("time-scale", "Simulation time scale: 0.25 | 0.5 | 1 | 2 | 4 | 8", "x", "1");
    QCommandLineOption viewOpt("view", "Camera: chase | wheels | top | front | map", "view", "chase");
    QCommandLineOption hazardOpt("hazard", "Hazard type index for placing (0..6)", "index");
    QCommandLineOption placeOpt("place-ahead", "Place the selected hazard ahead of the vehicle after <ms>", "ms");
    QCommandLineOption runOpt("run", "Start driving immediately");
    QCommandLineOption shotOpt("screenshot", "Save a screenshot of the window to <file>", "file");
    QCommandLineOption shotAfterOpt("screenshot-after", "Delay before the screenshot [ms]", "ms", "5000");
    QCommandLineOption quitOpt("quit", "Quit after the screenshot");
    parser.addOptions({ mapOpt, vehicleOpt, startOpt, startAtOpt, speedOpt, scaleOpt, viewOpt,
                        hazardOpt, placeOpt, runOpt, shotOpt, shotAfterOpt, quitOpt });
    parser.process(app);

    MainWindow w;
    w.show();

    const QStringList maps = { "highway", "city", "national" };
    const QStringList vehicles = { "compact", "sedan", "suv", "truck" };
    const QStringList views = { "chase", "wheels", "top", "front", "map" };

    const int map = maps.indexOf(parser.value(mapOpt).toLower());
    if (map > 0)
        w.setMap(static_cast<MapType>(map));
    const int vehicle = vehicles.indexOf(parser.value(vehicleOpt).toLower());
    if (vehicle >= 0 && vehicle != 3)
        w.setVehicle(static_cast<VehicleType>(vehicle));
    if (parser.value(startOpt).toInt() > 0)
        w.setStartSection(parser.value(startOpt).toInt());
    if (parser.isSet(startAtOpt))
        w.setStartPosition(parser.value(startAtOpt).toDouble());
    if (parser.isSet(speedOpt))
        w.setSpeedKmh(parser.value(speedOpt).toInt());
    w.setTimeScale(parser.value(scaleOpt).toDouble());
    if (parser.isSet(hazardOpt))
        w.setHazardKind(parser.value(hazardOpt).toInt());
    const int view = views.indexOf(parser.value(viewOpt).toLower());
    if (view > 0)
        w.setCameraView(view);
    if (parser.isSet(runOpt))
        QTimer::singleShot(300, &w, [&w]() { w.startSimulation(); });
    if (parser.isSet(placeOpt))
        QTimer::singleShot(parser.value(placeOpt).toInt(), &w, [&w]() { w.placeHazardAhead(); });

    if (parser.isSet(shotOpt))
    {
        const QString file = parser.value(shotOpt);
        const bool quit = parser.isSet(quitOpt);
        QTimer::singleShot(parser.value(shotAfterOpt).toInt(), &w, [&w, file, quit]()
        {
            w.saveScreenshot(file);
            if (quit)
                QApplication::quit();
        });
    }

    return app.exec();
}
