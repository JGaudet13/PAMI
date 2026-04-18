#include <QTRSensors.h>
class LineSensor {
public:
  LineSensor(double alpha);
  void init();
  void calibrateLineSensor();
  double readSensorCOM();
  double readSensorBAD();
  QTRSensors qtr;
  double filtered;
  double alpha;
  double positions[8] = {
    -33.3375, // Capteur 0 (Extrême gauche)
    -23.8125, // Capteur 1
    -14.2875, // Capteur 2
    -4.7625,  // Capteur 3
     4.7625,  // Capteur 4
     14.2875, // Capteur 5
     23.8125, // Capteur 6
     33.3375  // Capteur 7 (Extrême droite)
  };
};
