class CustomServo {
  int freq;
  int channel;
  int resolution;
  int pin;
  int min;
  int max;
  int angle = 0;
public:
  CustomServo();
  void attach(int pin, int min, int max, int freq = 50, int channel = 0, int resolution = 12);
  void detach();
  void write(int angle);

};