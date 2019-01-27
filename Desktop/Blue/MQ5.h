class MQ5
{
  public:
    MQ5();
    void attachPin(int pin);
    int RawAnalog();
    float PPM();
    void Cal();
  private:
    int _pin;
};
