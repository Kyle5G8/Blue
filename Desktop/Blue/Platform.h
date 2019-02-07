class Platform
{
  public:
  Platform();
  void attachPins(int leftTrack, int rightTrack);
  void passRaw(int left, int right);
  int* getVals();
  private:
  int _pinL;
  int _pinR;
};
