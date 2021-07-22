
/*---------
    Encoder Speed Filter
  ---------*/

// Low pass butterworth filter order=2 alpha1=0.016
class SpeedFilter
{
  public:
    SpeedFilter()
    {
      this -> reset();
    }
  private:
    double v[3];
  public:
    double step(double x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (2.357208772852337209e-3 * x)
             + (-0.86747213379166820957 * v[0])
             + (1.85804329870025886073 * v[1]);
      return
        (v[0] + v[2])
        + 2 * v[1];
    }

    void reset()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
    }
};
