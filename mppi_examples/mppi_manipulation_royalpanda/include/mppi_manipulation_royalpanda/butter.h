// from https://github.com/ZacharyTaylor/butter/blob/master/src/butter.cpp

class Butter2 {
 public:
  // constructor
  // Coefficients from https://www.meme.net.au/butterworth.html
  //

  // filtertype  = Butterworth
  // passtype  = Lowpass
  // ripple  =
  // order = 2
  // samplerate  = 200
  // corner1 = 50
  // corner2 =
  // adzero  =
  // logmin  = -20
  Butter2() {
    // 50 Hz cutoff
    // gain_ = 3.414213562e+00;
    // a_[0] = -0.1715728753;
    // a_[1] = 0.0000000000

    // 25 Hz cutoff
    gain_ = 1.024264069e+01;
    a_[0] = -0.3333333333;
    a_[1] = 0.9428090416;
    initalised = false;
  }
  /**
   * Add a new raw value to the filter
   *
   * @return retrieve the filtered result
   */
  double apply(double sample) {
    if (!initalised) {
      initalised = true;
      return reset(sample);
    }
    xs_[0] = xs_[1];
    xs_[1] = xs_[2];
    xs_[2] = sample / gain_;
    ys_[0] = ys_[1];
    ys_[1] = ys_[2];
    ys_[2] =
        (xs_[0] + xs_[2]) + 2 * xs_[1] + (a_[0] * ys_[0]) + (a_[1] * ys_[1]);
    return ys_[2];
  }

  /**
   * Reset the filter state to this value
   */
  double reset(double sample) {
    xs_[0] = sample;
    xs_[1] = sample;
    xs_[2] = sample;
    ys_[0] = sample;
    ys_[1] = sample;
    ys_[2] = sample;
    return sample;
  }

 private:
  bool initalised;
  double a_[2];

  double gain_;

  double xs_[3];
  double ys_[3];
};