class RCChannel {

  public:
    RCChannel(int pin);

    /** get the duration of the last RC pulse in microseconds */
    long getPulseDuration();

    /** set the RC pulse duration to 1500 */
    void reset();

    /** Call when an interrupt occurs */
    void handleInterrupt();

  private:
    int pin;
    volatile unsigned long pulseStart = 0;
    volatile long pulseDuration = 1500;
    volatile int skipCount = 0;
};
