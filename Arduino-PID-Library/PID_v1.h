
#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0


//Constants used in some of the functions below
#define AUTOMATIC   1
#define MANUAL      0
#define DIRECT      0
#define REVERSE     1

class PID
{
public:
    /**
     * 
     */
    PID()
    {}
    /**
     * 
     * @param 
     * @param 
     * @param 
     * @param 
     */
    void init(double Kp, double Ki, double Kd, int ControllerDirection);
    /**
     * sets PID to either Manual (0) or Auto (non-0)
     * 
     * @param Mode
     */
    void SetMode(int Mode);
    /**
     * performs the PID calculation. it should be called every time loop() cycles. ON/OFF and
     * calculation frequency can be set using SetMode SetSampleTime respectively
     * 
     * @return 
     */
    bool Compute();
    /**
     * clamps the output to a specific range. 0-255 by default, but it's likely the user will want to change this depending on
     * the application
     * 
     * @param 
     * @param 
     */
    void SetOutputLimits(double Min, double Max);
    /**
     * 
     * @param setpoint
     */
    void Setpoint(double setpoint)
    {
        mySetpoint = setpoint;
    }
    /**
     * 
     * @param input
     */
    void Input(double input)
    {
        myInput = input;
    }
    /**
     * 
     * @return 
     */
    double Output() 
    {
        return myOutput;
    }
    /**
     * While most users will set the tunings once in the constructor, this function gives the user the option
     * of changing tunings during runtime for Adaptive control
     * 
     * @param 
     * @param 
     * @param 
     */
    void SetTunings(double Kp, double Ki, double Kd);
    /**
     * Sets the Direction, or "Action" of the controller. DIRECT means the output will increase when error is positive. REVERSE
     * means the opposite.  it's very unlikely that this will be needed once it is set in the constructor.
     * 
     * @param 
     */
    void SetControllerDirection(int Direction);
    /**
     * sets the frequency, in Milliseconds, with which the PID calculation is performed.  default is 100
     * 
     * @param 
     */
    void SetSampleTime(int NewSampleTime);

    //Display functions ****************************************************************
    
    double GetKp();                     // These functions query the pid for interal values.
    double GetKi();                     // they were created mainly for the pid front-end,
    double GetKd();                     // where it's important to know what is actually 
    int GetMode();                      // inside the PID.
    int GetDirection();                 //
    double GetSetpoint();
    
private:
    void Initialize();

    double dispKp;                      // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;                      //   format for display purposes
    double dispKd;                      //

    double kp;                          // * (P)roportional Tuning Parameter
    double ki;                          // * (I)ntegral Tuning Parameter
    double kd;                          // * (D)erivative Tuning Parameter

    int controllerDirection;

    double myInput;                    
    double myOutput;                   
    double mySetpoint;                 

    unsigned long lastTime;
    double ITerm, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;
};

#endif
