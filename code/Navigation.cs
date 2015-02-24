using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using System.Windows.Forms;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double _x, _y, _theta;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double _currentEncoderPulseL, _currentEncoderPulseR;
        public double _lastEncoderPulseL, _lastEncoderPulseR;
        public double _wheelDistanceR, _wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short _desiredRotRateR, _desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 50;

        public double secondsElapsed;

        private static int encoderMax = 32767;
        public int PULSESPERROTATION = 190;
        public double WHEELRADIUS = 0.089;
        public double ROBOTRADIUS = 0.242;//0.232
        private double _angleTravelled, _distanceTravelled;
        private double _diffEncoderPulseL, _diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 0.5;
        private double Kalpha = 8;//8
        private double Kbeta = -1;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 45;//15;
        public short K_I = 8;//0;
        public short K_D = 5;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;
        public double e_L_last = 0;
        public double e_R_last = 0;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        private double _lastDiffL = 0;
        private double _lastDiffR = 0;
        private int _zeroCounterL = 0;
        private int _zeroCounterR = 0;

        private LinkedList<double> _movingAvgValuesL;
        private LinkedList<double> _movingAvgValuesR;
        private LinkedList<double> _movingAvgDErrL;
        private LinkedList<double> _movingAvgDErrR;
        public LinkedList<double> _trajX;
        public LinkedList<double> _trajY;
        public LinkedList<double> _trajT; 

        private readonly double pulsePerMeter;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            Initialize();

            _movingAvgValuesL = new LinkedList<double>();
            _movingAvgValuesR = new LinkedList<double>();
            _movingAvgDErrL = new LinkedList<double>();
            _movingAvgDErrR = new LinkedList<double>();

            _trajX = new LinkedList<double>();


            _trajY = new LinkedList<double>();


            _trajT = new LinkedList<double>();


            double wheelCircumference = WHEELRADIUS*2*Math.PI;
            pulsePerMeter = PULSESPERROTATION/wheelCircumference;

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            _x = 0;//initialX;
            _y = 0;//initialY;
            _theta = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            _wheelDistanceR = 0;
            _wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();

            _trajX.Clear();
            _trajY.Clear();
            _trajT.Clear();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);



            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).

                // Find elapsed time
                Stopwatch stopWatch = new Stopwatch();
                stopWatch.Start();
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                //Console.WriteLine("diffEncL: {0}, diffEncR:{1}", _diffEncoderPulseL, _diffEncoderPulseR);
                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    if (jaguarControl.trackTraj)
                        TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                stopWatch.Stop();
                long milliElapsed = stopWatch.ElapsedMilliseconds;
                int maxTime = Math.Max(deltaT - (int) milliElapsed,0);

                // Sleep to approximate 10 Hz update rate

                Thread.Sleep(maxTime); //not sure if this works anymore..... -wf

                stopWatch.Reset();
            }

        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(10);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        _currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        _currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        _lastEncoderPulseL = _currentEncoderPulseL;
                        _lastEncoderPulseR = _currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                _currentEncoderPulseL = 0;
                _currentEncoderPulseR = 0;
                _lastEncoderPulseL = 0;
                _lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                _currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                _currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    _currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    _currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }


        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(_desiredRotRateL);
            motorSignalR = (short)(_desiredRotRateR);

        }


        private double movingAverage(LinkedList<double> list, double newValue, int numElements)
        {
            list.AddFirst(newValue);
            if (list.Count > numElements) {list.RemoveLast();}

            var accum = 0.0;
            foreach (var value in list) {accum += value;}

            return accum/list.Count;
        }


        /*
        public void CalcMotorSignals()
        {
            motorSignalL = (short) (1.5*_desiredRotRateL);
            motorSignalR = (short) (1.5*_desiredRotRateR);
            Console.WriteLine("rotLDes: {0}, rotRDes: {1}", _desiredRotRateL, _desiredRotRateR);
            Console.WriteLine("ML: {0}, MR: {1}", motorSignalL, motorSignalR);

        }*/

        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            double K_p = 120; ;
            double K_i = 0.5;
            double K_d = 2;

            double maxErr = 8000 / deltaT;


            e_L = _desiredRotRateL - _diffEncoderPulseL / deltaT;
            e_R = _desiredRotRateR + _diffEncoderPulseR / deltaT;

            e_sum_L = .9 * e_sum_L + e_L * deltaT;
            e_sum_R = .9 * e_sum_R + e_R * deltaT;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr));
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * (e_L - e_L_last) / deltaT));
            e_L_last = e_L;

            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * (e_R - e_R_last) / deltaT));
            e_R_last = e_R;
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            const double deadBand = 5000.0;
            var deadBandLeft = u_L > 0 ? deadBand : -deadBand;
            var deadBandRight = u_R > 0 ? deadBand : -deadBand;

            motorSignalL = _desiredRotRateL == 0 ? (short) zeroOutput : (short)(zeroOutput + deadBandLeft + u_L);
            motorSignalR = _desiredRotRateR == 0 ? (short) zeroOutput : (short)(zeroOutput - deadBandRight - u_R);

            motorSignalL = (short)(zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));


        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        {

            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0,motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + _x.ToString() + " " + _y.ToString() + " " + _theta.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {
            // calculate x, y and theta to desired point
            var deltaX = desiredX - x_est;
            var deltaY = desiredY - y_est;

            const double distanceThreshold = 0.15;
            var distanceToTarget = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));

            Console.WriteLine("Distance to target : {0}", distanceToTarget);

            if (distanceToTarget > distanceThreshold)
            {
                travelToPoint(deltaX, deltaY);
                Console.WriteLine("Travel to point");
            }
            else
            {
                rotateToPoint();
                Console.WriteLine("Rotate to point");
            }


            const int threshold = 40;

            if (Math.Abs(_desiredRotRateL) < threshold)
                _desiredRotRateL = 0;
            if (Math.Abs(_desiredRotRateR) < threshold)
                _desiredRotRateR = 0;

        }


        private void rotateToPoint()
        {
            double headingX = Math.Cos(desiredT);
            double headingY = Math.Sin(desiredT);
            double rotateX = headingX*Math.Cos(-t_est) - headingY*Math.Sin(-t_est);
            double rotateY = headingX*Math.Sin(-t_est) + headingY*Math.Cos(-t_est);

            double rotateTheta = Math.Atan2(rotateY, rotateX);

            Console.WriteLine("rotateTheta: {0}", rotateTheta);

            if (Math.Abs(rotateTheta) > 0.17)
            {
                double desiredW = Math.Abs(Kbeta * .25) * rotateTheta;

                // calculate desired wheel rotation rate
                double rightV = (desiredW * ROBOTRADIUS) / WHEELRADIUS;
                double leftV = (-desiredW * ROBOTRADIUS) / WHEELRADIUS;

                double scaleRatio = limitMotorSpeed(rightV, leftV);

                _desiredRotRateR = (short)(pulsePerMeter * scaleRatio * rightV);
                _desiredRotRateL = (short)(pulsePerMeter * scaleRatio * leftV);

                return;
            }

            _desiredRotRateR = _desiredRotRateL = 0;

            //jaguarControl.controlMode = jaguarControl.MANUAL;
        }


        private void travelToPoint(double deltaX, double deltaY)
        {
            // check if robot is in front or behind of the robot
            var thetaX = Math.Cos(t_est);
            var thetaY = Math.Sin(t_est);

            var angleDifference = vectorAngle(thetaX, thetaY, deltaX, deltaY);

            // caculate rho and alpha
            var pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));

            var alpha = -t_est
                + ((angleDifference <= Math.PI / 2)
                ? Math.Atan2(deltaY, deltaX)
                : Math.Atan2(-deltaY, -deltaX));
            alpha = boundAngle(alpha);

            var beta = -t_est - alpha + desiredT;
            beta = boundAngle(beta);

            // calculate desired wheel velocities
            var desiredV = (angleDifference < Math.PI / 2)
                ? Kpho * pho
                : -Kpho * pho;


            var desiredW = Kalpha * alpha + Kbeta * beta;

            // calculate desired wheel rotation rate
            double rightV = (desiredV + desiredW * ROBOTRADIUS) / WHEELRADIUS;
            double leftV = (desiredV - desiredW * ROBOTRADIUS) / WHEELRADIUS;

            double scaleRatio = limitMotorSpeed(rightV, leftV);

            _desiredRotRateR = (short)(pulsePerMeter * scaleRatio * rightV);
            _desiredRotRateL = (short)(pulsePerMeter * scaleRatio * leftV);

        }


        private double limitMotorSpeed(double rightV, double leftV)
        {
            double leftRatio = Math.Max(1, Math.Abs(leftV)/.25);
            double rightRatio = Math.Max(1, Math.Abs(rightV)/.25);

            return 1/Math.Max(leftRatio, rightRatio);
        }


        private double vectorAngle(double u1, double u2, double v1, double v2)
        {
            // cos \theta = (A dot B) / (|A| |B|)
            var dotProduct = u1*v1 + u2*v2;
            var magProduct = Math.Sqrt(Math.Pow(u1, 2) + Math.Pow(u2, 2))*
                             Math.Sqrt(Math.Pow(v1, 2) + Math.Pow(v2, 2));

            if (magProduct == 0)
                return 0;

            return Math.Acos(dotProduct/magProduct);
        }


        private double Threshold(double x, double max, double min)
        {
            return Math.Max(Math.Min(x, max), min);
        }


        // This function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            if (_trajX.Count > 0)
            {
                // calculate x, y and theta to desired point
                var deltaX = desiredX - x_est;
                var deltaY = desiredY - y_est;

                const double distanceThreshold = 0.15;
                var distanceToTarget = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));


                if (distanceToTarget < distanceThreshold)
                {
                    desiredX = _trajX.First.Value;
                    desiredY = _trajY.First.Value;
                    desiredT = _trajT.First.Value;

                    _trajX.RemoveFirst();
                    _trajY.RemoveFirst();
                    _trajT.RemoveFirst();
                }
            }

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions

        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // _distanceTravelled and _angleTravelled respectively.

        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            _diffEncoderPulseL = _currentEncoderPulseL - _lastEncoderPulseL;
            _diffEncoderPulseR = _currentEncoderPulseR - _lastEncoderPulseR;

            if (Math.Abs(_diffEncoderPulseL) > encoderMax / 2.0)
            {
                if (_diffEncoderPulseL < 0)       // if going forwards
                    _diffEncoderPulseL += encoderMax;
                else if (_diffEncoderPulseL > 0) // if going backwards
                    _diffEncoderPulseL -= encoderMax;
            }

            if (Math.Abs(_diffEncoderPulseR) > encoderMax / 2.0)
            {
                if (_diffEncoderPulseR < 0)       // if going forwards
                    _diffEncoderPulseR += encoderMax;
                else if (_diffEncoderPulseR > 0) // if going backwards
                    _diffEncoderPulseR -= encoderMax;
            }

            // Check for zero outlier
            _diffEncoderPulseL = ZeroOutlier(_diffEncoderPulseL, ref _lastDiffL, ref _zeroCounterL);
            _diffEncoderPulseR = ZeroOutlier(_diffEncoderPulseR, ref _lastDiffR, ref _zeroCounterR);

            // Update encoder measurements
            _lastEncoderPulseL = _currentEncoderPulseL;
            _lastEncoderPulseR = _currentEncoderPulseR;


            // Calculate distance
            double numRotationL = _diffEncoderPulseL / PULSESPERROTATION;
            double numRotationR = _diffEncoderPulseR / PULSESPERROTATION;

            _wheelDistanceL = numRotationL * WHEELRADIUS * 2 * Math.PI;
            _wheelDistanceR = -numRotationR * WHEELRADIUS * 2 * Math.PI;

            // Distance and angle robot travelled
            _distanceTravelled = (_wheelDistanceL + _wheelDistanceR) / 2;
            _angleTravelled = (_wheelDistanceR - _wheelDistanceL) / (2 * ROBOTRADIUS);
        }

        public double ZeroOutlier(double currDiff, ref double lastDiff, ref int counter)
        {
            // Check if the diffEncoderPulse values have zero outliers, and if so
            // throw out, but keep a count of the number of recurring zeros

            if ((int) currDiff == 0 && counter < 4)
            {
                counter++;
                return lastDiff;
            }

            if (currDiff == 0 && counter >= 4)
            {
                lastDiff = currDiff;
                counter++;
                return currDiff;
            }

            lastDiff = currDiff;
            counter = 0;
            return currDiff;
        }
 


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with _angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {

            // calculate change in x and y position
            var deltX = _distanceTravelled * Math.Cos(_theta + _angleTravelled / 2);
            var deltY = _distanceTravelled * Math.Sin(_theta + _angleTravelled / 2);

            // Update the actual
            _x = _x + deltX;
            _y = _y + deltY;
            _theta = _theta + _angleTravelled;

            _theta = boundAngle(_theta);

        }


        private double boundAngle(double angle)
        {
            var newAngle = mod(angle, 2*Math.PI);

            if (newAngle > Math.PI)
                newAngle = newAngle - 2*Math.PI;

            return newAngle;
        }


        public double mod(double x, double m)
        {
            double r = x % m;
            return r < 0 ? r + m : r;
        }
        
        
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with _angleTravelled and
        // distance travelled.
       
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as _angleTravelled and _distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = _x;
            y_est = _y;
            t_est = _theta;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}
