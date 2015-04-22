using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using System.Xml.Schema;
using MathNet.Numerics.Statistics;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public double _goalX, _goalY;
        public int numWPs;
        public double[,] _waypoints;
        public int _currentWP;
        const int STARTWP = 0;

        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double _x, _y, _theta;
        public double x_est, y_est, t_est;
        public double initialX = 0;//3.3;//-3.3;
        public double initialY = 0;//-0.3;//-7.7;
        public double initialT = 3.14;//0;
        public double desiredX, desiredY, desiredT;
        public double desiredR;
        public double _actRotRateL, _actRotRateR;

        public double _currentEncoderPulseL, _currentEncoderPulseR;
        public double _lastEncoderPulseL, _lastEncoderPulseR;
        public double _wheelDistanceR, _wheelDistanceL;
        public double currentAccel_x, currentAccel_y, currentAccel_z;

        public bool newLaserData = false;
        public bool motionPlanRequired;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        const short maxPosOutput = 32767;
        const short takeOffMax = 1000;
        const double DEADBAND = 8000;
        private double _leftMot, _rightMot;
        private short _desiredRotRateR, _desiredRotRateL, _desiredRotRateRPrev, _desiredRotRateLPrev;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 50;
        private static int encoderMax = 32767;
        public int PULSESPERROTATION = 190;
        public double WHEELRADIUS = 0.089;//0.089;  //Ben Chasnov Corrections
        public double ROBOTRADIUS = 0.232;//0.242;//0.232 //Ben Chasnov Corrections
        public double _angleTravelled, _distanceTravelled;
        private double _diffEncoderPulseL, _diffEncoderPulseR;
        private double maxVelocity = 1.0;

        private double Kpho = 1;
        private double Kalpha = 5;//2//8
        private double Kbeta = -1;//-0.5//-1.0;

        double time = 0;
        DateTime startTime;


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
        private const short ZEROOUTPUT = 16383;
        public double _accumL = 0;
        public double _accumR = 0;


        private LinkedList<double> _movingAvgValuesL;
        private LinkedList<double> _movingAvgValuesR;
        private LinkedList<double> _movingAvgDErrL;
        private LinkedList<double> _movingAvgDErrR;
        public LinkedList<double> _trajX;
        public LinkedList<double> _trajY;
        public LinkedList<double> _trajT;

        public int count = 0;

        private long _milliElapsed;
        private readonly double pulsePerMeter;

        // PF Variables
        public Map map;
        public ParticleFilter pf;
        public Clustering cluster;

        public int numParticles = 300;
        public Random random = new Random();
        public double[] laserAngles;

        public Object thisLock = new object();

        private HashSet<int> blackList; 
        
        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;

        // clustering
        public double radius = .1;
        public double[][] clusterCount;

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }
        }


        #endregion



        #region Navigation Setup

        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            pf = new ParticleFilter(numParticles, this, map);
            cluster = new Clustering(this);

            

            // count number of lines in CSV file
            numWPs = File.ReadAllLines(@"..\..\waypoints.csv").Length + 1;
            _waypoints = new double[numWPs, 2];
            
            //_waypoints[0, 0] = initialX;
            //_waypoints[0, 1] = initialY;

            // open CSV file
            var reader = new StreamReader(File.OpenRead(@"..\..\waypoints.csv"));
            int r = 0;

            // read line by line
            while (!reader.EndOfStream)
            {
                var line = reader.ReadLine();
                var values = line.Split(',');
                _waypoints[r, 0] = Convert.ToDouble(values[0]);
                _waypoints[r, 1] = Convert.ToDouble(values[1]);
                r++;
            }
            reader.Close();
            _currentWP = STARTWP + 1;
            initialX = _waypoints[STARTWP, 0];
            initialY = _waypoints[STARTWP, 1];
            //initialT
            
            this.Initialize();

            _movingAvgValuesL = new LinkedList<double>();
            _movingAvgValuesR = new LinkedList<double>();
            _movingAvgDErrL = new LinkedList<double>();
            _movingAvgDErrR = new LinkedList<double>();

            _trajX = new LinkedList<double>();
            _trajY = new LinkedList<double>();
            _trajT = new LinkedList<double>();


            double wheelCircumference = WHEELRADIUS * 2 * Math.PI;
            pulsePerMeter = PULSESPERROTATION / wheelCircumference;

            blackList = new HashSet<int>();
            // Create black list
            for (int i = 0; i < LaserData.Length / 2; i++)
            {
                blackList.Add(i);
            }

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            _x = initialX;
            _y = initialY;
            _theta = initialT;

            // Initialize accumulation
            _accumL = 0;
            _accumR = 0;

            // Initialize state estimates
            x_est = initialX;
            y_est = initialY;
            t_est = initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Set previous desired rotation rate
            _desiredRotRateLPrev = 0;
            _desiredRotRateRPrev = 0;

            // Reset Localization Variables
            _wheelDistanceR = 0;
            _wheelDistanceL = 0;

            if (jaguarControl.Simulating())
            {
                // Zero actuator signals
                motorSignalL = 0;
                motorSignalR = 0;
                loggingOn = false;
            }
            else
            {
                // Zero actuator signals
                motorSignalL = ZEROOUTPUT;
                motorSignalR = ZEROOUTPUT;
                loggingOn = false;
            }

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;

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

            _currentWP = STARTWP+1;
            pf.ResetPF();
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
                Stopwatch stopWatch = new Stopwatch();
                stopWatch.Start();

                lock (thisLock)
                {
                    // ****************** Additional Student Code: Start ************

                    // Students can select what type of localization and control
                    // functions to call here. For lab 1, we just call the function
                    // WallPositioning to have the robot maintain a constant distance
                    // to the wall (see lab manual).

                    // Find elapsed time
                    

                    // Update Sensor Readings
                    UpdateSensorMeasurements();

                    // Determine the change of robot position, orientation (lab 2)
                    MotionPrediction();

                    // Update the global state of the robot - x,y,t (lab 2)
                    LocalizeRealWithOdometry();

                    // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                    LocalizeEstWithParticleFilter();

                    clusterCount = cluster.FindClusters(radius);
                    

                    // If using the point tracker, call the function
                    if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                    {

                        // Check if we need to create a new trajectory
                        /*if (motionPlanRequired)
                        {
                            // Construct a new trajectory (lab 5)
                           // PRMMotionPlanner();
                            motionPlanRequired = false;
                        }*/
                        // Drive the robot to a desired Point (lab 3)
                        //FlyToSetPoint();



                        // Follow the trajectory instead of a desired point (lab 3)
                        //TrackTrajectoryPRM();

                        // Follow the trajectory instead of a desired point (lab 3)
                        if (jaguarControl.AUTOMODE == jaguarControl.TRACKTRAJ)
                            TrackTrajectory();
                        //else if (jaguarControl.AUTOMODE == jaguarControl.CIRCLE)
                        //    TrajectoryCircle();

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
                }

                // Sleep to approximate 10 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf

                stopWatch.Stop();
                _milliElapsed = stopWatch.ElapsedMilliseconds;
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
                int iCounter = 0;
                while (!gotFirstEncoder && iCounter < 10)
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

                    }
                    catch (Exception e) { }
                    iCounter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                _currentEncoderPulseL = 0;
                _currentEncoderPulseR = 0;
                _lastEncoderPulseL = 0;
                _lastEncoderPulseR = 0;
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

                var segments = map.SegmentsWithinRadius(_x, _y, Map.MAXLASERDISTANCE);

                // Get most recent laser scanner measurements
                for (int i = 0; i < LaserData.Length; i++)
                {
                    LaserData[i] = (long)Math.Round(1000.0 * map.GetClosestWallDistance(_x, _y, _theta -1.57 + laserAngles[i], segments));
                }
                newLaserData = true;
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
            if (list.Count > numElements) { list.RemoveLast(); }

            var accum = 0.0;
            foreach (var value in list) { accum += value; }

            return accum / list.Count;
        }


        public void CalcMotorSignals()
        {

            //short maxPosOutput = 32767;
            //short takeOffMax = 1000;

            double K_p = 0.35;
            double K_f = 7.0;
            double K_i = 0.0;
            double K_d = 0.0;

            double deltaTs = _milliElapsed / 1000.0;

            //_desiredRotRateL = _desiredRotRateR = 190;

            double maxErr = 8000 / deltaTs;
            //_desiredRotRateL = 250;
            //_desiredRotRateR = 250;

            _actRotRateL = movingAverage(_movingAvgValuesL,_diffEncoderPulseL / deltaTs,10);
            _actRotRateR = movingAverage(_movingAvgValuesR,_diffEncoderPulseR / deltaTs,10);


            e_L = _desiredRotRateL - _actRotRateL;
            e_R = _desiredRotRateR + _actRotRateR;

            e_sum_L = .9 * e_sum_L + e_L * deltaTs;
            e_sum_R = .9 * e_sum_R + e_R * deltaTs;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr));
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            double dErrL = movingAverage(_movingAvgDErrL, (e_L - e_L_last) / deltaTs, 5);
            double dErrR = movingAverage(_movingAvgDErrR, (e_R - e_R_last) / deltaTs, 5);

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * dErrL));
            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * dErrR));

            e_R_last = e_R;
            e_L_last = e_L;
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            

            if (Math.Sign(_desiredRotRateL) == Math.Sign(_desiredRotRateLPrev))
            {
                _accumL += u_L;
            }
            else
            {
                _accumL = 0;
            }

            if (Math.Sign(_desiredRotRateR) == Math.Sign(_desiredRotRateRPrev))
            {
                _accumR += u_R;
            }
            else
            {
                _accumR = 0;
            }


            _leftMot = K_f * _desiredRotRateL + _accumL;
            _rightMot = K_f * _desiredRotRateR + _accumR;
            // 

            if (_desiredRotRateL == 0)
                motorSignalL = ZEROOUTPUT;
            else
            {
                //motorSignalL = (short) (ZEROOUTPUT + Math.Sign(_desiredRotRateL)*DEADBAND + K_f*_desiredRotRateL + _accumL);
                motorSignalL = (short)(ZEROOUTPUT + Math.Sign(_desiredRotRateL) * DEADBAND + _leftMot);
            }

            if (_desiredRotRateR == 0)
                motorSignalR = ZEROOUTPUT;
            else
            {
                //motorSignalR = (short) (ZEROOUTPUT - Math.Sign(_desiredRotRateR)*DEADBAND - K_f*_desiredRotRateR - _accumR);
                motorSignalR = (short)(ZEROOUTPUT - Math.Sign(_desiredRotRateR) * DEADBAND - _rightMot);
            }

            //******************Adding test code
            //short test = 300;
            //_leftMot = test;
            //_rightMot = -test;
            //motorSignalL = (short)(ZEROOUTPUT + Math.Sign(_leftMot) * DEADBAND + _leftMot);
            //motorSignalR = (short)(ZEROOUTPUT - Math.Sign(_rightMot) * DEADBAND - _rightMot);
            
            //*******************
            motorSignalL = (short)Math.Min(maxPosOutput-takeOffMax, Math.Max(takeOffMax, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput-takeOffMax, Math.Max(takeOffMax, (int)motorSignalR));

            _desiredRotRateLPrev = _desiredRotRateL;
            _desiredRotRateRPrev = _desiredRotRateR;

            //Console.WriteLine("ml: {0}, mr: {1}", motorSignalL, motorSignalR);


        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                //jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
                //Console.WriteLine("ml: {0}, mr: {1}", motorSignalL, motorSignalR);
                
                // If motor is operating subthreshold, add periodic nudges for turning.
                
                short tempMotorSignalL, tempMotorSignalR;
                count = (count + 1) % 4;
                bool pwmtest = count < 3;
                if (Math.Abs(_leftMot) < 1000)
                {
                    if (pwmtest)
                        tempMotorSignalL = (short)(ZEROOUTPUT + Math.Sign(_leftMot) * DEADBAND + Math.Sign(_leftMot)*Math.Min(Math.Abs(4 * _leftMot), 2000));
                    else
                        tempMotorSignalL = ZEROOUTPUT;
                }
                else
                {
                    tempMotorSignalL = motorSignalL;
                }

                if (Math.Abs(_rightMot) < 1000)
                {
                    if (pwmtest)
                        tempMotorSignalR = (short)(ZEROOUTPUT - Math.Sign(_rightMot) * DEADBAND - Math.Sign(_rightMot) * Math.Min(Math.Abs(4 * _rightMot), 2000));
                    else
                        tempMotorSignalR = ZEROOUTPUT;
                }
                else
                {
                    tempMotorSignalR = motorSignalR;
                }

                
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, tempMotorSignalL, tempMotorSignalR, 0);
                Console.WriteLine("ml: {0}, mr: {1}", tempMotorSignalL, tempMotorSignalR);
                
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
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
            logFile = File.CreateText(@"C:\Users\CAPCOM\Desktop\" + "JaguarData_" + date + ".csv");
            startTime = DateTime.Now;
            loggingOn = true;
            String header = "time, x odom, y odom, t odom, x st est, y st est, t st est, x Std, y Std, t Std, lat, long, actRotL, actRotR, desRotL, desRotR, uL, uR";
            logFile.WriteLine(header);
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
                double[] stdSpread = SpreadData();
                double xStd = stdSpread[0];
                double yStd = stdSpread[1];
                double tStd = stdSpread[2];

                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + ", " + _x.ToString() + ", " + _y.ToString() + ", " +
                     _theta.ToString() + ", " + x_est.ToString() + ", " + y_est.ToString() + ", " + t_est.ToString() 
                     + ", " + xStd.ToString() + ", " + yStd.ToString() + ", " + tStd.ToString() + ", " + this.jaguarControl.gpsRecord.latitude + ", " + this.jaguarControl.gpsRecord.longitude
                     +"," + _actRotRateL + "," + _actRotRateR + "," + _desiredRotRateL + "," + _desiredRotRateR+"," + motorSignalL + "," + motorSignalR;

                logFile.WriteLine(newData);
            }
        }

        private double[] SpreadData()
        {
            
            double[] x = new double[numParticles];
            double[] y = new double[numParticles];
            double[] t = new double[numParticles];
            for (int i = 0; i < numParticles; i++)
            {
                x[i] = pf.particles[i].x;
                y[i] = pf.particles[i].y;
                t[i] = pf.particles[i].t;
            }
            var statisticsX = new DescriptiveStatistics(x);
            var statisticsY = new DescriptiveStatistics(y);
            var statisticsT = new DescriptiveStatistics(t);

            double[] stdSpread = new double[3];

            stdSpread[0] = statisticsX.StandardDeviation;
            stdSpread[1] = statisticsY.StandardDeviation;
            stdSpread[2] = statisticsT.StandardDeviation;

            return stdSpread;
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

        private void PointTrack(double goalX, double goalY, double goalT)
        {
            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!
            _goalX = goalX;
            _goalY = goalY;

            double dx = goalX - x_est;
            double dy = goalY - y_est;
            double dt = goalT - t_est;

            double desiredW, desiredV;

            // Make decision whether to go forwards or backwards
            bool forwardCondition = (dx * Math.Cos(t_est) + dy * Math.Sin(t_est)) >= 0;

            

            // Translate problem into coordinates in pho, alpha, and beta.
            double pho = Math.Sqrt(dx * dx + dy * dy);
            double alpha = -t_est + Math.Atan2(dy, dx);
            //Console.WriteLine(goalX);
            //Console.WriteLine(goalY);
            //Console.WriteLine(dy);

            if (pho < 0.2)
                alpha = 0; //close enough, stop caring about alpha.
            double beta = alpha - dt;

            dt = boundAngle(dt);
            alpha = boundAngle(alpha);
            beta = boundAngle(beta);

            if (Math.Abs(alpha) > 1)
            {
                desiredV = 0;
                desiredW = 10 * Math.Sign(alpha); //turn at saturation
            }
            else
            {
                //Threshold close enough values to zero to avoid jitter
                if (pho < 0.2)
                    pho = 0;
                if (Math.Abs(beta) < 0.05)
                    beta = 0;

                // desired forward velocity
                desiredV = Kpho * pho;
                //Console.WriteLine(pho);
                double KbetaFactor = 0; //factor to which to scale Kbeta
                // HIGH LEVEL: ignore desiredT until within 1 meter, 
                // go against the desired direction until 0.1, then turn towards desired direction
                // Thus KbetaFactor is 0 until pho < 1, then it decreases to -2, 
                // and then increases back to 0 at 0.1, and 1 at 0.
                if (pho < 1)
                {
                    KbetaFactor = (pho - 1) /(1-0.4) * 2; //slope from 0 to -2 from 0.7 to 0.3
                    if (pho < 0.4) //slope from -2 to 0 from 0.3 to 0.1
                        KbetaFactor = (0.1 - pho) / 0.3 * 2;
                    if (pho < 0.3) //slope from 0 to 1 from 0.1 to 0.0
                        KbetaFactor = (0.3 - pho) / 0.3;
                }

                desiredW = Kalpha * alpha + KbetaFactor * Kbeta * beta;
                //Console.WriteLine(alpha);
                //Console.WriteLine(KbetaFactor * Kbeta * beta);
                //Console.WriteLine(desiredW);
            }
            //if (Math.Abs(desiredW * robotRadius) > maxVelocity)
            //    desiredW = (0.9*maxVelocity / robotRadius) * (desiredW/Math.Abs(desiredW));

            double leftWheelVelocity = desiredV - desiredW * ROBOTRADIUS;
            double rightWheelVelocity = desiredV + desiredW * ROBOTRADIUS;

            //Console.WriteLine(leftWheelVelocity);
            //// threshold wheel velocities at maxVelocity
            //if (Math.Abs(desiredV) > maxVelocity)
            //    desiredV = Math.Sign(desiredV) * maxVelocity;

            // threshold wheel velocities at maxVelocity
            if (Math.Abs(leftWheelVelocity) > maxVelocity)
            {
                rightWheelVelocity = maxVelocity / Math.Abs(leftWheelVelocity) * rightWheelVelocity;
                leftWheelVelocity = Math.Sign(leftWheelVelocity) * maxVelocity;
            }
            if (Math.Abs(rightWheelVelocity) > maxVelocity)
            {
                leftWheelVelocity = maxVelocity / Math.Abs(rightWheelVelocity) * leftWheelVelocity;
                rightWheelVelocity = Math.Sign(rightWheelVelocity) * maxVelocity;
            }

            double maxRotVelocity = 0.6;
            if (Math.Abs(rightWheelVelocity - leftWheelVelocity) > maxRotVelocity)
            {
                double scaleRatio = maxRotVelocity/Math.Abs(rightWheelVelocity - leftWheelVelocity);
                leftWheelVelocity = scaleRatio * leftWheelVelocity;
                rightWheelVelocity = scaleRatio * rightWheelVelocity;
            }

            

            _desiredRotRateL = (short)(leftWheelVelocity / (2 * Math.PI * WHEELRADIUS) * PULSESPERROTATION);
            _desiredRotRateR = (short)(rightWheelVelocity / (2 * Math.PI * WHEELRADIUS) * PULSESPERROTATION);
            //Console.WriteLine("desiredV: {0}, desiredOmega: {1} ", desiredV, desiredW);
            //Console.WriteLine("desired rot L: {0}, desired rot R:, {1}\n", _desiredRotRateL, _desiredRotRateR);

        }

        private bool LineTrack(double m, double xend, double yend, double x_est, double y_est, double velocity)
        {
            double a = (x_est + m * y_est - xend - m * yend) / (1 + m * m);
            double b = m * a;
            double d = velocity / Kpho;//1.0;
            double tgoal = Math.Atan2(-b, -a);

            double xgoal = xend + a + d * Math.Cos(tgoal);
            double ygoal = yend + b + d * Math.Sin(tgoal);
            
            bool next = false;
            if (Math.Pow(xend - x_est, 2) + Math.Pow(yend - y_est, 2) < d * d)
            {
                xgoal = xend;
                ygoal = yend;
                next = true;
            }
            PointTrack(xgoal, ygoal, tgoal);
            return next;
        }

        private int WaypointTrack(double xEst, double yEst, int currentWP, double velocity)
        {
            double m = (_waypoints[currentWP,1]-_waypoints[currentWP-1,1])/(_waypoints[currentWP,0]-_waypoints[currentWP-1,0]+0.001);
            bool next = LineTrack( m, _waypoints[currentWP,0], _waypoints[currentWP,1], xEst, yEst, velocity );

            if (next && currentWP < (numWPs - 2))
                ++currentWP;

            return currentWP;
        }
        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {
            //PointTrack(desiredX, desiredY, desiredT);
            return;
            //LineTrack(1, desiredX, desiredY, _x, _y);

            
            // calculate x, y and theta to desired point
            var deltaX = desiredX - x_est;
            var deltaY = desiredY - y_est;

            _goalX = desiredX;
            _goalY = desiredY;

            const double distanceThreshold = 0.15;
            var distanceToTarget = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));

            if (distanceToTarget > distanceThreshold)
            {
                travelToPoint(deltaX, deltaY);
            }
            else
            {
                rotateToPoint(0, 0.1);
            }


            const int threshold = 40;

            if (Math.Abs(_desiredRotRateL) < threshold)
                _desiredRotRateL = 0;
            if (Math.Abs(_desiredRotRateR) < threshold)
                _desiredRotRateR = 0;
        }

        public static double squareSumPoints(double x1, double y1, double x2, double y2)
        {

            var deltaX = x1 - x2;
            var deltaY = y1 - y2;

            var distanceToTarget = Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2);
            return distanceToTarget;
        }


        public static double normPoints(double x1, double y1, double x2, double y2)
        {

            var deltaX = x1 - x2;
            var deltaY = y1 - y2;

            var distanceToTarget = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            return distanceToTarget;
        }


        private void rotateToPoint(double desiredV, double threshold)
        {
            double headingX = Math.Cos(desiredT);
            double headingY = Math.Sin(desiredT);

            double rotateX = headingX * Math.Cos(-t_est) - headingY * Math.Sin(-t_est);
            double rotateY = headingX * Math.Sin(-t_est) + headingY * Math.Cos(-t_est);
            double rotateTheta = Math.Atan2(rotateY, rotateX);

            if (Math.Abs(rotateTheta) > threshold)
            {
                double desiredW = Math.Abs(Kbeta * .15) * rotateTheta;

                // calculate desired wheel rotation rate
                double rightV = (desiredV + desiredW * ROBOTRADIUS) / WHEELRADIUS;
                double leftV = (desiredV - desiredW * ROBOTRADIUS) / WHEELRADIUS;

                double scaleRatio = limitMotorSpeed(rightV, leftV, 0.3);

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

            double scaleRatio = limitMotorSpeed(rightV, leftV, 0.3);

            _desiredRotRateR = (short)(pulsePerMeter * scaleRatio * rightV);
            _desiredRotRateL = (short)(pulsePerMeter * scaleRatio * leftV);

        }


        private void TrajectoryCircle()
        {
            double rNorm = normPoints(desiredX, desiredY, x_est, y_est);
            double rHead = Math.Atan2(y_est - desiredY, x_est - desiredX);
            double errR = rNorm - desiredR;
            double rotDes = Math.Atan(errR);


            desiredT = boundAngle(rHead + Math.PI / 2 + rotDes + 0.2);

            rotateToPoint(.3, 0);
        }


        private double limitMotorSpeed(double rightV, double leftV, double maxSpeed)
        {
            double leftRatio = Math.Max(1, Math.Abs(leftV) / maxSpeed);
            double rightRatio = Math.Max(1, Math.Abs(rightV) / maxSpeed);

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


        public static double boundAngle(double angle)
        {
            var newAngle = mod(angle, 2*Math.PI);

            if (newAngle > Math.PI)
                newAngle = newAngle - 2*Math.PI;

            return newAngle;
        }


        public static double mod(double x, double m)
        {
            double r = x % m;
            return r < 0 ? r + m : r;
        }

        // This function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            /*
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
            */

            double vel = 1.0;
            if (_currentWP == 13 || _currentWP == 14)
            {
                vel = 0.7;
                maxVelocity = 1.0;
            }
            else
            {
                vel = 1.0;
                maxVelocity = 1.0;
            }
            
            if (_currentWP == 4 || _currentWP == 8 || _currentWP == 10
                || _currentWP == 11)
            {
                vel = 1.5;
                maxVelocity = 1.5;
            }
            _currentWP = WaypointTrack(x_est, y_est, _currentWP, vel);
            //Console.WriteLine(maxVelocity);
        }


        private void TrackTrajectoryPRM()
        {
            double distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));
            
            Console.WriteLine("desX: {0}, desY:{1}",desiredX,desiredY);
            
            if (distToCurrentNode < 0.17 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                desiredX = trajList[trajCurrentNode].x;
                desiredY = trajList[trajCurrentNode].y;
                desiredT = 0;
            }

            FlyToSetPoint();
        }


        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node
            Node startNode = new Node(x_est, y_est, 0, 0);
            Node goalNode = new Node(desiredX, desiredY, 0, 0);
            Console.WriteLine("desX: {0}, desY:{1}", desiredX,desiredY);

            AddNode(startNode);


            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            int randCellNumber;
            int randNodeNumber;
            Node randExpansionNode;
            Random randGenerator = new Random();

            double randDistance = 0;
            double randOrientation = 0;

            double distancex = 0;
            double distancey = 0;
            

            while (iterations < maxIterations && !pathFound)
            {
                // Get expansion node
                randCellNumber = random.Next(0, numOccupiedCells);
                randNodeNumber = random.Next(0, numNodesInCell[occupiedCellsList[randCellNumber]]);
                randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];
                
                // Generate new node
                randDistance = 2*random.NextDouble();
                randOrientation = 2*Math.PI*random.NextDouble()-Math.PI;
                distancex = Math.Cos(randOrientation)*randDistance;
                distancey = Math.Sin(randOrientation)*randDistance;

                Node newNode = new Node(randExpansionNode.x + distancex, randExpansionNode.y + distancey, numNodes, randExpansionNode.nodeIndex);

                // Colllision checking
                var collision = map.CollisionFound(randExpansionNode, newNode, ROBOTRADIUS);
                if (!collision)
                {
                    AddNode(newNode);

                    // check to goal node
                    var collisionToGoal = map.CollisionFound(newNode, goalNode, ROBOTRADIUS);

                    if (!collisionToGoal)
                    {
                        goalNode.nodeIndex = numNodes;
                        goalNode.lastNode = newNode.nodeIndex;
                        AddNode(goalNode);
                        pathFound = true;
                    }
                }

               

                // Increment number of iterations
                iterations++;
            }


            // Create the trajectory to follow
            BuildTraj(goalNode);


            // ****************** Additional Student Code: End   ************




        }




        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;

            return;
        }



        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        
        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
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
            //_diffEncoderPulseL = ZeroOutlier(_diffEncoderPulseL, ref _lastDiffL, ref _zeroCounterL);
            //_diffEncoderPulseR = ZeroOutlier(_diffEncoderPulseR, ref _lastDiffR, ref _zeroCounterR);

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


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
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


        public void LocalizeEstWithParticleFilter()
        {
            pf.Predict();
            //count = (count + 1) % 10;

            var lastFewWayPoints = 5;

            HashSet<int> blackSet;
            if (_currentWP > _waypoints.Length - lastFewWayPoints)
                blackSet = blackList;
            else
                blackSet = new HashSet<int>();

            if (Math.Abs(_diffEncoderPulseL) > 0 || Math.Abs(_diffEncoderPulseR) > 0)// && count % 5 == 0)
            {
                pf.Correct(blackSet);
            }

            var estState = pf.EstimatedState();
            x_est = estState[0];
            y_est = estState[1];
            t_est = estState[2];
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        public double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        #endregion

    }
}
