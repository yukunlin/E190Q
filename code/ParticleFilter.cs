using System;
using System.Collections.Generic;
using System.Drawing.Text;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.Statistics;
using MathNet.Numerics.Integration;

namespace DrRobot.JaguarControl
{
    public class ParticleFilter
    {

        public Particle[] particles;
        public Map m;
        public Navigation n;
        public Random rand;
        private double scaleDist = 0.2;
        private double baseDist = 0.005;
        private double scaleAngle = 0.1;
        private double baseAngle = 0.005;

        private double alphaFast = 0.4;
        private double alphaSlow = 0.2;

        private double wslow = 0.000000000000000000000000000000000000000000000001;
        private double wfast = 0.000000000000000000000000000000000000000000000001;


        private double SD = 1;
        /////////////////////////////////////////////////////////////////////////////////////////// 
        //TODO: CHANGE TO LARGE NUMBER TO GET ONE BEAM, ALSO REDUCE TO ONE PARTICLE
        /////////////////////////////////////////////////////////////////////////////////////////// 
        public int SENSORSTEP = 15;

        public ParticleFilter(int numParticles, Navigation n, Map m)
        {
            particles = new Particle[numParticles];
            this.n = n;
            this.m = m;
            rand = new Random();

            // randomly assigns particle location
            for (int i = 0; i < numParticles; i++)
            {

                double x = ContinuousUniform.Sample(rand, m.minX, m.maxX);
                double y = ContinuousUniform.Sample(rand, m.minY, m.maxX);
                double t = ContinuousUniform.Sample(rand, -Math.PI, Math.PI);
                
                particles[i] = new Particle(x, y, t);
            }
        }

        public void Predict()
        {
            for (int i = 0; i < particles.Length; i++)
            {
                double errorDist = Normal.Sample(rand, 0, Math.Abs(scaleDist * n._distanceTravelled));
                double errorAngle = Normal.Sample(rand, 0, Math.Abs(scaleAngle * n._angleTravelled + baseAngle));
                double distTravelled = n._distanceTravelled + errorDist;
                double angTravelled = n._angleTravelled + errorAngle;
                particles[i].Propagate(distTravelled, angTravelled);
            }
        }

        private void WeighParticle(int p)
        {
            long[] laserData = n.LaserData;
            List<double> laserAbridged = new List<double>();
            List<double> laserAngles = new List<double>();
            double startAng = DrRobot.JaguarControl.JaguarCtrl.startAng;

            // abridged laser scan data
            for (int i = 0; i < laserData.Length; i = i + SENSORSTEP)
            {
                laserAbridged.Add(laserData[i]/1000.0); // converted to meters
                laserAngles.Add(n.laserAngles[i] - Math.PI / 2 );
            }

            // calculate expected value based on odometry
            List<double> expectedWallDist = new List<double>();
            for (int i = 0; i < laserAngles.Count; i++)
            {
                double x = particles[p].x;
                double y = particles[p].y;
                double t = laserAngles[i]+particles[p].t;
                expectedWallDist.Add(m.GetClosestWallDistance(x,y,t));
            }

            // calculated error term: expected - measured
            double error;
            double intThres = 0.001;
            const double minWeight = 0.01;
            double productWeight = 1;
            double weightPar;
            for (int i = 0; i < laserAngles.Count; i++)
            {
                error = expectedWallDist[i] - laserAbridged[i];
                if (laserAbridged[i] == Map.MAXLASERDISTANCE)
                {
                    //make normal distribution
                    var dist = new Normal(0, SD);
                    var wPar = SimpsonRule.IntegrateComposite(dist.Density, error - intThres, error + intThres, 10);

                    weightPar = wPar + minWeight;
                }

                else
                {
                    //make normal distribution
                    var dist = new Normal(0, SD);
                    var wPar = SimpsonRule.IntegrateComposite(dist.Density, error - intThres, error + intThres, 10);

                    weightPar = wPar + minWeight;
                }

                productWeight = productWeight * weightPar;
            }
            particles[p].w = productWeight;

        }


        private void Resample()
        {
            Particle[] resamplePar = new Particle[particles.Count()];
            double r = ContinuousUniform.Sample(rand, 0, 1.0/particles.Count());
            double c = particles[0].w;
            int i = 1;

            double randProba = Math.Max(0, 1.0 - wfast/wslow);
            //SD = 0.1 + 0.7*randProba; // Scale sigma

            Console.WriteLine("SD: {0}", SD);
            for (int m = 1; m <= particles.Count(); m++)
            {
                double u = r + (m - 1.0)/particles.Count();
                while (u > c)
                {
                    i++;
                    c = c + particles[i - 1].w;
                }
                double dice = ContinuousUniform.Sample(rand, 0, 1);

                if (dice <= randProba)
                {
                    double radSd = 10;
                    double thetaSd = 0.707;

                    double randX = Normal.Sample(rand, particles[i-1].x, radSd);
                    double randY = Normal.Sample(rand, particles[i-1].y, radSd);
                    double randt = ContinuousUniform.Sample(rand, -Math.PI, Math.PI);

                    Particle p = new Particle(randX, randY, randt);
                    p.w = 0.000001;
                    resamplePar[m-1] = p;
                }
                else
                {
                    Particle p = new Particle(particles[i-1].x,particles[i-1].y,particles[i-1].t);
                    p.w = particles[i - 1].w;
                    resamplePar[m-1]=(p);
                }
            }
            particles = resamplePar;
        }


        public void Correct()
        {
            // calculate all particle weights
            double weightAccum = 0;
            for (int i = 0; i < particles.Length; i++)
            {
                WeighParticle(i);
                weightAccum += particles[i].w;
            }

            double wAvg = weightAccum/particles.Length;
            wslow = wslow + alphaSlow*(wAvg - wslow);
            wfast = wfast + alphaFast*(wAvg - wfast);

            double scalingFactor = 1.0 / weightAccum;
            
            // normalize particle weights
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].w *= scalingFactor;
            }

            Resample();
        }


        public double [] EstimatedState()
        {
            double xAvg = 0;
            double yAvg = 0;
            double tAvg = 0;

            foreach (Particle par in particles)
            {
                xAvg += par.x*par.w;
                yAvg += par.y*par.w;
                tAvg += par.t*par.w;
            }

            double[] estState = {xAvg, yAvg, tAvg};

            return estState;
        }


        public class Particle
        {
            public double x, y, t, w;
            private double minWeight = 0.00001;

            public Particle(double x,double y,double t)
            {
                this.x = x;
                this.y = y;
                this.t = t;
                w = minWeight;
            }

            public void Propagate(double distTravelled, double angTravelled)
            {
                double distX = distTravelled * Math.Cos(t + angTravelled / 2);
                double distY = distTravelled * Math.Sin(t + angTravelled / 2);

                x = x + distX;
                y = y + distY;
                t = Navigation.boundAngle(t + angTravelled);
            }


        }

    }
}
