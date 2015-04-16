using System;
using System.Collections.Generic;
using System.Drawing.Text;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.Providers.LinearAlgebra;
using MathNet.Numerics.Statistics;
using MathNet.Numerics.Integration;

namespace DrRobot.JaguarControl
{
    public class ParticleFilter_GKVK
    {
        public Particle[] particles, propagatedParticles;
        public Map map;
        public Navigation navigation;
        public Random rand;

        public int[] weightedParticleNums;
        const int MAXWEIGHT = 30;

        public int SENSORSTEP = 12;
        const double MAXERROR = 2000 * 2000; // 3000000
        public const double DISPERSION_RATE = 0.2;
        public int DISPERSION;

        public double K_wheelRandomness = 0.5;//1;//0.15;//0.25
        public double K_posRandomness = 0.1;

        public struct Particle
        {
            public double x, y, t, w;
        }

        public ParticleFilter_GKVK(int numParticles, Navigation n, Map m)
        {
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            this.navigation = n;
            this.map = m;
            rand = new Random();
            //dist = new Normal(0, SD);

            DISPERSION = (int)(numParticles * DISPERSION_RATE);
            weightedParticleNums = new int[MAXWEIGHT * numParticles];

            double x, y, t;
            // randomly assigns particle location
            for (int i = 0; i < numParticles; i++)
            {
                /*
                double x = ContinuousUniform.Sample(rand, m.minX, m.maxX);
                double y = ContinuousUniform.Sample(rand, m.minY, m.maxY);
                double t = ContinuousUniform.Sample(rand, -Math.PI, Math.PI);
                */
                
                x = ContinuousUniform.Sample(rand, -1, 1) -3;
                y = ContinuousUniform.Sample(rand, -1, 1) - 8;
                t = ContinuousUniform.Sample(rand, -Math.PI / 10, Math.PI / 10);//-1.57;
                
                particles[i] = new Particle();
                particles[i].x = x;
                particles[i].y = y;
                particles[i].t = t;

            }
        }

        public void Predict()
        {
            double newWheelDistanceL, newWheelDistanceR;
            double dTravelled, aTravelled, tForTravel;
            double xrandom, yrandom;

            for (int i = 0; i < particles.Length; i++)
            {
                /*
                double errorDist = Normal.Sample(rand, 0, Math.Abs(scaleDist * n._distanceTravelled));
                double errorAngle = Normal.Sample(rand, 0, Math.Abs(scaleAngle * n._angleTravelled + baseAngle));
                double distTravelled = n._distanceTravelled + errorDist;
                double angTravelled = n._angleTravelled + errorAngle;
                particles[i].Propagate(distTravelled, angTravelled);*/

                newWheelDistanceL = navigation._wheelDistanceL + navigation.RandomGaussian() * K_wheelRandomness * navigation._wheelDistanceL;
                newWheelDistanceR = navigation._wheelDistanceR + navigation.RandomGaussian() * K_wheelRandomness * navigation._wheelDistanceR;

                dTravelled = (newWheelDistanceL + newWheelDistanceR) / 2;
                aTravelled = (newWheelDistanceR - newWheelDistanceL) / (2 * (navigation.ROBOTRADIUS));

                tForTravel = particles[i].t + aTravelled * 0.5;
                tForTravel = navigation.constrainAngle(tForTravel);

                xrandom = K_posRandomness * dTravelled * navigation.RandomGaussian();
                yrandom = K_posRandomness * dTravelled * navigation.RandomGaussian();

                particles[i].x = particles[i].x + dTravelled * Math.Cos(tForTravel) + xrandom;
                particles[i].y = particles[i].y + dTravelled * Math.Sin(tForTravel) + yrandom;
                particles[i].t = navigation.constrainAngle(particles[i].t + aTravelled);
            }
        }

        private void WeighParticle(int p)
        {
            double ppx = particles[p].x;
            double ppy = particles[p].y;
            double ppt = particles[p].t;
            //if (map.inKeepout(ppx, ppy))
            //    return -100.0;

            double weight = 0;

            double error = 0;
            double errorTemp1 = 0, errorTemp2 = 0, errorTemp3 = 0;
            double errorTemp = 0;

            double estimate1, estimate2, estimate3;
            double measurement, angle;
            //int step = 8;
            int n = 0;//LaserData.Length/(step*laserStepSize) + 1;

            //estimate1 = (1000 * map.GetClosestWallDistance(ppx, ppy, ppt - 1.57 + laserAngles[0]));
            for (int i = 0; i < navigation.LaserData.Length; i = i + SENSORSTEP)
            {
                angle = navigation.laserAngles[i];
                estimate1 = (1000 * map.GetClosestWallDistance(ppx, ppy, ppt - 1.57 + angle - 0.1));
                estimate2 = (1000 * map.GetClosestWallDistance(ppx, ppy, ppt - 1.57 + angle));
                estimate3 = (1000 * map.GetClosestWallDistance(ppx, ppy, ppt - 1.57 + angle + 0.1));

                measurement = navigation.LaserData[i];
                //measurement

                // Laser Scanner returns small (<50) values when it doesn't see a wall
                if (measurement < 50)
                {
                    // If the estimated distance is greater than 2000, add no error,
                    if (estimate1 < 2000)
                    {
                        errorTemp = Math.Pow(measurement - estimate1, 2);
                        if (errorTemp > MAXERROR)
                            errorTemp1 += 1.1 * MAXERROR;
                        else
                            errorTemp1 += 1.1 * errorTemp;
                    }
                    if (estimate2 < 2000)
                    {
                        errorTemp = Math.Pow(measurement - estimate2, 2);
                        if (errorTemp > MAXERROR)
                            errorTemp2 += MAXERROR;
                        else
                            errorTemp2 += errorTemp;
                    }
                    if (estimate3 < 2000)
                    {
                        errorTemp = Math.Pow(measurement - estimate3, 2);
                        if (errorTemp > MAXERROR)
                            errorTemp3 += 1.1 * MAXERROR;
                        else
                            errorTemp3 += 1.1 * errorTemp;
                    }
                }
                else //else, it's a normal measurement..
                {
                    errorTemp = Math.Pow(measurement - estimate1, 2);
                    if (errorTemp > MAXERROR)
                        errorTemp1 += 1.1 * MAXERROR;
                    else
                        errorTemp1 += 1.1 * errorTemp;
                    errorTemp = Math.Pow(measurement - estimate2, 2);
                    if (errorTemp > MAXERROR)
                        errorTemp2 += MAXERROR;
                    else
                        errorTemp2 += errorTemp;
                    errorTemp = Math.Pow(measurement - estimate3, 2);
                    if (errorTemp > MAXERROR)
                        errorTemp3 += 1.1 * MAXERROR;
                    else
                        errorTemp3 += 1.1 * errorTemp;
                }
                ++n;

            }
            //error = Math.Min(errorTemp1, errorTemp2);
            //error = Math.Min(error, errorTemp3);
            error = (errorTemp1 < errorTemp2) ? ((errorTemp1 < errorTemp3) ? errorTemp1 : errorTemp3) : ((errorTemp2 < errorTemp3) ? errorTemp2 : errorTemp3);

            //double sigma = 1440000 * n; // 1.2m*1.2m//0.5 m^2 * n
            weight = 1 - error / (1440000 * n);

            particles[p].w = weight;
        }

        private void Resample()
        {
            //Resample Particles, filled weightedParticleNums vector
            int n = 0;
            //double wi;
            int weight;
            for (int i = 0; i < particles.Length; ++i)
            {
                WeighParticle(i);
                propagatedParticles[i] = particles[i];

                weight = (int)(MAXWEIGHT * particles[i].w);
                do
                {
                    weightedParticleNums[n] = i;
                    ++n;
                    --weight;
                } while (weight > 0);
            }

            double x, y, t;

            // Still resampling particles, choose new particles from weightedParticleNums vectors
            for (int i = 0; i < particles.Length; ++i)
            {
                //int r = (int)(random.NextDouble() * n);
                particles[i] = propagatedParticles[weightedParticleNums[(int)(rand.NextDouble() * n)]];
                //if (i < numParticles * 0.2 && particles[i].w < -1)
                if (i < DISPERSION && particles[i].w < 0)
                {
                    //SetRandomPos(i); // Randomize particles with 20% chance if weight < 0

                    x = ContinuousUniform.Sample(rand, -1, 1) + particles[i].x;
                    y = ContinuousUniform.Sample(rand, -1, 1) + particles[i].y;
                    t = ContinuousUniform.Sample(rand, -Math.PI / 10, Math.PI / 10);//-1.57;

                    particles[i].x = x;
                    particles[i].y = y;
                    particles[i].t = t;
                }
            }
        }

        public void Correct()
        {
            Resample();
        }


        public double[] EstimatedState()
        {
            double xAvg = 0;
            double yAvg = 0;
            double tAvgX = 0;
            double tAvgY = 0;
            double tAvg;

            double weight;
            for (int i = 0; i < particles.Length; ++i)
            {
                weight = particles[i].w;
                weight = (weight < 0) ? 0 : weight;
 
                xAvg += particles[i].x * weight;
                yAvg += particles[i].y * weight;
                //tAvg += par.t*par.w;
                tAvgX += Math.Cos(particles[i].t) * weight;
                tAvgY += Math.Sin(particles[i].t) * weight;
            }

            tAvg = Math.Atan2(tAvgY, tAvgX);

            double[] estState = { xAvg, yAvg, tAvg };

            return estState;
        }
    }    
}