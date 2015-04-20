using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Accord.MachineLearning;
using Accord.Statistics.Distributions.DensityKernels;



namespace DrRobot.JaguarControl
{
    public class Clustering
    {
        double[][] clust;
        double[][] partFull;
        Navigation n;

        public int[] labels;

        public Clustering(Navigation n) {
            this.n = n;

        }



        public double[][] FindClusters(double r)
        {

            clust = new double[n.numParticles][];
            partFull = new double[n.numParticles][];

            for (int i = 0; i < n.numParticles; i++)
            {
                clust[i] = new double[] { n.pf.particles[i].x, n.pf.particles[i].y };
                partFull[i] = new double[] { n.pf.particles[i].x, n.pf.particles[i].y, n.pf.particles[i].t, n.pf.particles[i].w };
            }
            UniformKernel kernel = new UniformKernel();
            MeanShift meanShift = new MeanShift(dimension: 2, kernel: kernel, bandwidth: 2*r);

            labels = meanShift.Compute(clust);
            double[][] clustersFound = new double[labels.Max()+1][];
            Array.Sort(labels, partFull);
            
            int count = 0;
            double xWeightedAvg = 0; 
            double yWeightedAvg = 0;
            double tWeightedAvg = 0;
            double wSum = 0;

            for (int i = 0; i < n.numParticles; i++)
            {
                if (labels[i] == count)
                {
                    xWeightedAvg += partFull[i][0] * partFull[i][3];
                    yWeightedAvg += partFull[i][1] * partFull[i][3];
                    tWeightedAvg += partFull[i][2] * partFull[i][3];
                    wSum += partFull[i][3];

                    if (i == partFull.Length - 1)
                    {
                        clustersFound[count] = new double[] { xWeightedAvg / wSum, yWeightedAvg / wSum, tWeightedAvg / wSum, wSum };
                    }

                }
                else
                {
                    double offset = 0.000000001;
                    clustersFound[count] = new double[] { xWeightedAvg / (wSum+offset), yWeightedAvg / (wSum+offset), tWeightedAvg / (wSum+offset), wSum };
                    count++;
                    
                        xWeightedAvg = partFull[i][0] * partFull[i][3];
                        yWeightedAvg = partFull[i][1] * partFull[i][3];
                        tWeightedAvg = partFull[i][2] * partFull[i][3];
                        wSum = partFull[i][3];
                    

        
                }

                
                }

            return clustersFound;
        }
    }
}
