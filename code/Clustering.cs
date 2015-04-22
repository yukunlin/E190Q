using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Accord.MachineLearning;
using Accord.Statistics.Distributions.DensityKernels;
using System.Diagnostics;


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
            Array.Sort(labels, partFull);

            double[] labelsClean = new double[n.numParticles];
            var prev = 0;
            int counter = 0;
            for (int i = 0; i < n.numParticles; i++)
            {
                if (prev == labels[i])
                {
                    labelsClean[i] = counter;
                }
                else
                {
                    counter++;
                    labelsClean[i] = counter;
                    prev = labels[i];
                }
            }
            
            var setOfLabel = new HashSet<int>(labels);
            var uniqueLabels = setOfLabel.Count;
            double[][] clustersFound = new double[uniqueLabels][];

            
            int count = 0;
            double xWeightedAvg = 0; 
            double yWeightedAvg = 0;
            double tWeightedAvg = 0;
            double wSum = 0;
            double offset = 0.000000001;
            int track = 0;


            var maxLabel = labelsClean[labelsClean.Length - 1];

            //Console.WriteLine("unique: {0}, max:{1}", uniqueLabels, maxLabel);

            Debug.Assert(maxLabel + 1 == uniqueLabels);

            //Console.WriteLine(maxLabel + 1 == uniqueLabels);
            //Console.WriteLine("unique labels: {0}", uniqueLabels);
            for (int i = 0; i < n.numParticles; i++)
            {
                if (labelsClean[i] == count)
                {
                    xWeightedAvg += partFull[i][0] * partFull[i][3];
                    yWeightedAvg += partFull[i][1] * partFull[i][3];
                    tWeightedAvg += partFull[i][2] * partFull[i][3];
                    wSum += partFull[i][3];

                    if (i == partFull.Length - 1)
                    {
                        clustersFound[count] = new double[] { xWeightedAvg / (wSum + offset), yWeightedAvg / (wSum + offset), tWeightedAvg / (wSum + offset), wSum };
                    }

                }
                else
                {

                    clustersFound[count] = new double[] { xWeightedAvg / (wSum+offset), yWeightedAvg / (wSum+offset), tWeightedAvg / (wSum+offset), wSum };
                    count++;
                    track++;
                    
                        xWeightedAvg = partFull[i][0] * partFull[i][3];
                        yWeightedAvg = partFull[i][1] * partFull[i][3];
                        tWeightedAvg = partFull[i][2] * partFull[i][3];
                        wSum = partFull[i][3];

                        if (i == partFull.Length - 1)
                        {
                            clustersFound[count] = new double[] { xWeightedAvg / (wSum + offset), yWeightedAvg / (wSum + offset), tWeightedAvg / (wSum + offset), wSum };
                        }
                   }

                
                }

            Debug.Assert(clustersFound.Length > 0);
            return clustersFound;
        }
    }
}
