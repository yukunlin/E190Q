using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;

        public const double MAXLASERDISTANCE = 5.6;
        private double[] slopes;
        private double[] segmentSizes;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        public Map()
        {

            // count number of lines in CSV file
            numMapSegments = File.ReadAllLines(@"C:\Users\CAPCOM\Desktop\map.csv").Length;
            mapSegmentCorners = new double[numMapSegments,2,2];
            segmentSizes = new double[numMapSegments];

            // open CSV file
            var reader = new StreamReader(File.OpenRead(@"C:\Users\CAPCOM\Desktop\map.csv"));
            int r = 0;

            // read line by line
            while (!reader.EndOfStream)
            {
                var line = reader.ReadLine();
                var values = line.Split(',');
                mapSegmentCorners[r, 0, 0] = Convert.ToDouble(values[0]);
                mapSegmentCorners[r, 0, 1] = Convert.ToDouble(values[1]);
                mapSegmentCorners[r, 1, 0] = Convert.ToDouble(values[2]);
                mapSegmentCorners[r, 1, 1] = Convert.ToDouble(values[3]);
                r++;
            }
            reader.Close();

             
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }



        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){
            // get wall points
            double wallX1 = mapSegmentCorners[segment, 0, 0];
            double wallY1 = mapSegmentCorners[segment, 0, 1];
            double wallX2 = mapSegmentCorners[segment, 1, 0];
            double wallY2 = mapSegmentCorners[segment, 1, 1];

            double wallLength = Navigation.normPoints(wallX1, wallY1, wallX2, wallY2);
            double xm = (wallX2 - wallX1)/wallLength;
            double ym = (wallY2 - wallY1)/wallLength;

            double laserD = ym*Math.Cos(t) - xm*Math.Sin(t);
            double wallD = ym*Math.Cos(t) - xm*Math.Sin(t);

            if (laserD == 0 || wallD == 0)
                return MAXLASERDISTANCE;

            double tlaser = (xm*y - x*ym + wallX1*ym - xm*wallY1)/ laserD;
            double twall = ((y-wallY1)*Math.Cos(t) + (-x+wallX1)* Math.Sin(t)) / wallD;

            // intersection in wrong direction
            if (tlaser < 0)
                return MAXLASERDISTANCE;

            // intersection not on wall segment
            if (twall < 0 || twall > wallLength)
                return MAXLASERDISTANCE;

            // intersection points
            double xInt = x + Math.Cos(t)*tlaser;
            double yInt = y + Math.Sin(t)*tlaser;

            return Navigation.normPoints(x, y, xInt, yInt);
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){

	        // ****************** Additional Student Code: Start ************

	        // Put code here that loops through segments, calling the
	        // function GetWallDistance.
            double minDist = MAXLASERDISTANCE;
            int minInd = 0;

            for (int i = 0; i < numMapSegments; i++)
            {
                double wallDist = GetWallDistance(x, y, t, i);
                minDist = Math.Min(minDist, wallDist);
                if (wallDist == minDist)
                    minInd = i;
            }

            double wallX1 = mapSegmentCorners[minInd, 0, 0];
            double wallY1 = mapSegmentCorners[minInd, 0, 1];
            double wallX2 = mapSegmentCorners[minInd, 1, 0];
            double wallY2 = mapSegmentCorners[minInd, 1, 1];
	        // ****************** Additional Student Code: End   ************

	        return minDist;
        }

       


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        bool CollisionFound(double n1x, double n1y, double n2x, double n2y, double tol){



	        
	        return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

    }
}
