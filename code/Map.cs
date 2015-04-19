using System;
using System.Collections.Generic;
using System.IO;
using System.Drawing;
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
        private double[] intercepts;
        private double[] segmentSizes;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        private Dictionary<Point, HashSet<int>> _gridToSegments; 

        public Map()
        {
            // count number of lines in CSV file
            numMapSegments = File.ReadAllLines(@"C:\Users\Olivier\Desktop\extended_map.csv").Length;
            mapSegmentCorners = new double[numMapSegments,2,2];
            segmentSizes = new double[numMapSegments];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];

            // open CSV file
            var reader = new StreamReader(File.OpenRead(@"C:\Users\Olivier\Desktop\extended_map.csv"));
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

	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));


                // Set wall segments to be horizontal
                slopes[i] = (mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1]) / (0.001 + mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0]);
                intercepts[i] = mapSegmentCorners[i, 0, 1] - slopes[i] * mapSegmentCorners[i, 0, 0];

                // Set wall segment lengths
                segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0], 2) + Math.Pow(mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1], 2));

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }

            // Initialize dictionary mapping grid to list of line segments in grid
            _gridToSegments = new Dictionary<Point, HashSet<int>>();

            // Add segments to grid
            for (int i = 0; i < numMapSegments; i++)
            {
                AddSegmentToGrid(i);
            }
        }


        public List<int> SegmentsWithinRadius(double x, double y, double radius)
        {
            var start = DiscretizePoint(x, y);
            var roundedRadius = (int) Math.Ceiling(radius);
            var segmentsInRadius = new HashSet<int>();

            var checkedGrid = new HashSet<Point>();
            var queue = new Queue<Point>();

            queue.Enqueue(start);
            checkedGrid.Add(start);

            while (queue.Count > 0)
            {
                // dequeue and add to hashset
                var cur = queue.Dequeue();

                // get line segments
                var lineSegments = GetSegmentsInGrid(cur);
                segmentsInRadius.UnionWith(lineSegments);

                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        var n = new Point(cur.X + i, cur.Y + j);
                        var dist = Navigation.normPoints(start.X, start.Y, n.X, n.Y);

                        // add to queue only if not checked yet and within radius
                        if (!checkedGrid.Contains(n) && dist < roundedRadius)
                        {
                            checkedGrid.Add(n);
                            queue.Enqueue(n);
                        }
                    }
                }
            }

            return segmentsInRadius.ToList();
        }


        private HashSet<int> GetSegmentsInGrid(Point gridPoint)
        {
            if (_gridToSegments.ContainsKey(gridPoint))
            {
                return _gridToSegments[gridPoint];
            }
            
            return new HashSet<int>();
        }


        private void AddSegmentToGrid(int segementId)
        {
            var x1 = mapSegmentCorners[segementId, 0, 0];
            var y1 = mapSegmentCorners[segementId, 0, 1];
            var x2 = mapSegmentCorners[segementId, 1, 0];
            var y2 = mapSegmentCorners[segementId, 1, 1];

            var segmentLength = Navigation.normPoints(x1, y1, x2, y2);
            
            // direction vector from (x1, y1) to (x2, y2)
            var dx = (x2 - x1)/segmentLength;
            var dy = (y2 - y1)/segmentLength;

            // parametic equation argument
            double t = 0;
            var currentX = x1;
            var currentY = y1;

            // Add start point
            AddToDictionary(DiscretizePoint(currentX, currentY), segementId);

            while (t < segmentLength)
            {
                // move forward 1 unit towards (x2, y2)
                t++;
                currentX += dx;
                currentY += dy;

                AddToDictionary(DiscretizePoint(currentX, currentY), segementId);
            } 

            // Add end point
            AddToDictionary(DiscretizePoint(x2, y2), segementId);
        }


        private Point DiscretizePoint(double x, double y)
        {
            return new Point((int)Math.Round(x), (int)Math.Round(y));
        }


        private void AddToDictionary(Point key, int segmentId)
        {
            if (_gridToSegments.ContainsKey(key))
            {
                var hashSet = _gridToSegments[key];
                hashSet.Add(segmentId);
            }
            else
            {
                var hashSet = new HashSet<int>();
                hashSet.Add(segmentId);
                _gridToSegments.Add(key, hashSet);
            }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){
            // get wall points
            //return MAXLASERDISTANCE;
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
                return Math.Pow(MAXLASERDISTANCE, 2);

            double tlaser = (xm*y - x*ym + wallX1*ym - xm*wallY1)/ laserD;
            double twall = ((y-wallY1)*Math.Cos(t) + (-x+wallX1)* Math.Sin(t)) / wallD;

            // intersection in wrong direction
            if (tlaser < 0)
                return Math.Pow(MAXLASERDISTANCE, 2);

            // intersection not on wall segment
            if (twall < 0 || twall > wallLength)
                return Math.Pow(MAXLASERDISTANCE, 2);

            // intersection points
            double xInt = x + Math.Cos(t)*tlaser;
            double yInt = y + Math.Sin(t)*tlaser;

            return Navigation.squareSumPoints(x, y, xInt, yInt);
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t, List<int> segments) {

            double minDist = Math.Pow(MAXLASERDISTANCE,2);
            int minInd = 0;

            foreach (var i in segments)
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

	        return Math.Sqrt(minDist);
        }

       

        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol) {

            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }

	        
	        return false;
        }

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y)
        {
            // Set wall vars
            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];
            double dist = 9999;

            // Put code here to calculated dist.
            // Calculate slope and intercept
            double angleSegmentPerpendicular = Math.PI / 2 + Math.Atan((Y2 - Y1) / (0.000001 + X2 - X1));
            double m = Math.Tan(angleSegmentPerpendicular);
            double b = y - m * x;

            // Get line intersection
            double x_intersect = (b - intercepts[segment]) / (slopes[segment] - m);
            double y_intersect = m * x_intersect + b;

            // Check for horiz/vert slopes
            if (Math.Abs(Y2 - Y1) < 0.001)
                y_intersect = Y1;
            if (Math.Abs(X2 - X1) < 0.001)
                x_intersect = X1;


            // Check to see if intersection LIES within segment
            double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect - X1, 2) + Math.Pow(y_intersect - Y1, 2));
            double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect - X2, 2) + Math.Pow(y_intersect - Y2, 2));
            if (dist_intersect_corner1 <= (segmentSizes[segment] + tol) && dist_intersect_corner2 <= (segmentSizes[segment] + tol))
            {
                dist = Math.Sqrt(Math.Pow(x - x_intersect, 2) + Math.Pow(y - y_intersect, 2));
            }

            // Check for distance to corners (for case where no intersection with segment
            double dist_point_corner1 = Math.Sqrt(Math.Pow(x - X1, 2) + Math.Pow(y - Y1, 2));
            double dist_point_corner2 = Math.Sqrt(Math.Pow(x - X2, 2) + Math.Pow(y - Y2, 2));
            dist = Math.Min(dist, dist_point_corner1);
            dist = Math.Min(dist, dist_point_corner2);

            return dist;
        }




        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

    }
}
