//taken from http://sandbox.mc.edu/~bennet/cs220/codeex/cl0_cc.html
#include<math.h>
// Class to represent points.
class Point {
	private:
        double xval, yval;
	public:
        // Constructor uses default arguments to allow calling with zero, one,
        // or two values.
        Point(double x = 0.0, double y = 0.0) {
                xval = x;
                yval = y;
        }

        // Extractors.
        double x() { return xval; }
        double y() { return yval; }

        // Distance to another point.  Pythagorean thm.
        double dist(Point other) {
                double xd = xval - other.xval;
                double yd = yval - other.yval;
                return sqrt(xd*xd + yd*yd);
        }

        // Add or subtract two points.
        Point add(Point b)
        {
                return Point(xval + b.xval, yval + b.yval);
        }
        Point sub(Point b)
        {
                return Point(xval - b.xval, yval - b.yval);
        }
			Point scale(double scale)
        {
                return Point(xval*scale, yval*scale);
        }

        // Move the existing point.
        void move(double a, double b)
        {
                xval += a;
                yval += b;
        }
		
		//rounds off values
		void round(int decPlaces = 0)
		{
			double remX, remY;
			remX = xval - floor(xval);
			remY = yval - floor(yval); 
			
			int multiplier = pow(10,decPlaces);//used to round off 
	
			xval *= multiplier;
			xval = (remX >= 0.5) ? ceil(xval) : floor(xval);
			xval /= multiplier;

			yval *= multiplier;
			yval = (remY >= 0.5) ? ceil(yval) : floor(yval);
			yval /= multiplier;
		}

		//prints whether or not the point is valid (both coordinates are positive)
		bool isValid()
		{
			return ((xval < 0) ? false : true);
		}

      // Print the point on the stream.  The class ostream is a base class
      // for output streams of various types.
      /*void print(std::ostream &strm)
      {
           strm << "(" << xval << "," << yval << ")";
      }*/
};
