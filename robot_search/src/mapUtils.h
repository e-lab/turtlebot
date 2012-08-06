//typedef boost::numeric::ublas::matrix<int> matrix;


Point botPosToMatrix(Point, double, double, double);
int readMap(const signed char*, int, int, int);
signed char* cloneMap(const signed char*, int, int);
void writeMap(signed char*, int, int, int, signed char);
void writeSquare(signed char*, int, int, int, int);
bool isSolid(const signed char*, int, int, int);
void paintLine(signed char*, int, std::list<Point>, int);
void printMap(signed char*, int, int);
int randint(int, int);
Point matrixToMapPos(Point, double, double, double);
