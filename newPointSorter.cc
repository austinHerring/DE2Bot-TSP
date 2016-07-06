#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;

#define DISTANCE_BETWEEN_WHEELS 224 //robot units
#define FFAST 512 //ticks
#define ROBOT_UNITS_PER_FT 290.2857143
#define THRESHOLD_ANGLE_MAX 20 //maximum angle to arc
#define THRESHOLD_RADIUS_MIN 10 //minimum radius to arc

class point {
public:
    double x;
    double y;
    int number;
    int priority;
    double distance(point);
};

class line {
public:
    point a;
    point b;
    double slope();
    double intercept();
    point intersect(line);
    line perpBisector();
};

class circle {
public:
    point center;
    double radius;
};

double angle(point A, point B, point C);
double angleDiff(double currAngle, point currPoint, point destPoint);
double arcLengthAB(point A, point B, point C);
double arcLengthBC(point A, point B, point C);
double arcLengthAC(point A, point B, point C);
circle circleFromPoints(point A, point B, point C);
double calculateSlowWheelSpeed(point A, point B, point C);

double point::distance(point otherPoint){
    double distance = sqrt(pow((this->x)-otherPoint.x,2) + pow((this->y)-otherPoint.y,2));
    return distance;
}

double line::slope(){
    double mySlope;
    mySlope = ((a.y-b.y)/(a.x-b.x));
    return mySlope;
}

double line::intercept(){
    double myIntercept;
    myIntercept = a.y - this->slope() * a.x;
    return myIntercept;
}

point line::intersect(line otherLine){
    point intersection;
    intersection.x = 0;
    intersection.y = 0;
    if(this->slope() == otherLine.slope()){
        cout << "Lines are parallel, no possible circle." << endl;
        return intersection;
    }
    intersection.x = (otherLine.intercept() - this->intercept())/(this->slope()-otherLine.slope());
    intersection.y = this->slope() * intersection.x + this->intercept();
    return intersection;
}

line line::perpBisector(){
    line myBisector;
    double mySlope;
    point midpoint;
    point otherPoint;
    mySlope = -1/this->slope();
    midpoint.x = (this->a.x + this->b.x)/2;
    midpoint.y = (this->a.y + this->b.y)/2;
    myBisector.a = midpoint;
    otherPoint.x = midpoint.x + 100; //arbitrary new x value
    otherPoint.y = mySlope*(otherPoint.x-midpoint.x)+midpoint.y;
    myBisector.b = otherPoint;
    return myBisector;
}

// double oldAngle(point A, point B, point C){
//     double theta;
//     double AB = A.distance(B);
//     double BC = B.distance(C);
//     double AC = A.distance(C);
//     theta = acos((pow(AC,2)-pow(AB,2)-pow(BC,2))/(-2*AB*BC));
//     return theta;
// }

double angle(point A, point B, point C){
    double v1x = A.x - B.x;
    double v1y = A.y - B.y;
    double v2x = C.x - B.x;
    double v2y = C.y - B.y;
    double theta = atan2(v1x, v1y) - atan2(v2x, v2y);
    theta = theta * 180 / M_PI;
    if (theta < 0){
        theta += 360;
    }
    return theta;
}

double angleDiff(double currAngle, point currPoint, point destPoint){
    double output;
    point newPoint;
    double theta = currAngle * M_PI / 180;
    newPoint.x = currPoint.x + 10*cos(theta);
    newPoint.y = currPoint.y + 10*sin(theta);
    output = angle(newPoint, currPoint, destPoint);
    return output;
}

double arcLengthAB(point A, point B, point C){
    circle myCircle;
    double arcAngle;
    double arcLength;
    myCircle = circleFromPoints(A,B,C);
    arcAngle = angle(A,myCircle.center,B);
    arcLength = 2*M_PI*myCircle.radius*arcAngle/360;
    return arcLength;
}

double arcLengthBC(point A, point B, point C){
    circle myCircle;
    double arcAngle;
    double arcLength;
    myCircle = circleFromPoints(A,B,C);
    arcAngle = angle(B,myCircle.center,C);
    arcLength = 2*M_PI*myCircle.radius*arcAngle/360;
    return arcLength;
}

double arcLengthAC(point A, point B, point C){
    circle myCircle;
    double arcAngle;
    double arcLength;
    myCircle = circleFromPoints(A,B,C);
    arcAngle = angle(A,myCircle.center,C);
    arcLength = 2*M_PI*myCircle.radius*arcAngle/360;
    return arcLength;
}

circle circleFromPoints(point A, point B, point C){
    circle output;
    line AB;
    line BC;
    point center;
    double radius;
    AB.a = A;
    AB.b = B;
    BC.a = B;
    BC.b = C;
    center = AB.perpBisector().intersect(BC.perpBisector());
    radius = center.distance(A);
    output.center = center;
    output.radius = radius;
    return output;
}

double calculateSlowWheelSpeed(point A, point B, point C){
    double slowWheelSpeed;
    double myRadius;
    circle myCircle;
    myCircle = circleFromPoints(A,B,C);
    myRadius = myCircle.radius*ROBOT_UNITS_PER_FT;
    slowWheelSpeed = ((2*FFAST*myRadius)/(112+myRadius))-FFAST;
    return slowWheelSpeed;
}

int main() {
    point givenPoints[12];
    point pointSet[13];
    int bestDestination;
    double bestDistance = 10000000000;
    bool inThreshold;
    double currAngle = 0;
    double myAngle;
    circle myCircle;
    int arcDirection;
    double slowWheelSpeed;
    double arcLength;

    for(int i=0; i<12; i++){
        cout << "x" << i+1 << ": ";
        cin >> givenPoints[i].x;
        cout << "y" << i+1 << ": ";
        cin >> givenPoints[i].y;
        givenPoints[i].priority = 0;
    }

    cout << endl;

    pointSet[0].x = 0;
    pointSet[0].y = 0;

    for(int i=0; i<12; i++){
        for(int j=0; j<12; j++){
            if(givenPoints[j].distance(pointSet[i]) < bestDistance){
                if (givenPoints[j].priority == 0){
                    bestDestination = j;
                    bestDistance = givenPoints[j].distance(pointSet[i]);
                }
            }
        }
        givenPoints[bestDestination].priority = i+1;
        pointSet[i+1].x = givenPoints[bestDestination].x;
        pointSet[i+1].y = givenPoints[bestDestination].y;
        pointSet[i+1].priority = i+1;
        pointSet[i+1].number = bestDestination+1;
        bestDistance = 10000000000;
    }

    ofstream myfile;
    myfile.open("BotAssembly.asm", ios::in | ios::out | ios::app);

    myfile << endl << endl << ";----------auto-generated code from newPointSorter.cc----------" << endl << endl;

    myfile << "TaskList:" << endl;

    //iterate through pointSet
    for(int i=0; i<12; i++){
        //check threshold
        myAngle = angleDiff(currAngle, pointSet[i], pointSet[i+1]);
        if(i<11){
            myCircle = circleFromPoints(pointSet[i],pointSet[i+1],pointSet[i+2]);
            if(myCircle.radius > THRESHOLD_RADIUS_MIN && myAngle < THRESHOLD_ANGLE_MAX){
                inThreshold = true;
            }else{
                inThreshold = false;
            }
        }else{
            inThreshold = false;
        }

        if(inThreshold){ //if arc
            //determine starting angle
            //print turn to angle
            myfile << "DW: 0 ;turn to angle" << endl;
            //print angle
            //print arc
            myfile << "DW: 2 ;drive arc" << endl;
            //determine left or right arc
            //print left or right arc
            if(arcDirection == 1){
                myfile << "DW: 1 ;arc right" << endl;
            }else{
                myfile << "DW: 0 ;arc left" << endl;
            }
            //calculate slow wheel speed
            //print slow wheel speed
            myfile << "DW: " << slowWheelSpeed << " ;slow wheel speed" << endl;
            //calculate 1st arc length
            //print 1st arc length
            myfile << "DW: " << arcLength << " ;1st arc length" << endl;
            //print announce
            myfile << "DW: 3 ;announce" << endl;
            //print point number
            myfile << "DW: " << hex << pointSet[i+1].number << " ;point number " << dec << pointSet[i+1].number << endl;
            //calculate 2nd arc length
            //print 2nd arc length
            myfile << "DW: " << arcLength << " ;2nd arc length" << endl;
            //print announce
            myfile << "DW: 3 ;announce" << endl;
            //print point number
            myfile << "DW: " << hex << pointSet[i+2].number << " ;point number " << dec << pointSet[i+2].number << endl;
            //advance points (twice?)
        }else{ //if not arc
            //deturmine destination angle
            //print turn to angle
            myfile << "DW: 0 ;turn to angle" << endl;
            //print angle
            //print go the distance
            myfile << "DW: 1 ;go the distance" << endl;
            //print the distance
            //print announce
            myfile << "DW: 3 ;announce" << endl;
            //print point number
            myfile << "DW: " << hex << pointSet[i+1].number << " ;point number " << dec << pointSet[i+1].number << endl;
            //advance points (once?)
        }
    }

    //print die

    myfile.close();
    return 0;
}
