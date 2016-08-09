#include "geometry2D.h"
#include <cmath>
#include <cassert>
#include "sp_matrix.h"
#include "stdlib.h"
#include "stdio.h"

using namespace std;

#if 0
double pi_to_pi(double angle) 
// An alternative implementation that uses fmod() rather than while-loops.
{
	angle = fmod(angle, 2.*PI);
	if (angle < -PI)
		angle += 2.*PI;
	else if (angle > PI)
		angle -= 2.*PI;
	return angle;
}
#endif

double dist(const Point& p, const Point& q) 
// Distance between two points.
{ 
	return sqrt(dist_sqr(p,q)); 
}

//Pose compute_relative_pose(const vector<Point>& a, const vector<Point>& b)
Pose compute_relative_pose(const vector<nrr_TrICP_Pack>& Pack, double eps)
// Determine the relative Pose that best aligns two point-sets.
// INPUTS: a, b are two equal-length sets of points, such that a[i] aligns with b[i].
// OUTPUT: Best-fit alignment.
//
// Closed-form algorithm from Appendix C of:
// F. Lu and E. Milios. Robot pose estimation in unknown environments by matching
// 2D range scans. Journal of Intelligent and Robotic Systems, 18:249275, 1997.
{
//	assert(a.size() == b.size() && a.size() != 0);

	int n= Pack.size()*eps;
	
//	printf(">>>> %d = %d * %f \n",n,Pack.size(),eps);
	double x1,x2,y1,y2,xx,yy,xy,yx;

	x1=x2=y1=y2=xx=yy=xy=yx=0.0;
	for (int i=0; i<n; ++i) { // calculate sums
		const Point& p1 = Pack[i].a;
		const Point& p2 = Pack[i].b;

		x1 += p1.x;
		x2 += p2.x;
		y1 += p1.y;
		y2 += p2.y;
		xx += p1.x*p2.x;
		yy += p1.y*p2.y;
		xy += p1.x*p2.y;
		yx += p1.y*p2.x;
	}

	double N = static_cast<double>(n);

	double Sxx = xx - x1*x2/N; // calculate S
	double Syy = yy - y1*y2/N;
	double Sxy = xy - x1*y2/N;
	double Syx = yx - y1*x2/N;

	double xm1 = x1/N; // calculate means
	double xm2 = x2/N; 
	double ym1 = y1/N; 
	double ym2 = y2/N; 

	double phi = atan2(Sxy-Syx, Sxx+Syy); 
	
	Pose pse; // calculate pose
	pse.p.x = xm2 - (xm1*cos(phi) - ym1*sin(phi));
	pse.p.y = ym2 - (xm1*sin(phi) + ym1*cos(phi));
	pse.phi = phi;

	return pse;
}

// ************************
// Function to do the least-squares but optimized for the metric
// ************************

Pose /*computeMatrixLMSOpt*/MbICP_compute_relative_pose(/*TAsoc *cp_ass, int cnt, Tsc *estimacion*/ const vector<nrr_TrICP_Pack>& Pack, double eps) 
{
   int MAXLASERPOINTS = Pack.size()*eps;
   int i;
   float LMETRICA2;
   float X1[MAXLASERPOINTS], Y1[MAXLASERPOINTS];
   float X2[MAXLASERPOINTS],Y2[MAXLASERPOINTS];
   float X2Y2[MAXLASERPOINTS],X1X2[MAXLASERPOINTS];
   float X1Y2[MAXLASERPOINTS], Y1X2[MAXLASERPOINTS];
   float Y1Y2[MAXLASERPOINTS];
   float K[MAXLASERPOINTS], DS[MAXLASERPOINTS];
   float DsD[MAXLASERPOINTS], X2DsD[MAXLASERPOINTS], Y2DsD[MAXLASERPOINTS];
   float Bs[MAXLASERPOINTS], BsD[MAXLASERPOINTS];
   float A1, A2, A3, B1, B2, B3, C1, C2, C3, D1, D2, D3;
   MATRIX matA,invMatA;
   VECTOR vecB,vecSol;

   A1=0;A2=0;A3=0;B1=0;B2=0;B3=0;
   C1=0;C2=0;C3=0;D1=0;D2=0;D3=0;


   LMETRICA2=8;

   for (i=0; i<MAXLASERPOINTS; i++){
      X1[i]=Pack[i].a.x*Pack[i].a.x;
      Y1[i]=Pack[i].a.y*Pack[i].a.y;
      X2[i]=Pack[i].b.x*Pack[i].b.x;
      Y2[i]=Pack[i].b.y*Pack[i].b.y;
      X2Y2[i]=Pack[i].b.x*Pack[i].b.y;

      X1X2[i]=Pack[i].a.x*Pack[i].b.x;  //cp_ass[i].nx*cp_ass[i].rx;
      X1Y2[i]=Pack[i].a.x*Pack[i].b.y;  //cp_ass[i].nx*cp_ass[i].ry;
      Y1X2[i]=Pack[i].a.y*Pack[i].b.x;  //cp_ass[i].ny*cp_ass[i].rx;
      Y1Y2[i]=Pack[i].a.y*Pack[i].b.y;  //cp_ass[i].ny*cp_ass[i].ry;

      K[i]=X2[i]+Y2[i] + LMETRICA2;
      DS[i]=Y1Y2[i] + X1X2[i];
      DsD[i]=DS[i]/K[i];
      X2DsD[i]=Pack[i].b.x*DsD[i];  //cp_ass[i].rx*DsD[i];
      Y2DsD[i]=Pack[i].b.y*DsD[i];  //cp_ass[i].ry*DsD[i];

      Bs[i]=X1Y2[i]-Y1X2[i];
      BsD[i]=Bs[i]/K[i];

      A1=A1 + (1-Y2[i]/K[i]);
      B1=B1 + X2Y2[i]/K[i];
      C1=C1 + (-Pack[i].a.y + Y2DsD[i]);
      D1=D1 + (Pack[i].a.x - Pack[i].b.x - Pack[i].b.y*BsD[i]);

      A2=B1;
      B2=B2 + (1-X2[i]/K[i]);
      C2=C2 + (Pack[i].a.x-X2DsD[i]);
      D2=D2 + (Pack[i].a.y - Pack[i].b.y + Pack[i].b.x*BsD[i]);

      A3=C1;
      B3=C2;
      C3=C3 + (X1[i] + Y1[i] - DS[i]*DS[i]/K[i]);
      D3=D3 + (Bs[i]*(-1+DsD[i]));
   }


   initialize_matrix(&matA,3,3);
   MDATA(matA,0,0)=A1;     MDATA(matA,0,1)=B1;     MDATA(matA,0,2)=C1;
   MDATA(matA,1,0)=A2;     MDATA(matA,1,1)=B2;     MDATA(matA,1,2)=C2;
   MDATA(matA,2,0)=A3;     MDATA(matA,2,1)=B3;     MDATA(matA,2,2)=C3;

   if (inverse_matrix (&matA, &invMatA)==-1)
   {
      print_matrix("not invert matrix", &invMatA);
      exit(0);
   }

#ifdef INTMATSM_DEB
        print_matrix("inverted matrix", &invMatA);
#endif

        initialize_vector(&vecB,3);
        VDATA(vecB,0)=D1; VDATA(vecB,1)=D2; VDATA(vecB,2)=D3;
        multiply_matrix_vector (&invMatA, &vecB, &vecSol);

//        estimacion->x=-VDATA(vecSol,0);
//        estimacion->y=-VDATA(vecSol,1);
//        estimacion->tita=-VDATA(vecSol,2);

	Pose pse; // calculate pose
	pse.p.x = -VDATA(vecSol,0);
	pse.p.y = -VDATA(vecSol,1);
	pse.phi = -VDATA(vecSol,2);

	return pse;

//        return 1;
}

Transform2D::Transform2D(const Pose& ref) : base(ref) 
{
	c = cos(ref.phi);
	s = sin(ref.phi);
}

bool intersection_line_line (Point& p, const Line& l, const Line& m) 
// Compute the intersection point of two lines.
// Returns false for parallel lines.
{
	double gl, gm, bl, bm;
	bool lVert = true, mVert = true;

	// calculate gradients 
	if ((gl = l.second.x - l.first.x) != 0.0) {
		gl = (l.second.y - l.first.y)/gl;
		lVert = false;
	}
	if ((gm = m.second.x - m.first.x) != 0.0) {
		gm = (m.second.y - m.first.y)/gm;
		mVert = false;
	}

	if (lVert == mVert) { // check for parallelism 
		if (gl == gm)
			return false;
	}

	bl = l.first.y - gl*l.first.x; // calculate y intercepts 
	bm = m.first.y - gm*m.first.x;

	if (lVert) { // calculate intersection 
		p.x = l.first.x;
		p.y = gm*p.x + bm;
	}
	else if (mVert) {
		p.x = m.first.x;
		p.y = gl*p.x + bl;
	}
	else {
		p.x = (bm - bl)/(gl - gm);
		p.y = gm*p.x + bm;
	}

	return true;
}

double distance_line_point (const Line& lne, const Point& p)
// Note: distance is -ve if point is on left of line and +ve if it is on 
// the right (when looking from first to second). 
{
	Point v;
	v.x= lne.second.x - lne.first.x;
	v.y= lne.second.y - lne.first.y;

	return ((lne.second.y - p.y)*v.x 
		  - (lne.second.x - p.x)*v.y) 
		  / sqrt(v.x*v.x + v.y*v.y);
}

void intersection_line_point(Point& p, const Line& l, const Point& q) 
// Compute the perpendicular intersection from a point to a line
{
	Line m; 	// create line through q perpendicular to l
	m.first = q;
	m.second.x = q.x + (l.second.y - l.first.y);
	m.second.y = q.y - (l.second.x - l.first.x);

	p.x=0; p.y=0;
	bool not_parallel = intersection_line_line(p, l, m);
/*	if (not_parallel)
		printf("\nTRUE\n");
	else
		printf("\nFALSE\n");*/
	if (!not_parallel){
	    printf("p = %f	%f\n",p.x,p.y);
	    printf("q = %f	%f\n",q.x,q.y);	    
	    printf("l = %f	%f	%f	%f\n",l.first.x,l.first.y,l.second.x,l.second.y);	    
	}
	assert(not_parallel);
}

