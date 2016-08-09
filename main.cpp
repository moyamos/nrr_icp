/*
 *   main.cpp
 *
 *   Copyright (C) 2006 Mechatronics Research Lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   Authors:
 *           Yaghobi Mostafa (mostafa@pctlinux.com)
 *
 *   Server for Naji Rescue Robot controller 
 */

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <ncurses.h>
#include <GL/glut.h>
#include <vector>
#include <cassert>
#include "geometry2D.h"
#include "icp.h"
#include "nrr_time.h"
#include "fast_math.h"
#include <math.h>

//#define DEBUG

GLUquadricObj *diskQuadric;
float transZ = -25.0;
bool extra_line = false;
FILE *f;
nrr_time time1,time2;

using namespace std;

Pose demo_icp(vector<Point> &p1,vector<Point> &p2,Pose init,const double &gate,int nits,double convergeErr, bool interp=false)
{
    ICP icp(p1,gate);
//    const vector<Point>& aa = icp.get_points_ref();
//    const vector<Point>& ab = icp.get_points_obs();    
//    assert(aa.size() == ab.size());
    return icp.align(p2,init,gate,nits,convergeErr,interp);
}

Pose transform_to_global(Pose p, Pose b)
{
    Transform2D tr(b);
    tr.transform_to_global(p);
    return(p);
}

Point transform_to_global(Point p, Pose b)
{
    Transform2D tr(b);
    tr.transform_to_global(p);
    return(p);
}

vector<Point> get_laser_data()
{
     float x,y,t;
     int r,r_temp,tt;
     fscanf(f,"%f %f %f %d %d",&x,&y,&t,&tt,&tt);
     
     for (int i=0;i<44;i++)
     {
 	fscanf(f,"%d",&r_temp);
#ifdef DEBUG
 	printf("%d ",r_temp);
#endif
     }	
#ifdef DEBUG
     printf("\n");
#endif
    
     vector<Point> p1;
     Point temp;
     for ( int i=0;i<769-43-44;i++)
     {
       fscanf(f,"%d",&r);
       temp.x=r*cos(0.3515625*i*PI/180);
       temp.y=r*sin(0.3515625*i*PI/180);
#ifdef DEBUG       
       printf("%f	%f	%d\n",temp.x,temp.y,r);
#endif
       p1.push_back(temp);
     }     
     
     for (int i=0;i<43;i++)
     {
 	fscanf(f,"%d",&r_temp);
#ifdef DEBUG	
 	printf("%d ",r_temp);
#endif
     }	
#ifdef DEBUG     
     printf("\n");
#endif
     return p1;
}

vector<Point> get_laser_data_newVersion( float *x, float *y, float *t)
{
     float r,r_temp;
     fscanf(f,"%f %f %f",x,y,t);
#ifdef DEBUG            
     printf("------%f		%f	%f\n",x,y,t);
#endif

     for (int i=0;i<0;i++)
     {
 	fscanf(f,"%f",&r_temp);
#ifdef DEBUG
 	printf("%f ",r_temp);
#endif
     }	
#ifdef DEBUG
     printf("\n");
#endif
     
     vector<Point> p1;
     Point temp;
     for ( int i=0;i<(769-43-44);i++)
     {
       fscanf(f,"%f",&r);
       if (r<20)
        continue;
       r /=10;
       temp.x=r*cos((0.3515625*i-120)*PI/180); //-135+16.1719   // 16.1719=44*0.3515625   
       temp.y=r*sin((0.3515625*i-120)*PI/180);
#ifdef DEBUG       
       printf("%f	%f	%f\n",temp.x,temp.y,r);
#endif
       p1.push_back(temp);
     }     
     
     for (int i=0;i<0;i++)
     {
 	fscanf(f,"%f",&r_temp);
#ifdef DEBUG
 	printf("%f ",r_temp);
#endif
     }	
#ifdef DEBUG
     printf("\n");
#endif     
     
     return p1;
}

void defualtDraw( bool angle_line=true )
{
   glLoadIdentity();
   glTranslatef(0,-5,transZ);
   glColor3f(0.3,0.3,0.6);
   
   glBegin(GL_LINE);
       glVertex2f(-20,0);
       glVertex2f(20,0);
       glVertex2f(0,-20);
       glVertex2f(0,20);
   glEnd();        
   
   if ( angle_line == true )
   {
       float newx=0.0,newy=0.0;
       newx = 20 * cos(3.1415/6);
       newy = 20 * sin(3.1415/6);
       glBegin(GL_LINE);
           glVertex2f(-newx,-newy);
           glVertex2f(newx,newy);
           glVertex2f(-newx,newy);
           glVertex2f(newx,-newy);
       glEnd();
   
       newx = 20 * cos(3.1415/3);
       newy = 20 * sin(3.1415/3);
       glBegin(GL_LINE);
           glVertex2f(-newx,-newy);
           glVertex2f(newx,newy);
           glVertex2f(-newx,newy);
           glVertex2f(newx,-newy);
       glEnd();
   }
     
   glColor3f(0.3,0.3,0.5);
   glLineWidth(1);
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
   for (int i=1;i<10;i++)
       gluDisk(diskQuadric,i,i,30,3);
   
   glColor3f(1.3,0.3,0.8);
   
   gluDisk(diskQuadric,0.215,0.215,30,1);
   glColor3f(1.0,0.5,0.0);
   glLineWidth(3);
   glBegin(GL_LINE);
       glVertex2f(0,0);
       glVertex2f(0.215,0);
   glEnd();
   glLineWidth(1);
   
   f=fopen("./laser15.nlog","r");
   vector<Point> p1,p2;

   time1.update();
   float xx1,yy1,tt1,xx2,yy2,tt2;
   p1=get_laser_data_newVersion(&xx1,&yy1,&tt1);
    
   Pose delta,x;
   x.p.x = 0; 	   x.p.y = 0;  	  x.phi = 0;
   delta.p.x = 0; delta.p.y = 0; delta.phi = 0;    
    
   int max_iteration = 30;
   int gate1 = 30;
   int gate2 = 30;
   float err = 0.0005;
   int num_scan = 6000;
    
   for (int j=0 ; j < num_scan ; j++)
   {    
       p2=get_laser_data_newVersion(&xx2,&yy2,&tt2);
	
       //	delta.p.x=xx2-xx1; delta.p.y=yy2-yy1; delta.phi=(tt2-tt1)*PI/180;
       //	printf("%f+++++%f++++%f\n",delta.p.x,delta.p.y, delta.phi);
	
       //        delta.p.x = 0; delta.p.y = 0; delta.phi = 0;    	
	
       if (j % 2 == 0)
       {
           delta = demo_icp(p1,p2,delta,gate1,max_iteration,err,true);
           //		printf("%f===%f===%f\n",delta.p.x,delta.p.y, delta.phi);

           delta = demo_icp(p1,p2,delta,gate2,max_iteration,err,true);	

           printf("%d	%f   %f    %f\n",j,x.p.x,x.p.y,x.phi*180/PI);	

           x = transform_to_global(delta,x);

           p1=p2;
           xx1=xx2; yy1=yy2; tt1=tt2;

           for (int i=0;i</*769-44-43*/p2.size();i++)
           {
               p2[i] =  transform_to_global(p2[i],x);
               glColor3f(1.0,1.0,1.0);
               glBegin(GL_POINTS);
               glVertex2f(p2[i].x/100,p2[i].y/100);
               glEnd();
           }
           glColor3f(0.0,1.0,0.0);
           glBegin(GL_POINTS);
           glVertex2f(x.p.x/100,x.p.y/100);
           glEnd();	
       }
       glutSwapBuffers();
   }
    
   time2.update();
   printf("Total Time: %ds.  Mean:%f\n",time2.secdifference(time1),(float)time2.secdifference(time1)/num_scan);
   printf("MAX_ITE. = %d\tGate1 = %d\t gate2 = %d\terr = %f\n",max_iteration,gate1,gate2,err);

   fclose(f);
}

void OnKey(unsigned char key, int x , int y)
{
   switch ( key )
   {
      case 'c':
      {
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);   
         defualtDraw( extra_line );
      }break;
      case 'd':
      {
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);   
      }break;      
      case 'a':
      {
         transZ -= 1.0;
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);   
         defualtDraw( extra_line );
      }break;
      case 'z':
      {
         transZ += 1.0;
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);   
         defualtDraw( extra_line );
      }break;
      case 'l' :
      {
         extra_line = not extra_line;
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);   
         defualtDraw( extra_line );
      }break;
   }
}

void OnReshape(int w, int h)
{
   if (h==0) {
      h=1;
   }

   // set the drawable region of the window
   glViewport(0,0,w,h);

   // set up the projection matrix 
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   // just use a perspective projection
   gluPerspective(45.0,w/h,1.0,500);

   // go back to modelview matrix so we can move the objects about
   glMatrixMode(GL_MODELVIEW);
   
   glClearColor(0.0,0.0,0.0,0.0);
   
   // clear the screen & depth buffer
   glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
   
   defualtDraw( extra_line );
}

void OnDraw() {
   // currently we've been drawing to the back buffer, we need
   // to swap the back buffer with the front one to make the image visible
   
   // clear the screen & depth buffer
//    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
//	printf("Draw\ne");   
   glutSwapBuffers();
}

void OnInit() {
    diskQuadric = gluNewQuadric();
}

void OnExit() {

}

void OnIdle() {
   // redraw the screen
    glutPostRedisplay();
}

int main( int argc, char* argv[] )
{
    // initialise glut
    glutInit(&argc,argv);

    // request a depth buffer, RGBA display mode, and we want double buffering
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE);

    // set the initial window size
    glutInitWindowSize(600,600);

    // create the window
    glutCreateWindow("NRR TrICP Mapping");

    // set the function to use to draw our scene
    glutDisplayFunc(OnDraw);

    // set the function to use to Keyboard
    glutKeyboardFunc(OnKey);

    // set the function to handle changes in screen size
    glutReshapeFunc(OnReshape);

    // set the idle callback
    glutIdleFunc(OnIdle);

    // run our custom initialisation
    OnInit();

    // set the function to be called when we exit
    atexit(OnExit);

    // this function runs a while loop to keep the program running.
    glutMainLoop();

    return 0;
}

