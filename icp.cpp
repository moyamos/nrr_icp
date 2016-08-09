#include "icp.h"
#include <cassert>
#include <algorithm>
#include <math.h>
#include <climits>
#include <stdio.h>

using namespace std;

ICP::ICP(vector<Point>& reference, const double& gate) : 
ref(reference), nn(new SweepSearch(reference, gate))
{
    before_E_dist=INT_MAX;
    before_pse.p.x=(double)LONG_MAX;
    before_pse.p.y=(double)LONG_MAX;    
    before_pse.phi=(double)LONG_MAX;    
    eps = 0.999;
}

void ICP::SetRef(vector<Point>& reference, const double& gate)
{
    ref = reference;
    nn->SetData(reference,gate);
}

double ICP::compute_epsilon(vector<nrr_TrICP_Pack>& dataset)
{
  double L = 2;
  int SC = dataset.size();  
  double EPS = eps;
  double EPSPOW = 0;
  double preEPS = EPS;
  double MAXPhiPhip = 0.1;
  double Phi = 1;
  double minEps = 1;
  double minPhi = double(LONG_MAX);
  for (int i=0; i<10 ; i++)
  {
      int index = SC * EPS;
      double tmse = 0;
      for (int j = 0; j<index; j++)
          tmse += dataset[j].di2;
      tmse /= index;
    
      EPSPOW = pow (EPS,-(L+1));
      Phi = tmse * EPSPOW;
      if (Phi < minPhi)
      {
          minPhi = Phi;
          minEps = EPS;
      }
      double tErr = dataset[index].di2;
      double PhiEpsP = (SC*(tErr-tmse))/((index-1)*EPS);
      double Phip = PhiEpsP * EPSPOW - (L+1) * EPSPOW * tmse / EPS;
      preEPS = EPS;
      if ( Phip == 0)
      {
          printf("Local Min.\n");
          break;
      }
      double PhiPhiP = Phi / Phip;
      double gamma = 1;
      if ( abs(PhiPhiP) > MAXPhiPhip )
          gamma = MAXPhiPhip / abs(PhiPhiP);
      EPS = EPS - gamma * PhiPhiP;
      if ( EPS < 0.8 )
          break;
      if ( EPS > 1.0 )
          break;
      if ( abs(preEPS - EPS) < 1/SC )
      {
          printf("Small Move\n");
          break;
      }
  }
  return minEps;
}

Pose ICP::align(const vector<Point>& obs, const Pose& init, double gate, int nits,double convergeErr, bool interp) 
{
	Pose pse = init; 
	double gate_sqr = sqr(gate); 
	int size_obs = obs.size();
	vector<int> index(2); // used if interp == true

	while (nits-- > 0) 
	{
		Transform2D tr(pse);
		
		TrICP_Pack.clear();
		
//		a.clear();
//		b.clear();

		// For each point in obs, find its NN in ref
		for (int i = 0; i < size_obs; ++i) { 
			Point p = obs[i];
			tr.transform_to_global(p); // transform obs[i] to estimated ref coord-frame

			Point q;
			if (interp == false) { // simple ICP
				int idx = nn->query(p);
				if (idx == SweepSearch::NOT_FOUND)
				{
//					printf("continue\n");
					continue;
				}

				q = ref[idx];
			} 
			else { // ICP with interpolation between 2 nearest points in ref
				(void) nn->query(p, index);
				assert(index.size() == 2);
				if (index[1] == SweepSearch::NOT_FOUND)
					continue;

				Line lne;
				lne.first  = ref[index[0]];
				lne.second = ref[index[1]];
				if (lne.first.x == 0 && lne.first.y == 0 && lne.second.x == 0 && lne.second.y == 0)
                { 
                    q.x=0; 
                    q.y=0; 
                }
				else
                    intersection_line_point(q, lne, p);
			}

			double dist_p_q = dist_sqr(p,q);
			if ( dist_p_q   < gate_sqr ) { // check if NN is close enough 
                nrr_TrICP_Pack temp;
                temp.a=obs[i];
				temp.b=q;
				temp.di2=dist_p_q;
//				a.push_back(obs[i]);
//				b.push_back(q);
//				di2.push_back(dist_p_q);
				TrICP_Pack.push_back(temp);
//				printf("push back= %f\n",dist_p_q);
			}
		}
//		assert(a.size() > 2); // TODO: replace this assert with a proper status flag
		assert(TrICP_Pack.size() > 2);
		
//        printf("Before SORT = %f  %f  %f \n",TrICP_Pack[0].di2,TrICP_Pack[1].di2,TrICP_Pack[2].di2);
		sort(TrICP_Pack.begin(),TrICP_Pack.end(),di2_order);
//        printf("After SORT = %f  %f  %f \n",TrICP_Pack[0].di2,TrICP_Pack[1].di2,TrICP_Pack[2].di2);

//		TrICP_Pack.erase(TrICP_Pack.begin()+TrICP_Pack.size()*0.92,TrICP_Pack.end());
		eps = compute_epsilon(TrICP_Pack);
		
//		pse = compute_relative_pose(a, b); // new iteration result
		pse = compute_relative_pose(TrICP_Pack,eps * 1.0); // new iteration result
		
		if (abs(before_pse.p.x-pse.p.x) < convergeErr && 
		    abs(before_pse.p.y-pse.p.y) < convergeErr && 
		    abs(before_pse.phi-pse.phi) < convergeErr)
		{
		    break;
		}
		else
		    before_pse=pse;
/*		Transform2D temp_tr(pse);
		double E_dist = 0;
		for (int i = 0; i < a.size(); ++i) 
		{ 
			Point p = a[i];
			temp_tr.transform_to_global(p); // transform a[i] "NewShut's point that correspondence to reference" to estimated ref coord-frame
			Point delta_p;
			delta_p.x=p.x-b[i].x;
			delta_p.y=p.y-b[i].y;
			E_dist += delta_p.x*delta_p.x + delta_p.y*delta_p.y;
		}
		//check changes of E_dist for convergence 
		if ( before_E_dist-E_dist < convergeErr ){  //converge thermo in "A Method for 3D shape registration"
		    break;
		}
		else
		    before_E_dist = E_dist;*/
//		printf("%f\t%d\n",E_dist,a.size());
	}
    if (nits < 0)
        printf("Max Iteration.\n");
    printf("Number of Remained Iteration = %d \t EPS = %f\n", nits, eps);
	return pse;
}

