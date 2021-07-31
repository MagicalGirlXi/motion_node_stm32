#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include "dist.h"

#define N 30
#define T 0.1


void Compute(Acceleration *A, Velocity *V, Position *P)
{
	int i;
	V->Velocity_X = 0;
	V->Velocity_Y = 0;
	V->Velocity_Z = 0;
	
	P->Position_X = 0;
	P->Position_Y = 0;
	P->Position_Z = 0;
	for(i=1; i<N; i++)
	{
		(V+i)->Velocity_X = (V+i-1)->Velocity_X + ((A+i-1)->Acceleration_X+(A+i)->Acceleration_X)*T/2;
		(V+i)->Velocity_Y = (V+i-1)->Velocity_Y + ((A+i-1)->Acceleration_Y+(A+i)->Acceleration_Y)*T/2;
		(V+i)->Velocity_Z = (V+i-1)->Velocity_Z + ((A+i-1)->Acceleration_Z+(A+i)->Acceleration_Z)*T/2;
		
		(P+i)->Position_X = (P+i-1)->Position_X + ((V+i-1)->Velocity_X+(V+i)->Velocity_X)*T/2;
		(P+i)->Position_Y = (P+i-1)->Position_Y + ((V+i-1)->Velocity_Y+(V+i)->Velocity_Y)*T/2;
		(P+i)->Position_Z = (P+i-1)->Position_Z + ((V+i-1)->Velocity_Z+(V+i)->Velocity_Z)*T/2;
	}
}

/*int main()
{
	int i;
	FILE *fp = fopen("res.csv", "a+");
	if(fp!=NULL)
	{
		fclose(fp);
		remove("res.csv");
		fp = fopen("res.csv", "a+");
	}
	init_rand_Accel(A);
	Compute(A,V,P);
	for(i=0;i<N;i++)
	{
		fprintf(fp, "%f,", i*T);
		fprintf(fp, "%f,%f,%f,", A[i].Acceleration_X,A[i].Acceleration_Y,A[i].Acceleration_Z);
		fprintf(fp, "%f,%f,%f,", V[i].Velocity_X, V[i].Velocity_Y, V[i].Velocity_Z);
		fprintf(fp, "%f,%f,%f\n", P[i].Position_X, P[i].Position_Y, P[i].Position_Z);
	}
	return 0;
}*/


