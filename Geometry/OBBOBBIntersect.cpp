#include "OBB.h"
#include "../math/mathlib.h"


  #pragma comment(linker, "/SUBSYSTEM:CONSOLE")




using namespace MathLib;

bool IntersectOBBOBB( COBB &obb1,  COBB &obb2)
{
	//true  collide
	// Cutoff for cosine of angles between box axes.  This is used to catch
	// the cases when at least one pair of axes are parallel.  If this
	// happens, there is no need to test for separation along the
	// Cross(A[i],B[j]) directions.
	//const double cutoff = 1.0 - std::numeric_limits<double>::epsilon();
#if 1
	bool existsParallelPair = false;
	int i,j;

	// Convenience variables.
	const std::vector<Vector3> &A = obb1.axis;
	const std::vector<Vector3> &B = obb2.axis;
	const Vector3 &EA = obb1.hsize;
	const Vector3 &EB = obb2.hsize;

	// Compute difference of box centers, D = C1-C0.
	
	//Vector3 centor1 = obb1.GetTransformedCenter();
	//Vector3 centor2 = obb1.GetTransformedCenter();
	Vector3 centor1 = obb1.cent;
	Vector3 centor2 = obb2.cent;
	const Vector3 D = centor2 - centor1;
	//Vector3 D = obb1.cent - obb2.cent;
	//double C[3][3];     // matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
	//double AbsC[3][3];  // |c_{ij}|
	//double AD[3];       // Dot(A_i,D)
	//double r0, r1, r;   // interval radii and distance between centers
	//double r01;         // = R0 + R1
	extern  FILE *file_lh;
	fprintf(file_lh, "A hsize %.2lf %.2lf %.2lf\n", EA[0], EA[1], EA[2]);
	fprintf(file_lh, "A axis_x %.2lf %.2lf %.2lf\n", A[0][0], A[0][1], A[0][2]);
	fprintf(file_lh, "A axis_y %.2lf %.2lf %.2lf\n", A[1][0], A[1][1], A[1][2]);
	fprintf(file_lh, "A axis_z %.2lf %.2lf %.2lf\n", A[2][0], A[2][1], A[2][2]);
	fprintf(file_lh, "A centor %.2lf %.2lf %.2lf\n", centor1[0], centor1[1], centor1[2]);
	for (size_t i = 0; i < 8; i++)
	{
		fprintf(file_lh, "A vertex %d %.2lf %.2lf %.2lf \n ",i, obb1.vp[i][0], obb1.vp[i][1], obb1.vp[i][2]);
	}


	fprintf(file_lh, "B hsize %.2lf %.2lf %.2lf\n", EB[0], EB[1], EB[2]);
	fprintf(file_lh, "B axis_x %.2lf %.2lf %.2lf\n", B[0][0], B[0][1], B[0][2]);
	fprintf(file_lh, "B axis_y %.2lf %.2lf %.2lf\n", B[1][0], B[1][1], B[1][2]);
	fprintf(file_lh, "B axis_z %.2lf %.2lf %.2lf\n", B[2][0], B[2][1], B[2][2]);
	fprintf(file_lh, "B centor %.2lf %.2lf %.2lf\n", centor2[0], centor2[1], centor2[2]);
	for (size_t i = 0; i < 8; i++)
	{
		fprintf(file_lh, "B vertex %d %.2lf %.2lf %.2lf \n ", i, obb2.vp[i][0], obb2.vp[i][1], obb2.vp[i][2]);
	}
	for (i = 0; i < 2; i++)
	{	
		
		double r1 = obb1.getProjectionRadius(B[i]);
		double r2 = obb2.getProjectionRadius(B[i]);
		double r3 = abs(D.dot(B[i]));
		//fprintf(file_lh, "A info cobb %.4lf\t %.4lf\t%.4lf\t \n", A[0]);
		fprintf(file_lh, "project to  b _x :%.4lf\t%.4lf\t%.4lf\n",r1,r2,r3);
		if ( (r1+r2)>r3 )
		{
			return true;
		}				
	}
	return false;

#else

	// Cutoff for cosine of angles between box axes.  This is used to catch
	// the cases when at least one pair of axes are parallel.  If this
	// happens, there is no need to test for separation along the
	// Cross(A[i],B[j]) directions.
	const double cutoff = 1.0 - std::numeric_limits<double>::epsilon();
	bool existsParallelPair = false;
	int i;

	// Convenience variables.
	const std::vector<Vector3> &A = obb1.axis;
	const std::vector<Vector3> &B = obb2.axis;
	const Vector3 &EA = obb1.hsize;
	const Vector3 &EB = obb2.hsize;

	// Compute difference of box centers, D = C1-C0.
	Vector3 D = obb1.cent - obb2.cent;

	double C[3][3];     // matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
	double AbsC[3][3];  // |c_{ij}|
	double AD[3];       // Dot(A_i,D)
	double r0, r1, r;   // interval radii and distance between centers
	double r01;         // = R0 + R1



	// axis C0+t*A0
	for (i = 0; i < 3; ++i)
	{
		C[0][i] = A[0].dot(B[i]);
		AbsC[0][i] = Abs(C[0][i]);
		if (AbsC[0][i] > cutoff)
		{
			existsParallelPair = true;
		}
	}
	AD[0] = A[0].dot(D);
	r = Abs(AD[0]);
	r1 = EB[0] * AbsC[0][0] + EB[1] * AbsC[0][1] + EB[2] * AbsC[0][2];
	r01 = EA[0] + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A1
	for (i = 0; i < 3; ++i)
	{
		C[1][i] = A[1].dot(B[i]);
		AbsC[1][i] = Abs(C[1][i]);
		if (AbsC[1][i] > cutoff)
		{
			existsParallelPair = true;
		}
	}
	AD[1] = A[1].dot(D);
	r = Abs(AD[1]);
	r1 = EB[0] * AbsC[1][0] + EB[1] * AbsC[1][1] + EB[2] * AbsC[1][2];
	r01 = EA[1] + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A2
	for (i = 0; i < 3; ++i)
	{
		C[2][i] = A[2].dot(B[i]);
		AbsC[2][i] = Abs(C[2][i]);
		if (AbsC[2][i] > cutoff)
		{
			existsParallelPair = true;
		}
	}
	AD[2] = A[2].dot(D);
	r = Abs(AD[2]);
	r1 = EB[0] * AbsC[2][0] + EB[1] * AbsC[2][1] + EB[2] * AbsC[2][2];
	r01 = EA[2] + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*B0
	r = Abs(B[0].dot(D));
	r0 = EA[0] * AbsC[0][0] + EA[1] * AbsC[1][0] + EA[2] * AbsC[2][0];
	r01 = r0 + EB[0];
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*B1
	r = Abs(B[1].dot(D));
	r0 = EA[0] * AbsC[0][1] + EA[1] * AbsC[1][1] + EA[2] * AbsC[2][1];
	r01 = r0 + EB[1];
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*B2
	r = Abs(B[2].dot(D));
	r0 = EA[0] * AbsC[0][2] + EA[1] * AbsC[1][2] + EA[2] * AbsC[2][2];
	r01 = r0 + EB[2];
	if (r > r01)
	{
		return false;
	}

	// At least one pair of box axes was parallel, so the separation is
	// effectively in 2D where checking the "edge" normals is sufficient for
	// the separation of the boxes.
	if (existsParallelPair)
	{
		return true;
	}

	// axis C0+t*A0xB0
	r = Abs(AD[2] * C[1][0] - AD[1] * C[2][0]);
	r0 = EA[1] * AbsC[2][0] + EA[2] * AbsC[1][0];
	r1 = EB[1] * AbsC[0][2] + EB[2] * AbsC[0][1];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A0xB1
	r = Abs(AD[2] * C[1][1] - AD[1] * C[2][1]);
	r0 = EA[1] * AbsC[2][1] + EA[2] * AbsC[1][1];
	r1 = EB[0] * AbsC[0][2] + EB[2] * AbsC[0][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A0xB2
	r = Abs(AD[2] * C[1][2] - AD[1] * C[2][2]);
	r0 = EA[1] * AbsC[2][2] + EA[2] * AbsC[1][2];
	r1 = EB[0] * AbsC[0][1] + EB[1] * AbsC[0][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A1xB0
	r = Abs(AD[0] * C[2][0] - AD[2] * C[0][0]);
	r0 = EA[0] * AbsC[2][0] + EA[2] * AbsC[0][0];
	r1 = EB[1] * AbsC[1][2] + EB[2] * AbsC[1][1];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A1xB1
	r = Abs(AD[0] * C[2][1] - AD[2] * C[0][1]);
	r0 = EA[0] * AbsC[2][1] + EA[2] * AbsC[0][1];
	r1 = EB[0] * AbsC[1][2] + EB[2] * AbsC[1][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A1xB2
	r = Abs(AD[0] * C[2][2] - AD[2] * C[0][2]);
	r0 = EA[0] * AbsC[2][2] + EA[2] * AbsC[0][2];
	r1 = EB[0] * AbsC[1][1] + EB[1] * AbsC[1][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A2xB0
	r = Abs(AD[1] * C[0][0] - AD[0] * C[1][0]);
	r0 = EA[0] * AbsC[1][0] + EA[1] * AbsC[0][0];
	r1 = EB[1] * AbsC[2][2] + EB[2] * AbsC[2][1];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A2xB1
	r = Abs(AD[1] * C[0][1] - AD[0] * C[1][1]);
	r0 = EA[0] * AbsC[1][1] + EA[1] * AbsC[0][1];
	r1 = EB[0] * AbsC[2][2] + EB[2] * AbsC[2][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	// axis C0+t*A2xB2
	r = Abs(AD[1] * C[0][2] - AD[0] * C[1][2]);
	r0 = EA[0] * AbsC[1][2] + EA[1] * AbsC[0][2];
	r1 = EB[0] * AbsC[2][1] + EB[1] * AbsC[2][0];
	r01 = r0 + r1;
	if (r > r01)
	{
		return false;
	}

	return true;
#endif

}