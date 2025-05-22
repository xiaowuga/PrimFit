#include "fitting.h"


#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace easy3d;

namespace QuadFit {
namespace levmar
{
	template< class ScalarT >
	struct LevMarFunc {
		virtual ScalarT operator()(const ScalarT* param) const = 0;
		virtual void operator()(const ScalarT* param, ScalarT* gradient) const = 0;
	};

	class LevMarLSWeight
	{
	public:
		float Weigh(float d) const { return d; }
		template< unsigned int N >
		void DerivWeigh(float d, float* gradient) const {}
	};

	template< class ScalarT >
	bool Cholesky(ScalarT* a, size_t n, ScalarT p[])
		/*Given a positive-definite symmetric matrix a[1..n][1..n], this routine constructs its Cholesky
		decomposition, A = L ?LT . On input, only the upper triangle of a need be given; it is not
		modified. The Cholesky factor L is returned in the lower triangle of a, except for its diagonal
		elements which are returned in p[1..n].*/
	{
		size_t i, j, k;
		ScalarT sum;
		for (i = 0; i < n; ++i)
		{
			for (j = i; j < n; ++j)
			{
				for (sum = a[i * n + j], k = i - 1; k != -1; --k)
					sum -= a[i * n + k] * a[j * n + k];
				if (i == j)
				{
					if (sum <= ScalarT(0))
						// a, with rounding errors, is not positive definite.
						return false;
					p[i] = std::sqrt(sum);
				}
				else
					a[j * n + i] = sum / p[i];
			}
		}
		return true;
	}

	template< class ScalarT, unsigned int N >
	bool Cholesky(ScalarT* a, ScalarT p[])
		/*Given a positive-definite symmetric matrix a[1..n][1..n], this routine constructs its Cholesky
		decomposition, A = L ?LT . On input, only the upper triangle of a need be given; it is not
		modified. The Cholesky factor L is returned in the lower triangle of a, except for its diagonal
		elements which are returned in p[1..n].*/
	{
		size_t i, j, k;
		ScalarT sum;
		for (i = 0; i < N; ++i)
		{
			for (j = i; j < N; ++j)
			{
				for (sum = a[i * N + j], k = i - 1; k != -1; --k)
					sum -= a[i * N + k] * a[j * N + k];
				if (i == j)
				{
					if (sum <= ScalarT(0))
						// a, with rounding errors, is not positive definite.
						return false;
					p[i] = std::sqrt(sum);
				}
				else
					a[j * N + i] = sum / p[i];
			}
		}
		return true;
	}

	template< class ScalarT >
	void CholeskySolve(ScalarT* a, size_t n, ScalarT p[], ScalarT b[], ScalarT x[])
		/*Solves the set of n linear equations A ?x = b, where a is a positive-definite symmetric matrix.
		a[1..n][1..n] and p[1..n] are input as the output of the routine choldc. Only the lower
		subdiagonal portion of a is accessed. b[1..n] is input as the right-hand side vector. The
		solution vector is returned in x[1..n]. a, n, and p are not modified and can be left in place
		for successive calls with different right-hand sides b. b is not modified unless you identify b and
		x in the calling sequence, which is allowed.*/
	{
		size_t i, k;
		ScalarT sum;
		for (i = 0; i < n; i++)
		{ // Solve L ?y = b, storing y in x.
			for (sum = b[i], k = i - 1; k != -1; --k)
				sum -= a[i * n + k] * x[k];
			x[i] = sum / p[i];
		}
		for (i = n - 1; i != -1; --i)
		{ // Solve LT ?x = y.
			for (sum = x[i], k = i + 1; k < n; ++k)
				sum -= a[k * n + i] * x[k];
			x[i] = sum / p[i];
		}
	}

	template< class ScalarT, unsigned int N >
	void CholeskySolve(ScalarT* a, ScalarT p[], ScalarT b[], ScalarT x[])
		/*Solves the set of n linear equations A ?x = b, where a is a positive-definite symmetric matrix.
		a[1..n][1..n] and p[1..n] are input as the output of the routine choldc. Only the lower
		subdiagonal portion of a is accessed. b[1..n] is input as the right-hand side vector. The
		solution vector is returned in x[1..n]. a, n, and p are not modified and can be left in place
		for successive calls with different right-hand sides b. b is not modified unless you identify b and
		x in the calling sequence, which is allowed.*/
	{
		size_t i, k;
		ScalarT sum;
		for (i = 0; i < N; i++)
		{ // Solve L ?y = b, storing y in x.
			for (sum = b[i], k = i - 1; k != -1; --k)
				sum -= a[i * N + k] * x[k];
			x[i] = sum / p[i];
		}
		for (i = N - 1; i != -1; --i)
		{ // Solve LT ?x = y.
			for (sum = x[i], k = i + 1; k < N; ++k)
				sum -= a[k * N + i] * x[k];
			x[i] = sum / p[i];
		}
	}

	template< class IteratorT, class FuncT >
	bool LevMar(IteratorT begin, IteratorT end, FuncT& func,
		typename FuncT::ScalarType* param)
	{
		typedef typename FuncT::ScalarType ScalarType;
		enum { paramDim = FuncT::NumParams };
		bool retVal = true;
		unsigned int totalSize = end - begin;
		if (!totalSize)
			return false;
		ScalarType lambda = ScalarType(0.0001);
		ScalarType* F0 = new ScalarType[totalSize * paramDim];
		ScalarType* U = new ScalarType[paramDim * paramDim];
		ScalarType* H = new ScalarType[paramDim * paramDim];
		ScalarType* v = new ScalarType[paramDim];
		ScalarType* d = new ScalarType[totalSize];
		ScalarType* temp = new ScalarType[totalSize];
		ScalarType* x = new ScalarType[paramDim];
		ScalarType* p = new ScalarType[paramDim];
		ScalarType* paramNew = new ScalarType[paramDim];
		size_t nu = 2;
		func.Normalize(param);
		ScalarType paramNorm = 0;

		// do fitting in different steps
		unsigned int subsets = std::max(int(std::floor(std::log((float)totalSize) / std::log(2.f))) - 8, 1);

		MiscLib::Vector< unsigned int > subsetSizes(subsets);
		for (unsigned int i = subsetSizes.size(); i;)
		{
			--i;
			subsetSizes[i] = totalSize;
			if (i)
				subsetSizes[i] = subsetSizes[i] >> 1;
			totalSize -= subsetSizes[i];
		}
		unsigned int curSubset = 0;
		unsigned int size = 0;

		// get current error
		ScalarType chi = 0, newChi = 0;
		ScalarType rho = 1;
		unsigned int outerIter = 0,
			maxOuterIter = 200 / subsetSizes.size(),
			usefulIter = 0, totalIter = 0;;
		do
		{
			// get current error
			size += subsetSizes[curSubset];
			newChi = func.Chi(param, begin, begin + size, d, temp);
			for (unsigned int i = 0; i < paramDim; ++i)
				paramNew[i] = param[i];
			outerIter = 0;
			if (rho < 0)
				rho = 1;
			do
			{
				++outerIter;
				++totalIter;
				if (rho > 0)
				{
					nu = 2;
					chi = newChi;
					for (size_t i = 0; i < paramDim; ++i)
						param[i] = paramNew[i];

					if (std::sqrt(chi / size) < ScalarType(1e-5)) // chi very small? -> will be hard to improve
					{
						//std::cout << "LevMar converged because of small chi" << std::endl;
						break;
					}
					paramNorm = 0;
					for (size_t i = 0; i < paramDim; ++i)
						paramNorm += param[i] * param[i];
					paramNorm = std::sqrt(paramNorm);
					// construct the needed matrices
					// F0 is the matrix constructed from param
					// F0 has gradient_i(param) as its ith row
					func.Derivatives(param, begin, begin + size, d, temp, F0);
					// U = F0_t * F0
					// v = F0_t * d(param) (d(param) = [d_i(param)])

					for (int i = 0; i < paramDim; ++i)
					{
						for (size_t j = i; j < paramDim; ++j) // j = i since only upper triangle is needed
						{
							U[i * paramDim + j] = 0;
							for (size_t k = 0; k < size; ++k)
							{
								U[i * paramDim + j] += F0[k * paramDim + i] *
									F0[k * paramDim + j];
							}
						}
					}
					ScalarType vmag = 0; // magnitude of v

					for (int i = 0; i < paramDim; ++i)
					{
						v[i] = 0;
						for (size_t k = 0; k < size; ++k)
							v[i] += F0[k * paramDim + i] * d[k];
						v[i] *= -1;

						vmag = std::max((ScalarType)fabs(v[i]), vmag);

					}

					// and check for convergence with magnitude of v

					if (vmag < ScalarType(1.0e-6))
					{
						//std::cout << "LevMar converged with small gradient" << std::endl;
						//retVal = chi < initialChi;
						//goto cleanup;
						break;
					}
					if (outerIter == 1)
					{
						// compute magnitue of F0
						ScalarType fmag = fabs(F0[0]);
						for (size_t i = 1; i < paramDim * size; ++i)
							if (fmag < fabs(F0[i]))
								fmag = fabs(F0[i]);
						lambda = 1e-3f * fmag;
					}
					else
						lambda *= std::max(ScalarType(0.3), 1 - ScalarType(std::pow(2 * rho - 1, 3)));
				}

				memcpy(H, U, sizeof(ScalarType) * paramDim * paramDim);
				for (size_t i = 0; i < paramDim; ++i)
					H[i * paramDim + i] += lambda; // * (ScalarType(1) + H[i * paramDim + i]);
				// now H is positive definite and symmetric
				// solve Hx = -v with Cholesky
				ScalarType xNorm = 0, L = 0;
				if (!Cholesky< ScalarType, paramDim >(H, p))
					goto increment;
				CholeskySolve< ScalarType, paramDim >(H, p, v, x);

				// magnitude of x small? If yes we are done
				for (size_t i = 0; i < paramDim; ++i)
					xNorm += x[i] * x[i];
				xNorm = std::sqrt(xNorm);

				if (xNorm <= ScalarType(1.0e-6) * (paramNorm + ScalarType(1.0e-6)))
				{
					//std::cout << "LevMar converged with small step" << std::endl;
					//goto cleanup;
					break;
				}

				for (size_t i = 0; i < paramDim; ++i)
					paramNew[i] = param[i] + x[i];
				func.Normalize(paramNew);

				// get new error
				newChi = func.Chi(paramNew, begin, begin + size, d, temp);

				// the following test is taken from
				// "Methods for non-linear least squares problems"
				// by Madsen, Nielsen, Tingleff
				L = 0;
				for (size_t i = 0; i < paramDim; ++i)
					L += .5f * x[i] * (lambda * x[i] + v[i]);
				rho = (chi - newChi) / L;
				if (rho > 0)
				{
					++usefulIter;

					if ((chi - newChi) < 1e-4 * chi)
					{
						//std::cout << "LevMar converged with small chi difference" << std::endl;
						chi = newChi;
						for (size_t i = 0; i < paramDim; ++i)
							param[i] = paramNew[i];
						break;
					}
					continue;
				}

			increment:
				rho = -1;
				// increment lambda
				lambda = nu * lambda;
				size_t nu2 = nu << 1;
				if (nu2 < nu)
					nu2 = 2;
				nu = nu2;
			} while (outerIter < maxOuterIter);
			++curSubset;
		} while (curSubset < subsetSizes.size());
		retVal = usefulIter > 0;
		delete[] F0;
		delete[] U;
		delete[] H;
		delete[] v;
		delete[] d;
		delete[] temp;
		delete[] x;
		delete[] p;
		delete[] paramNew;
		return retVal;
	}


	int dmat_solve(int n, int rhs_num, double a[])
		//******************************************************************************
		//
		//  Purpose:
		//
		//    DMAT_SOLVE uses Gauss-Jordan elimination to solve an N by N linear system.
		//
		//  Discussion:
		//
		//    The doubly dimensioned array A is treated as a one dimensional vector,
		//    stored by COLUMNS.  Entry A(I,J) is stored as A[I+J*N]
		//
		//  Modified:
		//
		//    29 August 2003
		//
		//  Author:
		//
		//    John Burkardt
		//
		//  Parameters:
		//
		//    Input, int N, the order of the matrix.
		//
		//    Input, int RHS_NUM, the number of right hand sides.  RHS_NUM
		//    must be at least 0.
		//
		//    Input/output, double A[N*(N+RHS_NUM)], contains in rows and columns 1
		//    to N the coefficient matrix, and in columns N+1 through
		//    N+RHS_NUM, the right hand sides.  On output, the coefficient matrix
		//    area has been destroyed, while the right hand sides have
		//    been overwritten with the corresponding solutions.
		//
		//    Output, int DMAT_SOLVE, singularity flag.
		//    0, the matrix was not singular, the solutions were computed;
		//    J, factorization failed on step J, and the solutions could not
		//    be computed.
		//
	{
		double apivot;
		double factor;
		int i;
		int ipivot;
		int j;
		int k;
		double temp;

		for (j = 0; j < n; j++)
		{
			//
			//  Choose a pivot row.
			//
			ipivot = j;
			apivot = a[j + j * n];

			for (i = j; i < n; i++)
			{
				if (fabs(apivot) < fabs(a[i + j * n]))
				{
					apivot = a[i + j * n];
					ipivot = i;
				}
			}

			if (apivot == 0.0)
			{
				return j;
			}
			//
			//  Interchange.
			//
			for (i = 0; i < n + rhs_num; i++)
			{
				temp = a[ipivot + i * n];
				a[ipivot + i * n] = a[j + i * n];
				a[j + i * n] = temp;
			}
			//
			//  A(J,J) becomes 1.
			//
			a[j + j * n] = 1.0;
			for (k = j; k < n + rhs_num; k++)
			{
				a[j + k * n] = a[j + k * n] / apivot;
			}
			//
			//  A(I,J) becomes 0.
			//
			for (i = 0; i < n; i++)
			{
				if (i != j)
				{
					factor = a[i + j * n];
					a[i + j * n] = 0.0;
					for (k = j; k < n + rhs_num; k++)
					{
						a[i + k * n] = a[i + k * n] - factor * a[j + k * n];
					}
				}

			}

		}

		return 0;
	}
}

class Plane : public SurfacePrimitive
{
public:

	Plane() :_d(0),SurfacePrimitive(SurfaceType::UNKNOWN) {};

	Plane(const vec3& point, const vec3& normal):SurfacePrimitive(SurfaceType::PLANE), _normal(normalize(normal))
	{
		_d = -dot(_normal,point);
		m_pos = point;
	}

	Plane(float distance_to_origin, const vec3& normal) :_d(distance_to_origin), SurfacePrimitive(SurfaceType::PLANE), _normal(normalize(normal)) { m_pos = -_d * normal; }

	virtual bool fit(const vector<Point>& pts)override
	{
		setType(SurfaceType::UNKNOWN);
		vector<vec3> points, normals;
		for (int i = 0; i < pts.size(); i++)
		{
			points.push_back(pts[i].p);
			normals.push_back(pts[i].n);
		}

		if (pts.size() < 1)
			return false;
		size_t c = points.size() ;
		GfxTL::Vector3Df meanNormal;
		if (!GfxTL::MeanOfNormals(normals.begin(), normals.end(), &meanNormal))
			return false;
		_normal=vec3(meanNormal.Data());
		GfxTL::Vector3Df mean;
		GfxTL::Mean(points.begin(), points.end(), &mean);
		m_pos=vec3(mean.Data());
		_d = -dot(m_pos,_normal);
		setType(SurfaceType::PLANE);
		return true;
	}

	virtual float signedDistance(const vec3& p) const override
	{
		return dot(_normal, p) + _d;
	}

	virtual vec3 project(const vec3& p) const override
	{
		return p - signedDistance(p) * _normal;
	}

	virtual vec3 normal(const vec3& p) const override
	{
		return _normal;
	}


	virtual SurfaceParameters getParameters() const
	{
		SurfaceParameters p;
		p.pos = m_pos;
		p.dir = _normal;
		p.r1 = _d;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters)
	{
		_normal = new_parameters.dir.normalize();
		_d = new_parameters.r1;
		m_pos = -_d * _normal;
	}

private:
	vec3 _normal;
	vec3 m_pos;
	float _d;
};

class Cylinder : public SurfacePrimitive
{
public:

	Cylinder() :SurfacePrimitive(SurfaceType::UNKNOWN), m_radius(0) {};

	Cylinder(const vec3& axisDir, const vec3& axisPos, float radius) :SurfacePrimitive(SurfaceType::CYLINDER)
	{
		m_axisDir = normalize(axisDir);
		m_axisPos = axisPos;
		m_radius = radius;
	}


	virtual bool fit(const vector<Point>& pts)override
	{

		setType(SurfaceType::UNKNOWN);
	//************************************ init average
		MiscLib::Vector< GfxTL::Vector3Df > normals;
		size_t c = pts.size();
		for (size_t i = 0; i < c; ++i)
		{
			normals.push_back(GfxTL::Vector3Df (pts[i].n));
			normals.push_back(GfxTL::Vector3Df (-pts[i].n));
		}
		GfxTL::MatrixXX< 3, 3, float > cov, eigenVectors;
		GfxTL::Vector3Df eigenValues;
		GfxTL::CovarianceMatrix(GfxTL::Vector3Df(0, 0, 0),
			normals.begin(), normals.end(), &cov);
		if (!GfxTL::Jacobi(cov, &eigenValues, &eigenVectors))
			return false;

		// find the minimal eigenvalue and corresponding vector
		float minEigVal = eigenValues[0];
		unsigned int minEigIdx = 0;
		for (unsigned int i = 1; i < 3; ++i)
			if (eigenValues[i] < minEigVal)
			{
				minEigVal = eigenValues[i];
				minEigIdx = i;
			}
		m_axisDir = vec3(eigenVectors[minEigIdx].Data());
		// get a point on the axis from all pairs
		m_axisPos = vec3(0, 0, 0);
		m_radius = 0;
		size_t pointCount = 0;
		for (size_t i = 0; i < c - 1; ++i)
			for (size_t j = i + 1; j < c; ++j)
			{
				// project first normal into plane
				float l = dot(m_axisDir, pts[i].n);
				vec3 xdir = pts[i].n - l * m_axisDir;
				xdir.normalize();
				vec3 ydir = cross(m_axisDir,xdir);
				ydir.normalize();
				// xdir is the x axis in the plane (y = 0) samples[i] is the origin
				float lineBnx = dot(ydir,pts[j].n);
				if (fabs(lineBnx) < .05f)
					continue;
				float lineBny = -dot(xdir,pts[j].n);
				// origin of lineB
				vec3 originB = pts[j].p - pts[i].p;
				float lineBOx = dot(xdir,originB);
				float lineBOy = dot(ydir,originB);
				float lineBd = lineBnx * lineBOx + lineBny * lineBOy;
				// lineB in the plane complete
				// point of intersection is y = 0 and x = lineBd / lineBnx
				float radius = lineBd / lineBnx;
				m_axisPos += pts[i].p + radius * xdir;
				m_radius += fabs(radius);
				m_radius += std::sqrt((radius - lineBOx) * (radius - lineBOx) + lineBOy * lineBOy);
				++pointCount;
			}
		if (!pointCount)
			return false;
		m_axisPos /= pointCount;
		m_radius /= pointCount * 2;
		if (m_radius > 1e6)
			return false;

		// find point on axis closest to origin
		float lambda = dot(m_axisDir,-m_axisPos);
		m_axisPos = m_axisPos + lambda * m_axisDir;


	//******************************* fitting levmar
		float param[7];
		for (size_t i = 0; i < 3; ++i)
			param[i] = m_axisPos[i];
		for (size_t i = 0; i < 3; ++i)
			param[i + 3] = m_axisDir[i];
		param[6] = m_radius;
		LevMarCylinder< levmar::LevMarLSWeight > levMarCylinder;
		if (levmar::LevMar(pts.begin(), pts.end(), levMarCylinder, param))
		{
			for (size_t i = 0; i < 3; ++i)
				m_axisPos[i] = param[i];
			for (size_t i = 0; i < 3; ++i)
				m_axisDir[i] = param[i + 3];
			m_radius = param[6];
		}
		setType(SurfaceType::CYLINDER);

		// compute half_height;

		float minh=FLT_MAX, maxh=-FLT_MAX;
		for (const auto &p : pts)
		{
			float l= dot((p.p - m_axisPos), m_axisDir);
			minh = min(minh, l);
			maxh = max(maxh, l);
		}
		half_height = (maxh - minh) / 2.;
		m_axisPos = m_axisPos + m_axisDir * ((maxh+minh)/2.);
		return true;
	}

	virtual float signedDistance(const vec3& p) const override
	{
		vec3 diff = p - m_axisPos;
		float lambda = dot(m_axisDir,diff);
		float axisDist = (diff - lambda * m_axisDir).length();
		return axisDist - m_radius;
	}

	virtual vec3 project(const vec3& p) const override
	{
		vec3 diff = m_axisPos - p;
		float lambda = dot(m_axisDir,diff);
		vec3 pp = (diff - lambda * m_axisDir);
		float l = pp.length();
		pp *= (l - m_radius) / l;
		pp += p;
		return pp;
	}

	virtual vec3 normal(const vec3& p) const override
	{
		vec3 diff = m_axisPos - p;
		float lambda = dot(m_axisDir, diff);
		vec3 pp = (diff - lambda * m_axisDir);
		return -normalize(pp);
	}


	virtual SurfaceParameters getParameters() const
	{
		SurfaceParameters p;
		p.dir = m_axisDir;
		p.pos = m_axisPos;
		p.r1 = m_radius;
		p.r2 = half_height;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters)
	{
		m_axisDir= new_parameters.dir;
		m_axisPos= new_parameters.pos;
		m_radius= new_parameters.r1;
		half_height = new_parameters.r2;
	}

private:
	vec3 m_axisDir;
	vec3 m_axisPos;
	float m_radius;
	float half_height;
private:

	template< class WeightT >
	class LevMarCylinder
		: public WeightT
	{
	public:
		enum { NumParams = 7 };
		typedef float ScalarType;

		template< class IteratorT >
		ScalarType Chi(const ScalarType* params, IteratorT begin, IteratorT end,
			ScalarType* values, ScalarType* temp) const
		{
			ScalarType chi = 0;
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				vec3 s;
				for (unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType u = params[5] * s[1] - params[4] * s[2];
				u *= u;
				ScalarType v = params[3] * s[2] - params[5] * s[0];
				u += v * v;
				v = params[4] * s[0] - params[3] * s[1];
				u += v * v;
				temp[idx] = std::sqrt(u);
				chi += (values[idx] = WeightT::Weigh(temp[idx] - params[6]))
					* values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType* params, IteratorT begin, IteratorT end,
			const ScalarType* values, const ScalarType* temp, ScalarType* matrix) const
		{
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				vec3 s;
				for (unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = s[0] * begin[idx][0] + s[1] * begin[idx][1]
					+ s[2] * begin[idx][2];
				if (temp[idx] < 1.0e-6)
				{
					matrix[idx * NumParams + 0] = std::sqrt(1 - params[3] * params[3]);
					matrix[idx * NumParams + 1] = std::sqrt(1 - params[4] * params[4]);
					matrix[idx * NumParams + 2] = std::sqrt(1 - params[5] * params[5]);
				}
				else
				{
					matrix[idx * NumParams + 0] = (params[3] * g - s[0]) / temp[idx];
					matrix[idx * NumParams + 1] = (params[4] * g - s[1]) / temp[idx];
					matrix[idx * NumParams + 2] = (params[5] * g - s[2]) / temp[idx];
				}
				matrix[idx * NumParams + 3] = g * matrix[idx * NumParams + 0];
				matrix[idx * NumParams + 4] = g * matrix[idx * NumParams + 1];
				matrix[idx * NumParams + 5] = g * matrix[idx * NumParams + 2];
				matrix[idx * NumParams + 6] = -1;
				WeightT::template DerivWeigh< NumParams >(temp[idx] - params[6],
					matrix + idx * NumParams);
			}
		}

		void Normalize(ScalarType* params) const
		{
			ScalarType l = std::sqrt(params[3] * params[3] + params[4] * params[4]
				+ params[5] * params[5]);
			for (unsigned int i = 3; i < 6; ++i)
				params[i] /= l;
			// find point on axis closest to origin
			float lambda = -(params[0] * params[3] + params[1] * params[4] +
				params[2] * params[5]);
			for (unsigned int i = 0; i < 3; ++i)
				params[i] = params[i] + lambda * params[i + 3];
		}
	};

};

class Sphere : public SurfacePrimitive
{
public:

	Sphere() :SurfacePrimitive(SurfaceType::UNKNOWN), m_radius(0) {};

	Sphere(const vec3& center, float radius) : SurfacePrimitive(SurfaceType::SPHERE), m_center(center),m_radius(radius){}


	virtual bool fit(const vector<Point>& pts)override
	{

		setType(SurfaceType::UNKNOWN);
	//********************************** init
		if (pts.size() < 2)
			return false;
		// get center
		size_t c = pts.size();
		m_center = vec3(0, 0, 0);
		size_t midCount = 0;
		for (size_t i = 0; i < c - 1; ++i)
			for (size_t j = i + 1; j < c; ++j)
			{
				vec3 mid;
				if (!Midpoint(pts[i].p, pts[i].n, pts[j].p, pts[j].n, &mid))
					continue;
				m_center += mid;
				++midCount;
			}
		if (!midCount)
			return false;
		m_center /= midCount;
		m_radius = 0;
		for (size_t i = 0; i < c; ++i)
		{
			float d = (pts[i].p - m_center).length();
			m_radius += d;
		}
		m_radius /= c;
	//********************************** levmar fitting
		LevMarSimpleSphere< levmar::LevMarLSWeight > levMarSphere;
		float param[4];
		for (size_t i = 0; i < 3; ++i)
			param[i] = m_center[i];
		param[3] = m_radius;
		if (!levmar::LevMar(pts.begin(), pts.end(), levMarSphere, param))
			return false;
		for (size_t i = 0; i < 3; ++i)
			m_center[i] = param[i];
		m_radius = param[3];
		setType(SurfaceType::SPHERE);
		return true;
	}

	virtual float signedDistance(const vec3& p) const override
	{
		return (m_center - p).length() - m_radius;
	}

	virtual vec3 project(const vec3& p) const override
	{
		vec3 pp = p - m_center;
		float l = pp.length();
		pp *= m_radius / l;
		pp += m_center;
		return pp;
	}

	virtual vec3 normal(const vec3& p) const override
	{
		return normalize(p - m_center);
	}


	virtual SurfaceParameters getParameters() const
	{
		SurfaceParameters p;
		p.pos = m_center;
		p.r1 = m_radius;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters)
	{
		m_center = new_parameters.pos;
		m_radius = new_parameters.r1;
	}
private:
	vec3 m_center;
	float m_radius;
private:
	bool Midpoint(const vec3& p1, const vec3& n1, const vec3& p2, const vec3& n2,
		vec3* mid)
	{
		float d1343, d4321, d1321, d4343, d2121;
		float numer, denom, mua, mub;

		vec3 p13 = p1 - p2;
		// p43 = n2
		// p21 = n1
		d1343 = p13[0] * n2[0] + p13[1] * n2[1] + p13[2] * n2[2];
		d4321 = n2[0] * n1[0] + n2[1] * n1[1] + n2[2] * n1[2];
		d1321 = p13[0] * n1[0] + p13[1] * n1[1] + p13[2] * n1[2];
		d4343 = n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2];
		d2121 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

		denom = d2121 * d4343 - d4321 * d4321;
		if (fabs(denom) < 1.0e-6)
			return false;
		numer = d1343 * d4321 - d1321 * d4343;

		mua = numer / denom;
		mub = (d1343 + d4321 * (mua)) / d4343;

		vec3 pa, pb;
		pa = p1 + mua * n1;
		pb = p2 + mub * n2;
		*mid = 0.5f * (pa + pb);
		return true;
	}
	template< class WeightT >
	class LevMarSimpleSphere
		: public WeightT
	{
	public:
		enum { NumParams = 4 };
		typedef float ScalarType;

		template< class IteratorT >
		ScalarType Chi(const ScalarType* params, IteratorT begin, IteratorT end,
			ScalarType* values, ScalarType* temp) const
		{
			ScalarType chi = 0;
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				float s = begin[idx][0] - params[0];
				s *= s;
				for (unsigned int j = 1; j < 3; ++j)
				{
					float ss = begin[idx][j] - params[j];
					s += ss * ss;
				}
				values[idx] = WeightT::Weigh(std::sqrt(s) - params[3]);
				chi += values[idx] * values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType* params, IteratorT begin, IteratorT end,
			const ScalarType* values, const ScalarType* temp, ScalarType* matrix) const
		{
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				float s[3];
				s[0] = begin[idx][0] - params[0];
				float sl = s[0] * s[0];
				for (unsigned int i = 1; i < 3; ++i)
				{
					s[i] = begin[idx][i] - params[i];
					sl += s[i] * s[i];
				}
				sl = std::sqrt(sl);
				matrix[idx * NumParams + 0] = -s[0] / sl;
				matrix[idx * NumParams + 1] = -s[1] / sl;
				matrix[idx * NumParams + 2] = -s[2] / sl;
				matrix[idx * NumParams + 3] = -1;
				WeightT::template DerivWeigh< NumParams >(sl - params[3],
					matrix + idx * NumParams);
			}
		}

		void Normalize(ScalarType*) const
		{}
	};

};

class Torus : public SurfacePrimitive
{
public:

	Torus() :SurfacePrimitive(SurfaceType::UNKNOWN), m_rminor(0), m_rmajor(0), m_appleHeight(0), m_appleShaped(false), m_cutOffAngle(0) {}

	Torus(const vec3& point, const vec3& normal, float r_major, float r_minor) :SurfacePrimitive(SurfaceType::TORUS), m_center(point), m_normal(normalize(normal)), m_rminor(r_minor), m_rmajor(r_major), m_appleHeight(0), m_cutOffAngle(0)
	{
		m_appleShaped = m_rmajor < m_rminor;
		ComputeAppleParams();
	}

	virtual bool fit(const vector<Point>& pts)override
	{

		setType(SurfaceType::UNKNOWN);
	//***************************************** init
		if (pts.size() < 4)
			return false;

		// the algorithm used is as follows:
		// 1. The axis of rotation is estimated
		// 2. A spin image of the samples around the axis is constructed
		// 3. A circle is fitted to the samples in the spin image

		// 1. Axis of rotation
		// The formula used can be found in "Geometric least-squares fitting of
		// spheres, cylinders, cones and tori" by Lukacs, Marshall and Martin 1997
		// solve quadratic equation
		size_t k = pts.size();
		MiscLib::Vector< GfxTL::Vector3Dd > dsamples;
		dsamples.reserve(pts.size()*2);
		for (size_t i = 0; i < pts.size(); ++i)
			dsamples.push_back(GfxTL::Vector3Dd(pts[i].p.data()));
		for (size_t i = 0; i < pts.size(); ++i)
			dsamples.push_back(GfxTL::Vector3Dd(pts[i].n.data()));
		// run over all four tuples until two axes are succesfully
		// established
		vec3 pos1, normal1, pos2, normal2;
		for (size_t w = 0; w < k/* - 3*/; ++w)
		{
			for (size_t x = 0/*w + 1*/; x < k/* - 2*/; ++x)
			{
				GfxTL::Vector3Dd n0xn1 = dsamples[k + w] % dsamples[k + x];
				for (size_t y = 0/*x + 1*/; y < k/* - 1*/; ++y)
				{
					for (size_t z = 0/*y + 1*/; z < k; ++z)
					{
						double a01 = n0xn1 * dsamples[k + y];
						double b01 = n0xn1 * dsamples[k + z];
						if (GfxTL::Math< double >::Abs(a01) < 1.0e-6
							|| GfxTL::Math< double >::Abs(b01) < 1.0e-6)
							continue;
						double a0 = ((dsamples[y] - dsamples[x])
							% dsamples[k + w]) * dsamples[k + y];
						double a1 = ((dsamples[w] - dsamples[y])
							% dsamples[k + x]) * dsamples[k + y];
						double a = ((dsamples[w] - dsamples[y])
							% (dsamples[x] - dsamples[w])) * dsamples[k + y];
						double b0 = ((dsamples[z] - dsamples[x])
							% dsamples[k + w]) * dsamples[k + z];
						double b1 = ((dsamples[w] - dsamples[z])
							% dsamples[k + x]) * dsamples[k + z];
						double b = ((dsamples[w] - dsamples[z])
							% (dsamples[x] - dsamples[w])) * dsamples[k + z];
						double cc = b01 / a01;
						double ccc = b0 - a0 * cc;
						double c = -(b1 - a1 * cc) / ccc;
						double d = (-b + a * cc) / ccc;
						double p = (a0 * c + a1 + a01 * d) / (2 * a01 * c);
						double q = (a + a0 * d) / (a01 * c);
						double rt = p * p - q;
						if (rt < -1e-8)
							continue;
						if (rt < 0)
							rt = 0;
						double t1 = -p + std::sqrt(rt);
						double t2 = -p - std::sqrt(rt);
						double s1 = c * t1 + d;
						double s2 = c * t2 + d;
						pos1 = pts[w].p + s1 * pts[w].n;
						normal1 = pos1 - (pts[x].p + t1 * pts[x].n);
						normal1.normalize();
						pos2 = pts[w].p + s2 * pts[w].n;
						normal2 = pos2 - (pts[x].p + t2 * pts[x].n);
						normal2.normalize();
						goto foundAxis;
					}
				}
			}
		}
		return false;

	foundAxis:
		// at this point there are two possible solutions for the axis
		MiscLib::Vector< GfxTL::Vector2Df > spin1, spin2;
		SpinImage(pos1, normal1, pts.begin(), pts.begin() + k,
			std::back_inserter(spin1));
		SpinImage(pos2, normal2, pts.begin(), pts.begin() + k,
			std::back_inserter(spin2));
		float minorRadius1, majorRadius1, minorRadius2, majorRadius2,
			distSum1 = std::numeric_limits< float >::infinity(),
			distSum2 = std::numeric_limits< float >::infinity();
		GfxTL::Vector2Df minorCenter1, minorCenter2;
		if (CircleFrom3Points(spin1.begin(), &minorRadius1, &minorCenter1))
		{
			majorRadius1 = minorCenter1[0];
			// compute the distance of the points to the torus
			distSum1 = 0;
			for (size_t i = 3; i < spin1.size(); ++i) {
				float tmp = (spin1[i] - minorCenter1).Length() - minorRadius1;
				distSum1 += tmp * tmp;
			}
		}
		if (CircleFrom3Points(spin2.begin(), &minorRadius2, &minorCenter2))
		{
			majorRadius2 = minorCenter2[0];
			// compute the distance of the points to the torus
			distSum2 = 0;
			for (size_t i = 3; i < spin2.size(); ++i) {
				float tmp = (spin2[i] - minorCenter2).Length() - minorRadius2;
				distSum2 += tmp * tmp;
			}
		}
		if (distSum1 != std::numeric_limits< float >::infinity()
			&& distSum1 < distSum2)
		{
			m_normal = normal1;
			m_rminor = minorRadius1;
			m_rmajor = majorRadius1;
			m_center = pos1 + minorCenter1[1] * m_normal;
		}
		else if (distSum2 != std::numeric_limits< float >::infinity())
		{
			m_normal = normal2;
			m_rminor = minorRadius2;
			m_rmajor = majorRadius2;
			m_center = pos2 + minorCenter2[1] * m_normal;
		}
		else
			return false;
		m_appleShaped = m_rmajor < m_rminor;
		ComputeAppleParams();
		setType(SurfaceType::TORUS);
		return true;
	}

	virtual float signedDistance(const vec3& p) const override
	{
		vec3 s = p - m_center;
		float spin1 = dot(m_normal,s);
		float spin0 = (s - spin1 * m_normal).length();
		spin0 -= m_rmajor;
		if (!m_appleShaped)
			return std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor;
		// apple shaped torus distance
		float minorAngle = std::atan2(spin1, spin0); // minor angle
		if (fabs(minorAngle) < m_cutOffAngle)
			return std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor;
		spin0 += 2 * m_rmajor - m_rminor;
		if (minorAngle < 0)
			spin1 += m_appleHeight;
		else
			spin1 -= m_appleHeight;
		return -std::sqrt(spin0 * spin0 + spin1 * spin1);
	}

	virtual vec3 project(const vec3& p) const override
	{
		vec3 pp;
		vec3 s = p - m_center;
		float spin1 = dot(m_normal,s);
		vec3 tmp = spin1 * m_normal;
		float spin0 = (s - tmp).length();
		spin0 -= m_rmajor;
		if (m_appleShaped)
		{
			float minorAngle = std::atan2(spin1, spin0); // minor angle
			if (fabs(minorAngle) > m_cutOffAngle)
			{
				pp = m_center + GfxTL::Math< float >::Sign(minorAngle) * m_normal;
				return pp;
			}
		}
		vec3 pln = cross(s,m_normal);
		vec3 plx = cross(m_normal,pln);
		plx.normalize();
		float d = std::sqrt(spin0 * spin0 + spin1 * spin1);
		pp = m_center + (m_rminor / d) * (spin0 * plx + tmp)
			+ m_rmajor * plx;
		return pp;
	}

	virtual vec3 normal(const vec3& p) const override
	{
		vec3 n;
		vec3 s = p - m_center;
		float spin1 = dot(m_normal,s);
		vec3 tmp = spin1 * m_normal;
		float spin0 = (s - tmp).length();
		spin0 -= m_rmajor;
		if (m_appleShaped)
		{
			float minorAngle = std::atan2(spin1, spin0); // minor angle
			if (fabs(minorAngle) > m_cutOffAngle)
			{
				n = m_normal;
				if (minorAngle < 0)
					n *= -1;
				return n;
			}
		}
		vec3 pln = cross(s,m_normal);
		vec3 plx = cross(m_normal,pln);
		plx.normalize();
		n = spin0 * plx + tmp;
		n /= std::sqrt(spin0 * spin0 + spin1 * spin1);
		return n;
	}


	virtual SurfaceParameters getParameters() const
	{
		SurfaceParameters p;
		p.dir = m_normal;
		p.pos = m_center;
		p.r1 = m_rmajor;
		p.r2 = m_rminor;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters)
	{
		m_normal = new_parameters.dir;
		m_center = new_parameters.pos;
		m_rmajor = new_parameters.r1;
		m_rminor = new_parameters.r2;
		m_appleShaped = m_rmajor < m_rminor;
		ComputeAppleParams();
	}

private:
	vec3 m_normal;
	vec3 m_center;
	float m_rminor;
	float m_rmajor;
	bool m_appleShaped; // an apple shaped torus has rminor > rmajor
	float m_cutOffAngle; // for an apple shaped torus
		// the minor circle is cut off
	float m_appleHeight; // height of the "apple" point

	template< class InIteratorT, class OutIteratorT >
	static void SpinImage(const vec3& axisPos, const vec3& axisDir,
		InIteratorT begin, InIteratorT end, OutIteratorT out)
	{
		for (; begin != end; ++begin)
		{
			vec3 s = begin->p - axisPos;
			GfxTL::Vector2Df spin;
			spin[1] = dot(s,axisDir);
			spin[0] = (s - spin[1] * axisDir).length();
			*out = spin;
		}
	}

	template< class InIteratorT >
	static bool CircleFrom3Points(InIteratorT i, float* r,
		GfxTL::Vector2Df* center)
	{
		float a;
		float b;
		float bot;
		float c;
		float top1;
		float top2;
		a = std::sqrt(std::pow(i[1][0] - i[0][0], 2)
			+ std::pow(i[1][1] - i[0][1], 2));
		b = std::sqrt(std::pow(i[2][0] - i[1][0], 2)
			+ std::pow(i[2][1] - i[1][1], 2));
		c = std::sqrt(std::pow(i[0][0] - i[2][0], 2)
			+ std::pow(i[0][1] - i[2][1], 2));

		bot = (a + b + c) * (-a + b + c) * (a - b + c) * (a + b - c);

		if (bot <= 0.f)
			return false;

		*r = a * b * c / std::sqrt(bot);
		//  center.
		top1 = (i[1][1] - i[0][1]) * c * c - (i[2][1] - i[0][1]) * a * a;
		top2 = (i[1][0] - i[0][0]) * c * c - (i[2][0] - i[0][0]) * a * a;
		bot = (i[1][1] - i[0][1]) * (i[2][0] - i[0][0])
			- (i[2][1] - i[0][1]) * (i[1][0] - i[0][0]);

		(*center)[0] = i[0][0] + 0.5f * top1 / bot;
		(*center)[1] = i[0][1] - 0.5f * top2 / bot;
		return true;
	}

	void ComputeAppleParams()
	{
		if (!m_appleShaped)
		{
			m_cutOffAngle = static_cast<float>(M_PI);
			m_appleHeight = 0;
			return;
		}
		m_cutOffAngle = std::acos((2 * m_rmajor - m_rminor) / m_rminor) + M_PI / 2.f;
		m_appleHeight = std::sin(m_cutOffAngle) * m_rminor;
	}

};

class Cone : public SurfacePrimitive
{
public:

	Cone() : SurfacePrimitive(SurfaceType::UNKNOWN), m_angle(0)
	{
		m_n2d[0] = 0;
		m_n2d[1] = 0;
	};

	Cone(const vec3& center, const vec3& axisDir, float angle) :SurfacePrimitive(SurfaceType::CONE)
		,m_angle(angle),m_axisDir(axisDir),m_center(center)
	{
		m_normal = vec3(std::cos(-m_angle), std::sin(-m_angle), 0);
		m_normalY = m_normal[1] * m_axisDir;
		m_n2d[0] = std::cos(m_angle);
		m_n2d[1] = -std::sin(m_angle);
	}


	virtual bool fit(const vector<Point>& pts)override
	{
		setType(SurfaceType::UNKNOWN);
		//***************************** init
		// setup all the planes
		size_t c = pts.size();
		MiscLib::Vector< GfxTL::Vector4Df > planes(c);

		for (int i = 0; i < static_cast<int>(c); ++i)
		{
			for (unsigned int j = 0; j < 3; ++j)
				planes[i][j] = pts[i][j];
			planes[i][3] = dot(pts[i].p,pts[i].n);
		}
		// compute center by intersecting the three planes given by (p1, n1)
		// (p2, n2) and (p3, n3)
		// set up linear system
		double a[4 * 3];
		double d1 = dot(pts[0].p, pts[0].n);
		double d2 = dot(pts[1].p, pts[1].n);
		double d3 = dot(pts[2].p, pts[2].n);
		// column major
		a[0 + 0 * 3] = pts[0].n[0];
		a[1 + 0 * 3] = pts[1].n[0];
		a[2 + 0 * 3] = pts[2].n[0];
		a[0 + 1 * 3] = pts[0].n[1];
		a[1 + 1 * 3] = pts[1].n[1];
		a[2 + 1 * 3] = pts[2].n[1];
		a[0 + 2 * 3] = pts[0].n[2];
		a[1 + 2 * 3] = pts[1].n[2];
		a[2 + 2 * 3] = pts[2].n[2];
		a[0 + 3 * 3] = d1;
		a[1 + 3 * 3] = d2;
		a[2 + 3 * 3] = d3;
		if (levmar::dmat_solve(3, 1, a))
			return false;
		float m_center0[3];
		m_center0[0] = a[0 + 3 * 3];
		m_center0[1] = a[1 + 3 * 3];
		m_center0[2] = a[2 + 3 * 3];

		LevMarPlaneDistance planeDistance;
		levmar::LevMar(planes.begin(), planes.end(), planeDistance,
			m_center0);
		m_center = vec3(m_center0);

		MiscLib::Vector< GfxTL::Vector3Df > spoints(c);

		for (int i = 0; i < static_cast<int>(c); ++i)
		{
			spoints[i] = GfxTL::Vector3Df(pts[i].p - m_center);
			spoints[i].Normalize();
		}
		GfxTL::Vector3Df axisDir;
		GfxTL::MeanOfNormals(spoints.begin(), spoints.end(), &axisDir);
		m_axisDir = vec3(axisDir.Data());

		// make sure axis points in good direction
		// the axis is defined to point into the interior of the cone
		float heightSum = 0;

		for (int i = 0; i < static_cast<int>(c); ++i)
			heightSum += dot(m_axisDir, pts[i].p - m_center);
		if (heightSum < 0)
			m_axisDir *= -1;

		float angleReduction = 0;

		for (int i = 0; i < static_cast<int>(c); ++i)
		{
			float angle = dot(m_axisDir,pts[i].n);
			if (angle < -1) // clamp angle to [-1, 1]
				angle = -1;
			else if (angle > 1)
				angle = 1;
			if (angle < 0)
				// m_angle = omega + 90
				angle = std::acos(angle) - float(M_PI) / 2;
			else
				// m_angle = 90 - omega
				angle = float(M_PI) / 2 - std::acos(angle);
			angleReduction += angle;
		}
		angleReduction /= c;
		m_angle = angleReduction;
		if (m_angle < 1.0e-6 || m_angle > float(M_PI) / 2 - 1.0e-6)
			return false;
		//if(m_angle > 1.3962634015954636615389526147909) // 80 degrees
		if (m_angle > 1.4835298641951801403851371532153f) // 85 degrees
			return false;
		m_normal = vec3(std::cos(-m_angle), std::sin(-m_angle), 0);
		m_normalY = m_normal[1] * m_axisDir;
		m_n2d[0] = std::cos(m_angle);
		m_n2d[1] = -std::sin(m_angle);
		//***************************** levmar fitting
		float param[7];
		for (unsigned int i = 0; i < 3; ++i)
			param[i] = m_center[i];
		for (unsigned int i = 0; i < 3; ++i)
			param[i + 3] = m_axisDir[i];
		param[6] = m_angle;
		LevMarCone< levmar::LevMarLSWeight > levMarCone;
		if (!levmar::LevMar(pts.begin(), pts.end(), levMarCone, param))
			return false;
		if (param[6] < 1.0e-6 || param[6] > float(M_PI) / 2 - 1.0e-6)
			return false;
		for (unsigned int i = 0; i < 3; ++i)
			m_center[i] = param[i];
		for (unsigned int i = 0; i < 3; ++i)
			m_axisDir[i] = param[i + 3];
		m_angle = param[6];
		m_normal = vec3(std::cos(-m_angle), std::sin(-m_angle), 0);
		m_normalY = m_normal[1] * m_axisDir;
		m_n2d[0] = std::cos(m_angle);
		m_n2d[1] = -std::sin(m_angle);
		// it could be that the axis has flipped during fitting
		// we need to detect such a case
		// for this we run over all points and compute the sum
		// of their respective heights. If that sum is negative
		// the axis needs to be flipped.
		heightSum = 0;

		for (int i = 0; i < static_cast<int>(c); ++i)
			heightSum += dot(m_axisDir, pts[i].p - m_center);
		if (heightSum < 0)
		{
			m_axisDir *= -1;
		}


		//if (m_angle < M_PI*5./180 || m_angle > M_PI * 85. / 180)
		//	return false;

		setType(SurfaceType::CONE);
		return true;

	}

	virtual float signedDistance(const vec3& p) const override
	{
		// this is for one sided cone!
		vec3 s = p - m_center;
		float g = dot(s,m_axisDir); // distance to plane orhogonal to
			// axisdir through center
		// distance to axis
		float sqrS = s.length2();
		float f = sqrS - (g * g);
		if (f <= 0)
			f = 0;
		else
			f = std::sqrt(f);
		float da = m_n2d[0] * f;
		float db = m_n2d[1] * g;
		if (g < 0 && da - db < 0) // is inside other side of cone -> disallow
			return std::sqrt(sqrS);
		return da + db;
	}

	virtual vec3 project(const vec3& p) const override
	{
		vec3 pp;
		// this is for one-sided cone !!!
		vec3 s = p - m_center;
		float g = dot(s,m_axisDir); // distance to plane orhogonal to
			// axisdir through center
		// distance to axis
		float sqrS = s.length2();
		float f = sqrS - (g * g);
		if (f <= 0)
			f = 0;
		else
			f = std::sqrt(f);
		float da = m_n2d[0] * f;
		float db = m_n2d[1] * g;
		if (g < 0 && da - db < 0) // is inside other side of cone -> disallow
		{
			pp = m_center;
			return pp;
		}
		float dist = -(da + db);
		// need normal
		vec3 plx = s - g * m_axisDir;
		plx.normalize();
		vec3 n = m_normal[0] * plx + m_normalY;
		pp = p + dist * n;
		return pp;
	}

	virtual vec3 normal(const vec3& p) const override
	{
		vec3 n;
		vec3 s = p - m_center;
		vec3 pln = cross(s,m_axisDir);
		vec3 plx = cross(m_axisDir,pln);
		plx.normalize();
		// we are not dealing with two-sided cone
		n = m_normal[0] * plx + m_normalY;
		return n;
	}


	virtual SurfaceParameters getParameters() const
	{
		SurfaceParameters p;
		p.dir = m_axisDir;
		p.pos = m_center;
		p.r1 = m_angle;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters)
	{
		m_axisDir = new_parameters.dir;
		m_center = new_parameters.pos;
		m_angle = new_parameters.r1;

		m_normal = vec3(std::cos(-m_angle), std::sin(-m_angle), 0);
		m_normalY = m_normal[1] * m_axisDir;
		m_n2d[0] = std::cos(m_angle);
		m_n2d[1] = -std::sin(m_angle);
	}

private:
	vec3 m_center; // this is the  of the cone
	vec3 m_axisDir; // the axis points into the interior of the cone
	float m_angle; // the opening angle
	vec3 m_normal;
	vec3 m_normalY; // precomputed normal part
	float m_n2d[2];
private:
	class LevMarPlaneDistance
	{
	public:
		enum { NumParams = 3 };
		typedef float ScalarType;

		template< class IteratorT >
		ScalarType Chi(const ScalarType* params, IteratorT begin, IteratorT end,
			ScalarType* values, ScalarType* temp) const
		{
			ScalarType chi = 0;
			intptr_t size = end - begin;

			for (intptr_t i = 0; i < size; ++i)
			{
				values[i] = 0;
				for (unsigned int j = 0; j < 3; ++j)
					values[i] += begin[i][j] * params[j];
				values[i] -= begin[i][3];
				chi += values[i] * values[i];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType* params, IteratorT begin, IteratorT end,
			ScalarType* values, ScalarType* temp, ScalarType* matrix) const
		{
			intptr_t size = end - begin;

			for (intptr_t i = 0; i < size; ++i)
			{
				for (unsigned int j = 0; j < 3; ++j)
					matrix[i * NumParams + j] = begin[i][j];
			}
		}

		void Normalize(ScalarType*) const {}
	};

	template< class WeightT >
	class LevMarCone
		: public WeightT
	{
	public:
		enum { NumParams = 7 };
		typedef float ScalarType;

		template< class IteratorT >
		ScalarType Chi(const ScalarType* params, IteratorT begin, IteratorT end,
			ScalarType* values, ScalarType* temp) const
		{
			ScalarType chi = 0;
			ScalarType cosPhi = std::cos(params[6]);
			ScalarType sinPhi = std::sin(params[6]);
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				vec3 s;
				for (unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = fabs(s[0] * params[3] + s[1] * params[4] + s[2] * params[5]);
				ScalarType f = s.length2() - (g * g);
				if (f <= 0)
					f = 0;
				else
					f = std::sqrt(f);
				temp[idx] = f;
				chi += (values[idx] = WeightT::Weigh(cosPhi * f - sinPhi * g))
					* values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType* params, IteratorT begin, IteratorT end,
			const ScalarType* values, const ScalarType* temp, ScalarType* matrix) const
		{
			ScalarType sinPhi = -std::sin(params[6]);
			ScalarType cosPhi = std::cos(params[6]);
			int size = end - begin;

			for (int idx = 0; idx < size; ++idx)
			{
				vec3 s;
				for (unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = fabs(s[0] * params[3] + s[1] * params[4] + s[2] * params[5]);
				ScalarType ggradient[6];
				for (unsigned int j = 0; j < 3; ++j)
					ggradient[j] = -params[j + 3];
				for (unsigned int j = 0; j < 3; ++j)
					ggradient[j + 3] = s[j] - params[j + 3] * g;
				ScalarType fgradient[6];
				if (temp[idx] < 1.0e-6)
				{
					fgradient[0] = std::sqrt(1 - params[3] * params[3]);
					fgradient[1] = std::sqrt(1 - params[4] * params[4]);
					fgradient[2] = std::sqrt(1 - params[5] * params[5]);
				}
				else
				{
					fgradient[0] = (params[3] * g - s[0]) / temp[idx];
					fgradient[1] = (params[4] * g - s[1]) / temp[idx];
					fgradient[2] = (params[5] * g - s[2]) / temp[idx];
				}
				fgradient[3] = g * fgradient[0];
				fgradient[4] = g * fgradient[1];
				fgradient[5] = g * fgradient[2];
				for (unsigned int j = 0; j < 6; ++j)
					matrix[idx * NumParams + j] =
					cosPhi * fgradient[j] + sinPhi * ggradient[j];
				matrix[idx * NumParams + 6] = temp[idx] * sinPhi - g * cosPhi;
				WeightT::template DerivWeigh< NumParams >(cosPhi * temp[idx] + sinPhi * g,
					matrix + idx * NumParams);
			}
		}

		void Normalize(float* params) const
		{
			// normalize direction
			ScalarType l = std::sqrt(params[3] * params[3] + params[4] * params[4] +
				params[5] * params[5]);
			for (unsigned int i = 3; i < 6; ++i)
				params[i] /= l;
			// normalize angle
			params[6] -= std::floor(params[6] / (2 * ScalarType(M_PI))) * (2 * ScalarType(M_PI)); // params[6] %= 2*M_PI
			if (params[6] > M_PI)
			{
				params[6] -= std::floor(params[6] / ScalarType(M_PI)) * ScalarType(M_PI); // params[6] %= M_PI
				for (unsigned int i = 3; i < 6; ++i)
					params[i] *= -1;
			}
			if (params[6] > ScalarType(M_PI) / 2)
				params[6] = ScalarType(M_PI) - params[6];
		}
	};

};

class Ellipsoid : public SurfacePrimitive {
public:

	Ellipsoid() :SurfacePrimitive(SurfaceType::UNKNOWN) {
	};



	virtual bool fit(const vector<Point>& pts)override {

		setType(SurfaceType::UNKNOWN);
		/*
			X = [x.*x y.*y z.*z x.*y x.*z y.*z x y z];
			Y = ones(length(x), 1);

			C = inv(X'*X)*X' * Y;

			M = [C(1) C(4) / 2 C(5) / 2;
			     C(4) / 2 C(2) C(6) / 2;
			     C(5) / 2 C(6) / 2 C(3)];

			Cent = -0.5*[C(7) C(8) C(9)]*inv(M)

			S = Cent*M*Cent'+1;
			[U,V]=eig(M);

			[~,indx] = max(abs(U(1,:)));
			[~,indy] = max(abs(U(2,:)));
			[~,indz] = max(abs(U(3,:)));

			R = [sqrt(S/V(indx,indx)) sqrt(S/V(indy,indy)) sqrt(S/V(indz,indz))]
		*/
		try {
			Eigen::MatrixXd X(pts.size(), 7), Y = Eigen::MatrixXd::Ones(pts.size(), 1);

			for (int i = 0; i < pts.size(); i++) {
				auto p = pts[i].p;
				X(i, 0) = p.x * p.x;
				X(i, 1) = p.y * p.y;
				X(i, 2) = p.z * p.z;
				X(i, 3) = p.x * p.y;
				X(i, 4) = p.x * p.z;
				X(i, 5) = p.y * p.z;
				X(i, 6) = p.x;
				X(i, 7) = p.y;
				X(i, 8) = p.z;
			}

			auto C = (X.transpose() * X).inverse() * X.transpose() * Y;
			Eigen::Matrix3d M;
			M << C(0), C(3) / 2., C(4) / 2.,
				C(3) / 2., C(1), C(5) / 2.,
				C(4) / 2., C(5) / 2., C(2);
			auto Cent = -0.5 * Eigen::RowVector3d(C(6), C(7), C(8)) * M.inverse();
			auto S = (Cent * M * Cent.transpose())(0) + 1.;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(M);
			auto U = eig.eigenvectors();
			auto V = eig.eigenvalues();
			Eigen::Matrix3d::Index i0, i1, i2, r;
			U.row(0).maxCoeff(&r, &i0);
			U.row(1).maxCoeff(&r, &i1);
			U.row(2).maxCoeff(&r, &i2);
			auto r0 = sqrt(S / V(i0));
			auto r1 = sqrt(S / V(i1));
			auto r2 = sqrt(S / V(i2));

			m_center = vec3(Cent(0), Cent(1), Cent(2));
			m_half_axis = vec3(r0,r1,r2)
				;
			setType(SurfaceType::SPHERE);
		}
		catch (exception& e) {
			std::cout << "Error fitting ellipsolid." << std::endl<< e.what() << std::endl;
		}

		return true;
	}

	virtual float signedDistance(const vec3& p) const override {

		return 0;
	}

	virtual vec3 project(const vec3& p) const override {
		double D = max(m_half_axis[0], max(m_half_axis[1], m_half_axis[2]));
		Eigen::Vector3d pp(p[0] - m_center[0], p[1] - m_center[1], p[2] - m_center[2]);

		Eigen::Vector3d x=pp.normalized()*D;
		//Eigen::Vector3d x(0,0,0);
		Eigen::Vector3d r(
			1. / m_half_axis[0] / m_half_axis[0],
			1. / m_half_axis[1] / m_half_axis[1],
			1. / m_half_axis[2] / m_half_axis[2]);
		Eigen::Vector3d last_x=x;
		while (true) {
			auto t0 = pp - x;
			auto t1 = t0.norm();
			Eigen::Vector3d t2 = r.array() * x.array();
			auto t3 = t2.norm();


			auto t4 = 1. / (t1 * t3);
			auto t5 = t2.dot(t0);


			double cost = -t5 * t4+1;
			Eigen::Vector3d grad =
				-Eigen::Vector3d((t4 * t0).array()*r.array())
				+t4*t2
				+(t5/(t1*t3*t3*t3)* Eigen::Vector3d(t2.array() * r.array())-t5/(t1*t1*t1*t3)*t0)
				;

			Eigen::Vector3d t6 = x.array()* x.array()* r.array();
			auto t7 = t6.norm();

			double cost2 = (t7-1)* (t7 - 1);
			Eigen::Vector3d grad_constraint = (4 * (t7 - 1)) / t7 * Eigen::Vector3d(t6.array() * r.array() * x.array());

			if (grad.hasNaN() || grad_constraint.hasNaN() ) {
				auto ran = x.Random()*0.1*D;
				x = x + ran;
				std::cout << "NAN,Rand=" << ran << std::endl;
			} else {
				x = x - 0.5 * grad;
				x = x - 0.02 * grad_constraint;
			}
			if (grad.norm()  < 1e-4*D && cost>1e-2*D) {
				auto ran = x.Random() * 0.1 * D;
				x = x + ran;
				std::cout << "Local-G,Rand=" << ran << std::endl;
			}
			if (grad_constraint.norm() < 1e-4 * D && cost2 > 1e-2 * D) {
				auto ran = x.Random() * 0.1 * D;
				x = x + ran;
				std::cout << "Local-C,Rand=" << ran << std::endl;
			}
			std::cout << "Cost=" << cost<<","<<cost2 << std::endl
				<< "Grad=" << grad << std::endl
				<< "X=" << x << std::endl
				<< std::endl;
			if (cost+cost2 < 1e-4*D)
				break;
		}
		return vec3(x[0],x[1],x[2]);
	}

	virtual vec3 normal(const vec3& p) const override {
		return normalize(p - m_center);
	}


	virtual SurfaceParameters getParameters() const {
		SurfaceParameters p;
		p.pos = m_center;
		p.dir = m_half_axis;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters) {
		m_center = new_parameters.pos;
		m_half_axis = new_parameters.dir;
	}
private:
	vec3 m_center;
	vec3 m_half_axis;
};

class GeneralQuadric : public SurfacePrimitive {
public:

	GeneralQuadric() :SurfacePrimitive(SurfaceType::GENERALQUADRIC) {
	};


	class CG_PolynomialRoot {
		//polynomial  \sum_i=0^degree Coeff[i]*t^i
	private:
		vector<double> realroot;
		vector<complex<double> > complexroot;


		double* p, * qp, * k, * qk, * svk;
		double sr, si, u, v, a, b, c, d, a1, a2;
		double a3, a6, a7, e, f, g, h, szr, szi, lzr, lzi;
		double eta, are, mre;
		int n, nn, nmi, zerok;

		double tr, ti, pvr, pvi, infin;
		double* pr, * pi, * hr, * hi, * qpr, * qpi, * qhr, * qhi, * shr, * shi;

	public:
		CG_PolynomialRoot(vector<double>& Coeff, double epsilon = 1.0e-12) {
			int degree = (int)Coeff.size() - 1;

			double* opr = new double[degree + 1];
			double* zeror = new double[degree + 1];
			double* zeroi = new double[degree + 1];

			for (int i = 0; i <= degree; i++) {
				opr[i] = Coeff[degree - i];
			}

			rpoly(opr, degree, zeror, zeroi);

			realroot.clear();
			complexroot.clear();
			for (int i = 0; i < degree; i++) {
				complex<double> cr(zeror[i], zeroi[i]);
				complexroot.push_back(cr);
				if (abs(zeroi[i]) <= epsilon) {
					realroot.push_back(zeror[i]);
				}
			}

			delete[] opr;
			delete[] zeror;
			delete[] zeroi;
		}

		CG_PolynomialRoot(vector<complex<double> >& Coeff, double epsilon = 1.0e-12) {
			int degree = (int)Coeff.size() - 1;

			double* opr = new double[degree + 1];
			double* opi = new double[degree + 1];
			double* zeror = new double[degree + 1];
			double* zeroi = new double[degree + 1];

			for (int i = 0; i <= degree; i++) {
				opr[i] = Coeff[degree - i].real();
				opi[i] = Coeff[degree - i].imag();
			}

			cpoly(opr, opi, degree, zeror, zeroi);

			realroot.clear();
			complexroot.clear();
			for (int i = 0; i < degree; i++) {
				complex<double> cr(zeror[i], zeroi[i]);
				complexroot.push_back(cr);
				if (abs(zeroi[i]) <= epsilon) {
					realroot.push_back(zeror[i]);
				}
			}

			delete[] opr;
			delete[] opi;
			delete[] zeror;
			delete[] zeroi;

		}

		~CG_PolynomialRoot() {
		}


		vector<double>& get_realroot() {
			return realroot;
		}

		vector<complex<double> >& get_allroot() {
			return complexroot;
		}

	private:

		//for real
		int rpoly(double* op, int degree, double* zeror, double* zeroi) {
			double t, aa, bb, cc, * temp, factor, rot;
			double* pt;
			double lo, max, min, xx, yy, cosr, sinr, xxx, x, sc, bnd;
			double xm, ff, df, dx, infin, smalno, base;
			int cnt, nz, i, j, jj, l, nm1, zerok;
			/*  The following statements set machine constants. */
			base = 2.0;
			eta = 2.22e-16;
			infin = 3.4e38;
			smalno = 1.2e-38;

			are = eta;
			mre = eta;
			lo = smalno / eta;
			/*  Initialization of constants for shift rotation. */
			xx = sqrt(0.5);
			yy = -xx;
			rot = 94.0;
			rot *= 0.017453293;
			cosr = cos(rot);
			sinr = sin(rot);
			n = degree;
			/*  Algorithm fails of the leading coefficient is zero. */
			if (op[0] == 0.0) return -1;
			/*  Remove the zeros at the origin, if any. */
			while (op[n] == 0.0) {
				j = degree - n;
				zeror[j] = 0.0;
				zeroi[j] = 0.0;
				n--;
			}
			if (n < 1) return -1;
			/*
			*  Allocate memory here
			*/
			temp = new double[degree + 1];
			pt = new double[degree + 1];
			p = new double[degree + 1];
			qp = new double[degree + 1];
			k = new double[degree + 1];
			qk = new double[degree + 1];
			svk = new double[degree + 1];
			/*  Make a copy of the coefficients. */
			for (i = 0; i <= n; i++)
				p[i] = op[i];
			/*  Start the algorithm for one zero. */
		_40:
			if (n == 1) {
				zeror[degree - 1] = -p[1] / p[0];
				zeroi[degree - 1] = 0.0;
				n -= 1;
				goto _99;
			}
			/*  Calculate the final zero or pair of zeros. */
			if (n == 2) {
				quad(p[0], p[1], p[2], &zeror[degree - 2], &zeroi[degree - 2],
					&zeror[degree - 1], &zeroi[degree - 1]);
				n -= 2;
				goto _99;
			}
			/*  Find largest and smallest moduli of coefficients. */
			max = 0.0;
			min = infin;
			for (i = 0; i <= n; i++) {
				x = fabs(p[i]);
				if (x > max) max = x;
				if (x != 0.0 && x < min) min = x;
			}
			/*  Scale if there are large or very small coefficients.
			*  Computes a scale factor to multiply the coefficients of the
			*  polynomial. The scaling si done to avoid overflow and to
			*  avoid undetected underflow interfering with the convergence
			*  criterion. The factor is a power of the base.
			*/
			sc = lo / min;
			if (sc > 1.0 && infin / sc < max) goto _110;
			if (sc <= 1.0) {
				if (max < 10.0) goto _110;
				if (sc == 0.0)
					sc = smalno;
			}
			l = (int)(log(sc) / log(base) + 0.5);
			factor = pow(base * 1.0, l);
			if (factor != 1.0) {
				for (i = 0; i <= n; i++)
					p[i] = factor * p[i];     /* Scale polynomial. */
			}
		_110:
			/*  Compute lower bound on moduli of roots. */
			for (i = 0; i <= n; i++) {
				pt[i] = (fabs(p[i]));
			}
			pt[n] = -pt[n];
			/*  Compute upper estimate of bound. */
			x = exp((log(-pt[n]) - log(pt[0])) / (double)n);
			/*  If Newton step at the origin is better, use it. */
			if (pt[n - 1] != 0.0) {
				xm = -pt[n] / pt[n - 1];
				if (xm < x)  x = xm;
			}
			/*  Chop the interval (0,x) until ff <= 0 */
			while (1) {
				xm = x * 0.1;
				ff = pt[0];
				for (i = 1; i <= n; i++)
					ff = ff * xm + pt[i];
				if (ff <= 0.0) break;
				x = xm;
			}
			dx = x;
			/*  Do Newton interation until x converges to two
			*  decimal places.
			*/
			while (fabs(dx / x) > 0.005) {
				ff = pt[0];
				df = ff;
				for (i = 1; i < n; i++) {
					ff = ff * x + pt[i];
					df = df * x + ff;
				}
				ff = ff * x + pt[n];
				dx = ff / df;
				x -= dx;
			}
			bnd = x;
			/*  Compute the derivative as the initial k polynomial
			*  and do 5 steps with no shift.
			*/
			nm1 = n - 1;
			for (i = 1; i < n; i++)
				k[i] = (double)(n - i) * p[i] / (double)n;
			k[0] = p[0];
			aa = p[n];
			bb = p[n - 1];
			zerok = (k[n - 1] == 0);
			for (jj = 0; jj < 5; jj++) {
				cc = k[n - 1];
				if (!zerok) {
					/*  Use a scaled form of recurrence if value of k at 0 is nonzero. */
					t = -aa / cc;
					for (i = 0; i < nm1; i++) {
						j = n - i - 1;
						k[j] = t * k[j - 1] + p[j];
					}
					k[0] = p[0];
					zerok = (fabs(k[n - 1]) <= fabs(bb) * eta * 10.0);
				} else {
					/*  Use unscaled form of recurrence. */
					for (i = 0; i < nm1; i++) {
						j = n - i - 1;
						k[j] = k[j - 1];
					}
					k[0] = 0.0;
					zerok = (k[n - 1] == 0.0);
				}
			}
			/*  Save k for restarts with new shifts. */
			for (i = 0; i < n; i++)
				temp[i] = k[i];
			/*  Loop to select the quadratic corresponding to each new shift. */
			for (cnt = 0; cnt < 20; cnt++) {
				/*  Quadratic corresponds to a double shift to a
				*  non-real point and its complex conjugate. The point
				*  has modulus bnd and amplitude rotated by 94 degrees
				*  from the previous shift.
				*/
				xxx = cosr * xx - sinr * yy;
				yy = sinr * xx + cosr * yy;
				xx = xxx;
				sr = bnd * xx;
				si = bnd * yy;
				u = -2.0 * sr;
				v = bnd;
				fxshfr(20 * (cnt + 1), &nz);
				if (nz != 0) {
					/*  The second stage jumps directly to one of the third
					*  stage iterations and returns here if successful.
					*  Deflate the polynomial, store the zero or zeros and
					*  return to the main algorithm.
					*/
					j = degree - n;
					zeror[j] = szr;
					zeroi[j] = szi;
					n -= nz;
					for (i = 0; i <= n; i++)
						p[i] = qp[i];
					if (nz != 1) {
						zeror[j + 1] = lzr;
						zeroi[j + 1] = lzi;
					}
					goto _40;
				}
				/*  If the iteration is unsuccessful another quadratic
				*  is chosen after restoring k.
				*/
				for (i = 0; i < n; i++) {
					k[i] = temp[i];
				}
			}
			/*  Return with failure if no convergence with 20 shifts. */
		_99:
			delete[] svk;
			delete[] qk;
			delete[] k;
			delete[] qp;
			delete[] p;
			delete[] pt;
			delete[] temp;

			return degree - n;
		}
		/*  Computes up to L2 fixed shift k-polynomials,
		*  testing for convergence in the linear or quadratic
		*  case. Initiates one of the variable shift
		*  iterations and returns with the number of zeros
		*  found.
		*/
		void fxshfr(int l2, int* nz) {
			double svu, svv, ui, vi, s;
			double betas, betav, oss, ovv, ss, vv, ts, tv;
			double ots, otv, tvv, tss;
			int type, i, j, iflag, vpass, spass, vtry, stry;

			*nz = 0;
			betav = 0.25;
			betas = 0.25;
			oss = sr;
			ovv = v;
			/*  Evaluate polynomial by synthetic division. */
			quadsd(n, &u, &v, p, qp, &a, &b);
			calcsc(&type);
			for (j = 0; j < l2; j++) {
				/*  Calculate next k polynomial and estimate v. */
				nextk(&type);
				calcsc(&type);
				newest(type, &ui, &vi);
				vv = vi;
				/*  Estimate s. */
				ss = 0.0;
				if (k[n - 1] != 0.0) ss = -p[n] / k[n - 1];
				tv = 1.0;
				ts = 1.0;
				if (j == 0 || type == 3) goto _70;
				/*  Compute relative measures of convergence of s and v sequences. */
				if (vv != 0.0) tv = fabs((vv - ovv) / vv);
				if (ss != 0.0) ts = fabs((ss - oss) / ss);
				/*  If decreasing, multiply two most recent convergence measures. */
				tvv = 1.0;
				if (tv < otv) tvv = tv * otv;
				tss = 1.0;
				if (ts < ots) tss = ts * ots;
				/*  Compare with convergence criteria. */
				vpass = (tvv < betav);
				spass = (tss < betas);
				if (!(spass || vpass)) goto _70;
				/*  At least one sequence has passed the convergence test.
				*  Store variables before iterating.
				*/
				svu = u;
				svv = v;
				for (i = 0; i < n; i++) {
					svk[i] = k[i];
				}
				s = ss;
				/*  Choose iteration according to the fastest converging
				*  sequence.
				*/
				vtry = 0;
				stry = 0;
				if (spass && (!vpass) || tss < tvv) goto _40;
			_20:
				quadit(&ui, &vi, nz);
				if (*nz > 0) return;
				/*  Quadratic iteration has failed. Flag that it has
				*  been tried and decrease the convergence criterion.
				*/
				vtry = 1;
				betav *= 0.25;
				/*  Try linear iteration if it has not been tried and
				*  the S sequence is converging.
				*/
				if (stry || !spass) goto _50;
				for (i = 0; i < n; i++) {
					k[i] = svk[i];
				}
			_40:
				realit(&s, nz, &iflag);
				if (*nz > 0) return;
				/*  Linear iteration has failed. Flag that it has been
				*  tried and decrease the convergence criterion.
				*/
				stry = 1;
				betas *= 0.25;
				if (iflag == 0) goto _50;
				/*  If linear iteration signals an almost double real
				*  zero attempt quadratic iteration.
				*/
				ui = -(s + s);
				vi = s * s;
				goto _20;
				/*  Restore variables. */
			_50:
				u = svu;
				v = svv;
				for (i = 0; i < n; i++) {
					k[i] = svk[i];
				}
				/*  Try quadratic iteration if it has not been tried
				*  and the V sequence is convergin.
				*/
				if (vpass && !vtry) goto _20;
				/*  Recompute QP and scalar values to continue the
				*  second stage.
				*/
				quadsd(n, &u, &v, p, qp, &a, &b);
				calcsc(&type);
			_70:
				ovv = vv;
				oss = ss;
				otv = tv;
				ots = ts;
			}
		}
		/*  Variable-shift k-polynomial iteration for a
		*  quadratic factor converges only if the zeros are
		*  equimodular or nearly so.
		*  uu, vv - coefficients of starting quadratic.
		*  nz - number of zeros found.
		*/
		void quadit(double* uu, double* vv, int* nz) {
			double ui, vi;
			double mp, omp, ee, relstp, t, zm;
			int type, i, j, tried;

			*nz = 0;
			tried = 0;
			u = *uu;
			v = *vv;
			j = 0;
			/*  Main loop. */
		_10:
			quad(1.0, u, v, &szr, &szi, &lzr, &lzi);
			/*  Return if roots of the quadratic are real and not
			*  close to multiple or nearly equal and of opposite
			*  sign.
			*/
			if (fabs(fabs(szr) - fabs(lzr)) > 0.01 * fabs(lzr)) return;
			/*  Evaluate polynomial by quadratic synthetic division. */
			quadsd(n, &u, &v, p, qp, &a, &b);
			mp = fabs(a - szr * b) + fabs(szi * b);
			/*  Compute a rigorous bound on the rounding error in
			*  evaluating p.
			*/
			zm = sqrt(fabs(v));
			ee = 2.0 * fabs(qp[0]);
			t = -szr * b;
			for (i = 1; i < n; i++) {
				ee = ee * zm + fabs(qp[i]);
			}
			ee = ee * zm + fabs(a + t);
			ee *= (5.0 * mre + 4.0 * are);
			ee = ee - (5.0 * mre + 2.0 * are) * (fabs(a + t) + fabs(b) * zm) + 2.0 * are * fabs(t);
			/*  Iteration has converged sufficiently if the
			*  polynomial value is less than 20 times this bound.
			*/
			if (mp <= 20.0 * ee) {
				*nz = 2;
				return;
			}
			j++;
			/*  Stop iteration after 20 steps. */
			if (j > 20) return;
			if (j < 2) goto _50;
			if (relstp > 0.01 || mp < omp || tried) goto _50;
			/*  A cluster appears to be stalling the convergence.
			*  Five fixed shift steps are taken with a u,v close
			*  to the cluster.
			*/
			if (relstp < eta) relstp = eta;
			relstp = sqrt(relstp);
			u = u - u * relstp;
			v = v + v * relstp;
			quadsd(n, &u, &v, p, qp, &a, &b);
			for (i = 0; i < 5; i++) {
				calcsc(&type);
				nextk(&type);
			}
			tried = 1;
			j = 0;
		_50:
			omp = mp;
			/*  Calculate next k polynomial and new u and v. */
			calcsc(&type);
			nextk(&type);
			calcsc(&type);
			newest(type, &ui, &vi);
			/*  If vi is zero the iteration is not converging. */
			if (vi == 0.0) return;
			relstp = fabs((vi - v) / vi);
			u = ui;
			v = vi;
			goto _10;
		}
		/*  Variable-shift H polynomial iteration for a real zero.
		*  sss - starting iterate
		*  nz  - number of zeros found
		*  iflag - flag to indicate a pair of zeros near real axis.
		*/
		void realit(double* sss, int* nz, int* iflag) {
			double pv, kv, t, s;
			double ms, mp, omp, ee;
			int i, j;

			*nz = 0;
			s = *sss;
			*iflag = 0;
			j = 0;
			/*  Main loop */
			while (1) {
				pv = p[0];
				/*  Evaluate p at s. */
				qp[0] = pv;
				for (i = 1; i <= n; i++) {
					pv = pv * s + p[i];
					qp[i] = pv;
				}
				mp = fabs(pv);
				/*  Compute a rigorous bound on the error in evaluating p. */
				ms = fabs(s);
				ee = (mre / (are + mre)) * fabs(qp[0]);
				for (i = 1; i <= n; i++) {
					ee = ee * ms + fabs(qp[i]);
				}
				/*  Iteration has converged sufficiently if the polynomial
				*  value is less than 20 times this bound.
				*/
				if (mp <= 20.0 * ((are + mre) * ee - mre * mp)) {
					*nz = 1;
					szr = s;
					szi = 0.0;
					return;
				}
				j++;
				/*  Stop iteration after 10 steps. */
				if (j > 10) return;
				if (j < 2) goto _50;
				if (fabs(t) > 0.001 * fabs(s - t) || mp < omp) goto _50;
				/*  A cluster of zeros near the real axis has been
				*  encountered. Return with iflag set to initiate a
				*  quadratic iteration.
				*/
				*iflag = 1;
				*sss = s;
				return;
				/*  Return if the polynomial value has increased significantly. */
			_50:
				omp = mp;
				/*  Compute t, the next polynomial, and the new iterate. */
				kv = k[0];
				qk[0] = kv;
				for (i = 1; i < n; i++) {
					kv = kv * s + k[i];
					qk[i] = kv;
				}
				if (fabs(kv) <= fabs(k[n - 1]) * 10.0 * eta) {
					/*  Use unscaled form. */
					k[0] = 0.0;
					for (i = 1; i < n; i++) {
						k[i] = qk[i - 1];
					}
				} else {
					/*  Use the scaled form of the recurrence if the value
					*  of k at s is nonzero.
					*/
					t = -pv / kv;
					k[0] = qp[0];
					for (i = 1; i < n; i++) {
						k[i] = t * qk[i - 1] + qp[i];
					}
				}
				kv = k[0];
				for (i = 1; i < n; i++) {
					kv = kv * s + k[i];
				}
				t = 0.0;
				if (fabs(kv) > fabs(k[n - 1] * 10.0 * eta)) t = -pv / kv;
				s += t;
			}
		}

		/*  This routine calculates scalar quantities used to
		*  compute the next k polynomial and new estimates of
		*  the quadratic coefficients.
		*  type - integer variable set here indicating how the
		*  calculations are normalized to avoid overflow.
		*/
		void calcsc(int* type) {
			/*  Synthetic division of k by the quadratic 1,u,v */
			quadsd(n - 1, &u, &v, k, qk, &c, &d);
			if (fabs(c) > fabs(k[n - 1] * 100.0 * eta)) goto _10;
			if (fabs(d) > fabs(k[n - 2] * 100.0 * eta)) goto _10;
			*type = 3;
			/*  Type=3 indicates the quadratic is almost a factor of k. */
			return;
		_10:
			if (fabs(d) < fabs(c)) {
				*type = 1;
				/*  Type=1 indicates that all formulas are divided by c. */
				e = a / c;
				f = d / c;
				g = u * e;
				h = v * b;
				a3 = a * e + (h / c + g) * b;
				a1 = b - a * (d / c);
				a7 = a + g * d + h * f;
				return;
			}
			*type = 2;
			/*  Type=2 indicates that all formulas are divided by d. */
			e = a / d;
			f = c / d;
			g = u * b;
			h = v * b;
			a3 = (a + g) * e + h * (b / d);
			a1 = b * f - a;
			a7 = (f + u) * a + h;
		}
		/*  Computes the next k polynomials using scalars
		*  computed in calcsc.
		*/
		void nextk(int* type) {
			double temp;
			int i;

			if (*type == 3) {
				/*  Use unscaled form of the recurrence if type is 3. */
				k[0] = 0.0;
				k[1] = 0.0;
				for (i = 2; i < n; i++) {
					k[i] = qk[i - 2];
				}
				return;
			}
			temp = a;
			if (*type == 1) temp = b;
			if (fabs(a1) <= fabs(temp) * eta * 10.0) {
				/*  If a1 is nearly zero then use a special form of the
				*  recurrence.
				*/
				k[0] = 0.0;
				k[1] = -a7 * qp[0];
				for (i = 2; i < n; i++) {
					k[i] = a3 * qk[i - 2] - a7 * qp[i - 1];
				}
				return;
			}
			/*  Use scaled form of the recurrence. */
			a7 /= a1;
			a3 /= a1;
			k[0] = qp[0];
			k[1] = qp[1] - a7 * qp[0];
			for (i = 2; i < n; i++) {
				k[i] = a3 * qk[i - 2] - a7 * qp[i - 1] + qp[i];
			}
		}
		/*  Compute new estimates of the quadratic coefficients
		*  using the scalars computed in calcsc.
		*/
		void newest(int type, double* uu, double* vv) {
			double a4, a5, b1, b2, c1, c2, c3, c4, temp;

			/* Use formulas appropriate to setting of type. */
			if (type == 3) {
				/*  If type=3 the quadratic is zeroed. */
				*uu = 0.0;
				*vv = 0.0;
				return;
			}
			if (type == 2) {
				a4 = (a + g) * f + h;
				a5 = (f + u) * c + v * d;
			} else {
				a4 = a + u * b + h * f;
				a5 = c + (u + v * f) * d;
			}
			/*  Evaluate new quadratic coefficients. */
			b1 = -k[n - 1] / p[n];
			b2 = -(k[n - 2] + b1 * p[n - 1]) / p[n];
			c1 = v * b2 * a1;
			c2 = b1 * a7;
			c3 = b1 * b1 * a3;
			c4 = c1 - c2 - c3;
			temp = a5 + b1 * a4 - c4;
			if (temp == 0.0) {
				*uu = 0.0;
				*vv = 0.0;
				return;
			}
			*uu = u - (u * (c3 + c2) + v * (b1 * a1 + b2 * a7)) / temp;
			*vv = v * (1.0 + c4 / temp);
			return;
		}

		/*  Divides p by the quadratic 1,u,v placing the quotient
		*  in q and the remainder in a,b.
		*/
		void quadsd(int nn, double* u, double* v, double* p, double* q,
			double* a, double* b) {
			double c;
			int i;
			*b = p[0];
			q[0] = *b;
			*a = p[1] - (*b) * (*u);
			q[1] = *a;
			for (i = 2; i <= nn; i++) {
				c = p[i] - (*a) * (*u) - (*b) * (*v);
				q[i] = c;
				*b = *a;
				*a = c;
			}
		}
		/*  Calculate the zeros of the quadratic a*z^2 + b1*z + c.
		*  The quadratic formula, modified to avoid overflow, is used
		*  to find the larger zero if the zeros are real and both
		*  are complex. The smaller real zero is found directly from
		*  the product of the zeros c/a.
		*/
		void quad(double a, double b1, double c, double* sr, double* si,
			double* lr, double* li) {
			double b, d, e;

			if (a == 0.0) {         /* less than two roots */
				if (b1 != 0.0)
					*sr = -c / b1;
				else
					*sr = 0.0;
				*lr = 0.0;
				*si = 0.0;
				*li = 0.0;
				return;
			}
			if (c == 0.0) {         /* one real root, one zero root */
				*sr = 0.0;
				*lr = -b1 / a;
				*si = 0.0;
				*li = 0.0;
				return;
			}
			/* Compute discriminant avoiding overflow. */
			b = b1 / 2.0;
			if (fabs(b) < fabs(c)) {
				if (c < 0.0)
					e = -a;
				else
					e = a;
				e = b * (b / fabs(c)) - e;
				d = sqrt(fabs(e)) * sqrt(fabs(c));
			} else {
				e = 1.0 - (a / b) * (c / b);
				d = sqrt(fabs(e)) * fabs(b);
			}
			if (e < 0.0) {      /* complex conjugate zeros */
				*sr = -b / a;
				*lr = *sr;
				*si = fabs(d / a);
				*li = -(*si);
			} else {
				if (b >= 0.0)   /* real zeros. */
					d = -d;
				*lr = (-b + d) / a;
				*sr = 0.0;
				if (*lr != 0.0)
					*sr = (c / *lr) / a;
				*si = 0.0;
				*li = 0.0;
			}
		}




		//for complex

		int cpoly(const double* opr, const double* opi, int degree, double* zeror, double* zeroi) {
			int cnt1, cnt2, idnn2, i, conv;
			double xx, yy, cosr, sinr, smalno, base, xxx, zr, zi, bnd;

			mcon(&eta, &infin, &smalno, &base);
			are = eta;
			mre = 2.0 * sqrt(2.0) * eta;
			xx = 0.70710678;
			yy = -xx;
			cosr = -0.060756474;
			sinr = -0.99756405;
			nn = degree;

			// Algorithm fails if the leading coefficient is zero
			if (opr[0] == 0 && opi[0] == 0)
				return -1;

			// Allocate arrays
			pr = new double[degree + 1];
			pi = new double[degree + 1];
			hr = new double[degree + 1];
			hi = new double[degree + 1];
			qpr = new double[degree + 1];
			qpi = new double[degree + 1];
			qhr = new double[degree + 1];
			qhi = new double[degree + 1];
			shr = new double[degree + 1];
			shi = new double[degree + 1];

			// Remove the zeros at the origin if any
			while (opr[nn] == 0 && opi[nn] == 0) {
				idnn2 = degree - nn;
				zeror[idnn2] = 0;
				zeroi[idnn2] = 0;
				nn--;
			}

			// Make a copy of the coefficients
			for (i = 0; i <= nn; i++) {
				pr[i] = opr[i];
				pi[i] = opi[i];
				shr[i] = cmod(pr[i], pi[i]);
			}

			// Scale the polynomial
			bnd = scale(nn, shr, eta, infin, smalno, base);
			if (bnd != 1)
				for (i = 0; i <= nn; i++) {
					pr[i] *= bnd;
					pi[i] *= bnd;
				}

		search:
			if (nn <= 1) {
				cdivid(-pr[1], -pi[1], pr[0], pi[0], &zeror[degree - 1], &zeroi[degree - 1]);
				goto finish;
			}

			// Calculate bnd, alower bound on the modulus of the zeros
			for (i = 0; i <= nn; i++)
				shr[i] = cmod(pr[i], pi[i]);

			cauchy(nn, shr, shi, &bnd);

			// Outer loop to control 2 Major passes with different sequences of shifts
			for (cnt1 = 1; cnt1 <= 2; cnt1++) {
				// First stage  calculation , no shift
				noshft(5);

				// Inner loop to select a shift
				for (cnt2 = 1; cnt2 <= 9; cnt2++) {
					// Shift is chosen with modulus bnd and amplitude rotated by 94 degree from the previous shif
					xxx = cosr * xx - sinr * yy;
					yy = sinr * xx + cosr * yy;
					xx = xxx;
					sr = bnd * xx;
					si = bnd * yy;

					// Second stage calculation, fixed shift
					fxshft(10 * cnt2, &zr, &zi, &conv);
					if (conv) {
						// The second stage jumps directly to the third stage ieration
						// If successful the zero is stored and the polynomial deflated
						idnn2 = degree - nn;
						zeror[idnn2] = zr;
						zeroi[idnn2] = zi;
						nn--;
						for (i = 0; i <= nn; i++) {
							pr[i] = qpr[i];
							pi[i] = qpi[i];
						}
						goto search;
					}
					// If the iteration is unsuccessful another shift is chosen
				}
				// if 9 shifts fail, the outer loop is repeated with another sequence of shifts
			}

			// The zerofinder has failed on two major passes
			// return empty handed with the number of roots found (less than the original degree)
			degree -= nn;

		finish:
			// Deallocate arrays
			delete[] pr;
			delete[] pi;
			delete[] hr;
			delete[] hi;
			delete[] qpr;
			delete[] qpi;
			delete[] qhr;
			delete[] qhi;
			delete[] shr;
			delete[] shi;

			return degree;
		}


		// COMPUTES  THE DERIVATIVE  POLYNOMIAL AS THE INITIAL H
		// POLYNOMIAL AND COMPUTES L1 NO-SHIFT H POLYNOMIALS.
		//
		void noshft(const int l1) {
			int i, j, jj, n, nm1;
			double xni, t1, t2;

			n = nn;
			nm1 = n - 1;
			for (i = 0; i < n; i++) {
				xni = nn - i;
				hr[i] = xni * pr[i] / n;
				hi[i] = xni * pi[i] / n;
			}
			for (jj = 1; jj <= l1; jj++) {
				if (cmod(hr[n - 1], hi[n - 1]) > eta * 10 * cmod(pr[n - 1], pi[n - 1])) {
					cdivid(-pr[nn], -pi[nn], hr[n - 1], hi[n - 1], &tr, &ti);
					for (i = 0; i < nm1; i++) {
						j = nn - i - 1;
						t1 = hr[j - 1];
						t2 = hi[j - 1];
						hr[j] = tr * t1 - ti * t2 + pr[j];
						hi[j] = tr * t2 + ti * t1 + pi[j];
					}
					hr[0] = pr[0];
					hi[0] = pi[0];
				} else {
					// If the constant term is essentially zero, shift H coefficients
					for (i = 0; i < nm1; i++) {
						j = nn - i - 1;
						hr[j] = hr[j - 1];
						hi[j] = hi[j - 1];
					}
					hr[0] = 0;
					hi[0] = 0;
				}
			}
		}

		// COMPUTES L2 FIXED-SHIFT H POLYNOMIALS AND TESTS FOR CONVERGENCE.
		// INITIATES A VARIABLE-SHIFT ITERATION AND RETURNS WITH THE
		// APPROXIMATE ZERO IF SUCCESSFUL.
		// L2 - LIMIT OF FIXED SHIFT STEPS
		// ZR,ZI - APPROXIMATE ZERO IF CONV IS .TRUE.
		// CONV  - LOGICAL INDICATING CONVERGENCE OF STAGE 3 ITERATION
		//
		void fxshft(const int l2, double* zr, double* zi, int* conv) {
			int i, j, n;
			int test, pasd, bol;
			double otr, oti, svsr, svsi;

			n = nn;
			polyev(nn, sr, si, pr, pi, qpr, qpi, &pvr, &pvi);
			test = 1;
			pasd = 0;

			// Calculate first T = -P(S)/H(S)
			calct(&bol);

			// Main loop for second stage
			for (j = 1; j <= l2; j++) {
				otr = tr;
				oti = ti;

				// Compute the next H Polynomial and new t
				nexth(bol);
				calct(&bol);
				*zr = sr + tr;
				*zi = si + ti;

				// Test for convergence unless stage 3 has failed once or this
				// is the last H Polynomial
				if (!(bol || !test || j == 12))
					if (cmod(tr - otr, ti - oti) < 0.5 * cmod(*zr, *zi)) {
						if (pasd) {
							// The weak convergence test has been passwed twice, start the third stage
							// Iteration, after saving the current H polynomial and shift
							for (i = 0; i < n; i++) {
								shr[i] = hr[i];
								shi[i] = hi[i];
							}
							svsr = sr;
							svsi = si;
							vrshft(10, zr, zi, conv);
							if (*conv) return;

							//The iteration failed to converge. Turn off testing and restore h,s,pv and T
							test = 0;
							for (i = 0; i < n; i++) {
								hr[i] = shr[i];
								hi[i] = shi[i];
							}
							sr = svsr;
							si = svsi;
							polyev(nn, sr, si, pr, pi, qpr, qpi, &pvr, &pvi);
							calct(&bol);
							continue;
						}
						pasd = 1;
					} else
						pasd = 0;
			}

			// Attempt an iteration with final H polynomial from second stage
			vrshft(10, zr, zi, conv);
		}

		// CARRIES OUT THE THIRD STAGE ITERATION.
		// L3 - LIMIT OF STEPS IN STAGE 3.
		// ZR,ZI   - ON ENTRY CONTAINS THE INITIAL ITERATE, IF THE
		//           ITERATION CONVERGES IT CONTAINS THE FINAL ITERATE ON EXIT.
		// CONV    -  .TRUE. IF ITERATION CONVERGES
		//
		void vrshft(const int l3, double* zr, double* zi, int* conv) {
			int b, bol;
			int i, j;
			double mp, ms, omp, relstp, r1, r2, tp;

			*conv = 0;
			b = 0;
			sr = *zr;
			si = *zi;

			// Main loop for stage three
			for (i = 1; i <= l3; i++) {
				// Evaluate P at S and test for convergence
				polyev(nn, sr, si, pr, pi, qpr, qpi, &pvr, &pvi);
				mp = cmod(pvr, pvi);
				ms = cmod(sr, si);
				if (mp <= 20 * errev(nn, qpr, qpi, ms, mp, are, mre)) {
					// Polynomial value is smaller in value than a bound onthe error
					// in evaluationg P, terminate the ietartion
					*conv = 1;
					*zr = sr;
					*zi = si;
					return;
				}
				if (i != 1) {
					if (!(b || mp < omp || relstp >= 0.05)) {
						// Iteration has stalled. Probably a cluster of zeros. Do 5 fixed
						// shift steps into the cluster to force one zero to dominate
						tp = relstp;
						b = 1;
						if (relstp < eta) tp = eta;
						r1 = sqrt(tp);
						r2 = sr * (1 + r1) - si * r1;
						si = sr * r1 + si * (1 + r1);
						sr = r2;
						polyev(nn, sr, si, pr, pi, qpr, qpi, &pvr, &pvi);
						for (j = 1; j <= 5; j++) {
							calct(&bol);
							nexth(bol);
						}
						omp = infin;
						goto _20;
					}

					// Exit if polynomial value increase significantly
					if (mp * 0.1 > omp) return;
				}

				omp = mp;

				// Calculate next iterate
			_20:  calct(&bol);
				nexth(bol);
				calct(&bol);
				if (!bol) {
					relstp = cmod(tr, ti) / cmod(sr, si);
					sr += tr;
					si += ti;
				}
			}
		}

		// COMPUTES  T = -P(S)/H(S).
		// BOOL   - LOGICAL, SET TRUE IF H(S) IS ESSENTIALLY ZERO.
		void calct(int* bol) {
			int n;
			double hvr, hvi;

			n = nn;

			// evaluate h(s)
			polyev(n - 1, sr, si, hr, hi, qhr, qhi, &hvr, &hvi);
			*bol = cmod(hvr, hvi) <= are * 10 * cmod(hr[n - 1], hi[n - 1]) ? 1 : 0;
			if (!*bol) {
				cdivid(-pvr, -pvi, hvr, hvi, &tr, &ti);
				return;
			}

			tr = 0;
			ti = 0;
		}

		// CALCULATES THE NEXT SHIFTED H POLYNOMIAL.
		// BOOL   -  LOGICAL, IF .TRUE. H(S) IS ESSENTIALLY ZERO
		//
		void nexth(const int bol) {
			int j, n;
			double t1, t2;

			n = nn;
			if (!bol) {
				for (j = 1; j < n; j++) {
					t1 = qhr[j - 1];
					t2 = qhi[j - 1];
					hr[j] = tr * t1 - ti * t2 + qpr[j];
					hi[j] = tr * t2 + ti * t1 + qpi[j];
				}
				hr[0] = qpr[0];
				hi[0] = qpi[0];
				return;
			}

			// If h[s] is zero replace H with qh
			for (j = 1; j < n; j++) {
				hr[j] = qhr[j - 1];
				hi[j] = qhi[j - 1];
			}
			hr[0] = 0;
			hi[0] = 0;
		}

		// EVALUATES A POLYNOMIAL  P  AT  S  BY THE HORNER RECURRENCE
		// PLACING THE PARTIAL SUMS IN Q AND THE COMPUTED VALUE IN PV.
		//
		void polyev(const int nn, const double sr, const double si, const double pr[], const double pi[], double qr[], double qi[], double* pvr, double* pvi) {
			int i;
			double t;

			qr[0] = pr[0];
			qi[0] = pi[0];
			*pvr = qr[0];
			*pvi = qi[0];

			for (i = 1; i <= nn; i++) {
				t = (*pvr) * sr - (*pvi) * si + pr[i];
				*pvi = (*pvr) * si + (*pvi) * sr + pi[i];
				*pvr = t;
				qr[i] = *pvr;
				qi[i] = *pvi;
			}
		}

		// BOUNDS THE ERROR IN EVALUATING THE POLYNOMIAL BY THE HORNER RECURRENCE.
		// QR,QI - THE PARTIAL SUMS
		// MS    -MODULUS OF THE POINT
		// MP    -MODULUS OF POLYNOMIAL VALUE
		// ARE, MRE -ERROR BOUNDS ON COMPLEX ADDITION AND MULTIPLICATION
		//
		double errev(const int nn, const double qr[], const double qi[], const double ms, const double mp, const double are, const double mre) {
			int i;
			double e;

			e = cmod(qr[0], qi[0]) * mre / (are + mre);
			for (i = 0; i <= nn; i++)
				e = e * ms + cmod(qr[i], qi[i]);

			return e * (are + mre) - mp * mre;
		}

		// CAUCHY COMPUTES A LOWER BOUND ON THE MODULI OF THE ZEROS OF A
		// POLYNOMIAL - PT IS THE MODULUS OF THE COEFFICIENTS.
		//
		void cauchy(const int nn, double pt[], double q[], double* fn_val) {
			int i, n;
			double x, xm, f, dx, df;

			pt[nn] = -pt[nn];

			// Compute upper estimate bound
			n = nn;
			x = exp(log(-pt[nn]) - log(pt[0])) / n;
			if (pt[n - 1] != 0) {
				// Newton step at the origin is better, use it
				xm = -pt[nn] / pt[n - 1];
				if (xm < x) x = xm;
			}

			// Chop the interval (0,x) until f < 0
			while (1) {
				xm = x * 0.1;
				f = pt[0];
				for (i = 1; i <= nn; i++)
					f = f * xm + pt[i];
				if (f <= 0)
					break;
				x = xm;
			}
			dx = x;

			// Do Newton iteration until x converges to two decimal places
			while (fabs(dx / x) > 0.005) {
				q[0] = pt[0];
				for (i = 1; i <= nn; i++)
					q[i] = q[i - 1] * x + pt[i];
				f = q[nn];
				df = q[0];
				for (i = 1; i < n; i++)
					df = df * x + q[i];
				dx = f / df;
				x -= dx;
			}

			*fn_val = x;
		}

		// RETURNS A SCALE FACTOR TO MULTIPLY THE COEFFICIENTS OF THE POLYNOMIAL.
		// THE SCALING IS DONE TO AVOID OVERFLOW AND TO AVOID UNDETECTED UNDERFLOW
		// INTERFERING WITH THE CONVERGENCE CRITERION.  THE FACTOR IS A POWER OF THE
		// BASE.
		// PT - MODULUS OF COEFFICIENTS OF P
		// ETA, INFIN, SMALNO, BASE - CONSTANTS DESCRIBING THE FLOATING POINT ARITHMETIC.
		//
		double scale(const int nn, const double pt[], const double eta, const double infin, const double smalno, const double base) {
			int i, l;
			double hi, lo, max, min, x, sc;
			double fn_val;

			// Find largest and smallest moduli of coefficients
			hi = sqrt(infin);
			lo = smalno / eta;
			max = 0;
			min = infin;

			for (i = 0; i <= nn; i++) {
				x = pt[i];
				if (x > max) max = x;
				if (x != 0 && x < min) min = x;
			}

			// Scale only if there are very large or very small components
			fn_val = 1;
			if (min >= lo && max <= hi) return fn_val;
			x = lo / min;
			if (x <= 1)
				sc = 1 / (sqrt(max) * sqrt(min));
			else {
				sc = x;
				if (infin / sc > max) sc = 1;
			}
			l = (int)(log(sc) / log(base) + 0.5);
			fn_val = pow(base, l);
			return fn_val;
		}

		// COMPLEX DIVISION C = A/B, AVOIDING OVERFLOW.
		//
		void cdivid(const double ar, const double ai, const double br, const double bi, double* cr, double* ci) {
			double r, d, t, infin;

			if (br == 0 && bi == 0) {
				// Division by zero, c = infinity
				mcon(&t, &infin, &t, &t);
				*cr = infin;
				*ci = infin;
				return;
			}

			if (fabs(br) < fabs(bi)) {
				r = br / bi;
				d = bi + r * br;
				*cr = (ar * r + ai) / d;
				*ci = (ai * r - ar) / d;
				return;
			}

			r = bi / br;
			d = br + r * bi;
			*cr = (ar + ai * r) / d;
			*ci = (ai - ar * r) / d;
		}

		// MODULUS OF A COMPLEX NUMBER AVOIDING OVERFLOW.
		//
		double cmod(const double r, const double i) {
			double ar, ai;

			ar = fabs(r);
			ai = fabs(i);
			if (ar < ai)
				return ai * sqrt(1.0 + pow((ar / ai), 2.0));

			if (ar > ai)
				return ar * sqrt(1.0 + pow((ai / ar), 2.0));

			return ar * sqrt(2.0);
		}

		// MCON PROVIDES MACHINE CONSTANTS USED IN VARIOUS PARTS OF THE PROGRAM.
		// THE USER MAY EITHER SET THEM DIRECTLY OR USE THE STATEMENTS BELOW TO
		// COMPUTE THEM. THE MEANING OF THE FOUR CONSTANTS ARE -
		// ETA       THE MAXIMUM RELATIVE REPRESENTATION ERROR WHICH CAN BE DESCRIBED
		//           AS THE SMALLEST POSITIVE FLOATING-POINT NUMBER SUCH THAT
		//           1.0_dp + ETA &gt; 1.0.
		// INFINY    THE LARGEST FLOATING-POINT NUMBER
		// SMALNO    THE SMALLEST POSITIVE FLOATING-POINT NUMBER
		// BASE      THE BASE OF THE FLOATING-POINT NUMBER SYSTEM USED
		//
		void mcon(double* eta, double* infiny, double* smalno, double* base) {
			*base = 2;
			*eta = DBL_EPSILON;
			*infiny = DBL_MAX;
			*smalno = DBL_MIN;
		}

	};


	void ComputePoly(double afA[3], double afB[3], double afD[3], double fC, double rkPoly[7])const {
		double afBPad[3] =
		{
			afB[0] + afA[0] * afD[0],
				afB[1] + afA[1] * afD[1],
				afB[2] + afA[2] * afD[2]
		};

		double afBSqr[3] =
		{
			afB[0] * afB[0],
				afB[1] * afB[1],
				afB[2] * afB[2]
		};

		double afDSum[4] =
		{
			afD[0] + afD[1],
				afD[0] + afD[2],
				afD[1] + afD[2],
				afD[0] + afD[1] + afD[2]
		};

		double afDPrd[4] =
		{
			afD[0] * afD[1],
				afD[0] * afD[2],
				afD[1] * afD[2],
				afD[0] * afD[1] * afD[2]
		};

		double afDSqr[3] =
		{
			afD[0] * afD[0],
				afD[1] * afD[1],
				afD[2] * afD[2]
		};

		rkPoly[0] = afA[0] * afBPad[0] + afA[1] * afBPad[1] + afA[2] * afBPad[2] + fC;

		rkPoly[1] = -afBSqr[0] - afBSqr[1] - afBSqr[2] + ((double)4.0) * (
			afA[0] * afBPad[0] * afDSum[2] + afA[1] * afBPad[1] * afDSum[1] +
			afA[2] * afBPad[2] * afDSum[0] + fC * afDSum[3]);

		rkPoly[2] = -afBSqr[0] * (afD[0] + ((double)4.0) * afDSum[2])
			- afBSqr[1] * (afD[1] + ((double)4.0) * afDSum[1]) - afBSqr[2] * (afD[2] +
				((double)4.0) * afDSum[0]) + ((double)4.0) * (afA[0] * afBPad[0] * (
					afDSum[2] * afDSum[2] + 2 * afDPrd[2]) +
					afA[1] * afBPad[1] * (afDSum[1] * afDSum[1] + 2 * afDPrd[1]) +
					afA[2] * afBPad[2] * (afDSum[0] * afDSum[0] + 2 * afDPrd[0]) +
					fC * (afDSqr[0] + afDSqr[1] + afDSqr[2] + ((double)4.0) * (
						afDPrd[0] + afDPrd[1] + afDPrd[2])));

		rkPoly[3] =
			-afBSqr[0] * (afD[1] * afDSum[0] + afD[2] * afDSum[1] + ((double)4.0) * afDPrd[2])
			- afBSqr[1] * (afD[0] * afDSum[0] + afD[2] * afDSum[2] + ((double)4.0) * afDPrd[1])
			- afBSqr[2] * (afD[0] * afDSum[1] + afD[1] * afDSum[2] + ((double)4.0) * afDPrd[0])
			+ ((double)4.0) * (afA[0] * afDPrd[2] * afBPad[0] * afDSum[2] +
				afA[1] * afDPrd[1] * afBPad[1] * afDSum[1] +
				afA[2] * afDPrd[0] * afBPad[2] * afDSum[0] +
				fC * (afDSqr[0] * afDSum[2] + afDSqr[1] * afDSum[1] + afDSqr[2] * afDSum[0] +
					((double)4.0) * afDPrd[3]));

		rkPoly[3] *= (double)4.0;

		rkPoly[4] =
			-afBSqr[0] * (afD[0] * (afDSqr[1] + afDSqr[2])
				+ ((double)4.0) * afDPrd[2] * afDSum[3])
			- afBSqr[1] * (afD[1] * (afDSqr[0] + afDSqr[2])
				+ ((double)4.0) * afDPrd[1] * afDSum[3])
			- afBSqr[2] * (afD[2] * (afDSqr[0] + afDSqr[1])
				+ ((double)4.0) * afDPrd[0] * afDSum[3])
			+ ((double)4.0) * (afA[0] * afDSqr[1] * afDSqr[2] * afBPad[0] +
				afA[1] * afDSqr[0] * afDSqr[2] * afBPad[1] +
				afA[2] * afDSqr[0] * afDSqr[1] * afBPad[2] +
				fC * (afDSqr[0] * afDSqr[1] + afDSqr[0] * afDSqr[2] + afDSqr[1] * afDSqr[2]
					+ ((double)4.0) * afDPrd[3] * afDSum[3]));

		rkPoly[4] *= (double)4.0;

		rkPoly[5] = ((double)16.0) * (afDPrd[0] + afDPrd[1] + afDPrd[2]) * (
			-afBSqr[0] * afDPrd[2] - afBSqr[1] * afDPrd[1] - afBSqr[2] * afDPrd[0] +
			((double)4.0) * fC * afDPrd[3]);

		rkPoly[6] = ((double)16.0) * afDPrd[3] * (-afBSqr[0] * afDPrd[2]
			- afBSqr[1] * afDPrd[1] - afBSqr[2] * afDPrd[0]
			+ ((double)4.0) * fC * afDPrd[3]);
	}


	void ComputeProject(vec3 m_rkVector,vec3& res_pt,double& res_dis)const {
		Eigen::Matrix3d kES(3, 3);

		kES(0,0) = Coeff[4];
		kES(0,1) = 0.5 * Coeff[5];
		kES(0,2) = 0.5 * Coeff[6];
		kES(1,0) = kES(0,1);
		kES(1,1) = Coeff[7];
		kES(1,2) = 0.5 * Coeff[8];
		kES(2,0) = kES(0,2);
		kES(2,1) = kES(1,2);
		kES(2,2) = Coeff[9];

		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> jama_keigen(kES);
		auto ei = jama_keigen.eigenvalues();
		auto eigenvec = jama_keigen.eigenvectors();

		vec3 akEVec[3];
		int i;
		for (i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				akEVec[i][j] = eigenvec(j,i);
			}
		}

		double afA[3], afB[3], afD[3], fC = Coeff[0];

		for (i = 0; i < 3; i++) {
			afA[i] = akEVec[i].x * m_rkVector.x + akEVec[i].y * m_rkVector.y
				+ akEVec[i].z * m_rkVector.z;
			afB[i] = akEVec[i].x * Coeff[1] +
				akEVec[i].y * Coeff[2] + akEVec[i].z * Coeff[3];
			afD[i] = ei[i];
		}

		//prepare polynomial
		double kPoly[7];

		ComputePoly(afA, afB, afD, fC, kPoly);

		int first = 6;
		for (int i = 6; i >= 0; i--) {
			//if (abs(kPoly[i])<1e-12) {
			if (kPoly[i] == 0) {
				first--;
			} else
				break;
		}

		std::vector<double> mpoly;
		for (int i = 0; i <= first; i++) {
			mpoly.push_back(kPoly[i]);

		}

		CG_PolynomialRoot r(mpoly);
		std::vector<double> afRoot = r.get_realroot();

		int iCount = (int)afRoot.size();
		if (iCount > 0) {
			double fMinDistSqr = DBL_MAX;
			int iMinIndex = -1;
			double afV[3], fDenom;
			for (int iIndex = 0; iIndex < iCount; iIndex++) {
				double rr = afRoot[iIndex];
				// compute closest point for this root
				for (int i = 0; i < 3; i++) {
					fDenom = 1.0 + 2.0 * afRoot[iIndex] * afD[i];
					if (abs(fDenom) == 0) {
						vec3 nV = m_rkVector + vec3(1e-8, 1e-8, 1e-8);
						vec3 res;
						double dis;
						ComputeProject(nV, res_pt, res_dis);
						return;
					}
					afV[i] = (afA[i] - afRoot[iIndex] * afB[i]) / fDenom;
				}
				vec3 m_kClosestPoint1;
				m_kClosestPoint1.x = akEVec[0][0] * afV[0] + akEVec[1][0] * afV[1]
					+ akEVec[2][0] * afV[2];
				m_kClosestPoint1.y = akEVec[0][1] * afV[0] + akEVec[1][1] * afV[1]
					+ akEVec[2][1] * afV[2];
				m_kClosestPoint1.z = akEVec[0][2] * afV[0] + akEVec[1][2] * afV[1]
					+ akEVec[2][2] * afV[2];

				// compute squared distance from point to quadric
				vec3 kDiff = m_kClosestPoint1 - m_rkVector;
				double fDistSqr = kDiff.length2();
				if (fDistSqr < fMinDistSqr) {
					res_pt = m_kClosestPoint1;
					fMinDistSqr = fDistSqr;
					iMinIndex = iIndex;
				}
			}
			res_dis = fMinDistSqr;
			return;
		} else {
			// should not happen
			assert(false);
			res_dis = -1;
			return;
		}
	}

	virtual bool fit(const vector<Point>& pts)override {

		double x, y, z, xy, xz, yz, xx, yy, zz, xxx, yyy, zzz, xxy, xxz, xyy, yyz, xzz, yzz, xyz;
		double totalweight = 0;
		Eigen::MatrixXd Mat = Eigen::MatrixXd::Zero(10, 10);

		for (int i = 0; i < pts.size(); i++) {
			double Weight_i = 1;
			x = pts[i][0];
			y = pts[i][1];
			z = pts[i][2];
			xx = x * x; xy = x * y; xz = x * z;
			yy = y * y; yz = y * z;
			zz = z * z;
			xxx = xx * x; yyy = yy * y; zzz = zz * z;
			xxy = xx * y; xxz = xx * z; yyz = yy * z;
			xzz = x * zz; yzz = y * zz; xyy = x * yy;
			xyz = x * y * z;

			totalweight += Weight_i;

			Mat(1, 0) += x * Weight_i;
			Mat(2, 0) += y * Weight_i;
			Mat(2, 1) += xy * Weight_i;
			Mat(3, 0) += z * Weight_i;
			Mat(3, 1) += xz * Weight_i;
			Mat(3, 2) += yz * Weight_i;
			Mat(4, 0) += xx * Weight_i;
			Mat(4, 1) += xxx * Weight_i;
			Mat(4, 2) += xxy * Weight_i;
			Mat(4, 3) += xxz * Weight_i;
			Mat(5, 2) += xyy * Weight_i;
			Mat(5, 3) += xyz * Weight_i;
			Mat(5, 4) += xx * xy * Weight_i;
			Mat(6, 3) += xzz * Weight_i;
			Mat(6, 4) += xx * xz * Weight_i;
			Mat(6, 5) += xx * yz * Weight_i;
			Mat(7, 0) += yy * Weight_i;
			Mat(7, 2) += yyy * Weight_i;
			Mat(7, 3) += yyz * Weight_i;
			Mat(7, 4) += xx * yy * Weight_i;
			Mat(7, 5) += xy * yy * Weight_i;
			Mat(7, 6) += xy * yz * Weight_i;
			Mat(8, 3) += yzz * Weight_i;
			Mat(8, 6) += xy * zz * Weight_i;
			Mat(8, 7) += yy * yz * Weight_i;
			Mat(9, 0) += zz * Weight_i;
			Mat(9, 3) += zzz * Weight_i;
			Mat(9, 4) += xx * zz * Weight_i;
			Mat(9, 6) += xz * zz * Weight_i;
			Mat(9, 7) += yy * zz * Weight_i;
			Mat(9, 8) += yz * zz * Weight_i;

			Mat(4, 4) += xx * xx * Weight_i;
			Mat(5, 5) += xy * xy * Weight_i;
			Mat(6, 6) += xz * xz * Weight_i;
			Mat(7, 7) += yy * yy * Weight_i;
			Mat(8, 8) += yz * yz * Weight_i;
			Mat(9, 9) += zz * zz * Weight_i;
		}

		Mat(0, 0) = totalweight; //(Real)iQuantity;
		Mat(1, 1) = Mat(4, 0); // xx
		Mat(2, 2) = Mat(7, 0); // yy
		Mat(3, 3) = Mat(9, 0); // zz
		Mat(5, 0) = Mat(2, 1); // xy
		Mat(5, 1) = Mat(4, 2); // xxy
		Mat(6, 0) = Mat(3, 1); // xz
		Mat(6, 1) = Mat(4, 3); //xxz
		Mat(6, 2) = Mat(5, 3); //xyz
		Mat(7, 1) = Mat(5, 2); //xyy
		Mat(8, 0) = Mat(3, 2); //yz
		Mat(8, 1) = Mat(5, 3); //xyz
		Mat(8, 2) = Mat(7, 3); //yyz
		Mat(8, 4) = Mat(6, 5); //xx*yz
		Mat(8, 5) = Mat(7, 6); //xy*yz
		Mat(9, 1) = Mat(6, 3); //xzz
		Mat(9, 2) = Mat(8, 3); //yzz
		Mat(9, 5) = Mat(8, 6); //xy*zz

		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < i; j++) {
				Mat(j, i) = Mat(i, j);
			}
		}
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> egs(Mat);
		auto e = egs.eigenvalues();
		auto eigenvector = egs.eigenvectors();

		int bestid = 0;
		double minvalue = 1e20;
		for (int i = 0; i < 10; i++) {
			if (e[i] < minvalue) {
				minvalue = e[i];
				bestid = i;
			}
		}
		for (int k = 0; k < 10; k++) {
			Coeff[k] = eigenvector(k, bestid);
		}

		return true;
	}

	virtual float signedDistance(const vec3& p) const override {
		double d;
		vec3 pn;
		ComputeProject(p, pn, d);
		return d;
	}

	virtual vec3 project(const vec3& p) const override {
		double d;
		vec3 pn;
		ComputeProject(p, pn, d);

		return pn;
	}

	virtual vec3 normal(const vec3& p) const override {
		return vec3();
	}


	virtual SurfaceParameters getParameters() const {
		SurfaceParameters p;
		p.coeff = Coeff;
		return p;
	}

	virtual void setParameters(SurfaceParameters new_parameters) {
		Coeff= new_parameters.coeff;
	}
private:
	Vec<10, double> Coeff;
};

void fit_quadratic(int iQuantity, const std::vector<vec3> &akPoint, const std::vector<double>& Weight, double Coeff[10]) {
	
}

float estimateError(SurfacePrimitive* s, const vector<Point>& points)
{
	float error = 0;
	for (auto& i : points)
	{
		auto d = s->distance(i.p);
		error += fabs(d);
	}
	error = error /points.size();
	return error;
}

SurfacePrimitive* fit_surface_raw(const vector<Point>& points,SurfaceType type)
{
	vector<SurfacePrimitive*> candidates;
	vector<float> cost;

	if (type == SurfaceType::UNKNOWN)
	{
		candidates.push_back(new Plane());
		candidates.push_back(new Sphere());
		candidates.push_back(new Cylinder());
		candidates.push_back(new Cone());
		//candidates.push_back(new Torus());
	}
	else
	{
		if(type==SurfaceType::PLANE)
			candidates.push_back(new Plane());
		else if(type==SurfaceType::CYLINDER)
			candidates.push_back(new Cylinder());
		else if (type == SurfaceType::SPHERE)
			candidates.push_back(new Sphere());
		else if (type == SurfaceType::CONE)
			candidates.push_back(new Cone());
		else if (type == SurfaceType::TORUS)
			candidates.push_back(new Torus());
		else if (type == SurfaceType::ELLIPSOID)
			candidates.push_back(new Ellipsoid());
		else if (type == SurfaceType::GENERALQUADRIC)
			candidates.push_back(new GeneralQuadric());
	}
	for (auto p : candidates)
	{
		p->fit(points);
		cost.push_back(estimateError(p, points));
//		cout << "cost "<< (int)p->getType() <<"="<<cost.back() << endl;
	}

	int best = -1;
	for (int i = 0; i < candidates.size(); i++)
	{
		if (candidates[i]->getType() != SurfaceType::UNKNOWN)// fitting succeeded
		{
			if (best == -1 || cost[i] < cost[best])
			{
				best = i;
				if (cost[i] < 5e-5)
					break;
			}
		}
	}
	for (int i = 0; i < candidates.size(); i++)
	{
		if (best!=i)
		{
			delete candidates[i];
		}
	}

	return best != -1 ? candidates[best] : nullptr;
}

SurfacePrimitive* fit_surface(const vector<Point>& points, SurfaceType type) {
	float rate = 0.8;
	auto p = fit_surface_raw(points, type);
	if (p == nullptr)
		return nullptr;
	type = p->getType();
	int times = 20;
	while (times--) {
		vector<pair<float,int>> error(points.size());
		for (int i=0;i<points.size();i++) {
			auto d = p->distance(points[i].p);
			error[i] = make_pair(d,i);
		}
		sort(error.begin(), error.end());
		int maj = rate * points.size();
		vector<Point> tmp_points;
		for (int i = 0; i < maj; i++) {
			tmp_points.push_back(points[i]);
		}
		p->fit(tmp_points);
	}
	return p;
}

SurfacePrimitive* construct_bytype(SurfaceType type, SurfaceParameters p) {
	if (type == SurfaceType::PLANE) {
		auto x= new Plane();
		x->setType(SurfaceType::PLANE);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::SPHERE) {
		auto x = new Sphere();
		x->setType(SurfaceType::SPHERE);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::CYLINDER) {
		auto x = new Cylinder();
		x->setType(SurfaceType::CYLINDER);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::CONE) {
		auto x = new Cone();
		x->setType(SurfaceType::CONE);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::TORUS) {
		auto x = new Torus();
		x->setType(SurfaceType::TORUS);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::ELLIPSOID) {
		auto x = new Ellipsoid();
		x->setType(SurfaceType::ELLIPSOID);
		x->setParameters(p);
		return x;
	} else if (type == SurfaceType::GENERALQUADRIC) {
		auto x = new GeneralQuadric();
		x->setType(SurfaceType::GENERALQUADRIC);
		x->setParameters(p);
		return x;
	} else {
		return new Plane();
	}
}


SurfacePrimitive* clone_surface_primitive(SurfacePrimitive* primitive) {
    if(primitive == nullptr) {
        return nullptr;
    }
    SurfaceType type = primitive->getType();
    SurfaceParameters para = primitive->getParameters();

    easy3d::vec3 pos = para.pos, dir = para.dir;
    float r1 = para.r1, r2 = para.r2;
    SurfacePrimitive* clone = construct_bytype(type, para);

    if(clone) {
        clone->setColor(primitive->getColor());
    }

    return clone;
}


int plane_orient(const easy3d::vec3& p,
                 const easy3d::vec3& n,
                 const easy3d::vec3& q) {
    float d = -easy3d::dot(p, n);

    double v = easy3d::dot(n, q) + d;
    if(abs(v) < 1e-15) {
        return 0;
    }

    return (v > 0.0 ? 1 : -1);
}

bool line_plane_intersection(const easy3d::vec3& p, const easy3d::vec3& n,
                             const easy3d::vec3& s, const easy3d::vec3& t, easy3d::vec3& q) {

    float d = -easy3d::dot(p, n);

    easy3d::vec3 dir = (t - s).normalize();
    double c = easy3d::dot(dir, n);
    if(std::fabs(c) < 1e-15) {
        return false;
    }

    double tmp = -(n[0] * s.x + n[1] * s.y + n[2] * s.z + d)
                 /  (n[0] * dir.x + n[1] * dir.y + n[2] * dir.z);

    q = s + dir * tmp;
    return true;
}

void compute_bases(const easy3d::vec3& n,
                   easy3d::vec3& base1, easy3d::vec3& base2) {
    if(n[0] == 0) {
        base1 = easy3d::vec3(1, 0, 0);
    } else if(n[1] == 0) {
        base1 = easy3d::vec3(0, 1, 0);
    } else if(n[2] == 0) {
        base1 = easy3d::vec3(0, 0, 1);
    } else {
        easy3d::vec3 b(-n[1], n[0], 0);
        base1 = easy3d::normalize(b);
    }

    base2 = easy3d::cross(n, base1);
    base2.normalize();
}

easy3d::vec2 plane_to_2d(const easy3d::vec3& p,
                         const easy3d::vec3& n,
                         easy3d::vec3& q) {
    easy3d::vec3 vec = q - p;
    vec3 base1, base2;
    compute_bases(n, base1, base2);
    double x = easy3d::dot(vec, base1);
    double y = easy3d::dot(vec, base2);
    return easy3d::vec2(x, y);
}



}