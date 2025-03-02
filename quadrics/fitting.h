#pragma once
#include <vector>
#include <cmath>
#include <map>
#include <easy3d/core/types.h>
#include <GfxTL/Mean.h>

namespace PrimFit {
	struct Point {
		easy3d::vec3 p;
		easy3d::vec3 n;

		float& operator[](unsigned int i)
		{
			return p[i];
		}
		const float operator[](unsigned int i) const
		{
			return p[i];
		}
	};

	enum class SurfaceType
	{
		UNKNOWN,
		PLANE,
		SPHERE,
		CYLINDER,
		CONE,
		TORUS,
		ELLIPSOID,
		GENERALQUADRIC
	};

	inline SurfaceType to_type(std::string typestring) {
		if (typestring == "PLANE") {
			return SurfaceType::PLANE;
		} else if (typestring == "SPHERE") {
			return SurfaceType::SPHERE;
		} else if (typestring == "CYLINDER") {
			return SurfaceType::CYLINDER;
		} else if (typestring == "CONE") {
			return SurfaceType::CONE;
		} else if (typestring == "TORUS") {
			return SurfaceType::TORUS;
		} else if (typestring == "ELLIPSOID") {
			return SurfaceType::ELLIPSOID;
		} else if (typestring == "GENERALQUADRIC") {
			return SurfaceType::GENERALQUADRIC;
		} else {
			return SurfaceType::UNKNOWN;
		}
	}

	inline std::string to_str(SurfaceType type) {
		if (type == SurfaceType::PLANE) {
			return "PLANE";
		} else if (type == SurfaceType::SPHERE) {
			return "SPHERE";
		} else if (type == SurfaceType::CYLINDER) {
			return "CYLINDER";
		} else if (type == SurfaceType::CONE) {
			return "CONE";
		} else if (type == SurfaceType::TORUS) {
			return "TORUS";
		} else if (type == SurfaceType::ELLIPSOID) {
			return "ELLIPSOID";
		} else if (type == SurfaceType::GENERALQUADRIC) {
			return "GENERALQUADRIC";
		} else {
			return "UNKNOWN";
		}
	}

    inline int to_int_label(SurfaceType type) {
        if (type == SurfaceType::PLANE) {
            return 0;
        } else if (type == SurfaceType::SPHERE) {
            return 1;
        } else if (type == SurfaceType::CYLINDER) {
            return 2;
        } else if (type == SurfaceType::CONE) {
            return 3;
        } else if (type == SurfaceType::TORUS) {
            return 4;
        } else if (type == SurfaceType::ELLIPSOID) {
            return 5;
        } else if (type == SurfaceType::GENERALQUADRIC) {
            return 6;
        } else {
            return -1;
        }
    }

    inline SurfaceType to_type(int type) {
        if (type == 0) {
            return SurfaceType::PLANE;
        } else if (type == 1) {
            return SurfaceType::SPHERE;
        } else if (type == 2) {
            return SurfaceType::CYLINDER;
        } else if (type == 3) {
            return SurfaceType::CONE;
        } else if (type == 4) {
            return SurfaceType::TORUS;
        } else if (type == 5) {
            return SurfaceType::ELLIPSOID;
        } else if (type == 6) {
            return SurfaceType::GENERALQUADRIC;
        } else {
            return SurfaceType::UNKNOWN;
        }
    }

	struct SurfaceParameters
	{
		SurfaceParameters():pos(0),dir(0),r1(0),r2(0), coeff(0){}
		SurfaceParameters(easy3d::vec3 pos,	easy3d::vec3 dir, float r1,	float r2) :pos(pos), dir(dir), r1(r1), r2(r2) {}
		SurfaceParameters(easy3d::Vec<10, double> &p) :coeff(p),r1(0),r2(0) {
		}
		easy3d::vec3 pos;
		easy3d::vec3 dir;
		float r1;
		float r2;
		easy3d::Vec<10, double> coeff;
	};
    class SurfacePrimitive
    {
    public:
        SurfacePrimitive(SurfaceType type = SurfaceType::UNKNOWN) :type(type), color(0,0,0) {}

        virtual bool fit(const std::vector<Point>& points) = 0;

        virtual easy3d::vec3 project(const easy3d::vec3& p)const = 0;

        virtual easy3d::vec3 normal(const easy3d::vec3& p)const = 0;

        virtual float signedDistance(const easy3d::vec3& p) const = 0;

        virtual SurfaceParameters getParameters() const = 0;

        virtual void setParameters(SurfaceParameters new_parameters) = 0;


        float distance(const easy3d::vec3& p) const
        {
            return abs(signedDistance(p));
        }

        SurfaceType getType() const
        {
            return type;
        }

        void setType(SurfaceType type)
        {
            this->type = type;
        }

        const easy3d::vec3 getColor()
        {
            return color;
        }
        void setColor(const easy3d::vec3& c)
        {
            this->color = c;
        }

    protected:
        SurfaceType type;
        easy3d::vec3 color;
    };

    SurfacePrimitive* fit_surface(const std::vector<Point>& points, SurfaceType type=SurfaceType::UNKNOWN	);

    SurfacePrimitive* fit_surface_raw(const std::vector<Point>& points, SurfaceType type = SurfaceType::UNKNOWN);

    SurfacePrimitive* construct_bytype(SurfaceType type, SurfaceParameters p);

    float estimateError(SurfacePrimitive* s, const std::vector<Point>& points);

    int plane_orient(const easy3d::vec3& p, const easy3d::vec3& n, const easy3d::vec3& q);

    bool line_plane_intersection(const easy3d::vec3& p, const easy3d::vec3& n,
                                 const easy3d::vec3& s, const easy3d::vec3& t, easy3d::vec3& q);

    void compute_bases(const easy3d::vec3& n, easy3d::vec3& base1, easy3d::vec3& base2);

    easy3d::vec2 plane_to_2d(const easy3d::vec3& p, const easy3d::vec3& n, easy3d::vec3& q);

    SurfacePrimitive* clone_surface_primitive(SurfacePrimitive* primitive);

}