#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>
#include <memory>
#include <ctime>

#define M_PI 3.141592653589793
#define MAX_RAY_DEPTH 5
const float kInfinity = std::numeric_limits<float>::max();
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0, 1);
//std::ofstream ofs1;

template<typename T>
class Vec3 {
public:
	T x, y, z;
	Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}

	T length2() const {
		return x * x + y * y + z * z;
	}
	T length() const {
		return sqrt(length2());
	}

	Vec3& normalize() {
		T nor2 = length2();
		if (nor2 > 0) {
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
		}
		return *this;
	}

	Vec3 crossProduct(const Vec3<T> &v) const {
		return Vec3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	T dotProduct(const Vec3<T> &v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	Vec3<T> operator * (const T &f) const { return Vec3<T>(x * f, y * f, z * f); }
	Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x * v.x, y * v.y, z * v.z); }
	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
	Vec3<T> operator / (const Vec3<T> &v) const { return Vec3<T>(x / v.x, y / v.y, z / v.z); }
    Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
    Vec3<T>& operator *= (const Vec3<T> &v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }
	friend std::ostream & operator << (std::ostream &os, const Vec3<T> &v) {
		os << "[" << v.x << " " << v.y << " " << v.z << "]";
		return os;
	}
};

typedef Vec3<float> Vec3f;

struct Options {
	uint32_t width;
	uint32_t height;
	float fov;	// field of view
	float imageAspectRatio;
	uint8_t maxDepth;
	Vec3f backgroundColor;
	float bias;
};

inline
float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }


inline
float mix(const float &a, const float &b, const float &mixVal) {
	return b * mixVal + a * (1 - mixVal);
}

inline
float deg2rad(const float &deg) {
	return deg * M_PI / 180;
}

class Object {
public:
	Vec3f surfaceColor, emissionColor;
	float transparency, reflection;
	float ior; // Index of Refraction
	Object(
		const Vec3f &sc,
		const Vec3f &ec = 0,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) : surfaceColor(sc), emissionColor(ec), transparency(transp),
		reflection(refl), ior(ior) {}

	virtual ~Object() {}
	virtual bool intersect(const Vec3f &, const Vec3f &, float &) const = 0;
	virtual void getSurfaceData(const Vec3f &, Vec3f &) const = 0;
};

class Sphere : public Object {
public:
	Vec3f center;
	float radius, radius2;
	Sphere(
		const Vec3f &c,
		const float &r,
		const Vec3f &sc,
		const Vec3f &ec,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) :
		Object(sc, ec, refl, transp, ior), center(c), radius(r), radius2(r * r) {
		// empty
	}

	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t) const {
		float t0, t1;
		Vec3f l = center - rayorig;
		float tca = l.dotProduct(raydir);
		if (tca < 0) return false;
		float d2 = l.dotProduct(l) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;
		if (t0 > t1) std::swap(t0, t1);
		if (t0 < 0) {
			t0 = t1;
			if (t0 < 0) return false;
		}
		t = t0;
		return true;
	}

	void getSurfaceData(const Vec3f &p, Vec3f &n) const {
		n = p - center;
		n.normalize();
	}
};

class AABBox : public Object {
public:
	Vec3f bounds[2];
	AABBox(
		const Vec3f &b0,
		const Vec3f &b1,
		const Vec3f &sc,
		const Vec3f &ec,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) : Object(sc, ec, refl, transp, ior) {
		bounds[0] = b0;
		bounds[1] = b1;

	}

	void getSurfaceData(const Vec3f &p, Vec3f &n) const {

		if (abs(p.z - bounds[1].z) < 0.01
				&& p.x > bounds[0].x && p.x < bounds[1].x
				&& p.y > bounds[0].y && p.y < bounds[1].y) {
			n = Vec3f(0, 0, 1);
		}
		if (abs(p.z - bounds[0].z) < 0.01
				&& p.x > bounds[0].x && p.x < bounds[1].x
				&& p.y > bounds[0].y && p.y < bounds[1].y) {
			n = Vec3f(0, 0, -1);
		}
		if (abs(p.x - bounds[1].x) < 0.01
				&& p.z > bounds[0].z && p.z < bounds[1].z
				&& p.y > bounds[0].y && p.y < bounds[1].y) {
			n = Vec3f(1, 0, 0);
		}
		if (abs(p.x - bounds[0].x) < 0.01
				&& p.z > bounds[0].z && p.z < bounds[1].z
				&& p.y > bounds[0].y && p.y < bounds[1].y) {
			n = Vec3f(-1, 0, 0);
		}
		if (abs(p.y - bounds[1].y) < 0.01
				&& p.z > bounds[0].z && p.z < bounds[1].z
				&& p.x > bounds[0].x && p.x < bounds[1].x) {
			n = Vec3f(0, 1, 0);
		}
		if (abs(p.y - bounds[0].y) < 0.01
				&& p.z > bounds[0].z && p.z < bounds[1].z
				&& p.x > bounds[0].x && p.x < bounds[1].x) {
			n = Vec3f(0, -1, 0);
		}

	}

	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t) const {

//		float t1 = (bounds[0] - rayorig) / raydir;
//		float t2 = (bounds[1] - rayorig) / raydir;

		float tmin = (bounds[0].x - rayorig.x) / raydir.x;
		float tmax = (bounds[1].x - rayorig.x) / raydir.x;

		if (tmin > tmax) std::swap(tmin, tmax);

		float tymin = (bounds[0].y - rayorig.y) / raydir.y;
		float tymax = (bounds[1].y - rayorig.y) / raydir.y;

		if (tymin > tymax) std::swap(tymin, tymax);

		if ((tmin > tymax) || (tymin > tmax)) return false;

		if (tymin > tmin) tmin = tymin;

		if (tymax < tmax) tmax = tymax;

		float tzmin = (bounds[0].z - rayorig.z) / raydir.z;
		float tzmax = (bounds[1].z - rayorig.z) / raydir.z;

		if (tzmin > tzmax) std::swap(tzmin, tzmax);

		if ((tmin > tzmax) || (tzmin > tmax)) return false;

		if (tzmin > tmin) tmin = tzmin;

		if (tzmax < tmax) tmax = tzmax;

		t = tmin;

		if (t < 0) {
			t = tmax;
			if (t < 0) return false;
		}

		return true;
	}
};

class Triangle : public Object {
public:
	Vec3f V0, V1, V2;
	Triangle(
		const Vec3f &v0,
		const Vec3f &v1,
		const Vec3f &v2,
		const Vec3f &sc,
		const Vec3f &ec = 0,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) :
		Object(sc, ec, refl, transp, ior), V0(v0), V1(v1), V2(v2) {
		// empty
	}

	void getSurfaceData(const Vec3f &p, Vec3f &n) const {
		Vec3f v0v1 = V1 - V0;
		Vec3f v0v2 = V2 - V0;
		n = v0v1.crossProduct(v0v2);
		n.normalize();
	}

	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t) const {
		Vec3f v0v1 = V1 - V0;
		Vec3f v0v2 = V2 - V0;
		Vec3f n = v0v1.crossProduct(v0v2);
		float area2 = n.length();

		float NdotRayDirction = n.dotProduct(raydir);
		if (fabs(NdotRayDirction) < 1e-4) return false;

		float d = n.dotProduct(V0);
		t = (n.dotProduct(rayorig) + d) / NdotRayDirction;
		if (t < 0) return false;

		Vec3f p = rayorig + raydir * t;
		Vec3f c;

		Vec3f edge0 = V1 - V0;
		Vec3f v0p = p - V0;
		c = edge0.crossProduct(v0p);
		if (n.dotProduct(c) < 0) return false;

		Vec3f edge1 = V2 - V1;
		Vec3f v1p = p - V1;
		c = edge1.crossProduct(v1p);
		if (n.dotProduct(c) < 0) return false;

		Vec3f edge2 = V0 - V2;
		Vec3f v2p = p - V2;
		c = edge2.crossProduct(v2p);
		if (n.dotProduct(c) < 0) return false;

		return true;
	}
};

class Polygon : public Object {
public:
	int nfaces;
	std::vector<int> faceIndex;
	std::vector<int> vertsIndex;
	std::vector<Vec3f> verts;
	Polygon(
//		const std::vector<std::unique_ptr<Object>> objects,
		const int nf,
		const std::vector<int> fIndex,
		const std::vector<int> vIndex,
		const std::vector<Vec3f> vs,
		const Vec3f &sc,
		const Vec3f &ec = 0,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) : Object(sc, ec, refl, transp, ior) {

		nfaces = nf;
		faceIndex = fIndex;
		vertsIndex = vIndex;
		verts = vs;

	}

	void devide(std::vector<std::unique_ptr<Object>> &objects) {
		for (int i = 0, k = 0; i < nfaces; i++) {
			for (int j = 0; j < faceIndex[i] - 2; j++) {
				Vec3f v0 = verts[vertsIndex[k]];
				Vec3f v1 = verts[vertsIndex[k + j + 1]];
				Vec3f v2 = verts[vertsIndex[k + j + 2]];
				Triangle *t = new Triangle(
					v0, v1, v2, Vec3f((float)(nfaces - i - 1 + 2) / (nfaces + 2)), emissionColor, reflection, transparency, ior);
				objects.push_back(std::unique_ptr<Triangle>(t));
//				std::cout << t->surfaceColor << std::endl;
//				std::cout << objects.size() << std::endl;
//				std::cout << surfaceColor + 0.05 * k << std::endl;
			}
			k += faceIndex[i];
		}

	}

	void getSurfaceData(const Vec3f &p, Vec3f &n) const {

	}

	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t) const {
		return false;
	}

};

void trace(
	const Vec3f &rayorig,
	const Vec3f &raydir,
	const std::vector<std::unique_ptr<Object>> &objects,
	float &tNear,
	const Object *&hitObject) {

	std::vector<std::unique_ptr<Object>>::const_iterator iter = objects.begin();
	for (; iter != objects.end(); iter++) {
		float t =kInfinity;
		if ((*iter)->intersect(rayorig, raydir, t) && t < tNear) {
			hitObject = iter->get();
			tNear = t;
		}
	}
//	return (hitObject != nullptr);
}


Vec3f castRay(
	const Vec3f &rayorig,
	const Vec3f &raydir,
	const std::vector<std::unique_ptr<Object>> &objects,
//	const std::vector<std::unique_ptr<Object>> &lights,
	const Options &options,
	const int &depth) {

	if (depth >= options.maxDepth) {
		return options.backgroundColor;
	}

	Vec3f hitColor = 0;
	float tNear = kInfinity;
	const Object *hitObject = nullptr;

	trace(rayorig, raydir, objects, tNear, hitObject);


	if (hitObject == nullptr) {
//		std::cout << "no hit object" << std::endl;
		return options.backgroundColor;
	}

//	this is a light
	if (hitObject->emissionColor.x > 0) {
		hitColor += hitObject->emissionColor;
	}

	Vec3f phit = rayorig + raydir * tNear;
	Vec3f nhit;

//	std::cout << "hit point: " << phit << std::endl;
//	std::cout << "hit object color: " << hitObject->surfaceColor << std::endl;
//	ofs1 << std::endl;
//	ofs1 << "depth: " << depth << std::endl;
//	ofs1 << "rayorig" << rayorig << std::endl;
//	ofs1 << "raydir" << raydir << std::endl;
//	ofs1 << "hit point: " << phit << std::endl;
//	ofs1 << "hit object color: " << hitObject->surfaceColor << std::endl;
	hitObject->getSurfaceData(phit, nhit);
	bool inside = false;
	if (raydir.dotProduct(nhit) > 0) {
		nhit = -nhit;
		inside = true;
	}
//	std::cout << nhit << std::endl;
//	exit(0);

//		ofs1 << "inside: " << inside << std::endl;
	if (hitObject->transparency > 0 || hitObject->reflection > 0) {
		float facingratio = -raydir.dotProduct(nhit);
//		float fresneleffect = mix(pow((1 - facingratio), 3), 1, 0.1);
		float fresneleffect = 1;
//		std::cout << "facingratio and fresnel: " << facingratio << " " << fresneleffect << std::endl;
//		ofs1 << "facingratio and fresnel: " << facingratio << " " << fresneleffect << std::endl;
//		exit(0);
		Vec3f refldir = raydir - nhit * 2 * raydir.dotProduct(nhit);
		refldir.normalize();
		Vec3f reflection = castRay(phit + nhit * options.bias,
				refldir, objects, options, depth + 1);
		Vec3f refraction = 0;
		if (hitObject->transparency) {
			float eta = (inside) ? hitObject->ior : 1 / hitObject->ior;
			float cosi = -nhit.dotProduct(raydir);
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3f refrdir = raydir * eta + nhit * (eta * cosi - sqrt(k));
			refrdir.normalize();
			refraction = castRay(phit - nhit * options.bias,
					refrdir, objects, options, depth + 1);
		}
//		std::cout <<  reflection << " " << refraction << std::endl;
//		exit(0);
//		ofs1 << "refl and refr" <<  reflection << " " << refraction << std::endl;
		hitColor = (reflection * fresneleffect + refraction *
				(1 - fresneleffect) * hitObject->transparency) * hitObject->surfaceColor;
//		std::cout << "hit color: " << hitColor << std::endl;
//		exit(0);
//		ofs1 << "hit color: " << hitColor << std::endl;
	}
//	else {
//		diffuse object
//		std::cout << "diffuse" << std::endl;
//		std::vector<std::unique_ptr<Object>>::const_iterator iter = lights.begin();
//		for (; iter != lights.end(); iter++) {
//			Vec3f transmission = 1;
//			Vec3f lightOrig = (*iter)->position;
//			Vec3f lightDir = lightOrig - phit;
//			lightDir.normalize();
//			const Object *shadowObject = nullptr;
//			float t = kInfinity;
//			std::cout << "phit: "  << phit << std::endl;
//			std::cout << "lightOrig: "  << lightOrig << std::endl;
//			std::cout << "lightDir: "  << lightDir << std::endl;
//			if (trace(lightOrig, lightDir, objects, t, shadowObject)) {
//				std::cout << shadowObject << " " << hitObject << std::endl;
//				if (shadowObject != hitObject) {
//						transmission = 0;
//				}
//			}
//			Vec3f lhit = phit + lightDir * t;
//			std::cout << "transmission: "  << transmission << std::endl;
//			std::cout << "lhit: "  << lhit << std::endl;
//			hitColor = hitColor + hitObject->surfaceColor * transmission *
//					std::max(float(0), nhit.dotProduct(lightDir)) * (*iter)->intensify;
//			std::cout << "hit color: " << hitColor << std::endl;
//			exit(0);
//		}
//	}
//	std::cout << "hit point: " << phit <<", hit color: " << hitColor << std::endl;

	return hitColor;
}


void render(
	const Options &options,
	const std::vector<std::unique_ptr<Object>> &objects) {

//	ofs1.open("./testByMe.txt");
	Vec3f *framebuffer = new Vec3f[options.width * options.height];
	Vec3f *pix = framebuffer;
	float scale = tan(deg2rad(options.fov * 0.5));
	float imageAspectRatio = options.width / (float)options.height;
	Vec3f rayorig(0);
	for (uint32_t j = 0; j < options.height; j++) {
		for (uint32_t i = 0; i < options.width; i++) {
			float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale;
			Vec3f raydir = Vec3f(x, y, -1).normalize();
			Vec3f ans;
			*(pix++)= ans = castRay(rayorig, raydir, objects, options, 0);
//			if (ans.x != 2) {
//				ofs1 << "j: " << j << ", i: " << i << std::endl;
//				ofs1 << "pix color: " << ans << std::endl;
//			}
		}
	}
//	ofs1.close();

	std::ofstream ofs("./out.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << options.width << " " << options.height << "\n255\n";
	for (unsigned i = 0; i < options.height * options.width; i++) {
//		std::cout << framebuffer[i] << std::endl;
		unsigned char r = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
		unsigned char g = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
		unsigned char b = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
		ofs << r << g << b;
//		ofs << (unsigned char)0 << (unsigned char)0 << (unsigned char)0;
	}
	ofs.close();

	delete [] framebuffer;
}


int main() {

	std::vector<std::unique_ptr<Object>> objects;

	Sphere *sph0 = new Sphere(Vec3f(-2, 0, -6), 2, Vec3f(1.00, 0.32, 0.36), Vec3f(0), 1, 0, 1.1);
	Sphere *sph1 = new Sphere(Vec3f(1.0, 3, -7), 2, Vec3f(0.90, 0.76, 0.46), Vec3f(0), 1, 0, 1.1);
	Sphere *sph2 = new Sphere(Vec3f(4.0, 0, -7), 2, Vec3f(0.65, 0.77, 0.97), Vec3f(0), 1, 0, 1.1);
	Sphere *sph3 = new Sphere(Vec3f(-3.5, 0, -8), 2, Vec3f(0.90, 0.90, 0.90), Vec3f(0), 1, 0, 1.1);
//	objects.push_back(std::unique_ptr<Sphere>(sph0));
//	objects.push_back(std::unique_ptr<Sphere>(sph1));
//	objects.push_back(std::unique_ptr<Sphere>(sph2));
//	objects.push_back(std::unique_ptr<Sphere>(sph3));

	AABBox *leftWall = new AABBox(Vec3f(-8, -5, -10), Vec3f(-7.5, 7, -5), Vec3f(3), Vec3f(0), 1, 0, 1.1);
	AABBox *backWall = new AABBox(Vec3f(-7.5, -5, -10.5), Vec3f(7.5, 7, -10), Vec3f(3), Vec3f(0), 1, 0, 1.1);
	AABBox *rightWall = new AABBox(Vec3f(7.5, -5, -10), Vec3f(8, 7, -5), Vec3f(3), Vec3f(0), 1, 0, 1.1);
	AABBox *downWall = new AABBox(Vec3f(-8, -5.5, -10.5), Vec3f(8, -5, -5), Vec3f(3), Vec3f(0), 1, 0, 1.1);

//	objects.push_back(std::unique_ptr<AABBox>(leftWall));
//	objects.push_back(std::unique_ptr<AABBox>(backWall));
//	objects.push_back(std::unique_ptr<AABBox>(rightWall));
//	objects.push_back(std::unique_ptr<AABBox>(downWall));

	Triangle *t = new Triangle(
			Vec3f(-8, -4, -4), Vec3f(2, -4, -15), Vec3f(7, -4, -4), Vec3f(0.5, 0.3, 0.3), Vec3f(0), 1, 0, 1.1);

//	objects.push_back(std::unique_ptr<Triangle>(t));

	int nfaces = 4;
	std::vector<int> faceIndex = {3, 3, 3, 3};
	std::vector<int> vertsIndex = {0, 1, 2, 2, 1, 3, 2, 3, 0, 0, 3, 1};
	std::vector<Vec3f> verts =
		{Vec3f(0, 0, -3), Vec3f(2, -2, -5), Vec3f(-1, 4, -4), Vec3f(-4, -2, -4)};

//	verts[] = {Vec3f(0, 0, -3), Vec3f(2, -2, -5), Vec3f(-1, 4, -4), Vec3f(-4, -2, -4)};

	Polygon *polygon = new Polygon
			(nfaces, faceIndex, vertsIndex, verts, Vec3f(0.3), Vec3f(0), 1, 0, 1.1);
//	polygon->devide(objects);

	int nfaces1 = 6;
	std::vector<int> faceIndex1 = {4, 4, 4, 4, 4, 4};
	std::vector<int> vertsIndex1 =
		{0, 1, 3, 2, 2, 3, 5, 4, 4, 5, 7, 6, 6, 7, 1, 0, 1, 7, 5, 3, 6, 0, 2, 4};
	std::vector<Vec3f> verts1 =
		{Vec3f(-0.5, -0.5, 0.5 - 2), Vec3f(0.5, -0.5, 0.5 - 2), Vec3f(-0.5, 0.5, 0.5 - 2), Vec3f(0.5, 0.5, 0.5 - 2),
		Vec3f(-0.5 + 1, 0.5 + 1, -0.5 - 2), Vec3f(0.5 + 1, 0.5 + 1, -0.5 - 2), Vec3f(-0.5 + 1, -0.5 + 1, -0.5 - 2), Vec3f(0.5 + 1, -0.5 + 1, -0.5 - 2)};

	//	verts[] = {Vec3f(0, 0, -3), Vec3f(2, -2, -5), Vec3f(-1, 4, -4), Vec3f(-4, -2, -4)};

	Polygon *polygon1 = new Polygon
			(nfaces1, faceIndex1, vertsIndex1, verts1, Vec3f(0.3), Vec3f(0), 1, 0, 1.1);
	polygon1->devide(objects);

//	std::cout << objects.size() << std::endl;
//	lights
	Sphere *l1 = new Sphere(Vec3f(-1, 10, 0), 1, Vec3f(0), Vec3f(50));
	Sphere *l2 = new Sphere(Vec3f(8, 0, 0), 2, Vec3f(0), Vec3f(30));
	Sphere *l3 = new Sphere(Vec3f(0, 0, -50), 2, Vec3f(0), Vec3f(30));

//	objects.push_back(std::unique_ptr<Object>(l1));
//	objects.push_back(std::unique_ptr<Object>(l2));
//	objects.push_back(std::unique_ptr<Object>(l3));

//	std::cout << lights.size() << std::endl;

	Options options;
	options.width = 1280;
	options.height = 1080;
	options.fov = 120;
	options.backgroundColor = Vec3f(1);
	options.maxDepth = 5;
	options.bias = 0.00001;

	render(options, objects);

	return 0;
}
