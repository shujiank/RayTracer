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

	Vec3& crossProduct(const Vec3<T> &v) const {
		return Vec3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	T dotProduct(const Vec3<T> &v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	Vec3<T> operator * (const T &f) const { return Vec3<T>(x * f, y * f, z * f); }
	Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x * v.x, y * v.y, z * v.z); }
	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }
	friend std::ostream & operator << (std::ostream &os, const Vec3<T> &v) {
		os << "[" << v.x << " " << v.y << " " << v.z << "]";
		return os;
	}
};

typedef Vec3<float> Vec3f;

class Object {
public:
	Vec3f surfaceColor;
	float transparency, reflection;
	float ior; // Index of Refraction
	Object(
		const Vec3f &sc,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) : surfaceColor(sc), transparency(transp),
		reflection(refl), ior(ior) {}

	virtual ~Object() {}
	virtual bool intersect(const Vec3f &, const Vec3f &, float &) const = 0;
	virtual void getSurfaceData(const Vec3f &, Vec3f &) const = 0;
};

class Light {
public:
	Vec3f position;
	Vec3f intensify;
	Light(const Vec3f &p, const Vec3f &i) : position(p), intensify(i) {}
};

class Sphere : public Object {
public:
	Vec3f center;
	float radius, radius2;
	Sphere(
		const Vec3f &c,
		const float &r,
		const Vec3f &sc,
		const float &refl = 0,
		const float &transp = 0,
		const float &ior = 0) :
		Object(sc, refl, transp, ior), center(c), radius(r), radius2(r * r) {
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


bool trace(
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
	return (hitObject != nullptr);
}

Vec3f castRay(
	const Vec3f &rayorig,
	const Vec3f &raydir,
	const std::vector<std::unique_ptr<Object>> &objects,
	const std::vector<std::unique_ptr<Light>> &lights,
	const Options &options,
	const int &depth) {

	if (depth >= options.maxDepth) {
		return options.backgroundColor;
	}

	Vec3f hitColor = 0;
	float tNear = kInfinity;
	const Object *hitObject = nullptr;
	if (trace(rayorig, raydir, objects, tNear, hitObject)) {
		Vec3f phit = rayorig + raydir * tNear;
		Vec3f nhit;

//		std::cout << "hit point: " << phit << std::endl;
//		std::cout << "hit object color: " << hitObject->surfaceColor << std::endl;
//		ofs1 << std::endl;
//		ofs1 << "depth: " << depth << std::endl;
//		ofs1 << "rayorig" << rayorig << std::endl;
//		ofs1 << "raydir" << raydir << std::endl;
//		ofs1 << "hit point: " << phit << std::endl;
//		ofs1 << "hit object color: " << hitObject->surfaceColor << std::endl;
		hitObject->getSurfaceData(phit, nhit);
		bool inside = false;
		if (raydir.dotProduct(nhit) > 0) {
			nhit = -nhit;
			inside = true;
		}
//		ofs1 << "inside: " << inside << std::endl;
		if (hitObject->transparency > 0 || hitObject->reflection > 0) {
			float facingratio = -raydir.dotProduct(nhit);
			float fresneleffect = mix(pow((1 - facingratio), 3), 1, 0.1);
//			std::cout << facingratio << " " << fresneleffect << std::endl;
//			ofs1 << "facingratio and fresnel: " << facingratio << " " << fresneleffect << std::endl;
//			exit(0);
			Vec3f refldir = raydir - nhit * 2 * raydir.dotProduct(nhit);
			refldir.normalize();
			Vec3f reflection = castRay(phit + nhit * options.bias,
					refldir, objects, lights, options, depth + 1);
			Vec3f refraction = 0;
			if (hitObject->transparency) {
				float eta = (inside) ? hitObject->ior : 1 / hitObject->ior;
				float cosi = -nhit.dotProduct(raydir);
				float k = 1 - eta * eta * (1 - cosi * cosi);
				Vec3f refrdir = raydir * eta + nhit * (eta * cosi - sqrt(k));
				refrdir.normalize();
				refraction = castRay(phit - nhit * options.bias,
						refrdir, objects, lights, options, depth + 1);
			}
//			std::cout <<  reflection << " " << refraction << std::endl;
//			ofs1 << "refl and refr" <<  reflection << " " << refraction << std::endl;
			hitColor = (reflection * fresneleffect + refraction *
					(1 - fresneleffect) * hitObject->transparency) * hitObject->surfaceColor;
//			std::cout << "hit color: " << hitColor << std::endl;
//			ofs1 << "hit color: " << hitColor << std::endl;

		} else {
			std::cout << "diffuse" << std::endl;
			std::vector<std::unique_ptr<Light>>::const_iterator iter = lights.begin();
			for (; iter != lights.end(); iter++) {
				Vec3f transmission = 1;
				Vec3f lightOrig = (*iter)->position;
				Vec3f lightDir = lightOrig - phit;
				lightDir.normalize();
				const Object *shadowObject = nullptr;
				float t = kInfinity;
//				std::cout << "phit: "  << phit << std::endl;
//				std::cout << "lightOrig: "  << lightOrig << std::endl;
//				std::cout << "lightDir: "  << lightDir << std::endl;

				if (trace(lightOrig, lightDir, objects, t, shadowObject)) {
//					std::cout << shadowObject << " " << hitObject << std::endl;
					if (shadowObject != hitObject) {
						transmission = 0;
					}
				}
				Vec3f lhit = phit + lightDir * t;
//				std::cout << "transmission: "  << transmission << std::endl;
//				std::cout << "lhit: "  << lhit << std::endl;
				hitColor = hitColor + hitObject->surfaceColor * transmission *
						std::max(float(0), nhit.dotProduct(lightDir)) * (*iter)->intensify;
//				std::cout << "hit color: " << hitColor << std::endl;
//				exit(0);
			}
		}
//		std::cout << "hit point: " << phit <<", hit color: " << hitColor << std::endl;
	} else {
		hitColor = options.backgroundColor;
	}
	return hitColor;
}


void render(
	const Options &options,
	const std::vector<std::unique_ptr<Object>> &objects,
	const std::vector<std::unique_ptr<Light>> &lights) {

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
			*(pix++)= ans = castRay(rayorig, raydir, objects, lights, options, 0);
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
		unsigned char r = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
		unsigned char g = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
		unsigned char b = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
		ofs << r << g << b;
	}
	ofs.close();

	delete [] framebuffer;
}


int main() {

	std::vector<std::unique_ptr<Object>> objects;
	std::vector<std::unique_ptr<Light>> lights;

	uint32_t numSpheres = 1;

	gen.seed(0);
	for (uint32_t i = 0; i < numSpheres; i++) {
//		std::cout << dis(gen) << std::endl;
//		Vec3f randPos(dis(gen) * 2, dis(gen) * 3, -dis(gen) * 15);
//		float randRadius = dis(gen) * 2 + 1;
//		Vec3f randSurfaceColor(dis(gen), dis(gen), dis(gen));
		Vec3f randSurfaceColor(1.00, 0.32, 0.36);
		Vec3f randPos(-2, 0, -40);
		float randRadius = 4;
		float refl = dis(gen);
		float transp = dis(gen);
		float ior = dis(gen) + 1;
//		std::cout << randPos << " " << randRadius << " " << randSurfaceColor << " "
//				<< refl << " " << transp << " " << ior << std::endl;

		Sphere *sph = new Sphere(randPos, randRadius, randSurfaceColor, 1, 0, 1.1);
		objects.push_back(std::unique_ptr<Sphere>(sph));
	}
	Vec3f pos(1.0, 1.0, -15);
	float rad = 2;
	Vec3f color(0.90, 0.76, 0.46);

	Sphere *sph1 = new Sphere(Vec3f(1.0, 1.0, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0, 1.1);
	Sphere *sph2 = new Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0, 1.1);
	Sphere *sph3 = new Sphere(Vec3f(-5.5, 0, -15), 3, Vec3f(0.90, 0.90, 0.90), 1, 0, 1.1);
//	objects.push_back(std::unique_ptr<Sphere>(sph));
	objects.push_back(std::unique_ptr<Sphere>(sph1));
	objects.push_back(std::unique_ptr<Sphere>(sph2));
	objects.push_back(std::unique_ptr<Sphere>(sph3));
	uint32_t numLights = 1;

	gen.seed(0);
	for (uint32_t i = 0; i < numLights; i++) {
//		Vec3f randPos(-dis(gen) * 10, -dis(gen) * 10, -dis(gen) * 10);
//		float intensify = dis(gen);
		Vec3f randPos(0, 20, -20);
		Vec3f intensify(20);
		Light *l = new Light(randPos, intensify);
		lights.push_back(std::unique_ptr<Light>(l));
	}

	Options options;
	options.width = 1280 / 2;
	options.height = 960 / 2;
	options.fov = 30;
	options.backgroundColor = Vec3f(2);
	options.maxDepth = 5;
	options.bias = 0.00001;

	render(options, objects, lights);

	return 0;
}
