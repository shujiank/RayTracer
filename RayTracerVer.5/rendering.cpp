//[header]
// A simple program to demonstrate some basic shading techniques
//[/header]
//[compile]
// Download the raytracetransform.cpp, geometry.h and teapot.geo file to a folder.
// Open a shell/terminal, and run the following command where the files are saved:
//
// c++ -o shading shading.cpp -std=c++11 -O3
//
// Run with: ./shading. Open the file ./out.png in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2012  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//[/ignore]


#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>
#include <vector>
#include <cstring>
#include <string>

#include "geometry.h"

#define M_PI 3.14159
static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;
static const Vec3f kDefaultBackgroundColor = Vec3f(1);
template <> const Matrix44f Matrix44f::kIdentity = Matrix44f();
const int MAX_LINE_LENGTH = 100;

inline
float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline
float deg2rad(const float &deg)
{ return deg * M_PI / 180; }

inline
Vec3f mix(const Vec3f &a, const Vec3f& b, const float &mixValue)
{ return a * (1 - mixValue) + b * mixValue; }

struct Options {
    uint32_t width;
    uint32_t height;
    float fov;
    Vec3f backgroundColor;
    Matrix44f cameraToWorld;
    float bias;
    uint32_t maxDepth;
};

enum MaterialType { kDiffuse, kReflection, kReflectionAndRefraction, kPhong };

struct Vertex {
	Vec3f position;
	Vec3f normal;
	Vec2f textureCoordinates;
};

struct Face {
	std::vector<Vertex> vertices;
};


struct HE_vert;
struct HE_edge;
struct HE_face;

struct HE_edge {
	HE_vert* sourceVert;
	HE_edge* pair;
	HE_face* face;
	HE_edge* next;
	Vertex edge_point;
	bool has_new = false;
};

struct HE_vert {
	Vertex vertex;
	HE_edge* edge;
	Vertex new_vertex;
	bool has_new = false;
};

struct HE_face {
	HE_edge* edge;
	Vertex face_point;
};


class Object {
 public:
    // [comment]
    // Setting up the object-to-world and world-to-object matrix
    // [/comment]
    Object(const Matrix44f &o2w) : objectToWorld(o2w), worldToObject(o2w.inverse()) {}
    virtual ~Object() {}
    virtual bool intersect(const Vec3f &, const Vec3f &, float &, uint32_t &, uint32_t &, Vec2f &) const = 0;
    virtual void getSurfaceProperties(const Vec3f &, const Vec3f &, const uint32_t &, const uint32_t &, const Vec2f &, Vec3f &, Vec2f &) const = 0;
    Matrix44f objectToWorld, worldToObject;
    Vec3f surfaceColor = Vec3f(1);
    MaterialType type = kPhong;
    float ior = 1.1; // refraction
    Vec3f albedo = 0.18;
    float Kd = 0.8; // phong model diffuse weight
    float Ks = 0.2; // phong model specular weight
    float n = 10;   // phong specular exponent
};


class Sphere : public Object {
public:
	Vec3f center;
	float radius, radius2;
	Sphere(
		const Matrix44f &o2w,
		const Vec3f &c,
		const float &r) :
		Object(o2w), center(c), radius(r), radius2(r * r) {}


	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t,
			uint32_t &faceIndex, uint32_t &triIndex, Vec2f &uv) const {
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

	void getSurfaceProperties(const Vec3f &hitPoint, const Vec3f &raydir, const uint32_t &faceIndex,
			const uint32_t &triIndex, const Vec2f &uv, Vec3f &hitNormal, Vec2f &hitTextureCoordinates) const {
		hitNormal = hitPoint - center;
		hitNormal.normalize();
	}
};


class AABBox : public Object {
public:
	Vec3f bounds[2];
	AABBox(
		const Matrix44f &o2w,
		const Vec3f &b0,
		const Vec3f &b1) : Object(o2w) {
		bounds[0] = b0;
		bounds[1] = b1;
	}

	void getSurfaceProperties(const Vec3f &p, const Vec3f &raydir, const uint32_t &faceIndex,
			const uint32_t &triIndex, const Vec2f &uv, Vec3f &hitNormal, Vec2f &hitTextureCoordinates) const {

		if (std::abs(p.z - bounds[1].z) < 0.0001
				&& p.x >= bounds[0].x && p.x <= bounds[1].x
				&& p.y >= bounds[0].y && p.y <= bounds[1].y) {
			hitNormal = Vec3f(0, 0, 1);
		}
		if (std::abs(p.z - bounds[0].z) < 0.0001
				&& p.x >= bounds[0].x && p.x <= bounds[1].x
				&& p.y >= bounds[0].y && p.y <= bounds[1].y) {
			hitNormal = Vec3f(0, 0, -1);
		}
		if (std::abs(p.x - bounds[1].x) < 0.0001
				&& p.z >= bounds[0].z && p.z <= bounds[1].z
				&& p.y >= bounds[0].y && p.y <= bounds[1].y) {
			hitNormal = Vec3f(1, 0, 0);
		}
		if (std::abs(p.x - bounds[0].x) < 0.0001
				&& p.z >= bounds[0].z && p.z <= bounds[1].z
				&& p.y >= bounds[0].y && p.y <= bounds[1].y) {
			hitNormal = Vec3f(-1, 0, 0);
		}
		if (std::abs(p.y - bounds[1].y) < 0.0001
				&& p.z >= bounds[0].z && p.z <= bounds[1].z
				&& p.x >= bounds[0].x && p.x <= bounds[1].x) {
			hitNormal = Vec3f(0, 1, 0);
		}
		if (std::abs(p.y - bounds[0].y) < 0.0001
				&& p.z >= bounds[0].z && p.z <= bounds[1].z
				&& p.x >= bounds[0].x && p.x <= bounds[1].x) {
			hitNormal = Vec3f(0, -1, 0);
		}

	}

	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t, uint32_t &faceIndex, uint32_t &triIndex, Vec2f &uv) const {

//		float t1 = (bounds[0] - rayorig) / raydir;
//		float t2 = (bounds[1] - rayorig) / raydir;
//		if (bounds[0].x == 0 && bounds[1].x == 0) return false;

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


class Triangle {
public:
	Vec3f v0, v1, v2;
	Vec3f midP;
	uint32_t faceIndex;
	uint32_t triIndex;


	Triangle(Vec3f &v0, Vec3f &v1, Vec3f &v2, uint32_t &faceIndex, uint32_t &triIndex)
		: v0(v0), v1(v1), v2(v2), faceIndex(faceIndex), triIndex(triIndex) {
		midP = (v0 + v1 + v2) / 3.0;
	}

	bool intersect(
	    const Vec3f &orig, const Vec3f &dir,
	    float &t, float &u, float &v)
	{
	    Vec3f v0v1 = v1 - v0;
	    Vec3f v0v2 = v2 - v0;
	    Vec3f pvec = dir.crossProduct(v0v2);
	    float det = v0v1.dotProduct(pvec);

	    // ray and triangle are parallel if det is close to 0
	    if (fabs(det) < kEpsilon) return false;

	    float invDet = 1 / det;

	    Vec3f tvec = orig - v0;
	    u = tvec.dotProduct(pvec) * invDet;
	    if (u < 0 || u > 1) return false;

	    Vec3f qvec = tvec.crossProduct(v0v1);
	    v = dir.dotProduct(qvec) * invDet;
	    if (v < 0 || u + v > 1) return false;

	    t = v0v2.dotProduct(qvec) * invDet;

	    return (t > 0) ? true : false;
	}

};


class KDNode {
public:
	AABBox BoundingBox;
	KDNode* left;
	KDNode* right;
	bool isLeaf = false;
	std::vector<Triangle> triangles;

	KDNode(): BoundingBox(Matrix44f::kIdentity, Vec3f(0), Vec3f(0)) {
		left = NULL;
		right = NULL;
		triangles = std::vector<Triangle>();
	}

	KDNode* build(const std::vector<Triangle> &tris, int depth) const {
		KDNode* node = new KDNode();
		node->triangles = tris;
//		std::cout << std::endl;
//		std::cout << "depth: " << depth << std::endl;
//		std::cout << "triangles num: " << node->triangles.size() << std::endl;
		if (tris.size() == 0) {
			return node;
		}
		node->BoundingBox = getBoundingBox(tris);
//		std::cout << "Bounding Box: " << node->BoundingBox.bounds[0] << std::endl;
		if (tris.size() == 1) {
			node->left = new KDNode();
			node->right = new KDNode();
			return node;
		}

		Vec3f midPoint(0);
		for (uint32_t i = 0; i < tris.size(); i++) {
			midPoint += tris[i].midP;
		}
		midPoint /= tris.size();

		std::vector<Triangle> left_tris;
		std::vector<Triangle> right_tris;

		for (uint32_t i = 0; i < tris.size(); i++) {
			switch(depth % 3) {
				case 0: {
					midPoint.x >= tris[i].midP.x ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
					break;
				}
				case 1: {
					midPoint.y >= tris[i].midP.y ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
					break;
				}
				case 2: {
					midPoint.z >= tris[i].midP.z ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
					break;
				}
			}
//			std::cout << "last member " << right_tris[i] << std::endl;
//			std::cout << "left size " << left_tris.size() << std::endl;
		}
//		std::cout << "left size " << left_tris.size() << std::endl;
//		std::cout << "right size " << right_tris.size() << std::endl;
//		exit(0);
		if (left_tris.size() >= 100 && depth < 20) {
//			std::cout << "go left " << std::endl;
			node->left = build(left_tris, depth + 1);
		} else {
//			std::cout << "IT'S LEAF!" << std::endl;
			node->left = new KDNode();
			node->left->BoundingBox = getBoundingBox(left_tris);
			node->left->triangles = left_tris;
			node->left->isLeaf = true;
		}

		if (right_tris.size() >= 100 && depth < 20) {
//			std::cout << "go right " << std::endl;
			node->right = build(right_tris, depth + 1);
		} else {
//			std::cout << "IT'S LEAF!" << std::endl;
			node->right = new KDNode();
			node->right->BoundingBox = getBoundingBox(right_tris);
			node->right->triangles = right_tris;
			node->right->isLeaf = true;
		}

		return node;
	}

	AABBox getBoundingBox(const std::vector<Triangle> &tris) const {
		float x0, y0, z0;
		float x1, y1, z1;
		x0 = y0 = z0 = kInfinity;
		x1 = y1 = z1 = -kInfinity;
//		std::cout << x0 << ", " << x1 << std::endl;
//		exit(0);

		for (uint32_t i = 0; i < tris.size(); i++) {

			if (tris[i].v0.x < x0) x0 = tris[i].v0.x;
			if (tris[i].v0.y < y0) y0 = tris[i].v0.y;
			if (tris[i].v0.z < z0) z0 = tris[i].v0.z;

			if (tris[i].v0.x > x1) x1 = tris[i].v0.x;
			if (tris[i].v0.y > y1) y1 = tris[i].v0.y;
			if (tris[i].v0.z > z1) z1 = tris[i].v0.z;

			if (tris[i].v1.x < x0) x0 = tris[i].v1.x;
			if (tris[i].v1.y < y0) y0 = tris[i].v1.y;
			if (tris[i].v1.z < z0) z0 = tris[i].v1.z;

			if (tris[i].v1.x > x1) x1 = tris[i].v1.x;
			if (tris[i].v1.y > y1) y1 = tris[i].v1.y;
			if (tris[i].v1.z > z1) z1 = tris[i].v1.z;

			if (tris[i].v2.x < x0) x0 = tris[i].v2.x;
			if (tris[i].v2.y < y0) y0 = tris[i].v2.y;
			if (tris[i].v2.z < z0) z0 = tris[i].v2.z;

			if (tris[i].v2.x > x1) x1 = tris[i].v2.x;
			if (tris[i].v2.y > y1) y1 = tris[i].v2.y;
			if (tris[i].v2.z > z1) z1 = tris[i].v2.z;
		}
		return AABBox(Matrix44f::kIdentity, Vec3f(x0, y0, z0) - 0.1, Vec3f(x1, y1, z1) + 0.1);
	}

	bool intersect(
		const Vec3f &orig, const Vec3f &dir,
	    float &tNear, uint32_t &faceIndex, uint32_t &triIndex, Vec2f &uv) {

		float t = kInfinity;
//		std::cout << triangles.size() << std::endl;
//		std::cout << "left size " << this->left->triangles.size() << std::endl;
//		std::cout << "right size " << this->right->triangles.size() << std::endl;
//		std::cout << BoundingBox.bounds[0] << std::endl;
		if (BoundingBox.intersect(orig, dir, t, faceIndex, triIndex, uv)) {
//			if (this->left->triangles.size() > 0 || this->right->triangles.size() > 0) {
			if (this->isLeaf == false) {
				bool hitleft = this->left->intersect(orig, dir, tNear, faceIndex, triIndex, uv);
				bool hitright = this->right->intersect(orig, dir, tNear, faceIndex, triIndex, uv);
				return hitleft || hitright;
			} else {
				bool isect = false;
				for (uint32_t i = 0; i < this->triangles.size(); i++) {
					float t = kInfinity, u, v;
					if (this->triangles[i].intersect(orig, dir, t, u, v)
						&& t < tNear) {
						tNear = t;
						uv.x = u;
						uv.y = v;
						faceIndex = this->triangles[i].faceIndex;
						triIndex = this->triangles[i].triIndex;
						isect = true;
					}
				}
				return isect;
			}
		}
//		std::cout << "end up intersecting" << std::endl;
		return false;
	}

	void displayLeaf() {

	}

};


class TriangleMesh : public Object {
public:

    bool smoothShading = true;              // smooth shading by default
    std::vector<Face> faces;
    std::vector<HE_face*> HE_Faces;
    KDNode* kdtree;

    TriangleMesh(
        const Matrix44f &o2w,
		const std::vector<Face> fs) :
        Object(o2w),
        faces(fs) {

        Matrix44f transformNormals = worldToObject.transpose();
        for (uint32_t i = 0; i < faces.size(); i++) {
        	for (uint32_t j = 0; j < faces[i].vertices.size(); j++) {
        		objectToWorld.multVecMatrix(fs[i].vertices[j].position, faces[i].vertices[j].position);
        		transformNormals.multDirMatrix(fs[i].vertices[j].normal, faces[i].vertices[j].normal);
        	}
        }

        std::cout << "start catmull_clark " << std::endl;
//        std::cout << "faces size: " << faces.size() << std::endl;

        for (uint32_t i = 0; i < 1; i++) {
        	std::cout << i << ": " << std::endl;
        	catmull_clark();
            std::cout << "faces size: " << faces.size() << std::endl;
        }
        std::cout << "end catmull_clark " << std::endl;

//        exit(0);

        std::vector<Triangle> tris;
        for (uint32_t i = 0; i < faces.size(); i++) {
        	Vec3f &v0 = faces[i].vertices[0].position;
        	for (uint32_t j = 1; j < faces[i].vertices.size() - 1; j++) {
        		Vec3f &v1 = faces[i].vertices[j].position;
        		Vec3f &v2 = faces[i].vertices[j + 1].position;
        		Triangle t(v0, v1, v2, i, j);
        		tris.push_back(t);
        	}
        }

        this->kdtree = new KDNode();
        this->kdtree = this->kdtree->build(tris, 0);
//        std::cout << this->kdtree->triangles.size() << std::endl;
//        exit(0);
//        std::cout << "end up building" << std::endl;
    }

    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear,
    		uint32_t &faceIndex, uint32_t &triIndex, Vec2f &uv) const {
//    	std::cout << "start intersecting" << std::endl;
    	return this->kdtree->intersect(orig, dir, tNear, faceIndex, triIndex, uv);
    }

    void getSurfaceProperties(
        const Vec3f &hitPoint,
        const Vec3f &viewDirection,
		const uint32_t &faceIndex,
        const uint32_t &triIndex,
        const Vec2f &uv,
        Vec3f &hitNormal,
        Vec2f &hitTextureCoordinates) const {
    	const Vec3f &n0 = faces[faceIndex].vertices[0].normal;
    	const Vec3f &n1 = faces[faceIndex].vertices[triIndex].normal;
    	const Vec3f &n2 = faces[faceIndex].vertices[triIndex + 1].normal;
    	hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;

        const Vec2f &st0 = faces[faceIndex].vertices[0].textureCoordinates;
        const Vec2f &st1 = faces[faceIndex].vertices[triIndex].textureCoordinates;
        const Vec2f &st2 = faces[faceIndex].vertices[triIndex + 1].textureCoordinates;
        hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;

    }

    void transferHE() {

//    	build half-edge data structure
        for (uint32_t i = 0; i < faces.size(); i++) {
        	HE_face* f1;
        	f1 = (HE_face*)malloc(sizeof(HE_face));
        	f1->face_point.position = Vec3f(0);
        	f1->face_point.normal = Vec3f(0);
        	f1->face_point.textureCoordinates = Vec2f(0);
        	f1->edge = NULL;
        	for (uint32_t j = 0; j < faces[i].vertices.size(); j++) {
        		HE_vert* v1;
        		HE_edge* e1;
        		HE_edge* prev = f1->edge;
        		v1 = (HE_vert*)malloc(sizeof(HE_vert));
        		e1 = (HE_edge*)malloc(sizeof(HE_edge));
        		e1->sourceVert = v1;
        		e1->pair = NULL;
        		e1->face = f1;
        		e1->next = NULL;
        		e1->edge_point.position = Vec3f(0);
        		e1->edge_point.normal = Vec3f(0);
        		e1->edge_point.textureCoordinates = Vec2f(0);
        		v1->vertex = faces[i].vertices[j];
        		v1->edge = e1;
        		v1->new_vertex.position = Vec3f(0);
        		v1->new_vertex.normal = Vec3f(0);
        		v1->new_vertex.textureCoordinates = Vec2f(0);

        		if (j == 0) {
        			f1->edge = e1;
        			continue;
        		}
        		while (prev->next != NULL) prev = prev->next;
        		prev->next = e1;
        		if (j == faces[i].vertices.size() - 1) e1->next = f1->edge;
        	}
        	HE_Faces.push_back(f1);
        }

//        find pair
        int count = 0;
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {
        	HE_edge* edge1 = HE_Faces[i]->edge;
        	do {
        		HE_vert* source1 = edge1->sourceVert;
        		HE_vert* target1 = edge1->next->sourceVert;
        		bool found = false;
        		for (uint32_t j = 0; j < HE_Faces.size(); j++) {
        			if (i == j) continue;
        			if (found) break;
        			HE_edge* edge2 = HE_Faces[j]->edge;
        			do {
        				HE_vert* source2 = edge2->sourceVert;
        				HE_vert* target2 = edge2->next->sourceVert;
        				if (std::abs(source1->vertex.position.x - target2->vertex.position.x) < 0.0000001 &&
        					std::abs(source1->vertex.position.y - target2->vertex.position.y) < 0.0000001 &&
							std::abs(source1->vertex.position.z - target2->vertex.position.z) < 0.0000001 &&
							std::abs(source2->vertex.position.x - target1->vertex.position.x) < 0.0000001 &&
							std::abs(source2->vertex.position.y - target1->vertex.position.y) < 0.0000001 &&
							std::abs(source2->vertex.position.z - target1->vertex.position.z) < 0.0000001) {
        					edge1->pair = edge2;
        					edge2->pair = edge1;
        					found = true;
        					break;
        				}
        				edge2 = edge2->next;
        			} while (edge2 != HE_Faces[j]->edge);
        		}
        		if (!found) {
        			count++;
        			std::cout << "source: " << source1->vertex.position << std::endl;
        			std::cout << "target: " << target1->vertex.position << std::endl;
        			std::cout << count << std::endl;
        			std::cout << std::endl;

        		}
        		edge1 = edge1->next;

        	} while (edge1 != HE_Faces[i]->edge);
        }
//        std::cout << "not found num: " << count << std::endl;
//        showHE();
    }

    void showHE() {
    	int num = 0;
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {
        	HE_edge* edge1 = HE_Faces[i]->edge;
//        	std::cout << "face point : " << HE_Faces[i]->face_point.position << std::endl;
        	do {

        		if (edge1->pair == NULL) {
//        			std::cout << "no pair" << std::endl;
        			num++;
        		}
//        		std::cout << "source vert: " << edge1->sourceVert->vertex.position << std::endl;
//        		std::cout << "new vert: " << edge1->sourceVert->new_vertex.position << std::endl;
//        		std::cout << "edge point : " << edge1->edge_point.position << std::endl;

        		edge1 = edge1->next;
        	} while (edge1 != HE_Faces[i]->edge);
//        	std::cout << std::endl;
        }
        std::cout << "no pair count: " << num << std::endl;
        if (num > 0) {
        	exit(0);
        }
    }

    void catmull_clark() {

//    	create half-edge data strucure
    	std::cout << "start transfer HE" << std::endl;
    	transferHE();
    	showHE();
    	std::cout << "end transfer HE" << std::endl;

    	std::cout << "start face point" << std::endl;

//    	new face point
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {
        	HE_edge* edge1 = HE_Faces[i]->edge;
        	uint32_t num = 0;
        	do {
        		HE_Faces[i]->face_point.position += edge1->sourceVert->vertex.position;
        		HE_Faces[i]->face_point.normal += edge1->sourceVert->vertex.normal;
        		HE_Faces[i]->face_point.textureCoordinates += edge1->sourceVert->vertex.textureCoordinates;
        		num += 1;
        		edge1 = edge1->next;
        	} while (edge1 != HE_Faces[i]->edge);

        	HE_Faces[i]->face_point.position /= num;
        	HE_Faces[i]->face_point.normal /= num;
        	HE_Faces[i]->face_point.textureCoordinates /= num;

        	HE_Faces[i]->face_point.normal.normalize();
        }

    	std::cout << "start edge point" << std::endl;

//        new edge point
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {

        	HE_edge* edge1 = HE_Faces[i]->edge;

        	do {

        		if (edge1->pair->has_new == false) {

        			edge1->edge_point.position = (edge1->sourceVert->vertex.position +
        				edge1->next->sourceVert->vertex.position + edge1->face->face_point.position +
						edge1->pair->face->face_point.position) / 4;


        			edge1->edge_point.normal = (edge1->sourceVert->vertex.normal +
        				edge1->next->sourceVert->vertex.normal + edge1->face->face_point.normal +
						edge1->pair->face->face_point.normal) / 4;

        			edge1->edge_point.normal.normalize();

        			edge1->edge_point.textureCoordinates = (edge1->sourceVert->vertex.textureCoordinates +
        				edge1->next->sourceVert->vertex.textureCoordinates + edge1->face->face_point.textureCoordinates +
						edge1->pair->face->face_point.textureCoordinates) / 4;

        			edge1->has_new = true;

        		} else {
        			edge1->edge_point = edge1->pair->edge_point;
        			edge1->has_new = true;
        		}

        		edge1 = edge1->next;

        	} while (edge1 != HE_Faces[i]->edge);
        }

    	std::cout << "start vert point" << std::endl;
//        new vert point
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {
        	HE_edge* edge1 = HE_Faces[i]->edge;
        	int count = 0;
        	do {

//        		std::cout << count++ << std::endl;
//        		did we calculate before
        		if (edge1->sourceVert->has_new == false) {
        			HE_edge* edge2 = edge1;
        			Vertex ek, fk1;
        			uint32_t num = 0;
        			do {
//        				std::cout << count++ << std::endl;
        				ek.position += edge2->next->sourceVert->vertex.position;
        				fk1.position += edge2->face->face_point.position;

        				ek.normal += edge2->next->sourceVert->vertex.normal;
        				fk1.normal += edge2->face->face_point.normal;

        				ek.textureCoordinates += edge2->next->sourceVert->vertex.textureCoordinates;
        				fk1.textureCoordinates += edge2->face->face_point.textureCoordinates;

        				num += 1;
        				edge2 = edge2->pair->next;
        			} while (edge2 != edge1);

        			edge1->sourceVert->new_vertex.position = (num - 2) / (float) num * edge1->sourceVert->vertex.position +
        				1 / (float) (num * num) * ek.position + 1 / (float) (num * num) * fk1.position;

        			edge1->sourceVert->new_vertex.normal = (num - 2) / (float) num * edge1->sourceVert->vertex.normal +
        				1 / (float) (num * num) * ek.normal + 1 / (float) (num * num) * fk1.normal;

        			edge1->sourceVert->new_vertex.normal.normalize();

        			edge1->sourceVert->new_vertex.textureCoordinates = (num - 2) / (float) num * edge1->sourceVert->vertex.textureCoordinates +
        				1 / (float) (num * num) * ek.textureCoordinates + 1 / (float) (num * num) * fk1.textureCoordinates;

        			edge1->sourceVert->has_new = true;

//        			change all edges who share the same vert
        			HE_edge* vert_edge = edge1->pair->next;
        			while (vert_edge != edge1) {

        				vert_edge->sourceVert->new_vertex = edge1->sourceVert->new_vertex;
        				vert_edge->sourceVert->has_new = true;
        				vert_edge = vert_edge->pair->next;
        			}
        		}

        		edge1 = edge1->next;

        	} while (edge1 != HE_Faces[i]->edge);

        }
        std::cout << "start connect " << std::endl;
//        connect new points to form new mesh
        faces.clear();
        for (uint32_t i = 0; i < HE_Faces.size(); i++) {
        	HE_edge* prev = HE_Faces[i]->edge;
        	HE_edge* edge = HE_Faces[i]->edge;
        	while (prev->next != HE_Faces[i]->edge) {
        		prev = prev->next;
        	}

        	do {
        		Face f;

        		f.vertices.push_back(edge->sourceVert->new_vertex);
        		f.vertices.push_back(edge->edge_point);
        		f.vertices.push_back(HE_Faces[i]->face_point);
        		f.vertices.push_back(prev->edge_point);
        		faces.push_back(f);

        		edge = edge->next;
        		prev = prev->next;
        	} while (edge != HE_Faces[i]->edge);

        }
        std::cout << "start delete" << std::endl;

//        empty half-edge data
        deletePointers();

    }

    void deletePointers() {
    	for (uint32_t i = 0; i < HE_Faces.size(); i++) {
    		HE_edge* current = HE_Faces[i]->edge->next;
    		HE_edge* next = current->next;
    		do {
    			current->sourceVert->edge = NULL;
    			delete (current->sourceVert);
    			current->sourceVert = NULL;
    			current->face = NULL;
    			current->pair = NULL;
    			current->next = NULL;
    			delete (current);
    			current = NULL;
    			current = next;
    			next = next->next;
    		} while (current != HE_Faces[i]->edge);

			current->sourceVert->edge = NULL;
			delete (current->sourceVert);
			current->sourceVert = NULL;
			current->face = NULL;
			current->pair = NULL;
			current->next = NULL;
			delete (current);
			current = NULL;

    		delete (HE_Faces[i]);
    		HE_Faces[i] = NULL;

    	}
    	HE_Faces.clear();
    }
};


TriangleMesh* loadObjFile(const char *file, const Matrix44f &o2w) {
    std::ifstream ifs;
    try {
        ifs.open(file);
        if (ifs.fail()) throw;
        std::string line;

        std::vector<Vec3f> vertices;
        std::vector<Vec2f> textures;
        std::vector<Vec3f> normals;
        std::vector<Face> faces;

        while (ifs.peek() != EOF) {
        	ifs >> line;
        	if (line == "v") {
        		Vec3f v;
        		ifs >> v.x >> v.y >> v.z;
        		vertices.push_back(v);
//        		std::cout << v << std::endl;
        	} else if (line == "vt") {
        		Vec2f vt;
        		ifs >> vt.x >> vt.y;
        		textures.push_back(vt);
//        		std::cout << vt << std::endl;
        	} else if (line == "vn") {
        		Vec3f vn;
        		ifs >> vn.x >> vn.y >> vn.z;
        		normals.push_back(vn);
//        		std::cout << vn << std::endl;
        	} else if (line == "f") {
        		std::string s1;
        		std::getline(ifs, s1, '\n');

        		Face ff;

        		std::stringstream ss(s1);
        		std::string f;
         		while (std::getline(ss, f, ' ')) {
//         			std::cout << f << std::endl;
         			if (f == "") continue;
         			std::stringstream sf(f);
         			std::string num;
         			Vertex vv;
         			std::getline(sf, num, '/');
         			vv.position = vertices[atoi(num.c_str()) - 1];
         			std::getline(sf, num, '/');
         			vv.textureCoordinates = textures[atoi(num.c_str()) - 1];
         			std::getline(sf, num, '/');
         			vv.normal = normals[atoi(num.c_str()) - 1];
         			ff.vertices.push_back(vv);
         		}
         		faces.push_back(ff);
        	} else {
        		std::getline(ifs, line, '\n');
        	}

        }
        /*
        for (uint32_t i = 0; i < faces.size(); i++) {
        	std::cout << "face number: " << i << std::endl;
        	std::cout << "face edges number: " << faces[i].edgesNumber << std::endl;
        	for (uint32_t j = 0; j < faces[i].vertices.size(); j++) {
        		std::cout << "vertex  " << j << ": " << faces[i].vertices[j].position << std::endl;
        		std::cout << "normal  " << j << ": " << faces[i].vertices[j].normal << std::endl;
        		std::cout << "texture " << j << ": " << faces[i].vertices[j].textureCoordinates << std::endl;
        	}
        	std::cout << std::endl;
        }
        */
        return new TriangleMesh(o2w, faces);
    }
    catch (...) {
        ifs.close();
    }
    ifs.close();

    return nullptr;
}


class Light
{
public:
    Light(const Matrix44f &l2w, const Vec3f &c = 1, const float &i = 1) :
    	lightToWorld(l2w), color(c), intensity(i) {

    }
    virtual ~Light() {}
    virtual void illuminate(const Vec3f &P, Vec3f &, Vec3f &, float &) const = 0;
    Vec3f color;
    float intensity;
    Matrix44f lightToWorld;
};

class DistantLight : public Light
{
    Vec3f dir;
public:
    DistantLight(const Matrix44f &l2w, const Vec3f &c = 1, const float &i = 1) : Light(l2w, c, i)
    {
        l2w.multDirMatrix(Vec3f(0, 0, -1), dir);
        dir.normalize(); // in case the matrix scales the light
    }
    void illuminate(const Vec3f &P, Vec3f &lightDir, Vec3f &lightIntensity, float &distance) const
    {
        lightDir = dir;
        lightIntensity = color * intensity;
        distance = kInfinity;
    }
};


class PointLight : public Light
{
    Vec3f pos;
public:
    PointLight(const Matrix44f &l2w, const Vec3f &c = 1, const float &i = 1) : Light(l2w, c, i) {
    	l2w.multVecMatrix(Vec3f(0), pos);
    }
    // P: is the shaded point
    void illuminate(const Vec3f &P, Vec3f &lightDir, Vec3f &lightIntensity, float &distance) const
    {
        lightDir = (P - pos);
        float r2 = lightDir.norm();
        distance = sqrt(r2);
        lightDir.x /= distance, lightDir.y /= distance, lightDir.z /= distance;
        // avoid division by 0
        lightIntensity = color * intensity ;
    }
};

enum RayType { kPrimaryRay, kShadowRay };

struct IsectInfo
{
    const Object *hitObject = nullptr;
    float tNear = kInfinity;
    Vec2f uv;
    uint32_t faceIndex = 0;
    uint32_t triIndex = 0;
};


bool trace(
    const Vec3f &orig, const Vec3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects,
    IsectInfo &isect,
    RayType rayType = kPrimaryRay)
{
    isect.hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNear = kInfinity;
        uint32_t faceIndex = 0;
        uint32_t triIndex = 0;
        Vec2f uv;
        if (objects[k]->intersect(orig, dir, tNear, faceIndex, triIndex, uv) && tNear < isect.tNear) {
            isect.hitObject = objects[k].get();
            isect.tNear = tNear;
            isect.faceIndex = faceIndex;
            isect.triIndex = triIndex;
            isect.uv = uv;
        }
    }

    return (isect.hitObject != nullptr);
}


Vec3f reflect(const Vec3f &I, const Vec3f &N)
{
    return I - 2 * I.dotProduct(N) * N;
}


Vec3f refract(const Vec3f &I, const Vec3f &N, const float &ior) 
{ 
    float cosi = clamp(-1, 1, I.dotProduct(N));
    float etai = 1, etat = ior; 
    Vec3f n = N; 
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; } 
    float eta = etai / etat; 
    float k = 1 - eta * eta * (1 - cosi * cosi); 
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n; 
}


void fresnel(const Vec3f &I, const Vec3f &N, const float &ior, float &kr) 
{ 
    float cosi = clamp(-1, 1, I.dotProduct(N));
    float etai = 1, etat = ior; 
    if (cosi > 0) { std::swap(etai, etat); } 
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi)); 
    // Total internal reflection
    if (sint >= 1) { 
        kr = 1; 
    } 
    else { 
        float cost = sqrtf(std::max(0.f, 1 - sint * sint)); 
        cosi = fabsf(cosi); 
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost)); 
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost)); 
        kr = (Rs * Rs + Rp * Rp) / 2; 
    } 
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}


inline float modulo(const float &f)
{
    return f - std::floor(f);
}


Vec3f castRay(
    const Vec3f &orig, const Vec3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects,
    const std::vector<std::unique_ptr<Light>> &lights,
    const Options &options,
    const uint32_t & depth = 0)
{
    if (depth > options.maxDepth) return options.backgroundColor;
    Vec3f hitColor = 0;
    IsectInfo isect;
    if (trace(orig, dir, objects, isect)) {
        Vec3f hitPoint = orig + dir * isect.tNear;
        Vec3f hitNormal;
        Vec2f hitTexCoordinates;

        isect.hitObject->getSurfaceProperties(hitPoint, dir, isect.faceIndex, isect.triIndex, isect.uv, hitNormal, hitTexCoordinates);

        switch (isect.hitObject->type) {
        	case kDiffuse:
        	{
//        		std::cout << "collide" << std::endl;
        		for (uint32_t i = 0; i < lights.size(); ++i) {
        			Vec3f lightDir, lightIntensity;
        			IsectInfo isectShad;
        			lights[i]->illuminate(hitPoint, lightDir, lightIntensity, isectShad.tNear);
        			bool vis = !trace(hitPoint + hitNormal * options.bias, -lightDir, objects, isectShad, kShadowRay);
        			hitColor += vis * isect.hitObject->surfaceColor * lightIntensity *
        					std::max(0.f, hitNormal.dotProduct(-lightDir)) *
							(isect.hitObject->albedo / M_PI);
//        			if (vis) {
//        				std::cout << hitColor << std::endl;
//        				exit(0);
//        			}
        		}
        		break;
        	}

        	case kReflection:
        	{
        		Vec3f R = reflect(dir, hitNormal).normalize();
        		hitColor += 0.9 * castRay(hitPoint + options.bias * hitNormal, R, objects, lights, options, depth + 1);
        		break;
        	}

        	case kReflectionAndRefraction:
        	{
        		Vec3f refractionColor = 0, reflectionColor = 0;
        		// compute fresnel
        		float kr;
        		fresnel(dir, hitNormal, isect.hitObject->ior, kr);
        		bool outside = dir.dotProduct(hitNormal) < 0;
        		Vec3f bias = options.bias * hitNormal;
        		// compute refraction if it is not a case of total internal reflection
        		if (kr < 1) {
        			Vec3f refractionDirection = refract(dir, hitNormal, isect.hitObject->ior).normalize();
        			Vec3f refractionRayOrig = outside ? hitPoint - bias : hitPoint + bias;
        			refractionColor = castRay(refractionRayOrig, refractionDirection, objects, lights, options, depth + 1);
        		}

        		Vec3f reflectionDirection = reflect(dir, hitNormal).normalize();
        		Vec3f reflectionRayOrig = outside ? hitPoint + bias : hitPoint - bias;
        		reflectionColor = castRay(reflectionRayOrig, reflectionDirection, objects, lights, options, depth + 1);

        		// mix the two
        		hitColor += isect.hitObject->surfaceColor * (reflectionColor * kr + refractionColor * (1 - kr));
        		break;
        	}

            case kPhong:
            {
                Vec3f diffuse = 0, specular = 0;
                for (uint32_t i = 0; i < lights.size(); ++i) {
                    Vec3f lightDir, lightIntensity;
                    IsectInfo isectShad;
                    lights[i]->illuminate(hitPoint, lightDir, lightIntensity, isectShad.tNear);

                    bool vis = !trace(hitPoint + hitNormal * options.bias, -lightDir, objects, isectShad, kShadowRay);

                    // compute the diffuse component
                    diffuse += vis * isect.hitObject->albedo * lightIntensity * std::max(0.f, hitNormal.dotProduct(-lightDir));
                    
                    // compute the specular component
                    // what would be the ideal reflection direction for this light ray
                    Vec3f R = reflect(lightDir, hitNormal);
                    specular += vis * lightIntensity * std::pow(std::max(0.f, R.dotProduct(-dir)), isect.hitObject->n);


                }
                hitColor = isect.hitObject->surfaceColor * (diffuse * isect.hitObject->Kd + specular * isect.hitObject->Ks);
                //std::cout << hitColor << std::endl;
                break;
            }
            default:
                break;
        }
    }
    else {
        hitColor = options.backgroundColor;
    }

    return hitColor;
}


void render(
    const Options &options,
    const std::vector<std::unique_ptr<Object>> &objects,
    const std::vector<std::unique_ptr<Light>> &lights)
{

    std::unique_ptr<Vec3f []> framebuffer(new Vec3f[options.width * options.height]);
    Vec3f *pix = framebuffer.get();
    float scale = tan(deg2rad(options.fov * 0.5));
    float imageAspectRatio = options.width / (float)options.height;
    Vec3f orig;
    options.cameraToWorld.multVecMatrix(Vec3f(0), orig);
    auto timeStart = std::chrono::high_resolution_clock::now();
    for (uint32_t j = 0; j < options.height; ++j) {
        for (uint32_t i = 0; i < options.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale;
            Vec3f dir;
            options.cameraToWorld.multDirMatrix(Vec3f(x, y, -1), dir);
            dir.normalize();
            *(pix++) = castRay(orig, dir, objects, lights, options);
        }
//        if (uint32_t(j / (float)options.height * 100) % 10 == 0)
        std::cout << uint32_t(j / (float)options.height * 100) << '%' << std::endl;
//        fprintf(stderr, "\r%3d%c", uint32_t(j / (float)options.height * 100), '%');
    }
    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
    std::cout << "\rDone: " << passedTime / 1000 << "(sec)" << std::endl;
//    fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);
    
    // save framebuffer to file
	std::ofstream ofs("./out.ppm", std::ios::out | std::ios::binary);
    ofs << "P6\n" << options.width << " " << options.height << "\n255\n";
    for (uint32_t i = 0; i < options.height * options.width; ++i) {
        char r = (char)(255 * clamp(0, 1, framebuffer[i].x));
        char g = (char)(255 * clamp(0, 1, framebuffer[i].y));
        char b = (char)(255 * clamp(0, 1, framebuffer[i].z));
        ofs << r << g << b;
    }
    ofs.close();
}


void addObjects(
	Options &options,
	std::vector<std::unique_ptr<Object>> &objects,
	std::vector<std::unique_ptr<Light>> &lights) {


    
    float degree = 90;

    Matrix44f scaling(0.05, 0, 0, 0,
					  0, 0.05, 0, 0,
					  0, 0, 0.05, 0,
					  0, 0, 0, 1);

    Matrix44f translation(1, 0, 0, 0,
       					  0, 1, 0, 0,
    					  0, 0, 1, 0,
    					  6, -0.8, -6, 1);

    Matrix44f xRotation(1, 0, 0, 0,
    					0, cos(deg2rad(degree)), -sin(deg2rad(degree)), 0,
						0, sin(deg2rad(degree)), cos(deg2rad(degree)), 0,
						0, 0, 0, 1);

    Matrix44f zRotation(cos(deg2rad(degree)), -sin(deg2rad(degree)), 0, 0,
    					sin(deg2rad(degree)), cos(deg2rad(degree)), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1);

    Matrix44f yRotation(cos(deg2rad(degree)), 0, sin(deg2rad(degree)), 0,
    					0, 1, 0, 0,
						-sin(deg2rad(degree)), 0, cos(deg2rad(degree)), 0,
						0, 0, 0, 1);



    Matrix44f xform = yRotation * scaling * translation;

    /*
    TriangleMesh *teapot = loadPolyMeshFromFile("./teapot.geo", xform);
    if (teapot != nullptr) {
    	teapot->smoothShading = true;
    	teapot->surfaceColor = Vec3f(0.5);
        objects.push_back(std::unique_ptr<Object>(teapot));
    }

    scaling[0][0] = 0.15;
    scaling[1][1] = 0.15;
    scaling[2][2] = 0.15;
    translation[3][0] = -2;
    translation[3][1] = -4;
    translation[3][2] = -4;

    xform = yRotation * scaling * translation;

    TriangleMesh *cow = loadPolyMeshFromFile("./cow.geo", xform);
    if (cow != nullptr) {
    	cow->smoothShading = true;
    	cow->surfaceColor = Vec3f(0.1);
        objects.push_back(std::unique_ptr<Object>(cow));
    }

//    /*
	AABBox *leftWall = new AABBox(Matrix44f::kIdentity, Vec3f(-8, -5, -10), Vec3f(-7.5, 10, 5));
	AABBox *backWall = new AABBox(Matrix44f::kIdentity, Vec3f(-7.5, -5, -10.5), Vec3f(7.5, 10, -10));
	AABBox *rightWall = new AABBox(Matrix44f::kIdentity, Vec3f(7.5, -5, -10), Vec3f(8, 10, 5));
	AABBox *downWall = new AABBox(Matrix44f::kIdentity, Vec3f(-8, -5.5, -10.5), Vec3f(8, -5, 5));
	AABBox *formerWall = new AABBox(Matrix44f::kIdentity, Vec3f(-7.5, -5, 5), Vec3f(7.5, 10, 6));

	leftWall->surfaceColor = Vec3f(0.7);
	backWall->surfaceColor = Vec3f(0.7);
	rightWall->surfaceColor = Vec3f(0.7);
	downWall->surfaceColor = Vec3f(0.7);
	formerWall->surfaceColor = Vec3f(0.7);

	leftWall->type = kDiffuse;
	backWall->type = kDiffuse;
	rightWall->type = kDiffuse;
	downWall->type = kDiffuse;
	formerWall->type = kDiffuse;

	objects.push_back(std::unique_ptr<AABBox>(leftWall));
	objects.push_back(std::unique_ptr<AABBox>(backWall));
	objects.push_back(std::unique_ptr<AABBox>(rightWall));
	objects.push_back(std::unique_ptr<AABBox>(downWall));
	objects.push_back(std::unique_ptr<AABBox>(formerWall));

	AABBox *mirror = new AABBox(Matrix44f::kIdentity, Vec3f(-5, -3, -10), Vec3f(6.5, 8, -9.9));

	mirror->type = kReflection;

	objects.push_back(std::unique_ptr<AABBox>(mirror));

	AABBox *deskLeg1 = new AABBox(Matrix44f::kIdentity, Vec3f(6.9,     -5, -7    ), Vec3f(7.2,     -1, -6.85    ));
	AABBox *deskLeg2 = new AABBox(Matrix44f::kIdentity, Vec3f(6.9 - 2, -5, -7    ), Vec3f(7.2 - 2, -1, -6.85    ));
	AABBox *deskLeg3 = new AABBox(Matrix44f::kIdentity, Vec3f(6.9,     -5, -7 + 2.3), Vec3f(7.2,     -1, -6.85 + 2.3));
	AABBox *deskLeg4 = new AABBox(Matrix44f::kIdentity, Vec3f(6.9 - 2, -5, -7 + 2.3), Vec3f(7.2 - 2, -1, -6.85 + 2.3));
	AABBox *deskTop  = new AABBox(Matrix44f::kIdentity, Vec3f(4.6, -1, -7.3), Vec3f(7.3, -0.9, -4.0));

	deskLeg1->surfaceColor = Vec3f(0.5, 0.3, 0);
	deskLeg2->surfaceColor = Vec3f(0.5, 0.3, 0);
	deskLeg3->surfaceColor = Vec3f(0.5, 0.3, 0);
	deskLeg4->surfaceColor = Vec3f(0.5, 0.3, 0);
	deskTop->surfaceColor = Vec3f(0.5, 0.3, 0);

	deskLeg1->type = kDiffuse;
	deskLeg2->type = kDiffuse;
	deskLeg3->type = kDiffuse;
	deskLeg4->type = kDiffuse;
	deskTop->type = kDiffuse;


	objects.push_back(std::unique_ptr<AABBox>(deskLeg1));
	objects.push_back(std::unique_ptr<AABBox>(deskLeg2));
	objects.push_back(std::unique_ptr<AABBox>(deskLeg3));
	objects.push_back(std::unique_ptr<AABBox>(deskLeg4));
	objects.push_back(std::unique_ptr<AABBox>(deskTop));


	AABBox *chairLeg1 = new AABBox(Matrix44f::kIdentity, Vec3f(3, -5, -6.2), Vec3f(3.1, -2.5, -6.1));
	AABBox *chairLeg2 = new AABBox(Matrix44f::kIdentity, Vec3f(5, -5, -6.2), Vec3f(5.1, -2.5, -6.1));
	AABBox *chairLeg3 = new AABBox(Matrix44f::kIdentity, Vec3f(3, -5, -4.9), Vec3f(3.1, -2.5, -4.8));
	AABBox *chairLeg4 = new AABBox(Matrix44f::kIdentity, Vec3f(5, -5, -4.9), Vec3f(5.1, -2.5, -4.8));
	AABBox *chairTop = new AABBox(Matrix44f::kIdentity, Vec3f(3, -2.5, -6.2), Vec3f(5.1, -2.4, -4.8));
	AABBox *chairBack = new AABBox(Matrix44f::kIdentity, Vec3f(3, -2.4, -6.2), Vec3f(3.1, 0.5, -4.8));

	chairLeg1->surfaceColor = Vec3f(0.5, 0.3, 0.3);
	chairLeg2->surfaceColor = Vec3f(0.5, 0.3, 0.3);
	chairLeg3->surfaceColor = Vec3f(0.5, 0.3, 0.3);
	chairLeg4->surfaceColor = Vec3f(0.5, 0.3, 0.3);
	chairTop->surfaceColor = Vec3f(0.5, 0.3, 0.3);
	chairBack->surfaceColor = Vec3f(0.5, 0.3, 0.3);

	chairLeg1->type = kDiffuse;
	chairLeg1->type = kDiffuse;
	chairLeg1->type = kDiffuse;
	chairLeg1->type = kDiffuse;
	chairTop->type = kDiffuse;
	chairBack->type = kDiffuse;


	objects.push_back(std::unique_ptr<AABBox>(chairLeg1));
	objects.push_back(std::unique_ptr<AABBox>(chairLeg2));
	objects.push_back(std::unique_ptr<AABBox>(chairLeg3));
	objects.push_back(std::unique_ptr<AABBox>(chairLeg4));
	objects.push_back(std::unique_ptr<AABBox>(chairTop));
	objects.push_back(std::unique_ptr<AABBox>(chairBack));

	Sphere *sph0 = new Sphere(Matrix44f::kIdentity, Vec3f(0, 0, -2), 1);
	Sphere *sph1 = new Sphere(Matrix44f::kIdentity, Vec3f(0, -3, -3), 1);
	Sphere *sph2 = new Sphere(Matrix44f::kIdentity, Vec3f(4.0, 1, -5), 1);
	Sphere *sph3 = new Sphere(Matrix44f::kIdentity, Vec3f(0, 2, -8), 2);

	sph0->surfaceColor = Vec3f(0.4);
	sph1->surfaceColor = Vec3f(0.3);
	sph2->surfaceColor = Vec3f(0.5);
	sph3->surfaceColor = Vec3f(0.90, 0.90, 0.90);


	sph0->type = kDiffuse;
	sph1->type = kDiffuse;
	sph2->type = kReflectionAndRefraction;
	sph3->type = kReflection;

//	objects.push_back(std::unique_ptr<Sphere>(sph0));
//	objects.push_back(std::unique_ptr<Sphere>(sph1));
//	objects.push_back(std::unique_ptr<Sphere>(sph2));
//	objects.push_back(std::unique_ptr<Sphere>(sph3));

//	*/


}


int main(int argc, char **argv)
{
    // loading gemetry
    std::vector<std::unique_ptr<Object>> objects;
    // lights
    std::vector<std::unique_ptr<Light>> lights;
    Options options;

    // aliasing example
    options.width = 1280 / 1;
    options.height = 1080 / 1;
    options.fov = 120;
    options.backgroundColor = kDefaultBackgroundColor;
    options.maxDepth = 5;
    options.bias = 0.0001;
    options.cameraToWorld = Matrix44f(1, 0, 0, 0,
    								  0, 1, 0, 0,
									  0, 0, 1, 0,
									  0, 0, 5, 1);

//    addObjects(options, objects, lights);

    float degree = 0;

    Matrix44f scaling(0.7, 0, 0, 0,
					  0, 0.7, 0, 0,
					  0, 0, 0.7, 0,
					  0, 0, 0, 1);

    Matrix44f translation(1, 0, 0, 0,
       					  0, 1, 0, 0,
    					  0, 0, 1, 0,
    					  6, -0.8, -6, 1);

    Matrix44f xRotation(1, 0, 0, 0,
    					0, cos(deg2rad(degree)), -sin(deg2rad(degree)), 0,
						0, sin(deg2rad(degree)), cos(deg2rad(degree)), 0,
						0, 0, 0, 1);

    Matrix44f zRotation(cos(deg2rad(degree)), -sin(deg2rad(degree)), 0, 0,
    					sin(deg2rad(degree)), cos(deg2rad(degree)), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1);

    Matrix44f yRotation(cos(deg2rad(degree)), 0, sin(deg2rad(degree)), 0,
    					0, 1, 0, 0,
						-sin(deg2rad(degree)), 0, cos(deg2rad(degree)), 0,
						0, 0, 0, 1);



    Matrix44f xform = Matrix44f();
//    Matrix44f xform = yRotation * scaling;

    TriangleMesh *cube = loadObjFile("./cow.obj", xform);
    if (cube != nullptr) {
    	cube->surfaceColor = Vec3f(0.1);
        objects.push_back(std::unique_ptr<Object>(cube));
    }

    Matrix44f l2w(1, 0, 0, 0,
			      0, 1, 0, 0,
			      0, 0, 1, 0,
			      0, 0, 12, 1);

    lights.push_back(std::unique_ptr<Light>(new PointLight(l2w, 1, 20)));

    render(options, objects, lights);

    return 0;
}


