
//-------------------------------------------------------------------------------
///
/// \file       objects.h
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    5.0
/// \date       September 24, 2015
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _OBJECTS_H_INCLUDED_
#define _OBJECTS_H_INCLUDED_

#include "scene.h"
#include "cyTriMesh.h"

//-------------------------------------------------------------------------------

class Sphere : public Object
{
public:
    virtual bool IntersectRay( const Ray &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const;
    virtual Box GetBoundBox() const { return Box(-1,-1,-1,1,1,1); }
    virtual void ViewportDisplay(const Material *mtl) const;
};

extern Sphere theSphere;

//-------------------------------------------------------------------------------

class Plane : public Object
{
public:
    bool IntersectRay( const Ray &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const
    {
        float zero = 0.001f;
        Point3 P = ray.p;
        Point3 d = ray.dir;
        Point3 N(0,0,1); //plane coordinate
        
        float t = -(P.z/d.z);
        if(t >= zero && t < BIGFLOAT && t<hInfo.z){
            Point3 Hitp = P+t*d;
            //check if range is in unit plane
            if(Hitp.x >= -1 && Hitp.x <= 1 && Hitp.y >=-1 && Hitp.y <=1){
                hInfo.z = t;
                hInfo.p = Hitp;
                hInfo.N = N;
                if(N.Dot(d) < 0.0){
                    hInfo.front = false;
                }
                else hInfo.front = true;
                return true;
            }
        }
        return false;
    }
    virtual Box GetBoundBox() const { return Box(-1,-1,0,1,1,0); }
    virtual void ViewportDisplay(const Material *mtl) const;
};

extern Plane thePlane;

//-------------------------------------------------------------------------------

class TriObj : public Object, public cyTriMesh
{
public:
    bool IntersectRay( const Ray &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const{
        float t_max = BIGFLOAT; //Should be BIGFLOAT!
        //get box
        Box box_tri = GetBoundBox();
        if(box_tri.IsInside(ray.p)) return true;
        if(!box_tri.IntersectRay(ray,t_max)) return false;
        
        bool hit = false;
        //for each triangle face
        for(int i=0;i<NF();i++){
            hit |=IntersectTriangle(ray,hInfo,hitSide,i);
        }
        return hit;
    }
    virtual Box GetBoundBox() const { return Box(GetBoundMin(),GetBoundMax()); }
    virtual void ViewportDisplay(const Material *mtl) const;
    
    bool Load(const char *filename)
    {
        if ( ! LoadFromFileObj( filename ) ) return false;
        if ( ! HasNormals() ) ComputeNormals();
        ComputeBoundingBox();
        return true;
    }
    
private:
    bool IntersectTriangle( const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID ) const{
        float bias = 0.001f;
        TriFace triface = F(faceID);
        Point3 A = V(triface.v[0]);
        Point3 B = V(triface.v[1]);
        Point3 C = V(triface.v[2]);
        
        Point3 tNormal = (B - A).Cross(C - A);
        tNormal.Normalize();
        if(tNormal.Dot(ray.p-A) < bias) return false; //>90 degree
        if(tNormal.Dot(ray.dir) == 0) return false; //dir is parallel to triangle face
        float t = tNormal.Dot(C-ray.p)/tNormal.Dot(ray.dir);
        if(t<bias || t>=hInfo.z || t>=BIGFLOAT) return false;
        
        //find plane to project to
        int maxN = max(fabs(tNormal.x),fabs(tNormal.y));
        maxN = max(fabs(tNormal.z),maxN);
        
        Point3 P = ray.p + t * ray.dir;
        
        Point2 p1,p2,p3,pp;
        if(maxN==fabs(tNormal.x)){
            p1 = Point2(A.y, A.z);
            p2 = Point2(B.y, B.z);
            p3 = Point2(C.y, C.z);
            pp = Point2(P.y, P.z);
        }
        else if(maxN==fabs(tNormal.y)){
            p1 = Point2(A.x, A.z);
            p2 = Point2(B.x, B.z);
            p3 = Point2(C.x, C.z);
            pp = Point2(P.x, P.z);
        }
        else{
            p1 = Point2(A.x, A.y);
            p2 = Point2(B.x, B.y);
            p3 = Point2(C.x, C.y);
            pp = Point2(P.x, P.y);
        }
        
        float area_tri = (p2 - p1).Cross(p3 - p1);
        float area_23p = (p3 - pp).Cross(p2 - pp);
        float area_13p = (pp - p1).Cross(p3 - p1);
        float alpha = area_23p/area_tri;
        float beta = area_13p/area_tri;
        float gamma = 1.0-alpha-beta;
        
        if(alpha<-bias || beta<-bias || gamma<-bias || alpha>1.0 || beta>1.0 || gamma>1.0 ) return false;
        
        Point3 P_Normal = GetNormal(faceID, Point3(alpha,beta,gamma));
        hInfo.front = true;
        hInfo.p = P;
        hInfo.N = P_Normal;
        hInfo.z = t;
        
        return true;
    }
};

//-------------------------------------------------------------------------------

#endif
