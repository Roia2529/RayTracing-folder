//-------------------------------------------------------------------------------
///
/// \file       objects.h
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    7.0
/// \date       October 6, 2015
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _OBJECTS_H_INCLUDED_
#define _OBJECTS_H_INCLUDED_

#include "scene.h"
#include "cyTriMesh.h"
#include "cyBVH.h"

//-------------------------------------------------------------------------------

class Sphere : public Object
{
public:
    bool IntersectRay( const Ray &ray, HitInfo &hitinfo, int hitSide=HIT_FRONT ) const{
            
            bool behitted = false;
            float a = ray.dir.Dot(ray.dir);
            float c = ray.p.Dot(ray.p)-1;
            float b = 2*ray.p.Dot(ray.dir);
            float insqrt = b*b-(4*a*c);
            float zero = 0.001f;
            if(insqrt>=zero){
                float t1 = (-b+sqrtf(insqrt))/(a*2);
                float t2 = (-b-sqrtf(insqrt))/(a*2);
                float prez = hitinfo.z;
                
                float min_t = t2;
                if(min_t>=prez ) return false;
                
                if(t1>zero && t2<zero && t1<prez) //one is negative
                {
                    float max_t = t1;
                    hitinfo.z = max_t;
                    hitinfo.front = false;
                    behitted = true;
                    hitinfo.p = hitinfo.z*ray.dir+ray.p;
                    hitinfo.N = hitinfo.p;
                    hitinfo.N.Normalize();
                    float u = 0.5 - atan2(hitinfo.p.x,hitinfo.p.y)/ (2 * M_PI);
                    float v = 0.5 + asin(hitinfo.p.z)/M_PI;
                    hitinfo.uvw = Point3(u,v,0);
                }
                else if(t1>zero && t2>zero && t2<prez){
                    
                    behitted = true;
                    hitinfo.z = min_t;
                    //do not forget to assign front=true
                    hitinfo.front = true;
                    //}
                    hitinfo.p = hitinfo.z*ray.dir+ray.p;
                    hitinfo.N = hitinfo.p;
                    hitinfo.N.Normalize();
                    float u = 0.5 - atan2(hitinfo.p.x,hitinfo.p.y)/ (2 * M_PI);
                    float temp = asin(hitinfo.p.z);
                    float v = 0.5 + asin(hitinfo.p.z)/M_PI;
                    hitinfo.uvw = Point3(u,v,0);
                }
            }
            return behitted;
        }
    virtual Box GetBoundBox() const { return Box(-1,-1,-1,1,1,1); }
    virtual void ViewportDisplay(const Material *mtl) const;
};

extern Sphere theSphere;

//-------------------------------------------------------------------------------

class Plane : public Object
{
public:
    
    
    bool IntersectRay( const Ray &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const{
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
                
                hInfo.uvw = Point3((hInfo.p.x+1)/2,(hInfo.p.y+1)/2,0);
                //calculateduvw(ray,hInfo);
                //hInfo.duvw[0] = Point3(0.001, 0, 0);
                //hInfo.duvw[1] = Point3(0,   0.001,0);
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
        
        if(TraceBVHNode(ray, hInfo, hitSide, bvh.GetRootNodeID())){
            return true;
        }
        return false;
    }
    virtual Box GetBoundBox() const { return Box(GetBoundMin(),GetBoundMax()); }
    virtual void ViewportDisplay(const Material *mtl) const;
    
    bool Load(const char *filename, bool loadMtl)
    {
        bvh.Clear();
        if ( ! LoadFromFileObj( filename, loadMtl ) ) return false;
        if ( ! HasNormals() ) ComputeNormals();
        ComputeBoundingBox();
        bvh.SetMesh(this,4);
        return true;
    }
    
private:
    cyBVHTriMesh bvh;
    
           
    bool IntersectTriangle( const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID ) const{
        float bias = 1e-7f;//0.001f;
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
        float maxN = fmax(fabs(tNormal.x),fabs(tNormal.y));
        maxN = fmax(fabs(tNormal.z),maxN);
        
        Point3 P = ray.p + t * ray.dir;
        
        Point2 pa,pb,pc,pp;
        if(maxN==fabs(tNormal.x)){
            pa = Point2(A.y, A.z);
            pb = Point2(B.y, B.z);
            pc = Point2(C.y, C.z);
            pp = Point2(P.y, P.z);
        }
        else if(maxN==fabs(tNormal.y)){
            pa = Point2(A.x, A.z);
            pb = Point2(B.x, B.z);
            pc = Point2(C.x, C.z);
            pp = Point2(P.x, P.z);
        }
        else{
            pa = Point2(A.x, A.y);
            pb = Point2(B.x, B.y);
            pc = Point2(C.x, C.y);
            pp = Point2(P.x, P.y);
        }
        
        float area_tri = (pa - pc).Cross(pb - pc);
        float area_bcp = (pp - pc).Cross(pb - pc);
        float area_acp = (pa - pc).Cross(pp - pc);
        float alpha = area_bcp/area_tri;
        float beta = area_acp/area_tri;
        float gamma = 1.0-alpha-beta;
        
        if(alpha<-bias || beta<-bias || gamma<-bias || alpha>1.0 || beta>1.0 || gamma>1.0 ) return false;
        
        Point3 P_Normal = GetNormal(faceID, Point3(alpha,beta,gamma));
        hInfo.front = true;
        hInfo.p = alpha * A + beta * B + gamma * C;
        hInfo.N = P_Normal;
        hInfo.N.Normalize();
        hInfo.z = t;
        hInfo.uvw = GetTexCoord(faceID, Point3(alpha,beta,gamma));
        //calculateduvw(ray,hInfo);
        return true;
    }
           
           
    bool TraceBVHNode( const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID ) const{
        
        Box nodebox = Box(bvh.GetNodeBounds(nodeID));
        bool Hitted = false;
        
        if(!nodebox.IntersectRay(ray, BIGFLOAT)){
            return false;
        }
        else{
            if(bvh.IsLeafNode(nodeID)){
                
                unsigned int* elements = (unsigned int*)bvh.GetNodeElements(nodeID);
                for(int i = 0; i < bvh.GetNodeElementCount(nodeID); i++){
                    if(IntersectTriangle(ray, hInfo, hitSide, elements[i])){
                        Hitted = true;
                    }
                }
                return Hitted;
            }
            
            else{
                if(TraceBVHNode(ray, hInfo, hitSide, bvh.GetFirstChildNode(nodeID))){
                    Hitted = true;
                }
                if(TraceBVHNode(ray, hInfo, hitSide, bvh.GetSecondChildNode(nodeID))){
                    Hitted = true;
                }
            }
            
        }
        return Hitted;
    }
};

//-------------------------------------------------------------------------------

#endif
