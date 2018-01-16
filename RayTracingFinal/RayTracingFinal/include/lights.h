//-------------------------------------------------------------------------------
///
/// \file       lights.h
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    13.0
/// \date       November 20, 2017
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _LIGHTS_H_INCLUDED_
#define _LIGHTS_H_INCLUDED_

#include "scene.h"
#define SHADOW_SAMPLES 4
#define MIN_SHADOW_SAMPLES 4
#define MAX_SHADOW_SAMPLES 16
//-------------------------------------------------------------------------------

class GenLight : public Light
{
protected:
    void SetViewportParam(int lightID, ColorA ambient, ColorA intensity, Point4 pos ) const;
    static float Shadow(Ray ray, float t_max=BIGFLOAT);
};

//-------------------------------------------------------------------------------

class AmbientLight : public GenLight
{
public:
    AmbientLight() : intensity(0,0,0) {}
    virtual Color Illuminate(const Point3 &p, const Point3 &N) const { return intensity; }
    virtual Point3 Direction(const Point3 &p) const { return Point3(0,0,0); }
    virtual bool IsAmbient() const { return true; }
    virtual void SetViewportLight(int lightID) const { SetViewportParam(lightID,ColorA(intensity),ColorA(0.0f),Point4(0,0,0,1)); }
    
    void SetIntensity(Color intens) { intensity=intens; }
private:
    Color intensity;
};

//-------------------------------------------------------------------------------

class DirectLight : public GenLight
{
public:
    DirectLight() : intensity(0,0,0), direction(0,0,1) {}
    virtual Color Illuminate(const Point3 &p, const Point3 &N) const { return Shadow(Ray(p,-direction)) * intensity; }
    virtual Point3 Direction(const Point3 &p) const { return direction; }
    virtual void SetViewportLight(int lightID) const { SetViewportParam(lightID,ColorA(0.0f),ColorA(intensity),Point4(-direction,0.0f)); }
    
    void SetIntensity(Color intens) { intensity=intens; }
    void SetDirection(Point3 dir) { direction=dir.GetNormalized(); }
private:
    Color intensity;
    Point3 direction;
};

//-------------------------------------------------------------------------------

class PointLight : public GenLight
{
public:
    PointLight() : intensity(0,0,0), position(0,0,0), size(0) {}
    Color Illuminate(const Point3 &p, const Point3 &N) const{
        
        //create a ray towards light
        Point3 dir = position - p;
        
        Ray sRay = Ray(p, dir);
        
        Point3 xAxis(1,0,0), yAxis(0,1,0), v1;
        
        if(dir.Dot(xAxis) > 0.8) {
            v1 = yAxis.Cross(dir);
        }
        else{
            v1 = xAxis.Cross(dir);
        }
        
        float shadow = 0.0;
        Point3 v2 = v1.Cross(dir);
        v2.Normalize();
        v1.Normalize();
        Point3 xv1 = v1;
        Point3 yv2 = v2;
        //    srand(time(NULL));
        float random = 0.0;
        
        for(int i =0; i< MIN_SHADOW_SAMPLES; i++){
            random = rand() / (float) RAND_MAX;
            float rRadius = sqrtf(random) * size;
            random = rand() / (float) RAND_MAX;
            float rAngle = random * (2.0 * M_PI);
            float xv = rRadius * cos(rAngle);
            float yv = rRadius * sin(rAngle);
            xv1 *= xv;
            yv2 *= yv;
            
            
            sRay.dir = (position + xv1.Length()+ yv2.Length() ) - p;
            //    sRay.dir.Normalize();
            shadow += Shadow(sRay, 1);
            xv1 = v1; yv2 = v2;
        }
        shadow /= (float)MIN_SHADOW_SAMPLES;
        
        if(shadow != 0.0 && shadow != 1.0)
        {
            shadow = 0.0;
            for(int i = 0; i< MAX_SHADOW_SAMPLES; i++){
                random = rand() / (float) RAND_MAX;
                float rRadius = sqrtf(random) * size;
                random = rand() / (float) RAND_MAX;
                float rAngle = random * (2.0 * M_PI);
                float xv = rRadius * cos(rAngle);
                float yv = rRadius * sin(rAngle);
                xv1 *= -xv;
                yv2 *= -yv;
                
                sRay.dir = (position +xv1.Length() + yv2.Length() ) - p;
                //            sRay.dir.Normalize();
                shadow += Shadow(sRay, 1);
                xv1 = v1; yv2 = v2;
            }
            shadow /= (float)(MAX_SHADOW_SAMPLES);
        }
        return intensity * shadow / (p - position).LengthSquared();

        /*
        float shadow_coef = 0.0;
        for(int i=0;i<SHADOW_SAMPLES;i++){
            float r = Halton(i, 2);
            r = sqrtf(r)*size;
            float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
            float gamma = M_PI * rand()/ (float) RAND_MAX;
            float dx =  r * sinf(gamma) * cosf(theta);
            float dy =  r * sinf(gamma) * sinf(theta);
            float dz =  r * cosf(gamma);
            
            //if(sqrtf(dx*dx+dy*dy+dz*dz)>size)
            //    std::cout << "sample out of range" <<std::endl;
            Point3 newlightPos(dx, dy, dz);
            newlightPos+=position;
            Point3 dir = newlightPos - p;
            //dir.Normalize();
            shadow_coef+=Shadow(Ray(p,dir),1);
        }
        Color avg_shadow = intensity*shadow_coef/SHADOW_SAMPLES;

        //inverse square fall off!
        float distance = (p-position).LengthSquared();
        
        return avg_shadow/distance;*/
    };
    
    virtual Point3 Direction(const Point3 &p) const { return (p-position).GetNormalized(); }
    virtual void SetViewportLight(int lightID) const;
    void SetIntensity(Color intens) { intensity=intens; }
    void SetPosition(Point3 pos) { position=pos; }
    void SetSize(float s) { size=s; }
    
    // Photon Extensions
    virtual bool    IsPhotonSource()        const { return true; }
    virtual Color   GetPhotonIntensity()    const { return intensity; }
    virtual Ray     RandomPhoton()          const;
    
private:
    Color intensity;
    Point3 position;
    float size;
};

//-------------------------------------------------------------------------------

#endif
