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
        
        return avg_shadow/distance;
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
