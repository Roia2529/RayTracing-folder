
//-------------------------------------------------------------------------------
///
/// \file       materials.h
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    13.0
/// \date       November 20, 2015
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _MATERIALS_H_INCLUDED_
#define _MATERIALS_H_INCLUDED_

#include "scene.h"

//-------------------------------------------------------------------------------

class MtlBlinn : public Material
{
public:
    MtlBlinn() : diffuse(0.5f,0.5f,0.5f), specular(0.7f,0.7f,0.7f), glossiness(20.0f), emission(0,0,0),
    reflection(0,0,0), refraction(0,0,0), absorption(0,0,0), ior(1),
    reflectionGlossiness(0), refractionGlossiness(0) {}
    virtual Color Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount, int specount) const;
    
    void SetDiffuse     (Color dif)     { diffuse.SetColor(dif); }
    void SetSpecular    (Color spec)    { specular.SetColor(spec); }
    void SetGlossiness  (float gloss)   { glossiness = gloss; }
    void SetEmission    (Color e)       { emission.SetColor(e); }
    
    void SetReflection  (Color reflect) { reflection.SetColor(reflect); }
    void SetRefraction  (Color refract) { refraction.SetColor(refract); }
    void SetAbsorption  (Color absorp ) { absorption = absorp; }
    void SetRefractionIndex(float _ior) { ior = _ior; }
    
    void SetDiffuseTexture   (TextureMap *map)  { diffuse.SetTexture(map); }
    void SetSpecularTexture  (TextureMap *map)  { specular.SetTexture(map); }
    void SetEmissionTexture  (TextureMap *map)  { emission.SetTexture(map); }
    void SetReflectionTexture(TextureMap *map)  { reflection.SetTexture(map); }
    void SetRefractionTexture(TextureMap *map)  { refraction.SetTexture(map); }
    void SetReflectionGlossiness(float gloss)   { reflectionGlossiness=gloss; }
    void SetRefractionGlossiness(float gloss)   { refractionGlossiness=gloss; }
    
    virtual void SetViewportMaterial(int subMtlID=0) const; // used for OpenGL display
    
    // Photon Extensions
    virtual bool IsPhotonSurface(int subMtlID=0) const { return diffuse.GetColor().Gray() > 0; } // if this method returns true, the photon will be stored
    
    virtual bool RandomPhotonBounce(Ray &r, Color &c, const HitInfo &hInfo) const{
            float prob_diffuse = diffuse.GetColor().Gray();
            float prob_specular = specular.GetColor().Gray();
            float prob_absorbtion = absorption.Gray();  //end
            float prob_refra = refraction.GetColor().Gray();
        
        float prob_sum = prob_diffuse+prob_specular+prob_absorbtion+prob_refra;
        prob_diffuse /= prob_sum;
        prob_specular/=prob_sum;
        prob_absorbtion/=prob_sum;
        prob_refra /=prob_sum;
        
        float bias = 0.0;//0.0001f;
        float rand_n = rand()/ (float) RAND_MAX;
        if(rand_n<=prob_diffuse){
            Point3 newz = hInfo.N;
            Point3 v1(1,0,0),v2(0,0,1);
            Point3 newx;
            
            if(newz.Dot(v1)<0.4)
                newx =newz^v1;
            else
                newx = newz^v2;
            
            newx.Normalize();
            Point3 newy = newz ^ newx;
            
            float phi = 2*M_PI*(rand()/ (float) RAND_MAX);
            float cosphi = cosf(phi);
            float ysquare = (rand()/ (float) RAND_MAX);
            float sintheta = sqrt(ysquare);
            float costheta = sqrt(1-ysquare);
            
            Point3 hemis_dir = sintheta * cosphi * newx + sintheta * sinf(phi) * newy + costheta * newz;
            hemis_dir.Normalize();
            //cout << newz.Length() << endl;
            
            //float dotN_wi = hemis_dir.Dot(newz);
            Ray Ray_idr = Ray(hInfo.p, hemis_dir );
            Ray_idr.dir.Normalize();
            r = Ray_idr;

            c = c * diffuse.GetColor()/prob_diffuse;
        }
        else if(rand_n<=prob_specular+prob_diffuse){
            
            //specular
            Point3 N = hInfo.N;
            Point3 P = hInfo.p;
            Point3 V = -r.dir.GetNormalized();
            float costheta = N.Dot(V);
            if(costheta<-1.0)
                costheta = -1.0;
            else if(costheta>1.0)
                costheta = 1.0;
            
            Point3 R = 2*costheta*N - V; //reflection vector
            //R.Normalize();
            Ray Ray_rf = Ray(P+bias*R, R);
            Ray_rf.dir.Normalize();
            r =Ray_rf;

            c = c *specular.GetColor() /prob_specular;
        }
        else if(rand_n<=prob_refra+prob_specular+prob_diffuse){
            Point3 P = hInfo.p;
            Point3 N = hInfo.N;
            Point3 V = -r.dir.GetNormalized();
            //costheta1 may be negative
            float costheta1 = fabsf(V.Dot(N));
            float sintheta1 = sqrtf(max(0.0f,1 - (costheta1 * costheta1)));
            
            float n1 = 1.0, n2 = 1.0;
            if(hInfo.front){ // out -> in
                n2 = ior;
            }
            else if(!hInfo.front){ //in->out
                n1 = ior;
                N = -N;
            }
            float ratio_n = n1 / n2;
            float sintheta2 = ratio_n*sintheta1;
            
            HitInfo hitinfo_ra;
            hitinfo_ra.Init();
            
            //float absorb = 0.0;
            if(sintheta2 <= 1.0){
                
                float costheta2 = sqrtf(max(0.0f,1 - (sintheta2 * sintheta2)));
                Point3 S = N ^ (N ^ V);
                N.Normalize();
                S.Normalize();
                Point3 T = -N * costheta2 + S * sintheta2;
                
                Ray Ray_rfa = Ray(P+bias*T, T);
                r = Ray_rfa;

                c = c * refraction.GetColor()/prob_refra;
             }
            
        }
        else{
            return false;
        }
        
        //c = c * diffuse.GetColor();
        
        return true;
    }// if this method returns true, a new photon with the given direction and color will be traced
    
private:
    TexturedColor diffuse, specular, reflection, refraction, emission;
    float glossiness;
    Color absorption;
    float ior;  // index of refraction
    float reflectionGlossiness, refractionGlossiness;
    
};

//-------------------------------------------------------------------------------

class MultiMtl : public Material
{
public:
    virtual ~MultiMtl() { for ( unsigned int i=0; i<mtls.size(); i++ ) delete mtls[i]; }
    
    virtual Color Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount,int scount) const { return hInfo.mtlID<(int)mtls.size() ? mtls[hInfo.mtlID]->Shade(ray,hInfo,lights,bounceCount,0) : Color(1,1,1); }
    
    virtual void SetViewportMaterial(int subMtlID=0) const { if ( subMtlID<(int)mtls.size() ) mtls[subMtlID]->SetViewportMaterial(); }
    
    void AppendMaterial(Material *m) { mtls.push_back(m); }
    
    // Photon Extensions
    virtual bool IsPhotonSurface(int subMtlID=0) const { return mtls[subMtlID]->IsPhotonSurface(); }
    virtual bool RandomPhotonBounce(Ray &r, Color &c, const HitInfo &hInfo) const { return hInfo.mtlID<(int)mtls.size() ? mtls[hInfo.mtlID]->RandomPhotonBounce(r,c,hInfo) : false; }
    
private:
    std::vector<Material*> mtls;
};

//-------------------------------------------------------------------------------

#endif
