
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
/**
 * Thresholds for Reflection and Refraction
 */
const float total_reflection_threshold = 1.001f;
const float refraction_color_threshold = 0.001f;
const float reflection_color_threshold = 0.001f;
const float glossiness_value_threshold = 0.001f;
const float glossiness_power_threshold = 0.f;
const float color_luma_threshold = 0.00001f;
//-------------------------------------------------------------------------------
inline Point3 CosineSampleHemisphere()
{
    float u1 = rand() / (float) RAND_MAX;
    float u2 = rand() / (float) RAND_MAX;
    const float r = sqrtf(u1);
    const float theta = 2 * M_PI * u2;
    
    const float x = r * cosf(theta);
    const float y = r * sinf(theta);
    
    return Point3(x, y, sqrtf(fmax(0.0f, 1 - u1)));
}

inline Point3 SampleHemisphere(){
    float u1 = rand() / (float) RAND_MAX;
    float u2 = rand() / (float) RAND_MAX;
    
    const float r = sqrtf(1.0f - u1 * u1);
    const float phi = 2 * M_PI * u2;
    
    return Point3(cosf(phi) * r, sinf(phi) * r, u1);
}

inline void createCoordinateSystem(const Point3 &N, Point3 &Nt,Point3 &Nb)
{
    Point3 v1(1, 0, 0), v2(0, 0, 1);
    if(N.Dot(v1) < 0.4)
        Nt = N ^ v1;
    else
        Nt = N ^v2;
    Nt.Normalize();
    Nb = N^Nt;
}
inline Color Attenuation(const Color &absorption, float l)
{
    const auto R = exp(-absorption.r * l);
    const auto G = exp(-absorption.g * l);
    const auto B = exp(-absorption.b * l);
    return Color(R, G, B); // attenuation
}

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
        //Color color = emission.GetColor();
        const Point3 V = -r.dir;
        const Point3 N = hInfo.N;
        const Point3 Y = N.Dot(V) > 0.f ? N : -N;
        const Point3 p = hInfo.p;
        
        // Reflection and Transmission
        Point3 tDir; //refraction Dir
        Point3 rDir; //reflection Dir
        
        float ein = 1; float eout = ior; // index
        if(!hInfo.front){
            ein = ior; eout = 1;
        }
        float eta = ein / eout;
        
        const Point3 Z = V.Cross(Y);
        Point3 X = Y.Cross(Z);
        X.Normalize();
        
        float cosI = N.Dot(V);
        float sinI = sqrtf(1 - cosI * cosI);
        float sinO = max(0.f, min(1.f, sinI * eta));
        float cosO = sqrtf(1.f - sinO * sinO);
        
        tDir = -X * sinO - Y * cosO;
        rDir = 2.f * N * (N.Dot(V)) - V;
        
        // reflection and transmission coefficients
        const float C0 = (eta - 1.f) * (eta - 1.f) / ((eta + 1.f) * (eta + 1.f));
        float rC = C0 + (1.f - C0) * pow(1.f - fabsf(cosI), 5.f);
        const float tC = 1.f - rC;
        
        const bool totReflection = (eta * sinI) > total_reflection_threshold;
        //bool totReflection = false;
        const Color tK = refraction.GetColor();
        const Color rK = reflection.GetColor();
        const Color sampleRefraction = totReflection ? Color(0.f) : tK * tC;
        const Color sampleReflection = totReflection ? (rK + tK) : (rK + tK * rC);
        const Color sampleDiffuse = diffuse.GetColor();
        const Color sampleSpecular = specular.GetColor();
        
        // select one property
        Point3 sampleDir;
        Color BxDF;
        float PDF = 1.f;
        float scale = 1.f;
        bool doShade = false;
        int selected;
        
        float random = rand() / (float)RAND_MAX;
        float diffuseProb = sampleDiffuse.Gray();
        float specularProb = sampleSpecular.Gray();
        float refractionProb = sampleRefraction.Gray();
        float reflectionProb = sampleReflection.Gray();
        float absorptionProb = absorption.Gray();
        float total = diffuseProb + reflectionProb + refractionProb + absorptionProb;
        diffuseProb /= total;
        refractionProb /= total;
        reflectionProb /= total;
        absorptionProb /= total;
        
        const float rcpCoefSum = 1.f / total;
        const float select = random * total;
        
        if (select <= refractionProb && refractionProb > color_luma_threshold) {
            //selectedMtl = TRANSMIT;
            selected = 0;
            scale = refractionProb * rcpCoefSum;
        } else if (select > refractionProb && select <= refractionProb + reflectionProb && reflectionProb > color_luma_threshold ) {
            //selectedMtl = REFLECT;
            selected = 1;
            scale = reflectionProb * rcpCoefSum;
        } else if (select > refractionProb + reflectionProb && select < refractionProb + reflectionProb + diffuseProb && diffuseProb > color_luma_threshold) {
            //selectedMtl = DIFFUSE;
            selected = 2;
            scale = diffuseProb * rcpCoefSum;
        } else {
            //selectedMtl = ABSORB;
            selected = 3;
            //scale = coefAbsorb * rcpCoefSum;
        }
        
        switch (selected){
            case(0): // transmission
            {
                if(refractionGlossiness > glossiness_power_threshold){
                    //sampleDir = CosineSampleHemisphere();
                    sampleDir = SampleHemisphere();
                    const Point3 L = sampleDir.GetNormalized();
                    const Point3 H = (V + L).GetNormalized();
                    const float cosVH = max(0.f, V.Dot(H));
                    const float glossiness = pow(cosVH, refractionGlossiness); // My Hack
                    BxDF = sampleRefraction * glossiness; // ==> rho / pi
                    PDF = 1.f;   // ==> cosTheta / pi cos-weighted hemisphere sampling
                } else {
                    sampleDir = tDir;
                    BxDF = sampleRefraction; // ==> reflect all light
                    PDF = 1.f;    // ==> PDF = 1
                }
                doShade = true;
                
            }
                break;
            case(1): // reflection
            {
                if(reflectionGlossiness > glossiness_power_threshold){
                    sampleDir = CosineSampleHemisphere();
                    const Point3 L = sampleDir.GetNormalized();
                    const Point3 H = (V + L).GetNormalized();
                    const float cosNH = max(0.f, N.Dot(H));
                    const float glossiness = pow(cosNH, reflectionGlossiness); // My Hack
                    BxDF = sampleReflection * glossiness; // ==> rho / pi
                    PDF = 1.f;   // ==> cosTheta / pi cos-weighted hemisphere sampling
                } else {
                    sampleDir = rDir;
                    BxDF = sampleReflection;
                    PDF = 1.f;
                }
                doShade = true;
            }
                break;
            case(2): // diffuse
            {
                if(hInfo.front){
                    Point3 Nt, Nb;
                    createCoordinateSystem(N, Nt, Nb);
                    float theta = (rand()/ (float)RAND_MAX) * M_PI_2;
                    float phi = rand()/(float)RAND_MAX * (2.0 * M_PI);
                    sampleDir = Nt * cosf(phi) * sinf(theta) + Nb * sinf(phi) * sinf(theta) + N * cosf(theta);
                    const Point3 L = sampleDir.GetNormalized();
                    const Point3 H = (V + L).GetNormalized();
                    const float cosNH = max(0.f, N.Dot(H));
                    const float gloss = pow(cosNH, glossiness); // My Hack
                    BxDF = sampleDiffuse + sampleSpecular * gloss;
                    PDF = 1.f;   // ==> cosTheta / pi cos-weighted hemisphere sampling
                    doShade = true;
                    
                }
            }
                break;
            case (3):
            {
                doShade = false;
            }
                break;
                
        } // end switch
        
        if (doShade) {
            r = Ray(hInfo.p, sampleDir);
            r.Normalize();
            c = c * BxDF / (PDF * scale);
            if (!hInfo.front) { c *= Attenuation(absorption, hInfo.z); }
            return true;
        }
        return false;
        /*
            float prob_diffuse = diffuse.GetColor().Gray();
            float prob_specular = specular.GetColor().Gray();
            float prob_absorbtion = absorption.Gray();  //end
            float prob_refra = refraction.GetColor().Gray();
        float prob_refle = reflection.GetColor().Gray();
        
        //float prob_sum = prob_diffuse+prob_specular+prob_absorbtion+prob_refra;
        float prob_sum = prob_diffuse+prob_refle+prob_absorbtion+prob_refra;
        prob_diffuse /= prob_sum;
        prob_specular/=prob_sum;
        prob_absorbtion/=prob_sum;
        prob_refra /=prob_sum;
        prob_refle /=prob_sum;
        
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
        //else if(rand_n<=prob_specular+prob_diffuse){
        else if(rand_n<=prob_refle+prob_diffuse){
            
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

            //c = c *specular.GetColor() /prob_specular;
            c = c *reflection.GetColor() /prob_refle;
        }
        //else if(rand_n<=prob_refra+prob_specular+prob_diffuse){
        else if(rand_n<=prob_refra+prob_refle+prob_diffuse){
            Point3 P = hInfo.p;
            Point3 N = hInfo.N;
            Point3 V = -r.dir.GetNormalized();
            //costheta1 may be negative
            float costheta1 = fabsf(V.Dot(N));
            float sintheta1 = sqrtf(max(0.0f,1 - (costheta1 * costheta1)));
            //Point3 Y = costheta1 > 0 ? N : -N;
            
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

                //c = c * refraction.GetColor()/prob_refra;
                c = c * refraction.GetColor()/prob_refra;
             }
            
        }
        else{
            return false;
        }
        
        //c = c * diffuse.GetColor();
        
        return true;*/
         
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
