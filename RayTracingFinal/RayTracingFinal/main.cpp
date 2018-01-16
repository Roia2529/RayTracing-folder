//
//  main.cpp
//  RayTracingP13 Photon Mapping
//
//  Created by Hsuan Lee on 12/3/17.
//  Copyright © 2017 Hsuan Lee. All rights reserved.
//

//#include "cyIrradianceMap.h"
#include <iostream>
#include <thread>
#include "viewport.cpp"
#include "xmlload.cpp"
#include "objects.h"
#include <math.h>
#include <algorithm> 
#include "cyPhotonMap.h"

#define CAM_SAMPLE 64
#define MIN_SAMPLE 4
#define MAX_SAMPLE 8
#define HALTON_BASE_1 2
#define HALTON_BASE_2 3
#define THRESHOLD 1e-3f
#define BOUNCE 4
#define HEMISPHERE_SAMPLE 30
#define MAX_NUM_OF_PHOTON 1000000
#define MAX_NUM_OF_CAUSTIC_PHOTON 1000000
#define PHOTON_BOUNCE 8
#define CAUSTIC_PHOTON_BOUNCE 5

#define gamma 2.2

using namespace std;

Node rootNode;
Camera camera;
RenderImage renderImage;
Sphere theSphere;
Plane thePlane;
MaterialList materials;
LightList lights;
ObjFileList objList;
TexturedColor environment;
TexturedColor background;
TextureList textureList;

cy::PhotonMap photonmap;
cy::PhotonMap causticmap;

/**
 * Thresholds for Reflection and Refraction
 */
/*
const float total_reflection_threshold = 1.001f;
const float refraction_color_threshold = 0.001f;
const float reflection_color_threshold = 0.001f;
const float glossiness_value_threshold = 0.001f;
const float glossiness_power_threshold = 0.f;
*/

//if not hit
Color black(0.0);

class pixelIterator{
private:
    atomic<int> ix;
    atomic<bool> stop;
public:
    void Init(){ ix = 0; stop = false;}
    bool GetPixel(int &x, int &y){
        if(stop) return false;
        int i=ix++;
        if(i>=camera.imgWidth*camera.imgHeight) return false;
        x = i%camera.imgWidth;
        y = i/camera.imgWidth;
        return true;
    }
    void setFlag(){
        stop = true;
    }
    void clearFlag(){
        stop = false;
    }
};
//multi-thread
pixelIterator pIt;
void PhotonTracing(Ray rayPre,HitInfo hitInfo,Color intensity,int &savedPhoton,int bounce);
void CausticTracing(Ray rayPre,HitInfo hitInfo,Color intensity,int &savedPhoton,int bounce,int hitspec);

bool Trace(const Ray &ray, HitInfo &hitinfo);
bool TraceNode(const Node &node, const Ray &ray, HitInfo &hitinfo);

bool Trace(const Ray &ray, HitInfo &hitinfo){
    return TraceNode(rootNode, ray, hitinfo);
}

float clamp( float v, const float lo, const float hi)
{
    if(v<lo)
        return lo;
    else if(v>hi)
        return hi;
    else
        return v;
}

bool TraceNode(const Node &node,const Ray &ray, HitInfo &hitinfo){
    //world space to model space
    Ray r = node.ToNodeCoords(ray);
    const Object *obj = node.GetNodeObj();
    bool hit = false;
    
    if(obj){
        if(obj->IntersectRay(r,hitinfo)){
            hit = true;
            hitinfo.node = &node;
            node.FromNodeCoords(hitinfo);
        }
    }
    
    for(int i=0;i<node.GetNumChild();i++){
        const Node &child = *node.GetChild(i);
        if(TraceNode(child,r,hitinfo)){
            node.FromNodeCoords(hitinfo);
            hit = true;
        }
    }
    return hit;
}

void setPixelColor(int index, float z, Color shade){
    Color24* color_pixel = renderImage.GetPixels();
    color_pixel[index]=(Color24)shade;
    
    float* zb_pixel = renderImage.GetZBuffer();
    *(zb_pixel+ index) = z;
    
}
void setPixelSampleCount(int index, float count){

    uchar* zb_pixel = renderImage.GetSampleCount();
    *(zb_pixel+ index) = count;
    
}

void generateSample(vector<Point3> &samplelist, int s, int e, const Point3 base, const float u, const float v){

    for(int j=s;j<e;j++){

        //get shift from Halton
        //float sx = 0;
        float sx = Halton(j,HALTON_BASE_1)*u;
        float sy = v * Halton(j,HALTON_BASE_2);
       
        sx += base.x;
        sy += base.y;
        
        Point3 sample = Point3(sx,sy,base.z);
        samplelist.push_back(sample);
    }
}

bool VariantOverThreshold(vector<Color> list){
    //return false;
    float ninverse = 1.0/list.size();
    float rsum = 0, gsum = 0, bsum = 0;
    float rsquare_sum = 0, gsquare_sum = 0, bsquare_sum = 0;
    for(int i=0;i<list.size();i++){
        Color tmpColor = list.at(i);
        rsum+=tmpColor.r;
        gsum+=tmpColor.g;
        bsum+=tmpColor.b;
        
        rsquare_sum+=pow(tmpColor.r,2);
        gsquare_sum+=pow(tmpColor.g,2);
        bsquare_sum+=pow(tmpColor.b,2);
    }
    
    float ravg = ninverse*rsum;
    float gavg = ninverse*gsum;
    float bavg = ninverse*bsum;
    
    float rvariance = rsquare_sum*ninverse + pow(ravg, 2) - 2*ravg*ninverse*rsum;
    float gvariance = gsquare_sum*ninverse + pow(gavg, 2) - 2*gavg*ninverse*gsum;
    float bvariance = bsquare_sum*ninverse + pow(bavg, 2) - 2*bavg*ninverse*bsum;
    
    return (rvariance>THRESHOLD) || (gvariance>THRESHOLD) || (bvariance>THRESHOLD);
}

Color averageColor(vector<Color> list){
    int size = (int)list.size();
    float n = 1/(float)size;
    Color c(0,0,0);
    for(int i=0; i<size; i++){
        c += list.at(i)*n;
    }
    return c;
}


void RenderPixel(pixelIterator &it){
    int x,y;
    
    float theta = camera.fov;
    //float l = 1.0;
    float l = camera.focaldist;
    float h = 2*l*tan(theta/2*(M_PI/180));
    float w = h*(float)camera.imgWidth/camera.imgHeight;
    Point3 b;
    b.Set(-w/2,h/2,-l);
    float u = w/camera.imgWidth;
    float v = -h/camera.imgHeight;
    float du = u/2, dv = v/2;
    
    b.x+=du;
    b.y+=dv;
    
    Point3 z_new = -1*(camera.dir);
    Point3 x_new = camera.up ^ z_new;
    camera.up.Normalize();
    z_new.Normalize();
    x_new.Normalize();
    Matrix3 m(x_new,camera.up, z_new);

    int bouncelimit = BOUNCE;
    
    while(it.GetPixel(x,y)){
        //debug
        //if (y<300) continue;
        if (x==312 && y == 346) {
            std::cout << "xx" << std::endl;
        }
        
        Point3 tmp(x*u,y*v,0);
        tmp+=b;
        
        HitInfo hitinfo;
        hitinfo.Init();
        bool hit = false;
        
        int index = y*camera.imgWidth+x;
        
        //Prepare 64 camera pos samples
        vector<Point3> camposSample;
        if(camera.dof){
            for(int i = 1; i<=CAM_SAMPLE; i++){
            float r = Halton(i, HALTON_BASE_1);
            
            r = sqrtf(r)*camera.dof;
            float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
            
            float dx =  r * cosf(theta);
            float dy =  r * sinf(theta);
            
            //if(sqrtf(dx*dx+dy*dy)>camera.dof)
            //   std::cout << "sample out of range" <<std::endl;
            Point3 newCamPos(dx, dy, 0);
            newCamPos = m * newCamPos;
            camposSample.push_back(newCamPos);
            }   
        }
        
        
        float hitz = 0;
        if(rootNode.GetNumChild()>0){
            vector<Color> colorlist;
            vector<Point3> samplelist;
            int s_start = 0;
            int s_end = MIN_SAMPLE;
            

            while(s_start==0 || (VariantOverThreshold(colorlist) && s_start!=MAX_SAMPLE)){
                //initial: 4 sample
                generateSample(samplelist, s_start, s_end, tmp, u, v);

                for(int k=s_start;k<s_end;k++){
                    
                    hitinfo.Init();

                    Point3 sampleloc = samplelist.at(k);
                    Point3 d_campos;
                    if(camera.dof)
                        d_campos = camposSample.at(rand()%CAM_SAMPLE);
                    else
                        d_campos = Point3(0,0,0);
                    
                    Ray ray_pixel(camera.pos+d_campos,sampleloc);
                    ray_pixel.dir=m*(ray_pixel.dir);
                    
                    ray_pixel.dir -= d_campos;
                    ray_pixel.dir.Normalize();

                    if(Trace(ray_pixel,hitinfo)){
                        //set color
                        hit = true;
                        colorlist.push_back(hitinfo.node->GetMaterial()->Shade(ray_pixel, hitinfo, lights,bouncelimit,0));
                        
                        hitz =hitinfo.z;
                    }//end if
                }//end for
                
                s_start=s_end; //s_end*=4;
                s_end = MAX_SAMPLE;
                if(!hit) break;
                
            }//end of while

            if(hit){
                //set average color in colorlist
                Color avgColor = averageColor(colorlist);
                if(colorlist.size()<=MIN_SAMPLE)
                    setPixelSampleCount(index,0);
                else
                    setPixelSampleCount(index,255);
                
                //gamma
                float r = powf(avgColor.r, 1.0/gamma);
                float g = powf(avgColor.g, 1.0/gamma);
                float b = powf(avgColor.b, 1.0/gamma);
                
                Color gammaCorrect(r,g,b);
                setPixelColor(index,hitz,gammaCorrect);
            }
            else{
                Point3 uvw = Point3((float)x/camera.imgWidth,(float)y/camera.imgHeight,0);
                
                Color avgColor = background.Sample(uvw);
                float r = powf(avgColor.r, 1.0/gamma);
                float g = powf(avgColor.g, 1.0/gamma);
                float b = powf(avgColor.b, 1.0/gamma);
                
                Color gammaCorrect(r,g,b);
                setPixelColor(index,BIGFLOAT,gammaCorrect);
                
                
                setPixelSampleCount(index,0);
            }
            renderImage.IncrementNumRenderPixel(1);
            //renderImage.IncrementNumRenderPixel(800*600);
        }
        
    }
}
void lightlist(){
    for(int i=0;i<lights.size();i++){
        
    }
}
void generatePhotonMap(){
    causticmap.AllocatePhotons(MAX_NUM_OF_PHOTON);
    photonmap.AllocatePhotons(MAX_NUM_OF_PHOTON);
    
    vector<Light *> photonLights;
    for(int i=0;i<lights.size();i++){
        if(!lights[i]->IsAmbient()){
            photonLights.push_back(lights[i]);
        }
    }
    int numofphoton = 0;
    //choose one light
    while(numofphoton < MAX_NUM_OF_PHOTON){
        //PointLight *_curLight = (PointLight *)(list_PointLights.at(0));
        //randomly choose light/PointLight?
        PointLight* pL;
        
        if((rand()/ (float) RAND_MAX)<0.5)
            pL = (PointLight*)( photonLights.at(0));
        else
            pL = (PointLight*)( photonLights.at(1));
        
        //pL = (PointLight*)( photonLights.at(0));
        
        Color lightColor = pL->GetPhotonIntensity();
        Ray rayFromL = pL->RandomPhoton();
        HitInfo hitInfo;
        hitInfo.Init();
        if(TraceNode(rootNode,rayFromL,hitInfo)){
            const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(hitInfo.node->GetMaterial());
            //if we hit specular in the beginning, we will not continue
            if(mtlb->IsPhotonSurface()){
                //save first hit when we want to use photon map for direct illumination
                /*
                Point3 pos = hitInfo.p;
                Point3 dir = rayFromL.dir;
                Color power = lightColor;
                photonmap.AddPhoton(pos, dir, power);
                */
                
                PhotonTracing(rayFromL, hitInfo, lightColor, numofphoton,PHOTON_BOUNCE);
                
                //CausticTracing(rayFromL, hitInfo, lightColor, numofphoton,PHOTON_BOUNCE, 0);
            }
        }//end of TraceNode
    }
    photonmap.ScalePhotonPowers(1.0*4*M_PI/photonmap.NumPhotons());
    cout<<"saving photon map\n";
    FILE *fp=fopen("/Users/hsuanlee/Documents/Cpp/RayTracing/PhotonMap/PhotonMap/photonmap.dat","wb");
    fwrite(photonmap.GetPhotons(),sizeof(cyPhotonMap::Photon),photonmap.NumPhotons(),fp);
    fclose(fp);
    
    photonmap.PrepareForIrradianceEstimation();

    //causticmap.AllocatePhotons(MAX_NUM_OF_CAUSTIC_PHOTON);
    
    int numofcausphoton = 0;
    //assume always choose first light
    /*
    while(numofcausphoton < MAX_NUM_OF_CAUSTIC_PHOTON){
        //numofcausphoton choose light/PointLight?
        const PointLight* pL = static_cast<const PointLight*>( lights[0]);
        Color lightColor = pL->GetPhotonIntensity();
        Ray rayFromL = pL->RandomPhoton();
        HitInfo hitInfo;
        hitInfo.Init();
        if(TraceNode(rootNode,rayFromL,hitInfo)){
            const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(hitInfo.node->GetMaterial());
            if(mtlb->IsPhotonSurface()){
                CausticTracing(rayFromL, hitInfo, lightColor, numofcausphoton,CAUSTIC_PHOTON_BOUNCE, 0);
            }
            else{
                CausticTracing(rayFromL, hitInfo, lightColor, numofcausphoton,CAUSTIC_PHOTON_BOUNCE, 1);
            }
            
        }//end of TraceNode
    }
    
    causticmap.ScalePhotonPowers(1.0*4*M_PI/causticmap.NumPhotons());
    cout<<"saving caustic photon \n";
    FILE *fpc=fopen("/Users/hsuanlee/Documents/Cpp/RayTracing/PhotonMap/PhotonMap/causticmap.dat","wb");
    fwrite(causticmap.GetPhotons(),sizeof(cyPhotonMap::Photon),causticmap.NumPhotons(),fpc);
    fclose(fpc);
    
    causticmap.PrepareForIrradianceEstimation();
     */
    
}

void PhotonTracing(Ray rayPre,HitInfo hitInfo,Color intensity,int &savedPhoton,int bounce){
    const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(hitInfo.node->GetMaterial());
    if(bounce > 0 && mtlb->RandomPhotonBounce(rayPre, intensity, hitInfo)){
        //rayPre and intensity are changed.
        HitInfo newhitInfo;
        newhitInfo.Init();
        if(Trace(rayPre, newhitInfo)){
            Point3 pos = newhitInfo.p;
            Point3 dir = rayPre.dir;
            Color power(intensity.r,intensity.g,intensity.b);
            
            const MtlBlinn *mtlb = static_cast<const MtlBlinn *>(newhitInfo.node->GetMaterial());
            if(mtlb->IsPhotonSurface()){
                photonmap.AddPhoton(pos, dir, power);
                savedPhoton++;
            }
            bounce--;
            PhotonTracing(rayPre, newhitInfo, intensity, savedPhoton,bounce);
        }
    }
}

void CausticTracing(Ray rayPre,HitInfo hitInfo,Color intensity,int &savedPhoton,int bounce, int hitspec){
    const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(hitInfo.node->GetMaterial());
    if(bounce > 0 && mtlb->RandomPhotonBounce(rayPre, intensity, hitInfo)){
        //rayPre and intensity are changed.
        HitInfo newhitInfo;
        newhitInfo.Init();
        if(Trace(rayPre, newhitInfo)){
            Point3 pos = newhitInfo.p;
            Point3 dir = rayPre.dir;
            Color power = intensity;
            
            const MtlBlinn *mtlb = static_cast<const MtlBlinn *>(newhitInfo.node->GetMaterial());
            if(mtlb->IsPhotonSurface() ){
                //photonmap.AddPhoton(pos, dir, power);
                if(hitspec>1)
                    causticmap.AddPhoton(pos, dir, power);
                savedPhoton++;
                //return;
            }
            else{
                hitspec++;
            }
            bounce--;
            CausticTracing(rayPre, newhitInfo, intensity, savedPhoton,bounce,hitspec);
        }
    }
}

Ray PointLight::RandomPhoton() const{
    float x = 2*(rand()/ (float) RAND_MAX)-1;
    float y = 2*(rand()/ (float) RAND_MAX)-1;
    float z = 2*(rand()/ (float) RAND_MAX)-1;
    Point3 p(x,y,z);
    Point3 dir = Direction(p+position);
    Ray Ray_fl(position,dir);
    return Ray_fl;
}

float GenLight::Shadow(Ray ray, float t_max){
    //bias
    float bias = 1e-14f;
    
    HitInfo hitInfo;

    hitInfo.Init();
    if(TraceNode(rootNode,ray,hitInfo))
    {
        if(hitInfo.z>bias && hitInfo.z < t_max){
            return 0.0;
        }
    }
    return 1.0;
}


Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount, int specount) const{
    Color color = emission.GetColor();
    
    const Node *node = hInfo.node;
    const Point3 p = hInfo.p; // hit point
    Point3 N = hInfo.N; // normal
    N.Normalize();
    Point3 direction = -ray.dir;
    direction.Normalize();
    
    // get material
    const Material *material = node -> GetMaterial();
    // cast to MtlBlinn
    const MtlBlinn *mtl = (MtlBlinn*) material;
    // get diffuse, specular and glossiness
    Color kd = mtl -> diffuse.Sample(hInfo.uvw, hInfo.duvw);
    Color ks = mtl -> specular.Sample(hInfo.uvw , hInfo.duvw);
    float gloss = mtl -> glossiness;
    // get reflection refraction
    Color reflection = mtl -> reflection.GetColor();
    Color refraction = mtl -> refraction.GetColor();
    // get ior absorption
    float ior = mtl -> ior;
    Color absorption = mtl -> absorption;
    float reflectionGlossiness = mtl -> reflectionGlossiness;
    float refractionGlossiness = mtl -> refractionGlossiness;
    Color intensity;
    
    // direct lights
    const float coef = lights.empty() ? 1.0f : 1.0f / lights.size();
    //for( int i = 0; i < lights.size(); i++){
    for(auto &light : lights){
        //Light *curlight = (GenLight*)lights[i];
        
        //intensity = curlight -> Illuminate(p, N);
        intensity = light -> Illuminate(p, N) * coef;
        
        if(hInfo.front){
            if(!light -> IsAmbient()){
                
                Point3 L  = (float)(-1) * light -> Direction(p);
                L.Normalize();
                Point3 H = (L + direction);
                H.Normalize();
                auto cosNL = max(0.f, N.Dot(L));
                auto cosNH = max(0.f, N.Dot(H));
                
                Color diffuse = kd * intensity * cosNL;
                Color specular = ks * intensity * pow(cosNH, gloss) * cosNL;
                
                color += diffuse + specular;
            }else{
                intensity = light -> Illuminate(p, N);
                color = color + kd * intensity;
            }
        } // end hit front
    } // end go through light
    
    /**
     * Reflection and Refraction
     */
    Point3 tDir; //refraction Dir
    Point3 rDir; //reflection Dir
    
    float ein = 1; float eout = ior; // index
    if(!hInfo.front){
        ein = ior; eout = 1;
    }
    float eta = ein / eout;
    
    Point3 Y = N.Dot(direction) > 0.f ? N : -N;
    const Point3 Z = direction.Cross(Y);
    Point3 X = Y.Cross(Z);
    X.Normalize();
    
    float cosI = N.Dot(direction);
    float sinI = sqrtf(1 - cosI * cosI);
    float sinO = max(0.f, min(1.f, sinI * eta));
    float cosO = sqrtf(1.f - sinO * sinO);
    
    tDir = -X * sinO - Y * cosO;
    rDir = 2.f * N * (N.Dot(direction)) - direction;
    
    // reflection and transmission coefficients
    const float C0 = (eta - 1.f) * (eta - 1.f) / ((eta + 1.f) * (eta + 1.f));
    float rC = C0 + (1.f - C0) * pow(1.f - fabsf(cosI), 5.f);
    //if(sinO >= 1) rC = 1;
    const float tC = 1.f - rC;
    
    const bool totReflection = (eta * sinI) > total_reflection_threshold;
    //bool totReflection = false;
    const Color sampleRefraction = refraction;
    const Color sampleReflection = reflection;
    const Color tK = totReflection ? Color(0.f) : sampleRefraction * tC;
    const Color rK = totReflection ? (sampleReflection + sampleRefraction) : (sampleReflection + sampleRefraction * rC);
    
    // reflection
    if(bounceCount > 0 && (rK.r > reflection_color_threshold || rK.g > reflection_color_threshold || rK.b > reflection_color_threshold)){
        Ray r(p, rDir);
        r.Normalize();
        HitInfo h;
        h.z = BIGFLOAT;
        if(Trace(r, h)){
            const auto *rMtl = h.node->GetMaterial();
            const auto K = rK * (h.front ? Color(1.f) : Attenuation(absorption, h.z));
            color += K * rMtl -> Shade(r, h, lights, bounceCount - 1,1); // reflection
        }
    }
    
    if(bounceCount > 0 && (tK.r > refraction_color_threshold || tK.g > refraction_color_threshold || tK.b > refraction_color_threshold)){
        Ray tRay(p, tDir);
        tRay.Normalize();
        HitInfo tHit;
        tHit.z = BIGFLOAT;
        if(Trace(tRay, tHit)){
            const auto *tMtl = tHit.node->GetMaterial();
            const auto K = tK * (tHit.front ? Color(1.f) : Attenuation(absorption, tHit.z));
            color += K * tMtl -> Shade(tRay, tHit, lights, bounceCount - 1,1);
        }else{
            color += tK * environment.SampleEnvironment(tRay.dir);
        }
        
    }
     Color idr_Color(0,0,0);//indirect color
        //indirect light
        //Path Tracing
        if(bounceCount == BOUNCE){
            Point3 newz = hInfo.N;
            Point3 v1(1,0,0),v2(0,0,1);
            //Point3 newx(newz.z,0,-newz.x);
            Point3 newx;
    
            if(newz.Dot(v1)<0.4)
                newx =newz^v1;
            else
                newx = newz^v2;
    
            newx.Normalize();
            Point3 newy = newz ^ newx;
            int Nofsample= HEMISPHERE_SAMPLE;
    
            for(int i=0;i<Nofsample;i++){
                float phi = 2*M_PI*(rand()/ (float) RAND_MAX);
                float cosphi = cosf(phi);
                float ysquare = (rand()/ (float) RAND_MAX);
                float sintheta = sqrt(ysquare);
                float costheta = sqrt(1-ysquare);
    
                Point3 hemis_dir = sintheta * cosphi * newx + sintheta * sinf(phi) * newy + costheta * newz;
                hemis_dir.Normalize();
    
                float dotN_wi = hemis_dir.Dot(newz);
                Color idrColor(0,0,0);
                if(dotN_wi<0.0) dotN_wi = 0.0;
                else{
                Ray Ray_idr = Ray(p, hemis_dir );
                Ray_idr.dir.Normalize();
                HitInfo hitinfo_idr;
                hitinfo_idr.Init();
    
                Color idrColor;
                if(TraceNode(rootNode,Ray_idr,hitinfo_idr)){
                    idrColor = hitinfo_idr.node->GetMaterial()->Shade(Ray_idr, hitinfo_idr, lights, bounceCount-1,1);
                }
                else{
                    idrColor = environment.SampleEnvironment(Ray_idr.dir);
                }
                }
                //
                idrColor = idrColor* dotN_wi * kd;
    
                //idrColor.r = max(0.0f,min(1.0f,idrColor.r));
                //idrColor.g = max(0.0f,min(1.0f,idrColor.g));
                //idrColor.b = max(0.0f,min(1.0f,idrColor.b));
    
                idr_Color+= 1.0 * idrColor/(float)Nofsample;
            }
        }
        else{
            Color photonrad;
            Point3 dir;
            float radius = 1;
            //Point3 N = hInfo.N;
            photonmap.EstimateIrradiance<400>(photonrad,dir,radius,hInfo.p, &N ,1.f,cy::PhotonMap::FilterType::FILTER_TYPE_CONSTANT);
            
            float theta = N.Dot(-dir);
            theta = (theta>0.0?theta:0.0);
            
            idr_Color += kd * photonrad * theta;
        }
    color+=idr_Color;
    return color;
    
//    Color ra_color(0.0);
//    Color re_color(0.0);
//    Color re_ra_color(0.0);
//
//    Color ambient_color(0.0);
//    Color diffuse_color(0.0);
//
//    Color all;
//    float bias = 0.0;//0.0001f;
//
//    Point3 N = hInfo.N;
//    Point3 P = hInfo.p;
//
//    const Material *material;
//    material = hInfo.node->GetMaterial();
//
//    const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(material);
//    Color Kd = mtlb->diffuse.Sample(hInfo.uvw, hInfo.duvw);
//    Color Ks = mtlb->specular.Sample(hInfo.uvw, hInfo.duvw);
//    float alpha = mtlb->glossiness;
//
//    
//    Color idr_Color(0,0,0);//indirect color
//    Color cau_Color(0,0,0);//indirect color
//    
//    for(int i=0; i<lights.size(); i++){
//        //ambient light
//        if(!hInfo.front) break; 
//        if(lights[i]->IsAmbient()){
//           ambient_color += lights[i]->Illuminate(P,N)*Kd;
//        }
//        else{
//            
//            //direct
//            
//            Color I_i = lights[i]->Illuminate(P,N);
//            Point3 L = -1*(lights[i]->Direction(P));
//            L.Normalize();
//            Point3 V = -ray.dir;
//            V.Normalize();
//            Point3 LpV = L+V;
//            Point3 H = LpV;
//            H.Normalize();
//            float cosNH = N.Dot(H);
//            if(cosNH < 0.0) cosNH = 0.0;
//            
//            Color kse = Ks*pow(cosNH,alpha)+Kd;
//            
//            float theta = N.Dot(L);
//            
//            diffuse_color += I_i*(theta>0.0?theta:0.0)*kse;
//
//        }
//    }
//    //indirect light
//    //Path Tracing
////    if(bounceCount == BOUNCE){
////        Point3 newz = hInfo.N;
////        Point3 v1(1,0,0),v2(0,0,1);
////        //Point3 newx(newz.z,0,-newz.x);
////        Point3 newx;
////        
////        if(newz.Dot(v1)<0.4)
////            newx =newz^v1;
////        else
////            newx = newz^v2;
////        
////        newx.Normalize();
////        Point3 newy = newz ^ newx;
////        int Nofsample= 1;
////        
////        for(int i=0;i<Nofsample;i++){
////            float phi = 2*M_PI*(rand()/ (float) RAND_MAX);
////            float cosphi = cosf(phi);
////            float ysquare = (rand()/ (float) RAND_MAX);
////            float sintheta = sqrt(ysquare);
////            float costheta = sqrt(1-ysquare);
////            
////            Point3 hemis_dir = sintheta * cosphi * newx + sintheta * sinf(phi) * newy + costheta * newz;
////            hemis_dir.Normalize();
////            
////            float dotN_wi = hemis_dir.Dot(newz);
////            if(dotN_wi<0.0) break;
////            
////            Ray Ray_idr = Ray(P, hemis_dir );
////            Ray_idr.dir.Normalize();
////            HitInfo hitinfo_idr;
////            hitinfo_idr.Init();
////            
////            Color idrColor;
////            if(TraceNode(rootNode,Ray_idr,hitinfo_idr)){
////                idrColor = hitinfo_idr.node->GetMaterial()->Shade(Ray_idr, hitinfo_idr, lights, bounceCount-1,1);
////            }
////            else{
////                idrColor = environment.SampleEnvironment(Ray_idr.dir);
////            }
////            
////            //
////            idrColor = idrColor* dotN_wi * Kd;
////            
////            //idrColor.r = max(0.0f,min(1.0f,idrColor.r));
////            //idrColor.g = max(0.0f,min(1.0f,idrColor.g));
////            //idrColor.b = max(0.0f,min(1.0f,idrColor.b));
////            
////            idr_Color+= 1.0 * idrColor ;///(float)Nofsample;
////        }
////    }
////    else{
////        Color photonrad;
////        Point3 dir;
////        float radius = 2;
////        Point3 N = hInfo.N;
////        photonmap.EstimateIrradiance<200>(photonrad,dir,radius,hInfo.p, &N ,1.f,cy::PhotonMap::FilterType::FILTER_TYPE_CONSTANT);
////        
////        float theta = N.Dot(-dir);
////        theta = (theta>0.0?theta:0.0);
////        
////        idr_Color += Kd * photonrad * theta;
////    }
//    //*/
//    //all = ambient_color + ((diffuse_color/M_PI)+idr_Color)*Kd;
//  
//    all = ambient_color + ((diffuse_color/M_PI)+idr_Color);
//    
//    /**
//     ** coefficient calculation
//     **/
//    float R0 = 0.0f, re_ratio = 0.0f, ra_ratio = 0.0f;
//    Point3 V = -ray.dir.GetNormalized();
//    
//    float n1 = 1.0, n2 = 1.0;
//    if(hInfo.front){ // out -> in
//        n2 = ior;
//    }
//    else if(!hInfo.front){ //in->out
//        n1 = ior;
//        //newN_rfa = -newN_rfa;
//    }
//    const Point3 Y = V.Dot(N) > 0.f ? N : -N;
//    
//    float ratio_n = n1 / n2;
//    float costheta1 = fabsf(V.Dot(N));
//    float sintheta1 = 1.f - (costheta1 * costheta1);
//    float sintheta2 = max(0.f, min(1.f, ratio_n*sintheta1));
//    
//    bool totalreflect = (sintheta2>=1.0f);
//    
//    if(!totalreflect){
//        //Shlick's approximation
//        R0 = (n1 - n2)/(n1 + n2);
//        R0 = R0*R0;
//        double  tmp = 1.0 - costheta1;
//        re_ratio = R0 + (1.0 - R0) * pow(tmp,5.0);
//        ra_ratio = 1.0 - re_ratio;
//    }
//    
//    Point3 newN_refl=hInfo.N;
//    if(reflectionGlossiness){
//        Point3 newx(1,0,0),newxalter(0,1,0);
//        //Point3 newx;
//        if(newx.Dot(newN_refl)<0.4)
//            newx =newN_refl^newx;
//        else
//            newx = newN_refl^newxalter;
//        
//        Point3 newy = newN_refl ^ newx;
//        float r = rand()/ (float) RAND_MAX;
//        r = sqrtf(r)*reflectionGlossiness;
//        float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
//        float dx =  r * cosf(theta);
//        float dy =  r * sinf(theta);
//        
//        newx.Normalize();
//        newy.Normalize();
//        newN_refl += dx*newx+dy*newy;
//        
//        newN_refl.Normalize();
//    }
//    
//    const Color tK = totalreflect ? Color(0.f) : refraction.GetColor() * ra_ratio;
//    const Color rK = totalreflect ? (reflection.GetColor() + refraction.GetColor()) : (reflection.GetColor() + refraction.GetColor() * re_ratio);
//
//    /**
//     ** reflection calculation
//    **/
//    
//    //bool refhit = false;
//    
//    if(bounceCount >0){
//        float costheta = clamp(newN_refl.Dot(V),-1.0,1.0);
//        Point3 R = 2*costheta*newN_refl - V; //reflection vector
//        Ray Ray_rf = Ray(P+bias*R, R);
//        Ray_rf.dir.Normalize();
//        HitInfo hitinfo_rf;
//        hitinfo_rf.Init();
//        
//        if(TraceNode(rootNode,Ray_rf,hitinfo_rf))
//        {
//            re_color = hitinfo_rf.node->GetMaterial()->Shade(Ray_rf, hitinfo_rf, lights, bounceCount - 1,specount);
//            
//            //absorption
//            if(!hitinfo_rf.front){
//                re_color.r *= exp(-absorption.r * hitinfo_rf.z);
//                re_color.g *= exp(-absorption.g * hitinfo_rf.z);
//                re_color.b *= exp(-absorption.b * hitinfo_rf.z);
//            }
//            all+= rK * re_color;
//        }
//        //else{
//        //    re_color = environment.SampleEnvironment(Ray_rf.dir);
//        //}
//        
//    }
//
//    /**
//     ** refraction calculation
//    **/
//    //V = -ray.dir.GetNormalized();
//     if(  bounceCount > 0 && !totalreflect){
//         
////         Point3 newN_rfa = hInfo.N;
////         
////         if(refractionGlossiness){
////             
////             Point3 newx(1,0,0),newxalter(0,1,0);
////             //Point3 newx;
////             
////             if(newx.Dot(newN_rfa)<0.4)
////                 newx =newN_rfa^newx;
////             else
////                 newx = newN_rfa^newxalter;
////             
////             Point3 newy = newN_rfa ^ newx;
////             float r = rand()/ (float) RAND_MAX;
////             r = sqrtf(r)*refractionGlossiness;
////             float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
////             float dx =  r * cosf(theta);
////             float dy =  r * sinf(theta);
////             
////             newx.Normalize();
////             newy.Normalize();
////             newN_rfa += dx*newx+dy*newy;
////             
////             newN_rfa.Normalize();
////         }
//         
//         HitInfo hitinfo_ra;
//         hitinfo_ra.Init();
//         
//             float costheta2 = sqrtf(1 - (sintheta2 * sintheta2));
//             Point3 S = N ^ (N ^ V);
//             //Y.Normalize();
//             Point3 T = -Y * costheta2 + S * sintheta2;
//             T.Normalize();
//             Ray Ray_rfa = Ray(P+bias*T, T);
//             
//             if(TraceNode(rootNode,Ray_rfa,hitinfo_ra)){
//                 ra_color  =  hitinfo_ra.node->GetMaterial()->Shade(Ray_rfa,hitinfo_ra,lights,bounceCount-1,1);
//                 if(!hitinfo_ra.front){
//                     ra_color.r *= exp(-absorption.r * hitinfo_ra.z);
//                     ra_color.g *= exp(-absorption.g * hitinfo_ra.z);
//                     ra_color.b *= exp(-absorption.b * hitinfo_ra.z);
//                 }
//             }
//             else{
//                 ra_color = environment.SampleEnvironment(Ray_rfa.dir);
//             }
//     
//         all+= ra_color * tK;
//        
//    }
//    
//    return all;
}

void BeginRender()
{
	pIt.clearFlag();
    unsigned num_thread = thread::hardware_concurrency()*2;
    //unsigned num_thread = 1;
    renderImage.SaveImage("prj14input.png");
    generatePhotonMap();
    cout<<"number of threads: "<<num_thread<<"\n";
    vector<thread> thr;
    for(int j=0;j<num_thread;j++){
        thread th(RenderPixel,ref(pIt));
        if(th.joinable()) th.detach();
        //thr.push_back(move(th));
    }
}

void saveImage(){
    cout << "Saving z-buffer image...\n";
    renderImage.ComputeZBufferImage();
    //renderImage.SaveZImage("prj13_boxzbuff.png");
    renderImage.SaveImage("prj13box.png");
    renderImage.ComputeSampleCountImage();
    renderImage.SaveSampleCountImage("prj13box_sc.png");
}
void StopRender(){
    //stop multithread
    pIt.setFlag();
    cout << "Stop thread\n";
}


int main(int argc, const char * argv[]) {
    pIt.Init();
    const char *file = "data/scene_b.xml";
    //const char *file = "scene-2.xml";
    LoadScene(file);
    //printm();
    ShowViewport();
    //saveImage();
    return 0;
}

void printm(){
    const MtlBlinn* mtlb =static_cast<const MtlBlinn*>( materials.Find("wallRed"));
   // cout<< mtlb->specular;
}
