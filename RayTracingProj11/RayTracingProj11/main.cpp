//
//  main.cpp
//  RayTracingP11 Monte Carlo GI
//
//  Created by Hsuan Lee on 11/12/17.
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

#define CAM_SAMPLE 64
#define MIN_SAMPLE 4
#define MAX_SAMPLE 16
#define HALTON_BASE_1 2
#define HALTON_BASE_2 3
#define THRESHOLD 1e-3f
#define BOUNCE 2
#define HEMISPHERE_SAMPLE 20

#define gamma 3.2

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

//if not hit
Color black(0.0);

class pixelIterator{
private:
    atomic<int> ix;
    atomic<bool> stop;
public:
    void Init(){ ix = 0; stop = false;}
    bool GetPixel(int &x, int &y){
        if(stop) return true;
        int i=ix++;
        if(i>=camera.imgWidth*camera.imgHeight) return false;
        x = i%camera.imgWidth;
        y = i/camera.imgWidth;
        return true;
    }
    void setFlag(){
        stop = ~stop;
    }
};
//multi-thread
pixelIterator pIt;

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
        //if (x!=431 || y!=227) continue;
        if (x==391 && y == 254) {
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
                        colorlist.push_back(hitinfo.node->GetMaterial()->Shade(ray_pixel, hitinfo, lights,bouncelimit));
                        
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
                if(colorlist.size()<5)
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

Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount) const{
    Color ra_color(0.0);
    Color re_color(0.0);
    Color re_ra_color(0.0);

    Color ambient_color(0.0);
    Color diffuse_color(0.0);

    Color all;
    float bias = 0.0;//0.0001f;

    Point3 N = hInfo.N;
    Point3 P = hInfo.p;

    const Material *material;
    material = hInfo.node->GetMaterial();

    const MtlBlinn* mtlb =static_cast<const MtlBlinn*>(material);
    Color Kd = mtlb->diffuse.Sample(hInfo.uvw, hInfo.duvw);
    Color Ks = mtlb->specular.Sample(hInfo.uvw, hInfo.duvw);
    float alpha = mtlb->glossiness;

    for(int i=0; i<lights.size(); i++){
        //ambient light
        if(lights[i]->IsAmbient()){
           ambient_color += lights[i]->Illuminate(P,N)*Kd;
        }
        else{
            Color I_i = lights[i]->Illuminate(P,N);
            Point3 L = -1*(lights[i]->Direction(P));
            Point3 V = -ray.dir;//camera.pos - hInfo.p; this will effect reflection calculation
            V.Normalize();
            Point3 LpV = L+V;
            Point3 H = LpV;
            H.Normalize();
            
            //
            //float theta = N.Dot(L);
            //diffuse_color += I_i*(theta>0.0?theta:0.0);
            
            Color kse = Ks*pow(N.Dot(H),alpha)+Kd;
            
            float theta = N.Dot(L);
            diffuse_color += I_i*(theta>0.0?theta:0.0)*kse;
            // */
        }
    }
    Color idr_Color(0,0,0);
    ///*
    //indirect color
    //if(bounceCount > 0
    if(bounceCount==BOUNCE //bounceCount > 0
       ){
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
        int Nofsample;
        if(bounceCount==BOUNCE)
            Nofsample = HEMISPHERE_SAMPLE;
        else
            Nofsample = 1;
        
        for(int i=0;i<Nofsample;i++){
            float r_x = 2 * (rand()/ (float) RAND_MAX)-1;
            float r_y = 2 * (rand()/ (float) RAND_MAX)-1;
            float r_z = rand()/ (float) RAND_MAX;
            Point3 hemis_dir = r_x*newx + r_y * newy + r_z * newz;
            hemis_dir.Normalize();
            //cout << newz.Length() << endl;
            
            float dotN_wi = hemis_dir.Dot(newz);
            Ray Ray_idr = Ray(P, hemis_dir );
            Ray_idr.dir.Normalize();
            HitInfo hitinfo_idr;
            hitinfo_idr.Init();
            
            Color idrColor;
            if(TraceNode(rootNode,Ray_idr,hitinfo_idr)){
                //idrColor = hitinfo_idr.node->GetMaterial()->Shade(Ray_idr, hitinfo_idr, lights, bounceCount - 1);
                idrColor = hitinfo_idr.node->GetMaterial()->Shade(Ray_idr, hitinfo_idr, lights, 1);
            }
            else{
                idrColor = environment.SampleEnvironment(Ray_idr.dir);
            }

            //
            idrColor = idrColor* dotN_wi;
            
            //idrColor.r = max(0.0f,min(1.0f,idrColor.r));
            //idrColor.g = max(0.0f,min(1.0f,idrColor.g));
            //idrColor.b = max(0.0f,min(1.0f,idrColor.b));
            
            idr_Color+= 2.0 * idrColor /(float)Nofsample;
            
            //diffuse_color += I_i*(theta>0.0?theta:0.0)*kse;
            //  */
            //idr_Color+= light /(float)Nofsample;
        }
    }
    //*/
    all = ambient_color + ((diffuse_color/M_PI)+idr_Color)*Kd;
    
    //all = ambient_color+diffuse_color+idr_Color*Kd*M_PI*2;

    /**
     ** reflection calculation
    **/
    Point3 V = -ray.dir.GetNormalized();
    if(/*re.Gray()>0 &&*/ bounceCount >0){
        Point3 newN=N;
        if(reflectionGlossiness){
            Point3 newx(1,0,0);
            newx = newN ^ newx;
            Point3 newy = newN ^ newx;
            float r = rand()/ (float) RAND_MAX;
            r = sqrtf(r)*reflectionGlossiness;
            float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
            float dx =  r * cosf(theta);
            float dy =  r * sinf(theta);
            newN += dx*newx+dy*newy;
             
            newN.Normalize();
        }
        N = newN;
        float costheta = clamp(N.Dot(V),-1.0,1.0);
        Point3 R = 2*costheta*N - V; //reflection vector
        //R.Normalize();
        Ray Ray_rf = Ray(P+bias*R, R);
        Ray_rf.dir.Normalize();
        HitInfo hitinfo_rf;
        hitinfo_rf.Init();
        if(TraceNode(rootNode,Ray_rf,hitinfo_rf))
        {
            re_color = hitinfo_rf.node->GetMaterial()->Shade(Ray_rf, hitinfo_rf, lights, bounceCount - 1);
        }
        else{
            re_color = environment.SampleEnvironment(Ray_rf.dir);
        }
    }

    all+=re_color*reflection.GetColor();
    
    /**
     ** refraction calculation
    **/
    
     if(  bounceCount>0){
         Point3 newN = hInfo.N;;
         if(refractionGlossiness){
             /*
             float r = rand()/ (float) RAND_MAX;
             r = sqrtf(r)*refractionGlossiness;
             float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
             float gamma = M_PI * rand()/ (float) RAND_MAX;
             float dx =  r * sinf(gamma) * cosf(theta);
             float dy =  r * sinf(gamma) * sinf(theta);
             float dz =  r * cosf(gamma);
             Point3 dn(dx,dy,dz);
             newN += dn;
             newN.Normalize();
              */
             Point3 newx(1,0,0);
             newx = newN ^ newx;
             Point3 newy = newN ^ newx;
             float r = rand()/ (float) RAND_MAX;
             r = sqrtf(r)*refractionGlossiness;
             float theta = M_PI * 2.0 * rand()/ (float) RAND_MAX;
             float dx =  r * cosf(theta);
             float dy =  r * sinf(theta);
             newN += dx*newx+dy*newy;
             
             newN.Normalize();
         }
         N = newN;
        
        float R0 = 0.0f, re_ratio = 0.0f, ra_ratio = 0.0f;
        
        V.Normalize();//unit vector
        
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
         
         float absorb = 0.0;
        if(sintheta2 <= 1.0){

            float costheta2 = sqrtf(max(0.0f,1 - (sintheta2 * sintheta2)));
            Point3 S = N ^ (N ^ V);
            N.Normalize();
            S.Normalize();
            Point3 T = -N * costheta2 + S * sintheta2;

            Ray Ray_rfa = Ray(P+bias*T, T);

            if(TraceNode(rootNode,Ray_rfa,hitinfo_ra)){
                ra_color  =  hitinfo_ra.node->GetMaterial()->Shade(Ray_rfa,hitinfo_ra,lights,bounceCount-1);
            }
            else{
                ra_color = environment.SampleEnvironment(Ray_rfa.dir);
            }
            absorb = exp(-absorption.r * hitinfo_ra.z);
            //
            //Shlick's approximation
            R0 = (n1 - n2)/(n1 + n2);
            R0 = R0*R0;
            double  tmp = 1.0 - costheta1;
            re_ratio = R0 + (1.0 - R0) * pow(tmp,5.0);
            ra_ratio = 1.0 - re_ratio;
        }
        else {
            // complete reflection
            re_ratio = 1.0f;
        }
        
        /*** 
        **   Reflection because of refraction
        ***/
        //already calculated
        re_ra_color = re_color;
         
        all += refraction.GetColor() * (ra_ratio * absorb * ra_color + re_ratio * re_ra_color);
        
    }

     
    return all;
}

void BeginRender()
{
	
    unsigned num_thread = thread::hardware_concurrency()*2;
    //unsigned num_thread = 1;
    //renderImage.SaveImage("prj11boxinput.png");
    
    cout<<"number of threads: "<<num_thread<<"\n";
    vector<thread> thr;
    for(int j=0;j<num_thread;j++){
        thread th(RenderPixel,ref(pIt));
        if(th.joinable()) th.detach();
        //thr.push_back(move(th));
    }
    
    //join
    //for(int j=0;j<num_thread;j++){
    //    thr.at(j).join();
    //}
    
    /*
    
     */
}

void saveImage(){
    cout << "Saving z-buffer image...\n";
    renderImage.ComputeZBufferImage();
    //renderImage.SaveZImage("prj11_boxzbuff.png");
    renderImage.SaveImage("prj11box.png");
    renderImage.ComputeSampleCountImage();
    //renderImage.SaveSampleCountImage("prj11box_sc.png");
}
void StopRender(){
    //stop multithread
    pIt.setFlag();
    cout << "Stop thread\n";
}


int main(int argc, const char * argv[]) {
    pIt.Init();
    const char *file = "scene-2.xml";
    //const char *file = "scene_simple.xml";
    LoadScene(file);
    ShowViewport();
    
    return 0;
}
