//
//  main.cpp
//  RayTracingP06
//
//  Created by Hsuan Lee on 9/27/17.
//  Copyright © 2017 Hsuan Lee. All rights reserved.
//

#include <iostream>
#include <thread>
#include "viewport.cpp"
#include "xmlload.cpp"
#include "objects.h"
#include <math.h>
#include <algorithm> 

using namespace std;

Node rootNode;
Camera camera;
RenderImage renderImage;
Sphere theSphere;
Plane thePlane;
MaterialList materials;
LightList lights;
ObjFileList objList;

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

void RenderPixel(pixelIterator &it){
    int x,y;
    
    float theta = camera.fov;
    float l = 1.0;
    float h = 2*l*tan(theta/2*(M_PI/180));
    float w = h*(float)camera.imgWidth/camera.imgHeight;
    Point3 b;
    b.Set(-w/2,h/2,-l);
    float u = w/camera.imgWidth;
    float v = -h/camera.imgHeight;
    b.x+=u/2;
    b.y+=v/2;
    
    Point3 z_new = -1*(camera.dir);
    Point3 x_new = camera.up ^ z_new;
    camera.up.Normalize();
    z_new.Normalize();
    x_new.Normalize();
    Matrix3 m(x_new,camera.up, z_new);

    int bouncelimit = 5;
    
    while(it.GetPixel(x,y)){
        //debug
        if (x==437 && y == 353) {
            std::cout << "xx" << std::endl;
        }
        
        Point3 tmp(x*u,y*v,0);
        tmp+=b;
        Ray ray_pixel(camera.pos,tmp);
        ray_pixel.dir=m*(ray_pixel.dir);
        ray_pixel.dir.Normalize();
        
        HitInfo hitinfo;
        hitinfo.Init();
        
        if(rootNode.GetNumChild()>0){
            
            int index = y*camera.imgWidth+x;
            if(Trace(ray_pixel,hitinfo)){
                //set color
                setPixelColor(index,hitinfo.z,hitinfo.node->GetMaterial()->Shade(ray_pixel, hitinfo, lights,bouncelimit));
            }
            else{
                setPixelColor(index,BIGFLOAT,black);
            }
            renderImage.IncrementNumRenderPixel(1);
        }
    }
}

float GenLight::Shadow(Ray ray, float t_max){
    //bias
    float bias = 1e-14f;
    
    HitInfo hitInfo;

    //*/
        hitInfo.Init();
        if(TraceNode(rootNode,ray,hitInfo))
        {
            if(hitInfo.z>bias && hitInfo.z < t_max){
                return 0.0;
            }
        }
    //*/
    
    //it is not shadow, intensity would not change.
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
    Color Kd = mtlb->diffuse;
    Color Ks = mtlb->specular;
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
            Point3 H = LpV;///LpV.Length();
            H.Normalize();

            Color kse = Ks*pow(N.Dot(H),alpha)+Kd;
            //cout<<"L.length: "<<L.Length()<<", N.l: "<<N.Length()<<endl;
            float theta = N.Dot(L);
            //
            diffuse_color += I_i*(theta>0.0?theta:0.0)*kse;
        }
    }

    all = ambient_color+diffuse_color;

    /**
     ** reflection calculation
    **/
    Point3 V = -ray.dir.GetNormalized();
    Color re = reflection;
    if(re.Gray()>0 && bounceCount >0){
        
        float costheta = clamp(N.Dot(V),-1.0,1.0);
        Point3 R = 2*costheta*N - V; //reflection vector
        R.Normalize();
        Ray Ray_rf = Ray(P+bias*R, R);

        HitInfo hitinfo_rf;
        hitinfo_rf.Init();
        if(TraceNode(rootNode,Ray_rf,hitinfo_rf))
        {
            re_color = hitinfo_rf.node->GetMaterial()->Shade(Ray_rf, hitinfo_rf, lights, bounceCount - 1);
        }
    }

    all+=re_color*reflection;
    
    /**
     ** refraction calculation
    **/
    
     if( refraction.Gray()>0 && bounceCount>0){
        //Color reflShade = Color(0,0,0);
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
            N = -hInfo.N;
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
                /*
                if(!hitinfo_ra.front){
                    ra_color.r *= exp(-absorption.r * hitinfo_ra.z);
                    ra_color.g *= exp(-absorption.g * hitinfo_ra.z);
                    ra_color.b *= exp(-absorption.b * hitinfo_ra.z);
                }
                 */
                absorb = exp(-absorption.r * hitinfo_ra.z);
                //
                //Shlick's approximation 
                R0 = (n1 - n2)/(n1 + n2);
                R0 = R0*R0;
                double  tmp = 1.0 - costheta1;
                re_ratio = R0 + (1.0 - R0) * pow(tmp,5.0);
                ra_ratio = 1.0 - re_ratio;
            }
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
         
        if(re.Gray()>0 && bounceCount >0) re_ra_color = re_color;
        // not yet calculated
        else if(re_ratio >0.0 && bounceCount >0){
            
            float costheta = clamp(N.Dot(V),-1.0,1.0);
            Point3 R = 2*costheta*N - V; //reflection vector

            Ray Ray_rf = Ray(P+bias*R, R);

            HitInfo hitinfo_rf;
            hitinfo_rf.Init();
            //re_ra_color = re_color;
            
            if(TraceNode(rootNode,Ray_rf,hitinfo_rf))
            {
                //don't need to multiply with reflection
                re_ra_color =  hitinfo_rf.node->GetMaterial()->Shade(Ray_rf, hitinfo_rf, lights, bounceCount - 1);
            }
            
        }
          
         
        all += refraction * (ra_ratio * absorb*ra_color + re_ratio * re_ra_color);
        
    }

     
    return all;
}

bool Sphere::IntersectRay( const Ray &ray, HitInfo &hitinfo,/*&hInfo,*/int hitSide ) const{
    
    
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
        }
        else if(t1>zero && t2>zero && t2<prez){
            //if(min_t<zero){
                //hitinfo.z = prez;
            //	return false;
            //}
            //else{
             	behitted = true;
                hitinfo.z = min_t;
                //do not forget to assign front=true
                hitinfo.front = true;
            //}
            hitinfo.p = hitinfo.z*ray.dir+ray.p;
            hitinfo.N = hitinfo.p;
            hitinfo.N.Normalize();
        }
     
        //hitinfo.p = hitinfo.z*ray.dir+ray.p;
        //hitinfo.N = hitinfo.p;
        //hitinfo.N.Normalize();
     
	}
     
	return behitted;
     
    /*
    bool hit = false;
    //        float bias = 0.05;
    float bias = 0.001;
    
    // node center and radius
    Point3 worldcenter;
    worldcenter.Set(0.0f, 0.0f, 0.0f);
    
    float radius = 1.0f;
    
    const float a = ray.dir.Dot(ray.dir);
    const float b = 2.0f * ray.p.Dot(ray.dir);
    const float c = ray.p.Dot(ray.p) - radius * radius;
    
    const float radical = b * b - 4 * a * c;
    
    if(radical < 0.0f) hit = false;
    
    const float srad = sqrtf(radical);
    
    const float t_in = ((-1) * b - srad) / (2.0f * a);
    const float t_out = ((-1) * b + srad) / (2.0f * a);
    
    float t;
    
    // if radical == 0 just have one hit point
    if(t_in == t_out && t_in > bias && hInfo.z > t_in){
        t = t_in;
        hit = true;
        hInfo.z = t;
        hInfo.p = ray.p + t * ray.dir;
        hInfo.N = hInfo.p;
        hInfo.front = true;
    }else if(t_in >= bias && t_in < t_out && t_in > bias && hInfo.z > t_in){
        t = t_in;
        hit = true;
        hInfo.z = t;
        hInfo.p = ray.p + t * ray.dir;
        hInfo.N = hInfo.p;
        hInfo.front = true;
    }else if(t_in < bias && t_out >= bias && t_out > bias && hInfo.z > t_out){
        // hit back face
        t = t_out;
        hit = true;
        hInfo.z = t;
        hInfo.p = ray.p + t * ray.dir;
        hInfo.N = hInfo.p;
        hInfo.front = false;
    }
    return hit;
    */

}




void BeginRender()
{
	
    unsigned num_thread = thread::hardware_concurrency()*2;
    //unsigned num_thread = 1;
    
    cout<<"number of threads: "<<num_thread<<"\n";
    vector<thread> thr;
    for(int j=0;j<num_thread;j++){
        thread th(RenderPixel,ref(pIt));
        thr.push_back(move(th));
    }
    
    //join
    for(int j=0;j<num_thread;j++){
        thr.at(j).join();
    }
    
    cout << "Saving z-buffer image...\n";
    //renderImage.ComputeZBufferImage();
    //renderImage.SaveZImage("prj6_zbuff.png");
    renderImage.SaveImage("prj6.png");
}

void StopRender(){
    //stop multithread
    pIt.setFlag();
    cout << "Stop thread\n";
}


int main(int argc, const char * argv[]) {
    pIt.Init();
    //const char *file = "box.xml";
    const char *file = "scene.xml";
    LoadScene(file);
    ShowViewport();
    
    return 0;
}
