//
//  main.cpp
//  RayTracingP02
//
//  Created by Hsuan Lee on 8/30/17.
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
MaterialList materials;
LightList lights;

//if not hit
Color black(0.0);

class pixelIterator{
private:
    atomic<int> ix;
public:
    void Init(){ ix = 0;}
    bool GetPixel(int &x, int &y){
        int i=ix++;
        if(i>=camera.imgWidth*camera.imgHeight) return false;
        x = i%camera.imgWidth;
        y = i/camera.imgWidth;
        return true;
    }
};
//multi-thread
pixelIterator pIt;

bool Trace(const Ray &ray, HitInfo &hitinfo);
bool TraceNode(const Node &node, const Ray &ray, HitInfo &hitinfo);

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
    
    while(it.GetPixel(x,y)){
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
                setPixelColor(index,hitinfo.z,hitinfo.node->GetMaterial()->Shade(ray_pixel, hitinfo, lights));
            }
            else{
                setPixelColor(index,BIGFLOAT,black);
            }
            renderImage.IncrementNumRenderPixel(1);
        }
    }
}

Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights) const{
    //Color shade_color(255.0);
    Color ambient_color(0.0);
    Color diffuse_color(0.0);
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
            Point3 V = camera.pos - hInfo.p;
            V.Normalize();
            Point3 LpV = L+V;
            Point3 H = LpV/LpV.Length();
            H.Normalize();

            Color kse = Ks*pow(N.Dot(H),alpha)+Kd;
            //cout<<"L.length: "<<L.Length()<<", N.l: "<<N.Length()<<endl;
            float theta = N.Dot(L);
            //
            diffuse_color += I_i*(theta>0?theta:0)*kse;
        }
    }

    return ambient_color+diffuse_color;
}

bool Sphere::IntersectRay( const Ray &ray, HitInfo &hitinfo, int hitSide ) const{
	bool behitted = false;
	float a = ray.dir.Dot(ray.dir);
	float c = ray.p.Dot(ray.p)-1;
	float b = 2*ray.p.Dot(ray.dir);
	float insqrt = b*b-(4*a*c);
	if(insqrt>=0){
		float t1 = (-b+sqrtf(insqrt))/(a*2);
		float t2 = (-b-sqrtf(insqrt))/(a*2);
		float prez = hitinfo.z;
        
        hitinfo.z = min(t1,t2);
        if(hitinfo.z<0)
        	return false;
        else if(hitinfo.z>=prez){
            hitinfo.z = prez;
            return false;
        }
        else
         	behitted = true;
        
        hitinfo.p = hitinfo.z*ray.dir+ray.p;
        hitinfo.N = hitinfo.p;
        //hitinfo.N.Normalize();
	}

	return behitted;
}

bool Trace(const Ray &ray, HitInfo &hitinfo){
    return TraceNode(rootNode, ray, hitinfo);
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
			hit = true;
		}
	}
    return hit;
}


void BeginRender()
{	
	cout<<"call by GlutKeyboard() in viewport.cpp\n";
	//renderImage.SaveImage("/Users/hsuanlee/Documents/Cpp/RayTracingP02/RayTracingP02/prj2input.png");
    
    unsigned num_thread = thread::hardware_concurrency();
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
    renderImage.ComputeZBufferImage();
    //renderImage.SaveZImage("/Users/hsuanlee/Documents/Cpp/RayTracingP02/RayTracingP02/prj2.png");
    //renderImage.SaveImage("/Users/hsuanlee/Documents/Cpp/RayTracingP02/RayTracingP02/prj2img.png");
}

void StopRender(){
    //stop multithread
    
}


int main(int argc, const char * argv[]) {
    pIt.Init();
    //const char *file = "simplescene.xml"; //can't load the file
    const char *file = "/Users/hsuanlee/Documents/Cpp/RayTracingP03/RayTracingP03/input.xml";
    LoadScene(file);
    ShowViewport();
    
    return 0;
}
