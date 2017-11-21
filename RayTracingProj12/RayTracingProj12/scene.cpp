//
//  scene.cpp
//  RayTracingProj8
//
//  Created by Hsuan Lee on 10/19/17.
//  Copyright © 2017 Hsuan Lee. All rights reserved.
//

#include "scene.h"

bool Box::IntersectRay(const Ray &r, float t_max) const{
    if (IsInside(r.p)) return true;//
    
    Point3 corner0 = Corner(0);
    Point3 corner7 = Corner(7);
    float tenter = -t_max;
    float texit = t_max;
    
    //check x axis boundary
    float bound_low = corner0.x;
    float bound_hi = corner7.x;
    
    if(r.dir.x!=0.0f){
        float tx0 = (bound_low-r.p.x)/r.dir.x;
        float tx1 = (bound_hi-r.p.x)/r.dir.x;
        //swap
        if(tx0>tx1){
            float tmp = tx0;
            tx0 = tx1;
            tx1 = tmp;
        }
        tenter = max(tx0,tenter);
        texit = min(tx1,texit);
    }
    if(r.dir.y!=0.0f){
        bound_low = corner0.y;
        bound_hi  = corner7.y;
        float ty0 = (bound_low-r.p.y)/r.dir.y;
        float ty1 = (bound_hi-r.p.y)/r.dir.y;
        //swap
        if(ty0>ty1){
            float tmp = ty0;
            ty0 = ty1;
            ty1 = tmp;
        }
        tenter = max(ty0,tenter);
        texit = min(ty1,texit);
    }
    if(r.dir.z!=0.0f){
        bound_low = corner0.z;
        bound_hi  = corner7.z;
        float tz0 = (bound_low-r.p.z)/r.dir.z;
        float tz1 = (bound_hi-r.p.z)/r.dir.z;
        //swap
        if(tz0>tz1){
            float tmp = tz0;
            tz0 = tz1;
            tz1 = tmp;
        }
        tenter = max(tz0,tenter);
        texit = min(tz1,texit);
    }
    return tenter<=texit && texit<=t_max;
    
}
