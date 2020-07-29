#include "rayt.hpp"
#include<memory>
#include<iostream>
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"


vec3 color(const rayt::Ray& r){
    vec3 d=normalize(r.direction());
    double t=(d.getY()+1.0)/2;
    return lerp(t,vec3(0.3,0.6,0.9),vec3(0,0,0));
}

int main(){
    int H=100;
    int W=200;
    std::unique_ptr<rayt::Image>image(std::make_unique<rayt::Image>(H,W));

    vec3 x(4,0,0);
    vec3 y(0,2,0);
    vec3 z(-2,-1,-1);

    std::unique_ptr<rayt::Camera>camera(std::make_unique<rayt::Camera>(x,y,z));

    for(int j=0;j<H;j++){
        for(int i=0;i<W;i++){
            double u=1.0*i/W;
            double v=1.0*j/H;

            rayt::Ray ray=camera->getRay(u,v);
            
            vec3 c=color(ray);

            image->write(i,j,c.getX(),c.getY(),c.getZ());
        }
    }

    stbi_write_bmp("reader.bmp",W,H,sizeof(rayt::Image::rgb),image->pixels());

    return 0;
}