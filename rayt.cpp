#include "rayt.hpp"
#include<memory>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"


int main(){
    int H=100;
    int W=200;
    std::unique_ptr<rayt::Image>image(std::make_unique<rayt::Image>(H,W));

    for(int j=0;j<H;j++){
        for(int i=0;i<W;i++){
            double r=1.0*i/W;
            double g=1.0*j/H;
            double b=0.5;

            image->write(i,j,r,g,b);
        }
    }

    stbi_write_bmp("reader.bmp",W,H,sizeof(rayt::Image::rgb),image->pixels());

    return 0;
}