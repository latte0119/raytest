#include<memory>
#include<random>
#include "sce_vectormath/include/vectormath/scalar/cpp/vectormath_aos.h"


using namespace Vectormath::Aos;
using vec3=Vector3;
using col3=Vector3;
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());
std::uniform_real_distribution<double>urd(0.0,1.0);
inline double drand48(){
    return urd(engine);
}
const double PI=acos(-1);
inline double radians(const double deg){return deg/180*PI;}
inline double degrees(const double rad){return rad/PI*180;}


namespace rayt{
    
    class Image{
        public:
        struct rgb{
            unsigned char r,g,b;
        };

        Image():m_pixels(nullptr){}
        Image(int h,int w){
            m_height=h;
            m_width=w;
            m_pixels.reset(new rgb[m_width*m_height]);
        }

        inline int height()const{return m_height;}
        inline int width()const{return m_width;}
        void* pixels()const{return m_pixels.get();}

        void write(int x,int y,double r,double g,double b){
            int idx=m_width*y+x;
            m_pixels[idx].r=static_cast<unsigned char>(r*255.99);
            m_pixels[idx].g=static_cast<unsigned char>(g*255.99);
            m_pixels[idx].b=static_cast<unsigned char>(b*255.99);
        }
        private:
        int m_height;
        int m_width;
        std::unique_ptr<rgb[]>m_pixels;
    };

    class Ray{
        public:
        Ray(){}
        Ray(const vec3& o,const vec3& dir):m_origin(o),m_direction(dir){}    

        const vec3& origin()const{return m_origin;}
        const vec3& direction()const{return m_direction;}
        const vec3 at(double t)const{return m_origin+t*m_direction;}

        private:
        vec3 m_origin;
        vec3 m_direction;
    };


    class Camera{
        public:
        Camera(const vec3& u,const vec3& v,const vec3& w){
            m_origin=vec3(0);
            m_uvw[0]=u;
            m_uvw[1]=v;
            m_uvw[2]=w;
        }

        Camera(const vec3& lookfrom,const vec3& lookat, const vec3& vup,double vfov,double aspect){
            const vec3 Z=normalize(lookat-lookfrom);
            const vec3 X=normalize(cross(vup,Z));
            const vec3 Y=cross(Z,X);

            const double rad=radians(vfov);
            const double h=tan(rad/2);
            const double w=h*aspect;

            m_uvw[0]=2*w*X;
            m_uvw[1]=2*h*Y;
            m_uvw[2]=lookfrom-w*X-h*Y-Z;
            m_origin=lookfrom;
        }

        Ray getRay(double u,double v){
            return Ray(m_origin,m_uvw[0]*u+m_uvw[1]*v+m_uvw[2]-m_origin);
        }

        private:
        vec3 m_origin;
        vec3 m_uvw[3];
    };
}