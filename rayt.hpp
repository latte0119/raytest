#include<memory>
#include<random>
#include<vector>
#include<iostream>
#include<numeric>
#include "sce_vectormath/include/vectormath/scalar/cpp/vectormath_aos.h"
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"



namespace rayt{
    using namespace Vectormath::Aos;
    using vec3=Vector3;
    using col3=Vector3;
    
    class Material;
    using MaterialPtr=std::shared_ptr<Material>;


    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<double>urd(0.0,1.0);
    inline double drand48(){
        return urd(engine);
    }
    const double PI=acos(-1);
    const double EPS=1e-6;
    const double INF=std::numeric_limits<double>::max();
    const double GAMMA_FACTOR=2.2;
    const int MAX_DEPTH=50;
    const int NUM_THREAD=6;

    inline double radians(const double deg){return deg/180*PI;}
    inline double degrees(const double rad){return rad/PI*180;}
    inline double recip(double x){return 1.0/x;}
    inline double pow2(double x){return x*x;}
    inline double pow5(double x){return x*x*x*x*x;}

    inline vec3 random_vector(){
        return vec3(drand48(),drand48(),drand48());
    }

    inline vec3 random_in_unit_sphere(){
        vec3 p;
        do{
            p=2*random_vector()-vec3(1);
        }while(lengthSqr(p)>1-EPS);
        return p;
    }

    inline vec3 reflect(const vec3& v,const vec3& n){
        return v-2.0*dot(v,n)*n;
    }

    inline bool refract(const vec3&  v,const vec3& n,double ni_over_nt,vec3& refracted){
        vec3 uv=normalize(v);
        double dt=dot(uv,n);
        double D=1-pow2(ni_over_nt)*(1-pow2(dt));

        if(D>EPS){
            refracted=-ni_over_nt*(uv-n*dt)-n*sqrt(D);
            return true;
        }
        else{
            return false;
        }
    }

    inline double schlick(double cosine,double ri){
        double r0=pow2((1-ri)/(1+ri));
        return r0+(1-r0)*pow5(1-cosine);
    }

    inline vec3 inverseGamma(const vec3& v,double gammaFactor){
        double recipGammaFactor=recip(gammaFactor);
        return vec3(
            pow(v.getX(),recipGammaFactor),
            pow(v.getY(),recipGammaFactor),
            pow(v.getZ(),recipGammaFactor)
        );
    }

    class ImageFilter{
        public:
        virtual vec3 filter(const vec3& c)const=0;
    };

    class sRGBFilter:public ImageFilter{
        public:
        sRGBFilter(double factor):m_factor(factor){}
        vec3 filter(const vec3& c)const override{
            return inverseGamma(c,m_factor);
        }
        private:
        double m_factor;
    };
    

    class Image{
        public:
        struct rgb{
            unsigned char r,g,b;
        };

        Image():m_pixels(nullptr){}
        Image(int w,int h){
            m_width=w;
            m_height=h;
            m_pixels.reset(new rgb[m_width*m_height]);
            m_filters.push_back(std::make_unique<sRGBFilter>(GAMMA_FACTOR));
        }

        inline int height()const{return m_height;}
        inline int width()const{return m_width;}
        void* pixels()const{return m_pixels.get();}

        void write(int x,int y,double r,double g,double b){
            vec3 c(r,g,b);
            for(auto& f:m_filters)c=f->filter(c);
            int idx=m_width*y+x;
            m_pixels[idx].r=static_cast<unsigned char>(round(c.getX()*255));
            m_pixels[idx].g=static_cast<unsigned char>(round(c.getY()*255));
            m_pixels[idx].b=static_cast<unsigned char>(round(c.getZ()*255));
        }
        private:
        int m_height;
        int m_width;
        std::unique_ptr<rgb[]>m_pixels;
        std::vector<std::unique_ptr<ImageFilter>>m_filters;
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
            const vec3 Z=normalize(lookfrom-lookat);
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


  

    class HitRec{
        public:
        double t;
        vec3 p;
        vec3 n;
        MaterialPtr mat;
    };



    class ScatterRec{
        public:
        Ray ray;
        vec3 albedo;
    };



    class Material{
        public:
        virtual bool scatter(const Ray&r ,const HitRec& hrec,ScatterRec& srec)const=0;
    };
    using MaterialPtr=std::shared_ptr<Material>;


    class Lambertian:public Material{
        public:

        Lambertian(const vec3& c):m_albedo(c){}

        bool scatter(const Ray& r,const HitRec& hrec,ScatterRec& srec)const override{
            vec3 target=hrec.p+hrec.n+random_in_unit_sphere();
            srec.ray=Ray(hrec.p,target-hrec.p);
            srec.albedo=m_albedo;
            return true;
        }

        private:
        vec3 m_albedo;
    };

    class Metal:public Material{
        public:
        Metal(const vec3& c,double fuzz):m_albedo(c),m_fuzz(fuzz){}

        bool scatter(const Ray& r,const HitRec& hrec,ScatterRec& srec)const override{
            vec3 reflected=reflect(normalize(r.direction()),hrec.n);
            reflected+=m_fuzz*random_in_unit_sphere();
            srec.ray=Ray(hrec.p,reflected);
            srec.albedo=m_albedo;
            return true;
        }

        private:
        vec3 m_albedo;
        double m_fuzz;
    };
   

    class Dielectric: public Material{
        public:
        Dielectric(double ri):m_ri(ri){}

        bool scatter(const Ray& r,const HitRec& hrec,ScatterRec& srec)const override{
            vec3 outward_normal;
            vec3 reflected=reflect(r.direction(),hrec.n);
            double ni_over_nt;
            double reflect_prob;
            double cosine;

            if(dot(r.direction(),hrec.n)>EPS){
                outward_normal=-hrec.n;
                ni_over_nt=m_ri;
            }
            else{
                outward_normal=hrec.n;
                ni_over_nt=recip(m_ri);
            }
            cosine=abs(dot(r.direction(),hrec.n))/length(r.direction());

            srec.albedo=vec3(1);

            vec3 refracted;
            if(refract(-r.direction(),outward_normal,ni_over_nt,refracted)){
                reflect_prob=schlick(cosine,m_ri);
            }
            else{
                reflect_prob=1;
            }


            if(drand48()<reflect_prob){
                srec.ray=Ray(hrec.p,reflected);
            }
            else{
                srec.ray=Ray(hrec.p,refracted);
            }
            return true;
        }
        private:
        double m_ri;
    };

    class Shape{
        public:
        virtual bool hit(const Ray& r,double t0,double t1,HitRec& hrec)const=0;
    };


    using ShapePtr=std::shared_ptr<Shape>;

    class Sphere :public Shape{
        public:
        Sphere(){}
        Sphere(const vec3& c,double r,MaterialPtr mat):m_center(c),m_radius(r),m_material(mat){}

        bool hit(const Ray& r,double t0,double t1,HitRec& hrec)const override{
            vec3 oc=r.origin()-m_center;
            double a=dot(r.direction(),r.direction());
            double b=2*dot(oc,r.direction());
            double c=dot(oc,oc)-m_radius*m_radius;

            double D=b*b-4*a*c;
            if(D<EPS)return false;


            double root=sqrt(D);
            double tmp=(-b-root)/(2*a);
            if(tmp+EPS<t1&&t0+EPS<tmp){
                hrec.t=tmp;
                hrec.p=r.at(hrec.t);
                hrec.n=(hrec.p-m_center)/m_radius;
                hrec.mat=m_material;
                return true;
            }

            tmp=(-b+root)/(2*a);
            if(tmp+EPS<t1&&t0+EPS<tmp){
                hrec.t=tmp;
                hrec.p=r.at(hrec.t);
                hrec.n=(hrec.p-m_center)/m_radius;
                hrec.mat=m_material;
                return true;
            }
            return false;
        }

        private:
        vec3 m_center;
        double m_radius;
        MaterialPtr m_material;
    };


    class ShapeList:public Shape{
        public:
        ShapeList(){}

        void add(const ShapePtr& shape){
            m_list.push_back(shape);
        }

        bool hit(const Ray& r,double t0,double t1,HitRec& hrec)const override{
            HitRec  tmp_hrec;
            bool hit_anything=false;
            double closest_so_far=INF;
            for(auto &p:m_list){
                if(p->hit(r,t0,closest_so_far,tmp_hrec)){
                    hit_anything=true;
                    closest_so_far=tmp_hrec.t;
                    hrec=tmp_hrec;
                }
            }
            return hit_anything;
        }

        private:
        std::vector<ShapePtr>m_list;
    };

    class Scene{
        public:
        Scene(int width,int height,int samples):m_image(std::make_unique<Image>(width,height)),m_backColor(0.2),m_samples(samples){}
        void build(){
            vec3 lookfrom(13,2,3);
            vec3 lookat(0,0,0);
            vec3 vup(0,1,0);
            double aspect = 1.0*m_image->width() /m_image->height();
            m_camera = std::make_unique<Camera>(lookfrom, lookat, vup, 20, aspect);
            
            ShapeList* world = new ShapeList();

            int N = 11;
            for (int i=-N; i<N; ++i) {
                for (int j=-N; j<N; ++j) {
                    float choose_mat = drand48();
                    vec3 center(i+0.9f*drand48(), 0.2f, j+0.9f*drand48());
                    if (length(center-vec3(4,0.2,0)) > 0.9f) {
                        if (choose_mat < 0.8f) {
                            world->add(std::make_shared<Sphere>(
                                center, 0.2f, 
                                std::make_shared<Lambertian>(mulPerElem(random_vector(),random_vector()))));
                        }
                        else if (choose_mat < 0.95f) {
                            world->add(std::make_shared<Sphere>(
                                center, 0.2f,
                                std::make_shared<Metal>(0.5f * (random_vector()+vec3(1)), 0.5f*drand48())));
                        }
                        else {
                            world->add(std::make_shared<Sphere>(
                                center, 0.2f,
                                std::make_shared<Dielectric>(1.5f)));
                        }
                    }
                }
            }


            world->add(std::make_shared<Sphere>(
                vec3(0, -1000, -1), 1000,
                std::make_shared<Lambertian>(vec3(0.5, 0.5, 0.5))));

            m_world.reset(world);
        }


        vec3 color(const rayt::Ray& r,const Shape* world,int depth)const{
            HitRec hrec;
            if(world->hit(r,EPS,INF,hrec)){
                ScatterRec srec;
                if(depth<MAX_DEPTH&&hrec.mat->scatter(r,hrec,srec)){
                    return mulPerElem(srec.albedo,color(srec.ray,world,depth+1));
                }
                else{
                    return vec3(0);
                }
            }
            return backgroundSky(r.direction());
        }
        vec3 backgroundSky(const vec3& d)const{
            vec3 v=normalize(d);
            double t=0.5*(v.getY()+1.0);
            return lerp(t,vec3(1),vec3(0.5,0.7,1.0));
        }

        void render(){
            build();
            int W=m_image->width();
            int H=m_image->height();
#pragma omp parallel for schedule(dynamic, 1) num_threads(NUM_THREAD)
            for(int y=0;y<H;y++){
                for(int x=0;x<W;x++){
                    vec3 c(0);
                    for(int s=0;s<m_samples;s++){
                        double u=(x+drand48())/W;
                        double v=(y+drand48())/H;
                        Ray r=m_camera->getRay(u,v);
                        c+=color(r,m_world.get(),0);
                    }
                    c/=m_samples;
                    m_image->write(x,H-1-y,c.getX(),c.getY(),c.getZ());
                }
            }

            stbi_write_png("render.png",W,H,sizeof(Image::rgb),m_image->pixels(),W*sizeof(Image::rgb));
        }

        private:
        std::unique_ptr<Camera>m_camera;
        std::unique_ptr<Image>m_image;
        std::unique_ptr<Shape>m_world;
        vec3 m_backColor;
        int m_samples;
        
    };
}