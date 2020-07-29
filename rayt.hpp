#include<memory>
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
}