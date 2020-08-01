#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include "sce_vectormath/include/vectormath/scalar/cpp/vectormath_aos.h"
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

namespace rayt {
using namespace Vectormath::Aos;
using vec3 = Vector3;
using col3 = Vector3;

class Material;
using MaterialPtr = std::shared_ptr<Material>;

std::random_device seed_gen;
std::default_random_engine engine(seed_gen());
std::uniform_real_distribution<double> urd(0.0, 1.0);
inline double drand48() {
    return urd(engine);
}
const double PI = acos(-1);
const double EPS = 1e-6;
const double INF = std::numeric_limits<double>::max();
const double GAMMA_FACTOR = 2.2;
const int MAX_DEPTH = 50;
const int NUM_THREAD = 6;

inline double radians(const double deg) { return deg / 180 * PI; }
inline double degrees(const double rad) { return rad / PI * 180; }
inline double recip(double x) { return 1.0 / x; }
inline double pow2(double x) { return x * x; }
inline double pow5(double x) { return x * x * x * x * x; }

inline vec3 random_vector() {
    return vec3(drand48(), drand48(), drand48());
}

inline vec3 random_in_unit_sphere() {
    vec3 p;
    do {
        p = 2 * random_vector() - vec3(1);
    } while (lengthSqr(p) > 1 - EPS);
    return p;
}

inline vec3 reflect(const vec3 &v, const vec3 &n) {
    return v - 2.0 * dot(v, n) * n;
}

inline bool refract(const vec3 &v, const vec3 &n, double ni_over_nt, vec3 &refracted) {
    vec3 uv = normalize(v);
    double dt = dot(uv, n);
    double D = 1 - pow2(ni_over_nt) * (1 - pow2(dt));

    if (D > EPS) {
        refracted = -ni_over_nt * (uv - n * dt) - n * sqrt(D);
        return true;
    } else {
        return false;
    }
}

inline double schlick(double cosine, double ri) {
    double r0 = pow2((1 - ri) / (1 + ri));
    return r0 + (1 - r0) * pow5(1 - cosine);
}

inline vec3 inverseGamma(const vec3 &v, double gammaFactor) {
    double recipGammaFactor = recip(gammaFactor);
    return vec3(
        pow(v.getX(), recipGammaFactor),
        pow(v.getY(), recipGammaFactor),
        pow(v.getZ(), recipGammaFactor));
}

inline void get_sphere_uv(const vec3 &p, double &u, double &v) {
    double phi = atan2(p.getZ(), p.getX());
    double theta = asin(p.getY());
    u = 1 - (phi + PI) / (2 * PI);
    v = (theta + PI / 2) / PI;
}

inline vec3 random_cosine_direction() {
    double r1 = drand48();
    double r2 = drand48();

    double z = sqrt(1 - r2);
    double phi = 2 * PI * r1;

    double x = cos(phi) * sqrt(r2);
    double y = sin(phi) * sqrt(r2);
    return vec3(x, y, z);
}

class ONB {
   public:
    ONB() {}
    inline vec3 &operator[](int i) {
        return m_axis[i];
    }
    inline const vec3 &operator[](int i) const {
        return m_axis[i];
    }
    const vec3 &u() const {
        return m_axis[0];
    }
    const vec3 &v() const {
        return m_axis[1];
    }
    const vec3 &w() const {
        return m_axis[2];
    }
    vec3 local(double a, double b, double c) const {
        return a * u() + b * v() + c * w();
    }
    vec3 local(const vec3 &a) const {
        return a.getX() * u() + a.getY() * v() + a.getZ() * w();
    }
    void build_from_w(const vec3 &n) {
        m_axis[2] = normalize(n);
        vec3 a;
        if (abs(w().getX()) > 0.9) {
            a = vec3(0, 1, 0);
        } else {
            a = vec3(1, 0, 0);
        }
        m_axis[1] = normalize(cross(w(), a));
        m_axis[0] = cross(w(), v());
    }

   private:
    vec3 m_axis[3];
};

class ImageFilter {
   public:
    virtual vec3 filter(const vec3 &c) const = 0;
};

class sRGBFilter : public ImageFilter {
   public:
    sRGBFilter(double factor) : m_factor(factor) {}
    vec3 filter(const vec3 &c) const override {
        return inverseGamma(c, m_factor);
    }

   private:
    double m_factor;
};

class TonemapFilter : public ImageFilter {
   public:
    TonemapFilter() {}
    vec3 filter(const vec3 &c) const override {
        return minPerElem(maxPerElem(c, vec3(0)), vec3(1));
    }
};

class Image {
   public:
    struct rgb {
        unsigned char r, g, b;
    };

    Image() : m_pixels(nullptr) {}
    Image(int w, int h) {
        m_width = w;
        m_height = h;
        m_pixels.reset(new rgb[m_width * m_height]);
        m_filters.push_back(std::make_unique<sRGBFilter>(GAMMA_FACTOR));
        m_filters.push_back(std::make_unique<TonemapFilter>());
    }

    inline int height() const { return m_height; }
    inline int width() const { return m_width; }
    void *pixels() const { return m_pixels.get(); }

    void write(int x, int y, double r, double g, double b) {
        vec3 c(r, g, b);
        for (auto &f : m_filters)
            c = f->filter(c);
        int idx = m_width * y + x;
        m_pixels[idx].r = static_cast<unsigned char>(c.getX() * 255.99f);
        m_pixels[idx].g = static_cast<unsigned char>(c.getY() * 255.99f);
        m_pixels[idx].b = static_cast<unsigned char>(c.getZ() * 255.99f);
    }

   private:
    int m_height;
    int m_width;
    std::unique_ptr<rgb[]> m_pixels;
    std::vector<std::unique_ptr<ImageFilter>> m_filters;
};

class Ray {
   public:
    Ray() {}
    Ray(const vec3 &o, const vec3 &dir) : m_origin(o), m_direction(dir) {}

    const vec3 &origin() const { return m_origin; }
    const vec3 &direction() const { return m_direction; }
    const vec3 at(double t) const { return m_origin + t * m_direction; }

   private:
    vec3 m_origin;
    vec3 m_direction;
};

class Camera {
   public:
    Camera(const vec3 &u, const vec3 &v, const vec3 &w) {
        m_origin = vec3(0);
        m_uvw[0] = u;
        m_uvw[1] = v;
        m_uvw[2] = w;
    }

    Camera(const vec3 &lookfrom, const vec3 &lookat, const vec3 &vup, double vfov, double aspect) {
        const vec3 Z = normalize(lookfrom - lookat);
        const vec3 X = normalize(cross(vup, Z));
        const vec3 Y = cross(Z, X);

        const double rad = radians(vfov);
        const double h = tan(rad / 2);
        const double w = h * aspect;

        m_uvw[0] = 2 * w * X;
        m_uvw[1] = 2 * h * Y;
        m_uvw[2] = lookfrom - w * X - h * Y - Z;
        m_origin = lookfrom;
    }

    Ray getRay(double u, double v) {
        return Ray(m_origin, m_uvw[0] * u + m_uvw[1] * v + m_uvw[2] - m_origin);
    }

   private:
    vec3 m_origin;
    vec3 m_uvw[3];
};

class Texture {
   public:
    virtual vec3 value(double u, double v, const vec3 &p) const = 0;
};

class ColorTexture : public Texture {
   public:
    ColorTexture(const vec3 &c) : m_color(c) {}

    vec3 value(double u, double v, const vec3 &p) const override {
        return m_color;
    }

   private:
    vec3 m_color;
};
using TexturePtr = std::shared_ptr<Texture>;

class CheckerTexture : public Texture {
   public:
    CheckerTexture(const TexturePtr &t0, const TexturePtr &t1, double freq) : m_odd(t0), m_even(t1), m_freq(freq) {}

    vec3 value(double u, double v, const vec3 &p) const override {
        double sines = sin(m_freq * p.getX()) * sin(m_freq * p.getY()) * sin(m_freq * p.getZ());

        if (sines < 0)
            return m_odd->value(u, v, p);
        else
            return m_even->value(u, v, p);
    }

   private:
    TexturePtr m_odd;
    TexturePtr m_even;
    double m_freq;
};

class ImageTexture : public Texture {
   public:
    ImageTexture(const char *name) {
        int nn;
        m_texels = stbi_load(name, &m_width, &m_height, &nn, 0);
    }
    ~ImageTexture() {
        stbi_image_free(m_texels);
    }

    vec3 value(double u, double v, const vec3 &p) const override {
        int i = u * m_width;
        int j = (1 - v) * m_height;
        return sample(i, j);
    }

    vec3 sample(int u, int v) const {
        u = u < 0 ? 0 : u >= m_width ? m_width - 1 : u;
        v = v < 0 ? 0 : v >= m_height ? m_height - 1 : v;

        return vec3(
            int(m_texels[3 * (u + m_width * v)]) / 255.0,
            int(m_texels[3 * (u + m_width * v)]) / 255.0,
            int(m_texels[3 * (u + m_width * v)]) / 255.0);
    }

   private:
    int m_width;
    int m_height;
    unsigned char *m_texels;
};

class HitRec {
   public:
    double t;
    double u;
    double v;
    vec3 p;
    vec3 n;
    MaterialPtr mat;
};

class ScatterRec {
   public:
    Ray ray;
    vec3 albedo;
    double pdf_value;
};

class Material {
   public:
    virtual bool scatter(const Ray &r, const HitRec &hrec, ScatterRec &srec) const = 0;
    virtual vec3 emitted(const Ray &r, const HitRec &hrec) const { return vec3(0); }
    virtual double scattering_pdf(const Ray &r, const HitRec &hrec) const { return 0; }
};
using MaterialPtr = std::shared_ptr<Material>;

class Lambertian : public Material {
   public:
    Lambertian(const TexturePtr &a) : m_albedo(a) {}

    bool scatter(const Ray &r, const HitRec &hrec, ScatterRec &srec) const override {
        ONB onb;
        onb.build_from_w(hrec.n);
        vec3 direction = onb.local(random_cosine_direction());
        direction = normalize(direction);  //uuuu
        srec.ray = Ray(hrec.p, direction);
        srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
        srec.pdf_value = dot(hrec.n, srec.ray.direction()) / PI;
        return true;
    }

    double scattering_pdf(const Ray &r, const HitRec &hrec) const override {
        return std::max(dot(normalize(r.direction()), hrec.n), 0.0f) / PI;
    }

   private:
    TexturePtr m_albedo;
};

class Metal : public Material {
   public:
    Metal(const TexturePtr &a, double fuzz) : m_albedo(a), m_fuzz(fuzz) {}

    bool scatter(const Ray &r, const HitRec &hrec, ScatterRec &srec) const override {
        vec3 reflected = reflect(normalize(r.direction()), hrec.n);
        reflected += m_fuzz * random_in_unit_sphere();
        srec.ray = Ray(hrec.p, reflected);
        srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
        return true;
    }

   private:
    TexturePtr m_albedo;
    double m_fuzz;
};

class Dielectric : public Material {
   public:
    Dielectric(double ri) : m_ri(ri) {}

    bool scatter(const Ray &r, const HitRec &hrec, ScatterRec &srec) const override {
        vec3 outward_normal;
        vec3 reflected = reflect(r.direction(), hrec.n);
        double ni_over_nt;
        double reflect_prob;
        double cosine;

        if (dot(r.direction(), hrec.n) > EPS) {
            outward_normal = -hrec.n;
            ni_over_nt = m_ri;
        } else {
            outward_normal = hrec.n;
            ni_over_nt = recip(m_ri);
        }
        cosine = abs(dot(r.direction(), hrec.n)) / length(r.direction());

        srec.albedo = vec3(1);

        vec3 refracted;
        if (refract(-r.direction(), outward_normal, ni_over_nt, refracted)) {
            reflect_prob = schlick(cosine, m_ri);
        } else {
            reflect_prob = 1;
        }

        if (drand48() < reflect_prob) {
            srec.ray = Ray(hrec.p, reflected);
        } else {
            srec.ray = Ray(hrec.p, refracted);
        }
        return true;
    }

   private:
    double m_ri;
};

class DiffuseLight : public Material {
   public:
    DiffuseLight(const TexturePtr &emit) : m_emit(emit) {}

    bool scatter(const Ray &r, const HitRec &hrec, ScatterRec &srec) const override {
        return false;
    }

    vec3 emitted(const Ray &r, const HitRec &hrec) const override {
        return m_emit->value(hrec.u, hrec.v, hrec.p);
    }

   private:
    TexturePtr m_emit;
};

class Shape {
   public:
    virtual bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const = 0;
};

using ShapePtr = std::shared_ptr<Shape>;

class Sphere : public Shape {
   public:
    Sphere() {}
    Sphere(const vec3 &c, double r, MaterialPtr mat) : m_center(c), m_radius(r), m_material(mat) {}

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        vec3 oc = r.origin() - m_center;
        double a = dot(r.direction(), r.direction());
        double b = 2 * dot(oc, r.direction());
        double c = dot(oc, oc) - m_radius * m_radius;

        double D = b * b - 4 * a * c;
        if (D < EPS)
            return false;

        double root = sqrt(D);
        double tmp = (-b - root) / (2 * a);
        if (tmp + EPS < t1 && t0 + EPS < tmp) {
            hrec.t = tmp;
            hrec.p = r.at(hrec.t);
            hrec.n = (hrec.p - m_center) / m_radius;
            hrec.mat = m_material;
            get_sphere_uv(hrec.p - m_center, hrec.u, hrec.v);
            return true;
        }

        tmp = (-b + root) / (2 * a);
        if (tmp < t0 + EPS || t1 < tmp + EPS)
            return false;
        hrec.t = tmp;
        hrec.p = r.at(hrec.t);
        hrec.n = (hrec.p - m_center) / m_radius;
        hrec.mat = m_material;
        get_sphere_uv(hrec.p - m_center, hrec.u, hrec.v);
        return true;
    }

   private:
    vec3 m_center;
    double m_radius;
    MaterialPtr m_material;
};

class Rect : public Shape {
   public:
    enum AxisType {
        kXY = 0,
        kXZ,
        kYZ
    };

    Rect() {}
    Rect(double x0, double x1, double y0, double y1, double k, AxisType axis, const MaterialPtr &m) : m_x0(x0),
                                                                                                      m_x1(x1),
                                                                                                      m_y0(y0),
                                                                                                      m_y1(y1),
                                                                                                      m_k(k),
                                                                                                      m_axis(axis),
                                                                                                      m_material(m) {}

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        int xi, yi, zi;
        vec3 axis;
        switch (m_axis) {
            case kXY:
                xi = 0;
                yi = 1;
                zi = 2;
                axis = vec3::zAxis();
                break;
            case kXZ:
                xi = 0;
                yi = 2;
                zi = 1;
                axis = vec3::yAxis();
                break;
            case kYZ:
                xi = 1;
                yi = 2;
                zi = 0;
                axis = vec3::xAxis();
                break;
        }

        double t = (m_k - r.origin()[zi]) / r.direction()[zi];
        if (t < t0 + EPS || t1 < t + EPS)
            return false;
        double x = r.origin()[xi] + t * r.direction()[xi];
        double y = r.origin()[yi] + t * r.direction()[yi];

        if (x < m_x0 || m_x1 < x || y < m_y0 || m_y1 < y)
            return false;

        hrec.u = (x - m_x0) / (m_x1 - m_x0);
        hrec.v = (y - m_y0) / (m_y1 - m_y0);
        hrec.t = t;
        hrec.mat = m_material;
        hrec.p = r.at(t);
        hrec.n = axis;
        return true;
    }

   private:
    double m_x0, m_x1, m_y0, m_y1, m_k;
    AxisType m_axis;
    MaterialPtr m_material;
};

class ShapeList : public Shape {
   public:
    ShapeList() {}

    void add(const ShapePtr &shape) {
        m_list.push_back(shape);
    }

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        HitRec tmp_hrec;
        bool hit_anything = false;
        double closest_so_far = t1;
        for (auto &p : m_list) {
            if (p->hit(r, t0, closest_so_far, tmp_hrec)) {
                hit_anything = true;
                closest_so_far = tmp_hrec.t;
                hrec = tmp_hrec;
            }
        }
        return hit_anything;
    }

   private:
    std::vector<ShapePtr> m_list;
};

class FlipNormals : public Sphere {
   public:
    FlipNormals(const ShapePtr &shape) : m_shape(shape) {}

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        if (m_shape->hit(r, t0, t1, hrec)) {
            hrec.n = -hrec.n;
            return true;
        } else {
            return false;
        }
    }

   private:
    ShapePtr m_shape;
};

class Translate : public Shape {
   public:
    Translate(const ShapePtr &sp, const vec3 &displacement) : m_shape(sp),
                                                              m_offset(displacement) {}

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        Ray move_r(r.origin() - m_offset, r.direction());
        if (m_shape->hit(move_r, t0, t1, hrec)) {
            hrec.p += m_offset;
            return true;
        }
        return false;
    }

   private:
    ShapePtr m_shape;
    vec3 m_offset;
};

class Rotate : public Shape {
   public:
    Rotate(const ShapePtr &sp, const vec3 &axis, double angle) : m_shape(sp),
                                                                 m_quat(Quat::rotation(radians(angle), axis)) {}

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        Quat revq = conj(m_quat);

        vec3 origin = rotate(revq, r.origin());
        vec3 direction = rotate(revq, r.direction());

        Ray rot_r(origin, direction);

        if (m_shape->hit(rot_r, t0, t1, hrec)) {
            hrec.n = rotate(m_quat, hrec.n);
            hrec.p = rotate(m_quat, hrec.p);
            return true;
        }
        return false;
    }

   private:
    ShapePtr m_shape;
    Quat m_quat;
};

class Box : public Shape {
   public:
    Box() {}
    Box(const vec3 &p0, const vec3 &p1, const MaterialPtr &m) : m_p0(p0), m_p1(p1), m_list(std::make_unique<ShapeList>()) {
        ShapeList *l = new ShapeList();
        l->add(
            std::make_shared<Rect>(
                p0.getX(), p1.getX(), p0.getY(), p1.getY(), p1.getZ(), Rect::kXY, m));

        l->add(
            std::make_shared<FlipNormals>(
                std::make_shared<Rect>(
                    p0.getX(), p1.getX(), p0.getY(), p1.getY(), p0.getZ(), Rect::kXY, m)));

        l->add(
            std::make_shared<Rect>(
                p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p1.getY(), Rect::kXZ, m));

        l->add(
            std::make_shared<FlipNormals>(
                std::make_shared<Rect>(
                    p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p0.getY(), Rect::kXZ, m)));

        l->add(
            std::make_shared<Rect>(
                p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p1.getX(), Rect::kYZ, m));

        l->add(
            std::make_shared<FlipNormals>(
                std::make_shared<Rect>(
                    p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p0.getX(), Rect::kYZ, m)));
        m_list.reset(l);
    }

    bool hit(const Ray &r, double t0, double t1, HitRec &hrec) const override {
        return m_list->hit(r, t0, t1, hrec);
    }

   private:
    vec3 m_p0, m_p1;
    std::unique_ptr<ShapeList> m_list;
};

class ShapeBuilder {
   public:
    ShapeBuilder() {}
    ShapeBuilder(const ShapePtr &sp)
        : m_ptr(sp) {
    }

    ShapeBuilder &reset(const ShapePtr &sp) {
        m_ptr = sp;
        return *this;
    }

    ShapeBuilder &sphere(const vec3 &c, float r, const MaterialPtr &m) {
        m_ptr = std::make_shared<Sphere>(c, r, m);
        return *this;
    }

    ShapeBuilder &rect(float x0, float x1, float y0, float y1, float k, Rect::AxisType axis, const MaterialPtr &m) {
        m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, axis, m);
        return *this;
    }
    ShapeBuilder &rectXY(float x0, float x1, float y0, float y1, float k, const MaterialPtr &m) {
        m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXY, m);
        return *this;
    }
    ShapeBuilder &rectXZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr &m) {
        m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXZ, m);
        return *this;
    }
    ShapeBuilder &rectYZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr &m) {
        m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kYZ, m);
        return *this;
    }

    ShapeBuilder &rect(const vec3 &p0, const vec3 &p1, float k, Rect::AxisType axis, const MaterialPtr &m) {
        switch (axis) {
            case Rect::kXY:
                m_ptr = std::make_shared<Rect>(
                    p0.getX(), p1.getX(), p0.getY(), p1.getY(), k, axis, m);
                break;
            case Rect::kXZ:
                m_ptr = std::make_shared<Rect>(
                    p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), k, axis, m);
                break;
            case Rect::kYZ:
                m_ptr = std::make_shared<Rect>(
                    p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), k, axis, m);
                break;
        }
        return *this;
    }
    ShapeBuilder &rectXY(const vec3 &p0, const vec3 &p1, float k, const MaterialPtr &m) {
        return rect(p0, p1, k, Rect::kXY, m);
    }
    ShapeBuilder &rectXZ(const vec3 &p0, const vec3 &p1, float k, const MaterialPtr &m) {
        return rect(p0, p1, k, Rect::kXZ, m);
    }
    ShapeBuilder &rectYZ(const vec3 &p0, const vec3 &p1, float k, const MaterialPtr &m) {
        return rect(p0, p1, k, Rect::kYZ, m);
    }

    ShapeBuilder &box(const vec3 &p0, const vec3 &p1, const MaterialPtr &m) {
        m_ptr = std::make_shared<Box>(p0, p1, m);
        return *this;
    }

    ShapeBuilder &flip() {
        m_ptr = std::make_shared<FlipNormals>(m_ptr);
        return *this;
    }

    ShapeBuilder &translate(const vec3 &t) {
        m_ptr = std::make_shared<Translate>(m_ptr, t);
        return *this;
    }

    ShapeBuilder &rotate(const vec3 &axis, float angle) {
        m_ptr = std::make_shared<Rotate>(m_ptr, axis, angle);
        return *this;
    }

    const ShapePtr &get() const { return m_ptr; }

   private:
    ShapePtr m_ptr;
};

class Scene {
   public:
    Scene(int width, int height, int samples) : m_image(std::make_unique<Image>(width, height)), m_backColor(0.2), m_samples(samples) {}
    void build() {
        m_backColor = vec3(0);

        // Camera

        vec3 lookfrom(278, 278, -800);
        vec3 lookat(278, 278, 0);
        vec3 vup(0, 1, 0);
        float aspect = float(m_image->width()) / float(m_image->height());
        m_camera = std::make_unique<Camera>(lookfrom, lookat, vup, 40, aspect);

        // Shapes

        MaterialPtr red = std::make_shared<Lambertian>(
            std::make_shared<ColorTexture>(vec3(0.65f, 0.05f, 0.05f)));
        MaterialPtr white = std::make_shared<Lambertian>(
            std::make_shared<ColorTexture>(vec3(0.73f)));
        MaterialPtr green = std::make_shared<Lambertian>(
            std::make_shared<ColorTexture>(vec3(0.12f, 0.45f, 0.15f)));
        MaterialPtr light = std::make_shared<DiffuseLight>(
            std::make_shared<ColorTexture>(vec3(15.0f)));

        ShapeList *world = new ShapeList();
        ShapeBuilder builder;
        world->add(builder.rectYZ(0, 555, 0, 555, 555, green).flip().get());
        world->add(builder.rectYZ(0, 555, 0, 555, 0, red).get());
        world->add(builder.rectXZ(213, 343, 227, 332, 554, light).get());
        world->add(builder.rectXZ(0, 555, 0, 555, 555, white).flip().get());
        world->add(builder.rectXZ(0, 555, 0, 555, 0, white).get());
        world->add(builder.rectXY(0, 555, 0, 555, 555, white).flip().get());

        world->add(builder.box(vec3(0), vec3(165), white)
                       .rotate(vec3::yAxis(), -18)
                       .translate(vec3(130, 0, 65))
                       .get());

        world->add(builder.box(vec3(0), vec3(165, 330, 165), white)
                       .rotate(vec3::yAxis(), 15)
                       .translate(vec3(265, 0, 295))
                       .get());

        m_world.reset(world);
    }

    vec3 color(const rayt::Ray &r, const Shape *world, int depth) const {
        HitRec hrec;
        if (world->hit(r, 0.001, INF, hrec)) {
            vec3 emitted = hrec.mat->emitted(r, hrec);
            ScatterRec srec;

            if (depth < MAX_DEPTH && hrec.mat->scatter(r, hrec, srec) && srec.pdf_value > 0) {
                vec3 albedo = srec.albedo * hrec.mat->scattering_pdf(srec.ray, hrec);
                return emitted + mulPerElem(albedo, color(srec.ray, world, depth + 1)) / srec.pdf_value;
            } else {
                return emitted;
            }
        }
        return background(r.direction());
    }

    vec3 backgroundSky(const vec3 &d) const {
        vec3 v = normalize(d);
        double t = 0.5 * (v.getY() + 1.0);
        return lerp(t, vec3(1), vec3(0.5, 0.7, 1.0));
    }

    vec3 background(const vec3 &d) const {
        return m_backColor;
    }

    void render() {
        build();
        int W = m_image->width();
        int H = m_image->height();
#pragma omp parallel for schedule(dynamic, 1) num_threads(NUM_THREAD)
        for (int y = 0; y < H; y++) {
            std::cout << y << std::endl;
            for (int x = 0; x < W; x++) {
                vec3 c(0);
                for (int s = 0; s < m_samples; s++) {
                    double u = (x + drand48()) / W;
                    double v = (y + drand48()) / H;
                    Ray r = m_camera->getRay(u, v);
                    c += color(r, m_world.get(), 0);
                }
                c /= m_samples;
                m_image->write(x, H - 1 - y, c.getX(), c.getY(), c.getZ());
            }
        }

        stbi_write_png("render.png", W, H, sizeof(Image::rgb), m_image->pixels(), W * sizeof(Image::rgb));
    }

   private:
    std::unique_ptr<Camera> m_camera;
    std::unique_ptr<Image> m_image;
    std::unique_ptr<Shape> m_world;
    vec3 m_backColor;
    int m_samples;
};
}  // namespace rayt