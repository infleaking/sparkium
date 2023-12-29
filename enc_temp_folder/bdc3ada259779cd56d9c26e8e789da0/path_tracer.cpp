#include "sparks/renderer/path_tracer.h"

#include "sparks/util/util.h"

#include <glm/ext.hpp>

std::ostream& operator<<(std::ostream& os, const glm::vec3& vector3) {
    os << "glm::vec3 values: " << vector3.x << ", " << vector3.y << ", " << vector3.z;
    return os;
}
namespace sparks {
PathTracer::PathTracer(const RendererSettings *render_settings,
                       const Scene *scene) {
    render_settings_ = render_settings;
    scene_ = scene;
    seed = 0;
    lightid.clear();
    auto &entities = scene_->GetEntities();
    for (int i=0;i<entities.size();i++){
        // std::cerr<<entities[i].GetName()<<std::endl;
        // std::cerr<<entities[i].GetMaterial().material_type<<std::endl;
        if (entities[i].GetMaterial().material_type != MATERIAL_TYPE_EMISSION) continue;
        lightid.push_back(i);
    }  
}
glm::vec3 sample_on_direction(std::mt19937 &rd){
    glm::vec3 randvec{1.0f};
    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
    float r1 = distribution(rd) * PI * 2;
    float r2 = distribution(rd);
    // std::cerr<< randvec << std::endl;
    return glm::normalize( glm::vec3(sin(r1)*sqrt(1-r2), cos(r1)*sqrt(1-r2), sqrt(r2)));
}
glm::vec3 sample_on_square(const Entity &entity, std::mt19937 &rd){
    auto vertices = entity.GetModel()->GetVertices();
    // std::cerr<< (vertices.size())<<" vectorsize"<< std::endl;
    assert(vertices.size()==6);
    float a = std::uniform_real_distribution<float>(0.0f, 1.0f)(rd);
    float b = std::uniform_real_distribution<float>(0.0f, 1.0f)(rd);

    return a*b*vertices[0].position+a*(1-b)*vertices[1].position+(1-a)*(1-b)*vertices[5].position+(1-a)*b*vertices[2].position;

}
float area_of_square(const Entity &entity){
    auto vertices = entity.GetModel()->GetVertices();
    glm::vec3 u = vertices[0].position, v = vertices[5].position;
    float x = abs(u.x-v.x), y=abs(u.y-v.y), z=abs(u.z-v.z);
    if (x<z) std::swap(x,z);
    if (y<z) std::swap(y,z);
    return x*y;
}
int randInt(int n, std::mt19937 &rd){
    return (int)std::uniform_int_distribution<int>(0,n-1)(rd);
}
float randFloat(std::mt19937 &rd){
    return std::uniform_real_distribution<float>(0.0f,1.0f)(rd);
}
glm::vec3 PathTracer::SampleLight(float &pdf, glm::vec3 &sampled, glm::vec3 &normal, std::mt19937 &rd) {
    int num_ls = lightid.size();
    auto &entities = scene_->GetEntities();
    auto &obj = scene_->GetEntity(lightid[randInt(num_ls, rd)]);
    // auto &obj_ = (scene_->GetEntity(randInt(num_ls, rd))).GetModel()->GetVertices();
    auto vertex = (obj.GetModel())->GetVertices();
    int num_surface = vertex.size()/3, sampleid = 3*randInt(num_surface, rd);
    glm::vec3 v0 = vertex[sampleid].position, v1 = vertex[sampleid+1].position, v2=vertex[sampleid+2].position;
    float x = std::uniform_real_distribution<float>(0.0f,1.0f)(rd), y = std::uniform_real_distribution<float>(0.0f,1.0f)(rd);
    if (x+y>1) {
        x = 1-y;
        y = 1-x;
    }
    sampled = v0 + x*(v1-v0) + y*(v2-v0);
    normal = (glm::cross(v1-v0,v2-v0));
    float area = glm::length(normal) * 0.5;
    normal = glm::normalize (normal);
    pdf = 1.0/num_ls/num_surface/area;
    Material material = obj.GetMaterial();
    return material.emission * material.emission_strength / pdf;
}

float objbsdf(Material material, glm::vec3 normal, glm::vec3 d_in, glm::vec3 d_out, glm::vec3 &f) {
    f = glm::vec3{0.0f};
    if (material.material_type == MATERIAL_TYPE_LAMBERTIAN){
        float pdf = glm::dot(normal, d_out) * INV_PI;
        f = material.albedo_color * INV_PI * glm::dot (normal, d_out);
        return pdf;
    }
    return 0;
}

glm::vec3 PathTracer::TracePath(glm::vec3 origin, glm::vec3 direction, int x, int y, int sample) {
    glm::vec3 radiance{0.0f}, throughput{1.0f};
    HitRecord hit_record;
    const int max_bounce = render_settings_->num_bounces;
    seed = seed ^ x ^ y ^ sample;
    std::mt19937 rd(seed);

    const float rr = 0.95;
    float lightratio = 1.0;
    for (int i = 0; ; i++){
        float drop = std::uniform_real_distribution<float>(0.0f, 1.0f)(rd);
        if (drop > rr)break;
        throughput /= rr;
        auto t = scene_->TraceRay(origin, direction, 1e-3f, 1e4f, &hit_record);
        if (t < 0.0f){
            radiance += throughput * glm::vec3{scene_->SampleEnvmap(direction)};
            break;
        }
        glm::vec3 N = hit_record.geometry_normal;
        auto &material =
            scene_->GetEntity(hit_record.hit_entity_id).GetMaterial();
        if (material.material_type == MATERIAL_TYPE_EMISSION) {
            radiance += lightratio * throughput * material.emission * material.emission_strength;
            break;
        }
        if (material.material_type == MATERIAL_TYPE_SPECULAR) {
            direction -= 2.0f * glm::dot(N, direction) * N;
            origin = hit_record.position;
            lightratio = 1;
            continue;
        }
        glm::vec3 sampled, light_normal, light_strenth;
        float light_pdf;
        light_strenth = SampleLight(light_pdf, sampled, light_normal, rd);
        glm::vec3 tolight = glm::normalize (sampled - hit_record.position);
        HitRecord raytolight;
        auto t1 = scene_->TraceRay(hit_record.position, tolight, 1e-3f, 1e4f, &raytolight);
        if (glm::length(sampled - raytolight.position) > 1e-4){
            light_strenth = glm::vec3{0.0f};
            light_pdf = 0;
            // std::cerr<< hit_record.position << '-'<<tolight<<'-'<< t1 <<'-'<< raytolight.position<<std::endl;
        }
        else {      
            // std::cerr<<throughput<<' '<<light_strenth<<' '<<material.albedo_color<<' '<<glm::dot(N, tolight)<<std::endl;
          
            float cosine_ = abs(glm::dot(light_normal, tolight)), dist = glm::dot (sampled - hit_record.position, sampled - hit_record.position);
            light_strenth *= cosine_ / dist;
            light_pdf /= cosine_ / dist;
        }
        if (material.material_type == MATERIAL_TYPE_LAMBERTIAN) {
            lightratio = 0;
            radiance += throughput * light_strenth * material.albedo_color * glm::dot(N, tolight) * INV_PI;
            glm::vec3 relative_dir = sample_on_direction(rd);
            glm::vec3 x = glm::cross(N, glm::vec3{0.0f,1.0f,0.0f});
            if (abs(glm::dot(N, glm::vec3{0.0f,1.0f,0.0f})) > 0.9)
                x = glm::cross(N, glm::vec3{1.0f,0.0f,1.0f});
            x = glm::normalize(x);
            glm::vec3 y = glm::normalize(glm::cross(N,x));
            glm::vec3 outdir = x * relative_dir.x + y * relative_dir.y + N * relative_dir.z;
            throughput *= glm::dot(N, glm::normalize(outdir)) * material.albedo_color / relative_dir.z;
            origin = hit_record.position;
            direction = outdir;
            // TODO
        }
    }
    seed ^= rd();
    // std::cerr<<radiance<<'\n';
    return radiance;
}

glm::vec3 PathTracer::SampleRay(glm::vec3 origin,
                                glm::vec3 direction,
                                int x,
                                int y,
                                int sample) const {
    // return glm::vec3{100.0f};
  glm::vec3 throughput{1.0f};
  glm::vec3 radiance{0.0f};
  HitRecord hit_record;
  const int max_bounce = render_settings_->num_bounces;
//   static std::mt19937 rd(sample ^ x ^ y);
  for (int i = 0; i < max_bounce; i++) {
    auto t = scene_->TraceRay(origin, direction, 1e-3f, 1e4f, &hit_record);
    if (t > 0.0f) {
      auto &material =
          scene_->GetEntity(hit_record.hit_entity_id).GetMaterial();
      if (material.material_type == MATERIAL_TYPE_EMISSION) {
        radiance += throughput * material.emission * material.emission_strength;
        break;
      } else {
        throughput *=
            material.albedo_color *
            glm::vec3{scene_->GetTextures()[material.albedo_texture_id].Sample(
                hit_record.tex_coord)};
        origin = hit_record.position;
        direction = scene_->GetEnvmapLightDirection();
        radiance += throughput * scene_->GetEnvmapMinorColor();
        // std::cerr<< glm::dot(direction, hit_record.geometry_normal) << ' ' << glm::dot(direction, hit_record.normal)<< std::endl;
        // if (glm::dot(direction, hit_record.geometry_normal)>0){
        //     std::cerr<<hit_record.hit_entity_id<<std::endl;
        // }
        // assert(glm::dot(direction, hit_record.geometry_normal)<=0);
        // if (scene_->GetEntity(hit_record.hit_entity_id).GetName()=="Bunny"){
        //     std::cerr<< hit_record.normal.x<<' '<< hit_record.normal.y <<' '<< hit_record.normal.z<<std::endl;
        //     std::cerr<< direction.x<<' '<< direction.y <<' '<< direction.z<<std::endl;
        //     std::cerr<< glm::dot(direction, hit_record.normal) <<std::endl;
        //     std::cerr<< x<<' '<<y<<std::endl;
        // }
        throughput *=
            std::max(glm::dot(direction, hit_record.normal), 0.0f) * 2.0f;
        auto t = scene_->TraceRay(origin, direction, 1e-3f, 1e4f, &hit_record);
        // if (scene_->GetEntity(hit_record.hit_entity_id).GetName() == "Ceiling") {
        //   radiance += throughput * scene_->GetEnvmapMajorColor();
        //   std::cerr<< x<<' '<<y<<std::endl;
        //   std::cerr<< scene_->GetEnvmapMinorColor() << std::endl;
        //   std::cerr<< scene_->GetEnvmapMajorColor() << std::endl;

        // }
        break;
      }
    } else {
      radiance += throughput * glm::vec3{scene_->SampleEnvmap(direction)};
      break;
    }
  }
  return radiance;
}
}  // namespace sparks
