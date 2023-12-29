#include "sparks/renderer/path_tracer.h"

#include "sparks/util/util.h"

#include <glm/ext.hpp>

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
std::ostream& operator<<(std::ostream& os, const glm::vec3& vector3) {
    os << "glm::vec3 values: " << vector3.x << ", " << vector3.y << ", " << vector3.z;
    return os;
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
void PathTracer::SampleLight(glm::vec3 origin, glm::vec3 normal, glm::vec3 &dir, glm::vec3 &eval, float &pdf, std::mt19937 &rd) {
    int num_ls = lightid.size();
    auto &entities = scene_->GetEntities();
    auto &obj = scene_->GetEntity(lightid[randInt(num_ls, rd)]);
    // auto &obj_ = (scene_->GetEntity(randInt(num_ls, rd))).GetModel()->GetVertices();
    auto vertex = (obj.GetModel())->GetVertices();
    int num_surface = vertex.size()/3, sampleid = 3*randInt(num_surface, rd);
    glm::vec3 v0 = vertex[sampleid].position, v1 = vertex[sampleid+1].position, v2=vertex[sampleid+2].position;

    glm::vec3 mesh_normal = (cross(v1-v0,v2-v0));
    float area = length(mesh_normal) / 2;
    mesh_normal = glm::normalize(mesh_normal);
    float r0 = randFloat(rd), r1 = randFloat(rd), r2;
    if (r0+r1>1){
        r0=1-r0;
        r1=1-r1;
    }
    r2=1-r0-r1;
    glm::vec3 position = r0*v0 +r1*v1 +r2*v2;
    dir =normalize( position - origin);
    HitRecord hit_record;
    auto t = scene_->TraceRay(origin, dir, 1e-3f, 1e4f, &hit_record);
    if (dot (normal, dir) > 0 && t > 1e-4 && glm::length(position - hit_record.position)<1e-4){
        Material m = obj.GetMaterial();
        float proj = -glm::dot(hit_record.geometry_normal, dir), dist = glm::dot (position - origin, position - origin);
        pdf = 1.0/num_ls/num_surface/area/proj * dist;
        eval = m.emission * m.emission_strength * (num_ls * num_surface * area * proj / dist);
    }
    else {
        eval = glm::vec3{0.0f};
        pdf = 0;
    }
}

glm::vec3 PathTracer::shade(glm::vec3 origin, glm::vec3 direction, int x, int y, int sample, std::mt19937 &rd) const{
    glm::vec3 throughput{1.0f};
    glm::vec3 radiance{0.0f};
    HitRecord hit_record, hit_light_record;
    const int max_bounce = render_settings_->num_bounces;
    int i;
    float rr=0.95;
    auto &entities = scene_->GetEntities();
    for (i = 0; ; i++) {
        auto t = scene_->TraceRay(origin, direction, 1e-3f, 1e4f, &hit_record);
        
        float drop = std::uniform_real_distribution<float>(0.0f, 1.0f)(rd);
        if (drop > rr)break;
        if (t > 0.0f) {
            auto &material =
                scene_->GetEntity(hit_record.hit_entity_id).GetMaterial();
            if (material.material_type == MATERIAL_TYPE_LAMBERTIAN) {
                glm::vec3 location = hit_record.position, normal = hit_record.geometry_normal, light_direction = scene_->GetEnvmapLightDirection();
                glm::vec3 outdir = sample_on_direction(rd);
                if (glm::dot(outdir, normal)<0) outdir = -outdir;
                throughput *=
                    material.albedo_color *
                    glm::vec3{scene_->GetTextures()[material.albedo_texture_id].Sample(
                        hit_record.tex_coord)};
                glm::vec3 L_dir = glm::vec3{0.0f};
                
                for (int j = 0; j < entities.size(); j++){
                    if (entities[j].GetMaterial().material_type != MATERIAL_TYPE_EMISSION) continue;
                    glm::vec3 sample = sample_on_square(entities[j], rd);
                    glm::vec3 tolight = glm::normalize(sample - location);

                    auto &lightobj = entities[j].GetMaterial();
                    auto t = scene_->TraceRay(location, tolight, 1e-3f, 1e4f, &hit_light_record);
                    float dist = glm::dot(sample - location, sample - location);
                    // std::cerr<<entities[ hit_record.hit_entity_id].GetName()<<' '<<entities[j].GetName()<<' '<< std::max(glm::dot(tolight, hit_record.normal), 0.0f)<< ' '<<std::max(-glm::dot(tolight, hit_light_record.normal), 0.0f)<<lightobj.emission * lightobj.emission_strength<<std::endl<< \
                    //  tolight << "---1" << hit_record.normal <<"---2"<< hit_light_record.normal<<"---3"<<sample<<"---4"<<location<<"---5"<<hit_light_record.geometry_normal<<"---6"<<hit_record.geometry_normal<< std::endl;
                    if (t > 0 && hit_light_record.hit_entity_id == j){
                        L_dir += throughput * std::max(glm::dot(tolight, hit_record.geometry_normal), 0.0f) *std::max(-glm::dot(tolight, hit_light_record.geometry_normal), 0.0f) * lightobj.emission * lightobj.emission_strength / dist *area_of_square(entities[j]) ;
                    }
                }
                radiance += L_dir ;
                // if (scene_->TraceRay(origin, light_direction, 1e-3f, 1e4f, nullptr) < 0.0f) {
                //     L_dir = throughput * std::max(glm::dot(light_direction, hit_record.normal), 0.0f) * 2.0f * scene_->GetEnvmapMajorColor();
                //     radiance += L_dir;
                // }
                throughput *= glm::dot (outdir, hit_record.geometry_normal) / (2* 3.1415926535);
                origin = hit_record.position;
                direction = outdir;
                t = scene_->TraceRay(origin, direction, 1e-3f, 1e4f, &hit_record);
                if (t > 0.0f)
                    if (scene_->GetEntity(hit_record.hit_entity_id).GetMaterial().material_type == MATERIAL_TYPE_EMISSION)
                        break;
            }

            else if (material.material_type == MATERIAL_TYPE_EMISSION) {
                radiance += throughput * material.emission * material.emission_strength * glm::dot(-direction, hit_record.geometry_normal);
                break;
            } else if (material.material_type == MATERIAL_TYPE_SPECULAR){
                glm::vec3 location = hit_record.position, normal = hit_record.geometry_normal, light_direction = scene_->GetEnvmapLightDirection();
                glm::vec3 reflect = 2*glm::dot(normal, -direction)*normal +direction;
                origin = location;
                direction = reflect;
                throughput *= material.albedo_color;
                // throughput *=
                //     material.albedo_color *
                //     glm::vec3{scene_->GetTextures()[material.albedo_texture_id].Sample(
                //         hit_record.tex_coord)};
                // origin = hit_record.position;
                // direction = scene_->GetEnvmapLightDirection();
                // radiance += throughput * scene_->GetEnvmapMinorColor();
                // throughput *=
                //     std::max(glm::dot(direction, hit_record.normal), 0.0f) * 2.0f;
                // if (scene_->TraceRay(origin, direction, 1e-3f, 1e4f, nullptr) < 0.0f) {
                //     radiance += throughput * scene_->GetEnvmapMajorColor();
                // }
            }
            else break;
        } else {
            radiance += throughput * glm::vec3{scene_->SampleEnvmap(direction)};
            break;
        }
    }
    // if (i>=1){
    //     std::cerr<<i<<' '<<radiance<<std::endl;
    // }
    return radiance/ rr;
    // return glm::vec3{0.0f};
    // glm::vec3 shift{radiance.y, radiance.z, radiance.x};
    // return shift;
}
glm::vec3 PathTracer::PathTracing(glm::vec3 origin, glm::vec3 direction, int x, int y, int sample) {
    glm::vec3 radiance_sum{0.0f};
    HitRecord hit_record;
    const int max_bounce = render_settings_->num_bounces, num_samples = 1;
    seed = seed ^ x ^ y ^ sample;
    std::mt19937 rd(seed);
    for (int _ = 0; _ < num_samples; _ ++){
        radiance_sum += shade(origin, direction, x, y, sample, rd);
    }
    seed ^= rd();
    return radiance_sum /(1.0f * num_samples);

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
        auto &material =
            scene_->GetEntity(hit_record.hit_entity_id).GetMaterial();
        if (material.material_type == MATERIAL_TYPE_EMISSION) {
            radiance += lightratio * throughput * material.emission * material.emission_strength;
            break;
        }
        glm::vec3 N = hit_record.geometry_normal;
        glm::vec3 light, obj, tolight;
        float pdf, brds_pdf;
        SampleLight(hit_record.position, N, tolight, light, pdf, rd);
        brds_pdf = objbsdf(material, N, direction, tolight, obj);
        // std::cerr<<x<<" "<<y<<' '<<scene_->GetEntity(hit_record.hit_entity_id).GetName()<<'\n';
        if (material.material_type == MATERIAL_TYPE_LAMBERTIAN){
            pdf = 1;
            brds_pdf = 0;
        }
        if (pdf > 1e-3){
            radiance += throughput * light *obj * (pdf * pdf )/ (pdf*pdf +brds_pdf*brds_pdf);
            lightratio = 1-(pdf * pdf )/ (pdf*pdf +brds_pdf*brds_pdf);
        }
        else lightratio = 1;
        // std::cerr<<material.material_type<<' '<<lightratio<<' '<<pdf<<' '<<brds_pdf<<' '<<light<<' '<<obj <<'\n';
        if (material.material_type == MATERIAL_TYPE_SPECULAR) {
            direction -= 2.0f * glm::dot(N, direction) * N;
            origin = hit_record.position;
            lightratio = 1;
        }
        else if (material.material_type == MATERIAL_TYPE_LAMBERTIAN) {
            glm::vec3 relative_dir = sample_on_direction(rd);
            glm::vec3 x{N.z - N.y, N.x - N.z, N.y - N.x};
            if (glm::length(x) < 1e-5f)
                x = glm::vec3(N.z - N.y, N.x + N.z, -N.y - N.x);
            x = glm::normalize(x);
            glm::vec3 y = glm::normalize(glm::cross(N,x));
            glm::vec3 outdir = x * relative_dir.x + y * relative_dir.y + N * relative_dir.z;
            float pdf = relative_dir.z * INV_PI;
            throughput *= glm::dot(N, glm::normalize(outdir)) * INV_PI * material.albedo_color / pdf;
            origin = hit_record.position;
            direction = outdir;
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
