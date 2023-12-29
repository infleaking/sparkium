#pragma once
#include "random"
#include "sparks/assets/scene.h"
#include "sparks/renderer/renderer_settings.h"

namespace sparks {
class PathTracer {
 public:
  PathTracer(const RendererSettings *render_settings, const Scene *scene);
  [[nodiscard]] glm::vec3 SampleRay(glm::vec3 origin,
                                    glm::vec3 direction,
                                    int x,
                                    int y,
                                    int sample) const;
  [[nodiscard]] glm::vec3 PathTracing(glm::vec3 origin,
                                    glm::vec3 direction,
                                    int x,
                                    int y,
                                    int sample) ;
  [[nodiscard]] glm::vec3 TracePath(glm::vec3 origin,
                                    glm::vec3 direction,
                                    int x,
                                    int y,
                                    int sample) ;
  [[nodiscard]] glm::vec3 shade(glm::vec3 origin,
                                    glm::vec3 direction,
                                    int x,
                                    int y,
                                    int sample,
                                    std::mt19937 &rd) const;
  [[nodiscard]] glm::vec3 SampleLight(float &pdf, glm::vec3 &sampled, glm::vec3 &normal, std::mt19937 &rd);

  unsigned seed=0; 
 private:
  const RendererSettings *render_settings_{};
  const Scene *scene_{};
  std::vector<int> lightid;
};
}  // namespace sparks
