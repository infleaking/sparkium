#pragma once
#include "sparks/assets/model.h"
#include "sparks/assets/util.h"
#include "sparks/assets/vertex.h"
#include "vector"

namespace sparks {
struct kdtnode{
    AxisAlignedBoundingBox bbox;
    uint32_t l,r;
    kdtnode(const AxisAlignedBoundingBox& boundingBox, uint32_t leftChild, uint32_t rightChild)
        : bbox(boundingBox), l(leftChild), r(rightChild) {
    }
};
class Mesh : public Model {
 public:
  Mesh() = default;
  Mesh(const Mesh &mesh);
  Mesh(const std::vector<Vertex> &vertices,
       const std::vector<uint32_t> &indices);
  explicit Mesh(const tinyxml2::XMLElement *element);
  ~Mesh() override = default;
  [[nodiscard]] float TraceRay(const glm::vec3 &origin,
                               const glm::vec3 &direction,
                               float t_min,
                               HitRecord *hit_record) const override;
  const char *GetDefaultEntityName() override;
  [[nodiscard]] AxisAlignedBoundingBox GetAABB(
      const glm::mat4 &transform) const override;
  [[nodiscard]] std::vector<Vertex> GetVertices() const override;
  [[nodiscard]] std::vector<uint32_t> GetIndices() const override;
  static Mesh Cube(const glm::vec3 &center, const glm::vec3 &size);
  static Mesh Sphere(const glm::vec3 &center = glm::vec3{0.0f},
                     float radius = 1.0f);
  static bool LoadObjFile(const std::string &obj_file_path, Mesh &mesh);
  void WriteObjFile(const std::string &file_path) const;
  void MergeVertices();
  uint32_t init_kdt(std::vector<uint32_t> &faces);

 protected:
    std::vector<Vertex> vertices_;
    std::vector<uint32_t> indices_;
    std::vector<kdtnode> T;
    std::vector<std::vector<uint32_t>> faces;
    uint32_t nodes, root;
    uint32_t Mesh::leafnode(std::vector<uint32_t> &face);
    void Mesh::searchface(const uint32_t root,
                          const glm::vec3 &origin,
                          const glm::vec3 &direction,
                          float t_min,
                          HitRecord *hit_record,
                          float &result) const;
};
}  // namespace sparks
