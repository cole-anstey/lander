#pragma once

#include <cmath>
#include <numbers>
#include <tuple>
#include <vector>

#define SCREENWIDTH         256
#define SCREENHEIGHT        176

// ======================================================================
//  Low-level canvas access.
// ======================================================================

//void canvas = document.getElementById("canvas");
//void canvas_context = canvas.getContext("2d");
//void canvas_buffer = canvas_context.getImageData(0, 0, SCREENWIDTH, SCREENHEIGHT);
//void canvas_pitch = canvas_buffer.width * 4;

struct Vertex;
struct Vertex4;
struct Mat4x4;
struct Pt;
struct Triangle;
struct Camera;
struct Model;
struct Plane;
struct Instance;


// ======================================================================
//  Depth buffer.
// ======================================================================

bool UpdateDepthBufferIfCloser(uint16_t x, uint16_t y, float_t inv_z);
void ClearAll();


// ======================================================================
//  Rasterization code.
// ======================================================================

// Computes k * vec.
Vertex Multiply(float_t k, Vertex vec);

// Computes dot product.
float_t Dot(Vertex v1, Vertex v2);

// Computes cross product.
Vertex Cross(Vertex v1, Vertex v2);

// Computes v1 + v2.
Vertex Add(Vertex v1, Vertex v2);

// Makes a transform matrix for a rotation around the OY axis.
Mat4x4 MakeOYRotationMatrix(uint16_t degrees);

// Makes a transform matrix for a translation.
Mat4x4 MakeTranslationMatrix(Vertex translation);

// Makes a transform matrix for a scaling.
Mat4x4 MakeScalingMatrix(float_t scale);

// Multiplies a 4x4 matrix and a 4D vector.
Vertex4 MultiplyMV(Mat4x4 mat4x4, Vertex4 vec4);
// Multiplies two 4x4 matrices.
Mat4x4 MultiplyMM4(Mat4x4 matA, Mat4x4 matB);
// Transposes a 4x4 matrix.
Mat4x4 Transposed(Mat4x4 mat);


// ======================================================================
//  Data model.
// ======================================================================

// A Point.
struct Pt {
  //uint16_t x;
  //uint16_t y;
  //uint16_t h;
  union {
    struct {
      uint16_t x;
      uint16_t y;
      uint16_t h;
    };
    uint16_t v[3];
  };

  Pt() {}
  Pt(uint16_t x, uint16_t y, uint16_t h = -1) {
    this->x = x;
    this->y = y;
    this->h = h;
  }
};

// A 3D vertex.
struct Vertex {
  float_t x;
  float_t y;
  float_t z;

  Vertex() {}
  Vertex(float_t x, float_t y, float_t z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  Vertex(const struct Vertex4* ver) {
    //this->x = ver.x;
    //this->y = ver.y;
    //this->z = ver.z;
  }

  //float& operator[](int i) {
  //  assert(i >= 0);
  //  assert(i < 3);
  //  return (i == 0) ? x : y;
  //}
  //const float& operator[](int i) const {
  //  assert(i >= 0);
  //  assert(i < 3);
  //  return (i == 0) ? x : y;
  //}

  //operator struct Vertex4() const { return struct Vertex4(x, y, z, 1); } // conversion function
};

// A 4D vertex (a 3D vertex in homogeneous coordinates).
struct Vertex4 {
  //float_t x;
  //float_t y;
  //float_t z;
  //float_t w;
  union {
    struct {
      float_t x1;
      float_t y1;
      float_t z1;
      float_t w1;
    };
    float_t v[4];
  };

  Vertex4() {}
  Vertex4(const struct Vertex* arg1) {
    //this->x = arg1.x;
    //this->y = arg1.y;
    //this->z = arg1.z;
    this->w1 = 1;
  }
  Vertex4(const Vertex4& arg1, float_t y, float_t z, float_t w) {
    this->x1 = arg1.x1;
    this->y1 = arg1.y1;
    this->z1 = arg1.z1;
    this->w1 = arg1.w1;
  }

  Vertex4(float_t x, float_t y, float_t z, float_t w) {
    this->x1 = x;
    this->y1 = y;
    this->z1 = z;
    this->w1 = w;
  }

  operator struct Vertex() const { return struct Vertex(x1, y1, z1); } // conversion function
};

// A 4x4 matrix.
struct Mat4x4 {
  Vertex4 data[4];

  Mat4x4() {}
  Mat4x4(const Vertex4& one, const Vertex4& two, const Vertex4& three, const Vertex4& four) {
    this->data[0] = one;
    this->data[1] = two;
    this->data[2] = three;
    this->data[3] = four;
  };
};

// A Triangle.
struct Triangle {
  uint16_t indexes[3];
  Color color;

  Triangle() {}
  Triangle(uint16_t index_one, uint16_t index_two, uint16_t index_three, Color color) {
    this->indexes[0] = index_one;
    this->indexes[1] = index_two;
    this->indexes[2] = index_three;
    this->color = color;
  }
};

const uint8_t MAX_TRIANGLES = 100;
const uint8_t MAX_VERTICES = 100;

// A Model.
struct Model {
  Vertex* vertices;
  uint16_t vertices_length;
  Triangle* triangles;
  uint16_t triangles_length;
  Vertex bounds_center;
  float_t bounds_radius;
#
  Model() {}
  Model(Vertex vertices[], Triangle triangles[], Vertex bounds_center, float_t bounds_radius) {
    this->vertices = vertices;
    this->vertices_length = sizeof(vertices);
    this->triangles = triangles;
    this->triangles_length = sizeof(triangles);
    this->bounds_center = bounds_center;
    this->bounds_radius = bounds_radius;
  }
};


// An Instance.
struct Instance {
  Model* model;
  Vertex position;
  Mat4x4 orientation;
  float_t scale;
  Mat4x4 transform;

  Instance() {}
  Instance(Model* model, Vertex position, Mat4x4 orientation, float_t scale = 1.0) {
    this->model = model;
    this->position = position;
    this->orientation = orientation;
    this->scale = scale;

    this->transform = MultiplyMM4(
      MakeTranslationMatrix(this->position),
      MultiplyMM4(this->orientation, MakeScalingMatrix(this->scale)));
  }
};

// A Clipping Plane.
struct Plane {
  Vertex normal;
  float_t distance;

  Plane() {}
  Plane(Vertex normal, float_t distance) {
    this->normal = normal;
    this->distance = distance;
  }
};

// The Camera.
struct Camera {
  Vertex position;
  Mat4x4 orientation;
  Plane* clipping_planes;
  uint16_t clipping_planes_length;

  Camera() {}
  Camera(Vertex position, Mat4x4 orientation, Plane clipping_planes[]) {
    this->position = position;
    this->orientation = orientation;
    this->clipping_planes = clipping_planes;
    this->clipping_planes_length = sizeof(clipping_planes);
  }
};
