#pragma once

#include <cmath>
#include <numbers>
#include <tuple>

#define SCREENWIDTH         256
#define SCREENHEIGHT        176

// Controls depth buffering and backface culling.
bool depthBufferingEnabled = true;
bool backfaceCullingEnabled = true;
bool drawOutlines = false;

// ======================================================================
//  Low-level canvas access.
// ======================================================================

//void canvas = document.getElementById("canvas");
//void canvas_context = canvas.getContext("2d");
//void canvas_buffer = canvas_context.getImageData(0, 0, SCREENWIDTH, SCREENHEIGHT);
//void canvas_pitch = canvas_buffer.width * 4;


// The PutPixel() function.
void PutPixel(uint16_t x, uint16_t y, Color color) {
  //x = SCREENWIDTH / 2 + (x | 0);
  //y = SCREENHEIGHT / 2 - (y | 0) - 1;

  //if (x < 0 || x >= SCREENWIDTH || y < 0 || y >= SCREENHEIGHT) {
  //  return;
  //}

  //uint16_t offset = 4 * x + canvas_pitch * y;
  //canvas_buffer.data[offset++] = color[0];
  //canvas_buffer.data[offset++] = color[1];
  //canvas_buffer.data[offset++] = color[2];
  //canvas_buffer.data[offset++] = 255; // Alpha = 255 (full opacity)
}


// Displays the contents of the offscreen buffer into the canvas.
void UpdateCanvas() {
  //canvas_context.putImageData(canvas_buffer, 0, 0);
}


// ======================================================================
//  Depth buffer.
// ======================================================================
int8_t* depth_buffer = new int8_t[SCREENWIDTH * SCREENHEIGHT];
const int8_t UNDEFINED = -1;

bool UpdateDepthBufferIfCloser(uint16_t x, uint16_t y, int8_t inv_z) {
  x = SCREENWIDTH / 2 + (x | 0);
  y = SCREENHEIGHT / 2 - (y | 0) - 1;

  if (x < 0 || x >= SCREENWIDTH || y < 0 || y >= SCREENHEIGHT) {
    return false;
  }

  uint16_t offset = x + SCREENWIDTH * y;
  if (depth_buffer[offset] == UNDEFINED || depth_buffer[offset] < inv_z) {
    depth_buffer[offset] = inv_z;
    return true;
  }
  return false;
}

void ClearAll() {
  for (int8_t offset = 0; offset < sizeof(depth_buffer); offset++) {
    depth_buffer[offset] = UNDEFINED;
  }
}


// ======================================================================
//  Data model.
// ======================================================================

// A Point.
struct Pt {
  uint16_t x;
  uint16_t y;
  uint16_t h;

  Pt() {}
  Pt(uint16_t x, uint16_t y, uint16_t h) {
    this->x = x;
    this->y = y;
    this->h = h;
  }
};


// A 3D vertex.
struct Vertex {
  uint16_t x;
  uint16_t y;
  uint16_t z;

  Vertex() {}
  Vertex(uint16_t x, uint16_t y, uint16_t z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
};


// A 4D vertex (a 3D vertex in homogeneous coordinates).
struct Vertex4 {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t w;

  Vertex4() {}
  Vertex4(Vertex arg1, uint16_t y, uint16_t z, uint16_t w) {
    this->x = arg1.x;
    this->y = arg1.y;
    this->z = arg1.z;
    this->w = 1;
  }
  Vertex4(Vertex4 arg1, uint16_t y, uint16_t z, uint16_t w) {
    this->x = arg1.x;
    this->y = arg1.y;
    this->z = arg1.z;
    this->w = arg1.w;
  }

  Vertex4(uint16_t x, uint16_t y, uint16_t z, uint16_t w) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
  }
};


// A 4x4 matrix.
struct Mat4x4 {
  Vertex4 data[4];

  Mat4x4() {}
  Mat4x4(Vertex4 one, Vertex4 two, Vertex4 three, Vertex4 four) {
    this->data[0] = one;
    this->data[1] = two;
    this->data[2] = three;
    this->data[3] = four;
  };
};

Mat4x4 Identity4x4( {1, 0, 0, 0}, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } );

// A Triangle.
struct Triangle {
  Vertex* indexes;
  Color color;

  Triangle() {}
  Triangle(Vertex indexes[3], Color color) {
    this->indexes = indexes;
    this->color = color;
  }
};

const uint8_t MAX_TRIANGLES = 100;
const uint8_t MAX_VERTICES = 100;

// A Model.
struct Model {
  Vertex4* vertices;
  Triangle* triangles;
  Pt bounds_center;
  uint16_t bounds_radius;
#
  Model() {}
  Model(Vertex4* vertices, Triangle* triangles, Pt bounds_center, uint16_t bounds_radius) {
    this->vertices = vertices;
    this->triangles = triangles;
    this->bounds_center = bounds_center;
    this->bounds_radius = bounds_radius;
  }
};


// An Instance.
struct Instance {
  Model model;
  Pt position;
  Mat4x4 orientation;
  float scale;
  Mat4x4 transform;

  Instance() {}
  Instance(Model model, Pt position, Mat4x4 orientation, float scale) {
    this->model = model;
    this->position = position;
    this->orientation = orientation || Identity4x4;
    this->scale = scale || 1.0;

    this->transform = MultiplyMM4(
      MakeTranslationMatrix(this->position),
      MultiplyMM4(this->orientation, MakeScalingMatrix(this->scale)));
  }
};

// The Camera.
struct Camera {
  Pt position;
  Mat4x4 orientation;
  Plane clipping_planes[5];

  Camera() {}
  Camera(Pt position, Mat4x4 orientation) {
    this->position = position;
    this->orientation = orientation;
    //this->clipping_planes = [];
  }
};


// A Clipping Plane.
struct Plane {
  Mat4x4 normal;
  float distance;

  Plane() {}
  Plane(Mat4x4 normal, float distance) {
    this->normal = normal;
    this->distance = distance;
  }
};


// ======================================================================
//  Linear algebra and helpers.
// ======================================================================

// Computes k * vec.
Vertex Multiply(uint16_t k, Vertex vec) {
  return Vertex(k * vec.x, k * vec.y, k * vec.z);
}


// Computes dot product.
uint16_t Dot(Vertex v1, Vertex v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}


// Computes cross product.
Vertex Cross(Vertex v1, Vertex v2) {
  return Vertex(
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.x);
}


// Computes v1 + v2.
Vertex Add(Vertex v1, Vertex v2) {
  return Vertex(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}


// Makes a transform matrix for a rotation around the OY axis.
Mat4x4 MakeOYRotationMatrix(uint16_t degrees) {
  float cos = std::cos(degrees * std::numbers::pi_v<float> / 180.0);
  float sin = std::sin(degrees * std::numbers::pi_v<float> / 180.0);

  return Mat4x4([[cos, 0, -sin, 0],
    [0, 1, 0, 0],
    [sin, 0, cos, 0],
    [0, 0, 0, 1]] )
}


// Makes a transform matrix for a translation.
Mat4x4 MakeTranslationMatrix(Pt translation) {
  return Mat4x4([[1, 0, 0, translation.x],
    [0, 1, 0, translation.y],
    [0, 0, 1, translation.z],
    [0, 0, 0, 1]] );
}


// Makes a transform matrix for a scaling.
Mat4x4 MakeScalingMatrix(float scale) {
  return Mat4x4([[scale, 0, 0, 0],
    [0, scale, 0, 0],
    [0, 0, scale, 0],
    [0, 0, 0, 1]] );
}


// Multiplies a 4x4 matrix and a 4D vector.
Vertex4 MultiplyMV(Mat4x4 mat4x4, Vertex4 vec4) {
  Vertex4 result[4] = { {0} , {0} , {0} , {0} };
  Vertex vec[4] = { vec4.x, vec4.y, vec4.z, vec4.w };

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      result[i] += mat4x4.data[i][j] * vec[j];
    }
  }

  return Vertex4(result[0], result[1], result[2], result[3]);
}


// Multiplies two 4x4 matrices.
Mat4x4 MultiplyMM4(Mat4x4 matA, Mat4x4 matB) {
  Mat4x4 result = Mat4x4([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]] );

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      for (uint8_t k = 0; k < 4; k++) {
        result.data[i][j] += matA.data[i][k] * matB.data[k][j];
      }
    }
  }

  return result;
}


// Transposes a 4x4 matrix.
Mat4x4 Transposed(mat) {
  Mat4x4 result = Mat4x4([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]] );
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      result.data[i][j] = mat.data[j][i];
    }
  }
  return result;
}


void Shuffle(vec) {
  for (uint8_t i = vec.length - 1; i > 0; --i) {
    float rand = std::floor(Math.random() * (i + 1));
    [vec[i], vec[rand]] = [vec[rand], vec[i]];
  }
}


void MultiplyColor(uint8_t k, Color color) {
  return[k * color[0], k * color[1], k * color[2]];
}

// ======================================================================
//  Rasterization code.
// ======================================================================

// Scene setup.
uint8_t viewport_size = 1;
uint8_t projection_plane_z = 1;


uint16_t[] Interpolate(uint16_t i0, uint16_t d0, uint16_t i1, uint16_t d1) {
  if (i0 == i1) {
    return[d0];
  }

  uint16_t values = [];
  uint16_t a = (d1 - d0) / (i1 - i0);
  uint16_t d = d0;
  for (uint8_t i = i0; i <= i1; i++) {
    values.push(d);
    d += a;
  }

  return values;
}


void DrawLine(Pt p0, Pt p1, Color color) {
  uint16_t dx = p1.x - p0.x, dy = p1.y - p0.y;

  if (std::abs(dx) > std::abs(dy)) {
    // The line is horizontal-ish. Make sure it's left to right.
    if (dx < 0) { uint16_t swap = p0; p0 = p1; p1 = swap; }

    // Compute the Y values and draw.
    uint16_t[] ys = Interpolate(p0.x, p0.y, p1.x, p1.y);
    for (uint8_t x = p0.x; x <= p1.x; x++) {
      PutPixel(x, ys[(x - p0.x) | 0], color);
    }
  }
  else {
    // The line is verical-ish. Make sure it's bottom to top.
    if (dy < 0) { uint16_t swap = p0; p0 = p1; p1 = swap; }

    // Compute the X values and draw.
    uint16_t[] xs = Interpolate(p0.y, p0.x, p1.y, p1.x);
    for (uint8_t y = p0.y; y <= p1.y; y++) {
      PutPixel(xs[(y - p0.y) | 0], y, color);
    }
  }
}


void DrawWireframeTriangle(p0, p1, p2, color) {
  DrawLine(p0, p1, color);
  DrawLine(p1, p2, color);
  DrawLine(p0, p2, color);
}


// Converts 2D viewport coordinates to 2D canvas coordinates.
Pt ViewportToCanvas(Pt p2d) {
  return Pt(
    (p2d.x * SCREENWIDTH / viewport_size) | 0,
    (p2d.y * SCREENHEIGHT / viewport_size) | 0);
}


Pt ProjectVertex(Vertex v) {
  return ViewportToCanvas(Pt(v.x * projection_plane_z / v.z,
    v.y * projection_plane_z / v.z));
}


// Sort the points from bottom to top.
// Technically, sort the indexes to the vertex indexes in the triangle from bottom to top.
uint8_t[3] SortedVertexIndexes(Vertex vertex_indexes[3], Pt projected) {
  uint8_t indexes[3] = { 0, 1, 2 };

  if (projected[vertex_indexes[indexes[1]]].y < projected[vertex_indexes[indexes[0]]].y) { void swap = indexes[0]; indexes[0] = indexes[1]; indexes[1] = swap; }
  if (projected[vertex_indexes[indexes[2]]].y < projected[vertex_indexes[indexes[0]]].y) { void swap = indexes[0]; indexes[0] = indexes[2]; indexes[2] = swap; }
  if (projected[vertex_indexes[indexes[2]]].y < projected[vertex_indexes[indexes[1]]].y) { void swap = indexes[1]; indexes[1] = indexes[2]; indexes[2] = swap; }

  return indexes;
}


Vertex ComputeTriangleNormal(Vertex v0, Vertex v1, Vertex) {
  Vertex v0v1 = Add(v1, Multiply(-1, v0));
  Vertex v0v2 = Add(v2, Multiply(-1, v0));
  return Cross(v0v1, v0v2);
}


std::tuple<uint16_t, uint16_t> EdgeInterpolate(uint16_t y0, uint16_t v0, uint16_t y1, uint16_t v1, uint16_t y2, uint16_t v2) {
  uint16_t[] v01 = Interpolate(y0, v0, y1, v1);
  uint16_t[] v12 = Interpolate(y1, v1, y2, v2);
  uint16_t[] v02 = Interpolate(y0, v0, y2, v2);
  v01.pop();
  uint16_t v012 = v01.concat(v12);
  return{ v02, v012 };
}


void RenderTriangle(Triangle triangle, vertices, Pt projected) {
  // Sort by projected point Y.
  uint8_t indexes[3] = SortedVertexIndexes(triangle.indexes, projected);

  Vertex v0 = vertices[triangle.indexes[indexes[0]]];
  Vertex v1 = vertices[triangle.indexes[indexes[1]]];
  Vertex v2 = vertices[triangle.indexes[indexes[2]]];

  // Compute triangle normal. Use the unsorted vertices, otherwise the winding of the points may change.
  Vertex normal = ComputeTriangleNormal(vertices[triangle.indexes[0]], vertices[triangle.indexes[1]], vertices[triangle.indexes[2]]);

  // Backface culling.
  if (backfaceCullingEnabled) {
    Vertex4 centre = Multiply(-1.0 / 3.0, Add(Add(vertices[triangle.indexes[0]], vertices[triangle.indexes[1]]), vertices[triangle.indexes[2]]));
    if (Dot(centre, normal) < 0) {
      return;
    }
  }

  // Get attribute values (X, 1/Z) at the vertices.
  Pt p0 = projected[triangle.indexes[i0]];
  Pt p1 = projected[triangle.indexes[i1]];
  Pt p2 = projected[triangle.indexes[i2]];

  // Compute attribute values at the edges.
  uint16_t x02;
  uint16_t x012;
  std::tie(x02, x012) = EdgeInterpolate(p0.y, p0.x, p1.y, p1.x, p2.y, p2.x);

  uint16_t iz02;
  uint16_t iz012;
  std::tie(iz02, iz012) = EdgeInterpolate(p0.y, 1.0 / v0.z, p1.y, 1.0 / v1.z, p2.y, 1.0 / v2.z);


  // Determine which is left and which is right.
  uint16_t m = (x02.length / 2) | 0;
  if (x02[m] < x012[m]) {
    uint16_t[x_left, x_right] = [x02, x012];
    uint16_t[iz_left, iz_right] = [iz02, iz012];
  }
  else {
    uint16_t[x_left, x_right] = [x012, x02];
    uint16_t[iz_left, iz_right] = [iz012, iz02];
  }

  // Draw horizontal segments.
  for (uint8_t y = p0.y; y <= p2.y; y++) {
    uint16_t[xl, xr] = [x_left[y - p0.y] | 0, x_right[y - p0.y] | 0];

    // Interpolate attributes for this scanline.
    uint16_t[zl, zr] = [iz_left[y - p0.y], iz_right[y - p0.y]];
    uint16_t[] zscan = Interpolate(xl, zl, xr, zr);

    for (uint8_t x = xl; x <= xr; x++) {
      if (!depthBufferingEnabled || UpdateDepthBufferIfCloser(x, y, zscan[x - xl])) {
        PutPixel(x, y, triangle.color);
      }
    }
  }

  if (drawOutlines) {
    Color outline_color = MultiplyColor(0.75, triangle.color);
    DrawLine(p0, p1, outline_color);
    DrawLine(p0, p2, outline_color);
    DrawLine(p2, p1, outline_color);
  }
}


// Clips a triangle against a plane. Adds output to triangles and vertices.
void ClipTriangle(triangle, plane, triangles, vertices) {
  Vertex v0 = vertices[triangle.indexes[0]];
  Vertex v1 = vertices[triangle.indexes[1]];
  Vertex v2 = vertices[triangle.indexes[2]];

  uint16_t in0 = Dot(plane.normal, v0) + plane.distance > 0;
  uint16_t in1 = Dot(plane.normal, v1) + plane.distance > 0;
  uint16_t in2 = Dot(plane.normal, v2) + plane.distance > 0;

  uint16_t in_count = in0 + in1 + in2;
  if (in_count == 0) {
    // Nothing to do - the triangle is fully clipped out.
  }
  else if (in_count == 3) {
    // The triangle is fully in front of the plane.
    triangles.push(triangle);
  }
  else if (in_count == 1) {
    // The triangle has one vertex in. Output is one clipped triangle.
  }
  else if (in_count == 2) {
    // The triangle has two vertices in. Output is two clipped triangles.
  }
}


Model TransformAndClip(Plane clipping_planes[5], model, scale, transform) {
  // Transform the bounding sphere, and attempt early discard.
  Vertex4 center = MultiplyMV(transform, Vertex4(model.bounds_center));
  float radius = model.bounds_radius * scale;
  for (uint8_tp = 0; p < clipping_planes.length; p++) {
    void distance = Dot(clipping_planes[p].normal, center) + clipping_planes[p].distance;
    if (distance < -radius) {
      return null;
    }
  }

  // Apply modelview transform.
  Vertex4 vertices = [];
  for (uint8_t i = 0; i < sizeof(model.vertices); i++) {
    vertices.push(MultiplyMV(transform, Vertex4(model.vertices[i])));
  }

  // Clip the entire model against each successive plane.
  Triangle triangles = model.triangles.slice();
  for (uint8_tp = 0; p < clipping_planes.length; p++) {
    new_triangles = []
      for (uint8_t i = 0; i < triangles.length; i++) {
        ClipTriangle(triangles[i], clipping_planes[p], new_triangles, vertices);
      }
    triangles = new_triangles;
  }

  return Model(vertices, triangles, center, model.bounds_radius);
}


void RenderModel(Model model) {
  Pt projected = [];
  for (uint8_t i = 0; i < sizeof(model.vertices); i++) {
    projected.push(ProjectVertex(Vertex4(model.vertices[i])));
  }
  for (uint8_t i = 0; i < model.triangles.length; i++) {
    RenderTriangle(model.triangles[i], model.vertices, projected);
  }
}


void RenderScene(Camera camera, Instance instances) {
  Mat4x4 cameraMatrix = MultiplyMM4(Transposed(camera.orientation), MakeTranslationMatrix(Multiply(-1, camera.position)));

  for (uint8_t i = 0; i < instances.length; i++) {
    Mat4x4 transform = MultiplyMM4(cameraMatrix, instances[i].transform);
    void clipped = TransformAndClip(camera.clipping_planes, instances[i].model, instances[i].scale, transform);
    if (clipped != null) {
      RenderModel(clipped);
    }
  }
}


void ShuffleCubeTriangles() {
  Shuffle(cube.triangles);
  Render();
}

void SetDepthEnabled(bool enabled) {
  depthBufferingEnabled = enabled;
  backfaceCullingEnabled = enabled;
  Render();
}

void SetOutlinesEnabled(bool enabled) {
  drawOutlines = enabled;
  Render();
}

void Render() {
  ClearAll();
  // This lets the browser clear the canvas before blocking to render the scene.
  setTimeout(function() {
    RenderScene(camera, instances);
    UpdateCanvas();
  }, 0);
}

void fred() {
  Vertex vertices[] = [
    Vertex(1, 1, 1),
      Vertex(-1, 1, 1),
      Vertex(-1, -1, 1),
      Vertex(1, -1, 1),
      Vertex(1, 1, -1),
      Vertex(-1, 1, -1),
      Vertex(-1, -1, -1),
      Vertex(1, -1, -1)
  ];

  Color RED = Color(255, 0, 0);
  Color GREEN = Color(0, 255, 0);
  Color BLUE = Color(0, 0, 255);
  Color YELLOW = Color(255, 255, 0);
  Color PURPLE = Color(255, 0, 255);
  Color CYAN = Color(0, 255, 255);

  void triangles = [
    Triangle([0, 1, 2], RED),
      Triangle([0, 2, 3], RED),
      Triangle([1, 5, 6], YELLOW),
      Triangle([1, 6, 2], YELLOW),
      Triangle([2, 6, 7], CYAN),
      Triangle([2, 7, 3], CYAN),
      Triangle([4, 0, 3], GREEN),
      Triangle([4, 1, 0], PURPLE),
      Triangle([4, 3, 7], GREEN),
      Triangle([4, 5, 1], PURPLE),
      Triangle([5, 4, 7], BLUE),
      Triangle([5, 7, 6], BLUE),
  ];

  Shuffle(triangles);

  Model cube(vertices, triangles, Vertex(0, 0, 0), std::sqrt(3));
  Instance instances[2] = {
    Instance(cube, Vertex(-1.5, 0, 7), Identity4x4, 0.75),
    Instance(cube, Vertex(1.25, 2.5, 7.5), MakeOYRotationMatrix(195))
  };

  Camera camera(Vertex(-3, 1, 2), MakeOYRotationMatrix(-30));

  float s2 = std::sqrt(2);
  camera.clipping_planes = [
    Plane(Vertex(0, 0, 1), -1), // Near
      Plane(Vertex(s2, 0, s2), 0), // Left
      Plane(Vertex(-s2, 0, s2), 0), // Right
      Plane(Vertex(0, -s2, s2), 0), // Top
      Plane(Vertex(0, s2, s2), 0), // Bottom
  ];

  Render();
}