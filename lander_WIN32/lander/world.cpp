#include "stdafx.h"
#include "world.h"

#include <assert.h>

// Controls depth buffering and backface culling.
bool depthBufferingEnabled = true;
bool backfaceCullingEnabled = true;
bool drawOutlines = false;

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
constexpr uint16_t depth_buffer_length = SCREENWIDTH * SCREENHEIGHT;
float_t* depth_buffer = new float_t[depth_buffer_length];
const float_t UNDEFINED = -1.0f;

bool UpdateDepthBufferIfCloser(uint16_t x, uint16_t y, float_t inv_z) {
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
  for (uint16_t offset = 0; offset < depth_buffer_length; offset++) {
    depth_buffer[offset] = UNDEFINED;
  }
}

// ======================================================================
//  Data model.
// ======================================================================

// ======================================================================
//  Linear algebra and helpers.
// ======================================================================

// Computes k * vec.
Vertex Multiply(float_t k, Vertex vec) {
  return Vertex(k * vec.x, k * vec.y, k * vec.z);
}


// Computes dot product.
float_t Dot(Vertex v1, Vertex v2) {
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
  float_t cos = std::cosf((float_t)degrees * std::numbers::pi_v<float_t> / 180.0f);
  float_t sin = std::sinf((float_t)degrees * std::numbers::pi_v<float_t> / 180.0f);

  return Mat4x4(
    { cos, 0, -sin, 0 },
    { 0, 1, 0, 0 },
    { sin, 0, cos, 0 },
    { 0, 0, 0, 1 });
}


// Makes a transform matrix for a translation.
Mat4x4 MakeTranslationMatrix(Vertex translation) {
  return Mat4x4(
    { 1, 0, 0, translation.x },
    { 0, 1, 0, translation.y },
    { 0, 0, 1, translation.z },
    { 0, 0, 0, 1 });
}


// Makes a transform matrix for a scaling.
Mat4x4 MakeScalingMatrix(float_t scale) {
  return Mat4x4(
    { scale, 0, 0, 0 },
    { 0, scale, 0, 0 },
    { 0, 0, scale, 0 },
    { 0, 0, 0, 1 });
}


// Multiplies a 4x4 matrix and a 4D vector.
Vertex4 MultiplyMV(Mat4x4 mat4x4, Vertex4 vec4) {
  float_t result[4] = { 0 , 0 , 0 , 0 };
  float_t vec[4] = { vec4.x1, vec4.y1, vec4.z1, vec4.w1 };

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      result[i] += mat4x4.data[i].v[j] * vec[j];
    }
  }

  return Vertex4(result[0], result[1], result[2], result[3]);
}


// Multiplies two 4x4 matrices.
Mat4x4 MultiplyMM4(Mat4x4 matA, Mat4x4 matB) {
  Mat4x4 result = Mat4x4(
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 });

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      for (uint8_t k = 0; k < 4; k++) {
        result.data[i].v[j] += matA.data[i].v[k] * matB.data[k].v[j];
      }
    }
  }

  return result;
}


// Transposes a 4x4 matrix.
Mat4x4 Transposed(Mat4x4 mat) {
  Mat4x4 result = Mat4x4(
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 },
    { 0, 0, 0, 0 });

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      result.data[i].v[j] = mat.data[j].v[i];
    }
  }
  return result;
}

Color MultiplyColor(float_t k, Color color) {
  return Color((byte)(k * (float_t)color.Red), (byte)(k * (float_t)color.Green), (byte)(k * (float_t)color.Blue));
}


// ======================================================================
//  Rasterization code.
// ======================================================================

// Scene setup.
uint8_t viewport_size = 1;
float_t projection_plane_z = 1.0f;

std::vector<float_t> Interpolate(float_t i0, float_t d0, float_t i1, float_t d1) {
  std::vector<float_t> values;

  if (i0 == i1) {
    values.push_back(d0);
  } else {
    float_t a = (d1 - d0) / (i1 - i0);
    float_t d = d0;

    for (std::size_t i = (std::size_t)i0; i <= i1; i++) {
      values.push_back(d);
      d += a;
    }
  }

  return values;
}


void DrawLine(Pt p0, Pt p1, Color color) {
  uint16_t dx = p1.x - p0.x, dy = p1.y - p0.y;

  if (std::abs(dx) > std::abs(dy)) {
    // The line is horizontal-ish. Make sure it's left to right.
    if (dx < 0) { Pt swap = p0; p0 = p1; p1 = swap; }

    // Compute the Y values and draw.
    std::vector<float_t> ys = Interpolate(p0.x, p0.y, p1.x, p1.y);
    for (uint16_t x = p0.x; x <= p1.x; x++) {
      PutPixel(x, (uint16_t)ys[(uint16_t)(x - p0.x)], color);
    }
  } else {
    // The line is verical-ish. Make sure it's bottom to top.
    if (dy < 0) { Pt swap = p0; p0 = p1; p1 = swap; }

    // Compute the X values and draw.
    std::vector<float_t> xs = Interpolate(p0.y, p0.x, p1.y, p1.x);
    for (uint16_t y = p0.y; y <= p1.y; y++) {
      PutPixel((uint16_t)xs[(uint16_t)(y - p0.y)], y, color);
    }
  }
}


void DrawWireframeTriangle(Pt p0, Pt p1, Pt p2, Color color) {
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
  return ViewportToCanvas(Pt(
    (uint16_t)(v.x * projection_plane_z / v.z),
    (uint16_t)(v.y * projection_plane_z / v.z)));
}


// Sort the points from bottom to top.
// Technically, sort the indexes to the vertex indexes in the triangle from bottom to top.
void SortedVertexIndexes(uint16_t* triangle_indexes, Pt* projected, uint8_t* indexes) {
  indexes[0] = 0;
  indexes[1] = 1;
  indexes[2] = 2;

  if (projected[triangle_indexes[indexes[1]]].y < projected[triangle_indexes[indexes[0]]].y) { uint8_t swap = indexes[0]; indexes[0] = indexes[1]; indexes[1] = swap; }
  if (projected[triangle_indexes[indexes[2]]].y < projected[triangle_indexes[indexes[0]]].y) { uint8_t swap = indexes[0]; indexes[0] = indexes[2]; indexes[2] = swap; }
  if (projected[triangle_indexes[indexes[2]]].y < projected[triangle_indexes[indexes[1]]].y) { uint8_t swap = indexes[1]; indexes[1] = indexes[2]; indexes[2] = swap; }
}


Vertex ComputeTriangleNormal(Vertex v0, Vertex v1, Vertex v2) {
  Vertex v0v1 = Add(v1, Multiply(-1, v0));
  Vertex v0v2 = Add(v2, Multiply(-1, v0));
  return Cross(v0v1, v0v2);
}


std::tuple<std::vector<float_t>, std::vector<float_t>> EdgeInterpolate(float_t y0, float_t v0, float_t y1, float_t v1, float_t y2, float_t v2) {
  std::vector<float_t> v01 = Interpolate(y0, v0, y1, v1);
  std::vector<float_t> v12 = Interpolate(y1, v1, y2, v2);
  std::vector<float_t> v02 = Interpolate(y0, v0, y2, v2);
  v01.pop_back();

  // Concatenate.
  std::vector<float_t> v012;
  v012.insert(v012.end(), v01.begin(), v01.end());
  v012.insert(v012.end(), v12.begin(), v12.end());

  return{ v02, v012 };
}


void RenderTriangle(Triangle* triangle, Vertex* vertices, Pt* projected) {
  // Sort by projected point Y.
  uint8_t indexes[3];
  SortedVertexIndexes(&triangle->indexes[0], projected, &indexes[0]);
  uint8_t i0 = indexes[0];
  uint8_t i1 = indexes[1];
  uint8_t i2 = indexes[2];

  Vertex v0 = vertices[triangle->indexes[indexes[0]]];
  Vertex v1 = vertices[triangle->indexes[indexes[1]]];
  Vertex v2 = vertices[triangle->indexes[indexes[2]]];

  // Compute triangle normal. Use the unsorted vertices, otherwise the winding of the points may change.
  Vertex normal = ComputeTriangleNormal(vertices[triangle->indexes[0]], vertices[triangle->indexes[1]], vertices[triangle->indexes[2]]);

  // Backface culling.
  if (backfaceCullingEnabled) {
    Vertex centre = Multiply(
      -1.0f / 3.0f,
      Add(
        Add(vertices[triangle->indexes[0]], vertices[triangle->indexes[1]]),
        vertices[triangle->indexes[2]]));

    if (Dot(centre, normal) < 0) {
      return;
    }
  }

  // Get attribute values (X, 1/Z) at the vertices.
  Pt p0 = projected[triangle->indexes[i0]];
  Pt p1 = projected[triangle->indexes[i1]];
  Pt p2 = projected[triangle->indexes[i2]];

  // Compute attribute values at the edges.
  std::vector<float_t> x02;
  std::vector<float_t> x012;
  std::tie(x02, x012) = EdgeInterpolate(p0.y, p0.x, p1.y, p1.x, p2.y, p2.x);

  std::vector<float_t> iz02;
  std::vector<float_t> iz012;
  std::tie(iz02, iz012) = EdgeInterpolate(p0.y, 1.0f / v0.z, p1.y, 1.0f / v1.z, p2.y, 1.0f / v2.z);


  // Determine which is left and which is right.
  std::size_t m = (x02.size() / 2);  // 0

  std::vector<float_t> x_left, x_right;
  std::vector<float_t> iz_left, iz_right;

  if (x02[m] < x012[m]) {
    x_left = x02;
    x_right = x012;
    iz_left = iz02;
    iz_right = iz012;
  } else {
    x_left = x012;
    x_right = x02;
    iz_left = iz012;
    iz_right = iz02;
  }

  // Draw horizontal segments.
  for (uint16_t y = p0.y; y <= p2.y; y++) {
    float_t xl = x_left[y - p0.y];  // 0;
    float_t xr = x_right[y - p0.y]; // 0;

    // Interpolate attributes for this scanline.
    float_t zl = iz_left[y - p0.y];
    float_t zr = iz_right[y - p0.y];
    std::vector<float_t> zscan = Interpolate(xl, zl, xr, zr);

    for (uint16_t x = (uint16_t)xl; x <= xr; x++) {
      uint16_t i = x - (uint16_t)xl;
      if (!depthBufferingEnabled || UpdateDepthBufferIfCloser(x, y, zscan[i])) {
        PutPixel(x, y, triangle->color);
      }
    }
  }

  if (drawOutlines) {
    Color outline_color = MultiplyColor(0.75, triangle->color);
    DrawLine(p0, p1, outline_color);
    DrawLine(p0, p2, outline_color);
    DrawLine(p2, p1, outline_color);
  }
}


// Clips a triangle against a plane. Adds output to triangles and vertices.
void ClipTriangle(Triangle* triangle, Plane* plane, std::vector<Triangle*>& triangles, std::vector<Vertex>& vertices) {
  Vertex v0 = vertices[triangle->indexes[0]];
  Vertex v1 = vertices[triangle->indexes[1]];
  Vertex v2 = vertices[triangle->indexes[2]];

  uint16_t in0 = Dot(plane->normal, v0) + plane->distance > 0;
  uint16_t in1 = Dot(plane->normal, v1) + plane->distance > 0;
  uint16_t in2 = Dot(plane->normal, v2) + plane->distance > 0;

  uint16_t in_count = in0 + in1 + in2;
  if (in_count == 0) {
    // Nothing to do - the triangle is fully clipped out.
  }
  else if (in_count == 3) {
    // The triangle is fully in front of the plane.
    triangles.push_back(triangle);
  }
  else if (in_count == 1) {
    // The triangle has one vertex in. Output is one clipped triangle.
  }
  else if (in_count == 2) {
    // The triangle has two vertices in. Output is two clipped triangles.
  }
}


bool TransformAndClip(Camera* camera, Model* model, float_t scale, Mat4x4 transform, Model* clipped_model) {
  assert(clipped_model != nullptr);

  // Check if there is anything to clip.
  if ((model->vertices_length == 0) ||
    (model->triangles_length == 0)) {
    return false;
  }

  // Transform the bounding sphere, and attempt early discard.
  Vertex center = MultiplyMV(transform, Vertex4(&model->bounds_center));
  float_t radius = model->bounds_radius * scale;
  for (uint8_t p = 0; p < camera->clipping_planes_length; p++) {
    float_t distance = Dot(camera->clipping_planes[p].normal, center) + camera->clipping_planes[p].distance;
    if (distance < -radius) {
      return false; // Not clipped.
    }
  }

  // Apply modelview transform.
  std::vector<Vertex> new_vertices;
  for (uint8_t i = 0; i < model->vertices_length; i++) {
    new_vertices.push_back(Vertex(MultiplyMV(transform, Vertex4(&model->vertices[i]))));
  }

  // Clip the entire model against each successive plane.
  Triangle* original_triangles = model->triangles; // 1..*

  Triangle* triangles = new Triangle[model->triangles_length];
  std::copy(model->triangles, model->triangles + model->triangles_length, triangles);
  uint16_t triangles_length = model->triangles_length;

  for (uint8_t p = 0; p < camera->clipping_planes_length; p++) {
    std::vector<Triangle*> new_triangles;

    for (uint8_t i = 0; i < model->triangles_length; i++) {
      ClipTriangle(&model->triangles[i], &camera->clipping_planes[p], new_triangles, new_vertices);
    }

    // Update the triangles for the next iteration.
    Triangle** triangle = new_triangles.data(); // 0..*

    delete[] triangles;

    if (new_triangles.size() == 0)
    {
      return 0;
    }

    triangles = new Triangle[new_triangles.size()];
    std::copy(*new_triangles.data(), *new_triangles.data() + new_triangles.size(), triangles);
    triangles_length = static_cast<uint16_t>(new_triangles.size());
  }

  Vertex* vertices = new Vertex[new_vertices.size()];
  std::copy(new_vertices.data(), new_vertices.data() + new_vertices.size(), vertices);

  // Update the result.
  clipped_model->vertices = vertices;
  clipped_model->vertices_length = static_cast<uint16_t>(new_vertices.size());
  clipped_model->triangles = triangles;
  clipped_model->triangles_length = static_cast<uint16_t>(triangles_length);
  clipped_model->bounds_center = center;
  clipped_model->bounds_radius = model->bounds_radius;

  return true;
}


void RenderModel(Model* model) {
  std::vector<Pt> projected;

  for (uint8_t i = 0; i < model->vertices_length; i++) {
    projected.push_back(ProjectVertex(model->vertices[i]));
  }
  for (uint8_t i = 0; i < model->triangles_length; i++) {
    RenderTriangle(&model->triangles[i], model->vertices, projected.data());
  }
}


void RenderScene(Camera* camera, Instance* instances, int16_t instances_count) {
  Mat4x4 cameraMatrix = MultiplyMM4(Transposed(camera->orientation), MakeTranslationMatrix(Multiply(-1, camera->position)));

  for (uint8_t i = 0; i < instances_count; i++) {
    Mat4x4 transform = MultiplyMM4(cameraMatrix, instances[i].transform);

    Model clipped_model;
    ;
    if (TransformAndClip(camera, instances[i].model, instances[i].scale, transform, &clipped_model)) {
      RenderModel(&clipped_model);

      delete clipped_model.vertices;
      delete clipped_model.triangles;
    }
  }
}


void Render() {
  ClearAll();
  // This lets the browser clear the canvas before blocking to render the scene.
  //setTimeout(function() {
  //  RenderScene(camera, instances);
  //  UpdateCanvas();
  //}, 0);
}

void Shuffle(Triangle* vec, uint16_t length) {
  for (uint8_t i = length - 1; i > 0; --i) {
    uint8_t rand = (uint8_t)std::floor(std::rand() * (i + 1));
    vec[i] = vec[rand];
    vec[rand] = vec[i];
  }
}

void ShuffleCubeTriangles(Triangle* triangles, uint16_t length) {
  Shuffle(triangles, length);
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

// ======================================================================
//  Data model.
// ======================================================================

Mat4x4 Identity4x4({ 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 });


// ======================================================================
//  Miscellaneous.
// ======================================================================

void fred() {
  Vertex vertices[] = {
    { 1, 1, 1 },
    { -1, 1, 1 },
    { -1, -1, 1 },
    { 1, -1, 1 },
    { 1, 1, -1 },
    { -1, 1, -1 },
    { -1, -1, -1 },
    { 1, -1, -1 } };

  Color RED = Color(255, 0, 0);
  Color GREEN = Color(0, 255, 0);
  Color BLUE = Color(0, 0, 255);
  Color YELLOW = Color(255, 255, 0);
  Color PURPLE = Color(255, 0, 255);
  Color CYAN = Color(0, 255, 255);

  Triangle triangles[] = {
    { 0, 1, 2, RED },
    { 0, 2, 3, RED },
    { 1, 5, 6, YELLOW},
    { 1, 6, 2, YELLOW},
    { 2, 6, 7, CYAN},
    { 2, 7, 3, CYAN},
    { 4, 0, 3, GREEN},
    { 4, 1, 0, PURPLE},
    { 4, 3, 7, GREEN},
    { 4, 5, 1, PURPLE},
    { 5, 4, 7, BLUE},
    { 5, 7, 6, BLUE } };

  Shuffle(&triangles[0], sizeof(triangles));

  Model cube(vertices, triangles, Vertex(0, 0, 0), std::sqrtf(3.0f));
  Instance instances[] = {
    { &cube, Vertex(-1.5, 0, 7), Identity4x4, 0.75},
    { &cube, Vertex(1.25, 2.5, 7.5), MakeOYRotationMatrix(195), 1.0 }
  };

  float_t s2 = std::sqrtf(2);
  Plane clipping_planes[] = {
    Plane(Vertex(0, 0, 1), -1),   // Near
    Plane(Vertex(s2, 0, s2), 0),  // Left
    Plane(Vertex(-s2, 0, s2), 0), // Right
    Plane(Vertex(0, -s2, s2), 0), // Top
    Plane(Vertex(0, s2, s2), 0)   // Bottom
  };

  Camera camera(Vertex(-3, 1, 2), MakeOYRotationMatrix(-30), clipping_planes);

  Render();
}
