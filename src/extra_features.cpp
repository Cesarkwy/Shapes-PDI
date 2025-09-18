#include "surf.h"
#include "curve.h"
#include <GL/glut.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

/* Implementação de funcionalidades adicionais:
 * 1. Materiais e texturas
 * 2. Interface interativa para curvas
 * 3. Splines Catmull-Rom
 */

// Definição de materiais
struct Material {
    float ambient[4];
    float diffuse[4];
    float specular[4];
    float shininess;
};

// Alguns materiais predefinidos
const Material MATERIAL_GOLD = {
    {0.24725f, 0.1995f, 0.0745f, 1.0f},    // ambient
    {0.75164f, 0.60648f, 0.22648f, 1.0f},  // diffuse
    {0.628281f, 0.555802f, 0.366065f, 1.0f}, // specular
    51.2f                                   // shininess
};

const Material MATERIAL_GLASS = {
    {0.1f, 0.1f, 0.1f, 0.5f},  // ambient
    {0.4f, 0.4f, 0.4f, 0.5f},  // diffuse
    {0.9f, 0.9f, 0.9f, 0.5f},  // specular
    96.0f                       // shininess
};

// Aplica um material a um objeto
void applyMaterial(const Material& mat) {
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat.ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat.diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat.specular);
    glMaterialf(GL_FRONT, GL_SHININESS, mat.shininess);
}

/* Implementação de Catmull-Rom splines
 * Características:
 * - Interpolação suave entre pontos de controle
 * - Continuidade C1
 * - Útil para animação e trajetórias suaves
 */
Curve evalCatmullRom(const std::vector<Vector3f>& points, unsigned steps) {
    Curve curve;
    
    if (points.size() < 4) {
        std::cerr << "Catmull-Rom requires at least 4 control points" << std::endl;
        return curve;
    }
    
    // Matriz base Catmull-Rom
    // [0.5  -0.5   0.5  -0.5]
    // [-1.5   2   -2    1.5]
    // [1.5   -2.5   2   -1  ]
    // [-0.5   1   -0.5   0  ]
    
    for (size_t i = 1; i < points.size() - 2; i++) {
        Vector3f p0 = points[i-1];
        Vector3f p1 = points[i];
        Vector3f p2 = points[i+1];
        Vector3f p3 = points[i+2];
        
        for (unsigned j = 0; j <= steps; j++) {
            float t = (float)j / steps;
            float t2 = t * t;
            float t3 = t2 * t;
            
            // Calcula posição usando a matriz de Catmull-Rom
            Vector3f V = 0.5f * (
                (-t3 + 2*t2 - t) * p0 +
                (3*t3 - 5*t2 + 2) * p1 +
                (-3*t3 + 4*t2 + t) * p2 +
                (t3 - t2) * p3
            );
            
            // Calcula tangente (primeira derivada)
            Vector3f T = 0.5f * (
                (-3*t2 + 4*t - 1) * p0 +
                (9*t2 - 10*t) * p1 +
                (-9*t2 + 8*t + 1) * p2 +
                (3*t2 - 2*t) * p3
            ).normalized();
            
            // Calcula sistema de coordenadas local
            Vector3f N, B;
            Vector3f tmp = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            B = Vector3f::cross(T, tmp).normalized();
            N = Vector3f::cross(B, T).normalized();
            
            CurvePoint cp = {V, T, N, B};
            curve.push_back(cp);
        }
    }
    
    return curve;
}
