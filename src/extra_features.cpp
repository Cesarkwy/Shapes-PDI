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
/**
 * @struct Material
 * @brief Define as propriedades de um material para renderização.
 * @details Inclui componentes de cor ambiente, difusa, especular e o brilho (shininess)
 *          que determinam como a luz interage com a superfície de um objeto.
 */
struct Material {
    float ambient[4];   ///< Cor ambiente (R, G, B, A)
    float diffuse[4];   ///< Cor difusa (R, G, B, A)
    float specular[4];  ///< Cor especular (R, G, B, A)
    float shininess;    ///< Expoente de brilho especular
};

// Alguns materiais predefinidos
/**
 * @brief Material predefinido que simula ouro.
 */
const Material MATERIAL_GOLD = {
    {0.24725f, 0.1995f, 0.0745f, 1.0f},    // ambiente
    {0.75164f, 0.60648f, 0.22648f, 1.0f},  // difusa
    {0.628281f, 0.555802f, 0.366065f, 1.0f}, // especular
    51.2f                                   // brilho
};

/**
 * @brief Material predefinido que simula vidro, com transparência.
 */
const Material MATERIAL_GLASS = {
    {0.1f, 0.1f, 0.1f, 0.5f},  // ambiente
    {0.4f, 0.4f, 0.4f, 0.5f},  // difusa
    {0.9f, 0.9f, 0.9f, 0.5f},  // especular
    96.0f                       // brilho
};

const Material MATERIAL_RUBY = {
    {0.1745f, 0.01175f, 0.01175f, 1.0f},    // ambiente
    {0.61424f, 0.04136f, 0.04136f, 1.0f},   // difusa
    {0.727811f, 0.626959f, 0.626959f, 1.0f}, // especular
    76.8f                                    // brilho
};

const Material MATERIAL_CHROME = {
    {0.25f, 0.25f, 0.25f, 1.0f},     // ambiente
    {0.4f, 0.4f, 0.4f, 1.0f},        // difusa
    {0.774597f, 0.774597f, 0.774597f, 1.0f}, // especular
    76.8f                                    // brilho
};

const Material MATERIAL_PEARL = {
    {0.25f, 0.20725f, 0.20725f, 1.0f},      // ambiente
    {1.0f, 0.829f, 0.829f, 1.0f},           // difusa
    {0.296648f, 0.296648f, 0.296648f, 1.0f}, // especular
    11.264f                                  // brilho
};

// Definição da variável global
int NUM_MATERIALS = 5;  // Número total de materiais


/**
 * @brief Aplica as propriedades de um material ao estado atual do OpenGL.
 * @param mat O material a ser aplicado.
 * @details Esta função configura as propriedades de material (ambiente, difuso, especular e brilho)
 *          para as faces frontais dos polígonos a serem renderizados.
 */
void applyMaterial(const Material& mat) {
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat.ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat.diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat.specular);
    glMaterialf(GL_FRONT, GL_SHININESS, mat.shininess);
}

// Retorna o material correspondente ao índice
const Material& getMaterialByIndex(int index) {
    switch(index) {
        case 0: return MATERIAL_GOLD;
        case 1: return MATERIAL_GLASS;
        case 2: return MATERIAL_RUBY;
        case 3: return MATERIAL_CHROME;
        case 4: return MATERIAL_PEARL;
        default: return MATERIAL_GOLD;
    }
}


/* Implementação de Catmull-Rom splines
 * Características:
 * - Interpolação suave entre pontos de controle
 * - Continuidade C1
 * - Útil para animação e trajetórias suaves
 */

/**
 * @brief Avalia uma curva spline de Catmull-Rom a partir de um conjunto de pontos de controle.
 * @param points Vetor de pontos de controle (Vector3f) que definem a trajetória da spline.
 * @param steps O número de segmentos a serem gerados entre cada par de pontos de controle.
 * @return Uma `Curve` que representa a spline interpolada.
 * @details A spline de Catmull-Rom é uma curva interpoladora que passa por todos os pontos de controle,
 *          exceto o primeiro e o último. Ela requer pelo menos 4 pontos de controle para ser definida.
 *          A função calcula a posição (V), a tangente (T), a normal (N) e a binormal (B) para cada ponto da curva,
 *          formando um sistema de coordenadas local (frame de Frenet).
 */
Curve evalCatmullRom(const std::vector<Vector3f>& points, unsigned steps) {
    Curve curve;
    
    if (points.size() < 4) {
        std::cerr << "Catmull-Rom requer pelo menos 4 pontos de controle" << std::endl;
        return curve;
    }
    
    // Matriz base de Catmull-Rom
    // [ 0.5  -0.5   0.5  -0.5 ]
    // [ -1.5   2.0  -2.0   1.5 ]
    // [ 1.5  -2.5   2.0  -1.0 ]
    // [ -0.5   1.0  -0.5   0.0 ]
    
    for (size_t i = 1; i < points.size() - 2; i++) {
        Vector3f p0 = points[i-1];
        Vector3f p1 = points[i];
        Vector3f p2 = points[i+1];
        Vector3f p3 = points[i+2];
        
        for (unsigned j = 0; j <= steps; j++) {
            float t = (float)j / steps;
            float t2 = t * t;
            float t3 = t2 * t;
            
            // Calcula a posição usando a formulação de Catmull-Rom
            Vector3f V = 0.5f * (
                (-t3 + 2*t2 - t) * p0 +
                (3*t3 - 5*t2 + 2) * p1 +
                (-3*t3 + 4*t2 + t) * p2 +
                (t3 - t2) * p3
            );
            
            // Calcula a tangente (primeira derivada)
            Vector3f T = 0.5f * (
                (-3*t2 + 4*t - 1) * p0 +
                (9*t2 - 10*t) * p1 +
                (-9*t2 + 8*t + 1) * p2 +
                (3*t2 - 2*t) * p3
            ).normalized();
            
            // Calcula o sistema de coordenadas local (frame de Frenet)
            Vector3f N, B;
            // Escolhe um vetor temporário não colinear com a tangente para evitar problemas de degeneração.
            Vector3f tmp = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            B = Vector3f::cross(T, tmp).normalized();
            N = Vector3f::cross(B, T).normalized();
            
            CurvePoint cp = {V, T, N, B};
            curve.push_back(cp);
        }
    }
    
    return curve;
}
