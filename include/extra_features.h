#ifndef EXTRA_FEATURES_H
#define EXTRA_FEATURES_H

#include "curve.h"
#include <vecmath.h>
#include <vector>

/* Funcionalidades Adicionais
 * Este arquivo contém extensões do projeto básico:
 * 1. Materiais e texturas personalizáveis
 * 2. Splines Catmull-Rom para interpolação suave
 * 3. Interface para edição de curvas
 */

// Estrutura para materiais
struct Material {
    float ambient[4];   // Componente ambiente
    float diffuse[4];   // Componente difusa
    float specular[4];  // Componente especular
    float shininess;    // Brilho
};

// Materiais predefinidos
extern const Material MATERIAL_GOLD;
extern const Material MATERIAL_GLASS;
extern const Material MATERIAL_RUBY;
extern const Material MATERIAL_CHROME;
extern const Material MATERIAL_PEARL;
extern int NUM_MATERIALS; //Número total de materiais

// Funções para materiais
void applyMaterial(const Material& mat);
const Material& getMaterialByIndex(int index); // Função para obter material por índice

// Funções para splines Catmull-Rom
Curve evalCatmullRom(const std::vector<Vector3f>& points, unsigned steps);

#endif // EXTRA_FEATURES_H
