#include "surf.h"
#include "extra.h"
#include <cmath>
using namespace std;

namespace
{
    /**
     * @brief Verifica se uma curva de perfil está contida no plano XY.
     * @param profile A curva a ser verificada.
     * @return `true` se todos os pontos da curva e seus vetores tangente e normal têm a componente Z igual a zero, `false` caso contrário.
     * @details Esta verificação é um pré-requisito para a geração de superfícies de revolução e cilindros generalizados,
     *          que assumem que o perfil é 2D.
     */
    static bool checkFlat(const Curve &profile) {
        for (unsigned i = 0; i < profile.size(); i++) {
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
        }
        return true;
    }

    /**
     * @brief Cria uma matriz de transformação 4x4 a partir de um sistema de coordenadas local (frame de Frenet).
     * @param T O vetor Tangente.
     * @param N O vetor Normal.
     * @param B O vetor Binormal.
     * @param V O ponto de origem do sistema de coordenadas (vértice).
     * @return Uma `Matrix4f` que representa a transformação do espaço do objeto para o espaço do mundo,
     *         posicionando e orientando um objeto de acordo com o frame fornecido.
     * @details A matriz é construída com os vetores N, B, T como as três primeiras colunas (base ortonormal)
     *          e o vetor V como a quarta coluna (translação).
     */
    static Matrix4f createFrame(const Vector3f& T, const Vector3f& N, 
                              const Vector3f& B, const Vector3f& V) {
        Matrix4f M;
        M.setCol(0, Vector4f(N[0], N[1], N[2], 0));
        M.setCol(1, Vector4f(B[0], B[1], B[2], 0));
        M.setCol(2, Vector4f(T[0], T[1], T[2], 0));
        M.setCol(3, Vector4f(V[0], V[1], V[2], 1));
        return M;
    }
}

/**
 * @brief Cria uma superfície de revolução rotacionando uma curva de perfil em torno do eixo Y.
 * @param profile A curva de perfil, que deve estar no plano XY.
 * @param steps O número de segmentos a serem usados na rotação (resolução da superfície).
 * @return Uma `Surface` representando o objeto 3D gerado.
 * @details A função pega cada ponto da curva de perfil e o rotaciona em `steps` passos ao redor do eixo Y,
 *          formando uma série de anéis. Os vértices e normais são calculados para cada ponto rotacionado.
 *          A malha de triângulos (faces) é construída conectando os vértices de anéis adjacentes.
 */
Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "A curva de perfil para makeSurfRev deve estar no plano xy." << endl;
        exit(0);
    }

    const size_t nProfilePts = profile.size();
    const size_t nCirclePts = steps + 1;

    // Para cada ponto do perfil
    for (size_t i = 0; i < nProfilePts; i++) {
        const Vector3f& V = profile[i].V;
        const Vector3f& N = profile[i].N;
        
        // Gera um círculo de pontos rotacionando o ponto do perfil
        for (size_t j = 0; j < nCirclePts; j++) {
            double theta = 2.0 * M_PI * double(j) / steps;
            double cosTheta = cos(theta);
            double sinTheta = sin(theta);
            
            // Rotaciona o ponto V em torno do eixo y
            // A coordenada x do perfil se torna o raio da rotação.
            Vector3f rotatedV(
                V[0] * cosTheta,  // x' = r * cos(θ), onde r = V[0]
                V[1],             // y' = y (altura não muda)
                V[0] * sinTheta   // z' = r * sin(θ)
            );
            
            // Rotaciona a normal da mesma forma que o vértice
            Vector3f rotatedN(
                N[0] * cosTheta,
                N[1],
                N[0] * sinTheta
            );
            
            // Adiciona o vértice e sua normal à superfície
            surface.VV.push_back(rotatedV);
            surface.VN.push_back(rotatedN.normalized());
            
            // Gera triângulos para conectar os anéis de vértices
            if (i < nProfilePts - 1 && j < steps) {
                unsigned int i0 = i * nCirclePts + j;
                unsigned int i1 = i0 + 1;
                unsigned int i2 = (i + 1) * nCirclePts + j;
                unsigned int i3 = i2 + 1;
                
                // Cria dois triângulos para formar um quadrilátero
                surface.VF.push_back(Tup3u(i0, i2, i1));
                surface.VF.push_back(Tup3u(i1, i2, i3));
            }
        }
    }
    
    return surface;
}

/**
 * @brief Cria uma superfície de cilindro generalizado, varrendo uma curva de perfil ao longo de uma curva de trajetória.
 * @param profile A curva de perfil 2D (deve estar no plano XY).
 * @param sweep A curva de trajetória 3D ao longo da qual o perfil é varrido.
 * @return Uma `Surface` representando o objeto 3D gerado.
 * @details Para cada ponto na curva de trajetória (`sweep`), a função cria um sistema de coordenadas local (frame de Frenet).
 *          A curva de perfil é então transformada para este sistema de coordenadas, efetivamente "carimbando" o perfil
 *          na posição e orientação corretas ao longo da trajetória. A malha é construída conectando os perfis estampados adjacentes.
 */
Surface makeGeneralCylinder(const Curve &profile, const Curve &sweep)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "A curva de perfil deve estar no plano xy." << endl;
        exit(0);
    }
    
    /* Implementação de Cilindro Generalizado
     * 1. A curva de perfil é movida ao longo da curva de varredura
     * 2. O sistema de coordenadas local da curva de varredura é usado
     * 3. A orientação do perfil é mantida consistente usando o frame de Frenet
     */
    
    // Para cada ponto na curva de varredura (trajetória)
    for (unsigned i = 0; i < sweep.size(); i++) {
        // Cria a matriz de transformação para o sistema de coordenadas local no ponto atual da trajetória
        Matrix4f frame = createFrame(sweep[i].T, sweep[i].N, sweep[i].B, sweep[i].V);
        
        // Para cada ponto no perfil
        for (unsigned j = 0; j < profile.size(); j++) {
            Vector3f V = profile[j].V;
            Vector3f N = profile[j].N;
            
            // Transforma o ponto e a normal do perfil do seu espaço 2D para o espaço 3D da trajetória
            Vector4f vertex(V[0], V[1], V[2], 1.0f);
            Vector4f normal(N[0], N[1], N[2], 0.0f);
            
            vertex = frame * vertex;
            normal = frame * normal;
            
            surface.VV.push_back(Vector3f(vertex[0], vertex[1], vertex[2]));
            surface.VN.push_back(Vector3f(normal[0], normal[1], normal[2]).normalized());
            
            // Gera triângulos para conectar os perfis adjacentes
            if (i < sweep.size() - 1 && j < profile.size() - 1) {
                unsigned int i0 = i * profile.size() + j;
                unsigned int i1 = i0 + 1;
                unsigned int i2 = i0 + profile.size();
                unsigned int i3 = i2 + 1;
                
                surface.VF.push_back(Tup3u(i0, i2, i1));
                surface.VF.push_back(Tup3u(i1, i2, i3));
            }
        }
    }
    
    return surface;
}

/**
 * @brief Wrapper para `makeGeneralCylinder`. Cria um cilindro generalizado.
 * @param profile A curva de perfil 2D.
 * @param sweep A curva de trajetória 3D.
 * @return Uma `Surface` representando o objeto gerado.
 */
Surface makeGenCyl(const Curve &profile, const Curve &sweep)
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "A curva de perfil para genCyl deve estar no plano xy." << endl;
        exit(0);
    }

    // Cilindro generalizado é funcionalmente idêntico a makeGeneralCylinder
    return makeGeneralCylinder(profile, sweep);

    return surface;
}

/**
 * @brief Desenha uma superfície usando OpenGL.
 * @param surface A `Surface` a ser desenhada.
 * @param shaded Se `true`, desenha a superfície com sombreamento (iluminação e preenchimento).
 *               Se `false`, desenha em modo wireframe.
 * @details A função configura o modo de renderização do OpenGL (preenchido ou linha) e, em seguida,
 *          itera sobre todas as faces triangulares da superfície, enviando os vértices e suas normais
 *          correspondentes para a pipeline de renderização.
 */
void drawSurface(const Surface &surface, bool shaded)
{
    // Salva o estado atual do OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // Usa a cor do material e as posições de luz atuais.
        // Basta configurá-los em drawScene().
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Informa ao OpenGL para *não* desenhar triângulos virados para trás.
        // Isso é mais eficiente e ajuda a garantir que seus triângulos
        // sejam desenhados na ordem correta.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

/**
 * @brief Desenha as normais de cada vértice de uma superfície.
 * @param surface A `Surface` cujas normais serão desenhadas.
 * @param len O comprimento das linhas que representam as normais.
 * @details Esta função é útil para depuração, permitindo visualizar a direção das normais
 *          em cada vértice da malha. As normais são desenhadas como pequenas linhas saindo de cada vértice.
 */
void drawNormals(const Surface &surface, float len)
{
    // Salva o estado atual do OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1); // Cor ciano para as normais
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

/**
 * @brief Escreve os dados de uma superfície em um stream no formato de arquivo Wavefront OBJ.
 * @param out O stream de saída (por exemplo, um `ofstream`) para onde os dados serão escritos.
 * @param surface A `Surface` a ser exportada.
 * @details A função exporta os vértices (`v`), as normais dos vértices (`vn`) e as faces (`f`)
 *          da superfície. O formato de face inclui índices de vértice e de normal.
 */
void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl; // Coordenada de textura padrão (não usada aqui)
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1; // Índices em OBJ são baseados em 1
            out << a << "/" << "1" << "/" << a << " "; // Formato v/vt/vn
        }
        out << endl;
    }
}
