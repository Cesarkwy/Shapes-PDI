#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

namespace
{
    // Aproximadamente igual a. Não queremos usar == por causa de
    // problemas de precisão com ponto flutuante.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}
    
/**
 * @brief Avalia uma curva de Bézier a partir de um conjunto de pontos de controle.
 * @param P Um vetor de pontos de controle (Vector3f). O número de pontos deve ser 3n+1.
 * @param steps O número de segmentos a serem gerados para cada segmento de Bézier cúbico.
 * @return Uma `Curve` que representa a curva de Bézier interpolada.
 * @details A função processa segmentos de Bézier cúbicos (4 pontos de controle por vez).
 *          Para cada segmento, ela calcula a posição (V), a tangente (T), a normal (N) e a binormal (B)
 *          para um número `steps` de pontos, formando um sistema de coordenadas local (frame de Frenet).
 *          A curva resultante é uma composição de múltiplos segmentos de Bézier cúbicos.
 */
Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Verifica se o número de pontos de controle é válido (3n+1)
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier deve ser chamada com 3n+1 pontos de controle." << endl;
        exit( 0 );
    }

    /* Implementação de Curvas de Bézier
     *
     * 1. Equação da Curva de Bézier Cúbica:
     *    B(t) = (1-t)³P₀ + 3t(1-t)²P₁ + 3t²(1-t)P₂ + t³P₃
     *    onde t ∈ [0,1]
     *
     * 2. Cálculo da Tangente (primeira derivada):
     *    B'(t) = 3(1-t)²(P₁-P₀) + 6t(1-t)(P₂-P₁) + 3t²(P₃-P₂)
     *
     * 3. Sistema de Coordenadas Local (Frame de Frenet):
     *    - T: Tangente (direção do movimento)
     *    - N: Normal (perpendicular à tangente)
     *    - B: Binormal (produto vetorial de T e N)
     */
    
    Curve curve;
    
    // Para cada segmento da curva de Bézier (composto por 4 pontos)
    for (size_t i = 0; i < P.size() - 3; i += 3) {
        Vector3f P0 = P[i];
        Vector3f P1 = P[i + 1];
        Vector3f P2 = P[i + 2];
        Vector3f P3 = P[i + 3];
        
        // Gera pontos ao longo do segmento
        for (unsigned j = 0; j <= steps; ++j) {
            float t = (float)j / steps;
            
            // Cálculo do ponto na curva usando a fórmula de Bézier (polinômios de Bernstein)
            float t2 = t * t;
            float t3 = t2 * t;
            float mt = 1 - t;
            float mt2 = mt * mt;
            float mt3 = mt2 * mt;
            
            // Posição do ponto (V)
            Vector3f V = mt3 * P0 + 3 * mt2 * t * P1 + 
                        3 * mt * t2 * P2 + t3 * P3;
            
            // Tangente (T) - primeira derivada da curva de Bézier, normalizada
            Vector3f T = (-3 * mt2 * P0 + 3 * (1 - 4*t + 3*t2) * P1 + 
                         3 * (2*t - 3*t2) * P2 + 3 * t2 * P3).normalized();
            
            // Cálculo do frame TNB (Tangente, Normal, Binormal) usando o método de Frenet
            Vector3f B, N;
            
            // Escolhe um vetor auxiliar não paralelo a T para evitar degeneração no produto vetorial.
            // Se a componente x da tangente for pequena, usa o eixo y como auxiliar; caso contrário, usa o eixo x.
            Vector3f aux = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            
            // Calcula a Binormal (B) como o produto vetorial entre a Tangente (T) e o vetor auxiliar.
            B = Vector3f::cross(T, aux).normalized();
            
            // Calcula a Normal (N) como o produto vetorial entre a Binormal (B) e a Tangente (T),
            // garantindo que N seja ortogonal a T e B.
            N = Vector3f::cross(B, T).normalized();
            
            // Adiciona o ponto calculado à curva
            CurvePoint cp;
            cp.V = V;  // Posição
            cp.T = T;  // Tangente
            cp.N = N;  // Normal
            cp.B = B;  // Binormal
            
            // Só adiciona o ponto se ele não for uma duplicata do último ponto adicionado,
            // para evitar pontos redundantes.
            if (curve.empty() || !approx(curve.back().V, cp.V)) {
                curve.push_back(cp);
            }
        }
    }

    cerr << "\t>>> Pontos de controle (tipo vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Passos (tipo steps): " << steps << endl;
    cerr << "\t>>> Retornando curva vazia." << endl;

    // A implementação original retornava uma curva vazia. Agora retorna a curva calculada.
    return curve;
}

/**
 * @brief Avalia uma curva B-spline cúbica uniforme a partir de um conjunto de pontos de controle.
 * @param P Um vetor de pontos de controle (Vector3f). Requer pelo menos 4 pontos.
 * @param steps O número de segmentos a serem gerados para cada seção da B-spline.
 * @return Uma `Curve` que representa a curva B-spline.
 * @details A B-spline é uma curva de aproximação que não passa necessariamente pelos pontos de controle,
 *          mas oferece continuidade C² (curvatura contínua). A função usa a matriz de base B-spline
 *          para calcular a posição (V) e a tangente (T) em cada ponto. O sistema de coordenadas local
 *          (N e B) também é calculado.
 */
Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Verifica se há pontos de controle suficientes
    if( P.size() < 4 )
    {
        cerr << "evalBspline deve ser chamada com 4 ou mais pontos de controle." << endl;
        exit( 0 );
    }

    /* Implementação de B-splines Cúbicas
     *
     * 1. Matriz de Base B-spline uniforme:
     *    M = (1/6) * [-1  3 -3  1]
     *                 [ 3 -6  3  0]
     *                 [-3  0  3  0]
     *                 [ 1  4  1  0]
     *
     * 2. Equação da Curva:
     *    S(t) = 1/6 * [t³ t² t 1] * M * [P₀ P₁ P₂ P₃]ᵀ
     *         = (1/6) * [(-t³+3t²-3t+1)P₀ + (3t³-6t²+4)P₁ + 
     *                    (-3t³+3t²+3t+1)P₂ + (t³)P₃]
     *
     * 3. Propriedades:
     *    - Continuidade C² entre segmentos
     *    - Controle local (cada ponto afeta apenas 4 segmentos)
     *    - Curva não passa pelos pontos de controle
     */
    
    Curve curve;
    
    // Para cada segmento da B-spline (definido por 4 pontos de controle consecutivos)
    for (size_t i = 0; i < P.size() - 3; ++i) {
        Vector3f P0 = P[i];
        Vector3f P1 = P[i + 1];
        Vector3f P2 = P[i + 2];
        Vector3f P3 = P[i + 3];
        
        // Gera pontos ao longo do segmento
        for (unsigned j = 0; j <= steps; ++j) {
            float t = (float)j / steps;
            float t2 = t * t;
            float t3 = t2 * t;
            
            // Coeficientes da base B-spline (funções de base)
            float b0 = (-t3 + 3*t2 - 3*t + 1) / 6.0f;
            float b1 = (3*t3 - 6*t2 + 4) / 6.0f;
            float b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6.0f;
            float b3 = t3 / 6.0f;
            
            // Posição do ponto na curva (combinação linear dos pontos de controle com as funções de base)
            Vector3f V = b0 * P0 + b1 * P1 + b2 * P2 + b3 * P3;
            
            // Derivadas das funções de base para o cálculo da tangente
            float d0 = (-3*t2 + 6*t - 3) / 6.0f;
            float d1 = (9*t2 - 12*t) / 6.0f;
            float d2 = (-9*t2 + 6*t + 3) / 6.0f;
            float d3 = (3*t2) / 6.0f;
            
            // Tangente (T) - primeira derivada da curva, normalizada
            Vector3f T = (d0 * P0 + d1 * P1 + d2 * P2 + d3 * P3).normalized();
            
            // Sistema de coordenadas local (Frame de Frenet)
            Vector3f N, B;
            
            // Escolhe um vetor auxiliar não paralelo a T
            Vector3f aux = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            
            // Calcula a Binormal (B)
            B = Vector3f::cross(T, aux).normalized();
            
            // Calcula a Normal (N)
            N = Vector3f::cross(B, T).normalized();
            
            // Adiciona o ponto à curva
            CurvePoint cp;
            cp.V = V;
            cp.T = T;
            cp.N = N;
            cp.B = B;
            
            // Evita pontos duplicados
            if (curve.empty() || !approx(curve.back().V, cp.V)) {
                curve.push_back(cp);
            }
        }
    }
    
    return curve;
}

/**
 * @brief Gera uma curva circular no plano XY.
 * @param radius O raio do círculo.
 * @param steps O número de segmentos que formarão o círculo.
 * @return Uma `Curve` representando o círculo.
 * @details Esta função cria um círculo centrado na origem, no plano XY.
 *          Para cada ponto, ela calcula a posição (V), a tangente (T), a normal (N) e a binormal (B).
 *          A curva é gerada no sentido anti-horário.
 */
Curve evalCircle( float radius, unsigned steps )
{
    // Esta é uma função de exemplo sobre como inicializar corretamente uma Curva
    // (que é um vector<CurvePoint>).
    
    // Pré-aloca uma curva com steps+1 CurvePoints
    Curve R( steps+1 );

    // Preenche no sentido anti-horário
    for( unsigned i = 0; i <= steps; ++i )
    {
        // Passo de 0 a 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Inicializa a posição
        // Estamos girando no sentido anti-horário em torno do eixo y
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // O vetor tangente é a primeira derivada
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // O vetor normal é a segunda derivada
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finalmente, a binormal está apontando para cima.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

/**
 * @brief Desenha uma curva na tela usando OpenGL.
 * @param curve A `Curve` a ser desenhada.
 * @param framesize O tamanho dos eixos do sistema de coordenadas local a serem desenhados em cada ponto.
 *                  Se for 0, os eixos não são desenhados.
 * @details A função desenha a curva como uma `GL_LINE_STRIP`. Opcionalmente, pode desenhar
 *          o sistema de coordenadas local (frame de Frenet: Tangente, Normal, Binormal) em cada ponto da curva
 *          para visualização da orientação.
 */
void drawCurve( const Curve& curve, float framesize )
{
    // Salva o estado atual do OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Configuração para desenho de linha
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Desenha a curva
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Desenha os sistemas de coordenadas se framesize for diferente de zero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 ); // Eixo N em vermelho
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 ); // Eixo B em verde
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 ); // Eixo T em azul
            glEnd();
            glPopMatrix();
        }
    }
    
    // Restaura o estado
    glPopAttrib();
}

