#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}
    

Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
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
    
    // Para cada segmento da curva
    for (size_t i = 0; i < P.size() - 3; i += 3) {
        Vector3f P0 = P[i];
        Vector3f P1 = P[i + 1];
        Vector3f P2 = P[i + 2];
        Vector3f P3 = P[i + 3];
        
        // Gera pontos ao longo do segmento
        for (unsigned j = 0; j <= steps; ++j) {
            float t = (float)j / steps;
            
            // Cálculo do ponto na curva usando a fórmula de Bézier
            float t2 = t * t;
            float t3 = t2 * t;
            float mt = 1 - t;
            float mt2 = mt * mt;
            float mt3 = mt2 * mt;
            
            // Posição do ponto (V)
            Vector3f V = mt3 * P0 + 3 * mt2 * t * P1 + 
                        3 * mt * t2 * P2 + t3 * P3;
            
            // Tangente (T) - primeira derivada normalizada
            Vector3f T = (-3 * mt2 * P0 + 3 * (1 - 4*t + 3*t2) * P1 + 
                         3 * (2*t - 3*t2) * P2 + 3 * t2 * P3).normalized();
            
            // Cálculo do frame TNB usando o método de Frenet
            Vector3f B, N;
            
            // Escolhe um vetor auxiliar não paralelo a T
            Vector3f aux = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            
            // Calcula B como produto vetorial de T e aux
            B = Vector3f::cross(T, aux).normalized();
            
            // Calcula N como produto vetorial de B e T
            N = Vector3f::cross(B, T).normalized();
            
            // Adiciona o ponto à curva
            CurvePoint cp;
            cp.V = V;  // Posição
            cp.T = T;  // Tangente
            cp.N = N;  // Normal
            cp.B = B;  // Binormal
            
            // Só adiciona se não for duplicado do último ponto
            if (curve.empty() || !approx(curve.back().V, cp.V)) {
                curve.push_back(cp);
            }
        }
    }

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;
    cerr << "\t>>> Returning empty curve." << endl;

    // Right now this will just return this empty curve.
    return Curve();
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
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
    
    // Para cada segmento da B-spline
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
            
            // Coeficientes da base B-spline
            float b0 = (-t3 + 3*t2 - 3*t + 1) / 6.0f;
            float b1 = (3*t3 - 6*t2 + 4) / 6.0f;
            float b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6.0f;
            float b3 = t3 / 6.0f;
            
            // Posição do ponto na curva
            Vector3f V = b0 * P0 + b1 * P1 + b2 * P2 + b3 * P3;
            
            // Derivadas da base para tangente
            float d0 = (-3*t2 + 6*t - 3) / 6.0f;
            float d1 = (9*t2 - 12*t) / 6.0f;
            float d2 = (-9*t2 + 6*t + 3) / 6.0f;
            float d3 = (3*t2) / 6.0f;
            
            // Tangente (primeira derivada)
            Vector3f T = (d0 * P0 + d1 * P1 + d2 * P2 + d3 * P3).normalized();
            
            // Sistema de coordenadas local (Frame de Frenet)
            Vector3f N, B;
            
            // Escolhe vetor auxiliar não paralelo a T
            Vector3f aux = (fabs(T[0]) < 0.9f) ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
            
            // Calcula B como produto vetorial de T e aux
            B = Vector3f::cross(T, aux).normalized();
            
            // Calcula N como produto vetorial de B e T
            N = Vector3f::cross(B, T).normalized();
            
            // Adiciona o ponto à curva
            CurvePoint cp;
            cp.V = V;
            cp.T = T;
            cp.N = N;
            cp.B = B;
            
            if (curve.empty() || !approx(curve.back().V, cp.V)) {
                curve.push_back(cp);
            }
        }
    }
    
    return curve;
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
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
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

