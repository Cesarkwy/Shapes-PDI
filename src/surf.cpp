#include "surf.h"
#include "extra.h"
#include <cmath>
using namespace std;

namespace
{
    // Verifica se uma curva está no plano xy
    static bool checkFlat(const Curve &profile) {
        for (unsigned i = 0; i < profile.size(); i++) {
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
        }
        return true;
    }

    // Cria uma matriz de transformação a partir dos vetores TNB e ponto V
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

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    const size_t nProfilePts = profile.size();
    const size_t nCirclePts = steps + 1;

    // Para cada ponto do perfil
    for (size_t i = 0; i < nProfilePts; i++) {
        const Vector3f& V = profile[i].V;
        const Vector3f& N = profile[i].N;
        
        // Gera círculo de pontos
        for (size_t j = 0; j < nCirclePts; j++) {
            double theta = 2.0 * M_PI * double(j) / steps;
            double cosTheta = cos(theta);
            double sinTheta = sin(theta);
            
            // Rotaciona o ponto V em torno do eixo y
            Vector3f rotatedV(
                V[0] * cosTheta,  // x' = x*cos(θ)
                V[1],            // y' = y (não muda)
                V[0] * sinTheta   // z' = x*sin(θ)
            );
            
            // Rotaciona a normal
            Vector3f rotatedN(
                N[0] * cosTheta,
                N[1],
                N[0] * sinTheta
            );
            
            // Adiciona o vértice e sua normal
            surface.VV.push_back(rotatedV);
            surface.VN.push_back(rotatedN.normalized());
            
            // Gera triângulos (exceto para o último ponto do perfil)
            if (i < nProfilePts - 1 && j < steps) {
                unsigned int i0 = i * nCirclePts + j;
                unsigned int i1 = i0 + 1;
                unsigned int i2 = (i + 1) * nCirclePts + j;
                unsigned int i3 = i2 + 1;
                
                surface.VF.push_back(Tup3u(i0, i2, i1));
                surface.VF.push_back(Tup3u(i1, i2, i3));
            }
        }
    }
    
    return surface;
}

Surface makeGeneralCylinder(const Curve &profile, const Curve &sweep)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "Profile curve must be flat on xy plane." << endl;
        exit(0);
    }
    
    /* Implementação de Cilindro Generalizado
     * 1. A curva de perfil é movida ao longo da curva de varredura
     * 2. O sistema de coordenadas local da curva de varredura é usado
     * 3. A orientação do perfil é mantida consistente usando Frenet frame
     */
    
    // Para cada ponto na curva de varredura
    for (unsigned i = 0; i < sweep.size(); i++) {
        Matrix4f frame = createFrame(sweep[i].T, sweep[i].N, sweep[i].B, sweep[i].V);
        
        // Para cada ponto no perfil
        for (unsigned j = 0; j < profile.size(); j++) {
            Vector3f V = profile[j].V;
            Vector3f N = profile[j].N;
            
            // Transforma o ponto e a normal para o sistema de coordenadas local
            Vector4f vertex(V[0], V[1], V[2], 1.0f);
            Vector4f normal(N[0], N[1], N[2], 0.0f);
            
            vertex = frame * vertex;
            normal = frame * normal;
            
            surface.VV.push_back(Vector3f(vertex[0], vertex[1], vertex[2]));
            surface.VN.push_back(Vector3f(normal[0], normal[1], normal[2]).normalized());
            
            // Gera triângulos
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

Surface makeGenCyl(const Curve &profile, const Curve &sweep)
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // Cilindro generalizado é igual ao GeneralCylinder
    return makeGeneralCylinder(profile, sweep);

    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
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

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
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

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
