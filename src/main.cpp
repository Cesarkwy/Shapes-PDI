#ifdef WIN32
#include <windows.h>
#endif

#include <cmath>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>

#include <GL/glut.h>
#include <vecmath.h>

#include "parse.h"
#include "curve.h"
#include "surf.h"
#include "extra.h"
#include "camera.h"
#include "extra_features.h"

using namespace std;


// If you're really interested in what "namespace" means, see
// Stroustup.  But basically, the functionality of putting all the
// globals in an "unnamed namespace" is to ensure that everything in
// here is only accessible to code in this file.
namespace
{
    // Variáveis globais aqui.

    // Esta é a câmera
    Camera camera;

    // Estas são variáveis de estado para a interface do usuário
    bool gMousePressed = false;
    int  gCurveMode = 1;
    int  gSurfaceMode = 1;
    int  gPointMode = 1;
    int gCurrentMaterial = 0; // Índice do material atual

    // Isso determina o quão grande desenhar as normais
    const float gLineLen = 0.1f;
    
    // Estes são arrays para listas de exibição para cada modo de desenho. A
    // convenção é que o modo de desenho 0 é "em branco", e outros modos de
    // desenho apenas chamam as listas de exibição apropriadas.
    GLuint gCurveLists[3];
    GLuint gSurfaceLists[3];
    GLuint gAxisList;
    GLuint gPointList;
   
    // Estes vetores STL armazenam os pontos de controle, curvas e
    // superfícies que acabarão sendo desenhados. Além disso, vetores
    // STL paralelos armazenam os nomes para as curvas e superfícies (conforme
    // fornecido pelos arquivos).
    vector<vector<Vector3f> > gCtrlPoints;
    vector<Curve> gCurves;
    vector<string> gCurveNames;
    vector<Surface> gSurfaces;
    vector<string> gSurfaceNames;

    // Declarações de funções cujas implementações ocorrem mais tarde.
    void arcballRotation(int endX, int endY);
    void keyboardFunc( unsigned char key, int x, int y);
    void specialFunc( int key, int x, int y );
    void mouseFunc(int button, int state, int x, int y);
    void motionFunc(int x, int y);
    void reshapeFunc(int w, int h);
    void drawScene(void);
    void initRendering();
    void loadObjects(int argc, char *argv[]);
    void makeDisplayLists();

    // Esta função é chamada sempre que uma tecla "Normal" é pressionada.
    void keyboardFunc( unsigned char key, int x, int y )
    {
        switch ( key )
        {
        case 27: // Tecla Escape
            exit(0);
            break;
        case ' ':
        {
            Matrix4f eye = Matrix4f::identity();
            camera.SetRotation(eye);
            camera.SetCenter(Vector3f(0,0,0));
            break;
        }
        case 'c':
        case 'C':
            gCurveMode = (gCurveMode+1)%3;
            break;
        case 's':
        case 'S':
            gSurfaceMode = (gSurfaceMode+1)%3;
            break;
        case 'p':
        case 'P':
            gPointMode = (gPointMode+1)%2;
            break;  
        case 'm':
        case 'M':
            gCurrentMaterial = (gCurrentMaterial + 1) % NUM_MATERIALS;
            makeDisplayLists(); // Recria as listas de exibição para aplicar o novo material
            cout << "Material atual: " << gCurrentMaterial << endl;
            break;          
        default:
            cout << "Tecla nao tratada " << key << "." << endl;        
        }

        glutPostRedisplay();
    }

    // Esta função é chamada sempre que uma tecla "Especial" é pressionada.
    // No momento, ela não faz nada.
    void specialFunc( int key, int x, int y )
    {
		/*
        switch ( key )
        {
		default:
			break;
        }
		*/

        //glutPostRedisplay();
    }

    // Chamada quando o botão do mouse é pressionado.
    void mouseFunc(int button, int state, int x, int y)
    {
        if (state == GLUT_DOWN)
        {
            gMousePressed = true;
            
            switch (button)
            {
            case GLUT_LEFT_BUTTON:
                camera.MouseClick(Camera::LEFT, x, y);
                break;
            case GLUT_MIDDLE_BUTTON:
                camera.MouseClick(Camera::MIDDLE, x, y);
                break;
            case GLUT_RIGHT_BUTTON:
                camera.MouseClick(Camera::RIGHT, x,y);
            default:
                break;
            }                       
        }
        else
        {
            camera.MouseRelease(x,y);
            gMousePressed = false;
        }
        glutPostRedisplay();
    }

    // Chamada quando o mouse é movido com o botão pressionado.
    void motionFunc(int x, int y)
    {
        camera.MouseDrag(x,y);        
    
        glutPostRedisplay();
    }

    // Chamada quando a janela é redimensionada
    // w, h - largura e altura da janela em pixels.
    void reshapeFunc(int w, int h)
    {
        camera.SetDimensions(w,h);

        camera.SetViewport(0,0,w,h);
        camera.ApplyViewport();

        // Configura uma visão em perspectiva, com proporção quadrada
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        camera.SetPerspective(50);
        camera.ApplyPerspective();
    }

    // Esta função é responsável por exibir o objeto.
    void drawScene(void)
    {
        // Limpa a janela de renderização
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode( GL_MODELVIEW );  
        glLoadIdentity();              

        // Cor da luz (RGBA)
        GLfloat Lt0diff[] = {1.0,1.0,1.0,1.0};
        GLfloat Lt0pos[] = {3.0,3.0,5.0,1.0};
        glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
        glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);

        camera.ApplyModelview();

        // Chama as listas de exibição relevantes.
        if (gSurfaceMode)
            glCallList(gSurfaceLists[gSurfaceMode]);

        if (gCurveMode)
            glCallList(gCurveLists[gCurveMode]);

        // Isso desenha os eixos de coordenadas quando você está girando, para
        // se manter orientado.
        if (gMousePressed)
        {
            glPushMatrix();
            glTranslated(camera.GetCenter()[0], camera.GetCenter()[1], camera.GetCenter()[2]);
            glCallList(gAxisList);
            glPopMatrix();
        }

        if (gPointMode)
            glCallList(gPointList);
                 
        // Envia a imagem para a tela.
        glutSwapBuffers();


    }

    // Inicializa os modos de renderização do OpenGL
    void initRendering()
    {
        glEnable(GL_DEPTH_TEST);   // Teste de profundidade deve estar ativado
        glEnable(GL_LIGHTING);     // Habilita cálculos de iluminação
        glEnable(GL_LIGHT0);       // Liga a luz #0.

        // Configuração do desenho de polígonos
        glShadeModel(GL_SMOOTH);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Antialiasing
        // Isso parece ruim
        /*
          glEnable(GL_BLEND);
          glEnable(GL_POINT_SMOOTH);
          glEnable(GL_LINE_SMOOTH);
          glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);    
          glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        */
        
        // Limpa para preto
        glClearColor(0,0,0,1);

        // Cores base do material (não mudam)
        GLfloat diffColor[] = {0.4, 0.4, 0.4, 1};
        GLfloat specColor[] = {0.9, 0.9, 0.9, 1};
        GLfloat shininess[] = {50.0};

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffColor);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specColor);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }

    // Carrega objetos da entrada padrão para as variáveis globais:
    // gCtrlPoints, gCurves, gCurveNames, gSurfaces, gSurfaceNames. Se o
    // carregamento falhar, o programa será encerrado.
    void loadObjects(int argc, char *argv[])
    {
        if (argc < 2)
        {
            cerr<< "uso: " << argv[0] << " ARQUIVO_SWP [PREFIXO_OBJ] " << endl;
            exit(0);
        }

        ifstream in(argv[1]);
        if (!in)
        {
            cerr<< argv[1] << " nao encontrado\a" << endl;
            exit(0);
        }

        
        cerr << endl << "*** carregando e construindo curvas e superficies ***" << endl;
        
        if (!parseFile(in, gCtrlPoints,
                       gCurves, gCurveNames,
                       gSurfaces, gSurfaceNames))
        {
            cerr << "\aErro no formato do arquivo\a" << endl;
            in.close();
            exit(-1);              
        }

        in.close();

        // Isso faz a saída do arquivo OBJ
        if (argc > 2)
        {
            cerr << endl << "*** escrevendo arquivos obj ***" << endl;
            
            string prefix(argv[2]);

            for (unsigned i=0; i<gSurfaceNames.size(); i++)
            {
                if (gSurfaceNames[i] != ".")
                {
                    string filename =
                        prefix + string("_")
                        + gSurfaceNames[i]
                        + string(".obj");

                    ofstream out(filename.c_str());

                    if (!out)
                    {
                        cerr << "\aNao foi possivel abrir o arquivo " << filename << ", pulando"<< endl;
                        out.close();
                        continue;
                    }
                    else
                    {
                        outputObjFile(out, gSurfaces[i]);
                        cerr << "escreveu " << filename <<  endl;
                    }
                }
            }
            
        }

        cerr << endl << "*** pronto ***" << endl;


    }

    void makeDisplayLists()
    {
        gCurveLists[1] = glGenLists(1);
        gCurveLists[2] = glGenLists(1);
        gSurfaceLists[1] = glGenLists(1);
        gSurfaceLists[2] = glGenLists(1);
        gAxisList = glGenLists(1);
        gPointList = glGenLists(1);

        // Compile the display lists
        
        glNewList(gCurveLists[1], GL_COMPILE);
        {
            for (unsigned i=0; i<gCurves.size(); i++)
                drawCurve(gCurves[i], 0.0);
        }
        glEndList();
                
        glNewList(gCurveLists[2], GL_COMPILE);
        {
            for (unsigned i=0; i<gCurves.size(); i++)
                drawCurve(gCurves[i], gLineLen);
        }
        glEndList();
        
        glNewList(gSurfaceLists[1], GL_COMPILE);
        {
            applyMaterial(getMaterialByIndex(gCurrentMaterial));  // Aplicar material atual
            for (unsigned i=0; i<gSurfaces.size(); i++)
                drawSurface(gSurfaces[i], true);
        }
        glEndList();

        glNewList(gSurfaceLists[2], GL_COMPILE);
        {
            applyMaterial(getMaterialByIndex(gCurrentMaterial));  // Aplicar material atual
            for (unsigned i=0; i<gSurfaces.size(); i++)
            {
                drawSurface(gSurfaces[i], false);
                drawNormals(gSurfaces[i], gLineLen);
            }
        }
        glEndList();

        glNewList(gAxisList, GL_COMPILE);
        {
            // Save current state of OpenGL
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            // This is to draw the axes when the mouse button is down
            glDisable(GL_LIGHTING);
            glLineWidth(3);
            glPushMatrix();
            glScaled(5.0,5.0,5.0);
            glBegin(GL_LINES);
            glColor4f(1,0.5,0.5,1); glVertex3d(0,0,0); glVertex3d(1,0,0);
            glColor4f(0.5,1,0.5,1); glVertex3d(0,0,0); glVertex3d(0,1,0);
            glColor4f(0.5,0.5,1,1); glVertex3d(0,0,0); glVertex3d(0,0,1);

            glColor4f(0.5,0.5,0.5,1);
            glVertex3d(0,0,0); glVertex3d(-1,0,0);
            glVertex3d(0,0,0); glVertex3d(0,-1,0);
            glVertex3d(0,0,0); glVertex3d(0,0,-1);
        
            glEnd();
            glPopMatrix();
            
            glPopAttrib();
        }
        glEndList();

        glNewList(gPointList, GL_COMPILE);
        {
            // Save current state of OpenGL
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            // Setup for point drawing
            glDisable(GL_LIGHTING);    
            glColor4f(1,1,0.0,1);
            glPointSize(4);
            glLineWidth(1);

            for (unsigned i=0; i<gCtrlPoints.size(); i++)
            {
                glBegin(GL_POINTS);
                for (unsigned j=0; j<gCtrlPoints[i].size(); j++)
                    glVertex(gCtrlPoints[i][j]);
                glEnd();

                glBegin(GL_LINE_STRIP);
                for (unsigned j=0; j<gCtrlPoints[i].size(); j++)
                    glVertex(gCtrlPoints[i][j]);
                glEnd();
            }
            
            glPopAttrib();
        }
        glEndList();

    }
    
}

// Rotina principal.
// Configura o OpenGL, define os callbacks e inicia o loop principal
int main( int argc, char* argv[] )
{

    // Carrega da entrada padrão
    loadObjects(argc, argv);

    glutInit(&argc,argv);

    // Vamos animar, então usamos buffer duplo
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

    // Parâmetros iniciais para posição e tamanho da janela
    glutInitWindowPosition( 60, 60 );
    glutInitWindowSize( 600, 600 );
    
    camera.SetDimensions(600, 600);

    camera.SetDistance(10);
    camera.SetCenter(Vector3f(0,0,0));
    
    glutCreateWindow("Assignment 1");

    // Inicializa os parâmetros do OpenGL.
    initRendering();

    // Configura as funções de callback para pressionamentos de tecla
    glutKeyboardFunc(keyboardFunc); // Lida com símbolos ascii "normais"
    glutSpecialFunc(specialFunc);   // Lida com teclas "especiais" do teclado

    // Configura as funções de callback para o mouse
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);

    // Configura a função de callback para redimensionamento de janelas
    glutReshapeFunc( reshapeFunc );

    // Chama sempre que a janela precisar ser redesenhada
    glutDisplayFunc( drawScene );

    // Dispara timerFunc a cada 20 ms
    //  glutTimerFunc(20, timerFunc, 0);

    makeDisplayLists();
        
    // Inicia o loop principal. glutMainLoop nunca retorna.
    glutMainLoop();

    return 0;	// Esta linha nunca é alcançada.
}
