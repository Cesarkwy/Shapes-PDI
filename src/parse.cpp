#include "parse.h"
#include "curve.h"
#include "extra_features.h"
#include <map>
using namespace std;


namespace {

    /**
     * @brief Lê um vetor de pontos de controle de uma stream de entrada.
     * @param in A stream de entrada (geralmente um arquivo).
     * @param dim A dimensão dos pontos de controle (2 para 2D, 3 para 3D).
     * @return Um `vector<Vector3f>` contendo os pontos de controle lidos.
     * @details A função primeiro lê o número de pontos de controle e, em seguida, itera para ler cada ponto.
     *          Os pontos são esperados no formato "(x, y)" para 2D ou "(x, y, z)" para 3D.
     *          Pontos 2D são armazenados como `Vector3f(x, y, 0)`.
     */
    vector<Vector3f> readCps(istream &in, unsigned dim)
    {    
        // número de pontos de controle
        unsigned n;
        in >> n;

        cerr << "  " << n << " pontos de controle" << endl;
    
        // vetor de pontos de controle
        vector<Vector3f> cps(n);

        char delim;
        float x;
        float y;
        float z;

        for( unsigned i = 0; i < n; ++i )
        {
            switch (dim)
            {
            case 2:
                in >> delim; // Lê '('
                in >> x;
                in >> y;
                cps[i] = Vector3f( x, y, 0 );
                in >> delim; // Lê ')'
                break;
            case 3:
                in >> delim; // Lê '('
                in >> x;
                in >> y;
                in >> z;
                cps[i] = Vector3f( x, y, z );
                in >> delim; // Lê ')'
                break;            
            default:
                abort();
            }
        }

        return cps;
    }
}


/**
 * @brief Analisa um arquivo de cena para construir curvas e superfícies.
 * @param in A stream de entrada contendo a descrição da cena.
 * @param ctrlPoints Vetor de vetores para armazenar os pontos de controle de cada objeto.
 * @param curves Vetor para armazenar as curvas geradas.
 * @param curveNames Vetor para armazenar os nomes das curvas.
 * @param surfaces Vetor para armazenar as superfícies geradas.
 * @param surfaceNames Vetor para armazenar os nomes das superfícies.
 * @return `true` se a análise for bem-sucedida, `false` caso contrário.
 * @details A função lê a stream objeto por objeto, identificando seu tipo (Bézier, B-spline, superfície de revolução, etc.),
 *          lendo seus parâmetros e pontos de controle, e chamando as funções de avaliação apropriadas
 *          para gerar as curvas e superfícies. Os objetos nomeados são armazenados em mapas para referência futura
 *          (por exemplo, para criar uma superfície de revolução a partir de uma curva nomeada).
 */
bool parseFile(istream &in,
               vector<vector<Vector3f> > &ctrlPoints, 
               vector<Curve>             &curves,
               vector<string>            &curveNames,
               vector<Surface>           &surfaces,
               vector<string>            &surfaceNames)
{
    ctrlPoints.clear();
    curves.clear();
    curveNames.clear();
    surfaces.clear();
    surfaceNames.clear();    
    
    string objType;

    // Para procurar índices de curvas por nome
    map<string,unsigned> curveIndex;

    // Para procurar índices de superfícies por nome
    map<string,unsigned> surfaceIndex;
        
    // Para armazenar a dimensão da curva
    vector<unsigned> dims;

    unsigned counter = 0;
    
    while (in >> objType) 
    {
        cerr << ">objeto " << counter++ << endl;
        string objName;
        in >> objName;

        bool named = (objName != ".");
        
        vector<Vector3f> cpsToAdd;
        
        if (curveIndex.find(objName) != curveIndex.end() ||
            surfaceIndex.find(objName) != surfaceIndex.end())
        {
            cerr << "erro, [" << objName << "] ja existe" << endl;
            return false;
        }

        unsigned steps;

        if (objType == "bez2")
        {
            in >> steps;
            cerr << " lendo bez2 " << "[" << objName << "]" << endl;
            curves.push_back( evalBezier(cpsToAdd = readCps(in, 2), steps) );
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
            
        }
        else if (objType == "bsp2")
        {
            cerr << " lendo bsp2 " << "[" << objName << "]" << endl;
            in >> steps;
            curves.push_back( evalBspline(cpsToAdd = readCps(in, 2), steps) );
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
        }
        else if (objType == "bez3")
        {
            cerr << " lendo bez3 " << "[" << objName << "]" << endl;
            in >> steps;
            curves.push_back( evalBezier(cpsToAdd = readCps(in, 3), steps) );
            curveNames.push_back(objName);
            dims.push_back(3);
            if (named) curveIndex[objName] = dims.size()-1;

        }
        else if (objType == "bsp3")
        {
            cerr << " lendo bsp3 " << "[" << objName << "]" << endl;
            in >> steps;
            curves.push_back( evalBspline(cpsToAdd = readCps(in, 3), steps) );
            curveNames.push_back(objName);
            dims.push_back(3);
            if (named) curveIndex[objName] = dims.size()-1;
        }
        else if (objType == "srev")
        {
            cerr << " lendo srev " << "[" << objName << "]" << endl;
            in >> steps;

            // Nome da curva de perfil
            string profName;
            in >> profName;

            cerr << "  perfil [" << profName << "]" << endl;
            
            map<string,unsigned>::const_iterator it = curveIndex.find(profName);

            // Verificações de falha
            if (it == curveIndex.end()) {                
                cerr << "falha: [" << profName << "] nao existe!" << endl; return false;
            }
            if (dims[it->second] != 2) {
                cerr << "falha: [" << profName << "] nao e 2d!" << endl; return false;
            }

            // Cria a superfície
            surfaces.push_back( makeSurfRev( curves[it->second], steps ) );
            surfaceNames.push_back(objName);
            if (named) surfaceIndex[objName] = surfaceNames.size()-1;
        }
        // ---------------------------
        // Catmull-Rom (2D)
        // ---------------------------
        else if (objType == "catmull" || objType == "catmullrom")
        {
            cerr << " lendo catmull-rom " << "[" << objName << "]" << endl;
            in >> steps;
            // lê pontos 2D e avalia Catmull-Rom
            curves.push_back( evalCatmullRom(cpsToAdd = readCps(in, 2), steps) );
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
        }


        else if (objType == "gcyl")
        {
            cerr << " lendo gcyl " << "[" << objName << "]" << endl;
            
            // Nome da curva de perfil e da curva de varredura
            string profName, sweepName;
            in >> profName >> sweepName;

            cerr << "  perfil [" << profName << "], varredura [" << sweepName << "]" << endl;

            map<string,unsigned>::const_iterator itP, itS;

            // Verificações de falha para o perfil
            itP = curveIndex.find(profName);
            
            if (itP == curveIndex.end()) {                
                cerr << "falha: [" << profName << "] nao existe!" << endl; return false;
            }
            if (dims[itP->second] != 2) {
                cerr << "falha: [" << profName << "] nao e 2d!" << endl; return false;
            }

            // Verificações de falha para a varredura
            itS = curveIndex.find(sweepName);
            if (itS == curveIndex.end()) {                
                cerr << "falha: [" << sweepName << "] nao existe!" << endl; return false;
            }

            // Cria a superfície
            surfaces.push_back( makeGenCyl( curves[itP->second], curves[itS->second] ) );
            surfaceNames.push_back(objName);
            if (named) surfaceIndex[objName] = surfaceNames.size()-1;

        }
        else if (objType == "circ")
        {
            cerr << " lendo circ " << "[" << objName << "]" << endl;

            unsigned steps;
            float rad;
            in >> steps >> rad;
            cerr << "  raio [" << rad << "]" << endl;

            curves.push_back( evalCircle(rad, steps) );
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
        }
        else
        {
            cerr << "falha: tipo " << objType << " nao reconhecido." << endl;
            return false;
        }

        ctrlPoints.push_back(cpsToAdd);
    }

    return true;
}




