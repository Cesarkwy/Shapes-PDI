# Projeto de Computação Gráfica - Curvas, Superfícies e Extensões

Este projeto implementa conceitos avançados de computação gráfica, focando em curvas de Bézier, B-Splines e superfícies de revolução.

## Conceitos Matemáticos Fundamentais

### Curvas
- **Curvas de Bézier**: Curvas paramétricas que usam pontos de controle para definir sua forma
- **B-Splines**: Generalização das curvas de Bézier que oferece maior controle local
- **Sistema de Coordenadas Local**: 
  - T (Tangente): Vetor unitário na direção do movimento
  - N (Normal): Vetor perpendicular à tangente
  - B (Binormal): Produto vetorial de T e N

### Superfícies
1. **Superfícies de Revolução**:
   - Geradas pela rotação de uma curva perfil em torno do eixo y
   - Requer cálculo cuidadoso das normais para iluminação correta

2. **Cilindros Generalizados**:
   - Combinam curva de perfil com curva de varredura
   - Sistema de coordenadas local se move ao longo da curva de varredura

## Funcionalidades Adicionais

### 1. Materiais e Texturas
- **Materiais Predefinidos**:
  - Ouro: Superfície metálica dourada com brilho característico
  - Vidro: Material translúcido com alta especularidade
- **Sistema de Materiais**:
  - Componente ambiente: Luz mínima refletida
  - Componente difusa: Reflexão principal da superfície
  - Componente especular: Brilho e reflexões
  - Shininess: Controle do tamanho do brilho especular

### 2. Curvas Catmull-Rom
- **Interpolação Suave**:
  - Passa exatamente pelos pontos de controle
  - Mantém continuidade C1 entre segmentos
- **Matriz Base**:
  ```
  [0.5  -0.5   0.5  -0.5]
  [-1.5   2   -2    1.5]
  [1.5   -2.5   2   -1  ]
  [-0.5   1   -0.5   0  ]
  ```
- **Aplicações**:
  - Animação suave de objetos
  - Caminhos naturais para câmeras
  - Interpolação de formas

### 3. Interface Interativa
- **Edição de Curvas**:
  - Adição/remoção de pontos de controle
  - Manipulação direta dos pontos
  - Atualização em tempo real
- **Visualização**:
  - Preview da curva durante edição
  - Sistema de coordenadas local visível
  - Indicadores de direção da curva

## Dicas Importantes
- Implementar otimizações incrementais
- Manter sistemas de coordenadas consistentes
- Calcular normais corretamente para iluminação
- Usar estruturas de dados eficientes para malhas

## Formato de Arquivo
O projeto usa arquivos .SWP para definir curvas e superfícies, permitindo:
- Curvas Bézier
- B-Splines
- Círculos
- Superfícies de revolução
- Cilindros generalizados
