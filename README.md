# Controle Fuzzy Aplicado a Robô Seguidor de Linha (GNU Octave)

Repositório destinado aos códigos-fonte e simulações computacionais do Trabalho de Conclusão de Curso (TCC) do **Bacharelado em Ciências Exatas e Tecnológicas (BCET)**.


Este trabalho consiste no desenvolvimento, modelagem e simulação de um **controlador baseado em Lógica Fuzzy** aplicado a um robô móvel autônomo seguidor de linha com acionamento diferencial. 

O ambiente de simulação foi implementado integralmente no **GNU Octave** por meio de blocos funcionais, utilizando uma matriz virtual de **8 sensores de refletância infravermelha** dispostos em linha. O estudo investiga o desempenho dinâmico do sistema e as tensões de acionamento dos motores diante de variações no raio de curvatura da pista, na velocidade de referência e nas condições de leitura dos sensores.

## Arquitetura da Simulação (Blocos Funcionais)

O sistema foi estruturado de forma modular no Octave para representar fielmente o comportamento físico e lógico do robô:

* **Geração de Trajetória:** Modelagem de circuitos com diferentes trechos e raios de curvatura.
* **Leitura de Sensores e Conversão de Sinais:** Simulação de uma barra com 8 sensores infravermelhos para detecção da linha e cálculo do erro de posição.
* **Controlador Fuzzy:** Sistema de inferência baseado em regras linguísticas para tomada de decisão contínua e adaptação a não linearidades.
* **Acionamento dos Motores:** Cálculo e resposta das tensões aplicadas aos atuadores esquerdo e direito.
* **Dinâmica e Cinemática do Robô:** Modelagem matemática discreta do deslocamento diferencial no plano.

## Métricas Avaliadas

Durante as simulações, o controlador é analisado sob os seguintes critérios:
* Erro de rastreamento da trajetória;
* Estabilidade durante trechos retos e curvas fechadas;
* Tempo necessário para completar o percurso;
* Comportamento das tensões aplicadas aos motores frente a desvios e ruídos.


## Autoria

* **Thaisy** — Graduanda do Bacharelado em Ciências Exatas e Tecnológicas (BCET) — [Perfil no GitHub](https://github.com)
