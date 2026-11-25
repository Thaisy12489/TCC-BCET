clear;
clc;
close all;
clear bloco_motor;   % zera a memória (persistent) da saída fuzzy

%% =========================================================
% BLOCO ANÁLISE DE RESULTADOS
% =========================================================
% São gerados, nesta ordem:
%   Parte A - gráficos de caracterização do controlador (metodologia)
%   Parte B - simulação de cenários de teste e gráficos de desempenho
%
% Arquivos necessários na mesma pasta:
%   trajetoria.m, simular_leitura_sensores.m, bloco_conversor.m,
%   bloco_controlador.m, bloco_motor.m, bloco_dinamica.m, rodar_cenario.m

%% 1. CONFIGURAÇÃO
Ts = 0.05;                 % tempo de amostragem [s]
salvar_figuras = true;    % true = as figuras são salvas em PNG na pasta atual
cenario_detalhe = 1;       % cenário cujas variáveis internas são detalhadas

figs = [];                 % identificadores das figuras
nomes_figs = {};           % nomes dos arquivos PNG

%% 2. CARREGA OS DADOS DA TRAJETÓRIA
[Xv, Yv, x0, y0, theta0] = trajetoria();
close(gcf);                % a figura aberta por trajetoria() não é utilizada


%% =========================================================
% PARTE A - CARACTERIZAÇÃO DO CONTROLADOR (METODOLOGIA)
% =========================================================

%% -----------------------------------------------------
% FIGURA A1: FUNÇÕES DE PERTINÊNCIA DAS ENTRADAS
% ------------------------------------------------------
% As três entradas compartilham os mesmos conjuntos (vetor P em
% bloco_controlador.m); no entanto, a entrada central recebe um offset de
% 50 em bloco_conversor.m, razão pela qual nunca atinge valores inferiores a 50.

valores = 0:1000;
per_alto  = zeros(size(valores));
per_medio = zeros(size(valores));
per_baixo = zeros(size(valores));

for i = 1:length(valores)
    Per = bloco_controlador(valores(i), 0, 0);   % [alto medio baixo]
    per_alto(i)  = Per(1);
    per_medio(i) = Per(2);
    per_baixo(i) = Per(3);
end

figs(end+1) = figure;
nomes_figs{end+1} = 'A1_pertinencia_entradas';
plot(valores, per_baixo, 'b', 'LineWidth', 2); hold on;
plot(valores, per_medio, 'k', 'LineWidth', 2);
plot(valores, per_alto,  'r', 'LineWidth', 2);
grid on;
ylim([0 1.1]);
xlabel('Leitura convertida');
ylabel('Grau de pertinência');
legend('baixo', 'médio', 'alto', 'Location', 'east');
title('Funções de pertinência das entradas (S_{dir}, S_{cen}, S_{esq})');

%% -----------------------------------------------------
% FIGURA A2: MODELO DO SENSOR
% ------------------------------------------------------
% Uma pista reta sobre o eixo X é empregada para que as leituras sejam
% obtidas em função exclusivamente do deslocamento lateral do robô, sendo
% este considerado positivo quando o robô se encontra à esquerda da linha
% (situação na qual a linha é vista pelos sensores da direita).

Xreta = linspace(-1, 1, 2001);
Yreta = zeros(size(Xreta));

desloc = -0.06:0.0005:0.06;          % [m]
leituras = zeros(length(desloc), 8);
entradas = zeros(length(desloc), 3); % [Sdir Scen Sesq]

for i = 1:length(desloc)
    % a barra de sensores é posicionada sobre x = 0
    sensores = bloco_sensor(-0.06, desloc(i), 0, Xreta, Yreta);
    leituras(i,:) = sensores;
    [Sdir, Scen, Sesq] = bloco_conversor(sensores);
    entradas(i,:) = [Sdir Scen Sesq];
end

figs(end+1) = figure;
nomes_figs{end+1} = 'A2_modelo_sensor';
subplot(2,1,1);
plot(1000*desloc, leituras, 'LineWidth', 1.5);
grid on;
ylabel('Leitura');
legend('S1','S2','S3','S4','S5','S6','S7','S8', 'Location', 'eastoutside');
title('Leitura dos sensores do QTR-8 em função do deslocamento lateral');

subplot(2,1,2);
plot(1000*desloc, entradas(:,1), 'r', 'LineWidth', 2); hold on;
plot(1000*desloc, entradas(:,2), 'k', 'LineWidth', 2);
plot(1000*desloc, entradas(:,3), 'b', 'LineWidth', 2);
grid on;
xlabel('Deslocamento lateral do robô [mm] (+ = robô à esquerda da linha)');
ylabel('Entrada do controlador');
legend('S_{dir}', 'S_{cen}', 'S_{esq}', 'Location', 'eastoutside');
title('Saídas do bloco conversor');

%% -----------------------------------------------------
% FIGURA A3: CONJUNTOS DE SAÍDA
% ------------------------------------------------------
% Os conjuntos são representados como triangulares, com vértice no
% centroide adotado em bloco_motor.m; caso outra forma tenha sido definida
% no texto do trabalho, esta representação deve ser ajustada.

limites = [0 40; 30 60; 50 130; 100 165; 140 180];
centros = mean(limites, 2);
nomes_saida = {'giraEsq', 'viraEsq', 'anda', 'viraDir', 'giraDir'};
cores = {'b', 'c', 'k', 'm', 'r'};

figs(end+1) = figure;
nomes_figs{end+1} = 'A3_conjuntos_saida';
hold on;
for j = 1:5
    plot([limites(j,1) centros(j) limites(j,2)], [0 1 0], cores{j}, 'LineWidth', 2);
    plot([centros(j) centros(j)], [0 1], [cores{j} '--']);
    text(centros(j), 1.06, nomes_saida{j}, 'HorizontalAlignment', 'center');
end
grid on;
xlim([0 180]);
ylim([0 1.15]);
xlabel('Saída fuzzy');
ylabel('Grau de pertinência');
title('Conjuntos de saída e centroides utilizados na defuzzificação');

%% -----------------------------------------------------
% FIGURA A4: MAPEAMENTO DA SAÍDA FUZZY PARA AS RODAS
% ------------------------------------------------------
% O trecho a seguir reproduz a lógica de acionamento de bloco_motor.m, a
% qual não pode ser chamada diretamente para um valor arbitrário de saída.
% Portanto, qualquer alteração em bloco_motor.m deve ser repetida aqui.

M_max = 5;   % [rad/s], idêntico a bloco_motor.m

% ponte H: nivelA = 0 -> frente; nivelA = 1 -> trás
satura = @(pwm) min(max(fix(pwm), 0), 255);
ponte  = @(nivelA, pwm) (nivelA == 0)*satura(pwm) - (nivelA == 1)*(255 - satura(pwm));

saida_eixo = 0:0.25:180;
M1_mapa = zeros(size(saida_eixo));
M2_mapa = zeros(size(saida_eixo));

for i = 1:length(saida_eixo)
    s = saida_eixo(i);
    if s <= 45                         % gira p esquerda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(1, s*5 + 30);
    elseif s < 88                      % vira p esquerda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(0, (s-46)*5 + 50);
    elseif s <= 92                     % anda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(0, 255);
    elseif s < 135                     % vira p direita
        vel_dir = ponte(0, (134-s)*5 + 50);
        vel_esq = ponte(0, 255);
    else                               % giro p direita
        vel_dir = ponte(1, (45-(s-135))*5 + 30);
        vel_esq = ponte(0, 255);
    end
    M1_mapa(i) = vel_esq / 255 * M_max;
    M2_mapa(i) = vel_dir / 255 * M_max;
end

figs(end+1) = figure;
nomes_figs{end+1} = 'A4_mapeamento_rodas';
plot(saida_eixo, M1_mapa, 'b', 'LineWidth', 2); hold on;
plot(saida_eixo, M2_mapa, 'r', 'LineWidth', 2);
grid on;
xlim([0 180]);
xlabel('Saída fuzzy');
ylabel('Velocidade da roda [rad/s]');
legend('M1 (esquerda)', 'M2 (direita)', 'Location', 'south');
title('Mapeamento da saída fuzzy para a velocidade das rodas');

%% -----------------------------------------------------
% FIGURA A5: SUPERFÍCIE DE CONTROLE
% ------------------------------------------------------
% A saída fuzzy é calculada em função das entradas laterais para três
% valores fixos da entrada central, correspondentes aos conjuntos alto,
% médio e baixo, evidenciando-se dessa forma a não linearidade do controlador.

eixo = 0:25:1000;
[SD, SE] = meshgrid(eixo, eixo);
Scen_fixo   = [850 350 50];
titulos_cen = {'S_{cen} = 850 (alto)', 'S_{cen} = 350 (médio)', 'S_{cen} = 50 (baixo)'};

figs(end+1) = figure;
nomes_figs{end+1} = 'A5_superficie_controle';
for c = 1:3
    SAIDA = zeros(size(SD));
    for i = 1:numel(SD)
        [PerS1, PerS2, PerS3] = bloco_controlador(SD(i), Scen_fixo(c), SE(i));
        [~, ~, SAIDA(i)] = bloco_motor(PerS1, PerS2, PerS3);
    end
    subplot(1,3,c);
    surf(SD, SE, SAIDA);
    xlabel('S_{dir}');
    ylabel('S_{esq}');
    zlabel('Saída fuzzy');
    zlim([0 180]);
    title(titulos_cen{c});
    view(-35, 30);
end
clear bloco_motor;   % a memória do bloco motor é zerada após a varredura


%% =========================================================
% PARTE B - CENÁRIOS DE TESTE (RESULTADOS)
% =========================================================
% Cada coluna dos vetores abaixo define um cenário. Os nomes foram
% escritos sem acentos para que a tabela impressa permaneça alinhada.
%
% ruido   = desvio-padrão do ruído dos sensores
% desloc  = deslocamento lateral inicial [m]
% angulo  = desvio angular inicial [graus]
% fator   = fator de velocidade das rodas (1 = M_max nominal)

nomes   = {'Nominal', 'Ruido 50', 'Ruido 200', 'Desloc 3 cm', 'Desloc 5 cm', ...
           'Angulo 45 graus', 'Velocidade 4x', 'Velocidade 8x', 'Combinado'};
ruidos  = [0    50   200  0     0     0    0    0    50  ];
deslocs = [0    0    0    0.03  0.05  0    0    0    0.03];
angulos = [0    0    0    0     0     45   0    0    20  ];
fatores = [1    1    1    1     1     1    4    8    3   ];

n_cen = length(nomes);
res = cell(1, n_cen);

for i = 1:n_cen
    cen.nome      = nomes{i};
    cen.ruido     = ruidos(i);
    cen.desloc    = deslocs(i);
    cen.angulo    = angulos(i);
    cen.fator_vel = fatores(i);

    fprintf('Simulando cenário %d de %d: %s\n', i, n_cen, cen.nome);
    clear bloco_motor;   % cada cenário parte da mesma condição inicial
    res{i} = rodar_cenario(cen, Xv, Yv, x0, y0, theta0, Ts);
end

%% -----------------------------------------------------
% TABELA DE MÉTRICAS
% ------------------------------------------------------
fprintf('\n');
fprintf('%-17s | %-9s | %-10s | %-10s | %-10s | %-10s\n', ...
        'Cenario', 'Completou', 'Volta [s]', 'RMS [mm]', 'Max [mm]', 'Fora [s]');
fprintf('%s\n', repmat('-', 1, 82));
for i = 1:n_cen
    r = res{i};
    if r.completou
        status = 'sim';
    elseif r.perdeu
        status = 'perdeu';
    else
        status = 'nao';
    end
    fprintf('%-17s | %-9s | %10.1f | %10.2f | %10.2f | %10.2f\n', ...
            r.nome, status, r.tempo_volta, r.erro_rms, r.erro_max, r.tempo_fora);
end
fprintf('\n');

%% -----------------------------------------------------
% FIGURA B1: TRAJETÓRIAS DE TODOS OS CENÁRIOS
% ------------------------------------------------------
figs(end+1) = figure;
nomes_figs{end+1} = 'B1_trajetorias';
for i = 1:n_cen
    subplot(3,3,i);
    plot(Xv, Yv, 'k', 'LineWidth', 3); hold on;
    plot(res{i}.x, res{i}.y, 'r', 'LineWidth', 1);
    plot(res{i}.x(1), res{i}.y(1), 'go', 'MarkerFaceColor', 'g');
    axis equal;
    grid on;
    title(res{i}.nome);
end

%% -----------------------------------------------------
% FIGURA B2: ERRO LATERAL EM FUNÇÃO DA DISTÂNCIA PERCORRIDA
% ------------------------------------------------------
% A distância percorrida é adotada como eixo horizontal, em lugar do tempo,
% para que cenários com velocidades distintas possam ser comparados
% na mesma posição da pista.

grupos = {[1 2 3], [1 4 5 6], [1 7 8]};
titulos_grupos = {'Efeito do ruído nos sensores', ...
                  'Efeito da condição inicial', ...
                  'Efeito da velocidade'};
cores_grupo = {'k', 'b', 'r', 'm'};

figs(end+1) = figure;
nomes_figs{end+1} = 'B2_erro_lateral';
for g = 1:3
    subplot(3,1,g);
    hold on;
    leg = {};
    for j = 1:length(grupos{g})
        i = grupos{g}(j);
        plot(res{i}.dist, 1000*res{i}.erro, cores_grupo{j}, 'LineWidth', 1.2);
        leg{end+1} = res{i}.nome;
    end
    grid on;
    ylabel('Erro lateral [mm]');
    legend(leg, 'Location', 'eastoutside');
    title(titulos_grupos{g});
end
xlabel('Distância percorrida [m]');

%% -----------------------------------------------------
% FIGURA B3: VARIÁVEIS INTERNAS DE UM CENÁRIO
% ------------------------------------------------------
r = res{cenario_detalhe};

figs(end+1) = figure;
nomes_figs{end+1} = 'B3_variaveis_internas';
subplot(4,1,1);
plot(r.t, 1000*r.erro, 'k');
grid on; ylabel('Erro [mm]');
title(['Variáveis internas - cenário: ' r.nome]);

subplot(4,1,2);
plot(r.t, r.S(1,:), 'r', r.t, r.S(2,:), 'k', r.t, r.S(3,:), 'b');
grid on; ylabel('Leitura');
legend('S_{dir}', 'S_{cen}', 'S_{esq}', 'Location', 'eastoutside');

subplot(4,1,3);
plot(r.t, r.saida, 'm');
grid on; ylabel('Saída fuzzy');
ylim([0 180]);

subplot(4,1,4);
plot(r.t, r.M(1,:), 'b', r.t, r.M(2,:), 'r');
grid on; ylabel('[rad/s]'); xlabel('Tempo [s]');
legend('M1 (esq)', 'M2 (dir)', 'Location', 'eastoutside');

%% -----------------------------------------------------
% FIGURA B4: COMPARAÇÃO DAS MÉTRICAS
% ------------------------------------------------------
erro_rms = zeros(1, n_cen);
erro_max = zeros(1, n_cen);
for i = 1:n_cen
    erro_rms(i) = res{i}.erro_rms;
    erro_max(i) = res{i}.erro_max;
end

figs(end+1) = figure;
nomes_figs{end+1} = 'B4_metricas';
bar([erro_rms; erro_max]');
grid on;
set(gca, 'XTick', 1:n_cen, 'XTickLabel', nomes);
ylabel('Erro lateral [mm]');
legend('RMS', 'Máximo', 'Location', 'northwest');
title('Erro lateral por cenário');

%% 3. SALVAMENTO DAS FIGURAS
if salvar_figuras
    for i = 1:length(figs)
        print(figs(i), [nomes_figs{i} '.png'], '-dpng', '-r200');
    end
    disp('Figuras salvas na pasta atual.');
end
