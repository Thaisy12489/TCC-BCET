clear;
clc;
close all;
clear bloco_motor;   % zera a memória (persistent) da saída fuzzy

%% =========================================================
% BLOCO SIMULAÇÃO
% =========================================================
% Cadeia: sensores (QTR-8 analógico) -> conversor -> controlador
%         (fuzzificação) -> motor (inferência + defuzzificação) -> dinâmica

%% 1. CARREGA OS DADOS DA TRAJETÓRIA
[Xv, Yv, xr, yr, theta_r] = trajetoria();   % trajetoria() abre uma figura
fig_anim = gcf;                             % reaproveita essa mesma janela
clf(fig_anim);                              % limpa o desenho estático

%% 2. TEMPO DE AMOSTRAGEM
Ts = 0.05;

%% 3. INICIALIZAÇÃO
fim_pista = false;
k = 0;
max_iteracoes = 10000; % evitar loop infinito caso o robô se perca

passo_animacao = 5;    % redesenha a cada N iterações (1 = mais lento e suave)

% Registro para análise posterior
hist_saida = zeros(1, max_iteracoes);
hist_M     = zeros(2, max_iteracoes);
hist_S     = zeros(3, max_iteracoes);   % [Sdir; Scen; Sesq]

% Registro para as métricas de desempenho
avanco_sensor = 0.06;                   % distância da barra ao eixo [m]
limite_fora   = 0.04875;                % erro a partir do qual nenhum sensor vê a linha [m]
dist_percorrida = 0;
comprimento_pista = sum(sqrt(diff(Xv).^2 + diff(Yv).^2));
hist_erro = zeros(1, max_iteracoes);    % erro lateral verdadeiro, com sinal [m]

%% 4. PREPARAÇÃO DA ANIMAÇÃO
figure(fig_anim);
plot(Xv,Yv,'k','LineWidth',4);
hold on;
grid on;
axis equal;
xlim([min(Xv)-0.5, max(Xv)+0.5]);
ylim([min(Yv)-0.5, max(Yv)+0.5]);
xlabel('X [m]');
ylabel('Y [m]');
title('Simulação do Robô Seguidor de Linha');

% Trajetória percorrida pelo robô
traj_x = xr;
traj_y = yr;
traj_robo = plot(traj_x, traj_y, 'r', 'LineWidth', 2);

% Robô
robo = plot(xr,yr,'bs','MarkerSize',10,'MarkerFaceColor','b');

% Orientação do robô
seta = quiver(xr,yr,0.3*cos(theta_r),0.3*sin(theta_r),0,'k','LineWidth',1.5);

drawnow;

%% 5. INÍCIO DO LOOP
while ~fim_pista && k < max_iteracoes
    k = k + 1;

    %% INTERFACE FÍSICA: LEITURA ANALÓGICA DO QTR-8 (S1...S8)
    sensores = bloco_sensor(xr, yr, theta_r, Xv, Yv);

    %% BLOCO CONVERSOR (8 sensores -> direita, centro, esquerda)
    [Sdir, Scen, Sesq] = bloco_conversor(sensores);

    %% BLOCO CONTROLADOR (Fuzzificação)
    [PerS1, PerS2, PerS3] = bloco_controlador(Sdir, Scen, Sesq);

    %% BLOCO MOTOR (Inferência + Defuzzificação)
    [M1, M2, saida] = bloco_motor(PerS1, PerS2, PerS3);

    %% BLOCO DINÂMICA
    [xr, yr, theta_r, v, omega] = bloco_dinamica(xr, yr, theta_r, M1, M2, Ts);

    % registro
    hist_saida(k) = saida;
    hist_M(:,k)   = [M1; M2];
    hist_S(:,k)   = [Sdir; Scen; Sesq];

    % registro das métricas (v só existe aqui, depois do bloco_dinamica)
    dist_percorrida = dist_percorrida + abs(v)*Ts;
    hist_erro(k) = erro_lateral(xr + avanco_sensor*cos(theta_r), ...
                                yr + avanco_sensor*sin(theta_r), Xv, Yv);

    %% ATUALIZAÇÃO DA ANIMAÇÃO
    traj_x(end+1) = xr;
    traj_y(end+1) = yr;

    if mod(k, passo_animacao) == 0
        set(traj_robo, 'XData', traj_x, 'YData', traj_y);
        set(robo, 'XData', xr, 'YData', yr);
        set(seta, 'XData', xr, 'YData', yr, ...
                  'UData', 0.3*cos(theta_r), 'VData', 0.3*sin(theta_r));
        drawnow;
        pause(0.001);   % necessário no Octave para a janela atualizar
    end

    % PARADA: percorreu quase toda a pista e voltou perto do ponto inicial
    if dist_percorrida > 0.9*comprimento_pista && ...
       norm([xr, yr] - [Xv(1), Yv(1)]) < 0.15
        fim_pista = true;
    end
end

% desenho final completo
set(traj_robo, 'XData', traj_x, 'YData', traj_y);
set(robo, 'XData', xr, 'YData', yr);
set(seta, 'XData', xr, 'YData', yr, ...
          'UData', 0.3*cos(theta_r), 'VData', 0.3*sin(theta_r));
drawnow;

if fim_pista
    disp('Volta completada com sucesso!');
else
    disp('A simulação atingiu o limite de tempo (o robô pode ter se perdido).');
end
fprintf('Iterações: %d  |  Tempo simulado: %.1f s\n', k, k*Ts);

%% MÉTRICAS DE DESEMPENHO
e = hist_erro(1:k);   T = k*Ts;
e_max = 1000*max(abs(e));
e_rms = 1000*sqrt(mean(e.^2));
IAE   = 1000*sum(abs(e))*Ts;                 % [mm.s]

% perdas da linha: contam-se os eventos (subidas), não as amostras
fora = abs(e) > limite_fora;
n_perdas = sum(diff([0 fora]) == 1);
t_fora   = Ts*sum(fora);

% oscilação, medida depois do transitório inicial (erro < 10 mm)
i0 = find(abs(e) < 0.010, 1);
if isempty(i0), i0 = 1; end
es = e(i0:end);
h = 0.003;                                   % histerese [m], maior que o viés de regime
estado = 0;  cruz = 0;
for j = 1:numel(es)
    if es(j) > h
        cruz = cruz + (estado == -1);
        estado = 1;
    elseif es(j) < -h
        cruz = cruz + (estado == 1);
        estado = -1;
    end
end
dist_ss    = dist_percorrida*numel(es)/k;
dM         = hist_M(2,1:k) - hist_M(1,1:k);
cruz_por_m = cruz/dist_ss;
TV         = sum(abs(diff(dM(i0:end))))/dist_ss;   % [rad/s por metro]

fprintf('Volta: %.1f s | vmed %.3f m/s | Max %.1f mm | RMS %.2f mm | IAE %.0f mm.s\n', ...
        T, dist_percorrida/T, e_max, e_rms, IAE);
fprintf('Perdas: %d (%.2f s) | Cruz./m: %.2f | TV: %.1f rad/s por m\n', ...
        n_perdas, t_fora, cruz_por_m, TV);

%% 6. GRÁFICOS DE ANÁLISE (janela separada, aberta só no final)
t = (1:k)*Ts;

figure('Name', 'Análise do controlador');
subplot(3,1,1);
plot(t, hist_S(1,1:k), 'r', t, hist_S(2,1:k), 'k', t, hist_S(3,1:k), 'b');
grid on; ylabel('Leitura');
legend('S_{dir}', 'S_{cen}', 'S_{esq}');
title('Entradas do controlador');

subplot(3,1,2);
plot(t, hist_saida(1:k), 'm');
grid on; ylabel('Saída fuzzy');
ylim([0 180]);

subplot(3,1,3);
plot(t, hist_M(1,1:k), 'b', t, hist_M(2,1:k), 'r');
grid on; ylabel('[rad/s]'); xlabel('Tempo [s]');
legend('M1 (esq)', 'M2 (dir)');

figure(fig_anim);   % traz a janela da pista de volta para a frente