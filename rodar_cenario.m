function res = rodar_cenario(cen, Xv, Yv, x0, y0, theta0, Ts)
% BLOCO CENÁRIO
%
% Uma volta completa do robô seguidor de linha é simulada, sem animação,
% para um cenário de teste previamente definido, sendo registrados os
% históricos das variáveis do controlador e as métricas de desempenho.
%
% Entradas:
% cen     = estrutura com os parâmetros do cenário
%           cen.nome      = nome do cenário
%           cen.ruido     = desvio-padrão do ruído dos sensores
%           cen.desloc    = deslocamento lateral inicial [m] (+ = esquerda)
%           cen.angulo    = desvio angular inicial [graus] (+ = anti-horário)
%           cen.fator_vel = fator multiplicativo da velocidade das rodas
% Xv, Yv  = pontos da trajetória de referência
% x0, y0  = posição inicial nominal do robô [m]
% theta0  = orientação inicial nominal do robô [rad]
% Ts      = tempo de amostragem [s]
%
% Saída:
% res     = estrutura com os históricos e as métricas de desempenho
%
% Observação: o fator de velocidade multiplica M1 e M2 na saída do bloco
% motor, o que equivale a alterar M_max em bloco_motor.m sem modificá-lo.


    %% PARÂMETROS DA SIMULAÇÃO

    avanco_sensor = 0.06;      % deve coincidir com simular_leitura_sensores.m [m]

    max_iteracoes = 10000;     % limite para o caso de o robô não completar a volta

    limite_perda = 0.30;       % afastamento a partir do qual o robô é dado como perdido [m]

    % Considera-se que o robô está fora da linha quando nenhum sensor a
    % detecta, ou seja, quando o afastamento da barra excede a soma da meia
    % largura da linha, da meia largura da barra e da região de borda.
    limite_fora = 0.0095 + 3.5*0.0095 + 0.006;     % [m]

    comprimento_pista = sum(sqrt(diff(Xv).^2 + diff(Yv).^2));


    %% SEMENTE DO GERADOR ALEATÓRIO

    % A semente é fixada para que os resultados com ruído sejam reprodutíveis.
    randn('state', 1);


    %% CONDIÇÃO INICIAL DO CENÁRIO

    xr = x0 - cen.desloc*sin(theta0);
    yr = y0 + cen.desloc*cos(theta0);
    theta_r = theta0 + cen.angulo*pi/180;


    %% PRÉ-ALOCAÇÃO DOS HISTÓRICOS

    hist_x     = zeros(1, max_iteracoes);
    hist_y     = zeros(1, max_iteracoes);
    hist_erro  = zeros(1, max_iteracoes);
    hist_dist  = zeros(1, max_iteracoes);
    hist_saida = zeros(1, max_iteracoes);
    hist_M     = zeros(2, max_iteracoes);
    hist_S     = zeros(3, max_iteracoes);     % [Sdir; Scen; Sesq]


    %% LOOP DE SIMULAÇÃO

    k = 0;
    dist_percorrida = 0;
    completou = false;
    perdeu = false;

    while ~completou && k < max_iteracoes
        k = k + 1;

        % interface física
        sensores = bloco_sensor(xr, yr, theta_r, Xv, Yv, cen.ruido);

        % bloco conversor
        [Sdir, Scen, Sesq] = bloco_conversor(sensores);

        % bloco controlador (fuzzificação)
        [PerS1, PerS2, PerS3] = bloco_controlador(Sdir, Scen, Sesq);

        % bloco motor (inferência + defuzzificação)
        [M1, M2, saida] = bloco_motor(PerS1, PerS2, PerS3);

        M1 = cen.fator_vel * M1;
        M2 = cen.fator_vel * M2;

        % bloco dinâmica
        [xr, yr, theta_r, v, omega] = bloco_dinamica(xr, yr, theta_r, M1, M2, Ts);

        dist_percorrida = dist_percorrida + abs(v)*Ts;

        % O erro lateral é medido na barra de sensores, uma vez que é esta
        % a grandeza efetivamente observada e regulada pelo controlador.
        xs = xr + avanco_sensor*cos(theta_r);
        ys = yr + avanco_sensor*sin(theta_r);
        erro = erro_lateral(xs, ys, Xv, Yv);

        % registro
        hist_x(k)     = xr;
        hist_y(k)     = yr;
        hist_erro(k)  = erro;
        hist_dist(k)  = dist_percorrida;
        hist_saida(k) = saida;
        hist_M(:,k)   = [M1; M2];
        hist_S(:,k)   = [Sdir; Scen; Sesq];

        % robô perdido
        if abs(erro) > limite_perda
            perdeu = true;
            break;
        end

        % volta completa
        if dist_percorrida > 0.9*comprimento_pista && ...
           norm([xr, yr] - [Xv(1), Yv(1)]) < 0.15
            completou = true;
        end
    end


    %% RESULTADOS

    res.nome       = cen.nome;
    res.t          = (1:k)*Ts;
    res.x          = hist_x(1:k);
    res.y          = hist_y(1:k);
    res.erro       = hist_erro(1:k);
    res.dist       = hist_dist(1:k);
    res.saida      = hist_saida(1:k);
    res.M          = hist_M(:,1:k);
    res.S          = hist_S(:,1:k);

    res.completou  = completou;
    res.perdeu     = perdeu;

    if completou
        res.tempo_volta = k*Ts;
    else
        res.tempo_volta = NaN;
    end

    res.erro_rms   = 1000*sqrt(mean(res.erro.^2));      % [mm]
    res.erro_max   = 1000*max(abs(res.erro));           % [mm]
    res.tempo_fora = Ts*sum(abs(res.erro) > limite_fora);  % [s]

end



