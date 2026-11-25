function sensores = bloco_sensor(xr, yr, theta_r, Xv, Yv, ruido)
% INTERFACE FÍSICA - SENSOR QTR-8 (MODO ANALÓGICO)
%
% A leitura analógica dos 8 sensores do QTR-8 é simulada a partir da
% posição real do robô em relação à pista, sendo cada sensor posicionado
% individualmente ao longo da barra, a qual se encontra à frente do eixo
% das rodas, como no robô real.
%
% Entradas:
% xr, yr  = posição atual do robô [m]
% theta_r = orientação atual do robô [rad]
% Xv, Yv  = pontos da trajetória de referência
% ruido   = desvio-padrão do ruído aditivo das leituras (opcional, padrão 0)
%
% Saída:
% sensores = [S1 ... S8], com valores de 0 (fundo branco) a 1000 (linha preta)
%
%   QTR-8:   S1  S2  S3 | S4  S5 | S6  S7  S8
%            esquerda   | centro | direita
%
% Exemplo (linha centralizada): [0 0 167 800 800 167 0 0]


    if nargin < 6
        ruido = 0;
    end


    %% PARÂMETROS FÍSICOS DA BARRA DE SENSORES

    avanco_sensor = 0.06;      % distância da barra ao eixo das rodas [m]

    espacamento = 0.0095;      % distância entre sensores do QTR-8 (9,525 mm) [m]

    largura_linha = 0.019;     % largura da fita isolante [m]

    transicao = 0.006;         % largura da região de borda da linha [m]

    valor_linha = 800;         % leitura sobre a linha preta

    valor_fundo = 0;           % leitura sobre o fundo branco


    %% POSIÇÃO DE CADA SENSOR NO PLANO

    % O deslocamento lateral é considerado positivo para a esquerda do
    % sentido de movimento, de modo que S1 ocupa a extremidade esquerda
    % da barra e S8 a extremidade direita.
    lateral = (4.5 - (1:8)) * espacamento;

    % centro da barra de sensores
    xb = xr + avanco_sensor*cos(theta_r);
    yb = yr + avanco_sensor*sin(theta_r);

    % posição de cada sensor ao longo da direção perpendicular ao movimento
    xs = xb - lateral*sin(theta_r);
    ys = yb + lateral*cos(theta_r);


    %% LEITURA DE CADA SENSOR

    sensores = zeros(1,8);

    meia_largura = largura_linha/2;

    for i = 1:8

        d = distancia_pista(xs(i), ys(i), Xv, Yv);

        % A leitura é máxima sobre a linha, decresce linearmente na região
        % de borda e torna-se nula sobre o fundo branco.
        if d <= meia_largura
            frac = 1;
        elseif d <= meia_largura + transicao
            frac = 1 - (d - meia_largura)/transicao;
        else
            frac = 0;
        end

        leitura = valor_fundo + frac*(valor_linha - valor_fundo) + ruido*randn;

        sensores(i) = min(max(round(leitura), 0), 1000);
    end

end


function d = distancia_pista(px, py, Xv, Yv)

% Menor distância do ponto (px, py) à pista, sendo esta tratada como uma
% sequência de segmentos de reta entre os pontos (Xv, Yv).

    ax = Xv(1:end-1);   ay = Yv(1:end-1);
    bx = Xv(2:end);     by = Yv(2:end);

    abx = bx - ax;
    aby = by - ay;

    L2 = abx.^2 + aby.^2;
    L2(L2 == 0) = eps;

    t = ((px - ax).*abx + (py - ay).*aby) ./ L2;
    t = min(max(t, 0), 1);

    qx = ax + t.*abx;
    qy = ay + t.*aby;

    d = min(sqrt((px - qx).^2 + (py - qy).^2));

end
