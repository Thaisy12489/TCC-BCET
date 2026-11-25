function [M1, M2, saida] = bloco_motor(PerS1, PerS2, PerS3)

% parte da defuzzificacao
%
% Entradas: PerS1, PerS2, PerS3 = [alto medio baixo] (direita, centro, esquerda)
% Saídas:   M1 = velocidade da roda ESQUERDA [rad/s]  
%           M2 = velocidade da roda DIREITA  [rad/s]
%           saida = saída fuzzy (0 a 180)

    persistent saida_mem
    if isempty(saida_mem)
        saida_mem = 90;   
    end

    M_max = 5;   % velocidade da roda com PWM = 255 [rad/s]

    %% ---------------- BASE DE REGRAS ----------------
    % Saídas: 1 = giraEsq | 2 = viraEsq | 3 = anda | 4 = viraDir | 5 = giraDir
    % Ordem das regras R1...R27: (dir, centro, esq), cada um em [alto medio baixo]
    CONS = [4 5 5   2 4 5   2 4 5 ...    % R1  - R9   (dir = alto)
            1 3 4   2 4 5   2 4 5 ...    % R10 - R18  (dir = médio)
            1 2 3   1 1 3   1 1 2];      % R19 - R27  (dir = baixo)

    PerSa = zeros(1,5);
    r = 0;
    for i = 1:3          % direita
        for j = 1:3      % centro
            for l = 1:3  % esquerda
                r = r + 1;
                ativacao = min([PerS1(i), PerS2(j), PerS3(l)]);
                PerSa(CONS(r)) = max(PerSa(CONS(r)), ativacao);
            end
        end
    end

    %% DEFUZZIFICAÇÃO 
    centros = [(0+40)/2, (30+60)/2, (50+130)/2, (100+165)/2, (140+180)/2];
    den = sum(PerSa);
    if den > 0
        saida_mem = sum(centros .* PerSa) / den;
    end
    saida = saida_mem;

    
    if saida <= 45                         % gira p esquerda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(1, saida*5 + 30);
    elseif saida < 88                      % vira p esquerda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(0, (saida-46)*5 + 50);
    elseif saida <= 92                     % anda
        vel_dir = ponte(0, 255);
        vel_esq = ponte(0, 255);
    elseif saida < 135                     % vira p direita
        vel_dir = ponte(0, (134-saida)*5 + 50);
        vel_esq = ponte(0, 255);
    else                                   % giro p direita
        vel_dir = ponte(1, (45-(saida-135))*5 + 30);
        vel_esq = ponte(0, 255);
    end

    M1 = vel_esq / 255 * M_max;
    M2 = vel_dir / 255 * M_max;
end

function vel = ponte(nivelA, pwm)
    % nivelA = 0 (LOW)  -> frente, velocidade = pwm
    % nivelA = 1 (HIGH) -> trás,   velocidade = -(255 - pwm)
    p = min(max(fix(pwm), 0), 255);
    if nivelA == 0
        vel = p;
    else
        vel = -(255 - p);
    end
end