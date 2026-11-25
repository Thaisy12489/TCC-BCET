function [PerS1, PerS2, PerS3] = bloco_controlador(S1, S23, S4)
% parte da fuzzificacao
%
% Entradas: S1 (direita), S23 (centro), S4 (esquerda)
% Saídas:   vetores [alto medio baixo] 

   
    P = [50 100 250 300 400 500 600 700];

    PerS1 = pertinencia_sensor(S1,  P);   % direita
    PerS2 = pertinencia_sensor(S23, P);   % centro
    PerS3 = pertinencia_sensor(S4,  P);   % esquerda
end

function Per = pertinencia_sensor(valor, P)
    p1=P(1); p2=P(2); p3=P(3); p4=P(4); p5=P(5); p6=P(6); p7=P(7); p8=P(8);

    % conjunto baixo
    if valor < p1
        baixo = 1.0;
    elseif valor < p3
        baixo = (p3 - valor) / (p3 - p1);
    else
        baixo = 0.0;
    end

    % conjunto médio
    if valor < p2
        medio = 0.0;
    elseif valor < p4
        medio = (valor - p2) / (p4 - p2);
    elseif valor < p5
        medio = 1.0;
    elseif valor < p7
        medio = (p7 - valor) / (p7 - p5);
    else
        medio = 0.0;
    end

    % conjunto alto
    if valor < p6
        alto = 0.0;
    elseif valor < p8
        alto = (valor - p6) / (p8 - p6);
    else
        alto = 1.0;
    end

    Per = [alto medio baixo];
end