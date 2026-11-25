function [xr, yr, theta_r, v, omega] = ...
    bloco_dinamica(xr, yr, theta_r, M1, M2, Ts)

% BLOCO DINÂMICA
%
% Entradas:
% xr      = posição atual em X
% yr      = posição atual em Y
% theta_r = orientação atual do robô
% M1      = tensão aplicada ao motor esquerdo [V]
% M2      = tensão aplicada ao motor direito [V]
% Ts      = tempo de amostragem [s]
%
% Saídas:
% xr      = nova posição em X
% yr      = nova posição em Y
% theta_r = nova orientação
% v       = velocidade linear
% omega   = velocidade angular




    omegaE = M1;

    omegaD = M2;


    %% PARÂMETROS GEOMÉTRICOS DO ROBÔ

    raio_roda = 0.02;         % [m]

    distancia_rodas = 0.10;   % [m]


    %% VELOCIDADE LINEAR DO ROBÔ

    v = raio_roda/2 * (omegaE + omegaD);


    %% VELOCIDADE ANGULAR DO ROBÔ

    omega = raio_roda/distancia_rodas * ...
        (omegaD - omegaE);


    %% CINEMÁTICA DO ROBÔ

    xr = xr + v*cos(theta_r)*Ts;

    yr = yr + v*sin(theta_r)*Ts;

    theta_r = theta_r + omega*Ts;


    %% NORMALIZAÇÃO DO ÂNGULO

    theta_r = atan2(sin(theta_r),cos(theta_r));

end