function [Sdir, Scen, Sesq] = bloco_conversor(sensores)

%
% Converte os 8 sensores do QTR-8 nas 3 entradas do controlador
% 
%
% Entrada:
%   sensores = [S1 ... S8] 
%                S1 S2 S3  S4  S5  S6  S7 S8
%                [0 0  0  800  800  0  0  0] 
% 
%
%   QTR-8:   S1  S2  S3 | S4  S5 | S6  S7  S8
%            esquerda   | centro | direita
%
% Saídas :
%   Sdir = média de S6, S7 e S8                
%   Scen = média de S4 e S5, mais offset de 50 
%   Sesq = média de S1, S2 e S3                

    Sdir = floor(mean(sensores(6:8)));
    Scen = floor((sensores(4) + sensores(5)) / 2) + 50;
    Sesq = floor(mean(sensores(1:3)));
end