function [Xv, Yv, xr, yr, theta_r] = trajetoria()

% BLOCO TRAJETÓRIA 
% Saídas:
% Xv, Yv  = pontos da trajetória de referência
% xr, yr  = posição inicial do robô
% theta_r = orientação inicial do robô


N = 200; 
R = 1.0;          
dist_centros = 4; 


xr = dist_centros / 2; % Centralizado em X
yr = -0.6;             % Posição em Y no ponto mais baixo do estreitamento
theta_r = 0;           % Apontando para a direita (0 rad)


x1 = linspace(dist_centros/2, dist_centros, N);
s = linspace(0, 1, N);

y1 = -0.6 + (-R - (-0.6)) * (3*s.^2 - 2*s.^3);


ang2 = linspace(-pi/2, pi/2, N);
x2 = dist_centros + R * cos(ang2);
y2 = R * sin(ang2);


x3 = linspace(dist_centros, dist_centros/2, N);
s = linspace(0, 1, N);
y3 = R + (0.6 - R) * (3*s.^2 - 2*s.^3);


x4 = linspace(dist_centros/2, 0, N);
s = linspace(0, 1, N);
y4 = 0.6 + (R - 0.6) * (3*s.^2 - 2*s.^3);


ang5 = linspace(pi/2, 3*pi/2, N);
x5 = 0 + R * cos(ang5);
y5 = R * sin(ang5);

x6 = linspace(0, dist_centros/2, N);
s = linspace(0, 1, N);
y6 = -R + (-0.6 - (-R)) * (3*s.^2 - 2*s.^3);


Xv = [x1 x2 x3 x4 x5 x6];
Yv = [y1 y2 y3 y4 y5 y6];

Xv(end+1) = Xv(1);
Yv(end+1) = Yv(1);


figure;
plot(Xv, Yv, 'k', 'LineWidth', 4);
hold on;


plot([dist_centros/2 dist_centros/2], [-0.75 -0.45], 'g', 'LineWidth', 5);
text(dist_centros/2 - 0.4, -0.35, 'INÍCIO / FIM', 'FontWeight', 'bold', 'FontSize', 10);
plot(dist_centros/2, -0.6, 'go', 'MarkerSize', 14, 'LineWidth', 2);


tamanho_robo = 0.14;
rectangle('Position', [xr-tamanho_robo/2, yr-tamanho_robo/2, tamanho_robo, tamanho_robo], ...
          'FaceColor', 'b', 'EdgeColor', 'k', 'LineWidth', 1.5);

comprimento_seta = 0.35;
quiver(xr, yr, comprimento_seta*cos(theta_r), comprimento_seta*sin(theta_r), 0, ...
       'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);


grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória da Pista');
xlim([-2 (dist_centros + 2)]);
ylim([-(R + 0.5) (R + 0.5)]);

end
