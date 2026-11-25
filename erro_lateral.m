function e = erro_lateral(px, py, Xv, Yv)

% Distância com sinal do ponto (px, py) à pista, sendo o sinal positivo
% quando o ponto se encontra à esquerda do sentido de percurso da pista.

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

    d = sqrt((px - qx).^2 + (py - qy).^2);
    [dmin, idx] = min(d);

    % produto vetorial entre a direção do segmento e o vetor até o ponto
    lado = abx(idx)*(py - qy(idx)) - aby(idx)*(px - qx(idx));

    e = sign(lado) * dmin;

end