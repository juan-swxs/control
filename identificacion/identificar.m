%% Identificación de G(s) = X(s)/Vef(s) de la planta piñón-cremallera
%  Entrada: Vm [V] en el motor (push-pull), sin la zona muerta:
%       Vef = Vm - u0p  (Vm > 0)      Vef = Vm + u0n  (Vm < 0)
%  Salida:  x [mm] (encoder), relativa a la posición de reposo antes del escalón.
%  Datos:   datos/escalones.csv y datos/estatica.csv (caracterizar.py), T = 20 ms.
%  Modelo:  G(s) = b0/(s^2 + a1 s + a0), un solo modelo para todos los escalones.
clear; clc; close all
here = fileparts(mfilename('fullpath'));
cd(here);
if ~exist('figuras', 'dir'), mkdir figuras; end
Ts = 0.02;

%% 1. Zona muerta desde la escalera estática (voltaje con el que arranca)
E = readtable(fullfile('datos', 'estatica.csv'), 'TextType', 'string');
u0 = struct();
for lado = [1 -1]
    pre = sprintf('est%+d_sube_', lado);
    sel = startsWith(E.prueba, pre);
    niv = unique(double(extractAfter(E.prueba(sel), pre)));
    x0 = NaN; arranque = NaN;
    for V = niv'
        tag = sprintf('%s%.2f', pre, V);
        xs = E.x(E.prueba == tag);
        if isnan(x0), x0 = xs(1); end
        if abs(xs(end) - x0) > 0.5 && isnan(arranque)   % > 0.5 mm: ya se movió
            arranque = V;
        end
    end
    if lado > 0, u0.p = arranque; else, u0.n = arranque; end
end
fprintf('Zona muerta (escalera estática): u0+ = %.2f V, u0- = %.2f V\n', u0.p, u0.n);

%% 2. Escalones
S = readtable(fullfile('datos', 'escalones.csv'), 'TextType', 'string');
tags = unique(S.prueba(startsWith(S.prueba, 'esc')), 'stable');
seg = struct('V', {}, 'u', {}, 'y', {});
for k = 1:numel(tags)
    d = S(S.prueba == tags(k), :);
    on = find(d.modo == 2);
    if numel(on) < 10, continue; end            % los de 6 y 7 V llegan al tope enseguida
    pre = find(d.modo == 0 & (1:height(d))' < on(1));
    idx = [pre; on];
    y = d.x(idx) - mean(d.x(pre));
    u = zeros(size(idx)); u(numel(pre)+1:end) = d.vm(on);
    seg(end+1) = struct('V', d.vm(on(1)), 'u', u, 'y', y); %#ok<SAGROW>
end
fprintf('Escalones usados: %s V\n', mat2str([seg.V]));

    function d = armar(seg, up, un, Ts)
        d = [];
        for i = 1:numel(seg)
            u = seg(i).u;
            uef = u - up*(u > 0) + un*(u < 0);
            di = iddata(seg(i).y, uef, Ts);
            if isempty(d), d = di; else, d = merge(d, di); end
        end
    end

%% 3. Ajuste: busca la zona muerta que mejor explica todos los escalones
opt = tfestOptions('InitialCondition', 'zero', 'EnforceStability', true);
grid = 0:0.1:2.5;
best = struct('fit', -Inf);
for up = grid
    for un = grid
        dat = armar(seg, up, un, Ts);
        m = tfest(dat, 2, 0, opt);
        [~, fitk] = compare(dat, m);
        f = mean(cell2mat(fitk(:)));
        if f > best.fit, best = struct('fit', f, 'up', up, 'un', un, 'm', m); end
    end
end
G = best.m;
[num, den] = tfdata(G, 'v');
den = den / den(1); num = num / den(1);
b0 = num(end); a1 = den(2); a0 = den(3);
fprintf('\nZona muerta del ajuste: u0+ = %.2f V, u0- = %.2f V (fit medio %.1f %%)\n', best.up, best.un, best.fit);

%% 4. Parámetros
K  = b0/a0;  wn = sqrt(a0);  zeta = a1/(2*wn);  p = roots(den);
ts = 4/min(abs(real(p)));
fprintf('\nG(s) = %.2f / (s^2 + %.3f s + %.3f)   [mm/V]\n', b0, a1, a0);
fprintf('K = %.3f mm/V, wn = %.3f rad/s, zeta = %.3f, polos = %s\n', K, wn, zeta, mat2str(p', 4));
fprintf('t_s (2%%) lazo abierto = %.3f s\n', ts);
if zeta > 1
    tau = -1./real(p);
    fprintf('Sobreamortiguado: tau1 = %.4f s, tau2 = %.4f s\n', max(tau), min(tau));
end

% también con la zona muerta medida en la escalera, para comparar
dat_e = armar(seg, u0.p, u0.n, Ts);
Ge = tfest(dat_e, 2, 0, opt);
[~, fe] = compare(dat_e, Ge);
fprintf('\nCon u0 de la escalera: G(s) = %s   (fit %.1f %%)\n', strtrim(evalc('disp(Ge.Numerator)')), mean(cell2mat(fe(:))));
disp(Ge)





%% 5. Figuras
f1 = figure('Position', [100 100 1100 650], 'Color', 'w'); theme(f1, 'light');
n = numel(seg);
for i = 1:n
    subplot(2, ceil(n/2), i);
    u = seg(i).u;
    uef = u - best.up*(u > 0) + best.un*(u < 0);
    t = (0:numel(u)-1)'*Ts;
    ym = lsim(G, uef, t);
    plot(t, seg(i).y, 'k.', 'MarkerSize', 6); hold on
    plot(t, ym, 'Color', [0.45 0.25 0.85], 'LineWidth', 1.6);
    grid on; title(sprintf('V_m = %+g V', seg(i).V));
    xlabel('t [s]'); ylabel('x [mm]');
end
legend('medido', 'modelo', 'Location', 'best');
sgtitle(sprintf('G(s) = %.1f/(s^2+%.2fs+%.2f),  u_0^+ = %.1f V, u_0^- = %.1f V', b0, a1, a0, best.up, best.un));
exportgraphics(f1, fullfile('figuras', 'ajuste_escalones.png'), 'Resolution', 150);

% curva estática x(V)
f2 = figure('Position', [100 100 700 450], 'Color', 'w'); theme(f2, 'light'); hold on; grid on
for lado = [1 -1]
    for fase = ["sube" "baja"]
        pre = sprintf('est%+d_%s_', lado, fase);
        sel = startsWith(E.prueba, pre);
        tg = unique(E.prueba(sel), 'stable');
        V = zeros(numel(tg), 1); X = V;
        for j = 1:numel(tg)
            V(j) = lado*double(extractAfter(tg(j), pre));
            xs = E.x(E.prueba == tg(j)); X(j) = xs(end);
        end
        if fase == "sube", st = '-o'; else, st = '--s'; end
        plot(V, X, st, 'MarkerSize', 4);
    end
end
Vg = linspace(-7.5, 7.5, 301);
Vef = Vg - best.up*(Vg > best.up) + best.un*(Vg < -best.un);
Vef(abs(Vg) <= 0 | (Vg > 0 & Vg <= best.up) | (Vg < 0 & Vg >= -best.un)) = 0;
plot(Vg, K*Vef, 'k', 'LineWidth', 1.5);
xlabel('V_m [V]'); ylabel('x de equilibrio [mm]');
legend('+ sube', '+ baja', '- sube', '- baja', 'modelo K·V_{ef}', 'Location', 'northwest');
title('Curva estática con histéresis (fricción)');
exportgraphics(f2, fullfile('figuras', 'curva_estatica.png'), 'Resolution', 150);

%% 6. Guardar
res = struct('b0', b0, 'a1', a1, 'a0', a0, 'K', K, 'wn', wn, 'zeta', zeta, 'polos_re', real(p'), 'polos_im', imag(p'), ...
             'ts', ts, 'u0p', best.up, 'u0n', best.un, 'u0p_escalera', u0.p, 'u0n_escalera', u0.n, ...
             'fit', best.fit, 'Ts', Ts);
save('modelo.mat', 'G', 'res');
fid = fopen('modelo.json', 'w'); fprintf(fid, '%s', jsonencode(res, 'PrettyPrint', true)); fclose(fid);
fprintf('\nGuardado: modelo.mat, modelo.json, figuras/*.png\n');
