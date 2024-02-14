function [delta_Fz_F,delta_Fz_R] = NonRigidLaterLoadWeightTransfer( h_uF,...
   h_uR, h_sF, h_sR, k_C, k_F, k_R, l, a_s, b_s, m_uF, m_uR, t_F, t_R,...
   z_F, z_R, LatAcc) 
%% Non-Rigid Lateral Load Weight Transfer - 
% Computes tire normal loads using Non-rigid chassis model by Sampo
% 
% Inputs:
%   LatAcc     - (n,1 numeric) Total Lateral Acceleration         {a_x}        [m/s^2]
%
% Outputs:
%   delta_Fz_F - (n,1 numeric) Front Lateral Load Weight Transfer {DeltaF_zF}} [N]
%   delta_Fz_R - (n,1 numeric) Rear Lateral Load Weight Transfer  {DeltaF_zR}} [N]
%
% Notes:
%
% Author(s): 
% Tristan Pham (atlpham@ucdavis.edu) [Sep 2020 - Present] 
% 
% Last Updated: 31-August-2023

%% Test Case
if nargin == 0
    m = 290;
    m_s = 225;
    m_uF = 28;
    m_uR = 37;
    h_G = 0.295;
    a = 0.843;
    b = 0.712;
    a_s = 0.830;
    b_s = 0.725;
    l = 1.555;
    t = 1.210;
    t_F = t;
    t_R = t;
    h_sF = 0.295;
    h_sR = 0.315;
    h_uF = 0.255;
    h_uR = 0.255;
    z_F = 0.025;
    z_R = 0.045;
    k_F = 205;
    k_R = 175;
    k_C = 380;
    LatAcc = linspace(0,2,20);
    z_uF = h_uF;
    m_sF = m_s*b_s/l;
    m_sR = m_s*a_s/l;
    d_sF = h_sF - z_F;
    d_sR = h_sR - z_R;
    h_s = (h_sF+h_sR)/2;
    z_s = (z_F + z_R) /2;
    d_s = h_s - z_s;
    
    lambda = linspace(0,1,100);
    mu = [0.1,0.2,0.5,1,2,5,10];
    figure(1)
    for i = 1:7
        X = ((lambda.^2 - (mu(i) + 1).* lambda)./(lambda.^2 - lambda - mu(i))) .* (d_sF./h_G) .*...
            (m_sF ./ m) - (mu(i) .* lambda) ./ (lambda.^2 - lambda - mu(i)) .* (d_sR ./ h_G).*(m_sR ./ m) + ...
            (z_F ./ h_G) .* (m_sF ./ m) + (z_uF ./ h_G) .* (m_uF ./ m);
        hold on
        plot(lambda,X)
    end
    legend('$\mu$ = 0.1','$\mu$ = 0.2','$\mu$ = 0.5','$\mu$ = 1','$\mu$ = 2','$\mu$ = 5','$\mu$ = 10','interpreter','latex','Location','Best')
    lambda = [0.2,0.3,0.4,0.5,0.6,0.7,0.8];
    mu = linspace(0.1,10,100);
    title("Lateral Load Transfer Proportion vs Roll Stiffness Distribution" + ...
        " at Normalized Chassis Torsional Stiffness.",'FontSize',14)
    xlabel("Front Roll Stiffness Percentage",'FontSize',14)
    ylabel("Percent Front Lateral Load Transfer",'FontSize',14)
figure(2)
set(gca,'XScale', 'log')
    for i = 1:7
        X = ((lambda(i).^2 - (mu + 1).* lambda(i))./(lambda(i).^2-lambda(i)-mu)) .* (d_sF./h_G) .*...
            (m_sF./m) - (mu.*lambda(i))./(lambda(i).^2-lambda(i)-mu) .* (d_sR./h_G).*(m_sR./m) + ...
            (z_F./h_G).*(m_sF./m) + (z_uF./h_G).*(m_uF./m);
        hold on
        plot(mu,X)
    end
legend('$\lambda$ = 0.2','$\lambda$ = 0.3','$\lambda$ = 0.4','$\lambda$ = 0.5',...
    '$\lambda$ = 0.6','$\lambda$ = 0.7','$\lambda$ = 0.8','interpreter','latex','Location','Best')
title("Lateral load transfer proportion vs normalised chassis torsional stiffness" + ...
        " for different values of roll stiffness distribution.")
    xlabel("$\mu$",'Interpreter','latex','FontSize',14)
    ylabel("$X$",'Interpreter','latex','FontSize',14)

lambda = linspace(0,1,100);
mu = linspace(0.1,10,1000);
[X_Test,Y_Test] = meshgrid(mu,lambda);
X = ((Y_Test.^2 - (X_Test + 1).* Y_Test)./(Y_Test.^2-Y_Test-X_Test)) .* (d_sF./h_G) .*...
            (m_sF./m) - (X_Test.*Y_Test)./(Y_Test.^2-Y_Test-X_Test) .* (d_sR./h_G).*(m_sR./m) + ...
            (z_F./h_G).*(m_sF./m) + (z_uF./h_G).*(m_uF./m);
figure(7)
surf(X_Test,Y_Test,X)
set(gca,'XScale', 'log')

lambda = linspace(0,1,100);
mu = linspace(0.1,10,100);
[XValues,YValues]=meshgrid(mu,lambda);
X = ((YValues.^2 - (XValues + 1).* YValues)./(YValues.^2-YValues-XValues)) .* (d_sF./h_G) .*...
            (m_sF./m) - (XValues.*YValues)./(YValues.^2-YValues-XValues) .* (d_sR./h_G).*(m_sR./m) + ...
            (z_F./h_G).*(m_sF./m) + (z_uF./h_G).*(m_uF./m);
X_0 = YValues .* (d_s./(h_G)) .* (m_s./m) + (z_F./h_G).*(m_sF./m)+(h_uF./h_G).*(m_uF./m);
ZValues = abs(X-X_0)./X_0;
figure(3)
contourf(XValues,YValues,ZValues,30)
colormap("turbo")
set(gca,'XScale', 'log')
colorbar
figure(4)
surf(XValues,YValues,ZValues)
set(gca,'XScale', 'log')
colormap("turbo")

syms x y
X = ((y.^2 - (x + 1).* y)./(y.^2-y-x)) .* (d_sF./h_G) .*...
            (m_sF./m) - (x.*y)./(y.^2-y-x) .* (d_sR./h_G).*(m_sR./m) + ...
            (z_F./h_G).*(m_sF./m) + (z_uF./h_G).*(m_uF./m);
X_0 = y .* (d_s./(h_G)) .* (m_s./m) + (z_F./h_G).*(m_sF./m)+(h_uF./h_G).*(m_uF./m);

lambda = linspace(0.00001,0.99999,100);
mu = linspace(0.1,10,100);
[XValues,YValues]=meshgrid(mu,lambda);

X_Prime = diff(X,y);

X_0_Prime = diff(X_0,y);

X_Final_Prime = abs(X_Prime-X_0_Prime)./X_0_Prime;

X_FP_Values = subs(X_Final_Prime,{x, y},{XValues, YValues});
figure(5)
contourf(XValues,YValues,X_FP_Values,50)
clim([0 2])
colormap("turbo")
colorbar
set(gca,'XScale', 'log')
figure(6)
X_FP_Values = double(X_FP_Values);
surf(XValues,YValues,X_FP_Values)
set(gca,'XScale', 'log')
colormap("turbo")

[delta_Fz_F,delta_Fz_R] = NonRigidLaterLoadWeightTransfer( h_uF,...
        h_uR, h_sF, h_sR, k_C, k_F, k_R, l, a_s, b_s, m_uF, m_uR, t_F, t_R,...
        z_F, z_R, LatAcc);
    return
end

%% Computation
m_sF = b_s./l;
m_sR = a_s./l;
d_sF = h_sF - z_F;
d_sR = h_sR - z_R;

delta_Fz_F = (k_F .* d_sF .* m_sF) ./ ( k_F + (k_R .* k_C ./ (k_R + k_C)));
delta_Fz_F = delta_Fz_F + ( ((k_F .* k_C ./ (k_F + k_C)) .* d_sR .* m_sR) ./ ...
    ( (k_F .* k_C ./ (k_F + k_C)) + k_R ) );
delta_Fz_F = delta_Fz_F + z_F .* m_sF + h_uF .* m_uF;
delta_Fz_F = delta_Fz_F .* (LatAcc./t_F);

delta_Fz_R = (k_R .* d_sR .* m_sR) ./ ( k_R + (k_F .* k_C ./ (k_F + k_C)));
delta_Fz_R = delta_Fz_R + ( ((k_R .* k_C ./ (k_R + k_C)) .* d_sF .* m_sF) ./ ...
    ( (k_R .* k_C ./ (k_R + k_C)) + k_F ) );
delta_Fz_R = delta_Fz_R + z_R .* m_sR + h_uR.*m_uR;
delta_Fz_R = delta_Fz_R .* (LatAcc./t_R);


end
