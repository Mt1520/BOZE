clear 
clc
close all

%%-------------------------------- Paramaters -------------------------
AR = 5.9; % aspect ratio picked from comparison to do 355
e = 0.8;
K_ld = 9; %taken from raymer
Swet_Sref = 4.3; %taken from raymer, estimate between cessna skylane rg and beech duchess
L_D_max = K_ld*sqrt(AR/(Swet_Sref));
Cd_0 = 0.25*pi*AR*e/(L_D_max)^2;
eta_p = 0.8;
V = convvel(120,"kts","m/s");
[~,~,~,rho] = atmosisa(0);
Cl_max = 1.5;
V_stall = convvel(60,"kts","m/s");


T_W = [0:1/300:1]';
P_W = T_W .* (V/eta_p);
W_S = [0:1500/300:1500]';


%% ------------------------------Requirements --------------------------
G = 8.3/100;
G_OEI = 1.5/100;
G_balked_landing = 3/100;

climb_req = (V/eta_p) .* (G + 2*sqrt(Cd_0 / (pi*AR*e)) ) * ones(length(T_W),1);
climb_req_OEI = 2*(V/eta_p) .* (G_OEI + 2*sqrt(Cd_0 / (pi*AR*e)) ) * ones(length(T_W),1);
climb_req_balked_landing = (V/eta_p) .* (G_balked_landing + 2*sqrt(Cd_0 / (pi*AR*e)) ) * ones(length(T_W),1);

stall_req = V_stall^2*(0.5 * rho * Cl_max) * ones(length(W_S),1);

%% ----------------------------- Constraint Map -------------------------
figure()
xlim([W_S(1) W_S(end)])
ylim([P_W(1) P_W(end)])
hold on
plot(W_S,climb_req, '--', 'LineWidth', 2,"DisplayName","Climb requirement")
plot(W_S,climb_req_OEI, '--', 'LineWidth', 2,"DisplayName","Climb requirement OEI")
plot(W_S,climb_req_balked_landing, '--', 'LineWidth', 2,"DisplayName","Climb requirement Balked Landing")
plot(stall_req,P_W, '--', 'LineWidth', 2,"DisplayName","Stall Requirement")
hold off
xlabel('Wing Loading, $\frac{W}{S}$', "Interpreter","latex", "FontSize", 16)
ylabel('Power to Weight, $\frac{P}{W}$', "Interpreter","latex", "FontSize", 16)
set(gca, "FontSize", 16, "TickLabelInterpreter", "latex")
title("Constraint Map", "Interpreter","latex", "FontSize", 16)

legend_handle = legend;
set(legend_handle, 'NumColumns', 1, 'Location','NorthEast', "Interpreter", "latex", "FontSize", 10)
grid minor


