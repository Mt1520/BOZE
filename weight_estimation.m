%% ------------------------- -----Script for Weight Estimation -------------------------------------------

%House Keeping
clear 
close all
clc

%% Intialisation

% Plane Vals
dragonfly.wing.AR = 5.9; % aspect ratio picked from comparison to do 355
dragonfly.e = 0.8;
dragonfly.K_ld = 9; %taken from raymer
dragonfly.wing.Swet_Sref = 4.3; %taken from raymer, estimate between cessna skylane rg and beech duchess
dragonfly.L_D_max = dragonfly.K_ld*sqrt(dragonfly.wing.AR/(dragonfly.wing.Swet_Sref));
dragonfly.Cd_0 = 0.25*pi*dragonfly.wing.AR*dragonfly.e/(dragonfly.L_D_max)^2;
dragonfly.eta_p = 0.8;

% Environmental Vals
gamma = 1.4;
V = convvel(120,"kts","m/s");
[~,~,P_0,rho_0] = atmosisa(0);
h_1 = convlength(1500,"ft","m");
[~,a_1,P_1,rho_1] = atmosisa(h_1);
Cl_max = 1.5;
V_stall = convvel(60,"kts","m/s");
max_G = 6;
max_speed = sqrt(max_G)*V_stall;
max_mach = max_speed/a_1;
delta_0 = (P_1/P_0) * (1 + ((gamma - 1)/2) * max_mach^2)^(gamma/(gamma-1));
T_T0 = delta_0*( 1 - 0.49*sqrt(max_mach));

%% Weight Approx (not very accurate)

% init params
dragonfly.wing.MAC = []; % MAC = mean aero chord
dragonfly.wing.Sw = []; % m^2
dragonfly.HTP.MAC = [];
dragonfly.HTP.Sw = [];
dragonfly.VTP.MAC = [];
dragonfly.VTP.Sw = [];
dragonfly.Fuselage.length = [];
dragonfly.Fuselage.Sw = []; % fuselage wetted area
dragonfly.TOGW = 959.673; % total gross weight kg
dragonfly.engine.weight = []; % actual weight of the engines 

% weight calculation
Wing_weight = 12 * dragonfly.wing.Sw; % kg/m^2
Wing_weight_loc = 0.4 * dragonfly.wing.MAC; % percentage of wing MAC
Horz_tail_weight = 10 * dragonfly.HTP.Sw; % kg/m^2
Horz_tail_weight_loc = 0.4 * dragonfly.HTP.MAC; % percentage of wing MAC
Vert_tail_weight = 10 * dragonfly.VTP.Sw; % kg/m^2
Vert_tail_weight_loc = 0.4 * dragonfly.VTP.MAC; % percentage of wing MAC
Fuselage_weight = 7 * dragonfly.Fuselage.Sw; % kg/m^2
Fuselage_weight_loc = 0.45 * dragonfly.Fuselage.length; % Percentage of total fuselage length
LandingGear_weight = 0.057 * dragonfly.TOGW; % of TOGW at centroid
Engine_weight = 1.4 * dragonfly.engine.weight; % of Engine Weight at centroid
All_else_empty = 0.1 * dragonfly.TOGW; % of TOGW 
All_else_empty_loc = 0.45 * dragonfly.Fuselage.length; % Percentage of total length

%% Weight Approx (Better) all values should be in lbs, ft 

%init params
n = []; % limit load factor
S_w = []; % trapezoidal wing area ft^2
W_fw = []; % weight of fuel in wing lbs
sweep = []; % at 0.25 MAC
sweep_ht = []; % self explanatory ft^2
sweep_vt = []; % self explanatory ft^2
q = []; % dynamic pressure at cruise lb/ft^2
lambda = []; % taper ratio
lambda_ht = []; % self explanatory
lambda_vt = []; % self explanatory
t_c = []; % thickness to chord ratio
N_z = 1.5 * n; % ultimate load factor where n = limit load factor
W_dg = []; % design gross weight lbs
S_ht = []; % HTP area ft^2
S_vt = []; % VTP area ft^2
S_f = []; % fuselage wetted area ft^2
L_t = []; % tail length ft
L_D = []; % lift to drag ratio
H_t = []; % HTP height above fuselage ft
H_v = []; % VTP height above fuselage ft
V_pr = []; % Volume of pressurisised section ft^3
P_delta = []; % cabin pressure differential psi
W_press = 11.9 * (V_pr * P_delta)^0.271; % weight penalty due to pressurisation 
N_gear = []; % gear limit load factor
N_l = N_gear * 1.5; % ultimate gear load factor
W_l = W_dg; % landing gross weight lbs
L_m = []; % extended main gear length, inches
L_n = []; % extended nose gear length, inches
W_en = []; % engine weight lbs
N_en = 2; % number of engines

% calculating weights
dragonfly.wing.weight = 0.036 * S_w^0.758 * W_fw^0.0035 * (dragonfly.wing.AR/(cosd(sweep)^2))^0.6 * q^0.006 * lambda^0.04 * (100*t_c/cosd(sweep))^(-0.03) * (N_z * W_dg)^0.49;
dragonfly.HTP.weight = 0.016 * (N_z * W_dg)^0.414 * q^0.168 * S_ht^0.896 * (100*t_c/cosd(sweep_ht))^(-0.12)  * (dragonfly.wing.AR/(cosd(sweep_ht)^2))^0.043 * lambda_ht^(-0.02);
dragonfly.VTP.weight = 0.073 * (1 + 0.2*(H_t/H_v)) * (N_z * W_dg)^0.376 * q^0.122 * S_vt^0.873 * (100*t_c/cosd(sweep_vt))^(-0.49)  * (dragonfly.wing.AR/(cosd(sweep_vt)^2))^0.357 * lambda_vt^(0.039);
dragonfly.Fuselage.weight = 0.052 * S_f^1.086 * (N_z * W_dg)^0.177 * L_t&(-0.051) * L_D^(-0.072) * q^0.241 + W_press;
MainG_weight = 0.095 * (N_l * W_l)^0.768 * (L_m/12)^0.409; % MainG = main gear
NoseG_weight = 0.125 * (N_l * W_l)^0.566 * (L_n/12)^0.845; % NoseG = nose gear
dragonfly.gear.weight = (1-0.014) * (MainG_weight + NoseG_weight);
dragonfly.engine.installed_weight = 0.257 * W_en^0.922 * N_en ;
