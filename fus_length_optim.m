clear
clc
close all

alpha_to = 10; %angle of attack limit for takeoff, degrees 
S_ref = 11.211; 
h_gc = 0.18; %propeller ground clearance height
c_ht = 0.8;
c_vt = 0.04; %0.07;
C_w = 1.48;
b_w = 8.06;
% S_w = 44.8;
h = 1.2;

HTP_AR = 4;
HTP_lambda = 0.45; %taper ratio

VTP_AR = 1.6;
VTP_lambda = 0.45;

l = 1:0.1:10 ; %2:0.1:10;

L_ht = l;
L_vt = l;

S_H = (c_ht .* C_w .* S_ref) ./ L_ht; %for horizontal stabaliser
S_V = (c_vt .* (b_w) .* S_ref) ./ L_vt; %for vertical stabaliser

HTP_b = sqrt(HTP_AR .* S_H);
VTP_b = sqrt(VTP_AR .* S_V);

[tot_fus_S,S_fusf,S_fusa] = fuselageAreaCalc(l);

S_wet = 2*S_ref + 2*S_H + 2*S_V + S_fusf + S_fusa;



%% Case - pilots sat next to eachother

[tot_fus_SV2,S_fusfV2,S_fusaV2] = fuselageAreaCalcV2(l);

S_wetV2 = 2*S_ref + 2*S_H + 2*S_V + S_fusfV2 + S_fusaV2;

figure()
hold on
plot(l,S_wet)
plot(l,S_wetV2)
hold off
legend ("V1","V2")

best_l = l(S_wet == min(S_wet))
tot_l = best_l + 2.7

%% Weight graphs

ULF = 2.5*1.5; % ultimate load factor
DG = convmass(1000,'kg','lbm'); % landing weight
V = convvel(120,"kts","m/s");
h_1 = convlength(1500,"ft","m");
[~,~,~,rho] = atmosisa(h_1);
q_cruise = convpres(1/2 * rho * V^2, "pa", "psf" );

fus_w = 0.052.*(tot_fus_S*10.76391041671).^1.086*(ULF*DG).^0.177 * q_cruise^0.241;

DFTE = 0;

l_slope = sqrt((l+0.5).^2 + (h-0.125).^2);
beta = atand((h-0.125)./(l+0.5));
r = 1.4187/2;
XMLG_bar = r*cosd(alpha_to) + 0.18 - ((l+0.5)./cosd(beta) .* cosd(90-beta + alpha_to));
XMLG = XMLG_bar./cosd(alpha_to);
% XMLG(XMLG <= 0.4) = 0.4;
% XMLG = l.*sind(alpha_to)+h_gc; %extended main gear length in m
XNLG = XMLG;

S_XMLG_bar = 0.1.*(XMLG.^2).*pi;
S_XMLG_wheel = (((12*2.54)/2)^2 * pi * 2 + 12*2.54 * pi * 6*2.54)/(100^2);
S_XMLG = S_XMLG_bar + S_XMLG_bar;
S_XNLG = S_XMLG;

S_wet = S_wet + (2*S_XMLG + S_XNLG);

MG_W = (0.0117-0.00112 * DFTE) * DG^0.95 .* convlength(XMLG,"m","in").^0.43;
NG_W = (0.048-0.0080 * DFTE) * DG^0.67 .* convlength(XNLG,"m","in").^0.43;

SHT = S_H.*10.76391041671;
HT_W = 0.016*SHT.^0.873 .* (ULF * DG)^0.414 * q_cruise^0.122;

HHT = 0;
SVT = S_V * 10.76391041671;
ARVT = VTP_AR;
SWPVT = 0;
CSVT = cos(SWPVT);
TCVT = 0.12; % thickness to chord vtp, yet to do, this is provisional value
VT_W = 0.073*(1 + 0.2 * HHT)*(ULF * DG)^0.376 *q_cruise^0.122 .* SVT.^0.873 .* (ARVT/CSVT^2)^0.357 / (100*TCVT/CSVT)^0.49;


TR = 0.4;
AR = 5.8;
SPAN = convlength(b_w,"m","ft");
SW = S_ref * 10.76391041671;
FSTRT = 0;
EMS = 1 - 0.25 * FSTRT;
TLAM = tand(0) - (2*(1-TR)/(AR*(1+TR)));
SLAM = TLAM/sqrt(1+TLAM^2);
FAERT = 0;
C4 = 1-0.5 * FAERT;
C6 = 0.5 * FAERT - 0.16 * FSTRT;
CAYA = AR-5;
CAYL = (1-SLAM^2)*(1+C6*SLAM^2 + 0.03 * CAYA * C4 * SLAM);
t_c = 0.15;
TCA = t_c;
BT = 0.215 * (0.37 + 0.7 * TR) * (SPAN^2/SW)^(EMS/CAYL*TCA);

FCOMP = 1;
CAYF = 1;
VFACT = 1;
PCTL = 1;
A1 = 30;
A2 = 0;
A3 = 0.25;
A4 = 0.5;
A5 = 0.5;
A6 = 0.16;
A7 = 1.2;
W1NIR = A1 * BT * (1 + sqrt(A2/SPAN)) * ULF * SPAN * (1- 0.4 * FCOMP) * (1 - 0.1 * FAERT) * CAYF * VFACT * (100/1000000);
SFLAP = 0.2 * SW; % Provisional Needs to be ammended
W2 = A3 * (1-0.17*FCOMP)*SFLAP^A4 * DG^A5;
W3 = A6 * (1-0.3*FCOMP)*SW^A7;
NEW = 0; % number of wing mounted engines
CAYE = 1 - 0.03*NEW;
W1 = (DG * CAYE * W1NIR + W2 + W3)/(1+W1NIR) - W2 - W3;
WWING =  W1 + W2 + W3;

tot_weight = fus_w + NG_W + MG_W + HT_W + VT_W + WWING;

figure()
plot(l,(332.1 + (107.5+16.5+8.25+47.52-(tot_weight-WWING).*0.453592))./S_wet)

%% pressure drag

load("digitised_fineness_plots.mat")

afterbody_fineness_rat = l./1.5;
aft_drag_ratio = fittedmodel(afterbody_fineness_rat);
aft_drag_ratio(afterbody_fineness_rat >= 2.5) = fittedmodel(2.5);
aft_drag_ratio(find(aft_drag_ratio == min(aft_drag_ratio)):end) = min(aft_drag_ratio);

figure()
plot(afterbody_fineness_rat,aft_drag_ratio)

figure()
k = aft_drag_ratio' .* S_wet;
hold on
% plot(l, k)
% scatter(l(find(k == min(k))),k(find(k == min(k))))
plot(l,(332.1 + (107.5+16.5+8.25+47.52-tot_weight-WWING))./k)
hold off

k_bar = (332.1 + (107.5+16.5+8.25+47.52-(tot_weight-WWING).*0.453592))./S_wet;
l(k_bar == max(k_bar))

