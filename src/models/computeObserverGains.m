clc
clear
close

%% Read in identified system matrices
load('./data/processedData/romSystemMatrices_12dim_5train_25test.mat')

%% Place poles for each method
poles = -0.1:-0.01:-0.21;

% era
L_era = place(A_era',C_era',poles)';
% dmdc
L_dmdc = place(A_dmdc',C_dmdc',poles)';
% opInf
L_opInf = place(A_opInf',C_opInf',poles)';
% lOpInf
L_lopinf = place(A_lopinf',C_lopinf',poles)';

%% Export system matrices with gains
save("./data/processedData/romSystemMatricesAndGains_12dim_5train_25test.mat")