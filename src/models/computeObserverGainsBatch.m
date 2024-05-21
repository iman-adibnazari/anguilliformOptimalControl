clc
clear
close


%% Setup filepaths and ROM dimensions
romDir = "data/archivedDataSets/FullAssembly_Constrained_FullSetForRAL_goodMatParams/ROMs/";
newRomDir = "data/archivedDataSets/FullAssembly_Constrained_FullSetForRAL_goodMatParams/ROMsWithObserverGains/";
ns_ROM = 4:4:40;
ns_ROM_lopinf = 4:4:20;

%% Iterate through each rom dimension
for i = ns_ROM
    % generate file names for each rom
    eraFile = romDir + sprintf("eraSystemMatrices_%ddim_3train.mat",i);
    dmdcFile = romDir + sprintf("dmdcSystemMatrices_%ddim_3train.mat",i);
    lopinfFile = romDir + sprintf("lopinfSystemMatrices_%ddim_3train.mat",i);
    % Load in files
    load(eraFile);
    load(dmdcFile);
    load(lopinfFile);
    
    % Place Poles for each ROM
    poles = linspace (-.5,.5,i); % Place i poles equidistantly from -.5 to .5
    % era
    L_era = place(A_era',C_era',poles)';
    % dmdc
    L_dmdc = place(A_dmdc',C_dmdc',poles)';
    % lOpInf
    L_lopinf = place(A_lopinf',C_lopinf',poles)';

    

    % Save new files with observer gains
    newEraFile = newRomDir + sprintf("eraSystemMatricesAndGains_%ddim_3train.mat",i);
    newDmdcFile = newRomDir + sprintf("dmdcSystemMatricesAndGains_%ddim_3train.mat",i);
    newLopinfFile = newRomDir + sprintf("lopinfSystemMatricesAndGains_%ddim_3train.mat",i);

    save(newEraFile, "A_era", "B_era", "C_era", "D_era", "L_era")
    save(newDmdcFile, "A_dmdc", "B_dmdc", "C_dmdc", "L_dmdc", "basis_dmdc")
    save(newLopinfFile, "A_lopinf", "B_lopinf", "C_lopinf", "D_lopinf", "L_lopinf", "basis_lopinf")
    

end