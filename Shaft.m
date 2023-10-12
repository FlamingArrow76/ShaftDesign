Weight = 200 * 9.81;
PCD = 0.40127;
SCF = 2.5;
d_T = 0.1;          %Front bearing = d_T + L
L = 1;              %Rear bearing = d_T
d_R = 0.3;          %Break = d_t + L - d_B
d_B = 0.2;
Prop_Torque = 667;
Belt_Ten = -1*(Prop_Torque/PCD);

Break_Torque = -1200;
MatNames = ["EN3A", "EN8", "EN16", "EN24"];
Stress = [215, 280, 525, 680]*10^6;
Prices = [1.25, 1.36, 1.95, 2.66];
Density = [7800, 7850,8080, 7840] ;

SCF_Feat = 2.5;
fillet_rad = 0.005;
shaftacc = 0.0005;
Safety_Factor = 2.05 * 1.1;
Fatigue = 1.5;
Shock_Factor = 1;
Design_Factor = Safety_Factor * Fatigue * Shock_Factor;
Gen_Torque = Belt_Ten * (PCD/2);
R_F = ((Weight * (L+d_R)) - (d_T * -1*Belt_Ten))/L   ;   %Reaction Forces
R_B = -1*Belt_Ten + Weight - R_F;
block_size = 0.2;
F_Bearing_Width = 0.018;
R_Bearing_Width = 0.016;
fd1 = 0.0763;
rd1 = 0.0525;
Circlip_Groove = 2*0.0025;
Circlip_Thick = 0.003;
%Finds best shaft using cheapest material
cheapest = 1*10^11;
for mat = 1:length(Stress)
    [shaft, ~, bending_moment, Torque, Tension] = Normal(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque);
    [diameter_Norm, min_di_Norm] =  Make_Shaft(SCF, Design_Factor, bending_moment, Torque, Tension, shaft, block_size, shaftacc, Stress(mat));
    [shaft, ~, bending_moment, Torque, Tension] = Emergency(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque, Break_Torque);
    [diameter_Emer, min_di_Emer] =  Make_Shaft(SCF, Design_Factor, bending_moment, Torque, Tension, shaft, block_size, shaftacc, Stress(mat));
    diameter_Norm = [diameter_Norm, diameter_Norm(length(diameter_Norm))];
    diameter_Emer = [diameter_Emer, diameter_Emer(length(diameter_Emer))];
    HighDi = [];
    for i = 1:length(diameter_Emer)
        if diameter_Emer(i) > diameter_Norm(i)
            HighDi = [HighDi , diameter_Emer(i)];
        else
            HighDi = [HighDi , diameter_Norm(i)];
        end
    end  
    mindi = [];
    for i = 1:length( min_di_Emer)
        if min_di_Emer(i) > min_di_Norm(i)
            mindi = [mindi ,  min_di_Emer(i)];
        else
            mindi = [mindi ,  min_di_Emer(i)];
        end
    end 
    HighDi(1:uint16(((d_T+0.05)/shaftacc)+1)) = max(HighDi(1:uint16(((d_T+0.05)/shaftacc)+1))); %Smoothes shaft for rear bearing
    Brake = d_T + L - d_B;
    F_Bear = d_T+L;
    HighDi(uint16(((F_Bear-0.05)/shaftacc)+1):uint16(((F_Bear+0.05)/shaftacc)+1)) = max(HighDi(uint16(((F_Bear-0.05)/shaftacc)+1):uint16(((F_Bear+0.05)/shaftacc)+1)));
    
    %Adds shoulder to rest front bearing against
    HighDi(uint16(((F_Bear-block_size)/shaftacc)+1):uint16(((F_Bear-(0.5*F_Bearing_Width))/shaftacc)+1)) = (HighDi(uint16((F_Bear/shaftacc)+1)) + fd1)/2;% + Circlip_Groove;
    %Adds groove for circlip
    HighDi(uint16(((F_Bear+(0.5*F_Bearing_Width)+Circlip_Thick)/shaftacc)+1):uint16(((F_Bear+block_size)/shaftacc)+1)) = (HighDi(uint16((F_Bear/shaftacc)+1))+ Circlip_Groove);    
    HighDi(uint16(((F_Bear-(0.5*F_Bearing_Width))/shaftacc)+1):uint16(((F_Bear+(0.5*F_Bearing_Width))/shaftacc)+1)) = (HighDi(uint16((F_Bear/shaftacc)+1))+ Circlip_Groove);    
   
    % HighDi(uint16(((F_Bear+(0.5*Bearing_Width))/shaftacc)+1):uint16((((F_Bear+(0.5*Bearing_Width)+Circlip_Depth)/shaftacc)+1))) = (HighDi(uint16((F_Bear/shaftacc)+1))) - Circlip_Groove;
    
    R_Bear = d_T;
    %Adds shoulder to rest rear bearing against
    %HighDi(uint16(((R_Bear+(0.5*R_Bearing_Width))/shaftacc)+1):uint16((block_size/shaftacc)+1)) = (HighDi(uint16((R_Bear/shaftacc)+1)) + rd1)/2 + Circlip_Groove;
    %Adds groove for circlip
    HighDi(uint16(1):uint16(((R_Bear-(0.5*R_Bearing_Width)-Circlip_Thick)/shaftacc)+1)) = (HighDi(uint16((R_Bear/shaftacc)+1))+ Circlip_Groove);    
    HighDi(uint16(((R_Bear-(0.5*R_Bearing_Width))/shaftacc)+1):uint16((R_Bear+(0.5*R_Bearing_Width))/shaftacc)+1) = (HighDi(uint16((R_Bear/shaftacc)+1))+ Circlip_Groove);    
    
    HighDi(uint16(((R_Bear+(0.5*R_Bearing_Width))/shaftacc)+1):uint16((block_size/shaftacc)+1)) = (HighDi(uint16((R_Bear/shaftacc)+1)) + rd1)/2;

    
    HighDi(uint16(((Brake-0.05)/shaftacc)+1):uint16(((Brake+0.05)/shaftacc)+1)) = max(HighDi(uint16(((Brake-0.05)/shaftacc)+1):uint16(((Brake+0.05)/shaftacc)+1)));

    Volume = 0;
    for i = 1:length(shaft)
        Volume = Volume + (0.25*pi*(HighDi(i)^2)*shaftacc);
    end
    Mass = Volume * Density(mat);
    Billet = 0.25*pi*(max(HighDi)^2);
    cost = Billet * Density(mat) * Prices(mat);
    if Mass < cheapest
      bestdi =  HighDi;
      bestmat = mat;
      bestcost = cost;
      bestvol = Volume;
      cheapest = cost;
    end
    diameter_Norm = [];
    diameter_Emer = [];
end

scf = [];
scf(1:length(bestdi)) = 1;
for i = 2:length(bestdi)
    if (bestdi(i) > bestdi(i-1))
        scf(i-(fillet_rad/shaftacc):i) = SCF;
    end
    if (bestdi(i) < bestdi(i-1))
        scf(i:i+(fillet_rad/shaftacc)) = SCF;
    end    
end

Norm_BMStress = [];
Norm_Shear_Stress = [];
Norm_Tensile_Stress = [];
Norm_Stress = [];
Norm_VonMisces = [];
[~, ~, bending_moment, Torque, Tension] = Normal(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque);
 for l = 1:length(bestdi)
    tempBM = scf(l)* bending_moment(l) * (32/(pi*bestdi(l)^3));
    Norm_BMStress = [Norm_BMStress, tempBM];
    tempSh = scf(l) * Torque(l) * (16/(pi*bestdi(l)^3));
    Norm_Shear_Stress = [Norm_Shear_Stress, tempSh];
    tempTen = scf(l) * Tension(l) * (4/(pi*bestdi(l)^2));
    Norm_Tensile_Stress = [Norm_Tensile_Stress, tempTen];
    Norm_Stress = [Norm_Stress, tempBM + tempTen];
    Norm_VonMisces = [Norm_VonMisces, Design_Factor*((Norm_Stress(l))^2 + 3*(Norm_Shear_Stress(l))^2)^0.5];
 end
 
Emer_BMStress = [];
Emer_Shear_Stress = [];
Emer_Tensile_Stress = [];
Emer_Stress = [];
Emer_VonMisces = [];
[~, ~, bending_moment, Torque, Tension] = Emergency(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque, Break_Torque);
for l = 1:length(bestdi)
    tempBM = scf(l)* bending_moment(l) * (32/(pi*bestdi(l)^3));
    Emer_BMStress = [Emer_BMStress, tempBM];
    tempSh = scf(l) * Torque(l) * (16/(pi*bestdi(l)^3));
    Emer_Shear_Stress = [Emer_Shear_Stress, tempSh];
    tempTen = scf(l) * Tension(l) * (4/(pi*bestdi(l)^2));
    Emer_Tensile_Stress = [Emer_Tensile_Stress, tempTen];
    Emer_Stress = [Emer_Stress, tempBM + tempTen];
    Emer_VonMisces = [Emer_VonMisces, Design_Factor*((Emer_Stress(l))^2 + 3*(Emer_Shear_Stress(l))^2)^0.5];
end
mindi = [mindi, mindi(end)];
tolerances = [];
for i = 1:length(bestdi)
    tolerances = [tolerances,(bestdi(i)- mindi(i))];
end
figure
plotthelot(Stress(bestmat),shaft, Norm_BMStress,Norm_Shear_Stress, Norm_Tensile_Stress,Norm_Stress,Norm_VonMisces,Emer_BMStress,Emer_Shear_Stress, Emer_Tensile_Stress,Emer_Stress,Emer_VonMisces)
figure
plot(shaft, 1000*0.5*bestdi)
hold on 
plot(shaft, 1000*-0.5*bestdi)
plot(shaft, 10*tolerances)

%plot(shaft, 5*scf)
fprintf('The best material is %s \n', MatNames(bestmat))
fprintf('The billet costs £%.2f and shaft weighs %.2fkg\n', bestcost,bestvol*Density(bestmat))


function [diameters, min_di] =  Make_Shaft(SCF, Design_Factor, bending_moment, Torque, Tension, shaft, block, shaftacc, Max_Stress)
    diameters = [];
    min_di = [];
    for num = 0:block:1.3
        start = uint16((num/shaftacc)+1);
        finish = uint16((num+block)/shaftacc);
        BM = bending_moment(start:finish);
        Tor = Torque(start:finish);
        Ten = Tension(start:finish);
        chosen = 0;
        for diameter = 0.005:0.005:1      
            BMStress = SCF* BM * (32/(pi*diameter^3));
            Shear_Stress = SCF* Tor * (16/(pi*diameter^3));
            Tensile_Stress = SCF* Ten * (4/(pi*diameter^2));
            Norm_Stress = Tensile_Stress + BMStress;
            VonMisces = [];
            for a = 1:length(BMStress)
                VonMisces = [VonMisces, Design_Factor*((Norm_Stress(a))^2 + 3*(Shear_Stress(a))^2)^0.5];
            end
            gap = Max_Stress - max(VonMisces);
            if chosen == 0
                if (gap > Max_Stress*0.005)
                    small_di = diameter;
                    chosen = 1;
                end
            end
                    
            if (gap > Max_Stress*0.02) 
                best_di = diameter;
                break
            end
        end
        for i = 0:shaftacc:block-shaftacc
            diameters = [diameters, best_di];
            min_di = [min_di, small_di];
        end
    end
end

function [shaft, shear_force, bending_moment, Torque, Tension] = Normal(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque)
shaft = [];
shear_force = [];
Gen_Torque = Belt_Ten * (PCD/2);

for l = 0:shaftacc:1.4
    shaft = [shaft, l];
    if l < d_T
        shear_force = [shear_force, Belt_Ten];
    elseif l >= d_T && l < (d_T+L)
        shear_force = [shear_force, (Belt_Ten) + R_B];
    elseif l > (d_T+L) && l < 1.4
        shear_force = [shear_force, Weight]; 
    else
        shear_force = [shear_force, 0];         
    end
end

bending_moment = [0];
for a = 1:length(shaft)-1
    tempar = shear_force(a)*shaftacc;
    bending_moment = [bending_moment, bending_moment(a)+tempar];
end

Torque = [];
for l = 0:shaftacc:1.4
    if l >= 1.4
        Torque = [Torque, Prop_Torque + Gen_Torque];
    else
        Torque = [Torque, Gen_Torque];
    end
end

Tension = [];
for l = 0:shaftacc:1.4
    if l >= 1.1 && l < 1.4
        Tension = [Tension, -1500];
    else
        Tension = [Tension, 0];
    end
end

end

function [shaft, shear_force, bending_moment, Torque, Tension] = Emergency(shaftacc, R_F, R_B, PCD, d_T, d_R, d_B, L, Belt_Ten,Weight, Prop_Torque, Break_Torque) 
shaft = [];
shear_force = [];
for l = 0:shaftacc:1.4
    shaft = [shaft, l];
    if l < d_T
        shear_force = [shear_force, 0];
    elseif l >= d_T && l < (d_T+L)
        shear_force = [shear_force, -R_B];
    elseif l > (d_T+L) && l < 1.4
        shear_force = [shear_force, Weight]; 
    else
        shear_force = [shear_force, 0];         
    end
end

bending_moment = [0];       %Area under shear force graph
for a = 1:length(shaft)-1
    tempar = shear_force(a)*shaftacc;
    bending_moment = [bending_moment, bending_moment(a)+tempar];
end

Torque = [];
for l = 0:shaftacc:1.4
    if l >= 0.9 && l < 1.4
          Torque = [Torque, Break_Torque];
    elseif l >= 1.4
          Torque = [Torque, -1200 + Prop_Torque];
        else 
          Torque = [Torque, 0];
    end
end

Tension = [];
for l = 0:shaftacc:1.4
    if l >= 1.1 && l < 1.4
        Tension = [Tension, -1500];
    else
        Tension = [Tension, 0];
    end
end

end

function plotthelot(Stress, shaft, Norm_BMStress,Norm_Shear_Stress, Norm_Tensile_Stress,Norm_Stress,Norm_VonMisces,Emer_BMStress,Emer_Shear_Stress, Emer_Tensile_Stress,Emer_Stress,Emer_VonMisces)
    subplot(5,2,1)
    plot(shaft, Norm_BMStress)
    title('Norm BMStress')

    subplot(5,2,3)
    plot(shaft, Norm_Shear_Stress)
    title('Norm Shear_Stress')

    subplot(5,2,5)
    plot(shaft, Norm_Tensile_Stress)
    title('Norm Tensile_Stress')

    subplot(5,2,7)
    plot(shaft, Norm_Stress)
    title('Norm_Stress')

    subplot(5,2,9)
    plot(shaft, Norm_VonMisces)
    title('Norm VonMisces')
    hold on
    yline(Stress);
    hold off

    subplot(5,2,2)
    plot(shaft, Emer_BMStress)
    title('Emer BMStress')

    subplot(5,2,4)
    plot(shaft, Emer_Shear_Stress)
    title('Emer Shear_Stress')

    subplot(5,2,6)
    plot(shaft, Emer_Tensile_Stress)
    title('Emer Tensile_Stress')

    subplot(5,2,8)
    plot(shaft, Emer_Stress)
    title('Emer Stress')

    subplot(5,2,10)
    plot(shaft, Emer_VonMisces)
    title('Emer VonMisces')
    hold on
    yline(Stress);
    hold off
end

