% 2016/3/2
% Seok Won Bang


clc;
clear all;
close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Macro
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RADTODEG = 180.0/pi;
DEGTORAD = pi/180;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Load dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Load raw dataset
%data_set_raw = load('./data_in/test0213_result2.txt');
%data_set_raw = load('C:\iLife\Work\path test\data_in/cam7_ilifepi5_Normal_Line_1170cm_20161222.txt');

% load encoder
% time, left, right
% 1560393549135132241,5655278,5634601
data_encoder_raw = load('./DR-data-0612-3/encoder.txt');

% load gyro
% time, angle, rate
% 1560393549082715222,-12.3,-1.74
data_gyro_raw = load('./DR-data-0612-3/gyro.txt');

figure(4);
subplot(2,1,1); plot(data_encoder_raw(:,1), data_encoder_raw(:,2), '.');
subplot(2,1,2); plot(data_gyro_raw(:,1), data_gyro_raw(:,2), '.');



    % col1: Time Image
    % col2: Image ID
    % col3: Gyro (x10 deg)
    % col4: Right Encoder Count
    % col5: Left Encoder Count
    % col6: X coordinate
    % col7: Y coordinate

%data_set_raw(:,3) = data_set_raw(:,3)./10;      % (x10 deg) -> (deg)
    % At this point, Completed to set data_set_raw.

%%
%%%% Pre-process data
totnum_step_raw = size (data_encoder_raw, 1);
data_set        = zeros( totnum_step_raw, 4);
% dataset: time, gyro, encoder left, encoder right

gyro_idx = 1;
for i=1:totnum_step_raw
    if i==1
        data_set(i, 1) = 0;
        data_set(i, 2) = 0;
        data_set(i, 3) = 0;
        data_set(i, 4) = 0;
    else
        t = data_encoder_raw(i,1);

        while (t>data_gyro_raw(gyro_idx,1))
            gyro_idx = gyro_idx + 1;
            if (gyro_idx>size(data_gyro_raw,1))
                break;
            end
        end

        data_set(i,1) = data_encoder_raw(i,1) - data_encoder_raw(i-1,1);
        data_set(i,2) = data_gyro_raw(gyro_idx-1,2);
        data_set(i,3) = data_encoder_raw(i,2) - data_encoder_raw(i-1,2);
        data_set(i,4) = data_encoder_raw(i,3) - data_encoder_raw(i-1,3);
    end
end


% data_set(:, 1)  = data_set_raw(:, 1);
% data_set(:, 2)  = data_set_raw(:, 2);
% data_set(:, 3)  = data_set_raw(:, 3) - data_set_raw(1, 3);
% data_set(:, 6)  = data_set_raw(:, 6)*0.0001816111;
% data_set(:, 7)  = data_set_raw(:, 7)*0.0001816111;



% for i = 1:totnum_step_raw,
%     if i == 1,
%         data_set(i, 1) = 0;
%         data_set(i, 4) = 0;
%         data_set(i, 5) = 0;
%     else
%         data_set(i, 1) = data_set_raw(i, 1) - data_set_raw((i-1), 1);
%         data_set(i, 4) = data_set_raw(i, 4) - data_set_raw((i-1), 4);
%         data_set(i, 5) = data_set_raw(i, 5) - data_set_raw((i-1), 5);
%     end
% end
    % At this point, Completed to set data_set.

%%
idx_step_end                = size(data_set, 1);

figure(1),
bar(data_set(:, 1),  'b');
title('Sampling Time Interval');

figure(2),
plot(data_set(:, 4), 'r-');
title('Right Encoder Increment');

figure(3),
plot(data_set(:, 3),  'r-');
title('Left Encoder Increment');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Arrange
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% set parameters
Param_pulse_per_rev         = 400000;
Param_radius_wheel          = 199.597/2.0 / 1000;          % (m)
Param_dist_bet_wheels       = 420.7 / 1000;          % (m)

% Do not edit below
Dist_OnePulse               = (2.0*pi*Param_radius_wheel)/Param_pulse_per_rev;


%%%% allocate for results
X_odo1  = 0.0;     % encoder + gyro
Y_odo1  = 0.0;
Th_odo1 = 0.0;     % (rad)

X_odo2  = 0.0;     % only encoder
Y_odo2  = 0.0;
Th_odo2 = 0.0;     % (rad)


%%%% allocate for results
X_odo1_set  = zeros(1, idx_step_end);
Y_odo1_set  = zeros(1, idx_step_end);
Th_odo1_set = zeros(1, idx_step_end);   % (rad)

X_odo2_set  = zeros(1, idx_step_end);
Y_odo2_set  = zeros(1, idx_step_end);
Th_odo2_set = zeros(1, idx_step_end);   % (rad)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for idx_step = 1:idx_step_end,
    cnt_eL      = data_set(idx_step, 3);                % encoder count (L)
    cnt_eR      = data_set(idx_step, 4);                % encoder count (R)
    Th_gyro     = data_set(idx_step, 2)*DEGTORAD;       % (rad)
    %Th_gyro     = correct_angle( Th_gyro );    ???
    
    dist_wL     = cnt_eL*Dist_OnePulse;
    dist_wR     = cnt_eR*Dist_OnePulse;
    dist_w_mean = 0.5*(dist_wL + dist_wR);              % (m)

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% type 1 (encoder + gyro)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dX = dist_w_mean*cos( Th_odo1 );    % Th_odo1 : Th at previous step
    dY = dist_w_mean*sin( Th_odo1 );
    
    X_odo1  = X_odo1 + dX;
    Y_odo1  = Y_odo1 + dY;
    %Th_odo1 = correct_angle( Th_gyro );    ????
    Th_odo1 = Th_gyro;
    
    %%%% store
    X_odo1_set (idx_step) = X_odo1;
    Y_odo1_set (idx_step) = Y_odo1;
    Th_odo1_set(idx_step) = Th_odo1;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %%%% type 2 (encoder only)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    dX = dist_w_mean*cos( Th_odo2 );    % Th_odo2 : Th at previous step
    dY = dist_w_mean*sin( Th_odo2 );
    
    X_odo2 = X_odo2 + dX;
    Y_odo2 = Y_odo2 + dY;
    
    dth = (1.0/Param_dist_bet_wheels)*(dist_wR - dist_wL);  % (rad)
   
    Th_odo2 = Th_odo2 + dth;
    %Th_odo2 = correct_angle( Th_odo2 );    ????
    
    
    %%%% store
    X_odo2_set (idx_step) = X_odo2;
    Y_odo2_set (idx_step) = Y_odo2;
    Th_odo2_set(idx_step) = Th_odo2;

end
    % At this point, 
    %       Completed to set X_odo1_set, Y_odo1_set, Th_odo1_set (encoder + gyro)
    %       Completed to set X_odo2_set, Y_odo2_set, Th_odo2_set (encoder only)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Common for show
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_min1 = min(X_odo1_set);
X_min2 = min(X_odo2_set);
X_min  = min([X_min1, X_min2]);

X_max1 = max(X_odo1_set);
X_max2 = max(X_odo2_set);
X_max  = max([X_max1, X_max2]);

Y_min1 = min(Y_odo1_set);
Y_min2 = min(Y_odo2_set);
Y_min  = min([Y_min1, Y_min2]);

Y_max1 = max(Y_odo1_set);
Y_max2 = max(Y_odo2_set);
Y_max  = max([Y_max1, Y_max2]);

X_min_ = X_min - 0.1;
Y_min_ = Y_min - 0.1;

X_max_ = X_max + 0.1;
Y_max_ = Y_max + 0.1;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Show results - animated
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if 0,
    figure(1),
    %pause;

    for idx_step = 1:idx_step_end,
        
        %%%% clear axis
        cla;
                
        %%%% draw robot path
        x_odo1_set_temp = X_odo1_set(1:idx_step);
        y_odo1_set_temp = Y_odo1_set(1:idx_step);
        
        plot(x_odo1_set_temp, y_odo1_set_temp, 'k');
        hold on;
        
        
        %%%% draw robot at current step
        x_odo1_temp     = X_odo1_set (idx_step);
        y_odo1_temp     = Y_odo1_set (idx_step);
        th_odo1_temp    = Th_odo1_set(idx_step);
        
        plot(x_odo1_temp, y_odo1_temp, 'b.', 'MarkerSize', 40);
        hold on;
        
        x_odo1_temp_dir = x_odo1_temp + (0.2)*cos(th_odo1_temp);
        y_odo1_temp_dir = y_odo1_temp + (0.2)*sin(th_odo1_temp);
        
        line([x_odo1_temp, x_odo1_temp_dir], [y_odo1_temp, y_odo1_temp_dir], 'Color', 'r', 'LineWidth', 2);
        hold on;
        
        %%%% set axis
        axis([X_min_, X_max_, Y_min_, Y_max_]);
        axis equal;
        
        str_title = sprintf('Robot path using encoders and gyro - Frame: [%d]', idx_step);
        title(str_title);
        
        pause(0.001);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Show results - still, overall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if 1,
    figure(10),
        hold on;
        plot(X_odo1_set, Y_odo1_set,'r'); %,data_set(:, 6),  data_set(:, 7), 'b');
        plot(X_odo1_set(1), Y_odo1_set(1),'r+'); %,data_set(:, 6),  data_set(:, 7), 'b');
        n = size(X_odo1_set);
        plot(X_odo1_set(n), Y_odo1_set(n),'ro'); %,data_set(:, 6),  data_set(:, 7), 'b');
        
        %axis([X_min_, X_max_, Y_min_, Y_max_]);
        grid on;
        legend('Encoder+Gyro','ILIFE(X,Y)')
        xlabel('m');
        ylabel('m');
        axis equal;
        title('Robot path using encoders and gyro');
               
    figure(20),
        hold on;
        plot(X_odo2_set, Y_odo2_set, 'k');
        plot(X_odo2_set(1), Y_odo2_set(1), 'k+');
        plot(X_odo2_set(n), Y_odo2_set(n), 'ko');
        axis([X_min_, X_max_, Y_min_, Y_max_]);
        axis equal;
        title('Robot path using encoders only');
end










