function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    % Place parameters like covarainces, etc. here:

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    
    dt =t - previous_t;
    
    A = [1 0 dt 0;   
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];

    C = [1 0 0 0;
         0 1 0 0];
    
    z_t = [x,y].';
    
    %motion error
    omega_m = [dt*dt/4  0    dt/2 0 ;
               0    dt*dt/4  0  dt/2;
               dt/2     0    1   0;
               0     dt/2     0  1];
    
    %measurement error
    omega_z = [0.01, 0;
               0, 0.01];
    
    
    P = A * param.P * transpose(A) +  omega_m;
    %disp(k)
    R = omega_z;
    K = P * transpose(C) * inv(R + C * P * transpose(C));
    predict = A * (state.') + K*(z_t - C * A * (state.'));
    param.P = P - K * C * P;
    state = predict.';
    
    predictx = predict(1) + predict(3)*(0.330);
    predicty = predict(2) + predict(4)*(0.330);
end
