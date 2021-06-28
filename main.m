clear;clc;
load('laser_data.mat')
%% Description One dimension test
% sensor: px
% state: px,py,pz,vx,vy,vz
% state_augment: px,py,pz,vx,vy,vz,0,0,0
% pv: process noisy
% pn: measurement noisy
% nx = 6
% nx_aug = 9
% nz = 3
pz = px;
gtpz = gtpx;
gtvz = gtvx;
% Parameter

% measurement noisy
std_laspx = 0.02;
std_laspy = 0.02;
std_laspz = 0.02;
% process noisy
std_ax = 0.2;
std_ay = 0.2;
std_az = 0.2;
mplot = 0;

% Init state and init P matrix

State = [px(1);
         py(1);
         pz(1);
         0;
         0;
         0];

P = [std_laspx*std_laspx 0 0 0 0 0;
     0 std_laspy*std_laspy 0 0 0 0;
     0 0 std_laspy*std_laspy 0.1 0 0;
     0 0 0 0.1 0 0;
     0 0 0 0 0.1 0
     0 0 0 0 0 0.1];

time_pre = 0; 

px_ukf = zeros(size(px));
py_ukf = zeros(size(py));
pz_ukf = zeros(size(pz));

px_ukf(1) = px(1);
py_ukf(1) = py(1);
pz_ukf(1) = py(1);

vx_ukf = zeros(size(px));
vy_ukf = zeros(size(py));
vz_ukf = zeros(size(pz));

vx_ukf(1) = 0.0;
vy_ukf(1) = 0.0;
vz_ukf(1) = 0.0;

for i = 2:size(px)
    
    
    % Sigma Points and Weights
    apha = 2;
    process_noisy = [std_ax;std_ay;std_az];
    [Sigma_Points, Weights] = calculateSigPntsandWeights(State, P, apha, process_noisy);
    
    if mplot ==1
        w = waitforbuttonpress;
        if w == 1
            plot(State(1,1), State(2,1),State(3,1),'b*');
            hold on;
        end
        w = waitforbuttonpress;
        if w == 1
            for j = 1:size(Sigma_Points,2)
                plot3(Sigma_Points(1,j),Sigma_Points(2,j),Sigma_Points(3,j),'bo')
                hold on;
            end
        end
    end
    % Prediction
    dt = time(i)-time_pre;
    time_pre = time(i);
    F = [1 0 0 dt 0 0 0.5*dt*dt 0 0;
        0 1 0 0 dt 0 0 0.5*dt*dt 0;
        0 0 1 0 0 dt 0 0 0.5*dt*dt;
        0 0 0 1 0 0 dt 0 0;
        0 0 0 0 1 0 0 dt 0;
        0 0 0 0 0 1 0 0 dt]; % nx*nx_aug
    H= [1 0 0 0 0 0;
        0 1 0 0 0 0
        0 0 1 0 0 0]; % nz*nx
    [Sigma_pred, State_pred, P_pred, Z_sigma, Z_pred] = PredictionUpdate(Sigma_Points, Weights, F, H);
    
    if mplot ==1
        w = waitforbuttonpress;
        if w == 1
            for j = 1:size(Sigma_pred,2)
                plot(Sigma_pred(1,j),Sigma_pred(2,j),'ro')
                hold on;
            end
        end
        w = waitforbuttonpress;
        if w == 1
            plot(State_pred(1,1), State_pred(2,1),State_pred(3,1),'c*');
            hold on;
        end
    end
    % Correction
    Observation = [px(i);
        py(i);
        pz(i)];
    R = [std_laspx*std_laspx 0 0;
        0 std_laspy*std_laspy 0;
        0 0 std_laspz*std_laspz];
    [State, P] = MeasurementUpdate(Observation, R, Weights, Sigma_pred, State_pred, P_pred, Z_sigma, Z_pred);
    
    px_ukf(i) = State(1);
    py_ukf(i) = State(2);
    pz_ukf(i) = State(3);
    vx_ukf(i) = State(4);
    vy_ukf(i) = State(5);
    vz_ukf(i) = State(6);
    
    if mplot ==1
        w = waitforbuttonpress;
        if w == 1
            plot3(Observation(1,1), Observation(2,1), Observation(3,1),'m*');
        end
    end
    
end


subplot(1,2,1)
plot3(px, py,pz,'o');
hold on;
plot3(gtpx,gtpy,gtpz);
hold on;
plot3(px_ukf, py_ukf,pz_ukf,'k');
grid on;
legend('观测','真实','ukf');

subplot(1,2,2)
plot3(gtvx, gtvy,gtvz,'o');
hold on;
plot3(vx_ukf, vy_ukf,vz_ukf,'*');
grid on;
legend('真实','ukf');




