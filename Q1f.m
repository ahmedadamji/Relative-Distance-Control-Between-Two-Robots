
% R2 speed - input
s_t = 0;
% R2 speed history
s = [];
% R1 speed - input
u_t = 0;
% R1 speed history
u = [];
% robot state
d_t = 1;
% robot state history
d = [];
% robot reference state
dr = 1;
% robot reference state history
dr_arr = [];
% system dynamics variable
d_dot = 0;

% noise magnitude
gauss_noise_std = 0.01;

% time array
time = [];
% Sampling time here is set to 0.2s
dt = 0.2;

% The discrete controller is:
%         U(z)   5.469 z - 4.531     5.469 - 4.531 z^-1
%  G(z) = ---- = ---------------- = ---------------------
%         E(z)         z - 1               1 - z^-1

%  U(z)(1 - z^-1) = (5.469 - 4.531 z^-1)E(z)

%  u[k] - u[k-1] = 5.469e[k] - 4.531e[k-1]

%  u[k] = 5.469e[k] - 4.531e[k-1] + u[k-1]

% initialise discrete controller
e = 0; % state error
e_prev = 0; % previous state error
u_t_prev = 0; % previous output generated from PID


for t = 0:dt:16

    % Reading R2 speed respective to time
    s_t = speed_of_r2(t);

    % Discretised PID control
    e = dr - d_t; %error
    u_t = 5.469*e - 4.531*e_prev + u_t_prev + (randn*gauss_noise_std); %output of discritised pid with noise
    e_prev = e; %storing previous error
    u_t_prev = u_t; %storing previous input

    % inverse kinematics
    d_dot = u_t - s_t;
    d_t = d_t + (d_dot*dt) + (randn*gauss_noise_std); %value of d with noise
    
    %Updating u, s, d, dr and time arrays for plotting:
    u = [u u_t];
    s = [s s_t];
    d = [d d_t];
    dr_arr = [dr_arr dr];
    time = [time t];
end

%figure to display u
figure('name', 'u(t)');
ax1 = gca;
view(ax1,2);
grid ON
grid MINOR
hold(ax1,'on')
axis auto
plot(ax1, time,u,"DisplayName","u(t)__disc__w/noise");
plot(ax1, time,s,"DisplayName","s(t)");
xlabel('time (s)');
ylabel('speed (m/s)');
legend(ax1);

%figure to display d
figure('name', 'd(t)');
ax2 = gca;
view(ax2,2);
grid ON
grid MINOR
hold(ax2,'on')
axis auto
plot(ax2, time,d,"DisplayName","d(t)__disc__w/noise");
plot(ax2, time,dr_arr,"DisplayName","dr");
xlabel('time (s)');
ylabel('distance (m)');
legend(ax2);
%ax2.YLim = [0 inf];

%Function that returns the speed of R2 as a function of time:
function s_t = speed_of_r2(t)
    if (0 <= t) && (t < 4)
        s_t = 0.1*t;
    elseif (4 <= t) && (t < 8)
        s_t = 0.4;
    elseif (8 <= t) && (t < 12)
        s_t = 0.8 - (0.05*t);
    elseif (12 <= t)
        s_t = 0.2;
    end
end