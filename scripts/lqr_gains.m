% --- A. PARAMETERS AND EQUILIBRIUM ---
ueq = 1.0;
m_s = 180; Iz_s = 466;
X_u_dot = 0; Y_v_dot = 0; N_r_dot = 0;
Xu = -100; Xuu = -150;
Yv = -100; Yvv = -100;
Nr = -800; Nrr = -800;
Taueq =[100+150*ueq*ueq,0];
m11 = m_s - X_u_dot; m22 = m_s - Y_v_dot; m33 = Iz_s - N_r_dot;

% --- B. REDUCED MATRICES (4 STATES: psi, u, v, r) ---
% Remove x and y to avoid redundancies with the integrators
% States: 1:psi, 2:u, 3:v, 4:r
Ac = [ 0, 0, 0, 1;                                % dpsi/dt = r
       0, (Xu + 2*Xuu*ueq)/m11, 0, 0;             % du/dt
       0, 0, Yv/m22, -m11*ueq/m22;                % dv/dt (Simplified Coriolis)
       0, 0, 0, Nr/m33 ];                         % dr/dt
Bc = [ 0, 0; 
       1/m11, 0; 
       0, 0; 
       0, 1/m33 ];
Cc = [ 1, 0, 0, 0;   % Output 1: psi
       0, 1, 0, 0 ];  % Output 2: u
Dc = zeros(2, 2);

% --- C. DISCRETIZATION ---
Tm = 0.1;
sysD = c2d(ss(Ac, Bc, Cc, Dc), Tm);
A = sysD.A; B = sysD.B; C = sysD.C; D = sysD.D;

% --- D. AUGMENTED STATE (4 states + 2 integrators = 6 total) ---
n = 4; p = 2; m = 2;
Aa = [ A,         zeros(n, p);
      -C,         eye(p, p) ];
Ba = [ B; 
       zeros(p, m) ];

% For LQR
% 1. Define how much you care about the ERROR (Matrix Q)
% The following are for the states (psi, u, v, r, zkpsi, zku).
% A large number = "correct this error very fast".
Q = diag([100000, 50000, 100, 5000, 1000, 1000]); 

% 2. Define how much you care about ACTUATOR EFFORT (Matrix R)
% 2x2 matrix because you have 2 inputs (Xs and Ns).
% A large number = "do not use so much force, be smooth".
R = [0.008, 0; 
     0, 0.008];
K_lqr = dlqr(Aa, Ba, Q, R);

% 4. Separate matrices into state feedback gain and integral gain
Hc_lqr = K_lqr(:, 1:4); 
Hi_lqr = K_lqr(:, 5:6);