%% 1) arxika dedomena k theseis mikrofwnwn
m1 = [0; 0]; % ok 
m2 = [4; 0]; % ok 
m3 = [0; 4]; % ok
c  = 343; % taxitita hxou m/s % ok 

% pinakas A 2x2 me ta dianusmata vasis
A = [(m2 - m1).'; (m3 - m1).']; %ok

%% 2) prosomoiwsi metrisewn TDOA (ftiaxnoume ta tau apo gnwsti pigi gia test)
s_true = [8; 5]; % i pragmatiki thesi tis pigis
r = @(m) norm(s_true - m);

tau21 = (r(m2) - r(m1)) / c; % kathisterisi mic2 - mic1 (sec) % to vrisko apo tin plaketa
tau31 = (r(m3) - r(m1)) / c; % kathisterisi mic3 - mic1 (sec)  % to vrisko apo tin plaketa

%% 3) ipologismos b0 k b1
d21 = c * tau21; % diafora apostasis se m (mic2 - mic1)
d31 = c * tau31; % diafora apostasis se m (mic3 - mic1)

% sunartisi K(m) = m.' * m
K = @(m) m.' * m;

% ftiaxnoume ta b0 k b1
b0 = 0.5 * [ K(m2) - K(m1) - d21^2 ; ...
    K(m3) - K(m1) - d31^2 ];
b1 = [ d21 ; d31 ];

%% 4) lusi twn voithitikwn sistimatwn (fevgei o A)
s0 = A \ b0;
s1 = A \ b1;

%% 5) suntelestes deuterovathmias (a, b, g)
% einai akrivws o tupos apo tin eikona: (s1.'*s1 - 1)*r1^2 - 2*(s0.'*s1)*r1 + s0.'*s0 = 0
alpha = (s1.' * s1) - 1;
beta  = -2 * (s0.' * s1);
gamma = s0.' * s0;

%% 6) vriskoume to r1 (rizes)
% i roots pairnei to [a, b, g] k vgazei tis luseis
all_roots = roots([alpha, beta, gamma]);

% kratame mono tin thetiki pragmatiki riza giati r1 > 0 (apostasi)
r1_estimated = all_roots(all_roots > 0);

%% 7) teliki thesi s = [x; y]
s_estimated = s0 - s1 * r1_estimated;

%% 8) tipwma apotelesmatwn
fprintf('--- ΑΠΟΤΕΛΕΣΜΑΤΑ TDOA SOLVER ---\n');
fprintf('Εκτιμώμενη Θέση (Solver): [%.4f; %.4f]\n', s_estimated(1), s_estimated(2));
fprintf('Υπολογισμένη Απόσταση r1: %.4f μέτρα\n', r1_estimated);
