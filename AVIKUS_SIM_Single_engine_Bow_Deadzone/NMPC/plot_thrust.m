% Parameters
s = 25;
k = 1;
KPrvs = 12.2293;
KPfwd = 15.8382;
dead_rps = 10.0;

a1 = KPrvs * dead_rps;
a2 = KPfwd * dead_rps;
b1 = KPrvs;
b2 = KPfwd;

% T_cmd 범위
T_cmd = linspace(-3, 3, 1000);

% T_state 계산
T_state = (1 ./ (1 + exp(s .* T_cmd))) .* (b1 .* T_cmd + tanh(k .* T_cmd) .* a1) + ...
          (1 ./ (1 + exp(-s .* T_cmd))) .* (b2 .* T_cmd + tanh(k .* T_cmd) .* a2);

% 그래프 출력
figure;
plot(T_cmd, T_state, 'LineWidth', 2);
grid on;
xlabel('T_{cmd}');
ylabel('T_{state}');
title('T_{state} vs T_{cmd}');
