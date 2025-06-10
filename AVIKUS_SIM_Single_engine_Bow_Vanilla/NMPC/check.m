dt = 0.5;
% bow_array = [0,0,1,-1];
% bow_array = [0.6,1];
bow_array = [0,0,1,0.8,0.6,0,-0.3,-0.6,1,-1];

Num = size(bow_array,2);
dwell_time = 2;

new_bow_array = bow_mapping(bow_array,dt,Num, dwell_time);

% x축 인덱스 생성
x1 = 1:length(bow_array);
x2 = 1:length(new_bow_array);

% 그래프 그리기
figure;
plot(x1, bow_array, '-o', 'DisplayName', 'bow\_array');
hold on;
plot(x2, new_bow_array, '-x', 'DisplayName', 'new\_bow\_array');
xlabel('Index');
ylabel('Value');
legend;
grid on;
title('Comparison of bow\_array and new\_bow\_array');