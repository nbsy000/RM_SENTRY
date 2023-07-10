load('acc1.mat')  
Fs = 100;%100Hz的采样频率
N = length(acc1);
x = (0:N-1)/Fs;%定义20001个取样点
y = acc1;%设计含有高频信号与低频信号的输入信号
figure(1);
plot(x,y);%画出输入信号图形
title('输入信号');

%使用fft绘制频谱
y0 = abs(fft(y));
f = (0:N-1)*Fs/N;
figure(2);
plot(f,y0);%fft频谱
title('频谱');


% tempy = y.*0;
% k = 0.03;
% for i = 2:N
%     tempy(i) = (1-k)*tempy(i-1)+(k)*y(i);
% end
% d = tempy;
Hd = acc_iir_2;%引入滤波器,Hd包含了fir_8滤波器的各项参数
d = filter(Hd,y);%通过filter函数将信号y送入参数为Hd的滤波器，输出信号d
figure(3);
plot(x,d);%画出通过滤波器的信号d的波形
title('输出信号');


figure(4);
plot(x(5000:6000),y(5000:6000),'r');%画出输入信号图形
hold on;%保持画出的输入信号图形
plot(x(5000:6000),d(5000:6000),'b');%画出输出信号波形
title('输入/输出信号');
legend('输出信号','输入信号');

figure(5);
plot(f,y0,'r');%画出输入频谱图形
hold on;%保持画出的输入信号图形
d0 = abs(fft(d));
plot(f,d0,'b');%画出输出信号波形
title('输入/输出信号');
legend('输出信号','输入信号');

