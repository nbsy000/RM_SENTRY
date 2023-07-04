    x = 0:0.01:4;%定义400个取样点
    y = 0.1*sin(pi*x) +0.1*sin(2*pi*49*x);%设计含有高频信号与低频信号的输入信号
    figure(1);
    plot(x,y);%画出输入信号图形
    title('输入信号');
    
    Hd = fir_8;%引入滤波器,Hd包含了fir_8滤波器的各项参数
    d = filter(Hd,y);%通过filter函数将信号y送入参数为Hd的滤波器，输出信号d
    figure(2);
    plot(x,d);%画出通过滤波器的信号d的波形
    title('输出信号');
    
    figure(3);
    plot(x,y,'r');%画出输入信号图形
    hold on;%保持画出的输入信号图形
    plot(x,d,'b');%画出输出信号波形
    title('输入/输出信号');
    legend('输出信号','输入信号');