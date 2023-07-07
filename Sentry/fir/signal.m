load('gimbalr_pitch.mat')  
Fs = 1000;%1000Hz�Ĳ���Ƶ��
N = length(gimbalrpitch);
x = (0:N-1)/Fs;%����20001��ȡ����
y = gimbalrpitch;%��ƺ��и�Ƶ�ź����Ƶ�źŵ������ź�
figure(1);
plot(x,y);%���������ź�ͼ��
title('�����ź�');

%ʹ��fft����Ƶ��
y0 = abs(fft(y));
f = (0:N-1)*Fs/N;
figure(2);
plot(f,y0);%fftƵ��
title('Ƶ��');

Hd = fir_8;%�����˲���,Hd������fir_8�˲����ĸ������
d = filter(Hd,y);%ͨ��filter�������ź�y�������ΪHd���˲���������ź�d
figure(3);
plot(x,d);%����ͨ���˲������ź�d�Ĳ���
title('����ź�');

figure(4);
plot(x,y,'r');%���������ź�ͼ��
hold on;%���ֻ����������ź�ͼ��
plot(x,d,'b');%��������źŲ���
title('����/����ź�');
legend('����ź�','�����ź�');

figure(5);
plot(f,y0,'r');%��������Ƶ��ͼ��
hold on;%���ֻ����������ź�ͼ��
d0 = abs(fft(d));
plot(f,d0,'b');%��������źŲ���
title('����/����ź�');
legend('����ź�','�����ź�');

