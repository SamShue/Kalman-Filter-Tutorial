clc;
clear all;
close all;

gaus = @(x,mu,sig,amp,vo)amp*exp(-(((x-mu).^2)/(2*sig.^2)))+vo;

x = -5:0.1:5;

v = normrnd(0, 0.1, 1, 100);
y = gaus(x, 0, 0.1, 1, 0);
subplot(4,2,1); plot(v); ylim([-2,2]); title('Std. Dev. 0.1')
subplot(4,2,2); plot(x,y); title('Std. Dev. 0.1')

v = normrnd(0, 0.2, 1, 100);
y = gaus(x, 0, 0.2, 1, 0); 
subplot(4,2,3); plot(v); ylim([-2,2]); title('Std. Dev. 0.2')
subplot(4,2,4); plot(x,y); title('Std. Dev. 0.1')

v = normrnd(0, 0.5, 1, 100);
y = gaus(x, 0, 0.5, 1, 0); 
subplot(4,2,5); plot(v); ylim([-2,2]); title('Std. Dev. 0.5')
subplot(4,2,6); plot(x,y); title('Std. Dev. 0.1')

v = normrnd(0, 1.0, 1, 100);
y = gaus(x, 0, 1.0, 1, 0);
subplot(4,2,7); plot(v); ylim([-2,2]); title('Std. Dev. 1.0')
subplot(4,2,8); plot(x,y); title('Std. Dev. 0.1')