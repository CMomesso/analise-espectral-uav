% clear the MATLAB workspace
clear all
% close all open plots
close all

% load the given matrices from the file
load('Panama.mat')

% extract the time step (in s) -> column vector
t=tid4374(:,1);
% extract the function values (in some unit) -> column vector
x_t=tid4374(:,2);

% remove the offset of the time
t=t-t(1);

% plot the time function
figure(1)
plot(t,x_t)
xlabel('time in s')
ylabel('function in some unit')

% do the fast Fourier transform
% f contains the frequencies (in Hz) -> row vector
% X_f contains the spectral components (in some unit) -> row vector
[f,X_f]=fourier(t,x_t,'sinus');

% plot the spectrum
figure(2)
% linear axis scaling
%plot(f,abs(X_f))
% double-logarithmic axis scaling
loglog(f,abs(X_f))
xlabel('frequency in Hz')
ylabel('function in some unit')

% display the DC component
disp(['Constant offset: ',num2str(X_f(1)/2),' (in some unit)']);