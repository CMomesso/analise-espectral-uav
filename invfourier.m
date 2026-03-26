% inverse Fourier transform via FFT
% original by Dr. Kochetov
% edited by Mathias Magdowski
% date: 2008-07-23
% eMail: mathias.magdowski@ovgu.de
%
% Input:
% f - frequency range (in Hz) -> vector of length N
%       - first frequency is 0
%       - equidistant frequency steps
% X_f - spectrum -> vector of length N or matrix with N rows
% modus - mode -> string
%   'pulse' -> transform of a pulse
%   'sinus' -> transform of a continuous signal
%   Background: The mode alone influences the normalization of the amplitude
%   of the spectrum.
%
% Output:
% t   - time range (in s) -> column vector of length 2*N or matrix with 2*N rows.
%       - first time is 0
%       - equidistant time steps
% x_t - time range -> column vector of length 2*N
%
% Remark:
% - both row and column vectors are allowed at the input
% - the length of the vectors should be a power of 2

function [t,x_t]=invfourier(f,X_f,modus) 
    % check if mode is set
    if nargin<3
        % set to default value
        modus='pulse';
    end

    % number of values -> scalar
    N=length(f);
    
    % frequency step (in Hz) -> scalar
    f_step=f(2);
    % maximum time (in s) -> scalar
    t_max=0.5/f_step;
    
    % convert row vector of X_f to column vector
    if size(X_f,1)==1
        X_f=X_f.';
    end

    % fill frequency vector -> vector
    % DC component (XX_f(1)) and positive frequencies in ascending direction
    XX_f(1:N,:)=X_f;
    % set highest frequency component to zero
    XX_f(N+1,:)=0;
    % negative mirror frequencies in decreasing direction
    XX_f(N+2:2*N,:)=conj(XX_f(N:-1:2,:));
    
    % perform inverse Fourier transform -> vector
    if strcmp(modus,'pulse')
        % the spectrum has the unit 1/Hz (e.g. V/Hz, A/Hz, ...)
        xx_t=N/t_max*ifft(XX_f);
    elseif strcmp(modus,'sinus')
        % the spectrum has the unit 1 (e.g. V, A, ...)
        xx_t=N*ifft(XX_f);
    else
        error(['The modus ',modus,' is unknown.'])
    end

    % create time response -> row vector(1,2:N)
    x_t(1:2*N,:)=xx_t(1:2*N,:);
    
    % create time range (in s) -> row vector(1,2:N)
    t=linspace(0,t_max*(2*N-1)/N,2*N);
    
    % convert row vector from t to column vector
    t=t.';
end

% Note: A multiple transformation using fourier and invfourier is 
% not reversible. In such a case it is better to transform directly 
% by means of fft and ifft and to keep the time or frequency vector.
