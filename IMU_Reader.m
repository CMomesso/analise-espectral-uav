clc; clear all; close all;

rawdata = ardupilotreader("SemFiltroSemCargaCerto.bin");


%% Dados do giro

imuMsg = readMessages(rawdata, MessageName="IMU");
imuData = imuMsg.MsgData{1};

imuData.Properties.VariableNames;

t_giro = seconds(imuData.TimeUS - imuData.TimeUS(1));
gx = imuData.GyrX;   % Velocidade angular [rad/s]
gy = imuData.GyrY;
gz = imuData.GyrZ;

%% Dados de atitude
% Sinal processado (EKF)

att = readMessages(rawdata, MessageName="ATT");
attData = att.MsgData{1};
attData.Properties.VariableNames;

t_att = seconds(attData.TimeUS - attData.TimeUS(1));

figure(1)
% ROLL
subplot(3,1,1)
plot(t_att, attData.DesRoll, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Roll, 'LineWidth', 1.2);
grid on;
ylabel('Roll (deg)');
legend('Desejado','Real');

% PITCH
subplot(3,1,2)
plot(t_att, attData.DesPitch, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Pitch, 'LineWidth', 1.2);
grid on;
ylabel('Pitch (deg)');
legend('Desejado','Real');

% YAW
subplot(3,1,3)
plot(t_att, attData.DesYaw, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Yaw, 'LineWidth', 1.2);
grid on;
ylabel('Yaw (deg)');
xlabel('Tempo (s)');
legend('Desejado','Real');

%% FFT

% Tempo de análise - tirando decolagem e pouso

t_inicio = 4;
t_final = 30;

t_inicio_index = find(t_giro>=t_inicio, 1, 'first');
t_final_index = find(t_giro>=t_final, 1, 'first');
t_corte = t_giro(t_inicio_index:t_final_index);

gx_corte = gx(t_inicio_index:t_final_index);
gy_corte = gy(t_inicio_index:t_final_index);
gz_corte = gz(t_inicio_index:t_final_index);


figure(2)
sgtitle('Dados Brutos do Giroscópio')

subplot(3,1,1)
plot(t_corte, gx_corte);
title('Eixo x')
xlabel('Tempo [s]')
ylabel('Velocidade Angular [rad/s]')
grid on

subplot(3,1,2)
plot(t_corte, gy_corte);
title('Eixo y')
xlabel('Segundos [s]')
ylabel('Valocidade angular [rad/s]')
grid on

subplot(3,1,3)
plot(t_corte, gz_corte);
title('Eixo z')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

% detrend???

t = t_corte - t_corte(1);

figure(3)
[f, X_f] = fourier(t, gx_corte, 'sinus');
loglog(f, abs(X_f))
xlabel('Frequência [Hz]')
ylabel('Eixo x [?]')