clc; clear all;

arduObj = ardupilotreader("ComFiltroSemCarga_Certo_final.bin");

att = readMessages(arduObj, MessageName="ATT");
attData = att.MsgData{1};
attData.Properties.VariableNames;

t = (attData.TimeUS - attData.TimeUS(1)) * 1e-6;

figure;

% ROLL
subplot(3,1,1)
plot(t, attData.DesRoll, 'LineWidth', 1.2); hold on;
plot(t, attData.Roll, 'LineWidth', 1.2);
grid on;
ylabel('Roll (deg)');
legend('Desejado','Real');

% PITCH
subplot(3,1,2)
plot(t, attData.DesPitch, 'LineWidth', 1.2); hold on;
plot(t, attData.Pitch, 'LineWidth', 1.2);
grid on;
ylabel('Pitch (deg)');
legend('Desejado','Real');

% YAW
subplot(3,1,3)
plot(t, attData.DesYaw, 'LineWidth', 1.2); hold on;
plot(t, attData.Yaw, 'LineWidth', 1.2);
grid on;
ylabel('Yaw (deg)');
xlabel('Tempo (s)');
legend('Desejado','Real');

