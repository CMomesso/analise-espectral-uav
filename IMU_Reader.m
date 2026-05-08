clc; clear all; close all;

rawdata = ardupilotreader("SemFiltroSemCargaCerto.bin");


%% ============================================================
%  DADOS DO GIRO - MENSAGEM IMU
% ============================================================

imuMsg = readMessages(rawdata, MessageName="IMU");
imuData = imuMsg.MsgData{1};

imuData.Properties.VariableNames;

t_giro = seconds(imuData.TimeUS - imuData.TimeUS(1));

gx = imuData.GyrX;   % Velocidade angular [rad/s]
gy = imuData.GyrY;
gz = imuData.GyrZ;


%% ============================================================
%  DADOS DE ATITUDE - MENSAGEM ATT
%  Sinal processado pelo EKF
% ============================================================

att = readMessages(rawdata, MessageName="ATT");
attData = att.MsgData{1};

attData.Properties.VariableNames;

t_att = seconds(attData.TimeUS - attData.TimeUS(1));

figure(1)
sgtitle('Atitude: desejado vs real')

% ROLL
subplot(3,1,1)
plot(t_att, attData.DesRoll, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Roll, 'LineWidth', 1.2);
grid on;
ylabel('Roll [deg]');
legend('Desejado','Real');

% PITCH
subplot(3,1,2)
plot(t_att, attData.DesPitch, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Pitch, 'LineWidth', 1.2);
grid on;
ylabel('Pitch [deg]');
legend('Desejado','Real');

% YAW
subplot(3,1,3)
plot(t_att, attData.DesYaw, 'LineWidth', 1.2); hold on;
plot(t_att, attData.Yaw, 'LineWidth', 1.2);
grid on;
ylabel('Yaw [deg]');
xlabel('Tempo [s]');
legend('Desejado','Real');


%% ============================================================
%  FFT SIMPLES USANDO IMU
%  Essa parte é só para comparação.
%  Não é a mesma lógica do Filter Review.
% ============================================================

% Tempo de análise
t_inicio_imu = 5;
t_final_imu  = 30;

t_inicio_index = find(t_giro >= t_inicio_imu, 1, 'first');
t_final_index  = find(t_giro >= t_final_imu, 1, 'first');

% Caso o log termine antes de t_final_imu
if isempty(t_final_index)
    t_final_index = length(t_giro);
end

t_corte = t_giro(t_inicio_index:t_final_index);

gx_corte = gx(t_inicio_index:t_final_index);
gy_corte = gy(t_inicio_index:t_final_index);
gz_corte = gz(t_inicio_index:t_final_index);

figure(2)
sgtitle('Dados brutos do giroscópio - IMU')

subplot(3,1,1)
plot(t_corte, gx_corte);
title('Eixo X')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

subplot(3,1,2)
plot(t_corte, gy_corte);
title('Eixo Y')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

subplot(3,1,3)
plot(t_corte, gz_corte);
title('Eixo Z')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

gx_proc = detrend(gx_corte);

t = t_corte - t_corte(1);

figure(3)
[f_imu, X_f] = fourier(t, gx_proc, 'sinus');
X_f = 20*log10(X_f);

plot(f_imu, X_f)
xlabel('Frequência [Hz]')
ylabel('Amplitude [dB]')
title('FFT simples - IMU - eixo X')
grid on


%% ============================================================
%  LEITURA DAS MENSAGENS BATCH - ISBH / ISBD
% ============================================================

isbhMsg = readMessages(rawdata, MessageName="ISBH");
isbhData = isbhMsg.MsgData{1};

isbdMsg = readMessages(rawdata, MessageName="ISBD");
isbdData = isbdMsg.MsgData{1};

disp("Variáveis ISBH:")
disp(isbhData.Properties.VariableNames)

disp("Variáveis ISBD:")
disp(isbdData.Properties.VariableNames)


%% ============================================================
%  SELEÇÃO DO SENSOR
%  type == 1      -> giroscópio
%  instance == 0  -> IMU 0
% ============================================================

idx_hdr = (isbhData.type == 1) & (isbhData.instance == 0);
hdr_sel = isbhData(idx_hdr, :);

fprintf('\nNúmero de batches selecionados: %d\n', height(hdr_sel));

if isempty(hdr_sel)
    error("Nenhum batch encontrado para type = 1 e instance = 0.");
end


%% ============================================================
%  PARÂMETROS PARA FFT BATCH
% ============================================================

% Primeiro usa tudo. Depois você ajusta.
t_inicio = 0;
t_final  = inf;

window_size = 1024;     % potência de 2
overlap = 0.5;          % 50%
window_step = round(window_size * (1 - overlap));

win = hann(window_size, "periodic");


%% ============================================================
%  RECONSTRUÇÃO DOS BATCHES ISBH/ISBD
% ============================================================

batches = struct([]);
batch_count = 0;

for i = 1:height(hdr_sel)

    N_atual = hdr_sel.N(i);

    % Seleciona os dados ISBD correspondentes ao batch atual
    dados_batch = isbdData(isbdData.N == N_atual, :);

    if isempty(dados_batch)
        continue
    end

    % Ordena pela sequência do batch
    dados_batch = sortrows(dados_batch, "seqno");

    % Reconstrói os vetores dos três eixos
    x = reshape(dados_batch.x.', [], 1);
    y = reshape(dados_batch.y.', [], 1);
    z = reshape(dados_batch.z.', [], 1);

    x = double(x);
    y = double(y);
    z = double(z);

    % Aplica fator de escala do batch
    if ismember("mul", hdr_sel.Properties.VariableNames)
        fator_escala = 1 / double(hdr_sel.mul(i));

        x = x * fator_escala;
        y = y * fator_escala;
        z = z * fator_escala;
    end

    % Frequência de amostragem do batch
    Fs_i = double(hdr_sel.smp_rate(i));

    % Tempo relativo do batch
    % No seu caso, TimeUS está vindo como duration.
    if isduration(hdr_sel.TimeUS)
        t0_i = seconds(hdr_sel.TimeUS(i) - hdr_sel.TimeUS(1));
    else
        t0_i = double(hdr_sel.TimeUS(i) - hdr_sel.TimeUS(1)) * 1e-6;
    end

    batch_count = batch_count + 1;

    batches(batch_count).N  = N_atual;
    batches(batch_count).Fs = Fs_i;
    batches(batch_count).t0 = t0_i;
    batches(batch_count).x  = x;
    batches(batch_count).y  = y;
    batches(batch_count).z  = z;

end

fprintf('Batches reconstruídos: %d\n', batch_count);

if batch_count == 0
    error("Nenhum batch foi reconstruído. Verifique ISBH/ISBD.");
end


%% ============================================================
%  INFORMAÇÕES DE AMOSTRAGEM
% ============================================================

Fs_values = [batches.Fs];
Fs = mean(Fs_values);

fprintf('\nFs médio = %.2f Hz\n', Fs);
fprintf('Nyquist = %.2f Hz\n', Fs/2);
fprintf('Window size = %d amostras\n', window_size);
fprintf('Resolução em frequência = %.2f Hz\n', Fs/window_size);


%% ============================================================
%  CONCATENAÇÃO DOS BATCHES PARA VISUALIZAÇÃO NO TEMPO
% ============================================================

x_all = [];
y_all = [];
z_all = [];
t_all = [];

for i = 1:length(batches)

    Fs_i = batches(i).Fs;
    n_i = length(batches(i).x);

    t_i = batches(i).t0 + (0:n_i-1)' / Fs_i;

    x_all = [x_all; batches(i).x];
    y_all = [y_all; batches(i).y];
    z_all = [z_all; batches(i).z];
    t_all = [t_all; t_i];

end

idx_tempo = (t_all >= t_inicio) & (t_all <= t_final);

fprintf('\nTempo reconstruído dos batches:\n');
fprintf('t_all mínimo = %.3f s\n', min(t_all));
fprintf('t_all máximo = %.3f s\n', max(t_all));
fprintf('Pontos no intervalo = %d\n', sum(idx_tempo));


%% ============================================================
%  PLOT DOS DADOS BATCH RECONSTRUÍDOS
% ============================================================

figure(4)
sgtitle('Dados brutos do giroscópio - ISBH/ISBD reconstruído')

subplot(3,1,1)
plot(t_all(idx_tempo), x_all(idx_tempo))
title('Eixo X')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

subplot(3,1,2)
plot(t_all(idx_tempo), y_all(idx_tempo))
title('Eixo Y')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on

subplot(3,1,3)
plot(t_all(idx_tempo), z_all(idx_tempo))
title('Eixo Z')
xlabel('Tempo [s]')
ylabel('Velocidade angular [rad/s]')
grid on


%% ============================================================
%  FFT JANELADA DOS BATCHES
% ============================================================

f = Fs * (0:window_size/2)' / window_size;

spec_x = [];
spec_y = [];
spec_z = [];

for i = 1:length(batches)

    Fs_i = batches(i).Fs;

    x = batches(i).x;
    y = batches(i).y;
    z = batches(i).z;

    n = length(x);

    if n < window_size
        continue
    end

    inicio_janela = 1;

    while (inicio_janela + window_size - 1) <= n

        fim_janela = inicio_janela + window_size - 1;

        centro_amostra = inicio_janela + window_size/2;
        t_centro = batches(i).t0 + (centro_amostra - 1) / Fs_i;

        if (t_centro >= t_inicio) && (t_centro <= t_final)

            xw = x(inicio_janela:fim_janela);
            yw = y(inicio_janela:fim_janela);
            zw = z(inicio_janela:fim_janela);

            % Remove tendência
            xw = detrend(xw);
            yw = detrend(yw);
            zw = detrend(zw);

            % Aplica janela Hann
            xw = xw .* win;
            yw = yw .* win;
            zw = zw .* win;

            % FFT
            X = fft(xw, window_size);
            Y = fft(yw, window_size);
            Z = fft(zw, window_size);

            % Espectro unilateral
            PX = abs(X / window_size);
            PY = abs(Y / window_size);
            PZ = abs(Z / window_size);

            PX = PX(1:window_size/2+1);
            PY = PY(1:window_size/2+1);
            PZ = PZ(1:window_size/2+1);

            PX(2:end-1) = 2 * PX(2:end-1);
            PY(2:end-1) = 2 * PY(2:end-1);
            PZ(2:end-1) = 2 * PZ(2:end-1);

            % Correção aproximada da janela Hann
            cg = mean(win);

            PX = PX / cg;
            PY = PY / cg;
            PZ = PZ / cg;

            spec_x = [spec_x, PX];
            spec_y = [spec_y, PY];
            spec_z = [spec_z, PZ];

        end

        inicio_janela = inicio_janela + window_step;

    end

end

if isempty(spec_x)
    error("Nenhuma janela FFT válida foi encontrada. Verifique t_inicio, t_final e window_size.");
end

fprintf('\nNúmero de janelas FFT usadas: %d\n', size(spec_x, 2));


%% ============================================================
%  MÉDIA DOS ESPECTROS
% ============================================================

Pmean_x = mean(spec_x, 2);
Pmean_y = mean(spec_y, 2);
Pmean_z = mean(spec_z, 2);

eps_val = 1e-12;

Pmean_x_dB = 20 * log10(Pmean_x + eps_val);
Pmean_y_dB = 20 * log10(Pmean_y + eps_val);
Pmean_z_dB = 20 * log10(Pmean_z + eps_val);


%% ============================================================
%  PLOT FFT BATCH - EIXOS X, Y, Z
% ============================================================

figure(5)
sgtitle('FFT janelada - Gyro batch - ISBH/ISBD')

subplot(3,1,1)
plot(f, Pmean_x_dB, 'LineWidth', 1.2)
grid on
xlim([0 Fs/2])
ylabel('X [dB]')
title('Eixo X')

subplot(3,1,2)
plot(f, Pmean_y_dB, 'LineWidth', 1.2)
grid on
xlim([0 Fs/2])
ylabel('Y [dB]')
title('Eixo Y')

subplot(3,1,3)
plot(f, Pmean_z_dB, 'LineWidth', 1.2)
grid on
xlim([0 Fs/2])
ylabel('Z [dB]')
xlabel('Frequência [Hz]')
title('Eixo Z')


%% ============================================================
%  PLOT FFT BATCH - APENAS EIXO X
% ============================================================

figure(6)
plot(f, Pmean_x_dB, 'LineWidth', 1.2)
grid on
xlim([0 Fs/2])
xlabel('Frequência [Hz]')
ylabel('Amplitude [dB]')
title('FFT janelada - Gyro batch - eixo X')