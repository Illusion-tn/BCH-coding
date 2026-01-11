function main_bch_16qam
    clear; clc; close all;
    rng(42); 

    %% 1. КОНФИГУРАЦИЯ СИСТЕМЫ (Cấu hình hệ thống)
    n = 127; k = 113; t = 2; 
    bchEncoder = comm.BCHEncoder(n, k);
    bchDecoder = comm.BCHDecoder(n, k);
    M = 16; bitsPerSym = log2(M);

    %% 2. КОНФИГУРАЦИЯ ПЕРЕМЕЖЕНИЯ (Matrix Interleaver)
    DEPTH = 100;     % Глубина перемежения (Depth)
    BURST_LEN = 50;  % Длина пакета ошибок (Burst Length)
    
    Total_Frames = 10000; 
    Total_Frames = ceil(Total_Frames/DEPTH) * DEPTH; 
    
    SNR_Range = 0:1:16; 

    Total_Bits_Info = k * Total_Frames;
    num_super_blocks = Total_Frames / DEPTH;

    %% 3. МОДЕЛИРОВАНИЕ (Simulation)
    fprintf('Запуск симуляции для построения графика...\n');
    
    data = randi([0 1], Total_Bits_Info, 1);
    BER_NoInt = zeros(size(SNR_Range));
    BER_WithInt = zeros(size(SNR_Range));

    for i = 1:numel(SNR_Range)
        EbN0 = SNR_Range(i);
        codeRate = k/n;
        snr_db = EbN0 + 10*log10(codeRate * bitsPerSym);

        err_no = 0; err_with = 0;

        for g = 1:num_super_blocks
            % Кодирование (Encoding)
            idx_s = (g-1)*DEPTH*k + 1; idx_e = g*DEPTH*k;
            bits_group = double(data(idx_s:idx_e));
            encoded_group = step(bchEncoder, bits_group); 
            
            % Матричное перемежение (Matrix Interleaver)
            mat_orig = reshape(encoded_group, n, DEPTH);
            tx_bits_with_int = mat_orig.'; 
            tx_bits_with_int = tx_bits_with_int(:);
            tx_bits_no_int   = encoded_group;

            % Модуляция 16-QAM
            L = numel(encoded_group);
            pad = mod(bitsPerSym - mod(L, bitsPerSym), bitsPerSym);
            if pad == bitsPerSym, pad = 0; end
            
            tx_sym_1 = qammod(double([tx_bits_no_int; zeros(pad,1)]), M, 'InputType','bit', 'UnitAveragePower',true);
            tx_sym_2 = qammod(double([tx_bits_with_int; zeros(pad,1)]), M, 'InputType','bit', 'UnitAveragePower',true);

            % Канал AWGN
            rx_sym_1 = awgn(tx_sym_1, snr_db, 'measured');
            rx_sym_2 = awgn(tx_sym_2, snr_db, 'measured');

            % Демодуляция
            b1 = qamdemod(rx_sym_1, M, 'OutputType','bit', 'UnitAveragePower',true);
            b2 = qamdemod(rx_sym_2, M, 'OutputType','bit', 'UnitAveragePower',true);
            
            r1 = logical(reshape(b1.', [], 1)); r1(end-pad+1:end)=[];
            r2 = logical(reshape(b2.', [], 1)); r2(end-pad+1:end)=[];

            % Внесение пакетных ошибок (Burst Error Injection)
            err_vec = false(L, 1);
            s_pos = randi([1, L - BURST_LEN]);
            err_vec(s_pos : s_pos + BURST_LEN - 1) = true;

            r1 = xor(r1, err_vec);
            r2 = xor(r2, err_vec);

            % Декодирование (Decoding)
            dec1 = step(bchDecoder, double(r1));
            err_no = err_no + sum(bits_group ~= dec1);

            mat_rx = reshape(r2, DEPTH, n);
            mat_deint = mat_rx.';
            dec2 = step(bchDecoder, double(mat_deint(:)));
            err_with = err_with + sum(bits_group ~= dec2);
        end

        BER_NoInt(i) = err_no / Total_Bits_Info;
        BER_WithInt(i) = err_with / Total_Bits_Info;
        
        if BER_WithInt(i) == 0, BER_WithInt(i) = 1e-10; end 
        
        % In kết quả ra console bằng tiếng Nga
        fprintf('Eb/N0=%2d дБ | BER Без_Перемежения: %.2e | BER С_Перемежением: %.2e\n', ...
            EbN0, BER_NoInt(i), BER_WithInt(i));
    end

    %% 4. ВИЗУАЛИЗАЦИЯ (Visualization in Russian)
    
    % Цветовая палитра (Màu sắc)
    cNoInt   = [0.8500 0.3250 0.0980]; % Оранжевый (Cam)
    cWithInt = [0.0000 0.4470 0.7410]; % Синий (Xanh)
    cGrid    = [0.4 0.4 0.4];

    figure('Units', 'pixels', 'Position', [100 100 1000 700], 'Color', 'w', 'Name', 'Russian Plot');
    
    % --- График 1: Без перемежения (No Interleaver) ---
    semilogy(SNR_Range, BER_NoInt, '--s', ...
        'LineWidth', 2.5, ...
        'MarkerSize', 10, ...
        'Color', cNoInt, ...
        'MarkerFaceColor', 'w', ... 
        'MarkerEdgeColor', cNoInt, ...
        'DisplayName', 'Без перемежения (Плато ошибок)'); 
    hold on;
    
    % --- График 2: С матричным перемежением (With Matrix Interleaver) ---
    semilogy(SNR_Range, BER_WithInt, '-^', ...
        'LineWidth', 2.5, ...
        'MarkerSize', 10, ...
        'Color', cWithInt, ...
        'MarkerFaceColor', cWithInt, ...
        'MarkerEdgeColor', cWithInt, ...
        'DisplayName', 'Матричное перемежение (Водопад)');

    % --- Настройка осей (Axes Settings) ---
    ax = gca;
    ax.FontSize = 16;           
    ax.LineWidth = 1.5;         
    ax.FontName = 'Arial';      % Arial hỗ trợ ký tự Cyrillic tốt
    ax.GridColor = cGrid;
    ax.GridAlpha = 0.3;         
    ax.MinorGridAlpha = 0.15;   
    
    grid on; 
    set(gca, 'YMinorGrid','on', 'XMinorGrid','on'); 

    % --- Подписи осей (Labels) ---
    % Eb/N0 (дБ)
    xlabel('Eb/N0 (дБ)', 'FontSize', 18, 'FontWeight', 'bold');
    % Вероятность битовой ошибки (BER)
    ylabel('Вероятность ошибки на бит (BER)', 'FontSize', 18, 'FontWeight', 'bold');
    
    % --- Заголовок (Title) ---
    % Производительность системы BCH(127,113) + 16-QAM
    % Длина пакета = %d бит | Глубина перемежения = %d
    title({'Производительность системы BCH(127,113) + 16-QAM', ...
           sprintf('Длина пакета = %d бит | Глубина перемежения = %d', BURST_LEN, DEPTH)}, ...
           'FontSize', 20, 'FontWeight', 'normal');

    % --- Легенда (Legend) ---
    lgd = legend('Location', 'southwest');
    lgd.FontSize = 16;
    lgd.EdgeColor = [0.8 0.8 0.8]; 
    
    xlim([0 16]);
    ylim([1e-9 1]); 
    
    hold off;
end