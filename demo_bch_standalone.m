function demo_bch_standalone_fixed
    clear; clc; close all;
    
    %% 1. THIẾT LẬP THÔNG SỐ (CONFIGURATION)
    % Cấu hình mã BCH (n, k)
    n = 127; 
    k = 113;
    
    % Khởi tạo đối tượng Mã hóa và Giải mã
    bchEncoder = comm.BCHEncoder(n, k);
    bchDecoder = comm.BCHDecoder(n, k);
    
    % Hiển thị thông tin
    fprintf('=== CẤU HÌNH HỆ THỐNG BCH ===\n');
    fprintf('Mã BCH(%d, %d)\n', n, k);
    
    % Cấu hình Điều chế
    M = 16; % 16-QAM
    bitsPerSym = log2(M); % 4 bit/symbol
    
    %% 2. CHUẨN BỊ MÔ PHỎNG
    numFrames = 10000; 
    totalBits = k * numFrames;
    
    % Dữ liệu ngẫu nhiên (Vector cột dài)
    data = randi([0 1], totalBits, 1);
    
    % Dải SNR
    EbN0_range = 0:1:20; 
    
    BER_uncoded = zeros(size(EbN0_range)); 
    BER_coded   = zeros(size(EbN0_range)); 
    
    fprintf('\nĐang chạy mô phỏng... (Tổng %d bits)\n', totalBits);
    
    %% 3. VÒNG LẶP MÔ PHỎNG
    for i = 1:length(EbN0_range)
        EbN0 = EbN0_range(i);
        
        % --- A. TÍNH TOÁN NHIỄU ---
        codeRate = k / n;
        snr_db = EbN0 + 10*log10(codeRate * bitsPerSym);
        
        % --- B. QUÁ TRÌNH PHÁT ---
        
        % 1. Mã hóa BCH
        % [FIX]: Không reshape thành ma trận. 
        % Đưa trực tiếp vector dài vào, Encoder tự biết cắt theo k=113
        tx_bits_bch = step(bchEncoder, data); 
        
        % 2. Padding cho 16-QAM
        num_tx_bits = length(tx_bits_bch);
        pad_len = mod(bitsPerSym - mod(num_tx_bits, bitsPerSym), bitsPerSym);
        if pad_len == bitsPerSym, pad_len = 0; end
        
        tx_bits_padded = [tx_bits_bch; zeros(pad_len, 1)];
        
        % 3. Điều chế 16-QAM
        tx_sym = qammod(tx_bits_padded, M, 'InputType', 'bit', 'UnitAveragePower', true);
        
        % --- C. KÊNH TRUYỀN ---
        rx_sym = awgn(tx_sym, snr_db, 'measured');
        
        % --- D. QUÁ TRÌNH THU ---
        
        % 1. Giải điều chế
        rx_bits_padded = qamdemod(rx_sym, M, 'OutputType', 'bit', 'UnitAveragePower', true);
        
        % 2. Bỏ Padding
        rx_bits_bch = rx_bits_padded(1:end-pad_len);
        
        % 3. Giải mã BCH
        % [FIX]: Đưa trực tiếp vector dài vào Decoder
        % Input phải là double
        rx_data = step(bchDecoder, double(rx_bits_bch));
        
        % --- E. TÍNH TOÁN LỖI ---
        
        % Tính BER Code
        [~, ratio_coded] = biterr(data, rx_data);
        BER_coded(i) = ratio_coded;
        
        % Tính BER Lý thuyết Uncoded
        BER_uncoded(i) = berawgn(EbN0, 'qam', M);
        
        fprintf('Eb/N0 = %2d dB | BER (BCH) = %.2e\n', EbN0, ratio_coded);
    end
    
    %% 4. VẼ ĐỒ THỊ
    figure('Name', 'BCH Performance', 'Color', 'w');
    semilogy(EbN0_range, BER_uncoded, 'k--', 'LineWidth', 1.5); hold on;
    semilogy(EbN0_range, BER_coded, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
    grid on;
    xlabel('Eb/N0 (dB)'); ylabel('BER');
    title(['Hiệu năng mã BCH(', num2str(n), ',', num2str(k), ') với 16-QAM']);
    legend('Uncoded 16-QAM (Theory)', 'Coded BCH (Simulated)');
    axis([min(EbN0_range) max(EbN0_range) 1e-6 1]);
end