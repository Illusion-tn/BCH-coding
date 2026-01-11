function main_concatenated_system_final
% MAIN_CONCATENATED_SYSTEM_FINAL
% Phiên bản sửa lỗi kích thước khung LDPC (Normal vs Short).
% Hệ thống: [Input] -> BCH -> LDPC -> Matrix Interleaver -> QAM -> Channel

    clear; clc; close all;
    rng(42); 

    %% 1. CẤU HÌNH HỆ THỐNG
    
    % --- A. BCH (OUTER CODE) ---
    n_bch = 127; k_bch = 113; 
    bchEncoder = comm.BCHEncoder(n_bch, k_bch);
    bchDecoder = comm.BCHDecoder(n_bch, k_bch);

    % --- B. LDPC (INNER CODE) ---
    % [FIX QUAN TRỌNG]: Thêm tham số 'short' để khớp với K=7200
    ldpc_cfg = dvbs2ldpc(1/2, 'short'); 
    
    ldpcEncoder = comm.LDPCEncoder(ldpc_cfg);
    ldpcDecoder = comm.LDPCDecoder(ldpc_cfg, 'MaximumIterationCount', 20);
    
    % Kích thước chuẩn của DVB-S2 Short Frame
    K_ldpc = 7200;  
    N_ldpc = 16200; 

    % --- C. INTERLEAVER ---
    DEPTH = 4; % Gom 4 khung LDPC
    
    % --- D. KÊNH ---
    M = 16; bitsPerSym = log2(M);
    BURST_LEN = 200; % Burst rất mạnh
    
    Total_LDPC_Frames = 100; 
    Total_LDPC_Frames = ceil(Total_LDPC_Frames/DEPTH) * DEPTH;
    
    SNR_Range = 0:1:10; 

    %% 2. TÍNH TOÁN RATE MATCHING
    % Số lượng từ mã BCH nhét vừa 1 khung LDPC
    Num_BCH_in_LDPC = floor(K_ldpc / n_bch); % = 56
    Bits_Use_Per_LDPC = Num_BCH_in_LDPC * n_bch; % = 7112 bit
    % Padding cho LDPC (Phần dư không đủ nhét 1 từ mã BCH)
    % Pad_LDPC = K_ldpc - Bits_Use_Per_LDPC; 
    
    % Tổng bit thông tin
    Total_Info_Bits = Total_LDPC_Frames * Num_BCH_in_LDPC * k_bch;
    
    %% 3. MÔ PHỎNG
    fprintf('=== HỆ THỐNG GHÉP KÊNH (DVB-S2 STYLE) ===\n');
    fprintf('BCH(%d,%d) -> LDPC(1/2, Short) -> Matrix Int (Depth=%d)\n', n_bch, k_bch, DEPTH);
    fprintf('K_ldpc = %d, N_ldpc = %d\n', K_ldpc, N_ldpc);
    
    data = randi([0 1], Total_Info_Bits, 1);
    
    BER_Final = zeros(size(SNR_Range));

    for i = 1:numel(SNR_Range)
        EbN0 = SNR_Range(i);
        % Code rate tổng
        totalRate = (k_bch/n_bch) * (K_ldpc/N_ldpc);
        snr_db = EbN0 + 10*log10(totalRate * bitsPerSym);
        noiseVar = 10^(-snr_db/10);

        err_final = 0;
        
        num_super_blocks = Total_LDPC_Frames / DEPTH;
        
        for g = 1:num_super_blocks
            % Lấy dữ liệu nguồn cho nhóm này
            bits_per_sb = DEPTH * Num_BCH_in_LDPC * k_bch;
            idx_s = (g-1)*bits_per_sb + 1; idx_e = g*bits_per_sb;
            sb_data = data(idx_s:idx_e); % Vector cột
            
            % --- 1. MÃ HÓA BCH ---
            % Đưa vector dài vào, Encoder tự cắt thành từng khối k_bch
            bch_out_stream = step(bchEncoder, sb_data); 
            
            % --- 2. CHUẨN BỊ INPUT CHO LDPC ---
            ldpc_input_mat = zeros(K_ldpc, DEPTH);
            
            ptr = 1;
            for f = 1:DEPTH
                % Lấy chunk dữ liệu BCH đủ cho 1 khung LDPC
                chunk = bch_out_stream(ptr : ptr + Bits_Use_Per_LDPC - 1);
                
                % Nhét vào ma trận đầu vào LDPC
                % Phần dư (từ 7113 đến 7200) tự động là 0 (Padding)
                ldpc_input_mat(1:Bits_Use_Per_LDPC, f) = chunk;
                
                ptr = ptr + Bits_Use_Per_LDPC;
            end
            
            % --- 3. MÃ HÓA LDPC ---
            ldpc_coded_mat = zeros(N_ldpc, DEPTH);
            for f = 1:DEPTH
                % Input phải là vector cột đúng kích thước K_ldpc (7200)
                % Nhờ fix 'short', giờ nó đã khớp!
                ldpc_coded_mat(:, f) = step(ldpcEncoder, ldpc_input_mat(:, f));
            end
            
            % --- 4. MATRIX INTERLEAVER ---
            mat_int = ldpc_coded_mat.'; 
            tx_bits = mat_int(:); 
            
            % --- 5. CHANNEL ---
            tx_sym = qammod(double(tx_bits), M, 'InputType','bit', 'UnitAveragePower',true);
            rx_sym = awgn(tx_sym, snr_db, 'measured');
            
            % Burst Noise (Cộng vào tín hiệu QAM)
            L_sym = length(rx_sym);
            burst_vec = zeros(L_sym, 1);
            b_start = randi([1, L_sym - ceil(BURST_LEN/bitsPerSym)]);
            % Nhiễu rất mạnh
            burst_vec(b_start : b_start + ceil(BURST_LEN/bitsPerSym)) = 10 + 10i; 
            rx_sym = rx_sym + burst_vec;
            
            % --- 6. RECEIVER ---
            rx_llr = qamdemod(rx_sym, M, 'OutputType','approxllr', ...
                'UnitAveragePower',true, 'NoiseVariance', noiseVar);
            
            % De-Interleaver
            rx_mat_deint_t = reshape(rx_llr, DEPTH, N_ldpc);
            rx_mat_deint = rx_mat_deint_t.'; 
            
            % Giải mã LDPC
            ldpc_decoded_stream = [];
            for f = 1:DEPTH
                llr_col = rx_mat_deint(:, f);
                decoded_bits = step(ldpcDecoder, llr_col);
                
                % Bỏ Padding của khung LDPC (Chỉ lấy phần chứa BCH)
                valid_bits = decoded_bits(1:Bits_Use_Per_LDPC);
                ldpc_decoded_stream = [ldpc_decoded_stream; valid_bits];
            end
            
            % Giải mã BCH (Dọn dẹp lỗi sót)
            % Input vào decoder phải là double
            final_bits = step(bchDecoder, double(ldpc_decoded_stream));
            
            err_final = err_final + sum(sb_data ~= logical(final_bits));
        end

        BER_Final(i) = err_final / Total_Info_Bits;
        if BER_Final(i) == 0, BER_Final(i) = 1e-8; end
        
        fprintf('Eb/N0=%2d dB | BER Final: %.2e\n', EbN0, BER_Final(i));
    end

    %% VẼ ĐỒ THỊ
    figure('Color','w');
    semilogy(SNR_Range, BER_Final, '-^', 'LineWidth', 3, 'MarkerSize', 10, ...
        'Color', [0 0.5 0], 'MarkerFaceColor', [0 0.5 0]);
    grid on;
    xlabel('Eb/N0 (dB)'); ylabel('BER');
    title({'Hệ thống ghép kênh (Concatenated Code)'; 'BCH + LDPC + Matrix Interleaver'});
    legend('Full System');
    ylim([1e-8 1]);
end