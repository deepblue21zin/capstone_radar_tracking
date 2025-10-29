% ===== 사용자 설정 =====
CLI = "COM13";                 % CLI 포트
CLI_BAUD = 115200;
DATA_BAUD = 921600;           % OOB 기본
CFG_FILE = "8f4b6278-77b3-4d1f-b4db-7e8a93ecd697.cfg"; % 위 cfg 파일

% 1) CLI 열기 & 초기화
cli = serialport(CLI, CLI_BAUD, "Timeout", 2);
configureTerminator(cli, "LF");
writeline(cli, "sensorStop"); pause(0.1);
writeline(cli, "flushCfg");   pause(0.1);

% 2) cfg 전송 (라인별, 빈줄/주석 무시)
fid = fopen(CFG_FILE, "r"); assert(fid>0, "cfg 열기 실패");
C = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
fclose(fid);
lines = string(C{1});
for ln = lines'
    s = strtrim(ln);
    if s=="" || startsWith(s,"%"), continue; end
    writeline(cli, s);
    pause(startsWith(s,"sensorStart")*0.4 + 0.02);
end

% 3) CLI 응답 비우기 (선택)
t0 = tic;
while toc(t0) < 1.0
    if cli.NumBytesAvailable > 0
        disp("CLI says: " + readline(cli));
    else
        pause(0.05);
    end
end

% 4) DATA 포트 자동 탐색(바이트 단위로 확인)
allPorts = serialportlist("available");
cand = setdiff(allPorts, CLI);
fprintf("스캔 대상: %s\n", strjoin(cand, ", "));

found = false; dataPort = ""; totalBytes = 0;
for p = cand
    try
        sp = serialport(p, DATA_BAUD, "Timeout", 0.1);
        flush(sp);
        t0 = tic; bytesSeen = 0;
        while toc(t0) < 5
            n = sp.NumBytesAvailable;
            if n > 0
                bytes = read(sp, n, "uint8");    % <-- readline 금지
                bytesSeen = bytesSeen + numel(bytes);
                if bytesSeen > 200
                    found = true; dataPort = p; totalBytes = bytesSeen;
                    break;
                end
            end
            pause(0.05);
        end
        clear sp
    catch
        % pass
    end
    if found, break; end
end

if ~found
    error("DATA 0바이트: 포트/baud/SOP/펌웨어/센서Start 재확인 필요");
else
    fprintf("확정 DATA 포트: %s (총 %d bytes 관측)\n", dataPort, totalBytes);
end

clear cli
