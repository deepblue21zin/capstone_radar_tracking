%% RealTimePeopleTracking_SensorTracker.m
% IWR6843 Detection + MATLAB Tracker
% CFG에서 정적 클러터 제거를 제어(코드에서는 설정 안 함)
clc; clear; close all;

%% 1) CFG 파일 경로
cfgFile = fullfile(pwd, 'tracking_config.cfg');
if ~exist(cfgFile,'file')
    error('CFG 파일을 찾을 수 없습니다: %s\n올바른 CFG 파일을 다운로드하세요!', cfgFile);
end

fprintf('\n╔════════════════════════════════════════════╗\n');
fprintf('║   CFG 파일 로드 중...                     ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');
fprintf('ℹ️  MATLAB 호환 CFG 파일 사용 (경고 없음)\n\n');

%% 2) mmWaveRadar 객체 생성
% 장치 관리자에서 확인한 실제 포트로 변경하세요.
configPort = 'COM13';
dataPort   = 'COM14';

try
    % RemoveStaticClutter 인자 제거 (CFG와 충돌 방지)
    rdr = mmWaveRadar("TI IWR6843ISK", ...
        'ConfigPort', configPort, ...
        'DataPort',   dataPort, ...
        ConfigFile = cfgFile, ...
        ReadMode = "latest", ...
        DetectionCoordinates = "Sensor rectangular", ...
        EnableDopplerGroups = true, ...
        EnableRangeGroups   = false);

    fprintf('✅ 센서 초기화 완료 (Detection Mode)\n\n');
catch ME
    fprintf(2, '\n❌ 센서 초기화 실패: %s\n', ME.message);
    fprintf(2, 'ℹ️  다음을 확인하세요:\n');
    fprintf(2, '    1) xwr68xx_mmw_demo 펌웨어 플래시\n');
    fprintf(2, '    2) ConfigPort=%s / DataPort=%s 매핑 확인(작은 번호=Config)\n', configPort, dataPort);
    fprintf(2, '    3) 포트 점유 프로그램(mmWave Studio/터미널) 종료\n');
    fprintf(2, '    4) Functional Mode 점퍼 + 전원 재인가\n');
    return;
end

%% 3) 필터링 파라미터
minSpeedThreshold = 0.05;  % m/s (낮춰서 느린 움직임도 감지)
maxRange          = 12;    % m

%% 3.5) MATLAB 트래커 초기화 - 트랙 분리 방지 최적화
tracker = trackerJPDA('FilterInitializationFcn', @initCustomFilter, ...
    'AssignmentThreshold', [30 inf], ...      % 200 → 30: 같은 객체를 더 잘 연결
    'ConfirmationThreshold', [3 5], ...       % [2 3] → [3 5]: 새 트랙 생성 어렵게
    'DeletionThreshold', [8 10], ...          % [3 3] → [8 10]: 트랙 유지 시간 증가
    'MaxNumTracks', 20, ...
    'OOSMHandling', 'Neglect');

isTrackerInitialized = false;
fprintf('✅ MATLAB 트래커 초기화 완료 (트랙 분리 방지 모드)\n\n');

% 커스텀 필터 초기화 함수 (프로세스 노이즈 조정)
function filter = initCustomFilter(detection)
    filter = initcvekf(detection);
    % 프로세스 노이즈 감소 → 급격한 움직임 변화 억제 → 트랙 안정성 증가
    filter.ProcessNoise = filter.ProcessNoise * 0.3;  % 30%로 감소
    % 측정 노이즈는 유지 (레이더 정확도 반영)
end

%% 4) 시각화 설정
stopTime = 90;
maxTrajectoryLength = 100;
trackHistory = containers.Map('KeyType', 'double', 'ValueType', 'any');

% 선명한 색상 팔레트
colorPalette = [
    0.0, 1.0, 1.0;  % Cyan
    1.0, 0.0, 1.0;  % Magenta
    1.0, 1.0, 0.0;  % Yellow
    0.0, 1.0, 0.0;  % Green
    1.0, 0.5, 0.0;  % Orange
    0.5, 0.0, 1.0;  % Purple
    1.0, 0.0, 0.0;  % Red
    0.0, 0.5, 1.0;  % Sky Blue
];

%% 5) Figure 생성
fig = figure('Name','IWR6843 MATLAB Tracker (People Tracking)', ...
    'Position', [100 100 1400 900], ...
    'Color', [0.05 0.05 0.15], ...
    'Renderer', 'opengl');

ax = axes('Parent', fig);
axis(ax, 'equal'); grid(ax, 'on');
set(ax, 'Color', [0.08 0.08 0.18], ...
    'GridColor', [0.2 0.2 0.3], ...
    'GridAlpha', 0.6, ...
    'LineWidth', 1.5);
xlabel(ax, 'X Distance (m)', 'FontSize', 14, 'Color', 'w', 'FontWeight', 'bold');
ylabel(ax, 'Y Distance (m)', 'FontSize', 14, 'Color', 'w', 'FontWeight', 'bold');
set(ax, 'XColor', 'w', 'YColor', 'w', 'FontSize', 12);
xlim(ax, [-5 5]); ylim(ax, [0 10]);
hold(ax, 'on');

% 레이더 위치 표시
plot(ax, 0, 0, 'p', 'MarkerSize', 20, 'MarkerFaceColor', [1 0.3 0.3], ...
    'MarkerEdgeColor', 'w', 'LineWidth', 2);
text(ax, 0, -0.5, 'RADAR', 'Color', 'w', 'FontSize', 10, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');

% 거리 원 (전방=Y+ 가정)
for r = [3, 6, 9]
    theta = linspace(0, pi, 100);
    plot(ax, r*sin(theta), r*cos(theta), ':', 'Color', [0.3 0.3 0.4], 'LineWidth', 1);
end

% Boundary Box (3D) - 표시좌표계 기준(Y 전방)
xMin_bb = -2; xMax_bb =  2;
yMin_bb =  0.5; yMax_bb = 8;
zMin_bb =  0;  zMax_bb = 2.5;

boxVertices = [
    xMin_bb, yMin_bb, zMin_bb;
    xMax_bb, yMin_bb, zMin_bb;
    xMax_bb, yMax_bb, zMin_bb;
    xMin_bb, yMax_bb, zMin_bb;
    xMin_bb, yMin_bb, zMax_bb;
    xMax_bb, yMin_bb, zMax_bb;
    xMax_bb, yMax_bb, zMax_bb;
    xMin_bb, yMax_bb, zMax_bb
];
boxFaces = [
    1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8
];
patch(ax, 'Vertices', boxVertices, 'Faces', boxFaces, ...
      'FaceColor', [0.5 0 0.5], 'FaceAlpha', 0.05, ...
      'EdgeColor', [0.5 0 0.5], 'LineWidth', 1.5, ...
      'DisplayName', 'Tracking Boundary');

% 타이틀/뷰
title_obj = title(ax, 'System Initializing...', ...
    'FontSize', 16, 'Color', [0.3 1 1], 'FontWeight', 'bold');
view(ax, 3); zlim(ax, [0 5]);
ylabel(ax, 'Y Distance (m)', 'FontSize', 14, 'Color', 'w', 'FontWeight', 'bold');
zlabel(ax, 'Z Distance (m)', 'FontSize', 14, 'Color', 'w', 'FontWeight', 'bold');
set(ax, 'ZColor', 'w');

%% 5.5) === 표시 전용 좌표 회전(요 보정): 전방을 Y+로 강제 정렬 ===
% 센서가 X+를 전방으로 내보내더라도, 화면은 Y+ 전방으로 보이게 함
alpha_deg = 90;                    % 기본 90° (필요시 85~95°로 미세 보정)
ca = cosd(alpha_deg);  sa = sind(alpha_deg);
R2 = [ca -sa; sa ca];              % 2D 회전행렬 (x,y만 회전)

%% 6) 그래픽 객체 초기화
h_rawPoints = scatter3(ax, [], [], [], 30, [0.5 0.5 0.5], '.', 'MarkerEdgeAlpha', 0.3);
h_dynamicPoints = scatter3(ax, [], [], [], 60, [0 1 1], 'o', ...
    'LineWidth', 1.5, 'MarkerEdgeColor', [0 1 1], 'MarkerFaceColor', 'none');

% 트랙 그래픽 저장용
trackBoxes = containers.Map('KeyType', 'double', 'ValueType', 'any');
trackTexts = containers.Map('KeyType', 'double', 'ValueType', 'any');
trackArrows = containers.Map('KeyType', 'double', 'ValueType', 'any');
trajectoryLines = containers.Map('KeyType', 'double', 'ValueType', 'any');

%% 7) 성능 모니터링
frameCount = 0;
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║   추적 시스템 시작                        ║\n');
fprintf('║   MATLAB 트래커 사용 중                   ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

%% 8) 메인 루프
tic; loopStartTime = tic; previousTrackerTime = 0;

while toc(loopStartTime) < stopTime
    % --- 데이터 읽기 ---
    [detections, ~] = rdr();
    time = toc(loopStartTime);

    % 시간 단조 증가 보장
    if time <= previousTrackerTime
        time = previousTrackerTime + 0.01;
    end

    % 빈 프레임 방어
    if isempty(detections)
        set(h_rawPoints, 'XData', [], 'YData', [], 'ZData', []);
        set(h_dynamicPoints, 'XData', [], 'YData', [], 'ZData', []);
        tracks = objectTrack.empty(0,1);
        pause(0.0001);
        continue;
    end

    % --- 전처리/필터 ---
    meas_sizes = cellfun(@(d) numel(d.Measurement), detections);
    x_all = cellfun(@(d) d.Measurement(1), detections);
    y_all = cellfun(@(d) d.Measurement(2), detections);
    z_all = cellfun(@(d) d.Measurement(3), detections);

    has_vel = meas_sizes >= 6;
    vel_magnitudes = zeros(size(detections));
    if any(has_vel)
        vel_magnitudes(has_vel) = cellfun(@(d) norm(d.Measurement(4:6)), detections(has_vel));
    end

    % 동적 판정: 속도 성분 있으면 임계치 적용, 없으면 true 유지
    isDynamic = true(size(detections));
    if any(has_vel)
        isDynamic = vel_magnitudes > minSpeedThreshold;
    end

    % 거리/높이 필터 (센서좌표 기준 거리)
    ranges = sqrt(x_all.^2 + y_all.^2 + z_all.^2);
    isDynamic = isDynamic & (ranges <= maxRange);
    isValidHeight = (z_all >= zMin_bb) & (z_all <= zMax_bb);
    isDynamic = isDynamic & isValidHeight;

    dynamicDetections = detections(isDynamic);

    % === (표시 전용) 센서좌표 -> 화면좌표 회전 ===
    xy = [x_all(:)'; y_all(:)'];         % 2×N
    xy_d = R2 * xy;                      % 회전된 표시좌표
    x_disp = reshape(xy_d(1,:), size(x_all));
    y_disp = reshape(xy_d(2,:), size(y_all));

    % 시각화(포인트) - 회전된 좌표 사용
    set(h_rawPoints,     'XData', x_disp, 'YData', y_disp, 'ZData', z_all);
    if any(isDynamic)
        set(h_dynamicPoints,'XData', x_disp(isDynamic), 'YData', y_disp(isDynamic), 'ZData', z_all(isDynamic));
    else
        set(h_dynamicPoints,'XData', [], 'YData', [], 'ZData', []);
    end

    % --- 트래커 호출 (센서측정 그대로 사용) ---
    if isTrackerInitialized
        tracks = tracker(dynamicDetections, time);
        previousTrackerTime = time;
        frameCount = frameCount + 1;
    else
        if ~isempty(dynamicDetections)
            tracks = tracker(dynamicDetections, time);
            isTrackerInitialized = true;
            previousTrackerTime = time;
            frameCount = frameCount + 1;
        else
            tracks = objectTrack.empty(0,1);
            pause(0.0001);
            continue;
        end
    end

    % --- 트랙 시각화 ---
    currentTrackIDs = [];
    if ~isempty(tracks), currentTrackIDs = [tracks.TrackID]; end

    % 오래된 그래픽 제거
    allTrackIDs_graphic = cell2mat(keys(trackBoxes));
    for tid = allTrackIDs_graphic
        if ~ismember(tid, currentTrackIDs)
            if isKey(trackBoxes, tid), delete(trackBoxes(tid)); remove(trackBoxes, tid); end
            if isKey(trackTexts, tid), delete(trackTexts(tid)); remove(trackTexts, tid); end
            if isKey(trackArrows, tid), delete(trackArrows(tid)); remove(trackArrows, tid); end
            if isKey(trajectoryLines, tid), delete(trajectoryLines(tid)); remove(trajectoryLines, tid); end
            if isKey(trackHistory, tid), remove(trackHistory, tid); end
        end
    end

    for i = 1:numel(tracks)
        t = tracks(i);
        trackID = t.TrackID;

        % --- 트랙 상태 (센서좌표) ---
        x_pos = t.State(1);
        y_pos = t.State(3);
        z_pos = t.State(5);

        if numel(t.State) >= 6
            vx = t.State(2); vy = t.State(4); vz = t.State(6);
        else
            vx = 0; vy = 0; vz = 0;
        end

        % --- (표시 전용) 트랙 위치/속도 회전: 전방=Y+ ---
        p_d = R2 * [x_pos; y_pos];
        x_draw = p_d(1);  y_draw = p_d(2);  z_draw = z_pos;

        v_d = R2 * [vx; vy];
        vx_draw = v_d(1); vy_draw = v_d(2); vz_draw = vz;

        speed = sqrt(vx_draw^2 + vy_draw^2 + vz_draw^2); % (회전 불변이지만 일관을 위해 draw값 사용)

        colorIdx = mod(trackID - 1, size(colorPalette, 1)) + 1;
        objColor = colorPalette(colorIdx, :);

        % 3D 박스 (표시좌표 기준)
        boxSize_xy = 0.6; boxHeight = 1.8;
        boxVertices_track = [
            x_draw - boxSize_xy/2, y_draw - boxSize_xy/2, z_draw - boxHeight/2;
            x_draw + boxSize_xy/2, y_draw - boxSize_xy/2, z_draw - boxHeight/2;
            x_draw + boxSize_xy/2, y_draw + boxSize_xy/2, z_draw - boxHeight/2;
            x_draw - boxSize_xy/2, y_draw + boxSize_xy/2, z_draw - boxHeight/2;
            x_draw - boxSize_xy/2, y_draw - boxSize_xy/2, z_draw + boxHeight/2;
            x_draw + boxSize_xy/2, y_draw - boxSize_xy/2, z_draw + boxHeight/2;
            x_draw + boxSize_xy/2, y_draw + boxSize_xy/2, z_draw + boxHeight/2;
            x_draw - boxSize_xy/2, y_draw + boxSize_xy/2, z_draw + boxHeight/2
        ];

        if isKey(trackBoxes, trackID)
            set(trackBoxes(trackID), 'Vertices', boxVertices_track);
        else
            hBox = patch(ax, 'Vertices', boxVertices_track, 'Faces', boxFaces, ...
                         'EdgeColor', objColor, 'LineWidth', 3, 'FaceAlpha', 0.1, 'FaceColor', objColor);
            trackBoxes(trackID) = hBox;
        end

        % ID 텍스트
        labelText = sprintf('ID: %d\n%.2f m/s', trackID, speed);
        if isKey(trackTexts, trackID)
            set(trackTexts(trackID), 'Position', [x_draw, y_draw + 0.4, z_draw + boxHeight/2 + 0.2], 'String', labelText);
        else
            hText = text(ax, x_draw, y_draw + 0.4, z_draw + boxHeight/2 + 0.2, labelText, ...
                'Color', objColor, 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', ...
                'BackgroundColor', [0.1 0.1 0.2], ...
                'EdgeColor', objColor, 'LineWidth', 1.5);
            trackTexts(trackID) = hText;
        end

        % 방향 화살표 (표시좌표 기준)
        if speed > 0.1
            arrowScale = 1.0;
            if isKey(trackArrows, trackID)
                delete(trackArrows(trackID)); remove(trackArrows, trackID);
            end
            hArrow = quiver3(ax, x_draw, y_draw, z_draw, vx_draw*arrowScale, vy_draw*arrowScale, vz_draw*arrowScale, 0, ...
                             'Color', objColor, 'LineWidth', 2.0);
            trackArrows(trackID) = hArrow;
        else
            if isKey(trackArrows, trackID), delete(trackArrows(trackID)); remove(trackArrows, trackID); end
        end

        % 궤적 저장/갱신 (표시좌표 기준)
        if isKey(trackHistory, trackID)
            history = trackHistory(trackID);
        else
            history = struct('x', [], 'y', [], 'z', [], 'time', [], 'color', objColor);
        end
        history.x(end+1) = x_draw;
        history.y(end+1) = y_draw;
        history.z(end+1) = z_draw;
        history.time(end+1) = time;

        if numel(history.x) > maxTrajectoryLength
            history.x = history.x(end-maxTrajectoryLength+1:end);
            history.y = history.y(end-maxTrajectoryLength+1:end);
            history.z = history.z(end-maxTrajectoryLength+1:end);
            history.time = history.time(end-maxTrajectoryLength+1:end);
        end
        trackHistory(trackID) = history;

        % 궤적 선
        if numel(history.x) > 1
            if isKey(trajectoryLines, trackID)
                set(trajectoryLines(trackID), 'XData', history.x, 'YData', history.y, 'ZData', history.z);
            else
                hTraj = plot3(ax, history.x, history.y, history.z, '--', 'Color', objColor, 'LineWidth', 2);
                trajectoryLines(trackID) = hTraj;
            end
        end

        % Boundary Box 판정 (표시좌표 기준)
        if ~(x_draw >= xMin_bb && x_draw <= xMax_bb && ...
             y_draw >= yMin_bb && y_draw <= yMax_bb && ...
             z_draw >= zMin_bb && z_draw <= zMax_bb)
            % out-of-bound 처리 hooks 가능
        end
    end

    % 오래된 궤적 정리 (10초 지난 트랙 히스토리 제거)
    if ~isempty(trackHistory)
        allHistoryIDs = cell2mat(keys(trackHistory));
        for tid = allHistoryIDs
            if ~ismember(tid, currentTrackIDs)
                if isKey(trackHistory, tid)
                    history = trackHistory(tid);
                    if ~isempty(history.time) && (time - history.time(end) > 10)
                        remove(trackHistory, tid);
                        if isKey(trajectoryLines, tid), delete(trajectoryLines, tid); remove(trajectoryLines, tid); end
                    end
                end
            end
        end
    end

    % 타이틀/로그
    if mod(frameCount, 5) == 0
        fps = frameCount / toc(loopStartTime);
        numDynamic_display = sum(isDynamic);
        numTracks_display = numel(tracks);
        title_obj.String = sprintf('⚡ %.1f FPS | Time: %.1fs | Raw: %d → Dynamic: %d → Tracks: %d', ...
            fps, time, numel(detections), numDynamic_display, numTracks_display);
    end

    drawnow;

    if mod(frameCount, 100) == 0
        fps = frameCount / toc(loopStartTime);
        fprintf('⏱️  %.1fs | 📡 %d frames | ⚡ %.1f FPS | 🎯 %d tracks\n', ...
            time, frameCount, fps, numel(tracks));
    end
end % while

%% 9) 최종 통계/정리
elapsedTime = toc(loopStartTime);
avgFPS = (frameCount>0) * (frameCount / elapsedTime);
totalTracks = length(keys(trackHistory));

fprintf('\n╔════════════════════════════════════════════╗\n');
fprintf('║          추적 완료!                       ║\n');
fprintf('╚════════════════════════════════════════════╝\n');
fprintf('📊 총 프레임: %d\n', frameCount);
fprintf('⚡ 평균 FPS: %.2f Hz\n', avgFPS);
fprintf('🎯 총 추적 객체: %d개\n', totalTracks);
fprintf('⏱️  총 실행 시간: %.1f초\n\n', elapsedTime);
fprintf('Δx_raw=%.2f, Δy_raw=%.2f (센서좌표 프레임 내 범위)\n', range(x_all), range(y_all));

title(ax, sprintf('✅ 완료 | %d개 객체 추적됨 | 평균 %.1f FPS', totalTracks, avgFPS), ...
    'FontSize', 16, 'Color', 'lime', 'FontWeight', 'bold');

try
    sensorStop(rdr);
catch
    warning('센서 정지 실패');
end
clear rdr;
