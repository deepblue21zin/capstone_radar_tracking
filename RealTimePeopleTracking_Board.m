%% RealTimePeopleTracking_Board.m
% IWR6843ISK 실시간 추적 (DetectionCoordinates = "sensor spherical")
clc; clear; close all;

%% 1. cfg 파일 경로 (파일명 변경했으면 여기 맞춰주세요)
cfgFile = fullfile(pwd, '8f4b6278-77b3-4d1f-b4db-7e8a93ecd697.cfg');
if ~exist(cfgFile,'file')
    error('CFG 파일을 찾을 수 없습니다: %s', cfgFile);
end

%% 2. mmWaveRadar 객체 생성 (sensor spherical 사용)
rdr = mmWaveRadar("TI IWR6843ISK", ...
    ConfigFile = cfgFile, ...
    ReadMode = "latest", ...
    DetectionCoordinates = "sensor spherical", ...  % <-- 수정된 부분
    EnableDopplerGroups = true, ...
    EnableRangeGroups = false, ...
    RemoveStaticClutter = true);

%% 3. 트래커 및 파라미터
minRangeRate = 0.2;   % 정적/동적 구분용
epsilon = 1; minNumPts = 2;   % DBSCAN

tracker = trackerJPDA(TrackLogic="Integrated");
tracker.FilterInitializationFcn = @initPeopleTrackingFilter;
azSpan = 60; rSpan = 25; dopplerSpan = 5; V = azSpan*rSpan*dopplerSpan;
tracker.ClutterDensity = 8/V;
tracker.NewTargetDensity = 0.01/V;
tracker.DetectionProbability = 0.85;
tracker.ConfirmationThreshold = 0.8;
tracker.DeletionThreshold = 1e-4;

%% 4. 실행 설정
stopTime = 90;   % 초 단위 (원하면 변경)
tracks = objectTrack.empty(0,1);

figure('Name','Real-Time People Tracking (spherical)');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
xlim([0 15]); ylim([-8 8]);
hold on;

trackHistories = containers.Map('KeyType','double','ValueType','any');

tic;
while toc < stopTime
    % 레이더에서 검출 읽기
    [detections, time] = rdr();
    if isempty(detections)
        drawnow; continue;
    end

    % (선택) 측정 노이즈 보정 — 필요하면 활성화
    for ii = 1:numel(detections)
        if size(detections{ii}.MeasurementNoise,1) >= 2
            detections{ii}.MeasurementNoise(1,1) = 9;    % azimuth var (deg^2)
            detections{ii}.MeasurementNoise(2,2) = 0.36; % range var (m^2)
        end
    end

    % 정적 반사 제거 (range-rate 기반)
    rr = cellfun(@(d) d.Measurement(3), detections); % range-rate
    isDynamic = abs(rr) > minRangeRate;
    detectionsDynamic = detections(isDynamic);

    % DBSCAN 파티셔닝 (spherical measurements)
    if isempty(detectionsDynamic)
        clusters = zeros(0,1,'uint32');
    else
        clusters = partitionDetections(detectionsDynamic, epsilon, minNumPts, 'Algorithm', 'DBSCAN');
    end

    % 그룹 병합 -> centroid detections
    clusteredDets = mergeDetections(detectionsDynamic, clusters);

    % 트래커 업데이트 (JIPDA)
    if isLocked(tracker) || ~isempty(clusteredDets)
        tracks = tracker(clusteredDets, time);
    end

    % 시각화: spherical -> Cartesian 변환 (x = r*cosd(az), y = r*sind(az))
    % raw points
    az_all = cellfun(@(d) d.Measurement(1), detections);
    r_all  = cellfun(@(d) d.Measurement(2), detections);
    x_all = r_all .* cosd(az_all);
    y_all = r_all .* sind(az_all);

    cla; hold on; grid on;
    scatter(x_all, y_all, 24, 'b', 'filled'); % raw
    % cluster centroids
    if ~isempty(clusteredDets)
        az_c = cellfun(@(d) d.Measurement(1), clusteredDets);
        r_c  = cellfun(@(d) d.Measurement(2), clusteredDets);
        x_c = r_c .* cosd(az_c);
        y_c = r_c .* sind(az_c);
        scatter(x_c, y_c, 80, 'r', 'filled');
    end
    % tracks
    % tracks 및 궤적 (개선된 시각화)
    currentTrackIDs = [];
    if ~isempty(tracks)
        % Map에서 삭제할 트랙 ID들을 미리 수집 (사라진 트랙 관리)
        trackIDsInHistory = keys(trackHistories);
        trackIDsToDelete = setdiff(trackIDsInHistory, arrayfun(@(t) t.TrackID, tracks));
        for id = trackIDsToDelete
            remove(trackHistories, id);
        end

        for k = 1:numel(tracks)
            t = tracks(k);
            currentTrackIDs = [currentTrackIDs, t.TrackID];

            % 현재 트랙 위치
            x_t_curr = t.State(1);
            y_t_curr = t.State(2);

            % 트랙 ID를 이용해 색상을 다르게 할 수도 있습니다.
            % 예를 들어, mod(t.TrackID, N)을 사용해 N가지 색상 순환
            trackColor = [0, 0.7, 0]; % 초록색

            scatter(x_t_curr, y_t_curr, 120, trackColor, 'filled', 'DisplayName', sprintf('Track %d', t.TrackID));
            text(x_t_curr + 0.3, y_t_curr + 0.3, num2str(t.TrackID), 'Color', trackColor, 'FontSize', 10);

            % 궤적 저장 및 그리기
            if ~isKey(trackHistories, t.TrackID)
                trackHistories(t.TrackID) = [x_t_curr; y_t_curr];
            else
                history = trackHistories(t.TrackID);
                % 궤적 길이를 제한할 수 있습니다. (예: 최근 50개 점만 저장)
                maxHistoryLength = 50; % 50프레임 동안의 궤적
                if size(history, 2) >= maxHistoryLength
                    history = history(:, (end-maxHistoryLength+1):end);
                end
                trackHistories(t.TrackID) = [history, [x_t_curr; y_t_curr]];
            end

            % 궤적 그리기
            history = trackHistories(t.TrackID);
            if size(history, 2) > 1
                plot(history(1,:), history(2,:), 'Color', trackColor, 'LineStyle', ':'); % 점선으로 궤적 표시
            end
        end
    end

    title(sprintf('Time = %.2f s | Detections = %d | Tracks = %d', time, numel(detections), numel(tracks))); % 트랙 개수 추가
    legend({'Raw points','Cluster centroids','Tracks'}, 'Location','northeast');
    xlim([0 15]); ylim([-8 8]);
    drawnow;
end

% 안전하게 정리
try
    sensorStop(rdr);   % 가능하면 정지
catch
    % sensorStop()가 없을 경우 무시
end
clear rdr;

%% Supporting function: 초기화 필터
function filter = initPeopleTrackingFilter(detection)
    filter3D = initcvekf(detection);
    state = filter3D.State(1:4);
    stateCov = filter3D.StateCovariance(1:4,1:4);

    velCov = stateCov([2 4],[2 4]);
    [v,d] = eig(velCov);
    D = diag(d); D(2) = 1;
    stateCov([2 4],[2 4]) = v*diag(D)*v';

    Q = 0.2*eye(2);
    filter = trackingEKF(State=state, ...
        StateCovariance=stateCov, ...
        StateTransitionFcn=@constvel, ...
        StateTransitionJacobianFcn=@constveljac, ...
        HasAdditiveProcessNoise=false, ...
        MeasurementFcn=@cvmeas, ...
        MeasurementJacobianFcn=@cvmeasjac, ...
        ProcessNoise=Q, ...
        MeasurementNoise=detection.MeasurementNoise);
end
