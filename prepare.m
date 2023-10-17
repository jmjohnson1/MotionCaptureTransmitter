load("/Users/james/Documents/MotionCaptureTransmitter/mocapData.mat");

% t = 1000;
% plot3(marker0(t, 1), marker0(t, 2), marker0(t, 3), 'r.', MarkerSize=100);

% Define initial and final time
t_0 = 374.756; % seconds
t_f = 679.212; % seconds


% Convert timeVec to seconds
timeVec_ps = timeVec/1000;
timeVec_ps = timeVec_ps - t_0;

for i = 1:length(timeVec_ps)
    num_avail = 0;
    avg_pos(i, :) = [0, 0, 0];

    avg_x = 0;
    avg_y = 0;
    avg_z = 0;

    if ~isnan(marker0(i, 1))
        avg_x = avg_x + marker0(i, 1);
        avg_y = avg_y + marker0(i, 2);
        avg_z = avg_z + marker0(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker1(i, 1))
        avg_x = avg_x + marker1(i, 1);
        avg_y = avg_y + marker1(i, 2);
        avg_z = avg_z + marker1(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker2(i, 1))
        avg_x = avg_x + marker2(i, 1);
        avg_y = avg_y + marker2(i, 2);
        avg_z = avg_z + marker2(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker3(i, 1))
        avg_x = avg_x + marker3(i, 1);
        avg_y = avg_y + marker3(i, 2);
        avg_z = avg_z + marker3(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker4(i, 1))
        avg_x = avg_x + marker4(i, 1);
        avg_y = avg_y + marker4(i, 2);
        avg_z = avg_z + marker4(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker5(i, 1))
        avg_x = avg_x + marker5(i, 1);
        avg_y = avg_y + marker5(i, 2);
        avg_z = avg_z + marker5(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker6(i, 1))
        avg_x = avg_x + marker6(i, 1);
        avg_y = avg_y + marker6(i, 2);
        avg_z = avg_z + marker6(i, 3);
        num_avail = num_avail + 1;
    end
    if ~isnan(marker7(i, 1))
        avg_x = avg_x + marker7(i, 1);
        avg_y = avg_y + marker7(i, 2);
        avg_z = avg_z + marker7(i, 3);
        num_avail = num_avail + 1;
    end

    if num_avail > 0
        avg_pos(i, :) = [avg_x, avg_y, avg_z]/num_avail;
    end
end

figure()
subplot(3, 1, 1)
plot(timeVec_ps, avg_pos(:, 1));
subplot(3, 1, 2)
plot(timeVec_ps, avg_pos(:, 2));
subplot(3, 1, 3)
plot(timeVec_ps, avg_pos(:, 3));

markers = avg_pos;

save("processedMocap.mat", "markers", "timeVec_ps");
