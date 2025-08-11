% clear all

wavelengths = ["0.67L", "1.00L", "1.33L", "1.67L", "2.00L"];
wavelengths_num = [0.67, 1.00, 1.33, 1.67, 2.00];
amplitudes = ["(30_30_30)", "(10_30_50)",  "(20_50_20)","(50_30_10)"];
% % Thrust data
% thrustdata = readmatrix('Thrust.csv');
% thrust = zeros(5, 4);
% tailamplitude = zeros(5, 4);

startTime = 10;
period = 2;
endTime = startTime + 5*period;

%%
for w_ = 1:length(wavelengths)
    wavelength = wavelengths(w_);

    for a_ = 1:length(amplitudes)
        amplitude = amplitudes(a_);

        [w_, a_]

        % Tracking data
        trackingdatapath = 'tracker_data/0.5hz_' + wavelength + '_' + amplitude + '.csv';
        trackingdata = readtable(trackingdatapath);
                
        numMarkers = (width(trackingdata) - 1) / 2;
        numFrames = size(trackingdata, 1);

        markersX = zeros(numFrames, numMarkers+1);
        markersY = zeros(numFrames, numMarkers+1);
        
        for i = 1:numMarkers
            markersX(:, 1+i) = trackingdata{:, 2*i};
            markersY(:, 1+i) = trackingdata{:, 2*i + 1};
        end

        % logicalIndex = (thrustdata(:, 2*(4*(w_-1)+a_)-1) >= startTime) & (thrustdata(:, 2*(4*(w_-1)+a_)-1) <= endTime);
        % thrust(w_, a_) = -mean(thrustdata(logicalIndex, 2*(4*(w_-1)+a_)));

        % figure(1);
        % subplot(5, 4, 4*(w_-1) + a_);
        % yline(0, 'k', 'LineWidth', 0.5); hold on
        % plot(1/12:1/12:(1/12)*length(-thrustdata(logicalIndex, 2*(4*(w_-1)+a_))), -thrustdata(logicalIndex, 2*(4*(w_-1)+a_)), 'k', 'LineWidth', 1)
        % ylim([-0.4, 0.5])

        figure(2);
        subplot(5, 4, 4*(w_-1) + a_); hold on
        
        for frame = startTime*12+1 : 4: endTime*12+1% (startTime+period)*12

            a = [markersX(frame, 2), markersY(frame, 2)];
            b = [markersX(frame, 3), markersY(frame, 3)];

            v = a - b;
            u = v / norm(v);

            d = 150; % tail length (have to calibrate!)
            tailtip = a + d*u;
            markersX(frame, 1) = tailtip(1);
            markersY(frame, 1) = tailtip(2);
            
            % linearly connecting the markers
            % plot(markersX(frame,:), markersY(frame, :), 'k.-')

            % constant curvature
            plot(markersX(frame,:), markersY(frame, :), 'k.')
            plot(markersX(frame,1:2), markersY(frame, 1:2), 'k.-')

            xlim([-10, 620]); ylim([50 410])
            pbaspect([630, 360, 1])

            for j = [2,4,6]
                P1 = [markersX(frame,j), markersY(frame,j)];
                V1 = [markersX(frame,j+1), markersY(frame,j+1)];
                V2 = [markersX(frame,j+2), markersY(frame,j+2)];
                P2 = [markersX(frame,j+3), markersY(frame,j+3)];
                [theta, xArc, yArc, center] = tangentarc(P1, V1, V2, P2);
            end
        
        tailamplitude(w_, a_) = (max(markersY(:, 1)) - min(markersY(:, 1)))/2;

        end
        
    end
end


%%
% thrust_norm = thrust./max(thrust(:));
% 
% for w_ = 1:length(wavelengths)
%     wavelength = wavelengths(w_);
% 
%     for a_ = 1:length(amplitudes)
%         amplitude = amplitudes(a_);
% 
%         figure(1);
%         subplot(5, 4, 4*(w_-1) + a_);
%         if thrust_norm(w_,a_) > 0
%             Ax = gca;
%             Ax.Color = [1,1,1] - 0.5*[thrust_norm(w_,a_), thrust_norm(w_,a_), 0];
%         else
%             Ax = gca;
%             Ax.Color = [1,1,1] + 0.5*[0, thrust_norm(w_,a_), thrust_norm(w_,a_)];
%         end
% 
%         figure(2);
%         subplot(5, 4, 4*(w_-1) + a_);
%         if thrust_norm(w_,a_) > 0
%             Ax = gca;
%             Ax.Color = [1,1,1]-0.5*[thrust_norm(w_,a_), thrust_norm(w_,a_), 0];
%         else
%             Ax = gca;
%             Ax.Color = [1,1,1]+0.5*[0, thrust_norm(w_,a_), thrust_norm(w_,a_)];
%         end
%     end
% end

%% export
% exportgraphics(figure(1),'thrust.pdf','ContentType','vector')
% exportgraphics(figure(2),'kinematics.pdf','ContentType','vector')

%%
% figure(11);
% for i = -5:1:10
%     thrust = 0.1*i;
%     subplot(4,4,i+6)
%     Ax = gca;
%     if thrust > 0; Ax.Color = [1,1,1]-0.5*thrust*[1,1,0];
%     else; Ax.Color = [1,1,1]+0.5*thrust*[0,1,1];
%     end
% end

%%
% close all
% figure(111);
% subplot(1,3,3)
% markersets = {'s', '<', 'diamond', '>'};
% colorsets = {'k', 'r', 'g', 'b'};
% alphasets = {0.2, 0.4, 0.6, 0.8, 1};
% % alphasets = {1,1,1,1,1};
% % sizesets = {20, 40, 60, 80, 100};
% % sizesets = {100,100,100,100, 100};
% sizesets = {200,200,200,200, 200};
% 
% for w_ = 1:length(wavelengths)
%     wavelength = wavelengths(w_);
%     for a_ = 1:length(amplitudes)
%         scatter(tailamplitude(w_, a_), thrust(w_, a_), sizesets{w_}, ...
%         'Marker', markersets{a_}, 'MarkerFaceColor', colorsets{a_}, ...
%         'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', alphasets{w_}); hold on
%         % plot(tailamplitude(w_, a_), thrust(w_, a_), '.', 'MarkerSize', 10,'Color', colorsets{a_},  'Marker', markersets{w_}); hold on
%     end
% end
% pbaspect([1,1,1])
% ylim([-0.04, 0.12]); xlim([90,220])
% ylabel('thrust [N]')
% xlabel('Tail amplitude (pixel)')
% 
% subplot(1,3,2)
% for w_ = 1:length(wavelengths)
%     wavelength = wavelengths_num(w_);
%     for a_ = 1:length(amplitudes)
%         % plot(wavelength*ones(size(thrust(w_, a_))), thrust(w_, a_), '.', 'MarkerSize', 10, 'Color', colorsets{a_}, 'Marker', markersets{w_}); hold on
%         % plot(wavelength*ones(size(thrust(w_, a_))), tailamplitude(w_, a_), '.', 'MarkerSize', 10, 'Color', colorsets{a_}, 'Marker', markersets{w_}); hold on  
%         scatter(wavelength*ones(size(thrust(w_, a_))), tailamplitude(w_, a_), sizesets{w_}, ...
%             'Marker', markersets{a_}, 'MarkerFaceColor', colorsets{a_}, ...
%             'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', alphasets{w_}); hold on
%     end
% end
% xlabel('Body wavelength (/active BL)')
% ylabel('Tail amplitude (pixel)')
% pbaspect([1,1,1])
% ylim([90 220]); xlim([0.25, 2.25])
% 
% subplot(1,3,1)
% for w_ = 1:length(wavelengths)
%     wavelength = wavelengths_num(w_);
%     for a_ = 1:length(amplitudes)
%         % plot(wavelength*ones(size(thrust(w_, a_))), thrust(w_, a_), '.', 'MarkerSize', 10, 'Color', colorsets{a_}, 'Marker', markersets{w_}); hold on
%         % plot(wavelength*ones(size(thrust(w_, a_))), tailamplitude(w_, a_), '.', 'MarkerSize', 10, 'Color', colorsets{a_}, 'Marker', markersets{w_}); hold on  
%         scatter(wavelength*ones(size(thrust(w_, a_))), thrust(w_, a_), sizesets{w_}, ...
%             'Marker', markersets{a_}, 'MarkerFaceColor', colorsets{a_}, ...
%             'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', alphasets{w_}); hold on
%     end
% end
% xlabel('Body wavelength (/active BL)')
% ylabel('thrust [N]')
% pbaspect([1,1,1])
% ylim([-0.04, 0.12]); xlim([0.25, 2.25])
% 
% % exportgraphics(figure(111),'scatter.pdf','ContentType','vector')

%%
function [theta, xArc, yArc, center] = tangentarc(P1, V1, V2, P2)
                    
    % Find the perpendicular bisector midpoints
    % midPoint1 = (P1 + V1) / 2;
    % midPoint2 = (P2 + V2) / 2;
    
    % Calculate the perpendicular slopes
    % slope1 = -1 / ((V1(2) - P1(2)) / (V1(1) - P1(1)));
    slope2 = -1 / ((V2(2) - P2(2)) / (V2(1) - P2(1)));
    
    syms x y
    % eq1 = y - midPoint1(2) == slope1 * (x - midPoint1(1));
    % eq2 = y - midPoint2(2) == slope2 * (x - midPoint2(1));
    
    if V2(2) - P2(2) == 0
        center = Inf;
        theta = 0;
        xArc = linspace(V1(1), V2(1), 100);
        yArc = linspace(V1(2), V2(2), 100);

    else        
        eq1 = (V1(1)-x)^2 + (V1(2)-y)^2 == (V2(1)-x)^2 + (V2(2)-y)^2;

        if slope2 ~= Inf; eq2 = y - V2(2) == slope2 * (x - V2(1));
        else; eq2 = x == V2(1);
        end
    
        sol = solve([eq1, eq2], [x, y]);
        if isempty(sol.x) || isempty(sol.y); xArc = []; yArc = []; theta = []; center = [];
        else
        
            % Center of the arc
            center = [double(sol.x), double(sol.y)];
            
            % Radius of the arc
            radius = (norm(center - V1) + norm(center - V2))/2;
            
            % Define the angles for the arc
            theta1 = atan2(V1(2) - center(2), V1(1) - center(1));
            theta2 = atan2(V2(2) - center(2), V2(1) - center(1));
            
            % Generate the points for the arc
            theta = linspace(theta1, theta2, 100);
            xArc = center(1) + radius * cos(theta);
            yArc = center(2) + radius * sin(theta);
        end

    end
    % Plot the arc
    plot(xArc, yArc, 'k');
    
    % Plot the center point
    % plot(center(1), center(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
end