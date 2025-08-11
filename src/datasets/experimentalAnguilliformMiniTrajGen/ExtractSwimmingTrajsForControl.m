clear all 
close all

wavelengths = ["0.67L", "1.00L", "1.33L", "1.67L", "2.00L"];
wavelengths_num = [0.67, 1.00, 1.33, 1.67, 2.00];
amplitudes = ["(30_30_30)", "(10_30_50)",  "(20_50_20)","(50_30_10)"];
pageInd = 1;

totalTrials = length(wavelengths)*length(amplitudes);
% % Thrust data

sampleRate = 12;  % Data sample rage [Hz]
simRate = 100; % Simulation sample rate [Timesteps/s]

startTime = 0;
period = 2; % s
endTime = 20;

temp = zeros(1,sampleRate*endTime);
tempTest = zeros(1,sampleRate*endTime);

% InitializeArrays to hold data for reference Trajectories
trialParams = strings(1,totalTrials); 
refTrajs_centered = zeros(40,(endTime-startTime)*simRate+1, totalTrials);
refTrajs_uncentered = zeros(40,(endTime-startTime)*simRate+1, totalTrials);

%% Iterate through every combination of wavelength and amplitude
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
        refPoints_notTimeInterpolatedX = zeros(endTime*sampleRate + 1, 18);
        refPoints_notTimeInterpolatedY = zeros(endTime*sampleRate + 1, 18);
        refPoints_timeInterpolatedX = zeros((endTime*simRate) + 1, 18);
        refPoints_timeInterpolatedY = zeros((endTime*simRate) + 1, 18);
        for frame = startTime*12+1 : 1: (endTime*12+1)% (startTime+period)*12

            a = [markersX(frame, 2), markersY(frame, 2)];
            b = [markersX(frame, 3), markersY(frame, 3)];

            v = a - b;
            u = v / norm(v);

            d = 225; % tail length (have to calibrate!) % TODO!
            tailtip = a + d*u;
            markersX(frame, 1) = tailtip(1);
            markersY(frame, 1) = tailtip(2);
            temp(1,frame) = markersY(frame,1);
            
            % Constant curvature interpolation scheme
            % setup array to hold entire curve 
            bodyCurveRawX = [markersX(frame, 1)];
            bodyCurveRawY = [markersY(frame, 1)];
            % figure(1)
            % pbaspect([1 1 1])
            % hold on
            for j = [2,4,6]
                P1 = [markersX(frame,j), markersY(frame,j)];
                V1 = [markersX(frame,j+1), markersY(frame,j+1)];
                V2 = [markersX(frame,j+2), markersY(frame,j+2)];
                P2 = [markersX(frame,j+3), markersY(frame,j+3)];
                [theta, xArc, yArc, center] = tangentarc(P1, V1, V2, P2);
                % Concat with existing body curve
                bodyCurveRawX = [bodyCurveRawX, xArc];
                bodyCurveRawY = [bodyCurveRawY,yArc];
            end
            % Interpolate along body arc 
            interpFracs= linspace(0,1,18); % Interpolation points as fractions of curve
            bodyCurveInterpolated = interparc(interpFracs, bodyCurveRawX, bodyCurveRawY, 'linear');            
            refPoints_notTimeInterpolatedX(frame,:) = bodyCurveInterpolated(:,1)';
            refPoints_notTimeInterpolatedY(frame,:) = bodyCurveInterpolated(:,2)';
            
            % % Plot for sanity check
            % plot(bodyCurveRawX, bodyCurveRawY, 'k');
            % scatter(bodyCurveInterpolated(:,1),bodyCurveInterpolated(:,2),'ro')
            % pbaspect([1 1 1])
            % axis equal

        
        end
        
        % Change reference frame
        xOffset = refPoints_notTimeInterpolatedX(1,18);
        refPoints_notTimeInterpolatedX = -(refPoints_notTimeInterpolatedX-xOffset); % Center and reflect
        yOffset = refPoints_notTimeInterpolatedY(1,18);
        refPoints_notTimeInterpolatedY = refPoints_notTimeInterpolatedY-yOffset;
        % Stretch everything to make appropriate size for simulation
        % reference trajectory
        stretchFactor = 665.0/470.0;
        refPoints_notTimeInterpolatedX = refPoints_notTimeInterpolatedX*stretchFactor;
        refPoints_notTimeInterpolatedY = refPoints_notTimeInterpolatedY*stretchFactor;
        


        % Interpolate over time to resample for simulation frame rate
        sampleTimesRaw = 0:1/sampleRate:(endTime-startTime);
        sampleTimesRef = 0:1/simRate:(endTime-startTime);
        for refInd = 1:1:18
            refPoints_timeInterpolatedX(:,refInd) = interp1(sampleTimesRaw,refPoints_notTimeInterpolatedX(:,refInd),sampleTimesRef, 'linear')';
            refPoints_timeInterpolatedY(:,refInd) = interp1(sampleTimesRaw,refPoints_notTimeInterpolatedY(:,refInd),sampleTimesRef, 'linear')';
        end

        % Save ref trajectory into array
        refTraj_trial_uncentered = zeros(40,(endTime-startTime)*simRate+1);
        refTraj_trial_centered = zeros(40,(endTime-startTime)*simRate+1);
        for refInd = 1:1:18
            refTraj_trial_uncentered(2*refInd-1,:) = refPoints_timeInterpolatedX(:,refInd)';
            refTraj_trial_uncentered(2*refInd,:)   = refPoints_timeInterpolatedY(:,refInd)';
        end


        trialParams(1,pageInd) = sprintf("amp_%s_freq_%s", amplitudes(a_),wavelengths(w_));
        refTrajs_uncentered(:,:,pageInd) = refTraj_trial_uncentered;
        refTrajs_centered(:,:,pageInd) = refTraj_trial_uncentered-refTraj_trial_uncentered(:,1);

        pageInd =pageInd+1;
        
        % Animate 

        figure(10)
        hold off
        for ind =1:10:length(sampleTimesRef)
            t_ref = sampleTimesRef(1,ind);
            temp = sampleTimesRaw((sampleTimesRaw+1/sampleRate-t_ref)>=0);
            t_raw = temp(1,1);
            raw_ind = find(sampleTimesRaw == t_raw);
            plot(refPoints_timeInterpolatedX(ind,:),refPoints_timeInterpolatedY(ind,:), 'k-')
            hold on 
            scatter(refPoints_notTimeInterpolatedX(raw_ind,:),refPoints_notTimeInterpolatedY(raw_ind,:),'ro')
            title(sprintf('t = %0.4f',t_ref))
            pbaspect([1 1 1])
            axis equal
            xlim([-50 1000])
            ylim([-500 500])
            hold off

            exportgraphics(gca,sprintf("test_expRobotTraj_amp_%s_freq_%s_fast.gif", amplitudes(a_),wavelengths(w_)),Append=true)



        end


    end


end

% Save file 
filename = "experimentalRefTrajectories.h5";
dataset_name = '/trialParameters';
data = trialParams;
h5create(filename,dataset_name,size(data), 'Datatype', 'string')
h5write(filename, dataset_name, data);
dataset_name = '/refsUncentered';
data = refTrajs_uncentered;
h5create(filename,dataset_name,size(data))
h5write(filename, dataset_name, data);
dataset_name = '/refsCentered';
data = refTrajs_centered;
h5create(filename,dataset_name,size(data))
h5write(filename, dataset_name, data);
dataset_name = '/tRef';
data = sampleTimesRef;
h5create(filename,dataset_name,size(data))
h5write(filename, dataset_name, data);


h5disp(filename)


%% Plot a couple points over time

figure(3)

for plotInd = 1:1:9
    subplot(9,2,2*plotInd-1)
    hold on
    plot(sampleTimesRef, refPoints_timeInterpolatedX(:,plotInd), 'k')
    plot(sampleTimesRaw, refPoints_notTimeInterpolatedX(:,plotInd), 'r--')
    subplot(9,2,2*plotInd)
    hold on
    plot(sampleTimesRef, refPoints_timeInterpolatedY(:,plotInd), 'k')
    plot(sampleTimesRaw, refPoints_notTimeInterpolatedY(:,plotInd), 'r--')

end
    subplot(9,2,1)
    scatter(sampleTimesRaw, markersX(1:frame,1),'g')
    subplot(9,2,2)
    scatter(sampleTimesRaw, markersY(1:frame,1),'g')



% Move the marker along the parabola and capture frames in a loop


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
    % % % Plot the arc
    % plot(xArc, yArc, 'r');
    % pbaspect([1 1 1])
    % axis equal


    % Plot the center point
    % plot(center(1), center(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
end