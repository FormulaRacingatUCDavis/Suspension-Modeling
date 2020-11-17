function Sweep= RollAndSteerAnalysis( Target, Points, Design )
%% Determine Design Axles
DesignAxle = find(~cellfun(@isempty,{Target.FVSA}));

%% Design Space Plot
figure;    
for i = DesignAxle
    plot3( Points(i).LA.W(1,:), Points(i).LA.W(2,:), Points(i).LA.W(3,:) , 'bd', ...
           Points(i).UA.W(1,:), Points(i).UA.W(2,:), Points(i).UA.W(3,:) , 'rd', ... 
           Points(i).TA.W(1,:), Points(i).TA.W(2,:), Points(i).TA.W(3,:) , 'gd', ...
           Points(i).LB.W(1,:), Points(i).LB.W(2,:), Points(i).LB.W(3,:) , 'bs', ...
           Points(i).UB.W(1,:), Points(i).UB.W(2,:), Points(i).UB.W(3,:) , 'rs', ...
           Points(i).TB.W(1,:), Points(i).TB.W(2,:), Points(i).TB.W(3,:) , 'gs'); hold on;
       
    plot3( (-1)^(i+1)*Target(i).Wheelbase/2, Target(i).Track/2  , Target(i).Rl, 'ko');
    plot3( (-1)^(i+1)*Target(i).Wheelbase/2, 0, Target(i).RollCenter, 'k+' );
    plot3( (-1)^(i+1)*Target(i).Wheelbase/2, Target(i).InstantCenter(1), ...
        Target(i).InstantCenter(2), 'kx');
end

fill3( [1000 1000 -1000 -1000], [750 -750 -750 750], zeros(4,1), [0.3 0.3 0.3] )
fill3( [1000 1000 -1000 -1000], zeros(4,1), [0 500 500 0], [0.8 0.8 0.8], 'FaceAlpha', 0.1 )
fill3( [1000 1000 -1000 -1000], [250 -250 -250 250], Target(1).Ride.*ones(4,1), [0, 0, 1], 'FaceAlpha', 0.1 )

title('3D Geometry Design Space Representation')

xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');

legend( {'LA', 'UA', 'TA', 'LB', 'UB', 'TB', ...
    'Tire Center', 'Instant Center', 'Roll Center'} )

axis equal; zlim([0 500]);
view(135,15);

%% Geometry Sweeps & Visualization
tic
for i = DesignAxle
    for j = 1 : size(Design,2)
        %%% Sweep Setup
        N = 9;
        M = 3;
        
        % Ride Sweep
        Sweep(i,j,1).Ride  = repmat( linspace( Target(1).Ride-25.4, Target(1).Ride+25.4, N ), M, 1);
        Sweep(i,j,1).Roll  = zeros(M,N);
        Sweep(i,j,1).Pitch = zeros(M,N);
        Sweep(i,j,1).Steer = linspace(-20,20,M)' * ones(1,N);

        % Roll Sweep
        Sweep(i,j,2).Ride  = zeros(M,N);
        Sweep(i,j,2).Roll  = repmat( linspace( -2.5, 2.5, N ), M, 1);
        Sweep(i,j,2).Pitch = zeros(M,N);
        Sweep(i,j,2).Steer = linspace(-20,20,M)' * ones(1,N);

        % Steer Sweep
        Sweep(i,j,3).Ride  = linspace(Target(1).Ride-25.4, Target(1).Ride+25.4,M)' * ones(1,N);
        Sweep(i,j,3).Roll  = zeros(M,N);
        Sweep(i,j,3).Pitch = zeros(M,N);
        Sweep(i,j,3).Steer = repmat( linspace( -25, 25, N ), M, 1);

        %%% Calculating Tire Orientation
        for k = 1 : size(Sweep,3)
            for m = 1 : size(Sweep(i,j,k).Ride,1)
                for n = 1 : size(Sweep(i,j,k).Ride,2)
                    Sample.Ride  = Sweep(i,j,k).Ride(m,n);
                    Sample.Roll  = Sweep(i,j,k).Roll(m,n);
                    Sample.Pitch = Sweep(i,j,k).Pitch(m,n);
                    Sample.Steer = Sweep(i,j,k).Steer(m,n);

                    [~, Sweep(i,j,k).Track(m,n,:), Sweep(i,j,k).Caster(m,n,:), ...
                        Sweep(i,j,k).Camber(m,n,:), Sweep(i,j,k).Toe(m,n,:), ~, ...
                        Sweep(i,j,k).RollCenter(m,n,:), ~, Sweep(i,j,k).Scrub(m,n,:)] = ...
                        KinematicsEvaluation2( Design(i,j), Sample, Target(i) );
                end
            end
        end
        fprintf( 'Design Sweep Complete: %4.0f sec elapsed \n', toc )
    end
    fprintf( 'Axle Sweep Complete: %4.0f sec elapsed \n', toc )
end

%%% Plotting
for i = DesignAxle
    figure; 
    subplot(3,2,1) % Bump Steer
    for j = 1 : size(Design,2)
        plot(Sweep(i,j,1).Ride', Sweep(i,j,1).Toe(:,:,1)' - Sweep(i,j,1).Toe(:,end/2,1)); hold on;
    end
    
    subplot(3,2,3) % Roll Steer
    for j = 1 : size(Design,2)
        plot(Sweep(i,j,2).Roll', Sweep(i,j,2).Toe(:,:,1)'); hold on;
    end
    
    subplot(3,2,5) % Steer-Steer
    for j = 1 : size(Design,2)
        plot(Sweep(i,j,3).Steer, Sweep(i,j,3).Toe(:,:,1)); hold on;
    end
    
    subplot(3,2,2) % Camber Gain

    subplot(3,2,4) % Roll Camber

    subplot(3,2,4) % Steer Induced Camber

end