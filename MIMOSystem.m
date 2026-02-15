classdef MIMOSystem < handle
    properties
        G           % transfer function matrix
        numOut      % number of outouts from mimo system
        numIn       % number of inputs to mimo system
        Kdc         % steady state gain
        RGA         % Relative gain array
        Kc          % controller gain matrix
        pairing     % input-output pairing for control design (matrix with 1 and 0)
        sampleTime  % recommended sample time for digital control design
        SR          % step response for each subsystem (cell array)
        mpcParam    % MPC design parameters
    end
    methods
        function obj = MIMOSystem(G)
            obj.G = G;
            obj.Kdc = dcgain(G);
            [obj.numOut, obj.numIn] = size(obj.G);
        end
        
        function obj = computeRGA(obj)
            % Compute and store RGA for square, invertible G

            [m,n] = size(obj.G);
            if m ~= n
                error('MIMOSystem:nonSquareG', ...
                    'RGA is only defined for square G. Got %d x %d.', m, n);
            end

            % Dynamic RGA
            obj.RGA.dynamic = inv(obj.G') .* obj.G;          % Λ(s) = G(s) ∘ (G(s)^{-T})

            % Static RGA
            obj.RGA.static  = dcgain(obj.RGA.dynamic);       % Λ(0) [file:1]
        end

        function  staticRGA_analysis(obj)
            % Analyze the static RGA to determine pairing
            % which input should be paired with which output for control design
            disp('Static RGA Analysis:');
            disp('Relative Gain Array (Static):');
            disp(obj.RGA.static);
            obj.pairing = zeros(size(obj.RGA.static)); % Initialize pairing matrix
            % Determine pairing based on the static RGA values
            for i = 1:obj.numOut
                [~, pairedInput] = max(abs(obj.RGA.static(i, :)));
                fprintf('Output %d is best paired with Input %d (RGA value: %.2f)\n', ...
                    i, pairedInput, obj.RGA.static(i, pairedInput));
                obj.pairing(i, pairedInput) = 1; % Mark the pairing in the matrix
            end

        end

        % Multiloop control design methods: (siso design for each loop, decoupling, etc.)
        function obj = DirectSynthesis(obj, DesiredCL)
            % Direct synthesis methods: (designing controllers based on desired closed-loop performance)
            % Y/Ysp = KcG'/(1+KcG') for each loop, where K is the controller gain and G is the transfer function of the system.
            % ideal feedback control design: Kc = 1/G' * (Y/Ysp)/(1-Y/Ysp) to achieve desired closed-loop performance.
            % G' is model of the system not the actual system G, so the performance may not be ideal due to model inaccuracies and disturbances.
            % Design SISO controllers for each loop based on desired performance

            % This is a placeholder function and should be implemented based on specific design criteria

            for i = 1:size(obj.G, 1) % For each output
                for j = 1:size(obj.G, 2) % For each input
                    if obj.pairing(i, j) == 1 % If this input is paired with this output
                        % Design a SISO controller for this loop
                        G_ij = obj.G(i, j); % Transfer function for this loop
                        K = obj.sisoDesign(G_ij, DesiredCL(i)); % Design a SISO controller for this loop
                        K_(i, j) = K; % Store the controller gain in the matrix
                    end
                end
            end
            obj.Kc = K_;
        end

        function K = sisoDesign(obj, G, desiredCL)
            % Placeholder function for SISO controller design
            % This function should be implemented based on specific design criteria (e.g., PID, lead-lag, etc.)
            % For demonstration purposes, we will use a simple proportional controller design
            s = tf('s');
            switch desiredCL.model
                case 'FO' % First-order system design
                    tau = desiredCL.tau; % Time constant
                    K = 1/G * (1/tau/s);
                case 'FOPTD' % First-order plus time delay system design
                    tau = desiredCL.tau; % Time constant
                    theta = desiredCL.theta; % Dead time
                    K = 1/G * exp(-theta*s)/(tau + theta)/s; % approximate with taylor expansion exp(-theta*s) ~ 1 - theta*s for small theta
                case 'SO' % Second-order system design
                    zeta = desiredCL.zeta; % Damping ratio
                    wn = desiredCL.wn; % Natural frequency
                    K = 1/G * (wn^2/(s^2 + 2*zeta*wn*s));
                case 'SOPTD' % Second-order plus time delay system design
                    zeta = desiredCL.zeta; % Damping ratio
                    wn = desiredCL.wn; % Natural frequency
                    theta = desiredCL.theta; % Dead time
                    K = 1/G * exp(-theta*s)*wn^2/(s^2 + 2*zeta*wn*s + wn^2*(1-exp(-theta*s))); % approximate with taylor expansion exp(-theta*s) ~ 1 - theta*s for small theta
                otherwise
            end
        end

        function obj = desiredSamplTime(obj)
            % Recommended sample time based on the dynamics of the system (e.g., 10 times faster than the fastest time constant of the system)
            % Find poles of the system to determine the fastest time constant
            poles = pole(obj.G);
            timeConstants = -1./real(poles); % Time constants are the negative inverse of the real part of the poles
            minTimeConstant = min(timeConstants); % Find the minimum time constant
            obj.sampleTime = minTimeConstant / 10; % Recommended sample time is 10 times faster than the fastest time constant
            fprintf('Recommended sample time: %.4f units\n', obj.sampleTime);
            % Find the time constants in the system
        end

        % MIMO design methods:
        % first: MPC
        function obj = MPCdesign(obj, mpcParams)
            % Model Predictive Control (MPC) design method
            % MPC is an advanced control strategy that uses a model of the system to predict future behavior and optimize control actions over a finite horizon.
            % The controller is designed to minimize a cost function that typically includes terms for tracking error and control effort, subject to constraints on inputs and outputs.
            % This is a placeholder function and should be implemented based on specific design criteria and optimization algorithms.
            if isfield(obj, 'Kc')
                obj.Kc = [];
            end
            obj.mpcParam = mpcParams;
            switch mpcParams.model
                % clear obj.kc if it exists
                case 'DMC'
                    obj = obj.dmcDesign(mpcParams);
                otherwise
            end
        end

        function obj = dmcDesign(obj, mpcParams)
            % DMC is a specific type of MPC that uses a step response model of the system to predict future behavior and optimize control actions.
            N = mpcParams.modelHorizon; % model horizon (step response length)
            if ~isfield(obj, 'sampleTime') & isempty(obj.sampleTime)
                obj.desiredSamplTime(); % Determine recommended sample time if not provided
            end
            ts = obj.sampleTime; % sampling time
            P = mpcParams.predictionHorizon; % prediction horizon
            M = mpcParams.controlHorizon; % control horizon
            Q = mpcParams.Q; % output weighting matrix
            R = mpcParams.R; % input weighting matrix
            
            if P > N
                error('Prediction horizon P must be <= model horizon N (length of step response).');
            end

            % Get step response for each subsystem (all of them)
            t = 0:ts:(N)*ts; % time vector for step response
            for i = 1:size(obj.G, 1) % For each output
                for j = 1:size(obj.G, 2) % For each input
                    G_ij = obj.G(i, j); % Transfer function for this loop
                    stepResponse = step(G_ij, t); % Get step response for this subsystem
                    obj.SR{i, j} = stepResponse(2:end); % Store the step response in a cell array
                end
            end
            % Construct dynamic matrix for each input-output pair
            S = zeros(P*size(obj.G, 1), M*size(obj.G, 2)); % Dynamic matrix
            Sk = cell(P, 1);
            % Sk = [SR11(k)  SR12(k)  ...  SR1n(k)  ;]
            %      [SR21(k)  SR22(k)  ...  SR2n(k)  ;]
            %      [...      ...      ...  ...      ;]
            %      [SRm1(k)  SRm2(k)  ...  SRmn(k)  ] m*n matrix for each k from 1 to P
            for k = 1 : P
                Sk{k, 1} = zeros(size(obj.G, 1), size(obj.G, 2)); % each m*n matrix
                for i = 1:size(obj.G, 1) % For each output
                    for j = 1:size(obj.G, 2) % For each input
                        Sk{k, 1}(i, j) = obj.SR{i,j}(k);
                    end
                end
            end
            % till now we have P times m*n matrices in Sk cell array
            % now we need to construct large S matrix
            % S = [Sk{1}  0        0        ...   0       ;]
            %     [Sk{2}  Sk{1}    0        ...   0       ;]
            %     [Sk{3}  Sk{2}    Sk{1}    ...   0       ;]
            %     [...    ...      ...      ...   ...     ;]
            %     [Sk{P}  Sk{P-1}  Sk{P-2}  ...   Sk{P-M+1}] P*m by M*n matrix

            for k = 1 : P
                for j = 1 : M
                    if j <= k
                        S((k-1)*size(obj.G, 1)+1:k*size(obj.G, 1), (j-1)*size(obj.G, 2)+1:j*size(obj.G, 2)) = Sk{k-j+1, 1};
                    end
                end
            end
            % Now we have the dynamic matrix S
            % Compute the DMC controller gain matrix
            [Q, R] = obj.mpcWeights();
            obj.Kc = (S'*Q*S + R)\(S'*Q); % DMC controller gain matrix
            
        end

        function Yr = desiredOutput(obj, yRef, k, index, num_out, Np, method)
            % Ref : referense signal (desired output)
            % i : current time
            % index : a time index to start simulation loop
            % num_out : number of outputs
            % Np : GPC predtiction horizon
            % method : 'programmed' or 'unprogrammed'

            Yd = cell(num_out,1);

            if strcmp(method , 'programmed')
                for i = 1:num_out
                    Yd{i,1} = yRef(k+1 :k+Np , i);
                    %         Yd{e,1} = filter(1-alpha, [1 -alpha], Ref(i-index +1 :i-index +Np , e));
                end
            elseif strcmp(method, 'unprogrammed')

                for i = 1:num_out
                    Yd{i,1} = yRef(k + 1, i) * ones(Np,1);
                    %         Yd{e,1} = filter(1-alpha, [1 -alpha], Ref(i-index +1 :i-index +Np , e));
                end
            else
                error('Unknown method. Use ''programmed'' or ''unprogrammed''.');
            end

            % y_ = [y1; y2; ...; ym] (m*1) vec
            % Yr = [y_1; y_2; ...y_p] (mP*1) vec
            Yr = zeros(num_out * Np, 1);
            for p = 1:Np
                for i = 1:num_out
                    Yr((p-1)*num_out + i) = Yd{i,1}(p);
                end
            end



        end

        function [index, Y, Ym, U, dU] = simMPC(obj, t, yRef, noise, disturbance)
            [num_out, num_in] = size(obj.G);
            ts = obj.sampleTime;
            Np = obj.mpcParam.predictionHorizon;
            Nu = obj.mpcParam.controlHorizon;
            method = obj.mpcParam.method;
            N = obj.mpcParam.modelHorizon;

            U = zeros(numel(t), num_in); % [U1, U2]
            dU = zeros(numel(t), num_in); % [dU1, dU2]
            Ym = zeros(numel(t), num_out); % [Ym1, Ym2]
            Y = zeros(numel(t), num_out); % [Y1, Y2]
            
            % now we need to simulate the system response to the control inputs and disturbances
            yk = zeros(num_out, 1); % initial process output
            yk_hat = zeros(num_out, 1); % initial model output
            u_opt = zeros(num_in, 1); % initial control input
            u_prev = zeros(num_in, 1); % previous input for delta-u
            du_hist = zeros(N, num_in); % past delta-u history (most recent first)
            index = N + 1; % time index to start simulation loop
            for  k = index : numel(t)
                % Desired/Reference output over horizon
                Yr = obj.desiredOutput(yRef, k, index, num_out, Np, method);

                % Update model output using step response and past delta-u
                yk_hat = zeros(num_out, 1);
                for i = 1:num_out
                    for j = 1:num_in
                        s = obj.SR{i, j};
                        for l = 1:N-1
                            yk_hat(i) = yk_hat(i) + s(l) * dU(k-l, j); 
                        end
                        yk_hat(i) = yk_hat(i) + s(N) * U(k-N, j); % contribution from the input N steps ago
                    end
                end

                % Process output = model + disturbance + noise
                yk = yk_hat + disturbance(k, :)' + noise(k, :)';

                Y(k, :) = yk';

                % Predicted free response over horizon (future delta-u = 0)
                Y_hat0 = zeros(Np * num_out, 1);
                for p = 1:Np
                    y_free_p = zeros(num_out, 1);
                    for i = 1:num_out
                        for j = 1:num_in
                            s = obj.SR{i, j};
                            for m = (p + 1):(N - 1)
                                y_free_p(i) = y_free_p(i) + s(m) * dU(k + p - m, j);
                            end
                            y_free_p(i) = y_free_p(i) + s(N) * U(k + p - N, j); % contribution from the input N steps ago
                        end
                    end
                    Y_hat0((p-1) * num_out + 1 : p * num_out) = y_free_p;
                end
                % Y_hat0 = repmat(yk_hat, Np, 1);  % block vector [yk_hat; yk_hat; ...]

                % Update model output history
                Ym(k, :) = yk_hat';

                % Model/real difference
                dm = yk - yk_hat;
                D = reshape(repmat(dm', Np, 1), [], 1);

                % Control signal update (delta-u)
                du = obj.controlSignal(Nu, Yr, Y_hat0, D, num_in);
                dU(k, :) = du; % history of delta-u

                % Update input and delta-u history for next step
                u_opt = u_prev + du(:);
                U(k, :) = u_opt'; % history of control efforts
                u_prev = u_opt(:);
            end

        end
        
        function du_opt = controlSignal(obj, Nu, Yd, Ypast, D, num_in)
            % Calculate the error
            E = Yd - Ypast - D;
            % Calculate control input
            dU = obj.Kc * E;
            % Extract the first Nu elements for each control-channels input
            du_opt = zeros(1, num_in);
            % for j = 1:num_in
            %     du_opt(j) = dU((j-1) * Nu + 1);
            % end
            du_opt = dU(1:num_in);
        end


        function [Q, R] = mpcWeights(obj)
            [num_out, num_in] = size(obj.G);
            Np = obj.mpcParam.predictionHorizon;
            Nu = obj.mpcParam.controlHorizon;
            q_weights = ones(1, num_out);
            q = obj.mpcParam.Q;
            r = obj.mpcParam.R;
            % Initialize cell array for Q matrices
            Q_matrices = cell(1, num_out);

            % Populate the Q_matrices cell array
            for i = 1:num_out
                Q_matrices{i} = q_weights(i) * eye(Np);
            end

            % Initialize the combined Q matrix
            Q = q * blkdiag(Q_matrices{:});

            a = obj.Kdc; % DC gain of G
            aa = a' * a;

            % Define the individual weights for each control input
            r_weights = ones(1, num_in);

            % Initialize cell array for R matrices
            R_matrices = cell(1, num_in);

            % Populate the R_matrices cell array
            for i = 1:num_in
                r_weights(i) = aa(i,i);
                R_matrices{i} = r_weights(i) * eye(Nu);
            end
            % Initialize the combined R matrix
            R = r * blkdiag(R_matrices{:});
        end

        function MPCplot(obj, t, index, Y, Ym, Ref, U, dU, disturbance, noise)
            % MPCplot - plot MPC simulation results stored in Y, Ym, U, dU, etc.
            %
            % obj  : MIMOSystem object
            % t    : time vector (column or row, same length as Y, U, ...)
            % Np   : prediction horizon (used only for info, not for indexing)
            % index: starting index of closed-loop simulation (same as in simMPC)
            % Y    : measured/actual outputs [length(t) x m]
            % Ym   : model outputs [length(t) x m]
            % Ref  : reference trajectories [length(t) x m]
            % U    : control inputs [length(t) x r]
            % dU   : control increments [length(t) x r]
            % disturbance: [length(t) x r] (or [] if unused)
            % noise      : [length(t) x m] (or [] if unused)

            [num_out, num_in] = size(obj.G);

            % Ensure column time vector
            t = t(:);

            % Effective simulation window (from index to end)
            t_sim = t(1:end);
            Y_sim  = Y(1:end, :);
            Ym_sim = Ym(1:end, :);
            Ref_sim = Ref(1:end - obj.mpcParam.predictionHorizon, :);
            Ref_sim(1: index, :) = zeros(index, num_out);
            U_sim  = U(1:end, :);
            dU_sim = dU(1:end, :);

            if ~isempty(disturbance)
                dist_sim = disturbance(1:end, :);
            else
                dist_sim = [];
            end

            if ~isempty(noise)
                noise_sim = noise(1:end, :);
            else
                noise_sim = [];
            end

            % Figure layout
            figparam = figure;
            nRow = 3; % (Y vs Ref & Ym), (U & dU), (disturbance & noise)
            nCol = 2 * max(num_in, num_out);

            color1 = [205/256, 79/256, 65/256];  % dark orange
            color2 = [40/256, 90/256, 110/256];  % dark green
            lw = 0.8;

            % --- Row 1: Y vs Ref and Y vs Ym ---

            % Y vs Ref
            for i = 1:num_out
                subplot(nRow, nCol, i);
                plot(t_sim, Y_sim(:, i), 'Color', color2, 'LineWidth', lw); hold on;
                plot(t_sim, Ref_sim(:, i), 'Color', color1, 'LineWidth', lw);
                legend(['y', num2str(i)], ['ref', num2str(i)], 'Location', 'best');
                xlabel('Time');
                ylabel(['y', num2str(i)]);
                title(['Output y', num2str(i), ' and reference']);
                grid on;
            end

            colBlock = max(num_in, num_out);

            % Y vs Ym
            for i = 1:num_out
                subplot(nRow, nCol, i + colBlock);
                plot(t_sim, Y_sim(:, i), 'Color', color2, 'LineWidth', lw); hold on;
                plot(t_sim, Ym_sim(:, i), 'Color', color1, 'LineWidth', lw);
                legend(['y', num2str(i)], ['y_m', num2str(i)], 'Location', 'best');
                xlabel('Time');
                ylabel(['y', num2str(i)]);
                title(['Output y', num2str(i), ' and model y_m']);
                grid on;
            end

            % --- Row 2: U and dU ---

            row2Offset = 2 * colBlock;

            % U
            for i = 1:num_in
                subplot(nRow, nCol, row2Offset + i);
                plot(t_sim, U_sim(:, i), 'Color', color2, 'LineWidth', lw);
                legend(['u', num2str(i)], 'Location', 'best');
                xlabel('Time');
                ylabel(['u', num2str(i)]);
                title(['Control input u', num2str(i)]);
                grid on;
            end

            % dU
            for i = 1:num_in
                subplot(nRow, nCol, row2Offset + colBlock + i);
                plot(t_sim, dU_sim(:, i), 'Color', color2, 'LineWidth', lw);
                legend(['\Delta u', num2str(i)], 'Location', 'best');
                xlabel('Time');
                ylabel(['\Delta u', num2str(i)]);
                title(['Control increment \Delta u', num2str(i)]);
                grid on;
            end

            % --- Row 3: disturbance and noise (if provided) ---

            row3Offset = 4 * colBlock;

            % Disturbance
            if ~isempty(dist_sim)
                for i = 1:size(dist_sim, 2)
                    subplot(nRow, nCol, row3Offset + i);
                    plot(t_sim, dist_sim(:, i), 'Color', color2, 'LineWidth', lw);
                    legend(['d', num2str(i)], 'Location', 'best');
                    xlabel('Time');
                    ylabel(['d', num2str(i)]);
                    title(['Disturbance d', num2str(i)]);
                    grid on;
                end
            end

            % Noise
            if ~isempty(noise_sim)
                for i = 1:size(noise_sim, 2)
                    subplot(nRow, nCol, row3Offset + colBlock + i);
                    plot(t_sim, noise_sim(:, i), 'Color', color2, 'LineWidth', lw);
                    legend(['n', num2str(i)], 'Location', 'best');
                    xlabel('Time');
                    ylabel(['n', num2str(i)]);
                    title(['Noise n', num2str(i)]);
                    grid on;
                end
            end
        end


    end
end
