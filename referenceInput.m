function Ref = referenceInput(num_out, t, Np, type, T, shiftscale, refAmplitude, alpha)
    % num_out : number of outputs
    % t : time vector
    % Np : prediction horizon
    % type : 'sine' or 'square' wave
    % T : period of signals
    % shiftscale : vector of shift scales for each signal (relative to the previous one)
    % refAmplitude : vector of amplitudes for each signal
    % alpha : filter parameter for smooth Yd

    % Initialize the reference signals matrix
    Ref = zeros(numel(t) + Np, num_out);
    ts = t(2) - t(1); % Sample time
    temp = t(end) + ts : ts : t(end) + Np * ts;
    t = [t; temp'];
    % Generate the reference signals
    for i = 1:num_out
        % Determine the shift for the current signal
        if i == 1
            Tshift = 0; % No shift for the first signal
        else
            Tshift = sum(shiftscale(1:i)) * T; % Sum of shifts for previous signals
        end
        
        % Generate the wave based on the type
        if strcmp(type, 'square')
            signal = refAmplitude(i) * square((2 * pi) / T * t);
        elseif strcmp(type, 'sine')
            signal = refAmplitude(i) * sin((2 * pi) / T * t);
        else
            error('Unknown wave type. Use ''sine'' or ''square''.');
        end
        
        % Apply the shift
        shiftSamples = round(Tshift / ts);
        Ref(:, i) = [zeros(shiftSamples, 1); signal(1:end-shiftSamples)];
        Ref(:, i) = filter(1-alpha, [1 -alpha], Ref(:,i));
    end
end