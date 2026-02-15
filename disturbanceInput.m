function D = disturbanceInput(num_out, t, varargin)
% D = disturbanceInput(num_out, t, 'events', events, 'alpha', alpha, 'seed', seed)
%
% Output:
%   D: numel(t) x num_out disturbance matrix (added to outputs in simMPC)
%
% events: cell array, events{i} is an array of structs for output i.
%   Supported event types:
%     'step'  : fields t0, amp
%     'pulse' : fields t0, tf, amp
%     'ramp'  : fields t0, tf, amp (ramps from 0 to amp over [t0,tf])
%     'sine'  : fields t0, amp, T, phase (phase optional, rad)
%     'square': fields t0, amp, T, duty (duty optional, 50 default)
%
% alpha: optional smoothing (same filter style as referenceInput)

p = inputParser;
p.addParameter('events', cell(1,num_out));
p.addParameter('alpha', 0);       % 0 => no filtering
p.addParameter('seed', []);       % reserved if you later add random events
p.parse(varargin{:});
events = p.Results.events;
alpha  = p.Results.alpha;

ts = t(2) - t(1);
N  = numel(t);
D  = zeros(N, num_out);

for i = 1:num_out
    if isempty(events) || numel(events) < i || isempty(events{i})
        continue;
    end

    evs = events{i};
    for k = 1:numel(evs)
        ev = evs(k);
        switch lower(ev.type)
            case 'step'
                idx = t >= ev.t0;
                D(idx,i) = D(idx,i) + ev.amp;

            case 'pulse'
                idx = (t >= ev.t0) & (t <= ev.tf);
                D(idx,i) = D(idx,i) + ev.amp;

            case 'ramp'
                idx = (t >= ev.t0) & (t <= ev.tf);
                if any(idx)
                    D(idx,i) = D(idx,i) + ev.amp * (t(idx) - ev.t0) / max(ev.tf - ev.t0, ts);
                end
                idx2 = t > ev.tf;
                D(idx2,i) = D(idx2,i) + ev.amp;

            case 'sine'
                if ~isfield(ev,'phase'), ev.phase = 0; end
                idx = t >= ev.t0;
                D(idx,i) = D(idx,i) + ev.amp * sin((2*pi/ev.T) * (t(idx) - ev.t0) + ev.phase);

            case 'square'
                if ~isfield(ev,'duty'), ev.duty = 50; end
                idx = t >= ev.t0;
                D(idx,i) = D(idx,i) + ev.amp * square((2*pi/ev.T) * (t(idx) - ev.t0), ev.duty);

            otherwise
                error("Unknown disturbance event type '%s'.", ev.type);
        end
    end

    if alpha > 0
        D(:,i) = filter(1-alpha, [1 -alpha], D(:,i));
    end
end
end
