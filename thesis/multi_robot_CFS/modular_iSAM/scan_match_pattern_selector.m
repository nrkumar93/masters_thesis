function corrected_var = scan_match_pattern_selector(mode, keyframe_rate, var, frame_id)

persistent var_history;

if nargin == 3
    assert(mode == 2 || mode == 3);
end

if mode == 1                                       % 1. intra frame factors
    if keyframe_rate == 1
        corrected_var = var;
    elseif fix(frame_id/keyframe_rate) ~= fix(var/keyframe_rate)
        corrected_var = var;
    else
        corrected_var = frame_id;
    end
elseif mode == 2                                     % 2. fixed lag factors
    if length(var_history) ~= keyframe_rate
        var_history = [var_history var];
    else
        var_history(1) = [];
        var_history = [var_history var];
    end
    
    if length(var_history) == keyframe_rate
        corrected_var = var_history(1);
    else 
        corrected_var = [];
    end    
elseif mode == 3                                    % 3. exhaustive factors
    if length(var_history) ~= keyframe_rate
        var_history = [var_history var];
    else
        var_history(1) = [];
        var_history = [var_history var];
    end
    corrected_var = var_history;
end


