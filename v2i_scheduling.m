function cID = v2i_scheduling(sInt,loc,SNR,policy)
    switch policy
        case 'greedy'
            cID = greedy_scheduling(sInt,loc,SNR);
        case 'random'
            cID = random_scheduling(sInt,loc);
        case 'fairness'
            cID = fair_scheduling(NCARS);
        case 'novel'
            cID = novel_scheduling(NCARS);
    end
end

function cID = greedy_scheduling(sInt,loc,SNR)
    limInf = sInt(1);  % Minimum location covered in the ROI
    limSup = 0;        % Maximum location covered in the ROI
    cIDCand = find(loc>limInf & loc<limSup);
    if ~isempty(cIDCand)
        [~,idx] = max(SNR(cIDCand));  % Selection that maximized the SNR
        cID = cIDCand(idx);
    else
        cID = 0;  % No car is reachible from the BS
    end
end

function cID = random_scheduling(sInt,loc)
    limInf = sInt(1);  % Minimum location covered in the ROI
    limSup = 0;        % Maximum location covered in the ROI
    cIDCand = find(loc>limInf & loc<limSup);
    if ~isempty(cIDCand)
        idx = randi([1 length(cIDCand)]);  % Random selection
        cID = cIDCand(idx);
    else
        cID = 0;  % No car is reachible from the BS
    end
end

function cID = fair_scheduling(NCARS)
    % To-Do
end

function cID = novel_scheduling(NCARS)
    % To-Do
end